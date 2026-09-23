/** board_imu.c - BNO08X on SPI2: bytes only, no meaning. */
#include "board_limits.h"
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "shtp.h"
#include "board_units.h"

#include <string.h>

/* PB12 (SPI2_NSS in the .ioc) as a GPIO: CS stays low across header and cargo. */
#define IMU_CS_PORT GPIOB
#define IMU_CS_PIN  GPIO_PIN_12

/* Both active low, and CubeMX drives both low at boot: MX_GPIO_Init writes
   GPIO_PIN_RESET to PD10|PD11. */
/* SPI2_SCK. Re-initialised with a pull-up - see Board_ImuInit. */
#define IMU_SCK_PORT GPIOB
#define IMU_SCK_PIN  GPIO_PIN_13

/* H_INTN, pin 14. */
#define IMU_INTN_PORT GPIOD
#define IMU_INTN_PIN  GPIO_PIN_8

/* How the part is woken again when it does not answer a wake: WAKE released
   for IMU_WAKE_RELEASE_MS and asserted again, IMU_WAKE_RETRIES times, then a
   reset with its advertisement drained. */
#define IMU_WAKE_RETRIES     3U
#define IMU_WAKE_RELEASE_MS  2U
#define IMU_RESET_DRAIN      16U
#define IMU_WRITE_DRAIN      8U      /**< reads before a write speaks */
#define IMU_SPI_TIMEOUT_MS   100U
#define IMU_WAKE_TEST_DRAIN  16U      /**< reads before the wake test asks */
#define IMU_QUIET_EMPTIES    3U       /**< empties in a row that mean quiet */
#define IMU_WAKE_NOT_READY   0xFFFFU  /**< the wake test's two answers that */
#define IMU_WAKE_BUSY        0xFFFEU  /**< are not a time */
/* PS0/WAKE, pin 6. */
#define IMU_WAKE_PORT GPIOD
#define IMU_WAKE_PIN  GPIO_PIN_9

#define IMU_RST_PORT  GPIOD
#define IMU_RST_PIN   GPIO_PIN_10
#define IMU_BOOT_PORT GPIOD
#define IMU_BOOT_PIN  GPIO_PIN_11

/* EVERY FEATURE ASKED FOR, not the last one. */
#define IMU_FEATURES 4U

/** The IMU driver's state: the link and its buffers, the part's clocks, the
    latest report, the bring-up stage, the features to set and the re-apply
    that waits for the part to go quiet. */
static struct
{
  bool ready;
  uint8_t seq[6];                 /* one per SHTP channel, section 1.3.1 */

  /* Static, not automatic. */
  uint8_t rx[IMU_BUF];
  uint8_t tx[IMU_BUF];

  uint32_t kernel_hz;
  uint32_t bitrate_hz;

  /* The loop's own record. */
  board_imu_state_t state;

  uint8_t stage;
  uint32_t stage_at;

  uint8_t feature_id_of[IMU_FEATURES];
  uint32_t feature_us_of[IMU_FEATURES];
  uint8_t features;

  /* The most recent, for the command layer's one-feature question. */
  uint8_t feature_id;
  uint32_t feature_us;

  /** Set when a reset has thrown the feature away and it has not been asked
      for again yet. */
  bool feature_pending;

  /** Which slot the re-apply has got to, since it does one a turn. */
  uint8_t feature_next;
  uint32_t cargoes_at_reset;      /**< to know the part has spoken */
  uint32_t last_cargo_ms;         /**< when the last one arrived */
} s;

static const uint8_t s_zeros[IMU_BUF];

/* The slowest divider that still clears the part's ceiling, chosen from the
   kernel clock the peripheral actually has rather than from a field in the
   .ioc. */
static uint32_t prescaler_under(uint32_t limit_hz)
{
  s.kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
  return board_spi_prescaler(s.kernel_hz, limit_hz, &s.bitrate_hz);
}

static bool intn_asserted(void)
{
  return HAL_GPIO_ReadPin(IMU_INTN_PORT, IMU_INTN_PIN) == GPIO_PIN_RESET;
}

static bool poll_due(void)
{
  static uint32_t last;
  const uint32_t now = Board_Cycles();
  const uint32_t gap = Board_SysClkHz() / IMU_POLL_HZ;

  if ((now - last) < gap)
  {
    return false;
  }
  last = now;
  return true;
}

/** True if the part asserted H_INTN within `ms`. */
static bool wait_intn(uint32_t ms)
{
  const uint32_t start = HAL_GetTick();

  while (!intn_asserted())
  {
    if ((uint32_t)(HAL_GetTick() - start) > ms)
    {
      return false;
    }
    Board_StoKeepalive();
  }

  return true;
}

static void wake(bool low)
{
  HAL_GPIO_WritePin(IMU_WAKE_PORT, IMU_WAKE_PIN,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void cs(bool low)
{
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void settle(void)
{
  board_spin_us(IMU_SETTLE_US);
}

void Board_ImuReset(void)
{
  /* Out of the bootloader first: BOOTN is sampled at reset, so it has to be
     high before NRSTN is released, not after. */
  HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(IMU_RESET_HOLD_MS);
  HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);

  /* Nothing waits on H_INTN here: the part is mid-reset and the edge would
     have to be caught to be useful, so the wait is the datasheet's number
     rather than an observation. */
  HAL_Delay(IMU_RESET_WAIT_MS);
}

bool Board_ImuBusInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* AFE_ON powers the part, not just the analog front end. */
  if (!Board_AfeOn())
  {
    return false;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Re-init rather than patch: HAL latches the mode into CFG1/CFG2 at
     HAL_SPI_Init, so changing the struct alone would configure nothing. */
  if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
  {
    return false;
  }

  /* The one limit a regeneration must not be able to break. */
  hspi2.Init.BaudRatePrescaler = prescaler_under(IMU_MAX_HZ);
  hspi2.Init.DataSize     = SPI_DATASIZE_8BIT;
  /* Mode 3, and set here because CubeMX's is not what runs: this re-inits
     the peripheral, so the .ioc value is overwritten. */
  hspi2.Init.CLKPolarity  = SPI_POLARITY_HIGH;   /* CPOL = 1 */
  hspi2.Init.CLKPhase     = SPI_PHASE_2EDGE;     /* CPHA = 1 */
  hspi2.Init.NSS          = SPI_NSS_SOFT;
  hspi2.Init.NSSPMode     = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.FirstBit     = SPI_FIRSTBIT_MSB;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    return false;
  }

  /* PB12 as a plain output, after HAL_SPI_Init and never before:
     HAL_SPI_MspDeInit runs HAL_GPIO_DeInit over PB12..PB15 and MspInit puts
     all four back as SPI2 alternate function, so a chip select configured
     ahead of the init is handed straight back to the peripheral. */
  gpio.Pin = IMU_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(IMU_CS_PORT, &gpio);
  cs(false);

  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Pin = IMU_WAKE_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_WAKE_PORT, &gpio);
  wake(false);

  /* Pulled up: H_INTN is driven low and released, not driven high. */
  gpio.Pin = IMU_INTN_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_INTN_PORT, &gpio);

  memset(s.seq, 0, sizeof(s.seq));
  s.ready = true;
  return true;
}

bool Board_ImuInit(void)
{
  /* The bus, then the part. */
  if (!Board_ImuBusInit())
  {
    return false;
  }

  Board_ImuReset();
  return true;
}

bool Board_ImuReady(void)
{
  /* AFE_ON off means the part lost its supply, and a part that has lost its
     supply needs a reset, not a resume: measured, with the AFE switched on
     under a part that was already "ready" the stream never started, and the
     same sequence with a reset after it gave 135 rotation vectors in four
     seconds. */
  if (!Board_AfeOn())
  {
    s.ready = false;
    return false;
  }

  return s.ready;
}

static bool imu_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  uint16_t done = 0U;

  while (done < len)
  {
    const uint16_t n = ((uint16_t)(len - done) > IMU_CHUNK)
                         ? IMU_CHUNK : (uint16_t)(len - done);

    if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx + done, rx + done,
                                n, IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
      return false;
    }
    done = (uint16_t)(done + n);
    Board_StoKeepalive();
  }
  return true;
}

/* One chip select assertion, however many bytes. */
static bool transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  HAL_StatusTypeDef st;

  if (!s.ready || (len == 0U) || (len > IMU_BUF))
  {
    return false;
  }

  if (tx == NULL)
  {
    tx = s_zeros;
  }

  cs(true);
  settle();
  st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, IMU_SPI_TIMEOUT_MS);
  settle();
  cs(false);

  return st == HAL_OK;
}

bool Board_ImuRead(uint8_t *channel, uint8_t *cargo, uint16_t cap,
                   uint16_t *len)
{
  shtp_header_t head;

  if ((channel == NULL) || (cargo == NULL) || (len == NULL))
  {
    return false;
  }

  *len = 0U;

  if (!s.ready)
  {
    return false;
  }

  /* No gate here. */
  (void)intn_asserted();

  /* One chip select assertion for the header AND the cargo behind it. */
  cs(true);
  settle();

  bool ok = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)s_zeros, s.rx,
                                    SHTP_HEADER_LEN, IMU_SPI_TIMEOUT_MS) == HAL_OK;

  if (ok && shtp_parse_header(s.rx, &head) && (head.length > SHTP_HEADER_LEN))
  {
    const uint16_t rest = (uint16_t)(head.length - SHTP_HEADER_LEN);

    /* Clocked out whole even when the caller cannot hold it: leaving bytes
       in the part desynchronises every later read. */
    const uint16_t take = (rest > (uint16_t)sizeof(s.rx))
                            ? (uint16_t)sizeof(s.rx) : rest;

    ok = imu_xfer(s_zeros, s.rx, take);
    if (ok)
    {
      *len = (take > cap) ? cap : take;
      memcpy(cargo, s.rx, *len);
    }
  }

  settle();
  cs(false);

  if (!ok)
  {
    *len = 0U;
    return false;
  }

  *channel = head.channel;
  return true;
}

void Board_ImuClock(uint32_t *kernel_hz, uint32_t *bitrate_hz)
{
  if (kernel_hz != NULL)  { *kernel_hz = s.kernel_hz; }
  if (bitrate_hz != NULL) { *bitrate_hz = s.bitrate_hz; }
}

bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select)
{
  /* No upper check: len is a uint8_t and IMU_BUF is 320, so one cannot
     exceed the other. */
  if ((out == NULL) || (len == 0U))
  {
    return false;
  }

  if (!s.ready && !Board_ImuInit())
  {
    return false;
  }

  /* Wait the same way a read does. */
  (void)wait_intn(IMU_INTN_WAIT_MS);

  if (select)
  {
    return transfer(NULL, out, len);
  }

  /* Deliberately without chip select. */
  settle();
  const bool ok = imu_xfer(s_zeros, out, len);
  settle();
  return ok;
}

uint16_t Board_ImuWakeTest(uint16_t ms)
{
  if (!s.ready && !Board_ImuInit())
  {
    return IMU_WAKE_NOT_READY;
  }

  /* Empty first: H_INTN stays asserted while anything is queued, and a line
     that is already low answers nothing about the wake. */
  (void)Board_ImuDrain(IMU_WAKE_TEST_DRAIN);

  if (intn_asserted())
  {
    return IMU_WAKE_BUSY;         /* the answer would be a lie */
  }

  const uint32_t start = HAL_GetTick();

  wake(true);
  while ((uint32_t)(HAL_GetTick() - start) < ms)
  {
    if (intn_asserted())
    {
      wake(false);
      return (uint16_t)(HAL_GetTick() - start);
    }
  }

  wake(false);
  return 0xFFFFU;                 /* never answered */
}

uint8_t Board_ImuPinCheck(uint8_t pin)
{
  GPIO_InitTypeDef gpio = {0};
  const uint16_t mask = (uint16_t)(1U << pin);
  uint8_t bits = 0U;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* CHIP SELECT DEASSERTED FIRST, and this is not housekeeping. */
  if (pin != BOARD_IMU_SPI_PIN_FIRST)
  {
    GPIO_InitTypeDef cs = {0};

    cs.Pin = GPIO_PIN_12;
    cs.Mode = GPIO_MODE_OUTPUT_PP;
    cs.Pull = GPIO_NOPULL;
    cs.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &cs);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   /* CSN idle high */
    HAL_Delay(1U);
  }

  gpio.Pin = mask;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, mask, GPIO_PIN_SET);
  HAL_Delay(1U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_SET)   { bits |= 0x01U; }

  HAL_GPIO_WritePin(GPIOB, mask, GPIO_PIN_RESET);
  HAL_Delay(1U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_RESET) { bits |= 0x02U; }

  /* Released, with the MCU's own pulls. */
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_Delay(2U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_SET)   { bits |= 0x04U; }

  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_Delay(2U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_RESET) { bits |= 0x08U; }

  /* The pin is left as an input. */
  s.ready = false;
  return bits;
}

static void note(uint8_t err)
{
  s.state.error = err;
  if (err != BOARD_IMU_ERR_NONE)
  {
    s.state.errors++;
    /* Kept, because `error` is cleared by the next good read and a host
       polling at 5 Hz never sees a fault at 400 reports a second. */
    s.state.last_fault = err;
  }
}

/** A rotation vector report into the shared record, and the ring. */
static void take_rotation(uint8_t id, const uint8_t *r)
{
  s.state.report_id = id;
  s.state.status    = r[2];
  s.state.i    = (int16_t)((uint16_t)r[4] | ((uint16_t)r[5] << 8));
  s.state.j    = (int16_t)((uint16_t)r[6] | ((uint16_t)r[7] << 8));
  s.state.k    = (int16_t)((uint16_t)r[8] | ((uint16_t)r[9] << 8));
  s.state.real = (int16_t)((uint16_t)r[10] | ((uint16_t)r[11] << 8));
  s.state.have = true;
  s.state.updates++;

  const int16_t logged[4] = { s.state.i, s.state.j, s.state.k, s.state.real };
  Board_LogPush(BOARD_LOG_SOURCE_IMU, logged, 4U);
  note(BOARD_IMU_ERR_NONE);
}

/* The three-axis reports share one shape with each other and with the
   rotation vector's first three fields: a five-byte header (id, sequence,
   status, delay) then x, y, z little-endian. */
static void take_vector(const uint8_t *r, int16_t *out, uint8_t *status)
{
  *status = r[2];
  out[0] = (int16_t)((uint16_t)r[4] | ((uint16_t)r[5] << 8));
  out[1] = (int16_t)((uint16_t)r[6] | ((uint16_t)r[7] << 8));
  out[2] = (int16_t)((uint16_t)r[8] | ((uint16_t)r[9] << 8));
  note(BOARD_IMU_ERR_NONE);
}

static void absorb(uint8_t channel, const uint8_t *cargo, uint16_t len)
{
  s.state.cargoes++;

  if ((channel != SHTP_CH_INPUT) && (channel != SHTP_CH_WAKE))
  {
    return;
  }

  uint16_t at = 0U;

  while (at < len)
  {
    const uint8_t  id = cargo[at];
    const uint16_t step = (uint16_t)shtp_report_len(id);

    /* A zero byte after the last report is padding, not a report. */
    if ((step == 0U) && (id == 0U))
    {
      break;
    }

    if ((step == 0U) || ((at + step) > len))
    {
      /* Which id, because "a report id with no length" and "the cargo was
         cut short" are two different defects and the counter alone cannot
         tell them apart. */
      s.state.last_fault_id = id;
      note(BOARD_IMU_ERR_FRAME);
      return;
    }

    if ((id == SH2_ROTATION_VECTOR) || (id == SH2_GAME_ROTATION_VECTOR))
    {
      take_rotation(id, &cargo[at]);
    }
    else if (id == SH2_REPORT_ACCELEROMETER)
    {
      take_vector(&cargo[at], s.state.accel, &s.state.accel_status);
      s.state.have_accel = true;
    }
    else if (id == SH2_REPORT_GYROSCOPE)
    {
      take_vector(&cargo[at], s.state.gyro, &s.state.gyro_status);
      s.state.have_gyro = true;
    }
    else if (id == SH2_REPORT_MAGNETIC_FIELD)
    {
      take_vector(&cargo[at], s.state.mag, &s.state.mag_status);
      s.state.have_mag = true;
    }

    at = (uint16_t)(at + step);
  }
}

/* Where the staged reset has got to. */
#define IMU_STAGE_BUS   0U
#define IMU_STAGE_HOLD  1U
#define IMU_STAGE_WAIT  2U

/** The last Set Feature asked for, so it can be asked for again. */

/** Remember one, replacing an entry for the same report. */
static bool feature_keep(uint8_t report_id, uint32_t interval_us)
{
  for (uint8_t i = 0U; i < s.features; i++)
  {
    if (s.feature_id_of[i] == report_id)
    {
      s.feature_us_of[i] = interval_us;
      return true;
    }
  }
  if (s.features >= IMU_FEATURES)
  {
    return false;
  }
  s.feature_id_of[s.features] = report_id;
  s.feature_us_of[s.features] = interval_us;
  s.features++;
  return true;
}

bool Board_ImuSetFeature(uint8_t report_id, uint32_t interval_us)
{
  uint8_t payload[17];

  if (shtp_set_feature(payload, sizeof(payload), report_id, interval_us) == 0U)
  {
    return false;
  }
  if (!Board_ImuWrite(SHTP_CH_CONTROL, payload, sizeof(payload)))
  {
    return false;
  }

  /* Remembered only once it took. */
  s.feature_id = report_id;
  s.feature_us = interval_us;
  (void)feature_keep(report_id, interval_us);
  return true;
}

void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us,
                           bool *pending)
{
  *report_id = s.feature_id;
  *interval_us = s.feature_us;
  *pending = s.feature_pending;
}

static void poll_init(void)
{
  switch (s.stage)
  {
    case IMU_STAGE_BUS:
      if (!Board_ImuBusInit())
      {
        note(BOARD_IMU_ERR_INIT);
        s.stage_at = HAL_GetTick();     /* the next attempt waits a stage */
        return;
      }
      HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
      s.stage = IMU_STAGE_HOLD;
      s.stage_at = HAL_GetTick();
      return;

    case IMU_STAGE_HOLD:
      if ((HAL_GetTick() - s.stage_at) < IMU_RESET_HOLD_MS)
      {
        return;
      }
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);
      s.stage = IMU_STAGE_WAIT;
      s.stage_at = HAL_GetTick();
      return;

    default:
      if ((HAL_GetTick() - s.stage_at) < IMU_RESET_WAIT_MS)
      {
        return;
      }
      s.stage = IMU_STAGE_BUS;
      s.state.loop = BOARD_IMU_LOOP_RUN;
      note(BOARD_IMU_ERR_NONE);

      /* Ask again for whatever was asked for before the reset - but not
         here. */
      s.feature_pending = (s.features != 0U);
      s.feature_next = 0U;
      s.cargoes_at_reset = s.state.cargoes;
      return;
  }
}

/* The part lost its supply. */
static void power_lost(void)
{
  if (s.state.loop == BOARD_IMU_LOOP_OFF)
  {
    return;
  }
  s.state.loop = BOARD_IMU_LOOP_OFF;
  s.state.have = false;
  s.ready = false;
  s.stage = IMU_STAGE_BUS;
  note(BOARD_IMU_ERR_POWER);
}

/** Nothing queued and a feature still missing: the quiet moment the re-apply
    was waiting for. */
static bool reapply_due(void)
{
  return s.feature_pending && (s.state.cargoes > s.cargoes_at_reset)
         && ((HAL_GetTick() - s.last_cargo_ms) > IMU_QUIET_MS)
         && !intn_asserted();
}

/** ONE PER TURN. */
static void reapply_one(void)
{
  if (Board_ImuSetFeature(s.feature_id_of[s.feature_next],
                          s.feature_us_of[s.feature_next]))
  {
    s.feature_next++;
    s.feature_pending = (s.feature_next < s.features);
  }
}

void Board_ImuPoll(void)
{
  static uint8_t cargo[IMU_BUF];

  if (!Board_AfeOn())
  {
    power_lost();
    return;
  }

  /* Not while the host is configuring it, and NOT DURING THE OBSERVER'S
     BORROW. */
  if ((s.state.loop == BOARD_IMU_LOOP_HELD)
      || Board_PowerHolds(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    return;
  }

  if (s.state.loop == BOARD_IMU_LOOP_OFF)
  {
    s.state.loop = BOARD_IMU_LOOP_INIT;
  }

  if (s.state.loop == BOARD_IMU_LOOP_INIT)
  {
    poll_init();
    return;
  }

  if (reapply_due())
  {
    reapply_one();
    return;
  }

  /* Nothing waiting is the common case and must cost nothing: one GPIO read
     and out. */
  if (!intn_asserted() && !poll_due())
  {
    return;
  }

  uint8_t  channel = 0U;
  uint16_t len = 0U;

  if (!Board_ImuRead(&channel, cargo, (uint16_t)sizeof(cargo), &len))
  {
    note(BOARD_IMU_ERR_READ);
    return;
  }

  if (len > 0U)
  {
    s.last_cargo_ms = HAL_GetTick();
    absorb(channel, cargo, len);
  }
}

void Board_ImuState(board_imu_state_t *out)
{
  if (out != NULL)
  {
    *out = s.state;
  }
}

void Board_ImuHold(void)
{
  s.state.loop = BOARD_IMU_LOOP_HELD;
  s.stage = IMU_STAGE_BUS;

  /* A hold that lands mid-staged-reset leaves NRSTN low and the part half
     up, and every command the host then sends is refused or ignored -
     measured: hold, reset, Set Feature, resume, and the loop absorbed
     nothing. */
  if (Board_AfeOn() && !s.ready && !Board_ImuInit())
  {
    note(BOARD_IMU_ERR_INIT);
  }
}

void Board_ImuResume(void)
{
  /* Back to RUN when the part is still up, because the usual hold is to
     enable a report and going through init would reset the part and throw
     that away - measured: hold, Set Feature, resume, and the loop absorbed
     nothing at all afterwards. */
  s.state.loop = s.ready ? BOARD_IMU_LOOP_RUN : BOARD_IMU_LOOP_INIT;
}

uint8_t Board_ImuDrain(uint8_t limit)
{
  /* Reads through s.rx like everything else and throws the result away, so
     it needs no buffer of its own. */
  static uint8_t scratch[8];
  uint8_t channel = 0U;
  uint16_t len = 0U;
  uint8_t taken = 0U;

  /* One empty read is not the end of the queue. */
  uint8_t quiet = 0U;

  for (uint8_t i = 0U; (i < limit) && (quiet < IMU_QUIET_EMPTIES); i++)
  {
    if (!Board_ImuRead(&channel, scratch, (uint16_t)sizeof(scratch), &len))
    {
      break;
    }
    if (len == 0U)
    {
      quiet++;
      HAL_Delay(2U);
      Board_StoKeepalive();
      continue;
    }
    quiet = 0U;
    taken++;
  }

  return taken;
}

bool Board_ImuWaitReady(uint32_t ms)
{
  return wait_intn(ms);
}

/* Measured on this board: the part answers a wake in under a millisecond,
   and then now and again does not answer one at all - twice in ten over
   eight seconds, and permanently after it had been left alone for a few
   minutes. */
static bool woken_by_retry(void)
{
  for (uint8_t again = 0U; again < IMU_WAKE_RETRIES; again++)
  {
    wake(false);
    HAL_Delay(IMU_WAKE_RELEASE_MS);
    wake(true);
    if (wait_intn(IMU_WAKE_WAIT_MS))
    {
      return true;
    }
  }
  wake(false);
  Board_ImuReset();
  (void)Board_ImuDrain(IMU_RESET_DRAIN);
  wake(true);
  return wait_intn(IMU_WAKE_WAIT_MS);
}

/** Empty the part before speaking, then WAKE and wait to be let in. */
static void wake_for_write(void)
{
  (void)Board_ImuDrain(IMU_WRITE_DRAIN);
  wake(true);
  if (!wait_intn(IMU_WAKE_WAIT_MS) && !woken_by_retry())
  {
    note(BOARD_IMU_ERR_NOWAKE);
  }
}

/** The built frame out, full duplex: the part clocks its own cargo out while
    this one goes in. */
static bool transfer_frame(size_t n)
{
  cs(true);
  settle();
  wake(false);

  bool ok = HAL_SPI_TransmitReceive(&hspi2, s.tx, s.rx, (uint16_t)n,
                                    IMU_SPI_TIMEOUT_MS) == HAL_OK;

  shtp_header_t incoming;

  if (ok && (n >= SHTP_HEADER_LEN) && shtp_parse_header(s.rx, &incoming) &&
      (incoming.length > (uint16_t)n))
  {
    uint16_t rest = (uint16_t)(incoming.length - (uint16_t)n);

    if (rest > (uint16_t)sizeof(s.rx))
    {
      rest = (uint16_t)sizeof(s.rx);
    }

    /* Discarded: this is the tail of something the part was already sending
       when the write went out, and the caller asked to write, not to read. */
    ok = imu_xfer(s_zeros, s.rx, rest);
  }

  settle();
  cs(false);
  return ok;
}

bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len)
{
  if (!s.ready ||
      (channel >= (uint8_t)(sizeof(s.seq) / sizeof(s.seq[0]))))
  {
    return false;
  }

  const size_t n = shtp_build(s.tx, sizeof(s.tx), channel, s.seq[channel],
                              payload, len);
  if ((n == 0U) || (n > IMU_BUF))
  {
    return false;
  }

  wake_for_write();

  if (!transfer_frame(n))
  {
    return false;
  }

  /* "Each channel and each direction has its own sequence number", 1.3.1. */
  s.seq[channel]++;
  return true;
}
