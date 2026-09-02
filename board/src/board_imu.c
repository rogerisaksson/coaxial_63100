/**
  ******************************************************************************
  * @file    board_imu.c
  * @brief   The BNO08X on SPI2: the bytes, and nothing about what they mean.
  *
  * SHTP framing and SH-2 decoding are in shtp/, hardware-free. This is the
  * other half - chip select, clocking, and the length field.
  *
  * CubeMX does not match the part (BNO080_085 v1.17), and this fixes it at
  * runtime rather than editing core/, which CubeMX regenerates:
  *
  *   - CS held across header AND cargo (1.2.4.2), so PB12 is plain GPIO;
  *     hardware NSS pulses per frame and would end the transaction between
  *
  * H_INTN on PD8 does assert - `feature()` needs the wake acknowledge and
  * works. An earlier reading of 77 highs is retracted: Modbus round trips
  * 15 ms apart cannot catch a microsecond pulse (FINDINGS).
  *
  * The header is polled anyway, rate limited: catching the edge is not
  * guaranteed, and without it the part streamed at 50 Hz into a loop that
  * read none.
  ******************************************************************************
  */
#include "board_limits.h"
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "shtp.h"

#include <string.h>

/* PB12 is SPI2_NSS in the .ioc. Driven by hand here - see the file comment. */
#define IMU_CS_PORT GPIOB
#define IMU_CS_PIN  GPIO_PIN_12

/* Both active low, and CubeMX drives both low at boot:
   MX_GPIO_Init writes GPIO_PIN_RESET to PD10|PD11. That is the part held in
   reset AND strapped for the bootloader - "BOOTN is sampled at reset. If low
   the BNO08X will enter bootloader mode" (section 1.2.2). Measured before
   this file drove them: the four header bytes read 00 00 00 00, which is a
   part holding MISO low rather than an absent one, which idles high. */
/* SPI2_SCK. Re-initialised with a pull-up - see Board_ImuInit. */
#define IMU_SCK_PORT GPIOB
#define IMU_SCK_PIN  GPIO_PIN_13

/* H_INTN, pin 14. Active low: the part drives it down when it wants
   attention and releases it "as soon as the chip select is detected"
   (1.2.4.3). Reading without waiting for it is reading blind - measured, the
   advertisement turned up in one sample out of six. */
#define IMU_INTN_PORT GPIOD
#define IMU_INTN_PIN  GPIO_PIN_8

/* PS0/WAKE, pin 6. Active low, and the way a host starts a conversation:
   "this function should initiate a write transaction by asserting WAKEN. The
   write transaction should continue, then, when the system responds to INTN
   being asserted" - SH-2 user guide, sh2_hal_tx. */
#define IMU_WAKE_PORT GPIOD
#define IMU_WAKE_PIN  GPIO_PIN_9

#define IMU_RST_PORT  GPIOD
#define IMU_RST_PIN   GPIO_PIN_10
#define IMU_BOOT_PORT GPIOD
#define IMU_BOOT_PIN  GPIO_PIN_11

static bool s_ready;
static uint8_t s_seq[6];        /* one per SHTP channel, section 1.3.1 */

/* Static, not automatic. The linker script gives this firmware a 1 KB stack
   (_Min_Stack_Size = 0x400) and the deepest path here - a command handler
   into Board_ImuWrite into Board_ImuDrain into Board_ImuRead - had 1280
   bytes of locals on it once IMU_BUF grew from 64 to 320 to hold the
   advertisement. That is an overflow underneath the whole Modbus call chain,
   and it read as the part resetting itself: garbage channels, cargoes that
   arrived out of order, and a write that worked twice and failed a third
   time. Nothing here is re-entrant - the board layer runs from one main
   loop - so one buffer each is enough.

   s_tx is separate from s_rx because a write builds its frame BEFORE
   draining, and the drain reads through s_rx. */
static uint8_t s_rx[IMU_BUF];
static uint8_t s_tx[IMU_BUF];
static const uint8_t s_zeros[IMU_BUF];

static uint32_t s_kernel_hz;
static uint32_t s_bitrate_hz;

/* The slowest divider that still clears the part's ceiling, chosen from the
   kernel clock the peripheral actually has rather than from a field in the
   .ioc. Returns the largest divider if even that is too fast, which cannot
   happen below a 768 MHz kernel clock. */
static uint32_t prescaler_under(uint32_t limit_hz)
{
  static const uint32_t DIVIDERS[] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };

  const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);

  s_kernel_hz = kernel;

  /* A kernel clock of zero means the peripheral clock is not configured, and
     the loop below would read 0 <= limit on the first divider and pick the
     fastest one there is. Slowest instead: too slow is a slow read, too fast
     is a part that never answers. */
  if (kernel == 0U)
  {
    s_bitrate_hz = 0U;
    return SPI_BAUDRATEPRESCALER_256;
  }

  for (uint32_t i = 0U; i < (sizeof(DIVIDERS) / sizeof(DIVIDERS[0])); i++)
  {
    if ((kernel >> (i + 1U)) <= limit_hz)
    {
      s_bitrate_hz = kernel >> (i + 1U);
      return DIVIDERS[i];
    }
  }

  s_bitrate_hz = kernel >> 8;
  return SPI_BAUDRATEPRESCALER_256;
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


/** True if the part asserted H_INTN within `ms`.
  *
  * Elapsed, not a deadline: `HAL_GetTick() + ms` wraps every 49.7 days and
  * the `>` against it then reads backwards - the wait returns at once or
  * spins for a month. The same shape as the lease sentinel.
  *
  * Pumps the STO charge pump while it spins, like every other busy wait on
  * this board: a keepalive that stops whenever the board is waiting is a
  * keepalive that lies.
  */
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
  const uint32_t per_us = SystemCoreClock / 1000000U;
  const uint32_t start = Board_Cycles();

  while ((uint32_t)(Board_Cycles() - start) < (IMU_SETTLE_US * per_us))
  {
    /* Busy wait - a chip select edge is not worth an interrupt - so it may
       as well feed the STO charge pump while it spins. */
    Board_StoKeepalive();
  }
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

  /* AFE_ON powers the part, not just the analog front end. Measured: with it
     off the BNO08X still drives MISO and still resets - enough to read a
     valid 276-byte advertisement from - but no write is ever acted on, and
     the wake handshake answers sometimes and not others. Every symptom then
     points at SPI, which is where a day went. Refuse instead of
     half-working, so the supply is the first thing the failure names. */
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

  /* The one limit a regeneration must not be able to break. Measured: with
     SYSCLK restored to 475 MHz the SPI2 kernel clock is 190 MHz and CubeMX's
     prescaler of 32 gives 5.94 MBit/s - the .ioc says so itself - against the
     part's 3 MHz maximum (Figure 6-8). Every read came back FF FF FF FF. At
     the 75 MHz the clock tree had briefly regressed to, the same field gave
     2.34 MBit/s and the part answered. A number that is only correct for one
     clock configuration is not a number: derive it. */
  hspi2.Init.BaudRatePrescaler = prescaler_under(IMU_MAX_HZ);
  hspi2.Init.DataSize     = SPI_DATASIZE_8BIT;
  /* Mode 3, and set here because CubeMX's is not what runs: this
     re-inits the peripheral, so the .ioc value is overwritten. */
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
     ahead of the init is handed straight back to the peripheral. Hardware
     NSS pulses per data frame; the part wants one assertion across a whole
     transaction (1.2.4.2). */
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

  memset(s_seq, 0, sizeof(s_seq));
  s_ready = true;
  return true;
}

bool Board_ImuInit(void)
{
  /* The bus, then the part. Blocking, because a command handler runs between
     two Modbus frames and 130 ms there costs nothing; Board_ImuPoll has its
     own staged version, because 130 ms inside the main loop is a Modbus
     request that times out - measured, as `fc 0x46: silence`. */
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
     seconds. Clearing the flag here is what makes the next command re-init. */
  if (!Board_AfeOn())
  {
    s_ready = false;
    return false;
  }

  return s_ready;
}

static bool imu_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  uint16_t done = 0U;

  while (done < len)
  {
    const uint16_t n = ((uint16_t)(len - done) > IMU_CHUNK)
                         ? IMU_CHUNK : (uint16_t)(len - done);

    if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx + done, rx + done,
                                n, 100U) != HAL_OK)
    {
      return false;
    }
    done = (uint16_t)(done + n);
    Board_StoKeepalive();
  }
  return true;
}


/* One chip select assertion, however many bytes. Full duplex because SPI is:
   reading a cargo clocks zeros out, writing one clocks whatever the part has
   to say in. */
static bool transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
  HAL_StatusTypeDef st;

  if (!s_ready || (len == 0U) || (len > IMU_BUF))
  {
    return false;
  }

  if (tx == NULL)
  {
    tx = s_zeros;
  }

  cs(true);
  settle();
  st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, 100U);
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

  if (!s_ready)
  {
    return false;
  }

  /* No gate here. Clocking a header at an idle part used to be what made
     every read come back with MISO idling high, but that was a part held
     in reset; this one answers a header with a length of zero, which costs
     one four-byte transfer and tells the truth. Whoever calls this decides
     how often - the poll loop rate limits itself, and a host read is one
     the operator asked for. Gating twice was worse than not gating: the
     loop spent its slot on `poll_due` and this consumed the next one, so
     neither ever read. */
  (void)intn_asserted();

  /* One chip select assertion for the header AND the cargo behind it. The
     reference driver reads the first bytes, takes the length out of them and
     continues the same transaction - Hillcrest's sh2_hal_spi.c, startOpShtp:
     it asserts CSN once and runs a header phase followed by a body phase.
     Releasing CSN between the two makes the part start the message over, and
     the second read then clocks the header again instead of the cargo. That
     is what this used to do. */
  cs(true);
  settle();

  bool ok = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)s_zeros, s_rx,
                                    SHTP_HEADER_LEN, 100U) == HAL_OK;

  if (ok && shtp_parse_header(s_rx, &head) && (head.length > SHTP_HEADER_LEN))
  {
    const uint16_t rest = (uint16_t)(head.length - SHTP_HEADER_LEN);

    /* Clocked out whole even when the caller cannot hold it: leaving bytes
       in the part desynchronises every later read. What does not fit is
       dropped here rather than upstream, and *len says how much arrived. */
    const uint16_t take = (rest > (uint16_t)sizeof(s_rx))
                            ? (uint16_t)sizeof(s_rx) : rest;

    ok = imu_xfer(s_zeros, s_rx, take);
    if (ok)
    {
      *len = (take > cap) ? cap : take;
      memcpy(cargo, s_rx, *len);
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
  if (kernel_hz != NULL)  { *kernel_hz = s_kernel_hz; }
  if (bitrate_hz != NULL) { *bitrate_hz = s_bitrate_hz; }
}

bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select)
{
  /* No upper check: len is a uint8_t and IMU_BUF is 320, so one cannot
     exceed the other. The comparison that used to be here was always false
     and -Wtype-limits said so the moment the buffer grew. */
  if ((out == NULL) || (len == 0U))
  {
    return false;
  }

  if (!s_ready && !Board_ImuInit())
  {
    return false;
  }

  /* Wait the same way a read does. Without it the probe clocks while the
     part has nothing to say, every answer is FF, and the comparison the
     `select` argument exists to make is not a comparison at all. */
  (void)wait_intn(IMU_INTN_WAIT_MS);

  if (select)
  {
    return transfer(NULL, out, len);
  }

  /* Deliberately without chip select. A part that answers here is a part
     that is not seeing H_CSN. */
  settle();
  const bool ok = imu_xfer(s_zeros, out, len);
  settle();
  return ok;
}

uint16_t Board_ImuWakeTest(uint16_t ms)
{
  if (!s_ready && !Board_ImuInit())
  {
    return 0xFFFFU;
  }

  /* Empty first: H_INTN stays asserted while anything is queued, and a line
     that is already low answers nothing about the wake. */
  (void)Board_ImuDrain(16U);

  if (intn_asserted())
  {
    return 0xFFFEU;               /* still busy - the answer would be a lie */
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

  /* CHIP SELECT DEASSERTED FIRST, and this is not housekeeping. Each call
     leaves the pin it tested as an input, so checking the four in turn left
     PB12 floating by the time MISO's turn came - and a chip select that
     floats low asserts the part, which then drives MISO exactly as it should.
     The check reported MISO as held by something else and it was the test's
     own doing. Measured 2026-08-29: bits 11 with CS floating.
     Skipped when PB12 is the pin under test, which cannot deassert itself. */
  if (pin != 12U)
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

  /* Released, with the MCU's own pulls. A pull-up that reads low or a
     pull-down that reads high is something else driving the net - which is
     the fault a drive test cannot see, because push-pull wins against it. */
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_Delay(2U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_SET)   { bits |= 0x04U; }

  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_Delay(2U);
  if (HAL_GPIO_ReadPin(GPIOB, mask) == GPIO_PIN_RESET) { bits |= 0x08U; }

  /* The pin is left as an input. The next IMU command re-runs the whole
     init, which hands PB12..PB15 back to SPI2. */
  s_ready = false;
  return bits;
}

/* The loop's own record. One writer - Board_ImuPoll - and one reader, both
   on the main loop, so there is nothing to lock. */
static board_imu_state_t s_state;

static void note(uint8_t err)
{
  s_state.error = err;
  if (err != BOARD_IMU_ERR_NONE)
  {
    s_state.errors++;
    /* Kept, because `error` is cleared by the next good read and a host
       polling at 5 Hz never sees a fault at 400 reports a second. Without
       it the counter says how many and nothing says of what. */
    s_state.last_fault = err;
  }
}

/* One cargo into the shared record. Only channel 3 and channel 4 carry
   sensor reports; the rest is the part talking about itself and is counted,
   not kept. */
/** A rotation vector report into the shared record, and the ring. */
static void take_rotation(uint8_t id, const uint8_t *r)
{
  s_state.report_id = id;
  s_state.status    = r[2];
  s_state.i    = (int16_t)((uint16_t)r[4] | ((uint16_t)r[5] << 8));
  s_state.j    = (int16_t)((uint16_t)r[6] | ((uint16_t)r[7] << 8));
  s_state.k    = (int16_t)((uint16_t)r[8] | ((uint16_t)r[9] << 8));
  s_state.real = (int16_t)((uint16_t)r[10] | ((uint16_t)r[11] << 8));
  s_state.have = true;
  s_state.updates++;

  const int16_t logged[4] = { s_state.i, s_state.j, s_state.k, s_state.real };
  Board_LogPush(BOARD_LOG_SOURCE_IMU, logged, 4U);
  note(BOARD_IMU_ERR_NONE);
}

/* The three-axis reports share one shape with each other and with the
   rotation vector's first three fields: a five-byte header (id, sequence,
   status, delay) then x, y, z little-endian. One reader, because three
   copies of the same offsets is three places for a byte order to go
   wrong - which it already did once here, on a feature interval. */
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
  s_state.cargoes++;

  if ((channel != SHTP_CH_INPUT) && (channel != SHTP_CH_WAKE))
  {
    return;
  }

  uint16_t at = 0U;

  while (at < len)
  {
    const uint8_t  id = cargo[at];
    const uint16_t step = (uint16_t)shtp_report_len(id);

    /* A zero byte after the last report is padding, not a report. Measured
       at 388 Hz: 46 frame errors in 30 s, every one of them id 0x00 - the
       walk was right to stop and wrong to call it a fault. A cargo cut short
       ends mid-report with real data and is caught by the length test
       below, so the two stay distinguishable. */
    if ((step == 0U) && (id == 0U))
    {
      break;
    }

    if ((step == 0U) || ((at + step) > len))
    {
      /* Which id, because "a report id with no length" and "the cargo was
         cut short" are two different defects and the counter alone cannot
         tell them apart. 0 length means unknown id; a known one means the
         cargo ended mid-report. */
      s_state.last_fault_id = id;
      note(BOARD_IMU_ERR_FRAME);
      return;
    }

    if ((id == SH2_ROTATION_VECTOR) || (id == SH2_GAME_ROTATION_VECTOR))
    {
      take_rotation(id, &cargo[at]);
    }
    else if (id == SH2_REPORT_ACCELEROMETER)
    {
      take_vector(&cargo[at], s_state.accel, &s_state.accel_status);
      s_state.have_accel = true;
    }
    else if (id == SH2_REPORT_GYROSCOPE)
    {
      take_vector(&cargo[at], s_state.gyro, &s_state.gyro_status);
      s_state.have_gyro = true;
    }
    else if (id == SH2_REPORT_MAGNETIC_FIELD)
    {
      take_vector(&cargo[at], s_state.mag, &s_state.mag_status);
      s_state.have_mag = true;
    }

    at = (uint16_t)(at + step);
  }
}

/* Where the staged reset has got to. The whole reason it is staged: the
   part wants NRSTN held for a millisecond and then 120 ms to come up, and
   waiting for either inside the main loop stalls Modbus long enough to time
   a request out. */
#define IMU_STAGE_BUS   0U
#define IMU_STAGE_HOLD  1U
#define IMU_STAGE_WAIT  2U

static uint8_t  s_stage;
static uint32_t s_stage_at;

/** The last Set Feature asked for, so it can be asked for again. The part
  * loses it on every reset and AFE_ON resets it; nothing re-applied it, so
  * one blink of the rail stopped the reports while the loop still said
  * `running`. Zero interval means nothing has been asked for. */
/* EVERY FEATURE ASKED FOR, not the last one. A host wanting the
   quaternion AND the three vectors asks four times, and a reset throws
   away all four - re-applying only the most recent left the other three
   silent while the loop still said `running`, which is the same defect
   the single slot was written to fix, one report wider. Four slots
   because four is what the part is asked for here; a fifth would say so
   by being refused. */
#define IMU_FEATURES 4U

static uint8_t  s_feature_id_of[IMU_FEATURES];
static uint32_t s_feature_us_of[IMU_FEATURES];
static uint8_t  s_features;

/* The most recent, for the command layer's one-feature question. */
static uint8_t  s_feature_id;
static uint32_t s_feature_us;


/** Remember one, replacing an entry for the same report. Returns false
  * when there is no room, which is a refusal the caller must pass on. */
static bool feature_keep(uint8_t report_id, uint32_t interval_us)
{
  for (uint8_t i = 0U; i < s_features; i++)
  {
    if (s_feature_id_of[i] == report_id)
    {
      s_feature_us_of[i] = interval_us;
      return true;
    }
  }
  if (s_features >= IMU_FEATURES)
  {
    return false;
  }
  s_feature_id_of[s_features] = report_id;
  s_feature_us_of[s_features] = interval_us;
  s_features++;
  return true;
}

/** Set when a reset has thrown the feature away and it has not been asked
  * for again yet. Applied from the poll's quiet path, never from the init:
  * `Board_ImuWrite` empties the part before speaking, a reset leaves three
  * announcements queued at 276 bytes each, and doing that inside poll_init
  * held the main loop long enough that the Modbus reply came back late -
  * measured 2026-08-29 as `fc 0x6E: silence` right after the rail returned.
  * Letting the ordinary read path consume the queue first makes the write
  * short. */
static bool s_feature_pending;

/** Which slot the re-apply has got to, since it does one a turn. */
static uint8_t s_feature_next;
static uint32_t s_cargoes_at_reset;   /**< to know the part has spoken */
static uint32_t s_last_cargo_ms;      /**< when the last one arrived   */

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

  /* Remembered only once it took. A request that failed is not what the part
     is doing, and re-applying it after a reset would be a second lie. */
  s_feature_id = report_id;
  s_feature_us = interval_us;
  (void)feature_keep(report_id, interval_us);
  return true;
}

void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us,
                           bool *pending)
{
  *report_id = s_feature_id;
  *interval_us = s_feature_us;
  *pending = s_feature_pending;
}


static void poll_init(void)
{
  switch (s_stage)
  {
    case IMU_STAGE_BUS:
      if (!Board_ImuBusInit())
      {
        note(BOARD_IMU_ERR_INIT);
        s_stage_at = HAL_GetTick();     /* and back off - see below */
        return;
      }
      HAL_GPIO_WritePin(IMU_BOOT_PORT, IMU_BOOT_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
      s_stage = IMU_STAGE_HOLD;
      s_stage_at = HAL_GetTick();
      return;

    case IMU_STAGE_HOLD:
      if ((HAL_GetTick() - s_stage_at) < IMU_RESET_HOLD_MS)
      {
        return;
      }
      HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);
      s_stage = IMU_STAGE_WAIT;
      s_stage_at = HAL_GetTick();
      return;

    default:
      if ((HAL_GetTick() - s_stage_at) < IMU_RESET_WAIT_MS)
      {
        return;
      }
      s_stage = IMU_STAGE_BUS;
      s_state.loop = BOARD_IMU_LOOP_RUN;
      note(BOARD_IMU_ERR_NONE);

      /* Ask again for whatever was asked for before the reset - but not
         here. The part came up with a queue, and emptying it is what makes
         the write long. Flagged, and done below once the queue is gone. */
      s_feature_pending = (s_features != 0U);
      s_feature_next = 0U;
      s_cargoes_at_reset = s_state.cargoes;
      return;
  }
}

void Board_ImuPoll(void)
{
  static uint8_t cargo[IMU_BUF];

  if (!Board_AfeOn())
  {
    /* The part lost its supply. Everything it was told is gone with it, so
       the loop goes back to the beginning rather than carrying on. */
    if (s_state.loop != BOARD_IMU_LOOP_OFF)
    {
      s_state.loop = BOARD_IMU_LOOP_OFF;
      s_state.have = false;
      s_ready = false;
      s_stage = IMU_STAGE_BUS;
      note(BOARD_IMU_ERR_POWER);
    }
    return;
  }

  if (s_state.loop == BOARD_IMU_LOOP_HELD)
  {
    return;                      /* the host is configuring it */
  }

  /* NOT DURING THE OBSERVER'S BORROW. It takes AFE_ON for about 500 ms every
     few seconds to read the NTC, and this part needs longer than that to come
     up - so it would start initialising, lose its supply mid-sequence, and do
     it again on the next borrow. Reset after reset, an errors counter that
     climbs and a part that never reports.

     A borrow is a measurement window, not a power-up. Who holds the rail is
     the reference count's to say, which is what BOARD_USER_THERMAL is for. */
  if (Board_PowerHolds(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    return;
  }


  if (s_state.loop == BOARD_IMU_LOOP_OFF)
  {
    s_state.loop = BOARD_IMU_LOOP_INIT;
  }

  if (s_state.loop == BOARD_IMU_LOOP_INIT)
  {
    poll_init();
    return;
  }

  /* Nothing waiting is the common case and must cost nothing: one GPIO read
     and out. Waiting here would put the main loop's latency on the part.

     `poll_due` is the second half, and here the only half - see the file
     comment. With only the line above, this returned every turn and the part
     streamed rotation vectors nobody collected. */
  /* Nothing queued and the feature still missing: this is the quiet moment
     the re-apply was waiting for. The part has been drained by the reads
     above, so the write's own drain finds nothing and costs 115 us. */
  /* NOT BEFORE THE PART HAS SPOKEN. `!intn_asserted()` is also true in the
     gap between the reset wait ending and the part producing its
     advertisement, and a Set Feature written into that gap is accepted at
     the SHTP level and then discarded - the write returns true, `pending`
     clears, and the part streams nothing for ever.

     Measured 2026-08-29 across an AFE power cycle: the loop came back
     `running` in 0.71 s with feature 5 @ 2500 us and pending false, and no
     report arrived in 15 s. Setting the same feature by hand 0.5 s later
     worked every time, which is what ruled out the part needing longer. */
  if (s_feature_pending && (s_state.cargoes > s_cargoes_at_reset)
      && ((HAL_GetTick() - s_last_cargo_ms) > IMU_QUIET_MS)
      && !intn_asserted())
  {
    /* ONE PER TURN. Each Set Feature empties the part before it
       speaks, and four in a row held the main loop long enough that
       a Modbus reply came back late - the same measurement that put
       this on the quiet path in the first place. */
    if (Board_ImuSetFeature(s_feature_id_of[s_feature_next],
                            s_feature_us_of[s_feature_next]))
    {
      s_feature_next++;
      s_feature_pending = (s_feature_next < s_features);
    }
    return;
  }

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
    s_last_cargo_ms = HAL_GetTick();
    absorb(channel, cargo, len);
  }
}

void Board_ImuState(board_imu_state_t *out)
{
  if (out != NULL)
  {
    *out = s_state;
  }
}

void Board_ImuHold(void)
{
  s_state.loop = BOARD_IMU_LOOP_HELD;
  s_stage = IMU_STAGE_BUS;

  /* A hold that lands mid-staged-reset leaves NRSTN low and the part half
     up, and every command the host then sends is refused or ignored -
     measured: hold, reset, Set Feature, resume, and the loop absorbed
     nothing. Finish it here instead. Blocking is what a command handler is
     allowed to do; the staging exists for the main loop, not for this. */
  if (Board_AfeOn() && !s_ready)
  {
    if (!Board_ImuInit())
    {
      note(BOARD_IMU_ERR_INIT);
    }
  }
}

void Board_ImuResume(void)
{
  /* Back to RUN when the part is still up, because the usual hold is to
     enable a report and going through init would reset the part and throw
     that away - measured: hold, Set Feature, resume, and the loop absorbed
     nothing at all afterwards. Init only when the part is genuinely not
     there, which is what a hold across a reset leaves behind. */
  s_state.loop = s_ready ? BOARD_IMU_LOOP_RUN : BOARD_IMU_LOOP_INIT;
}

uint8_t Board_ImuDrain(uint8_t limit)
{
  /* Reads through s_rx like everything else and throws the result away, so
     it needs no buffer of its own. */
  static uint8_t scratch[8];
  uint8_t channel = 0U;
  uint16_t len = 0U;
  uint8_t taken = 0U;

  /* One empty read is not the end of the queue. The advertisement is 276
     bytes and arrives as several cargoes with gaps between them, so
     stopping at the first gap left the part still talking - and a Set
     Feature sent into that is a write nobody acts on. Measured
     2026-08-27: reset, drain, Set Feature gave no rotation vector ever;
     the same write once the part had actually gone quiet took first time.
     Three empties in a row, a couple of milliseconds apart, is quiet. */
  uint8_t quiet = 0U;

  for (uint8_t i = 0U; (i < limit) && (quiet < 3U); i++)
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


bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len)
{
  if (!s_ready ||
      (channel >= (uint8_t)(sizeof(s_seq) / sizeof(s_seq[0]))))
  {
    return false;
  }

  const size_t n = shtp_build(s_tx, sizeof(s_tx), channel, s_seq[channel],
                              payload, len);
  if ((n == 0U) || (n > IMU_BUF))
  {
    return false;
  }

  /* Empty the part before speaking. H_INTN stays asserted until everything
     queued has been collected, so a write issued on top of it sees the line
     already low, clocks into a part that is mid-sentence, and loses both
     messages. Measured: with a reset's three announcements still queued,
     every write came back SERVER DEVICE FAILURE. */
  (void)Board_ImuDrain(8U);

  /* WAKE, then wait to be let in, then take the bus - and only release WAKE
     once chip select is down. That order is the reference driver's
     (Hillcrest sh2_hal_spi.c, startOpShtp: assert CSN, then "If there is
     stuff to transmit, deassert WAKE and do it now"). Releasing it before
     the transfer let the part go back to sleep between the handshake and
     the first clock. */
  /* WAKE is not optional: measured, a write with PS0 left alone fails
     outright - the part is asleep between transactions and does not hear it.
     What follows a wake on channel 0 is the part announcing itself again,
     which is what waking looks like from here, not a fault. */
  wake(true);

  /* A gate, not best effort. The part answers a wake by asserting H_INTN,
     "at which point the host can initiate SPI accesses" (1.2.4.3) - so
     clocking without it is clocking at a part that is not listening, and
     that is what a write that goes out and changes nothing looks like.
     This was best effort once, on the reading that two product id requests
     had succeeded through it; they had not - the part sends an unsolicited
     product id response after every reset, and that is what was being read.
     Measured on the same board: executable ON, SLEEP and RESET all produced
     the identical answer, which is only possible if none of the payloads
     arrived. */
  if (!wait_intn(IMU_WAKE_WAIT_MS))
  {
    /* Measured on this board: the part answers a wake in under a
       millisecond, and then now and again does not answer one at all -
       twice in ten over eight seconds, and permanently after it had been
       left alone for a few minutes. Releasing WAKE and asserting it again
       recovers the first kind; a reset recovers the second, and costs the
       configuration, which is why it is last and not first. */
    bool woken = false;

    for (uint8_t again = 0U; (again < 3U) && !woken; again++)
    {
      wake(false);
      HAL_Delay(2U);
      wake(true);
      woken = wait_intn(IMU_WAKE_WAIT_MS);
    }

    if (!woken)
    {
      wake(false);
      Board_ImuReset();
      (void)Board_ImuDrain(16U);
      wake(true);
      woken = wait_intn(IMU_WAKE_WAIT_MS);
    }

    if (!woken)
    {
      /* Every acknowledge failed, reset included. Write anyway and mark it
         NOWAKE rather than refuse: refusing made every feature request
         disappear and the part look dead for a day, and a missed edge is not
         proof the part is not listening. The gate still earns its place, so
         it stays as a last resort. What proves the write landed is `updates`
         climbing; nothing here claims it did. */
      note(BOARD_IMU_ERR_NOWAKE);
    }
  }

  cs(true);
  settle();
  wake(false);

  /* Full duplex: the part clocks its own cargo out while this one goes in.
     A transfer sized only to the frame being sent truncates whatever the
     part was saying, and both messages are lost - the write appears to go
     out and nothing acts on it. The reference reads the incoming header in
     the same transaction and clocks to whichever is longer (user guide,
     Interrupt Service: "any SPI operation performed should transfer enough
     bytes to accomodate the transmit buffer"). */
  bool ok = HAL_SPI_TransmitReceive(&hspi2, s_tx, s_rx, (uint16_t)n,
                                    100U) == HAL_OK;

  shtp_header_t incoming;

  if (ok && (n >= SHTP_HEADER_LEN) && shtp_parse_header(s_rx, &incoming) &&
      (incoming.length > (uint16_t)n))
  {
    uint16_t rest = (uint16_t)(incoming.length - (uint16_t)n);

    if (rest > (uint16_t)sizeof(s_rx))
    {
      rest = (uint16_t)sizeof(s_rx);
    }

    /* Discarded: this is the tail of something the part was already sending
       when the write went out, and the caller asked to write, not to read. */
    ok = imu_xfer(s_zeros, s_rx, rest);
  }

  settle();
  cs(false);

  if (!ok)
  {
    return false;
  }

  /* "Each channel and each direction has its own sequence number", 1.3.1.
     Advanced only on a transfer that went out. */
  s_seq[channel]++;
  return true;
}
