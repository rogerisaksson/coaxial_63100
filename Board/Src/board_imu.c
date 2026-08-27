/**
  ******************************************************************************
  * @file    board_imu.c
  * @brief   The BNO08X on SPI2: the bytes, and nothing about what they mean.
  *
  * SHTP framing and SH-2 decoding are in Shtp/, hardware-free and tested on a
  * host. This file is the other half - chip select, clocking, and the length
  * field's one job of saying how many bytes to ask for next.
  *
  * What CubeMX generated does not match the part, and this fixes what it can
  * at runtime rather than editing Core/, which CubeMX owns and regenerates.
  * Datasheet BNO080_085 v1.17, in datasheets/:
  *
  *   - SPI mode 3, CPOL=1 CPHA=1 (1.2.4.2, 6.5.2); the .ioc says mode 0.
  *   - byte oriented, "all data is passed in 8-bit segments" (1.2.4.2); the
  *     .ioc says SPI_DATASIZE_4BIT.
  *   - "Any number of bytes can be transferred in a single transaction (chip
  *     select assertion)" (1.2.4.2), so CS must be held across header and
  *     cargo both. Hardware NSS with NSSP pulses it per frame, which would
  *     end the transaction between the two. PB12 is driven as plain GPIO
  *     instead, which overrides the MSP's alternate-function setting.
  *
  * What cannot be fixed here, because it is a wire and not a register: the
  * BNO08X signals with H_INTN and is reset with NRSTN, and neither is
  * assigned to a pin. Without H_INTN this polls the four-byte header instead
  * of waiting to be told - workable for reading a product id at a bench, and
  * not workable for streaming: the datasheet asks for H_INTN to be serviced
  * "within 1/10 of the fastest sensor period" (1.2.4.1) or the part starves.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"
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

/* Figure 6-8: tnrst is 10 ns minimum, t1 is 90 ms of internal initialisation
   before the part is ready, t2 another 4 ms of configuration. One millisecond
   of reset is four orders of magnitude past the minimum and costs nothing;
   120 ms afterwards leaves margin on t1+t2 without a pin to be told on. */
#define IMU_RESET_HOLD_MS 1U
#define IMU_RESET_WAIT_MS 120U

/* One transaction's worth. Sized for the SHTP advertisement, which is the
   largest thing the part sends unprompted: measured on this board, 276 bytes
   including the header, carrying the channel map and version strings. Sixty
   four was enough for a product id response and refused the advertisement
   outright, which is what CMD_ERR_DEVICE on every read after a reset was. */
#define IMU_BUF 320U

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

/* Figure 6-8 puts the ceiling at 3 MHz. Aimed well under it rather than at
   it: the divider is a power of two, so the choice at a 190 MHz kernel clock
   is 2.97 MHz or 1.48 MHz, and the first leaves nothing for rise times on a
   real board. Measured at 2.97: every read came back FF. */
#define IMU_MAX_HZ 2000000U

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

/* How long to wait for the part to say it has something. Anything longer
   would hold the Modbus link past the master's patience for a part that is
   simply idle, which is not an error. */
#define IMU_INTN_WAIT_MS 5U

/* Waking is not polling. Asserting PS0/WAKE takes the part out of a sleep
   state and it answers by asserting H_INTN "at which point the host can
   initiate SPI accesses" (1.2.4.3) - that is a wake-up, not a sample period.
   Five milliseconds was enough for a part that was already awake and not for
   one that was not: measured, every write failed once the reset's queue had
   been drained. */
#define IMU_WAKE_WAIT_MS 50U

static bool intn_asserted(void)
{
  return HAL_GPIO_ReadPin(IMU_INTN_PORT, IMU_INTN_PIN) == GPIO_PIN_RESET;
}

/* How often to clock a header out when H_INTN has not asserted. The part
   signals with H_INTN and this board wires it to PD8 - SPI0.INT on the
   MCU sheet, a straight wire - but measured 2026-08-27 it never goes low:
   77 reads across the 1.2 s after a reset, all high, while a direct probe
   in the same window clocked out real SHTP cargoes (`14 00 02 00 f1 00 84`,
   a 20-byte message on channel 2). The part produces and does not ask.

   So the header is polled, which is what this file's own description has
   always said happens without H_INTN. Rate limited because it is not free:
   a four-byte transfer at 1.48 MHz is 27 us, and the main loop also carries
   Modbus, whose t1.5 at 115200 is 143 us. At 1 kHz that is 2.7 % of the
   loop against a report interval of 20 ms - fifty polls per report, which
   is enough to never be the reason one is late.

   Raw CYCCNT and unsigned subtraction, so the wrap costs nothing
   (invariant 2). */
#define IMU_POLL_HZ 1000U

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
  const uint32_t until = HAL_GetTick() + ms;

  while (!intn_asserted())
  {
    if (HAL_GetTick() > until)
    {
      return false;
    }
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

/* Figure 6-6: tcssu, chip select to the first clock edge, is 0.1 us minimum,
   and tcssh, the hold after the last one, 16.83 ns. A GPIO write followed
   straight away by HAL_SPI_TransmitReceive is a couple of core cycles - 27 ns
   at the 75 MHz this board currently runs at - so the setup was a quarter of
   what the part asks for. One microsecond is ten times the requirement and
   costs nothing at these transfer sizes. */
#define IMU_SETTLE_US 1U

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

  /* H_INTN is the signal that says "ready" and this board has no pin for it,
     so the wait is the datasheet's number rather than an observation. */
  HAL_Delay(IMU_RESET_WAIT_MS);
}

bool Board_ImuBusInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* AFE_ON powers the part, not just the analog front end. Measured: with it
     off the BNO08X still drives MISO and still resets - enough to read a
     valid 276-byte advertisement from - but no write is ever acted on, and
     the wake handshake answers sometimes and not others. That cost a day:
     every symptom looked like SPI. Refuse instead of half-working. */
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

/** One SPI transfer, split so the STO charge pump keeps getting edges.

   A 320-byte cargo at 1.48 MHz is 1.73 ms of blocking transfer, and the
   keepalive latch holds only a few hundred microseconds (FINDINGS). Chip
   select is ours - NSS_SOFT, PB12 by hand - and is NOT touched here, so the
   part still sees one unbroken transaction however this is chunked.
   Releasing it between chunks is the FF FF FF FF bug again.

   8 bytes is 43 us at 1.48 MHz, against the 52 us the main loop already
   takes per iteration once the IMU is being polled. Finer buys nothing the
   loop does not already cost; coarser makes this the worst gap in the
   system. */
#define IMU_CHUNK 8U

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
  }
}

/* One cargo into the shared record. Only channel 3 and channel 4 carry
   sensor reports; the rest is the part talking about itself and is counted,
   not kept. */
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

    if ((step == 0U) || ((at + step) > len))
    {
      note(BOARD_IMU_ERR_FRAME);
      return;
    }

    if ((id == SH2_ROTATION_VECTOR) || (id == SH2_GAME_ROTATION_VECTOR))
    {
      s_state.report_id = id;
      s_state.status    = cargo[at + 2U];
      s_state.i    = (int16_t)((uint16_t)cargo[at + 4U] |
                               ((uint16_t)cargo[at + 5U] << 8));
      s_state.j    = (int16_t)((uint16_t)cargo[at + 6U] |
                               ((uint16_t)cargo[at + 7U] << 8));
      s_state.k    = (int16_t)((uint16_t)cargo[at + 8U] |
                               ((uint16_t)cargo[at + 9U] << 8));
      s_state.real = (int16_t)((uint16_t)cargo[at + 10U] |
                               ((uint16_t)cargo[at + 11U] << 8));
      s_state.have = true;
      s_state.updates++;

      const int16_t logged[4] = { s_state.i, s_state.j, s_state.k,
                                  s_state.real };
      Board_LogPush(BOARD_LOG_SOURCE_IMU, logged, 4U);
      note(BOARD_IMU_ERR_NONE);
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

     `poll_due` is the second half, and on this board it is the only half:
     H_INTN reaches PD8 - SPI0.INT on the MCU sheet, a straight wire - and
     measured 2026-08-27 it never goes low, 77 reads across the 1.2 s after
     a reset, while a direct probe in the same window clocked out real
     cargoes. With only the line above, this returned every turn and the
     part streamed rotation vectors nobody collected. */
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
      /* Every acknowledge failed, including a reset. Write anyway, and say
         so - because on this board H_INTN never asserts at all. Measured
         2026-08-27: 77 reads of PD8 across the 1.2 s after a reset, all
         high, while a direct probe in the same window clocked out real
         SHTP cargoes. Refusing here is what made every feature request
         disappear and the part look dead for a day.

         The gate's own argument still stands where H_INTN works - a write
         clocked at a part that is not listening changes nothing and looks
         like it worked - so this does not remove it, it takes it as the
         last resort it now is and marks the reading NOWAKE. What proves
         the write landed is `updates` climbing afterwards; nothing here
         claims it did. */
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
