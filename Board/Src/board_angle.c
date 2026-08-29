/**
  ******************************************************************************
  * @file    board_angle.c
  * @brief   Allegro A1335 magnetic angle sensor on SPI4.
  *
  * A 20-bit packet per register and a poll loop keeping the newest reading in
  * shared memory - the IMU's shape, for the same reason: a Modbus round trip
  * per register is 45 ms, which is not a sample rate.
  *
  * From A1335-DS Rev. 12: 20-bit packet, mode 3, 0.1 to 10 MHz, CS high at
  * least 200 ns between frames, and the MOSI MSB must be 0 or IER asserts.
  *
  * The register map is not in that datasheet. The addresses come from
  * github.com/ScranchNew/Allegro-A1335-Sensor-library, which drives the same
  * registers over I2C: ANG 0x20, STA 0x22, ERR 0x24, XERR 0x26, TSEN 0x28,
  * FIELD 0x2A.
  *
  * Counts, never degrees: ANG is 360/4096 a count and TSEN eighths of a
  * kelvin, and both scalings belong to the host (invariant 10).
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"
#include "board_power.h"

#include <string.h>

/* PE4 as plain GPIO, and only after HAL_SPI_Init: MspInit hands PE2, PE4,
   PE5 and PE6 to SPI4 as alternate function, so a chip select configured
   ahead of the init is taken straight back. Hardware NSS pulses per frame
   and this part wants one assertion across the whole packet. */
#define ANGLE_CS_PORT GPIOE
#define ANGLE_CS_PIN  GPIO_PIN_4

/* Well under the datasheet's 10 MHz ceiling. The divider is a power of two,
   so at a 100 MHz kernel clock the choice either side is 6.25 MHz or
   1.56 MHz; the lower one costs 13 us a packet and buys the margin. */
#define ANGLE_MAX_HZ 3000000U

/* Registers. See the file header on where these come from - they are not in
   the datasheet in this directory. */
#define ANGLE_REG_ANG   0x20U
#define ANGLE_REG_STA   0x22U
#define ANGLE_REG_ERR   0x24U
#define ANGLE_REG_TSEN  0x28U

/* Figure 31's fields, from the bottom of a 20-bit word. */
#define ANGLE_ADDR_SHIFT 12U
#define ANGLE_DATA_SHIFT 4U
#define ANGLE_RW_SHIFT   18U

/* Figure 31 names this bit R/W and never says which way round. Measured on
   this board rather than assumed - see FINDINGS. */
#define ANGLE_RW_READ    0U
#define ANGLE_RW_WRITE   1U

static bool     s_ready;
static uint32_t s_kernel_hz;
static uint32_t s_bitrate_hz;

/* The loop's own record. One writer - Board_AnglePoll - and one reader, both
   on the main loop, so there is nothing to lock. */
static board_angle_state_t s_state;

/* Where the poll loop looks. Settable, because the register map above came
   from a reference implementation rather than from the datasheet in this
   tree, and a host that finds a better address must not need a rebuild. */
static uint8_t s_poll_reg = ANGLE_REG_ANG;

static uint32_t prescaler_under(uint32_t limit_hz)
{
  static const uint32_t DIVIDERS[] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };

  const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);

  s_kernel_hz = kernel;

  /* A kernel clock of zero means the peripheral clock is not configured, and
     the loop below would read 0 <= limit on the first divider and pick the
     fastest there is. Slowest instead: too slow is a slow read, too fast is
     a part that never answers. */
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

static void cs(bool low)
{
  HAL_GPIO_WritePin(ANGLE_CS_PORT, ANGLE_CS_PIN,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* tCS is 50 ns to the first clock edge and tCS_IDLE is 200 ns between
   frames. One microsecond covers both several times over and costs nothing
   at one packet per read. */
#define ANGLE_SETTLE_US 1U

static void settle(void)
{
  const uint32_t per_us = SystemCoreClock / 1000000U;
  const uint32_t start = Board_Cycles();

  while ((uint32_t)(Board_Cycles() - start) < (ANGLE_SETTLE_US * per_us))
  {
    /* Busy wait - a chip select edge is not worth an interrupt - so it may
       as well feed the STO charge pump while it spins. */
    Board_StoKeepalive();
  }
}

bool Board_AngleInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Re-init rather than patch: HAL latches the mode into CFG1/CFG2 at
     HAL_SPI_Init, so changing the struct alone would configure nothing. */
  if (HAL_SPI_DeInit(&hspi4) != HAL_OK)
  {
    return false;
  }

  hspi4.Init.BaudRatePrescaler = prescaler_under(ANGLE_MAX_HZ);
  /* Four 5-bit words, not one 20-bit one. stm32h7xx_hal_spi.c refuses a
     data size above 16 bits on any instance IS_SPI_HIGHEND_INSTANCE does not
     name, and that is SPI1, SPI2 and SPI3 only - SPI4 returns HAL_ERROR from
     HAL_SPI_Init. Four words of five bits put exactly twenty clock edges on
     the wire under one chip select, which is what the part counts. */
  hspi4.Init.DataSize     = SPI_DATASIZE_5BIT;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  /* Mode 3, and set here because CubeMX's is not what runs: this
     re-inits the peripheral, so the .ioc value is overwritten. */
  hspi4.Init.CLKPolarity  = SPI_POLARITY_HIGH;    /* CPOL = 1 */
  hspi4.Init.CLKPhase     = SPI_PHASE_2EDGE;      /* CPHA = 1 */
  hspi4.Init.NSS          = SPI_NSS_SOFT;
  hspi4.Init.NSSPMode     = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.FirstBit     = SPI_FIRSTBIT_MSB;

  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    return false;
  }

  gpio.Pin = ANGLE_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = 0U;
  HAL_GPIO_Init(ANGLE_CS_PORT, &gpio);
  cs(false);

  s_ready = true;
  return true;
}

bool Board_AngleReady(void)
{
  /* Losing AFE_ON means losing the part, not pausing it: what it was told is
     gone with its supply, so the next command re-initialises rather than
     carrying on. The IMU learned this the expensive way. */
  if (!Board_AfeOn())
  {
    s_ready = false;
    return false;
  }

  return s_ready;
}

void Board_AngleClock(uint32_t *kernel_hz, uint32_t *bitrate_hz)
{
  if (kernel_hz != NULL)  { *kernel_hz = s_kernel_hz; }
  if (bitrate_hz != NULL) { *bitrate_hz = s_bitrate_hz; }
}

/* One packet in, one packet out, chip select down across both. The part
   answers the address it was given in the same frame - there is no second
   transaction to fetch the result. */
#define ANGLE_WORDS 4U          /* 4 x 5 bits = the 20-bit packet */

static bool packet(uint32_t out, uint32_t *in)
{
  uint8_t tx[ANGLE_WORDS];
  uint8_t rx[ANGLE_WORDS] = {0};

  if (!s_ready)
  {
    return false;
  }

  /* Most significant five bits first, right-aligned in each byte: below
     eight bits the peripheral takes the low bits of the buffer element. */
  for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
  {
    tx[i] = (uint8_t)((out >> (5U * (ANGLE_WORDS - 1U - i))) & 0x1FU);
  }

  cs(true);
  settle();

  const bool ok = HAL_SPI_TransmitReceive(&hspi4, tx, rx, ANGLE_WORDS,
                                          100U) == HAL_OK;

  settle();
  cs(false);

  if (ok && (in != NULL))
  {
    uint32_t got = 0U;

    for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
    {
      got = (got << 5U) | (uint32_t)(rx[i] & 0x1FU);
    }
    *in = got;
  }

  return ok;
}

bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc)
{
  uint32_t got = 0U;

  /* SYNC is bit 19 and must be 0. The read/write bit's polarity is the one
     field Figure 31 names without defining; ANGLE_RW_READ is what answered
     on this board. The CRC on MOSI is only checked when the part has been
     programmed to check it, so this sends zeros rather than a polynomial
     the datasheet in this tree does not give. */
  const uint32_t frame = ((uint32_t)ANGLE_RW_READ << ANGLE_RW_SHIFT)
                       | (((uint32_t)reg & 0x3FU) << ANGLE_ADDR_SHIFT);

  /* Two frames, not one. Figure 31 draws MOSI and MISO side by side, but
     the address arrives on MOSI bits 17..12 while MISO has already shifted
     out bits 19..16 - the answer cannot be to the frame carrying the
     address, and it is not. Measured: asking TSEN, FIELD, TSEN in turn
     returned the previous register's value every time. The first frame
     posts the address; the second clocks the answer out.

     The second frame re-posts the same address, so a caller reading one
     register in a loop pays one packet after the first. */
  if (!packet(frame, NULL) || !packet(frame, &got))
  {
    return false;
  }

  if (value != NULL)
  {
    *value = (uint16_t)((got >> ANGLE_DATA_SHIFT) & 0xFFFFU);
  }
  if (crc != NULL)
  {
    *crc = (uint8_t)(got & 0x0FU);
  }

  return true;
}

/** The A1335's own die, centi-degrees C. False if it did not answer.
  *
  * TSEN is eighths of a kelvin - a property of the part, not a calibratable
  * parameter. DUPLICATED in `host/coaxial/angle.py`, which the observer
  * cannot reach; one should go, by cmd_angle appending the converted value.
  *
  * It measures its own DIE. As a board thermometer it FELL 1.88 K during a
  * run that warmed the board (2026-08-28); as its node's, that is signal.
  */
bool Board_AngleDie(int32_t *centidegc)
{
  uint16_t counts = 0U;

  if ((centidegc == NULL) || !Board_AfeOn())
  {
    return false;
  }
  if (!Board_AngleRead(ANGLE_REG_TSEN, &counts, NULL))
  {
    return false;
  }

  const float kelvin = (float)(counts & 0x0FFFU) / 8.0f;

  *centidegc = (int32_t)((kelvin - 273.15f) * 100.0f);
  return true;
}


bool Board_AngleWrite(uint8_t reg, uint8_t value)
{
  const uint32_t frame = ((uint32_t)ANGLE_RW_WRITE << ANGLE_RW_SHIFT)
                       | (((uint32_t)reg & 0x3FU) << ANGLE_ADDR_SHIFT)
                       | ((uint32_t)value << ANGLE_DATA_SHIFT);

  return packet(frame, NULL);
}

static void note(uint8_t err)
{
  s_state.error = err;
  if (err != BOARD_ANGLE_ERR_NONE)
  {
    s_state.errors++;
  }
}

void Board_AnglePoll(void)
{
  uint16_t value = 0U;
  uint8_t  crc = 0U;

  /* AFE_ON powers this part too, the same way it powers the BNO08X. A part
     without its supply is not a part that reads zero. */
  if (!Board_AfeOn())
  {
    if (s_state.loop != BOARD_ANGLE_LOOP_OFF)
    {
      s_state.loop = BOARD_ANGLE_LOOP_OFF;
      s_state.have = false;
      s_ready = false;
      note(BOARD_ANGLE_ERR_POWER);
    }
    return;
  }

  if (s_state.loop == BOARD_ANGLE_LOOP_HELD)
  {
    return;                        /* the host is configuring it */
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


  if (s_state.loop == BOARD_ANGLE_LOOP_OFF)
  {
    if (!Board_AngleInit())
    {
      note(BOARD_ANGLE_ERR_INIT);
      return;                      /* try again next time round */
    }
    s_state.loop = BOARD_ANGLE_LOOP_RUN;
    note(BOARD_ANGLE_ERR_NONE);
    return;
  }

  if (!Board_AngleRead(s_poll_reg, &value, &crc))
  {
    note(BOARD_ANGLE_ERR_READ);
    return;
  }

  /* All ones is what an absent or unpowered part clocks out, and it is not a
     reading: the low twelve bits would be a plausible angle. */
  if (value == 0xFFFFU)
  {
    s_state.have = false;
    note(BOARD_ANGLE_ERR_SILENT);
    return;
  }

  s_state.reg   = s_poll_reg;
  s_state.value = value;
  s_state.crc   = crc;
  s_state.have  = true;
  s_state.updates++;

  const int16_t logged[3] = { (int16_t)value, (int16_t)crc,
                              (int16_t)s_poll_reg };
  Board_LogPush(BOARD_LOG_SOURCE_ANGLE, logged, 3U);
  note(BOARD_ANGLE_ERR_NONE);
}

void Board_AngleState(board_angle_state_t *out)
{
  if (out != NULL)
  {
    *out = s_state;
  }
}

void Board_AngleHold(void)
{
  s_state.loop = BOARD_ANGLE_LOOP_HELD;

  /* A hold hands the host a part that is up, the way the IMU's does: one
     that landed before the bus was configured left every command after it
     refused for a reason that had nothing to do with the part. */
  if (!s_ready)
  {
    if (!Board_AngleInit())
    {
      note(BOARD_ANGLE_ERR_INIT);
    }
  }
}

void Board_AngleResume(void)
{
  s_state.loop = s_ready ? BOARD_ANGLE_LOOP_RUN : BOARD_ANGLE_LOOP_OFF;
}

bool Board_AnglePollReg(uint8_t reg)
{
  if (reg > 0x3FU)
  {
    return false;                  /* six address bits, Figure 31 */
  }

  s_poll_reg = reg;
  s_state.have = false;
  return true;
}

uint8_t Board_AnglePollRegGet(void)
{
  return s_poll_reg;
}
