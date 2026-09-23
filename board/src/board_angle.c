/** board_angle.c - Allegro A1335 magnetic angle sensor on SPI4. */
#include "board_limits.h"
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "board_units.h"

#include <string.h>

/* PE4 as plain GPIO, and only after HAL_SPI_Init: MspInit hands PE2, PE4,
   PE5 and PE6 to SPI4 as alternate function, so a chip select configured
   ahead of the init is taken straight back. */
#define ANGLE_CS_PORT GPIOE
#define ANGLE_CS_PIN  GPIO_PIN_4

/* Registers. */
#define ANGLE_REG_ANG   0x20U
#define ANGLE_REG_STA   0x22U
#define ANGLE_REG_ERR   0x24U
#define ANGLE_REG_TSEN  0x28U
#define ANGLE_TEMP_LSB_PER_K 8.0f   /* TSEN counts kelvin in eighths */

/* Figure 31's fields, from the bottom of a 20-bit word. */
#define ANGLE_ADDR_SHIFT 12U
#define ANGLE_DATA_SHIFT 4U
#define ANGLE_RW_SHIFT   18U

/* Figure 31 names this bit R/W and never says which way round. */
#define ANGLE_RW_READ    0U
#define ANGLE_RW_WRITE   1U
#define ANGLE_REG_MASK   0x3FU    /* a 6-bit register address */
#define ANGLE_CRC_MASK   0x0FU    /* the 4-bit CRC a reply ends with */
#define ANGLE_TEMP_MASK  0x0FFFU  /* TSEN's 12-bit count */
#define ANGLE_SPI_TIMEOUT_MS 100U

/** The angle sensor's state: the link, the latest word and its register, and
    the hold. */
static struct
{
  bool ready;
  uint32_t kernel_hz;
  uint32_t bitrate_hz;

  /* The loop's own record. */
  board_angle_state_t state;

  /* Where the poll loop looks. */
  uint8_t poll_reg;
} s = {
  .poll_reg = ANGLE_REG_ANG
};

static uint32_t prescaler_under(uint32_t limit_hz)
{
  s.kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);
  return board_spi_prescaler(s.kernel_hz, limit_hz, &s.bitrate_hz);
}

static void cs(bool low)
{
  HAL_GPIO_WritePin(ANGLE_CS_PORT, ANGLE_CS_PIN,
                    low ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void settle(void)
{
  board_spin_us(ANGLE_SETTLE_US);
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
  /* Four 5-bit words, not one 20-bit one. */
  hspi4.Init.DataSize     = SPI_DATASIZE_5BIT;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  /* Mode 3, and set here because CubeMX's is not what runs: this re-inits
     the peripheral, so the .ioc value is overwritten. */
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

  s.ready = true;
  return true;
}

bool Board_AngleReady(void)
{
  /* Losing AFE_ON means losing the part, not pausing it: what it was told is
     gone with its supply, so the next command re-initialises rather than
     carrying on. */
  if (!Board_AfeOn())
  {
    s.ready = false;
    return false;
  }

  return s.ready;
}

void Board_AngleClock(uint32_t *kernel_hz, uint32_t *bitrate_hz)
{
  if (kernel_hz != NULL)  { *kernel_hz = s.kernel_hz; }
  if (bitrate_hz != NULL) { *bitrate_hz = s.bitrate_hz; }
}

/* One packet in, one packet out, chip select down across both. */
#define ANGLE_WORDS 4U          /* 4 x 5 bits = the 20-bit packet */
#define ANGLE_WORD_BITS 5U
#define ANGLE_WORD_MASK 0x1FU

static bool packet(uint32_t out, uint32_t *in)
{
  uint8_t tx[ANGLE_WORDS];
  uint8_t rx[ANGLE_WORDS] = {0};

  if (!s.ready)
  {
    return false;
  }

  /* Most significant five bits first, right-aligned in each byte: below
     eight bits the peripheral takes the low bits of the buffer element. */
  for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
  {
    tx[i] = (uint8_t)((out >> (ANGLE_WORD_BITS * (ANGLE_WORDS - 1U - i))) & ANGLE_WORD_MASK);
  }

  cs(true);
  settle();

  const bool ok = HAL_SPI_TransmitReceive(&hspi4, tx, rx, ANGLE_WORDS,
                                          ANGLE_SPI_TIMEOUT_MS) == HAL_OK;

  settle();
  cs(false);

  if (ok && (in != NULL))
  {
    uint32_t got = 0U;

    for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
    {
      got = (got << ANGLE_WORD_BITS) | (uint32_t)(rx[i] & ANGLE_WORD_MASK);
    }
    *in = got;
  }

  return ok;
}

bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc)
{
  uint32_t got = 0U;

  /* SYNC is bit 19 and must be 0. */
  const uint32_t frame = ((uint32_t)ANGLE_RW_READ << ANGLE_RW_SHIFT)
                       | (((uint32_t)reg & ANGLE_REG_MASK) << ANGLE_ADDR_SHIFT);

  /* Two frames, not one. */
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
    *crc = (uint8_t)(got & ANGLE_CRC_MASK);
  }

  return true;
}

/** The A1335's own die, centi-degrees C. */
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

  const float kelvin = (float)(counts & ANGLE_TEMP_MASK) / ANGLE_TEMP_LSB_PER_K;

  *centidegc = (int32_t)((kelvin - KELVIN_AT_ZERO_C) * CENTI_PER_UNIT);
  return true;
}

bool Board_AngleWrite(uint8_t reg, uint8_t value)
{
  const uint32_t frame = ((uint32_t)ANGLE_RW_WRITE << ANGLE_RW_SHIFT)
                       | (((uint32_t)reg & ANGLE_REG_MASK) << ANGLE_ADDR_SHIFT)
                       | ((uint32_t)value << ANGLE_DATA_SHIFT);

  return packet(frame, NULL);
}

static void note(uint8_t err)
{
  s.state.error = err;
  if (err != BOARD_ANGLE_ERR_NONE)
  {
    s.state.errors++;
  }
}

/* The part's supply went: the loop is off, nothing is held, and the error
   says why - once, not every poll. */
static void power_lost(void)
{
  if (s.state.loop == BOARD_ANGLE_LOOP_OFF)
  {
    return;
  }
  s.state.loop = BOARD_ANGLE_LOOP_OFF;
  s.state.have = false;
  s.ready = false;
  note(BOARD_ANGLE_ERR_POWER);
}

void Board_AnglePoll(void)
{
  uint16_t value = 0U;
  uint8_t  crc = 0U;

  /* AFE_ON powers this part too, the same way it powers the BNO08X. */
  if (!Board_AfeOn())
  {
    power_lost();
    return;
  }

  if (s.state.loop == BOARD_ANGLE_LOOP_HELD)
  {
    return;                        /* the host is configuring it */
  }

  /* NOT DURING THE OBSERVER'S BORROW. */
  if (Board_PowerHolds(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    return;
  }

  if ((s.state.loop == BOARD_ANGLE_LOOP_OFF) && !Board_AngleInit())
  {
    note(BOARD_ANGLE_ERR_INIT);
    return;                        /* try again next time round */
  }
  if (s.state.loop == BOARD_ANGLE_LOOP_OFF)
  {
    s.state.loop = BOARD_ANGLE_LOOP_RUN;
    note(BOARD_ANGLE_ERR_NONE);
    return;
  }

  if (!Board_AngleRead(s.poll_reg, &value, &crc))
  {
    note(BOARD_ANGLE_ERR_READ);
    return;
  }

  /* All ones is what an absent or unpowered part clocks out, and it is not a
     reading: the low twelve bits would be a plausible angle. */
  if (value == 0xFFFFU)
  {
    s.state.have = false;
    note(BOARD_ANGLE_ERR_SILENT);
    return;
  }

  s.state.reg   = s.poll_reg;
  s.state.value = value;
  s.state.crc   = crc;
  s.state.have  = true;
  s.state.updates++;

  const int16_t logged[3] = { (int16_t)value, (int16_t)crc,
                              (int16_t)s.poll_reg };
  Board_LogPush(BOARD_LOG_SOURCE_ANGLE, logged, 3U);
  note(BOARD_ANGLE_ERR_NONE);
}

void Board_AngleState(board_angle_state_t *out)
{
  if (out != NULL)
  {
    *out = s.state;
  }
}

void Board_AngleHold(void)
{
  s.state.loop = BOARD_ANGLE_LOOP_HELD;

  /* A hold hands the host a part that is up, the way the IMU's does: one
     that landed before the bus was configured left every command after it
     refused for a reason that had nothing to do with the part. */
  if (!s.ready && !Board_AngleInit())
  {
    note(BOARD_ANGLE_ERR_INIT);
  }
}

void Board_AngleResume(void)
{
  s.state.loop = s.ready ? BOARD_ANGLE_LOOP_RUN : BOARD_ANGLE_LOOP_OFF;
}

bool Board_AnglePollReg(uint8_t reg)
{
  if (reg > ANGLE_REG_MASK)
  {
    return false;                  /* six address bits, Figure 31 */
  }

  s.poll_reg = reg;
  s.state.have = false;
  return true;
}

uint8_t Board_AnglePollRegGet(void)
{
  return s.poll_reg;
}
