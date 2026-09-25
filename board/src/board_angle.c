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
  bool dma;                 /* the streams pointed at SPI4 */
  uint32_t kernel_hz;
  uint32_t bitrate_hz;
  uint32_t cr1;             /* CR1 and CFG1 as HAL_SPI_Init left them */
  uint32_t cfg1;

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

/* One packet in, one packet out, chip select down across both. */
#define ANGLE_WORDS 4U          /* 4 x 5 bits = the 20-bit packet */
#define ANGLE_WORD_BITS 5U
#define ANGLE_WORD_MASK 0x1FU

/* SPI4's words by DMA1: stream 0 receives, stream 1 sends (DMAMUX1 channels
   0 and 1), in AXI SRAM - DMA1 reaches it, not DTCM. The receiver's end
   interrupts: main() starts a packet and finds it in later. */
#define ANGLE_DMA_RX    DMA1_Stream0
#define ANGLE_DMA_TX    DMA1_Stream1
#define ANGLE_DMA_IRQ   DMA1_Stream0_IRQn
#define ANGLE_DMA_PRIO  6U
#define ANGLE_DMA_FLAGS (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 \
                         | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CTCIF1 \
                         | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 \
                         | DMA_LIFCR_CFEIF1)
/* Bytes, the memory side stepping: in (DIR 00) and out (DIR 01). */
#define ANGLE_DMA_CR_RX (DMA_SxCR_MINC | DMA_SxCR_TCIE)
#define ANGLE_DMA_CR_TX (DMA_SxCR_MINC | DMA_SxCR_DIR_0)

static uint8_t s_tx[ANGLE_WORDS] __attribute__((section(".buffers")));
static uint8_t s_rx[ANGLE_WORDS] __attribute__((section(".buffers")));
static volatile bool s_done;

void DMA1_Stream0_IRQHandler(void)
{
  DMA1->LIFCR = DMA_LIFCR_CTCIF0;
  s_done = true;
}

static void dma_init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  ANGLE_DMA_RX->CR = 0U;
  ANGLE_DMA_TX->CR = 0U;
  DMAMUX1_Channel0->CCR = DMA_REQUEST_SPI4_RX;
  DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI4_TX;
  ANGLE_DMA_RX->PAR  = (uint32_t)&SPI4->RXDR;
  ANGLE_DMA_RX->M0AR = (uint32_t)s_rx;
  ANGLE_DMA_TX->PAR  = (uint32_t)&SPI4->TXDR;
  ANGLE_DMA_TX->M0AR = (uint32_t)s_tx;
  /* Written back each packet rather than read: a register read is cheap on
     the part and costly in the emulator. */
  s.cr1  = SPI4->CR1;
  s.cfg1 = SPI4->CFG1;
  HAL_NVIC_SetPriority(ANGLE_DMA_IRQ, ANGLE_DMA_PRIO, 0U);
  HAL_NVIC_EnableIRQ(ANGLE_DMA_IRQ);
  s.dma = true;
}

/* One packet started, RM0433's order for SPI with DMA: the receiver's
   requests, both streams, the transmitter's, SPE, CSTART. s_done when in. */
static void xfer_start(uint32_t out)
{
  /* Most significant five bits first, right-aligned in each byte: below
     eight bits the peripheral takes the low bits of the buffer element. */
  for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
  {
    s_tx[i] = (uint8_t)((out >> (ANGLE_WORD_BITS * (ANGLE_WORDS - 1U - i))) & ANGLE_WORD_MASK);
  }
  s_done = false;
  DMA1->LIFCR = ANGLE_DMA_FLAGS;
  SPI4->CR2 = ANGLE_WORDS;                             /* TSIZE */
  SPI4->CFG1 = s.cfg1 | SPI_CFG1_RXDMAEN;
  ANGLE_DMA_RX->NDTR = ANGLE_WORDS;
  ANGLE_DMA_RX->CR = ANGLE_DMA_CR_RX | DMA_SxCR_EN;
  ANGLE_DMA_TX->NDTR = ANGLE_WORDS;
  ANGLE_DMA_TX->CR = ANGLE_DMA_CR_TX | DMA_SxCR_EN;
  SPI4->CFG1 = s.cfg1 | SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;
  SPI4->CR1 = s.cr1 | SPI_CR1_SPE;
  SPI4->CR1 = s.cr1 | SPI_CR1_SPE | SPI_CR1_CSTART;
}

/* The packet's end: EOT cleared, SPE and the requests down. */
static void xfer_close(void)
{
  SPI4->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
  SPI4->CR1 = s.cr1;
  SPI4->CFG1 = s.cfg1;
}

/* A packet given up: the streams stopped, then the end. */
static void xfer_abort(void)
{
  if (!s.dma)
  {
    return;
  }
  ANGLE_DMA_RX->CR = 0U;
  ANGLE_DMA_TX->CR = 0U;
  xfer_close();
  s_done = false;
}

static uint32_t timeout_cycles(void)
{
  return ANGLE_SPI_TIMEOUT_MS * (SystemCoreClock / MS_PER_S);
}

static uint32_t settle_cycles(void)
{
  return ANGLE_SETTLE_US * (SystemCoreClock / US_PER_S);
}

/* The 20 bits the last packet brought back. */
static uint32_t word_in(void)
{
  uint32_t got = 0U;

  for (uint8_t i = 0U; i < ANGLE_WORDS; i++)
  {
    got = (got << ANGLE_WORD_BITS) | (uint32_t)(s_rx[i] & ANGLE_WORD_MASK);
  }
  return got;
}

/* A read's frame: SYNC, bit 19, is 0. */
static uint32_t read_frame(uint8_t reg)
{
  return ((uint32_t)ANGLE_RW_READ << ANGLE_RW_SHIFT)
       | (((uint32_t)reg & ANGLE_REG_MASK) << ANGLE_ADDR_SHIFT);
}

/* The poll's read, stepped from main(): two packets, each chip select down,
   a settle, the transfer, a settle, chip select up, and a settle up before
   the second - nothing waited for in place. */
typedef enum
{
  STEP_IDLE = 0,
  STEP_SETUP,     /* chip select down, settling */
  STEP_XFER,      /* the transfer in flight */
  STEP_HOLD,      /* in, settling before chip select goes up */
  STEP_GAP        /* chip select up between the two packets */
} angle_step_t;

static struct
{
  angle_step_t step;
  uint8_t  packet;
  uint8_t  reg;
  uint32_t at;
} r;

static bool waited(uint32_t cycles)
{
  return (uint32_t)(Board_Cycles() - r.at) >= cycles;
}

/* The poll's read given up where it stands: the bus back, chip select up. */
static void read_abort(void)
{
  if (r.step != STEP_IDLE)
  {
    xfer_abort();
    cs(false);
    r.step = STEP_IDLE;
  }
}

/* The read as far as it goes without waiting: true with the reply's 20 bits
   once both packets are in; *failed if a transfer never finished. */
static bool read_step(uint32_t *got, bool *failed)
{
  *failed = false;
  for (;;)
  {
    switch (r.step)
    {
    case STEP_IDLE:
      r.reg = s.poll_reg;
      r.packet = 0U;
      cs(true);
      r.at = Board_Cycles();
      r.step = STEP_SETUP;
      break;

    case STEP_SETUP:
      if (!waited(settle_cycles()))
      {
        return false;
      }
      xfer_start(read_frame(r.reg));
      r.at = Board_Cycles();
      r.step = STEP_XFER;
      break;

    case STEP_XFER:
      if (!s_done)
      {
        if (waited(timeout_cycles()))
        {
          read_abort();
          *failed = true;
        }
        return false;
      }
      xfer_close();
      r.at = Board_Cycles();
      r.step = STEP_HOLD;
      break;

    case STEP_HOLD:
      if (!waited(settle_cycles()))
      {
        return false;
      }
      cs(false);
      if (r.packet == 0U)
      {
        r.packet = 1U;            /* the reply comes back on the second */
        r.at = Board_Cycles();
        r.step = STEP_GAP;
        break;
      }
      *got = word_in();
      r.step = STEP_IDLE;
      return true;

    case STEP_GAP:
      if (!waited(settle_cycles()))
      {
        return false;
      }
      cs(true);
      r.at = Board_Cycles();
      r.step = STEP_SETUP;
      break;

    default:
      r.step = STEP_IDLE;
      return false;
    }
  }
}

bool Board_AngleInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  read_abort();

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
  dma_init();

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

/* One packet, waited for: the host's reads and writes. A poll's read in
   flight gives the bus up first. */
static bool packet(uint32_t out, uint32_t *in)
{
  if (!s.ready)
  {
    return false;
  }
  read_abort();

  cs(true);
  settle();
  xfer_start(out);

  const uint32_t t0 = Board_Cycles();
  while (!s_done)
  {
    if ((uint32_t)(Board_Cycles() - t0) >= timeout_cycles())
    {
      xfer_abort();
      cs(false);
      return false;
    }
    Board_StoKeepalive();
  }
  xfer_close();

  settle();
  cs(false);

  if (in != NULL)
  {
    *in = word_in();
  }
  return true;
}

bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc)
{
  uint32_t got = 0U;

  const uint32_t frame = read_frame(reg);

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
  read_abort();
  s.state.loop = BOARD_ANGLE_LOOP_OFF;
  s.state.have = false;
  s.ready = false;
  note(BOARD_ANGLE_ERR_POWER);
}

void Board_AnglePoll(void)
{
  uint32_t got = 0U;
  bool failed = false;

  /* AFE_ON powers this part too, the same way it powers the BNO08X. */
  if (!Board_AfeOn())
  {
    power_lost();
    return;
  }

  if (s.state.loop == BOARD_ANGLE_LOOP_HELD)
  {
    read_abort();
    return;                        /* the host is configuring it */
  }

  /* Not during the observer's borrow. */
  if (Board_PowerHolds(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    read_abort();
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

  if (!read_step(&got, &failed))
  {
    if (failed)
    {
      note(BOARD_ANGLE_ERR_READ);
    }
    return;
  }

  const uint16_t value = (uint16_t)((got >> ANGLE_DATA_SHIFT) & 0xFFFFU);
  const uint8_t  crc = (uint8_t)(got & ANGLE_CRC_MASK);

  /* All ones is what an absent or unpowered part clocks out, and it is not a
     reading: the low twelve bits would be a plausible angle. */
  if (value == 0xFFFFU)
  {
    s.state.have = false;
    note(BOARD_ANGLE_ERR_SILENT);
    return;
  }

  s.state.reg   = r.reg;
  s.state.value = value;
  s.state.crc   = crc;
  s.state.have  = true;
  s.state.updates++;

  const int16_t logged[3] = { (int16_t)value, (int16_t)crc,
                              (int16_t)r.reg };
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
  read_abort();

  /* A hold hands the host a part that is up, the way the IMU's does: one
     that lands before the bus is configured leaves every later command
     refused for a reason that has nothing to do with the part. */
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
