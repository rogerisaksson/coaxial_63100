/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   Discrete I/O: AFE_ON and PE15, plus the console-mode request.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include "link.h"


/* Every pin this board uses for something, reserved ones included. The
   direction is the MCU's: PB2 drives the AFE switch, PE15 senses it back
   inverted (HARDWARE.md, Discrete I/O). Kept here rather than in testrig.c
   so "what is PB10" has one answer. */
typedef struct
{
  char        port;
  uint8_t     number;
  const char *pin;
  uint8_t     dir;
  const char *signal;
  bool        usable;
} DigitalDesc;

static const DigitalDesc s_digital[] =
{
  { 'B',  2U, "PB2",  BOARD_DIR_OUT,   "AFE_ON",              true  },
  { 'E', 15U, "PE15", BOARD_DIR_IN,    "nFAULT",              true  },
  { 'B', 10U, "PB10", BOARD_DIR_OUT,   "USART3_TX",           false },
  { 'B', 11U, "PB11", BOARD_DIR_IN,    "USART3_RX",           false },
  { 'A', 13U, "PA13", BOARD_DIR_INOUT, "JTMS/SWDIO",          false },
  { 'A', 14U, "PA14", BOARD_DIR_IN,    "JTCK/SWCLK",          false },
  { 'A', 15U, "PA15", BOARD_DIR_IN,    "JTDI",                false },
  { 'B',  3U, "PB3",  BOARD_DIR_OUT,   "JTDO/TRACESWO",       false },
  { 'B',  4U, "PB4",  BOARD_DIR_IN,    "NJTRST",              false },
  { 'B', 12U, "PB12", BOARD_DIR_OUT,   "SPI2_NSS/H_CSN",      false },
  { 'B', 13U, "PB13", BOARD_DIR_OUT,   "SPI2_SCK",            false },
  { 'B', 14U, "PB14", BOARD_DIR_IN,    "SPI2_MISO",           false },
  { 'B', 15U, "PB15", BOARD_DIR_OUT,   "SPI2_MOSI",           false },
  { 'D',  8U, "PD8",  BOARD_DIR_IN,    "IMU H_INTN",          false },
  { 'D',  9U, "PD9",  BOARD_DIR_OUT,   "IMU PS0/WAKE",        false },
  { 'D', 10U, "PD10", BOARD_DIR_OUT,   "IMU NRSTN",           false },
  { 'D', 11U, "PD11", BOARD_DIR_OUT,   "IMU BOOTN",           false },
};

/* What is fitted, as against what it is wired to. One row per part, and the
   whole stack above reads it off the wire: add a part here and board_info,
   the MCP tools and the local model all report it without being told twice.
   `power` names what must be on for the part to work at all - the BNO08X
   answers reads with AFE_ON low and acts on no write, which is the reason
   this column exists. */
typedef struct
{
  const char *name;
  const char *what;
  const char *where;
  const char *power;
  uint8_t     probe;
} PartDesc;

#define PART_PROBE_NONE 0U
#define PART_PROBE_AFE  1U
#define PART_PROBE_IMU  2U

static const PartDesc s_parts[] =
{
  { "STM32H753VIT6", "the MCU, 475 MHz", "on board", "", PART_PROBE_NONE },
  { "BNO08X", "9-axis IMU, SHTP", "SPI2", "AFE_ON", PART_PROBE_IMU },
  { "AFE", "phase chains + ADC ref", "PB2 switches it", "", PART_PROBE_AFE },
  { "NTC", "thermistor", "ADC3", "AFE_ON", PART_PROBE_AFE },
  { "DC link divider", "49.9k/2.2k, 78.15 V FS", "ADC", "AFE_ON",
    PART_PROBE_AFE },
  { "USART3", "console or Modbus RTU", "PB10/PB11", "", PART_PROBE_NONE },
};

uint8_t Board_PartCount(void)
{
  return (uint8_t)(sizeof(s_parts) / sizeof(s_parts[0]));
}

bool Board_Part(uint8_t index, board_part_t *info)
{
  if ((index >= Board_PartCount()) || (info == NULL))
  {
    return false;
  }

  const PartDesc *p = &s_parts[index];

  info->name  = p->name;
  info->what  = p->what;
  info->where = p->where;
  info->power = p->power;

  switch (p->probe)
  {
    case PART_PROBE_AFE:
      info->state = Board_AfeOn() ? BOARD_PART_READY : BOARD_PART_UNPOWERED;
      break;

    case PART_PROBE_IMU:
      if (!Board_AfeOn())
      {
        info->state = BOARD_PART_UNPOWERED;
      }
      else
      {
        info->state = Board_ImuReady() ? BOARD_PART_READY : BOARD_PART_SILENT;
      }
      break;

    default:
      /* Nothing here can prove it either way, and inventing an answer is
         what invariant 10 forbids. */
      info->state = BOARD_PART_UNKNOWN;
      break;
  }

  return true;
}

uint8_t Board_DigitalCount(void)
{
  return (uint8_t)(sizeof(s_digital) / sizeof(s_digital[0]));
}

bool Board_DigitalChan(uint8_t index, board_dchan_t *info)
{
  if ((index >= Board_DigitalCount()) || (info == NULL))
  {
    return false;
  }

  const DigitalDesc *d = &s_digital[index];

  info->pin    = d->pin;
  info->dir    = d->dir;
  info->signal = d->signal;
  info->usable = d->usable;

  return true;
}

bool Board_PinUsable(char port, uint8_t pin)
{
  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if ((s_digital[i].port == port) && (s_digital[i].number == pin))
    {
      return s_digital[i].usable;
    }
  }

  /* Not in the table at all: nothing on this board claims it, so a fixture
     may have it. */
  return true;
}

bool Board_AfeOn(void)
{
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
}

void Board_SetAfeOn(bool on)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Board_Pe15(void)
{
  return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET);
}

void Board_RequestConsoleMode(void)
{
  link_request_close();
}
