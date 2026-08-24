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
  { 'E', 15U, "PE15", BOARD_DIR_IN,    "AFE_ON sense",        true  },
  { 'B', 10U, "PB10", BOARD_DIR_OUT,   "USART3_TX",           false },
  { 'B', 11U, "PB11", BOARD_DIR_IN,    "USART3_RX",           false },
  { 'A', 13U, "PA13", BOARD_DIR_INOUT, "JTMS/SWDIO",          false },
  { 'A', 14U, "PA14", BOARD_DIR_IN,    "JTCK/SWCLK",          false },
  { 'A', 15U, "PA15", BOARD_DIR_IN,    "JTDI",                false },
  { 'B',  3U, "PB3",  BOARD_DIR_OUT,   "JTDO/TRACESWO",       false },
  { 'B',  4U, "PB4",  BOARD_DIR_IN,    "NJTRST",              false },
};

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
