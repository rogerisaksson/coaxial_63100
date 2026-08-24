/**
  ******************************************************************************
  * @file    testrig.c
  * @brief   Raw pin access. See testrig.h for what is refused and why.
  ******************************************************************************
  */
#include "testrig.h"

#include "board.h"
#include "main.h"

static bool s_open;

/* Which pins are refused is the board's answer, not this file's: the list
   used to live here as well as in the pin table the channels command
   reports, and two lists of what PB10 is are one edit away from
   disagreeing. See Board_PinUsable. */

static GPIO_TypeDef *port_base(char port)
{
  switch (port)
  {
    case 'A': return GPIOA;
    case 'B': return GPIOB;
    case 'C': return GPIOC;
    case 'D': return GPIOD;
    case 'E': return GPIOE;
    case 'F': return GPIOF;
    case 'G': return GPIOG;
    case 'H': return GPIOH;
    case 'I': return GPIOI;
    case 'J': return GPIOJ;
    case 'K': return GPIOK;
    default:  return NULL;
  }
}

bool testrig_open(void)
{
  return s_open;
}

bool testrig_gate(uint32_t key, bool open)
{
  if (key != TESTRIG_KEY)
  {
    return false;
  }

  s_open = open;
  return true;
}

bool testrig_pin_allowed(char port, uint8_t pin)
{
  if ((port_base(port) == NULL) || (pin > 15U))
  {
    return false;
  }

  return Board_PinUsable(port, pin);
}

bool testrig_pin_mode(char port, uint8_t pin, uint8_t mode, uint8_t pull)
{
  if (!s_open || !testrig_pin_allowed(port, pin))
  {
    return false;
  }

  if ((mode > TESTRIG_MODE_ANALOG) || (pull > TESTRIG_PULL_DOWN))
  {
    return false;
  }

  static const uint32_t MODE[4] =
  {
    GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_OD, GPIO_MODE_ANALOG
  };
  static const uint32_t PULL[3] = { GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN };

  GPIO_InitTypeDef init = { 0 };

  init.Pin   = (uint32_t)(1UL << pin);
  init.Mode  = MODE[mode];
  init.Pull  = PULL[pull];
  init.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(port_base(port), &init);
  return true;
}

bool testrig_pin_read(char port, uint8_t pin, bool *level)
{
  /* Reads need no gate: sensing a pin cannot damage anything, and a fixture
     wants this far more often than it wants to drive. */
  if (!testrig_pin_allowed(port, pin))
  {
    return false;
  }

  *level = (HAL_GPIO_ReadPin(port_base(port), (uint16_t)(1U << pin)) == GPIO_PIN_SET);
  return true;
}

bool testrig_pin_write(char port, uint8_t pin, bool level)
{
  if (!s_open || !testrig_pin_allowed(port, pin))
  {
    return false;
  }

  HAL_GPIO_WritePin(port_base(port), (uint16_t)(1U << pin),
                    level ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return true;
}

bool testrig_port_read(char port, uint16_t *value)
{
  GPIO_TypeDef *g = port_base(port);

  if (g == NULL)
  {
    return false;
  }

  *value = (uint16_t)(g->IDR & 0xFFFFU);
  return true;
}

bool testrig_port_write(char port, uint16_t mask, uint16_t value)
{
  GPIO_TypeDef *g = port_base(port);

  if (!s_open || (g == NULL))
  {
    return false;
  }

  /* Mask off the reserved pins rather than refusing the whole write: a fixture
     driving a bank of outputs should not have to know which bits this board
     keeps for itself. */
  uint16_t safe = mask;

  for (uint8_t pin = 0U; pin < 16U; pin++)
  {
    if (!Board_PinUsable(port, pin))
    {
      safe &= (uint16_t)~(1U << pin);
    }
  }

  /* BSRR is atomic: set bits in the low half, reset in the high half, no
     read-modify-write and so no window where another writer can interfere. */
  const uint16_t set = (uint16_t)(safe & value);
  const uint16_t clr = (uint16_t)(safe & (uint16_t)~value);

  g->BSRR = (uint32_t)set | ((uint32_t)clr << 16);
  return true;
}
