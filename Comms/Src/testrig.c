/**
  ******************************************************************************
  * @file    testrig.c
  * @brief   Raw pin access. See testrig.h for what is refused and why.
  ******************************************************************************
  */
#include "testrig.h"

#include "main.h"

static bool s_open;

/* Pins that are never available, whatever the gate says. Kept as data so the
   list is auditable at a glance rather than spread through the checks. */
typedef struct
{
  char    port;
  uint8_t pin;
  const char *why;
} testrig_reserved_t;

static const testrig_reserved_t RESERVED[] =
{
  { 'B', 10U, "USART3_TX" },
  { 'B', 11U, "USART3_RX" },
  { 'A', 13U, "JTMS/SWDIO" },
  { 'A', 14U, "JTCK/SWCLK" },
  { 'A', 15U, "JTDI" },
  { 'B',  3U, "JTDO/TRACESWO" },
  { 'B',  4U, "NJTRST" },
};

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

  for (size_t i = 0U; i < (sizeof(RESERVED) / sizeof(RESERVED[0])); i++)
  {
    if ((RESERVED[i].port == port) && (RESERVED[i].pin == pin))
    {
      return false;
    }
  }

  return true;
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

  for (size_t i = 0U; i < (sizeof(RESERVED) / sizeof(RESERVED[0])); i++)
  {
    if (RESERVED[i].port == port)
    {
      safe &= (uint16_t)~(1U << RESERVED[i].pin);
    }
  }

  /* BSRR is atomic: set bits in the low half, reset in the high half, no
     read-modify-write and so no window where another writer can interfere. */
  const uint16_t set = (uint16_t)(safe & value);
  const uint16_t clr = (uint16_t)(safe & (uint16_t)~value);

  g->BSRR = (uint32_t)set | ((uint32_t)clr << 16);
  return true;
}
