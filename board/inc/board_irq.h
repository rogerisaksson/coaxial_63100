/** board_irq.h - PRIMASK over a register pair, restored only if it was clear. */
#ifndef BOARD_IRQ_H
#define BOARD_IRQ_H

#include <stdint.h>

#include "stm32h7xx.h"

static inline uint32_t Board_IrqHold(void)
{
  const uint32_t masked = __get_PRIMASK();

  __disable_irq();
  return masked;
}

static inline void Board_IrqRelease(uint32_t masked)
{
  if (masked == 0U)
  {
    __enable_irq();
  }
}

#endif /* BOARD_IRQ_H */
