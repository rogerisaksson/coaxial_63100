/**
  ******************************************************************************
  * @file    board_irq.h
  * @brief   PRIMASK held across a register pair, and given back only if it
  *          was not already held - a caller under its own critical section
  *          keeps it. One definition; it was the same four lines in five
  *          modules.
  ******************************************************************************
  */
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
