/** board_irq.h - PRIMASK natively: while it is set, native.c runs no interrupt. */
#ifndef BOARD_IRQ_H
#define BOARD_IRQ_H

#include <stdint.h>

extern uint32_t native_primask;

static inline uint32_t Board_IrqHold(void)
{
  const uint32_t was = native_primask;

  native_primask = 1U;
  return was;
}

static inline void Board_IrqRelease(uint32_t masked)
{
  native_primask = masked;
}

#endif /* BOARD_IRQ_H */
