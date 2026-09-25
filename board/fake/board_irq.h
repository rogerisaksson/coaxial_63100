/** board_irq.h - PRIMASK on the host: nothing interrupts the fake board. */
#ifndef BOARD_IRQ_H
#define BOARD_IRQ_H

#include <stdint.h>

static inline uint32_t Board_IrqHold(void)
{
  return 0U;
}

static inline void Board_IrqRelease(uint32_t masked)
{
  (void)masked;
}

#endif /* BOARD_IRQ_H */
