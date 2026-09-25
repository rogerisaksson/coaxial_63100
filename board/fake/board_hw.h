/** board_hw.h - The HAL the board layer's portable files take, on the host: the tick. */
#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>

/* Milliseconds, off the fake clock (fake_uart.c). */
uint32_t HAL_GetTick(void);

#endif /* BOARD_HW_H */
