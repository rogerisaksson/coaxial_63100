/** board_hw.h - The HAL the board layer's portable files take, on the host: the tick, SYSCLK. */
#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>

/* Milliseconds, off the fake clock (fake_uart.c). */
uint32_t HAL_GetTick(void);

/* SYSCLK in Hz, as CMSIS names it (fake_board.c). */
extern uint32_t SystemCoreClock;

#endif /* BOARD_HW_H */
