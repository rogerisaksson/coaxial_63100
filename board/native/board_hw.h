/** board_hw.h - The HAL the board layer takes, natively (native.c). */

/* The device (stm32h7xx.h here), the ADC handles, the tick and SYSCLK. */
#ifndef BOARD_HW_H
#define BOARD_HW_H

#include "stm32h7xx.h"
#include "board.h"
#include "board_units.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* Milliseconds, off the board's clock (native.c). */
uint32_t HAL_GetTick(void);

/* SYSCLK in Hz, as CMSIS names it (board/fake/fake_board.c). */
extern uint32_t SystemCoreClock;

#endif /* BOARD_HW_H */
