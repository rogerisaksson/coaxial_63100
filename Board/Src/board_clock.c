/**
  ******************************************************************************
  * @file    board_clock.c
  * @brief   Identity, clock tree queries, and the cycle-counter timebase.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"




/* ---- board seam for the comms stack ------------------------------------ */
/* Declared in Comms/Inc/board.h. Thin wrappers so the ADC helpers above stay
   static; the comms stack asks, this answers, and the dependency runs one way. */

const char *Board_Name(void)
{
  return "coaxial_63100 STM32H753";
}

uint32_t Board_SysClkHz(void)
{
  return HAL_RCC_GetSysClockFreq();
}

uint32_t Board_HclkHz(void)
{
  return HAL_RCC_GetHCLKFreq();
}

uint8_t Board_SysClkSource(void)
{
  const uint32_t src = __HAL_RCC_GET_SYSCLK_SOURCE();

  if (src == RCC_SYSCLKSOURCE_STATUS_HSI)    { return 0U; }
  if (src == RCC_SYSCLKSOURCE_STATUS_CSI)    { return 1U; }
  if (src == RCC_SYSCLKSOURCE_STATUS_HSE)    { return 2U; }
  if (src == RCC_SYSCLKSOURCE_STATUS_PLLCLK) { return 3U; }
  return 4U;
}

uint32_t Board_Cycles(void)
{
  return DWT->CYCCNT;
}

/* PLL1 fed from HSE is still the crystal. Kept here rather than in the console
   so the same judgement is available to any caller, including a test rig. */
bool Board_SysClkOnCrystal(void)
{
  const uint32_t src = __HAL_RCC_GET_SYSCLK_SOURCE();

  if (src == RCC_SYSCLKSOURCE_STATUS_HSE)
  {
    return true;
  }

  if (src != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
    return false;
  }

  return (RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC) == RCC_PLLCKSELR_PLLSRC_HSE;
}

/* Enables the Cortex-M7 cycle counter. Not a measurement: the comms stack uses
   CYCCNT raw as its timebase, because it wraps at exactly 2^32 and unsigned
   elapsed-time arithmetic stays correct across the wrap. */
void Board_TimebaseInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
