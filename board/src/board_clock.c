/**
  ******************************************************************************
  * @file    board_clock.c
  * @brief   Identity, clock tree queries, and the cycle-counter timebase.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"




/* ---- board seam for the comms stack ------------------------------------ */
/* Declared in comms/inc/board.h. */

const char *Board_Name(void)
{
  return "coaxial_63100 STM32H753";
}

uint32_t Board_SysClkHz(void)
{
  return HAL_RCC_GetSysClockFreq();
}

uint32_t Board_AdcClockHz(void)
{
  /* The kernel first - whichever source the RCC was told to use - then the
     common prescaler in ADC12's CCR, which ADC3 shares the encoding of. */
  static const uint16_t divider[16] = { 1U, 2U, 4U, 6U, 8U, 10U, 12U,
                                        16U, 32U, 64U, 128U, 256U,
                                        256U, 256U, 256U, 256U };
  const uint32_t kernel = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
  const uint32_t presc = (ADC12_COMMON->CCR & ADC_CCR_PRESC)
                         >> ADC_CCR_PRESC_Pos;

  return kernel / divider[presc & 0xFU];
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

/* PLL1 fed from HSE is still the crystal. */
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

/* Enables the Cortex-M7 cycle counter. */
void Board_TimebaseInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
