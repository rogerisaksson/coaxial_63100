/**
  ******************************************************************************
  * @file    board_pwm.c
  * @brief   The three-phase bridge: duty in, gates out, and the interlocks.
  *
  * TIM1 CH1/CH1N, CH2/CH2N and CH3/CH3N drive the three 2EDL8034 half bridges
  * on PE8..PE13; PE15 is TIM1_BKIN. This file owns the compare registers and
  * the master output enable. It does NOT configure the timer - see
  * Board_PwmReady().
  *
  * Written against the CMSIS registers rather than the htim1 handle: the
  * write that matters - clearing MOE - is then one store that cannot fail
  * partway, and drops every output to its idle level without the timer.
  *
  * Nothing here judges a duty. The board is a dumb slave (invariant 10): it
  * takes compare ticks and reports what it took. What it will NOT do is
  * accept anything at all until the timer exists and has been armed on
  * purpose, because on the other side of these six pins are gate drivers
  * with FETs fitted.
  ******************************************************************************
  */
#include "board.h"
#include "stm32h7xx.h"

/** Compare value per phase, mirrored so a read does not race the timer. */
static uint16_t s_duty[BOARD_PWM_PHASES];

/** Set by Board_PwmEnable, cleared by Board_PwmDisable and by a break. */
static bool s_armed;


bool Board_PwmReady(void)
{
  /* Clocked, and counting over a period somebody chose. An unclocked TIM1
     reads back zeros, which is exactly what an unconfigured one looks like -
     so this answers "has MX_TIM1_Init happened yet" without guessing. */
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0U)
  {
    return false;
  }
  return (TIM1->ARR != 0U);
}


uint32_t Board_PwmPeriod(void)
{
  return Board_PwmReady() ? (TIM1->ARR + 1U) : 0U;
}


bool Board_PwmFault(void)
{
  /* The break flag latches. It is the gate drivers' nFAULT arriving through
     TIM1_BKIN, which is a hardware path: the outputs are already off by the
     time any of this runs.

     It does NOT come from the gate drivers. A 2EDL8034 in PG-DSO-8 has
     eight pins and no fault output; PE15 carries FAULTIN from the STO
     chain. Active low, so BDTR.BKP is TIM_BREAKPOLARITY_LOW and AOE stays
     off - nothing re-arms itself.

     With no pilot tone on RS485 the STO chain holds this asserted and the
     bridge cannot start. That is the interlock, not a fault to clear. */
  return Board_PwmReady() && ((TIM1->SR & TIM_SR_BIF) != 0U);
}


void Board_PwmDisable(void)
{
  /* The one operation that must work whatever else is true. Clearing MOE
     drops every output to its idle level in hardware, without waiting for
     an update event. */
  s_armed = false;

  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
  {
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_duty[phase] = 0U;
  }
}


bool Board_PwmSetBreakBypass(bool on)
{
  /* Clearing the LATCH is not enough and never was: with BKE set and PE15
     low the break is a level, so the hardware holds MOE clear and software
     cannot set it at all. The bypass has to disconnect the input.

     What makes this safe is not this file. The STO chain gates the gate
     drivers' own DC/DC, which no MCU pin reaches - with no pilot tone on
     RS485 the drivers have no supply, so the six outputs toggle into
     unpowered inputs and the FETs cannot switch. This removes the MCU's
     interlock, not the board's. A reset restores it: MX_TIM1_Init sets BKE
     and nothing here persists. */
  if (!Board_PwmReady())
  {
    return false;
  }

  if (on)
  {
    TIM1->BDTR &= ~TIM_BDTR_BKE;
    TIM1->SR &= ~TIM_SR_BIF;
  }
  else
  {
    TIM1->BDTR |= TIM_BDTR_BKE;
  }
  return true;
}


bool Board_PwmBreakBypassed(void)
{
  return Board_PwmReady() && ((TIM1->BDTR & TIM_BDTR_BKE) == 0U);
}


bool Board_PwmClearFault(void)
{
  if (!Board_PwmReady())
  {
    return false;
  }

  /* Clearing the latch does not re-enable anything: the caller has to arm
     again, deliberately, after it has decided the fault is gone. A driver
     still pulling nFAULT low will simply latch it again. */
  TIM1->SR &= ~TIM_SR_BIF;
  return true;
}


bool Board_PwmEnable(void)
{
  if (!Board_PwmReady())
  {
    return false;
  }
  if (((TIM1->SR & TIM_SR_BIF) != 0U) && !Board_PwmBreakBypassed())
  {
    return false;               /* a latched break outranks any request */
  }

  /* Arm at zero, always. Enabling into whatever the compare registers
     happened to hold is how a bridge gets a step it was never asked for. */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_duty[phase] = 0U;
  }

  TIM1->BDTR |= TIM_BDTR_MOE;
  s_armed = true;
  return true;
}


bool Board_PwmIsEnabled(void)
{
  return s_armed && Board_PwmReady() && ((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
}


bool Board_PwmSetDuty(uint8_t phase, uint16_t ticks)
{
  if (phase >= BOARD_PWM_PHASES || !Board_PwmIsEnabled())
  {
    return false;
  }
  if (ticks > TIM1->ARR)
  {
    return false;               /* not a limit - it is off the end of ARR */
  }

  switch (phase)
  {
    case 0U:  TIM1->CCR1 = ticks; break;
    case 1U:  TIM1->CCR2 = ticks; break;
    default:  TIM1->CCR3 = ticks; break;
  }

  s_duty[phase] = ticks;
  return true;
}


bool Board_PwmSetAll(const uint16_t *ticks)
{
  if (ticks == NULL || !Board_PwmIsEnabled())
  {
    return false;
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if (ticks[phase] > TIM1->ARR)
    {
      return false;             /* all three or none: no half update */
    }
  }

  /* One update event applies all three, so the bridge never runs a cycle
     with two phases from this call and one from the last. */
  TIM1->CCR1 = ticks[0];
  TIM1->CCR2 = ticks[1];
  TIM1->CCR3 = ticks[2];

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_duty[phase] = ticks[phase];
  }
  return true;
}


uint16_t Board_PwmGetDuty(uint8_t phase)
{
  return (phase < BOARD_PWM_PHASES) ? s_duty[phase] : 0U;
}


void Board_PwmState(board_pwm_state_t *out)
{
  if (out == NULL)
  {
    return;
  }

  out->ready = Board_PwmReady();
  out->enabled = Board_PwmIsEnabled();
  out->fault = Board_PwmFault();
  out->period = Board_PwmPeriod();
  out->deadtime = out->ready ? (uint8_t)(TIM1->BDTR & TIM_BDTR_DTG) : 0U;
  out->bypassed = Board_PwmBreakBypassed();

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    out->duty[phase] = s_duty[phase];
  }
}


bool Board_PwmInit(void)
{
  /* The lazy shape the rest of Board/ uses - `if (!Ready() && !Init())`.
     Leaves the counter running with MOE clear. OSSI forces the idle level
     only where CCxE or CCxNE is set, so enabling the six outputs here is
     what holds the gates down in hardware rather than in nobody's hands. */
  Board_PwmDisable();

  if (!Board_PwmReady())
  {
    return false;
  }

  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE
              | TIM_CCER_CC2E | TIM_CCER_CC2NE
              | TIM_CCER_CC3E | TIM_CCER_CC3NE;
  TIM1->CR1 |= TIM_CR1_CEN;

  /* Measured on target: BIF is latched by the time this runs. PE15 is AF
     open-drain with no pull and floats while MX_TIM1_Init enables BKE, so
     the break trips on our own start-up. Clearing it here makes
     Board_PwmFault() mean the pin; a pin really low latches it straight
     back. */
  TIM1->SR &= ~TIM_SR_BIF;
  return true;
}
