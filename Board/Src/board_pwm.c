/**
  ******************************************************************************
  * @file    board_pwm.c
  * @brief   The three-phase gate drivers: duty in, gates out, and the interlocks.
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

/** What was asked for, in ticks Q16.16, and the fraction not yet spent.
    One tick of ARR 2375 is 0.0421 % of duty, so 34.54 % lands between 820
    and 821 and neither is it. Rounding throws the difference away; this
    keeps it and pays it back. */
static uint32_t s_want_q16[BOARD_PWM_PHASES];
static uint32_t s_residue[BOARD_PWM_PHASES];
static bool     s_dither;

static uint8_t s_skew;                 /* DTG counts, one way then the other */
static bool    s_skew_up;              /* true: the up-count edge gets more  */
static uint8_t s_deadtime;             /* what was asked for, in DTG counts  */
static volatile uint8_t s_half;        /* which half of the period this is   */

/* The update interrupt, which the dither and the dead-time skew both need.
   Two owners and one switch: whoever stops has to ask whether the other is
   still using it, or the first one to finish turns it off under the second.
   Measured before that was true: the skew was set, nothing armed the
   dither, and DTG read the same value six times over SWD. */
static void update_irq(bool wanted)
{
  if (wanted)
  {
    TIM1->DIER |= TIM_DIER_UIE;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  }
  else
  {
    TIM1->DIER &= ~TIM_DIER_UIE;
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  }
}


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

     It does NOT come from the gate_drivers. A 2EDL8034 in PG-DSO-8 has
     eight pins and no fault output; PE15 carries FAULTIN from the STO
     chain. Active low, so BDTR.BKP is TIM_BREAKPOLARITY_LOW and AOE stays
     off - nothing re-arms itself.

     With no pilot tone on RS485 the STO chain holds this asserted and the
     the gate drivers cannot start. That is the interlock, not a fault to clear. */
  return Board_PwmReady() && ((TIM1->SR & TIM_SR_BIF) != 0U);
}


void Board_PwmDisable(void)
{
  /* The one operation that must work whatever else is true. Clearing MOE
     drops every output to its idle level in hardware, without waiting for
     an update event. */
  s_armed = false;
  s_dither = false;

  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
  {
    TIM1->DIER &= ~TIM_DIER_UIE;
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
     happened to hold is how a stage gets a step it was never asked for. */
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


void Board_PwmDitherStep(void)
{
  /* First-order sigma-delta on the compare register, once per PWM period.
     Each period spends the whole ticks and carries the fraction; when the
     carry passes one, that period gets a tick more. The mean is then the
     asked-for duty exactly rather than the nearest tick.

     First order, so it has idle tones - the pattern is periodic and its
     lines sit in the band below the switching frequency. That is the price
     of three adds in an interrupt at 50 kHz, and it is written down here
     rather than discovered later.

     Short on purpose: this runs in TIM1's update interrupt. */
  if (!s_dither)
  {
    return;
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    uint32_t whole = s_want_q16[phase] >> 16;

    s_residue[phase] += (s_want_q16[phase] & 0xFFFFU);
    if (s_residue[phase] >= 0x10000U)
    {
      s_residue[phase] -= 0x10000U;
      whole++;
    }
    if (whole > TIM1->ARR)
    {
      whole = TIM1->ARR;
    }
    s_duty[phase] = (uint16_t)whole;
  }

  TIM1->CCR1 = s_duty[0];
  TIM1->CCR2 = s_duty[1];
  TIM1->CCR3 = s_duty[2];
}


const char *Board_PwmSetAllFine(const uint32_t *ticks_q16)
{
  /* Ticks in Q16.16 rather than a percentage: the board does no division
     and the caller keeps whatever precision it had. */
  if (ticks_q16 == NULL)
  {
    return "no duties given - pass three";
  }
  if (!Board_PwmIsEnabled())
  {
    return "the gate drivers are not enabled - enable it first, and clear or "
           "bypass the break if one is latched";
  }

  const uint32_t limit = (uint32_t)TIM1->ARR << 16;

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if (ticks_q16[phase] > limit)
    {
      /* All three or none: a half update runs one cycle with two phases
         from this call and one from the last. */
      return "a duty is past ARR - the largest is period minus one, which "
             "the state reports";
    }
  }

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_want_q16[phase] = ticks_q16[phase];
    s_residue[phase] = 0U;
  }
  s_dither = true;
  if (!masked)
  {
    __enable_irq();
  }

  /* Turned on with the first fractional duty and off again with the next
     whole one. The cost is small - 4 us of the keepalive's worst gap,
     measured - but it is not zero and nothing dithering needs it. */
  TIM1->SR = ~TIM_SR_UIF;
  update_irq(true);
  return NULL;
}


void Board_PwmDutyRequested(uint32_t *ticks_q16)
{
  if (ticks_q16 == NULL)
  {
    return;
  }
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    ticks_q16[phase] = s_want_q16[phase];
  }
}


const char *Board_PwmSetAll(const uint16_t *ticks)
{
  if (ticks == NULL)
  {
    return "no duties given - pass three";
  }
  if (!Board_PwmIsEnabled())
  {
    return "the gate drivers are not enabled - enable it first, and clear or "
           "bypass the break if one is latched";
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if (ticks[phase] > TIM1->ARR)
    {
      /* All three or none: a half update runs one cycle with two phases
         from this call and one from the last. */
      return "a duty is past ARR - the largest is period minus one, which "
             "the state reports";
    }
  }

  /* Whole ticks, so the dither has nothing to carry and stops moving the
     register out from under this. The two paths cannot both own CCR - but
     the interrupt has a second owner now, so it goes only when the skew
     does not want it either. */
  s_dither = false;
  update_irq(s_skew != 0U);
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_want_q16[phase] = (uint32_t)ticks[phase] << 16;
    s_residue[phase] = 0U;
  }

  /* One update event applies all three, so the gate drivers never run a cycle
     with two phases from this call and one from the last. */
  TIM1->CCR1 = ticks[0];
  TIM1->CCR2 = ticks[1];
  TIM1->CCR3 = ticks[2];

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s_duty[phase] = ticks[phase];
  }
  return NULL;
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

  /* The six outputs as they stand this instant, and where the counter was
     when they were read. One IDR load, so the six are the same instant -
     six separate reads at 50 kHz would straddle an edge and show a leg with
     both FETs on, which is the one thing that cannot happen. TIM1->CNT is
     read second and is a few cycles later; at 237.5 MHz that is under a
     tick of the 4.21 ns dead time and the caller is told it is separate. */
  const uint32_t idr = GPIOE->IDR;

  out->pins = (uint8_t)(((idr >> 8) & 0x3FU));   /* PE8..PE13, in order */
  out->at = (uint16_t)TIM1->CNT;
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

  /* The update interrupt the dither runs on is NOT enabled here: an
     interrupt that does nothing should not run at 50 kHz. Measured, it
     costs less than it looks - worst keepalive gap 190.4 us with it on
     against 186.5 off, and the edge rate 71.4 kHz against 75.1. Both are
     inside the noise of what else the loop is doing. */
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);

  /* The six gate signals, at a speed CubeMX does not set.
   *
   * MX_TIM1_Init's MSP configures PE8..PE13 with GPIO_SPEED_FREQ_LOW -
   * its default when the .ioc names no speed - and measured over SWD,
   * GPIOE OSPEEDR read 0x00000300: every one of the six at 00. A slow
   * edge into the 2EDL8034's TTL input holds the input stage in its
   * linear region while it crosses, and the driver dissipates the
   * difference. Reported from the bench: two of the three drivers ran
   * much hotter than the FETs they drive.
   *
   * The .ioc carries the speed now as well, so a regeneration keeps it.
   * This is here because the MSP runs inside HAL_TIM_Init and undoes
   * anything set before it, and because a driver heating for want of one
   * register field should not depend on remembering to regenerate.
   */
  {
    GPIO_InitTypeDef gate = {0};

    gate.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
             | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    gate.Mode = GPIO_MODE_AF_PP;
    gate.Pull = GPIO_NOPULL;
    gate.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gate.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gate);
  }

  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE
              | TIM_CCER_CC2E | TIM_CCER_CC2NE
              | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  /* RCR 0, so the update lands at every overflow AND every underflow -
     twice a PWM period. The .ioc asks for 1, which is once, and once is
     not enough to give the two transitions different dead times: DTG has
     no preload, so the only place to change it is between them. The
     handler counts halves and runs the dither on every second one, which
     is the rate it always ran at. */
  TIM1->RCR = 0U;
  s_deadtime = (uint8_t)(TIM1->BDTR & TIM_BDTR_DTG);
  s_half = 0U;

  TIM1->CR1 |= TIM_CR1_CEN;

  /* Measured on target: BIF is latched by the time this runs. PE15 is AF
     open-drain with no pull and floats while MX_TIM1_Init enables BKE, so
     the break trips on our own start-up. Clearing it here makes
     Board_PwmFault() mean the pin; a pin really low latches it straight
     back. */
  TIM1->SR &= ~TIM_SR_BIF;
  return true;
}


/* Dead time, at runtime.
 *
 * The floor is 20 ns and it is a floor, not a default: the 2EDL8034 has no
 * interlock of its own, so this number is the only thing between the two
 * FETs of a leg, and a caller that asks for less gets 20 ns and is told.
 *
 * DTG is not linear. Only the low range - DTG[7:5] == 0 - steps by one
 * t_DTS; above it the encoding stretches. This uses the low range alone,
 * which caps it at 127 x t_DTS, 535 ns at 237.5 MHz. That is six times what
 * this bridge needs and the arithmetic stays one multiply.
 */
#define BOARD_PWM_DEADTIME_MIN_NS 20U
#define BOARD_PWM_DTG_MAX 127U


static uint32_t dts_ps(void)
{
  /* Picoseconds per DTG count, so the ns arithmetic stays integer. CKD is
     0 - checked in the silicon, CR1 0xB1 - so t_DTS is one timer tick. */
  const uint32_t hz = Board_SysClkHz() / 2UL;   /* TIM1 kernel, 237.5 MHz */

  return (hz != 0UL) ? (1000000000000ULL / hz) : 0UL;
}

uint8_t Board_PwmDeadTimeFloor(void)
{
  const uint32_t ps = dts_ps();

  if (ps == 0UL)
  {
    return 1U;
  }

  /* Round up: a floor that rounded down would be under the floor. */
  const uint32_t counts = ((BOARD_PWM_DEADTIME_MIN_NS * 1000UL) + ps - 1UL) / ps;

  return (counts < 1UL) ? 1U : (uint8_t)counts;
}

uint32_t Board_PwmDeadTimeNs(void)
{
  /* What was asked for, not what BDTR holds this half-period. With a skew
     running the register alternates, and reading it gave 105 ns for an
     80 ns request - whichever half the read happened to land in. */
  return (uint32_t)(((uint64_t)s_deadtime * dts_ps()) / 1000ULL);
}

const char *Board_PwmSetDeadTime(uint32_t ns)
{
  if (!Board_PwmReady())
  {
    return "TIM1 is not running, so there is no dead time to set - the "
           "board has not finished starting";
  }

  const uint32_t ps = dts_ps();

  if (ps == 0UL)
  {
    return "the timer clock reads zero, so a dead time in nanoseconds "
           "cannot be worked out";
  }

  uint32_t counts = ((uint64_t)ns * 1000ULL) / ps;
  const uint8_t floor_counts = Board_PwmDeadTimeFloor();

  if (counts < floor_counts)
  {
    counts = floor_counts;
  }

  if ((counts + s_skew) > BOARD_PWM_DTG_MAX)
  {
    return "that dead time plus its skew is past DTG's linear range - ask "
           "for 535 ns or less, which is already six times what this "
           "bridge needs";
  }

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  s_deadtime = (uint8_t)counts;
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | counts;
  if (!masked)
  {
    __enable_irq();
  }

  return NULL;
}


/* The skew, which is why the update runs twice a period.
 *
 * TIM1's dead time generator puts the same DTG on both transitions, so a
 * skew cannot come from moving a compare register - that shifts the whole
 * transition and leaves the gap alone. What it can come from is writing DTG
 * itself between the two: with RCR 0 the update event lands at every
 * overflow AND every underflow, so the handler gets a turn ahead of each
 * transition and gives it its own number.
 *
 * The sign is which way round: positive lengthens the dead time on the
 * transition the counter reaches going up and shortens the other by the
 * same, so the pair still averages what Board_PwmSetDeadTime was asked for.
 * Neither half is allowed under the floor.
 *
 * NOT MEASURED. What it does to the gates needs two probes and a scope; all
 * that is checked here is that DTG reads back what the handler last wrote
 * (invariant 10).
 */
const char *Board_PwmSetDeadTimeSkew(int8_t counts)
{
  const uint8_t floor_counts = Board_PwmDeadTimeFloor();
  const int32_t low = (int32_t)s_deadtime - (counts < 0 ? -counts : counts);
  const int32_t high = (int32_t)s_deadtime + (counts < 0 ? -counts : counts);

  if (low < (int32_t)floor_counts)
  {
    return "that skew would take one of the two dead times under the 20 ns "
           "floor - raise the dead time first, or skew it less";
  }
  if (high > (int32_t)BOARD_PWM_DTG_MAX)
  {
    return "that skew would take one of the two dead times past DTG's "
           "linear range - lower the dead time first, or skew it less";
  }

  s_skew = (uint8_t)(counts < 0 ? -counts : counts);
  s_skew_up = (counts >= 0);
  update_irq((s_skew != 0U) || s_dither);

  if (s_skew == 0U)
  {
    /* Back to the one number, or the last half-period's value would stay. */
    const uint32_t masked = __get_PRIMASK();
    __disable_irq();
    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | s_deadtime;
    if (!masked)
    {
      __enable_irq();
    }
  }
  return NULL;
}

int8_t Board_PwmDeadTimeSkew(void)
{
  return s_skew_up ? (int8_t)s_skew : (int8_t)-(int8_t)s_skew;
}


/** TIM1's update, once per PWM period with RepetitionCounter at 1.
  *
  * Overridden here rather than in Core/: main.c holds CubeMX functions and
  * the poll calls, and a compare register belongs beside the code that owns
  * it. Priority 2 - below ADC3's 1, which is the current loop's, and above
  * everything else.
  */
void TIM1_UP_IRQHandler(void)
{
  if ((TIM1->SR & TIM_SR_UIF) == 0U)
  {
    return;
  }

  TIM1->SR = ~TIM_SR_UIF;

  /* Two updates a period with RCR 0 - one at overflow, one at underflow -
     so the dither, which is per period, runs on every second one. Counted
     here rather than read off CR1's DIR: the flag says which way the
     counter is going *now*, and by the time this reads it the direction has
     already turned. */
  s_half ^= 1U;

  if (s_half == 0U)
  {
    Board_PwmDitherStep();
  }

  /* The next transition's dead time. Written now, ahead of it, because DTG
     has no preload - it takes effect the moment it lands. With no skew this
     writes the same number twice a period and costs two stores. */
  if (s_skew != 0U)
  {
    const bool more = (s_half != 0U) == s_skew_up;
    const uint32_t dtg = more ? (uint32_t)(s_deadtime + s_skew)
                              : (uint32_t)(s_deadtime - s_skew);

    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | dtg;
  }
}
