/** board_pwm.c - TIM1 gate drivers: duty in, gates out, interlocks. */
#include "board_limits.h"
#include "board.h"
#include "board_irq.h"
#include "stm32h7xx.h"

#define PS_PER_S  1000000000000ULL
#define PS_PER_NS 1000UL
/* The gate-short probe's settle after driving a pin: the neighbour follows a
   real short within 76 ns, measured; this is a few microseconds. */
#define PROBE_SETTLE_SPINS 4000U

/** The stage's state: the compares as mirrored, the fine duty and its
    residue, the alternating triples, the dead-time skew, the counted hold,
    the arm, and the drive's next triple. */
static struct
{
  /** Compare value per phase, mirrored so a read does not race the timer. */
  uint16_t duty[BOARD_PWM_PHASES];

  /** What was asked for, in ticks Q16.16, and the fraction not yet spent. */
  uint32_t want_q16[BOARD_PWM_PHASES];
  uint32_t residue[BOARD_PWM_PHASES];
  bool dither;

  /** Two triples the update interrupt swaps between, one per PWM period, and
      which of them the next period gets. */
  uint16_t alt[2][BOARD_PWM_PHASES];
  volatile bool alternate;
  volatile uint8_t alt_next;

  uint8_t skew;                   /* DTG counts, one way then the other */
  bool skew_up;                   /* true: the up-count edge gets more */
  uint8_t deadtime;               /* what was asked for, in DTG counts */
  volatile uint8_t half;          /* which half of the period this is */

  /* Periods left of a counted hold, 0 when free-running. */
  volatile uint32_t countdown;

  /** Set by Board_PwmEnable, cleared by Board_PwmDisable and by a break. */
  bool armed;

  /** The drive's next triple, left by ADC3's interrupt and committed by
      TIM1's update at the underflow. */
  uint16_t next[BOARD_PWM_PHASES];
  volatile bool next_pending;
  volatile bool drive_owns;
} s;

/* The update interrupt, which the dither and the dead-time skew both need. */
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

bool Board_PwmReady(void)
{
  /* Clocked, and counting over a period somebody chose. */
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
  /* The break flag latches. */
  return Board_PwmReady() && ((TIM1->SR & TIM_SR_BIF) != 0U);
}

void Board_PwmSessionDrop(void)
{
  (void)Board_PwmSetBreakBypass(false);
  Board_PwmDisable();
}

void Board_PwmDisable(void)
{
  /* The one operation that must work whatever else is true. */
  s.armed = false;
  s.dither = false;
  s.alternate = false;
  s.countdown = 0U;
  s.drive_owns = false;
  s.next_pending = false;

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
    s.duty[phase] = 0U;
  }
}

bool Board_PwmSetBreakBypass(bool on)
{
  /* Clearing the latch is not enough: with BKE set and PE15 low the break is
     a level, so the hardware holds MOE clear and software cannot set it. */
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
     again, deliberately, after it has decided the fault is gone. */
  TIM1->SR &= ~TIM_SR_BIF;
  return true;
}

/** Drive one leg's low-side input low, then high, and read the neighbour:
    only a path well below its pull-down lifts it. */
static bool leg_follows(uint32_t drv, uint32_t obs)
{
  GPIOE->BSRR = 1UL << (drv + GPIO_BSRR_BR0_Pos);
  for (volatile uint32_t d = 0U; d < PROBE_SETTLE_SPINS; d++) { }
  if (((GPIOE->IDR >> obs) & 1UL) != 0U)
  {
    return false;              /* high with the driver low: no path */
  }

  GPIOE->BSRR = 1UL << drv;
  for (volatile uint32_t d = 0U; d < PROBE_SETTLE_SPINS; d++) { }
  return ((GPIOE->IDR >> obs) & 1UL) != 0U;
}

uint8_t Board_PwmGateShorts(void)
{
  /* Each leg's two gate pins, low side first. */
  static const uint8_t leg[BOARD_PWM_PHASES][2] =
    { { 8U, 9U }, { 10U, 11U }, { 12U, 13U } };

  uint8_t shorts = 0U;

  if (!Board_PwmReady() || Board_PwmIsEnabled())
  {
    return 0U;
  }

  for (uint8_t k = 0U; k < BOARD_PWM_PHASES; k++)
  {
    const uint32_t drv   = leg[k][0];
    const uint32_t obs   = leg[k][1];
    const uint32_t moder = GPIOE->MODER;
    const uint32_t pupdr = GPIOE->PUPDR;
    const uint32_t both  = (3UL << (2U * drv)) | (3UL << (2U * obs));

    GPIOE->PUPDR = (pupdr & ~both) | (2UL << (2U * obs));
    GPIOE->MODER = (moder & ~both) | (1UL << (2U * drv));

    if (leg_follows(drv, obs))
    {
      shorts |= (uint8_t)(1U << k);
    }

    GPIOE->BSRR   = 1UL << (drv + GPIO_BSRR_BR0_Pos);
    GPIOE->MODER  = moder;
    GPIOE->PUPDR  = pupdr;
  }

  return shorts;
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

  /* Arm at zero, always. */
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.duty[phase] = 0U;
  }

  TIM1->BDTR |= TIM_BDTR_MOE;
  s.armed = true;
  return true;
}

bool Board_PwmIsEnabled(void)
{
  return s.armed && Board_PwmReady() && ((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
}

void Board_PwmDitherStep(void)
{
  /* First-order sigma-delta on the compare register, once per PWM period. */
  if (!s.dither)
  {
    return;
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    uint32_t whole = s.want_q16[phase] >> 16;

    s.residue[phase] += (s.want_q16[phase] & 0xFFFFU);
    if (s.residue[phase] >= 0x10000U)
    {
      s.residue[phase] -= 0x10000U;
      whole++;
    }
    if (whole > TIM1->ARR)
    {
      whole = TIM1->ARR;
    }
    s.duty[phase] = (uint16_t)whole;
  }

  TIM1->CCR1 = s.duty[0];
  TIM1->CCR2 = s.duty[1];
  TIM1->CCR3 = s.duty[2];
}

/* Why no duty may be written now, or NULL: the stage off (a latched break
   included), or the drive holding the compares. */
static const char *compares_refused(void)
{
  if (!Board_PwmIsEnabled())
  {
    return "the gate drivers are not enabled - enable it first, and clear or "
           "bypass the break if one is latched";
  }
  if (s.drive_owns)
  {
    return "the drive holds the compares - set drive mode 0 (off) first, "
           "and it lets go";
  }
  return NULL;
}

const char *Board_PwmSetAllFine(const uint32_t *ticks_q16)
{
  /* Ticks in Q16.16 rather than a percentage: the board does no division and
     the caller keeps whatever precision it had. */
  if (ticks_q16 == NULL)
  {
    return "no duties given - pass three";
  }
  const char *refused = compares_refused();

  if (refused != NULL)
  {
    return refused;
  }

  const uint32_t limit = (uint32_t)TIM1->ARR << 16;

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if (ticks_q16[phase] > limit)
    {
      /* All three or none: a half update runs one cycle with two phases from
         this call and one from the last. */
      return "a duty is past ARR - the largest is period minus one, which "
             "the state reports";
    }
  }

  const uint32_t masked = Board_IrqHold();
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.want_q16[phase] = ticks_q16[phase];
    s.residue[phase] = 0U;
  }
  s.dither = true;
  s.alternate = false;
  s.countdown = 0U;
  Board_IrqRelease(masked);

  /* Turned on with the first fractional duty and off again with the next
     whole one. */
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
    ticks_q16[phase] = s.want_q16[phase];
  }
}

const char *Board_PwmSetAll(const uint16_t *ticks)
{
  if (ticks == NULL)
  {
    return "no duties given - pass three";
  }
  const char *refused = compares_refused();

  if (refused != NULL)
  {
    return refused;
  }

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if (ticks[phase] > TIM1->ARR)
    {
      /* All three or none: a half update runs one cycle with two phases from
         this call and one from the last. */
      return "a duty is past ARR - the largest is period minus one, which "
             "the state reports";
    }
  }

  /* Whole ticks, so the dither has nothing to carry and stops moving the
     register out from under this. */
  s.dither = false;
  s.alternate = false;
  s.countdown = 0U;
  update_irq(s.skew != 0U);
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.want_q16[phase] = (uint32_t)ticks[phase] << 16;
    s.residue[phase] = 0U;
  }

  /* One update event applies all three, so the gate drivers never run a
     cycle with two phases from this call and one from the last. */
  TIM1->CCR1 = ticks[0];
  TIM1->CCR2 = ticks[1];
  TIM1->CCR3 = ticks[2];

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.duty[phase] = ticks[phase];
  }
  return NULL;
}

const char *Board_PwmSetAllCounted(const uint16_t *ticks, uint32_t periods)
{
  /* The same triple, held for exactly `periods` PWM periods and then zeroed
     by the update interrupt. */
  const char *why = Board_PwmSetAll(ticks);

  if (why != NULL)
  {
    return why;
  }
  if (periods != 0U)
  {
    s.countdown = periods;
    update_irq(true);
  }
  return NULL;
}

uint32_t Board_PwmPeriodsLeft(void)
{
  return s.countdown;
}

const char *Board_PwmSetAlternate(const uint16_t *a, const uint16_t *b)
{
  if ((a == NULL) || (b == NULL))
  {
    return "no duties given - pass two triples";
  }
  const char *refused = compares_refused();

  if (refused != NULL)
  {
    return refused;
  }
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    if ((a[phase] > TIM1->ARR) || (b[phase] > TIM1->ARR))
    {
      return "a duty is past ARR - the largest is period minus one, which "
             "the state reports";
    }
  }

  /* The interrupt owns the compares from here: the dither is off, A is in
     the registers now and B is what the next overflow writes. */
  const uint32_t masked = Board_IrqHold();
  s.dither = false;
  s.countdown = 0U;
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.alt[0][phase] = a[phase];
    s.alt[1][phase] = b[phase];
    s.want_q16[phase] = (uint32_t)a[phase] << 16;
    s.residue[phase] = 0U;
    s.duty[phase] = a[phase];
  }
  TIM1->CCR1 = a[0];
  TIM1->CCR2 = a[1];
  TIM1->CCR3 = a[2];
  s.alt_next = 1U;
  s.alternate = true;
  Board_IrqRelease(masked);

  TIM1->SR = ~TIM_SR_UIF;
  update_irq(true);
  return NULL;
}

void Board_PwmDriveOwn(bool on)
{
  if (on)
  {
    s.dither = false;
    s.alternate = false;
    s.countdown = 0U;
    s.next_pending = false;
    s.drive_owns = true;
    TIM1->SR = ~TIM_SR_UIF;
    update_irq(true);
    return;
  }
  s.drive_owns = false;
  s.next_pending = false;
  update_irq((s.skew != 0U) || s.dither);
}

void Board_PwmSetNext(const uint16_t *ticks)
{
  /* From ADC3's interrupt, above TIM1_UP's, so these stores are never split
     by the reader - it copies under PRIMASK. */
  const uint32_t arr = TIM1->ARR;

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.next[phase] = (ticks[phase] > arr) ? (uint16_t)arr : ticks[phase];
  }
  s.next_pending = true;
}

uint16_t Board_PwmGetDuty(uint8_t phase)
{
  if (phase >= BOARD_PWM_PHASES)
  {
    return 0U;
  }
  if (s.alternate)
  {
    /* The mean over the pair of periods, which is the leg's load: the
       compare itself swaps at 50 kHz, and the thermal observer sampling it
       at its own rate sat phase-locked on one triple - the U driver was
       charged with the whole run while V ran the same pulses. */
    return (uint16_t)(((uint32_t)s.alt[0][phase] + s.alt[1][phase]) / 2U);
  }
  return s.duty[phase];
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
     when they were read. */
  const uint32_t idr = GPIOE->IDR;

  out->pins = (uint8_t)(((idr >> 8) & 0x3FU));   /* PE8..PE13, in order */
  out->at = (uint16_t)TIM1->CNT;
  out->bypassed = Board_PwmBreakBypassed();

  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    out->duty[phase] = s.duty[phase];
  }
}

static uint32_t dts_ps(void)
{
  /* Picoseconds per DTG count, so the ns arithmetic stays integer. */
  const uint32_t hz = Board_SysClkHz() / 2UL;   /* TIM1 kernel, 237.5 MHz */

  return (hz != 0UL) ? (PS_PER_S / hz) : 0UL;
}

uint8_t Board_PwmDeadTimeFloor(void)
{
  const uint32_t ps = dts_ps();

  if (ps == 0UL)
  {
    return 1U;
  }

  /* Round up: a floor that rounded down would be under the floor. */
  const uint32_t counts = ((BOARD_PWM_DEADTIME_MIN_NS * PS_PER_NS) + ps - 1UL) / ps;

  return (counts < 1UL) ? 1U : (uint8_t)counts;
}

uint32_t Board_PwmDeadTimeNs(void)
{
  /* What was asked for, not what BDTR holds this half-period. */
  return (uint32_t)(((uint64_t)s.deadtime * dts_ps()) / PS_PER_NS);
}

bool Board_PwmInit(void)
{
  /* The lazy shape the rest of board/ uses - `if (!Ready() && !Init())`. */
  Board_PwmDisable();

  if (!Board_PwmReady())
  {
    return false;
  }

  /* The dither's update interrupt is not enabled here: one that does
     nothing should not run at 50 kHz. */
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);

  /* The six gate signals, at a speed CubeMX does not set. */
  {
    GPIO_InitTypeDef gate = {0};

    gate.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
             | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    gate.Mode = GPIO_MODE_AF_PP;
    gate.Pull = GPIO_NOPULL;
    gate.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gate.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gate);

    /* BKIN is active low and CubeMX generates it AF_OD with no pull, so an
       unconnected fault line floats and the break fires on noise. */
    gate.Pin = GPIO_PIN_15;
    gate.Mode = GPIO_MODE_AF_OD;
    gate.Pull = GPIO_PULLUP;
    gate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gate);
  }

  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE
              | TIM_CCER_CC2E | TIM_CCER_CC2NE
              | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  /* RCR 0, so the update lands at every overflow and every underflow - twice
     a PWM period. */
  TIM1->RCR = 0U;

  /* Not the dead time. */
  s.deadtime = (uint8_t)(TIM1->BDTR & TIM_BDTR_DTG);
  s.half = 0U;

  TIM1->CR1 |= TIM_CR1_CEN;

  /* Measured on target: BIF is latched by the time this runs. */
  TIM1->SR &= ~TIM_SR_BIF;
  return true;
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

  /* Rounded up, like the floor above and for the same reason: a dead time
     that rounded down is under what was asked for, and the direction that is
     wrong is the one that shortens it. */
  uint32_t counts = (((uint64_t)ns * PS_PER_NS) + ps - 1ULL) / ps;
  const uint8_t floor_counts = Board_PwmDeadTimeFloor();

  if (counts < floor_counts)
  {
    counts = floor_counts;
  }

  if ((counts + s.skew) > BOARD_PWM_DTG_MAX)
  {
    return "that dead time plus its skew is past DTG's linear range - ask "
           "for 535 ns or less, which is already six times what this "
           "bridge needs";
  }

  const uint32_t masked = Board_IrqHold();
  s.deadtime = (uint8_t)counts;
  TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | counts;
  Board_IrqRelease(masked);

  return NULL;
}

/* The skew, and why the update runs twice a period. */
const char *Board_PwmSetDeadTimeSkew(int8_t counts)
{
  const uint8_t floor_counts = Board_PwmDeadTimeFloor();
  const int32_t low = (int32_t)s.deadtime - (counts < 0 ? -counts : counts);
  const int32_t high = (int32_t)s.deadtime + (counts < 0 ? -counts : counts);

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

  s.skew = (uint8_t)(counts < 0 ? -counts : counts);
  s.skew_up = (counts >= 0);
  update_irq((s.skew != 0U) || s.dither);

  if (s.skew == 0U)
  {
    /* Back to the one number, or the last half-period's value would stay. */
    const uint32_t masked = Board_IrqHold();
    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | s.deadtime;
    Board_IrqRelease(masked);
  }
  return NULL;
}

int8_t Board_PwmDeadTimeSkew(void)
{
  return s.skew_up ? (int8_t)s.skew : (int8_t)-(int8_t)s.skew;
}

/* The counted hold, one period a tick: whether this update is the one that
   runs it out. */
static bool hold_counted_down(void)
{
  if ((s.countdown == 0U) || (s.half != 0U))
  {
    return false;
  }
  s.countdown--;
  return s.countdown == 0U;
}

/* The hold ran out: every compare to zero, every duty forgotten, the
   alternate and the dither off - stood down before the mode branches so that
   on the expiring event neither writes a compare after the zero lands. */
static void hold_expired(void)
{
  s.alternate = false;
  s.dither = false;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  for (uint8_t phase = 0U; phase < BOARD_PWM_PHASES; phase++)
  {
    s.duty[phase] = 0U;
    s.want_q16[phase] = 0U;
    s.residue[phase] = 0U;
  }
  if (s.skew == 0U)
  {
    update_irq(false);
  }
}

/* The drive's next triple, just past the underflow - DIR reads up - so a
   triple written here lands, preloaded, at the next overflow and shapes one
   symmetric pulse centred on the underflow after it. */
static void land_next_triple(void)
{
  if (!s.next_pending || ((TIM1->CR1 & TIM_CR1_DIR) != 0U))
  {
    return;
  }
  __disable_irq();
  TIM1->CCR1 = s.next[0];
  TIM1->CCR2 = s.next[1];
  TIM1->CCR3 = s.next[2];
  s.duty[0] = s.next[0];
  s.duty[1] = s.next[1];
  s.duty[2] = s.next[2];
  s.next_pending = false;
  __enable_irq();
}

/** TIM1's update, once per PWM period with RepetitionCounter at 1. */
void TIM1_UP_IRQHandler(void)
{
  if ((TIM1->SR & TIM_SR_UIF) == 0U)
  {
    return;
  }

  TIM1->SR = ~TIM_SR_UIF;

  /* Two updates a period with RCR 0 - one at overflow, one at underflow - so
     the dither, which is per period, runs on every second one. */
  s.half ^= 1U;

  /* The counted hold. */
  if (hold_counted_down())
  {
    hold_expired();
  }

  if (s.alternate && ((TIM1->CR1 & TIM_CR1_DIR) != 0U))
  {
    /* Just past the overflow - DIR already reads down - so these preloaded
       compares land at the underflow and the whole next period, both slopes,
       is one triple. */
    const uint16_t *next = s.alt[s.alt_next];

    TIM1->CCR1 = next[0];
    TIM1->CCR2 = next[1];
    TIM1->CCR3 = next[2];
    s.duty[0] = next[0];
    s.duty[1] = next[1];
    s.duty[2] = next[2];
    s.alt_next ^= 1U;
  }
  else if (s.drive_owns)
  {
    land_next_triple();
  }
  else if (s.half == 0U)
  {
    Board_PwmDitherStep();
  }

  /* The next transition's dead time. */
  if (s.skew != 0U)
  {
    const bool more = (s.half != 0U) == s.skew_up;
    const uint32_t dtg = more ? (uint32_t)(s.deadtime + s.skew)
                              : (uint32_t)(s.deadtime - s.skew);

    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG) | dtg;
  }
}
