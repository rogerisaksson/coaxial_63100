/**
  ******************************************************************************
  * @file    board_drive.c
  * @brief   Runs the control law on this hardware, one PWM period at a time.
  *
  * `drive/` is the arithmetic and knows no hardware. This converts the
  * injected triple and the link to amperes and volts through the
  * calibration record (invariant 7: one conversion, defined once), steps
  * the law, and hands the duties to board_pwm.c to commit at the next
  * underflow. Its parameters come out of the record too, so a board runs
  * the same drive after a reset that it ran before one.
  *
  * Called from ADC3's injected interrupt, so short: no HAL, no talking.
  * The only judgement here is the drive's own - a current past the trip it
  * was given drops the stage - and that is the thermal ceiling's exception
  * again (invariant 10).
  ******************************************************************************
  */
#include "board.h"
#include "board_drive.h"
#include "board_hw.h"

#include <math.h>

static drive_t  s_drive;
static bool     s_ready;
/** What the thermal envelope is scaling the current clamp by, 1 to 0.
    Held here rather than read from the observer in the interrupt: the
    poll runs at 10 Hz on the main loop and this is read at 50 kHz. */
static float    s_derate = 1.0f;
/** The clamp the record asked for, before any derating. */
static float    s_i_max_cal = 0.0f;
static bool     s_owned;          /**< the drive holds the compares       */
static uint32_t s_cycles_last;    /**< what one step cost, raw CYCCNT     */
static uint32_t s_cycles_max;
/** Where TIM1 stood when the step ended, in ticks past the trigger: the
    conversion, the HAL's interrupt entry and the step, all of it. The
    cycle count above starts inside the step and misses the first two. */
static uint16_t s_exit_ticks_max;

void Board_DriveDerate(float factor)
{
  /* Clamped here rather than trusted: this is a multiplier on a current
     limit, and a value outside 0..1 would be a limit RAISED by the
     thermal envelope, which is the one thing it must never do. */
  if (!(factor >= 0.0f))
  {
    factor = 0.0f;
  }
  if (factor > 1.0f)
  {
    factor = 1.0f;
  }
  s_derate = factor;
  if (s_ready)
  {
    s_drive.p.i_max = s_i_max_cal * s_derate;
  }
}


float Board_DriveDerating(void)
{
  return s_derate;
}


/** The conversions, affine and cached: refreshed with the parameters.
    A call into board_adc.c per sample was most of the interrupt. */
static int32_t s_i_off[BOARD_PWM_PHASES];
static float   s_i_k[BOARD_PWM_PHASES];
static int32_t s_v_off;
static float   s_v_k;

#define TWO_PI_F 6.2831853f


static float milli(uint32_t v)
{
  return (float)v / 1000.0f;
}


static float micro(uint32_t v)
{
  return (float)v / 1000000.0f;
}


static float milli_signed(uint32_t v)
{
  return (float)(int32_t)v / 1000.0f;
}


void Board_DriveParamsFromCal(void)
{
  const board_cal_t *cal = Board_Cal();
  drive_params_t *p = &s_drive.p;

  p->r = micro(cal->motor_r_uohm);
  p->ld = (float)cal->motor_ld_nh * 1e-9f;
  p->lq = (float)cal->motor_lq_nh * 1e-9f;
  p->lambda = micro(cal->motor_lambda_uvs);
  p->pole_pairs = (float)cal->motor_pole_pairs;
  p->kp = milli(cal->drv_kp_mv_per_a);
  p->ki = (float)cal->drv_ki_v_per_as;
  p->l1 = milli(cal->drv_l1_milli);
  p->l2 = milli(cal->drv_l2_milli);
  p->inj_volts = milli(cal->drv_inj_mv);
  p->inj_periods = (uint16_t)cal->drv_inj_periods;
  p->inj_phase = milli_signed(cal->drv_inj_phase_mrad);
  p->eps_gain = (float)(int32_t)cal->drv_eps_gain_ua_per_rad / 1000000.0f;
  s_i_max_cal = milli(cal->drv_i_max_ma);
  p->i_max = s_i_max_cal * s_derate;
  p->i_trip = milli(cal->drv_i_trip_ma);
  p->v_frac = micro(cal->drv_v_frac_ppm);
  p->sign = ((int32_t)cal->drv_sign < 0) ? -1.0f : 1.0f;
  p->w_lo = milli(cal->drv_w_lo_mrad_s);
  p->w_hi = milli(cal->drv_w_hi_mrad_s);
  p->dt_step = milli(cal->drv_dt_step_ma);
  for (uint8_t k = 0U; k < DRIVE_DT_POINTS; k++)
  {
    p->dt_volts[k] = milli(cal->drv_dt_mv[k]);
  }

  for (uint8_t k = 0U; k < BOARD_PWM_PHASES; k++)
  {
    Board_PhaseScale(k, &s_i_off[k], &s_i_k[k]);
  }
  Board_DcBusScale(&s_v_off, &s_v_k);

  /* The sample point the commissioning chose, if it chose one. Zero is
     "never measured" and leaves the sync's own default alone. */
  if (cal->drv_trigger_ticks != 0U)
  {
    (void)Board_SyncSetTrigger((uint16_t)cal->drv_trigger_ticks);
  }
}


void Board_DriveInit(void)
{
  /* Centre-aligned, so a period is twice ARR ticks of the timer clock,
     which is half SYSCLK. 20.000 us at ARR 2375 and 475 MHz. */
  const uint32_t ticks = 2U * (Board_PwmPeriod() - 1U);
  const uint32_t hz = Board_SysClkHz() / 2U;
  const float ts = ((ticks != 0U) && (hz != 0U))
                   ? ((float)ticks / (float)hz) : 20e-6f;

  drive_init(&s_drive, ts);
  s_drive.cycles = Board_Cycles;
  Board_DriveParamsFromCal();
  s_owned = false;
  s_cycles_max = 0U;
  s_ready = true;
}


const drive_t *Board_Drive(void)
{
  return &s_drive;
}


float Board_DriveTs(void)
{
  return s_drive.ts;
}


const char *Board_DriveSetMode(uint8_t mode)
{
  if (!s_ready)
  {
    return "the drive has not been initialised - the board has not "
           "finished starting";
  }
  if ((mode != (uint8_t)DRIVE_OFF) && !Board_SyncArmed())
  {
    const char *why = Board_SyncArm();

    if (why != NULL)
    {
      return why;             /* no triple, no loop: the sync's own words */
    }
  }

  /* The record may have been edited since the last mode change; a drive
     running yesterday's gains after today's calibration is the stale copy
     invariant 7 exists to prevent. */
  Board_DriveParamsFromCal();

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  const char *why = drive_set_mode(&s_drive, (drive_mode_t)mode,
                                   Board_PwmIsEnabled(), Board_AfeOn());
  if (!masked)
  {
    __enable_irq();
  }
  return why;
}


const char *Board_DriveSetpoint(uint8_t id, int32_t value)
{
  drive_setpoints_t *sp = &s_drive.sp;
  const float f = (float)value;

  switch (id)
  {
    case 0U: sp->id_ref = f / 1000.0f; break;
    case 1U: sp->iq_ref = f / 1000.0f; break;
    case 2U: sp->theta = f / 1000.0f; break;
    case 3U: sp->omega_target = f / 1000.0f; break;
    case 4U: sp->accel = f / 1000.0f; break;
    case 5U: sp->vd = f / 1000.0f; break;
    case 6U: sp->vq = f / 1000.0f; break;
    case 7U: sp->pol_volts = f / 1000.0f; break;
    case 8U:
      if ((value <= 0) || (value > 65535))
      {
        return "pol_periods is 1..65535 PWM periods";
      }
      sp->pol_periods = (uint16_t)value;
      break;
    case 9U:
      if ((value < 0) || (value > 65535))
      {
        return "pol_gap is 0..65535 PWM periods";
      }
      sp->pol_gap = (uint16_t)value;
      break;
    default:
      return "no such setpoint - 0 id_ref mA, 1 iq_ref mA, 2 theta mrad, "
             "3 omega_target mrad/s, 4 accel mrad/s2, 5 vd mV, 6 vq mV, "
             "7 pol_volts mV, 8 pol_periods, 9 pol_gap";
  }
  return NULL;
}


void Board_DriveSetpointsGet(int32_t *out)
{
  const drive_setpoints_t *sp = &s_drive.sp;

  out[0] = (int32_t)(sp->id_ref * 1000.0f);
  out[1] = (int32_t)(sp->iq_ref * 1000.0f);
  out[2] = (int32_t)(sp->theta * 1000.0f);
  out[3] = (int32_t)(sp->omega_target * 1000.0f);
  out[4] = (int32_t)(sp->accel * 1000.0f);
  out[5] = (int32_t)(sp->vd * 1000.0f);
  out[6] = (int32_t)(sp->vq * 1000.0f);
  out[7] = (int32_t)(sp->pol_volts * 1000.0f);
  out[8] = (int32_t)sp->pol_periods;
  out[9] = (int32_t)sp->pol_gap;
}


const char *Board_DriveSetSource(uint8_t source)
{
  if (source > 1U)
  {
    return "source is 0 for the converters or 1 for the model";
  }
  if (s_drive.mode != DRIVE_OFF)
  {
    return "the drive is running - mode 0 first, then change where its "
           "samples come from";
  }

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  s_drive.source = (source != 0U) ? DRIVE_SOURCE_MODEL : DRIVE_SOURCE_ADC;
  drive_model_init(&s_drive.model);
  if (!masked)
  {
    __enable_irq();
  }
  return NULL;
}


const char *Board_DriveModelParam(uint8_t id, int32_t value)
{
  drive_model_params_t *p = &s_drive.model.p;
  const float f = (float)value;

  switch (id)
  {
    case 0U:  p->r = f / 1e6f; break;
    case 1U:  if (value <= 0) { return "ld is nanohenry, above zero"; }
              p->ld = f * 1e-9f; break;
    case 2U:  if (value <= 0) { return "lq is nanohenry, above zero"; }
              p->lq = f * 1e-9f; break;
    case 3U:  p->lambda = f / 1e6f; break;
    case 4U:  if (value <= 0) { return "pole pairs is a count above zero"; }
              p->pole_pairs = f; break;
    case 5U:  p->sat = f / 1e6f; break;
    case 6U:  if (value <= 0) { return "i_sat is milliamperes, above zero"; }
              p->i_sat = f / 1e3f; break;
    case 7U:  if (value <= 0) { return "J is nano kg m2, above zero"; }
              p->j = f * 1e-9f; break;
    case 8U:  p->b = f * 1e-9f; break;
    case 9U:  p->load = f / 1e6f; break;
    case 10U: p->v_dt = f / 1e3f; break;
    case 11U: if (value <= 0) { return "i_knee is milliamperes, above zero"; }
              p->i_knee = f / 1e3f; break;
    case 12U: if (value <= 0) { return "vdc is millivolts, above zero"; }
              p->vdc = f / 1e3f; break;
    case 13U: p->noise = f / 1e6f; break;
    case 14U: p->theta0 = f / 1e6f; break;
    case 15U: if ((value < 1) || (value > 16)) { return "substeps is 1..16"; }
              p->sub = (uint8_t)value; break;
    default:
      return "no such model parameter - 0 r uohm, 1 ld nH, 2 lq nH, 3 lambda "
             "uVs, 4 pole pairs, 5 sat ppm, 6 i_sat mA, 7 J nkgm2, 8 B nNms, "
             "9 load uNm, 10 v_dt mV, 11 i_knee mA, 12 vdc mV, 13 noise uA, "
             "14 theta0 urad, 15 substeps";
  }
  return NULL;
}


void Board_DriveModelReset(void)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  drive_model_init(&s_drive.model);
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DriveSetTheta(int32_t microradians)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  drive_set_theta(&s_drive, (float)microradians / 1000000.0f);
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DriveWindowTake(drive_window_t *out)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  drive_window_take(&s_drive, out);
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DriveMomentsArm(uint32_t periods)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  drive_moments_arm(&s_drive, periods);
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DriveMoments(drive_moments_t *out)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  *out = s_drive.mom;
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DriveCycles(uint32_t *last, uint32_t *max)
{
  *last = s_cycles_last;
  *max = s_cycles_max;
}


void Board_DriveCyclesReset(void)
{
  s_cycles_max = 0U;
  s_exit_ticks_max = 0U;
}


uint16_t Board_DriveExitTicks(void)
{
  return s_exit_ticks_max;
}


bool Board_DriveOwnsCompares(void)
{
  return s_owned;
}


/** What the law's duties do to the compares this period: nothing while
  * the stage is down, the next triple while a mode runs on an armed stage,
  * and one zero triple when a mode has just ended - polarity finishing, a
  * stage drop, the host asking for OFF - before the compares are let go. */
static void commit_duties(const drive_out_t *out, bool enabled, bool running)
{
  if (enabled && (s_drive.mode != DRIVE_OFF))
  {
    uint16_t ticks[BOARD_PWM_PHASES];
    const float arr = (float)(Board_PwmPeriod() - 1U);

    for (uint8_t k = 0U; k < BOARD_PWM_PHASES; k++)
    {
      ticks[k] = (uint16_t)lrintf(out->duty[k] * arr);
    }
    if (!s_owned)
    {
      Board_PwmDriveOwn(true);
      s_owned = true;
    }
    Board_PwmSetNext(ticks);
    return;
  }
  if (!(running || s_owned) || !s_owned)
  {
    return;
  }

  const uint16_t zeros[BOARD_PWM_PHASES] = { 0U, 0U, 0U };

  Board_PwmSetNext(zeros);
  Board_PwmDriveOwn(false);
  s_owned = false;
}

void Board_DriveOnSample(const int16_t *phase, uint32_t dcbus_raw)
{
  /* In ADC3's interrupt, straight after the triple was latched. Nothing
     here talks (invariant 5). */
  if (!s_ready)
  {
    return;
  }

  const uint32_t t0 = Board_Cycles();
  int32_t codes[DRIVE_MOMENT_CHANNELS];
  drive_sample_t in;
  drive_out_t out;

  for (uint8_t k = 0U; k < BOARD_PWM_PHASES; k++)
  {
    codes[k] = (int32_t)phase[k];
    in.i[k] = (float)(codes[k] - s_i_off[k]) * s_i_k[k];
  }
  codes[3] = (int32_t)dcbus_raw;
  in.vdc = (float)(codes[3] - s_v_off) * s_v_k;
  drive_moments_feed(&s_drive, codes);

  const bool enabled = Board_PwmIsEnabled();
  const bool running = (s_drive.mode != DRIVE_OFF);
  /* The model as the source: the law runs on its currents whether or
     not a stage is armed, and the duties below reach the gates only if
     one is - which is how the rotor observer is watched on this bench, where
     the converters and the drivers are never powered together. */
  const bool trip = (s_drive.source == DRIVE_SOURCE_MODEL)
                    ? drive_step_virtual(&s_drive, &out)
                    : drive_step(&s_drive, &in, enabled, &out);

  if (trip)
  {
    /* The trip: MOE down in hardware before this returns. */
    Board_PwmDisable();
    s_owned = false;
  }
  else
  {
    commit_duties(&out, enabled, running);
  }

  /* The ring, when armed for this source: dq current, the rotor observer's
     angle as a turn in 65536, the innovation in 0.1 mrad. Converted only
     then - four lrintf calls a period for a ring nobody armed were in the
     8 us this interrupt cost. */
  if ((Board_LogSources() & (1U << BOARD_LOG_SOURCE_DRIVE)) != 0U)
  {
    const int16_t logged[4] = {
      (int16_t)lrintf(s_drive.id * 100.0f),
      (int16_t)lrintf(s_drive.iq * 100.0f),
      (int16_t)(uint16_t)lrintf(s_drive.theta_hat / TWO_PI_F * 65536.0f),
      (int16_t)lrintf(s_drive.eps * 10000.0f),
    };

    Board_LogPush(BOARD_LOG_SOURCE_DRIVE, logged, 4U);
  }

  s_cycles_last = Board_Cycles() - t0;
  s_cycles_max = (s_cycles_last > s_cycles_max) ? s_cycles_last : s_cycles_max;

  /* The trigger fires on the down-slope at CCR5; the counter has fallen
     since, or turned at zero and climbed. Either way the ticks since. */
  const uint32_t cnt = TIM1->CNT;
  const uint32_t trigger = TIM1->CCR5;
  const uint32_t since = ((TIM1->CR1 & TIM_CR1_DIR) != 0U)
                         ? (trigger - cnt) : (trigger + cnt);

  if (since > s_exit_ticks_max)
  {
    s_exit_ticks_max = (uint16_t)since;
  }
}
