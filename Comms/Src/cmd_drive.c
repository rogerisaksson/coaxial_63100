/**
  ******************************************************************************
  * @file    cmd_drive.c
  * @brief   The control law's operations behind command 0x6E, device 10.
  *
  * Integers in the unit that makes them integers: microradians for angles,
  * milliradians a second for speeds, milliamperes, millivolts, microamperes
  * and microvolts for the window's means and deviations. The drive holds
  * floats and the wire never sees one.
  *
  * Reports and refusals only. Which mode to enter, which gains to run,
  * where the sample point sits - all of it arrives from the host; the one
  * thing the board decides is to drop the stage past a current it was
  * given (invariant 10, the same exception the thermal ceiling holds).
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "board_drive.h"
#include "wire.h"

#include <math.h>


static int32_t micro_of(float x)
{
  return (int32_t)lrintf(x * 1000000.0f);
}


static int32_t milli_of(float x)
{
  return (int32_t)lrintf(x * 1000.0f);
}


/** op 0 - what the drive is doing, from one interrupt's worth of state. */
static cmd_status_t h_drive_state(wr_t *out)
{
  const drive_t *d = Board_Drive();
  uint32_t last, max;

  Board_DriveCycles(&last, &max);

  wr_u8(out, (uint8_t)d->mode);
  wr_u8(out, (uint8_t)d->fault);
  wr_u8(out, (uint8_t)((Board_PwmIsEnabled() ? 0x01U : 0U)
                     | (Board_AfeOn() ? 0x02U : 0U)
                     | (d->inj_valid ? 0x04U : 0U)
                     | (Board_DriveOwnsCompares() ? 0x08U : 0U)
                     | (Board_SyncArmed() ? 0x10U : 0U)));
  wr_i32(out, micro_of(d->theta_hat));
  wr_i32(out, milli_of(d->omega_hat));
  wr_i32(out, micro_of(d->theta_cmd));
  wr_i32(out, milli_of(d->omega_cmd));
  wr_i32(out, milli_of(d->id));
  wr_i32(out, milli_of(d->iq));
  wr_i32(out, milli_of(d->vd));
  wr_i32(out, milli_of(d->vq));
  wr_i32(out, milli_of(d->vdc));
  wr_i32(out, micro_of(d->eps));
  wr_i32(out, micro_of(d->eps_amps));
  wr_i32(out, micro_of(d->ih));
  wr_i32(out, micro_of(d->e_bemf));
  wr_u32(out, d->periods);
  wr_u32(out, last);
  wr_u32(out, max);
  wr_i32(out, milli_of(d->pol_pos));
  wr_i32(out, milli_of(d->pol_neg));
  wr_u16(out, Board_SyncTrigger());
  wr_u32(out, (uint32_t)lrintf(Board_DriveTs() * 1e9f));
  wr_u16(out, Board_DriveExitTicks());
  /* The virtual step block by block, raw cycles: the model's sample,
     the law, the model's advance. Zero on the converters. */
  wr_u32(out, d->cyc_sample);
  wr_u32(out, d->cyc_step);
  wr_u32(out, d->cyc_advance);
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 1 - enter a mode. The refusals are the drive's own words. */
static cmd_status_t h_drive_mode(rd_t *in, wr_t *out)
{
  const uint8_t mode = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  cmd_took(out, Board_DriveSetMode(mode));
  return CMD_OK;
}


/** op 2 - one setpoint, by id, in its integer unit. */
static cmd_status_t h_drive_setpoint(rd_t *in, wr_t *out)
{
  const uint8_t id = rd_u8(in);
  const int32_t value = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  cmd_took(out, Board_DriveSetpoint(id, value));
  return CMD_OK;
}


/** op 3 - every setpoint as the drive holds it. */
static cmd_status_t h_drive_setpoints(wr_t *out)
{
  int32_t v[10];

  Board_DriveSetpointsGet(v);
  wr_u8(out, 10U);
  for (uint8_t i = 0U; i < 10U; i++)
  {
    wr_i32(out, v[i]);
  }
  return CMD_OK;
}


/** op 4 - put both frames at an angle: the polarity flip, or a start. */
static cmd_status_t h_drive_theta(rd_t *in, wr_t *out)
{
  const int32_t urad = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  Board_DriveSetTheta(urad);
  cmd_took(out, NULL);
  return CMD_OK;
}


static void wr_field(wr_t *out, const drive_acc_t *a, float scale)
{
  /* Mean and standard deviation in the field's integer unit. The deviation
     rather than the variance because a variance in microamperes squared
     overflows anything the wire carries; the host squares it back. */
  wr_u32(out, a->n);
  if (a->n == 0U)
  {
    wr_i32(out, 0);
    wr_u32(out, 0U);
    return;
  }
  const double mean = a->sum / (double)a->n;
  double var = a->sumsq / (double)a->n - mean * mean;

  var = (var < 0.0) ? 0.0 : var;
  wr_i32(out, (int32_t)lrint(mean * (double)scale));
  wr_u32(out, (uint32_t)lrint(sqrt(var) * (double)scale));
}


/** op 5 - the window since the last take: means, deviations, and the
  * innovation's autocorrelation for the whiteness test. Resets. */
static cmd_status_t h_drive_window(wr_t *out)
{
  drive_window_t w;

  Board_DriveWindowTake(&w);

  wr_u32(out, w.n);
  wr_field(out, &w.acc[DRIVE_ACC_ID], 1e6f);     /* uA   */
  wr_field(out, &w.acc[DRIVE_ACC_IQ], 1e6f);
  wr_field(out, &w.acc[DRIVE_ACC_VD], 1e6f);     /* uV   */
  wr_field(out, &w.acc[DRIVE_ACC_VQ], 1e6f);
  wr_field(out, &w.acc[DRIVE_ACC_EPS], 1e6f);    /* urad */
  wr_field(out, &w.acc[DRIVE_ACC_IH], 1e6f);     /* uA   */
  wr_field(out, &w.acc[DRIVE_ACC_VDC], 1e3f);    /* mV   */

  wr_u8(out, (uint8_t)DRIVE_LAGS);
  for (uint8_t j = 1U; j <= DRIVE_LAGS; j++)
  {
    const double rho = (w.lag[0] > 0.0) ? (w.lag[j] / w.lag[0]) : 0.0;

    wr_i32(out, (int32_t)lrint(rho * 1e6));
  }
  wr_i32(out, milli_of(w.i_peak));
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 6 - count raw codes at the sample point for this many periods. */
static cmd_status_t h_drive_moments_arm(rd_t *in, wr_t *out)
{
  const uint32_t periods = rd_u32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (!Board_SyncArmed())
  {
    cmd_took(out, "the sync is not armed, so no triple arrives to count - "
                  "arm it (gate drivers op 3) and ask again");
    return CMD_OK;
  }
  Board_DriveMomentsArm(periods);
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 7 - the moments so far: mean, deviation, lowest, highest per
  * channel, in codes. Done when n reached what was asked. */
static cmd_status_t h_drive_moments(wr_t *out)
{
  drive_moments_t m;

  Board_DriveMoments(&m);

  wr_u8(out, ((m.want != 0U) && (m.n >= m.want)) ? 1U : 0U);
  wr_u32(out, m.n);
  wr_u32(out, m.want);
  wr_u16(out, Board_SyncTrigger());
  for (uint8_t k = 0U; k < DRIVE_MOMENT_CHANNELS; k++)
  {
    if (m.n == 0U)
    {
      wr_i32(out, 0);
      wr_u32(out, 0U);
      wr_i32(out, 0);
      wr_i32(out, 0);
      continue;
    }
    const double mean = (double)m.sum[k] / (double)m.n;
    double var = (double)m.sumsq[k] / (double)m.n - mean * mean;

    var = (var < 0.0) ? 0.0 : var;
    wr_i32(out, (int32_t)lrint(mean * 1000.0));           /* milli-codes */
    wr_u32(out, (uint32_t)lrint(sqrt(var) * 1000.0));
    wr_i32(out, m.lo[k]);
    wr_i32(out, m.hi[k]);
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 8 - take the parameters out of the record again. */
static cmd_status_t h_drive_reload(wr_t *out)
{
  Board_DriveParamsFromCal();
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 10 - where the samples come from. */
static cmd_status_t h_drive_source(rd_t *in, wr_t *out)
{
  const uint8_t source = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  cmd_took(out, Board_DriveSetSource(source));
  return CMD_OK;
}


/** op 11 - one model parameter, by id, in its integer unit. */
static cmd_status_t h_drive_model_param(rd_t *in, wr_t *out)
{
  const uint8_t id = rd_u8(in);
  const int32_t value = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  cmd_took(out, Board_DriveModelParam(id, value));
  return CMD_OK;
}


/** op 12 - the model's truth: the rotor the observer is judged by. */
static cmd_status_t h_drive_model(wr_t *out)
{
  const drive_t *d = Board_Drive();

  wr_u8(out, (uint8_t)d->source);
  wr_i32(out, micro_of(d->model.theta));
  wr_i32(out, milli_of(d->model.omega));
  wr_i32(out, milli_of(d->model.id));
  wr_i32(out, milli_of(d->model.iq));
  wr_i32(out, milli_of(d->model.p.vdc));
  /* The estimate in the same reply as the truth. Two requests are
     15 ms apart, and at 440 rad/s that is six radians of rotor. */
  wr_i32(out, micro_of(d->theta_hat));
  wr_i32(out, milli_of(d->omega_hat));
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 13 - the rotor back to theta0, at rest. */
static cmd_status_t h_drive_model_reset(wr_t *out)
{
  Board_DriveModelReset();
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 9 - forget the worst step cost, so a run is measured on its own. */
static cmd_status_t h_drive_cycles_reset(wr_t *out)
{
  Board_DriveCyclesReset();
  wr_u8(out, 1U);
  return CMD_OK;
}


cmd_status_t cmd_drive_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case DRIVE_OP_STATE:       return h_drive_state(out);
    case DRIVE_OP_MODE:        return h_drive_mode(in, out);
    case DRIVE_OP_SETPOINT:    return h_drive_setpoint(in, out);
    case DRIVE_OP_SETPOINTS:   return h_drive_setpoints(out);
    case DRIVE_OP_THETA:       return h_drive_theta(in, out);
    case DRIVE_OP_WINDOW:      return h_drive_window(out);
    case DRIVE_OP_MOMENTS_ARM: return h_drive_moments_arm(in, out);
    case DRIVE_OP_MOMENTS:     return h_drive_moments(out);
    case DRIVE_OP_RELOAD:      return h_drive_reload(out);
    case DRIVE_OP_CYCLES_RESET: return h_drive_cycles_reset(out);
    case DRIVE_OP_SOURCE:      return h_drive_source(in, out);
    case DRIVE_OP_MODEL_PARAM: return h_drive_model_param(in, out);
    case DRIVE_OP_MODEL:       return h_drive_model(out);
    case DRIVE_OP_MODEL_RESET: return h_drive_model_reset(out);
    default:                   return CMD_ERR_VALUE;
  }
}
