/** board_ctrl.c - The board's loop: ctrl/'s runner, ticked from the drive's sample. */
#include "board.h"
#include "board_ctrl.h"
#include "board_drive.h"
#include "board_irq.h"
#include "ctrl.h"

#include <math.h>

#define ANGLE_BITS    0x0FFFU                /* the A1335's twelve */
#define DEG_PER_COUNT (360.0f / 4096.0f)
#define DIVIDER_MAX   65535.0f

static struct
{
  ctrl_runner_t r;
  bool     running;
  bool     wired;
  uint8_t  measured;
  uint8_t  command;
  uint16_t divider;                          /* PWM periods a tick */
  uint16_t count;
  float    dt;
  uint32_t blind;
} s;

/** The slots in pass order. */
static ctrl_part_t *slot_part(uint8_t slot)
{
  ctrl_part_t *const parts[BOARD_CTRL_SLOTS] = {
    &s.r.f.prefilter, &s.r.f.measure, &s.r.f.estimator, &s.r.f.regulator,
  };

  return parts[slot];
}

/** What the loop measures now; NAN without a reading. */
static float measure(const drive_t *d)
{
  board_angle_state_t a;

  switch (s.measured)
  {
    case BOARD_CTRL_OMEGA: return d->omega_hat;
    case BOARD_CTRL_IQ:    return d->iq;
    default:
      Board_AngleState(&a);
      return a.have ? (float)(a.value & ANGLE_BITS) * DEG_PER_COUNT : NAN;
  }
}

void Board_CtrlTick(drive_t *d)
{
  if (!s.running || (++s.count < s.divider))
  {
    return;
  }
  s.count = 0U;

  const float x = measure(d);

  if (isnan(x))
  {
    s.blind++;
    return;
  }

  const float u = ctrl_runner_step(&s.r, s.dt, x);

  if (s.command == BOARD_CTRL_IQ_REF)
  {
    d->sp.iq_ref = u;
  }
  else
  {
    d->sp.theta = u;
  }
}

void Board_CtrlState(board_ctrl_state_t *out)
{
  const uint32_t masked = Board_IrqHold();

  out->running = s.running;
  out->playing = s.r.playing;
  out->measured = s.measured;
  out->command = s.command;
  out->hz = (s.dt > 0.0f) ? (uint16_t)lrintf(1.0f / s.dt) : 0U;
  out->free = ctrl_rows_free(&s.r);
  out->rows = (uint16_t)(CTRL_ROWS - 1U - out->free);
  out->queued_s = ctrl_rows_seconds(&s.r);
  out->played = s.r.played;
  out->idle = s.r.idle;
  out->blind = s.blind;
  out->setpoint = s.r.setpoint;
  out->ref = s.r.f.ref;
  out->value = s.r.f.value;
  out->estimate = s.r.f.estimate;
  out->out = s.r.f.command;
  Board_IrqRelease(masked);
}

const char *Board_CtrlSlot(uint8_t slot, uint8_t kind, const float *params, uint8_t n)
{
  if (s.running)
  {
    return "the loop is running - op 5 off first";
  }
  if (slot >= BOARD_CTRL_SLOTS)
  {
    return "slot is 0 prefilter, 1 measure, 2 estimator, 3 regulator";
  }
  if (kind >= (uint8_t)CTRL_KINDS)
  {
    return "no such kind - 0 none, 1 Gain, 2 Slew, 3 Wrap, 4 LowPass, 5 SpeedKalman, "
           "6 PI, 7 AngleHold, 8 Direct, 9 SpeedPI";
  }
  if (n != ctrl_kind_params[kind])
  {
    return "that kind takes another number of parameters (machine.parts PARAMS)";
  }
  (void)ctrl_part_set(slot_part(slot), kind, params);
  return NULL;
}

const char *Board_CtrlWire(uint8_t measured, uint8_t command, uint16_t hz)
{
  const float ts = Board_DriveTs();

  if (s.running)
  {
    return "the loop is running - op 5 off first";
  }
  if (measured >= BOARD_CTRL_MEASURES)
  {
    return "measured is 0 angle deg, 1 omega_hat rad/s, 2 iq A";
  }
  if (command >= BOARD_CTRL_COMMANDS)
  {
    return "command is 0 theta rad, 1 iq_ref A";
  }
  if ((hz == 0U) || !(ts > 0.0f))
  {
    return "hz is 1 up to the PWM rate, and the drive needs its period first";
  }

  const float periods = 1.0f / (ts * (float)hz);

  if (periods < 0.5f)
  {
    return "hz is past the PWM rate";
  }
  s.divider = (uint16_t)lrintf(fminf(fmaxf(periods, 1.0f), DIVIDER_MAX));
  s.dt = (float)s.divider * ts;
  s.measured = measured;
  s.command = command;
  s.wired = true;
  return NULL;
}

const char *Board_CtrlRows(const uint16_t *ms, const float *setpoint, uint8_t n)
{
  if (n > ctrl_rows_free(&s.r))
  {
    return "more rows than the ring has room for - op 0 says how many";
  }
  for (uint8_t i = 0U; i < n; i++)
  {
    (void)ctrl_rows_push(&s.r, ms[i], setpoint[i]);
  }
  return NULL;
}

void Board_CtrlClear(void)
{
  const uint32_t masked = Board_IrqHold();

  ctrl_rows_clear(&s.r);
  Board_IrqRelease(masked);
}

const char *Board_CtrlRun(bool on)
{
  if (!on)
  {
    s.running = false;
    return NULL;
  }
  if (!s.wired)
  {
    return "nothing wired - op 2 first";
  }
  if (s.r.f.regulator.kind == (uint8_t)CTRL_NONE)
  {
    return "no regulator in slot 3 - op 1 first";
  }

  const uint32_t masked = Board_IrqHold();

  ctrl_feedback_reset(&s.r.f);
  s.count = 0U;
  s.running = true;
  Board_IrqRelease(masked);
  return NULL;
}
