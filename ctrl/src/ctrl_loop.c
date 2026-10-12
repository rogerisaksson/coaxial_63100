/** ctrl_loop.c - A board's loop on its drive: ctrl's runner ticked in the drive's sample. */
#include "ctrl_loop.h"

#include <math.h>
#include <stddef.h>

#define DIVIDER_MAX 65535.0f

/** The slots in pass order. */
static ctrl_part_t *slot_part(ctrl_loop_t *l, uint8_t slot)
{
  ctrl_part_t *const parts[CTRL_LOOP_SLOTS] = {
    &l->r.f.prefilter, &l->r.f.measure, &l->r.f.estimator, &l->r.f.regulator,
  };

  return parts[slot];
}

bool ctrl_loop_due(ctrl_loop_t *l)
{
  if (!l->running || (++l->count < l->divider))
  {
    return false;
  }
  l->count = 0U;
  return true;
}

void ctrl_loop_step(ctrl_loop_t *l, drive_t *d, float angle_deg)
{
  const float x = (l->measured == CTRL_LOOP_OMEGA) ? d->omega_hat
                  : ((l->measured == CTRL_LOOP_IQ) ? d->iq : angle_deg);

  if (isnan(x))
  {
    l->blind++;
    return;
  }

  const bool amps = l->command == CTRL_LOOP_IQ_REF;
  /* A current asked is carried as far as the drive's clamp, its derate in it, and its
     link's volts. */
  const float u = ctrl_runner_step(&l->r, l->dt, x,
                                   amps ? fminf(d->p.i_max, drive_room(d)) : INFINITY);

  if (amps)
  {
    d->sp.iq_ref = u;
  }
  else
  {
    /* Written to the setpoint alone the held frame stood where the mode began. */
    drive_set_theta_setpoint(d, u);
  }
}

void ctrl_loop_state(const ctrl_loop_t *l, ctrl_loop_state_t *out)
{
  out->running = l->running;
  out->playing = l->r.playing;
  out->measured = l->measured;
  out->command = l->command;
  out->hz = (l->dt > 0.0f) ? (uint16_t)lrintf(1.0f / l->dt) : 0U;
  out->free = ctrl_rows_free(&l->r);
  out->rows = (uint16_t)(CTRL_ROWS - 1U - out->free);
  out->queued_s = ctrl_rows_seconds(&l->r);
  out->played = l->r.played;
  out->idle = l->r.idle;
  out->blind = l->blind;
  out->setpoint = l->r.setpoint;
  out->ref = l->r.f.ref;
  out->value = l->r.f.value;
  out->estimate = l->r.f.estimate;
  out->out = l->r.f.command;
}

const char *ctrl_loop_slot(ctrl_loop_t *l, uint8_t slot, uint8_t kind, const float *params,
                           uint8_t n)
{
  if (l->running)
  {
    return "the loop is running - op 5 off first";
  }
  if (slot >= CTRL_LOOP_SLOTS)
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
  (void)ctrl_part_set(slot_part(l, slot), kind, params);
  return NULL;
}

const char *ctrl_loop_wire(ctrl_loop_t *l, float ts, uint8_t measured, uint8_t command,
                           uint16_t hz)
{
  if (l->running)
  {
    return "the loop is running - op 5 off first";
  }
  if (measured >= CTRL_LOOP_MEASURES)
  {
    return "measured is 0 angle deg, 1 omega_hat rad/s, 2 iq A";
  }
  if (command >= CTRL_LOOP_COMMANDS)
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
  l->divider = (uint16_t)lrintf(fminf(fmaxf(periods, 1.0f), DIVIDER_MAX));
  l->dt = (float)l->divider * ts;
  l->measured = measured;
  l->command = command;
  l->wired = true;
  return NULL;
}

const char *ctrl_loop_rows(ctrl_loop_t *l, const uint16_t *ms, const float *setpoint, uint8_t n)
{
  if (n > ctrl_rows_free(&l->r))
  {
    return "more rows than the ring has room for - op 0 says how many";
  }
  for (uint8_t i = 0U; i < n; i++)
  {
    (void)ctrl_rows_push(&l->r, ms[i], setpoint[i]);
  }
  return NULL;
}

const char *ctrl_loop_run(ctrl_loop_t *l, bool on)
{
  if (!on)
  {
    l->running = false;
    return NULL;
  }
  if (!l->wired)
  {
    return "nothing wired - op 2 first";
  }
  if (l->r.f.regulator.kind == (uint8_t)CTRL_NONE)
  {
    return "no regulator in slot 3 - op 1 first";
  }
  ctrl_feedback_reset(&l->r.f);
  l->count = 0U;
  l->running = true;
  return NULL;
}
