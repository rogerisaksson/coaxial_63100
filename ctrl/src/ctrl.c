/** ctrl.c - The machine's parts and a feedback, as host/machine/parts.py steps them. */
#include "ctrl.h"

#include <math.h>
#include <string.h>

#define CTRL_TAU     6.28318530718f
#define CTRL_RAD_DEG 0.01745329252f

const char *const ctrl_kind_names[CTRL_KINDS] = {
  "", "Gain", "Slew", "Wrap", "LowPass", "SpeedKalman", "PI", "AngleHold", "Direct", "SpeedPI"
};

const uint8_t ctrl_kind_params[CTRL_KINDS] = { 0U, 1U, 1U, 1U, 1U, 5U, 3U, 5U, 1U, 7U };

static float clamp(float v, float limit)
{
  return (v > limit) ? limit : ((v < -limit) ? -limit : v);
}

bool ctrl_part_set(ctrl_part_t *part, uint8_t kind, const float *params)
{
  if (kind >= (uint8_t)CTRL_KINDS)
  {
    return false;
  }
  memset(part, 0, sizeof(*part));
  part->kind = kind;
  if (params != NULL)
  {
    memcpy(part->p, params, (size_t)ctrl_kind_params[kind] * sizeof(float));
  }
  ctrl_part_reset(part);
  return true;
}

void ctrl_part_reset(ctrl_part_t *part)
{
  part->x = 0.0f;
  part->y = 0.0f;
  part->at = 0.0f;
  part->was = (part->kind == (uint8_t)CTRL_SPEED_KALMAN) ? part->p[4] : 0.0f;
  part->primed = false;
}

float ctrl_filter(ctrl_part_t *part, float dt, float x)
{
  const float *p = part->p;

  switch (part->kind)
  {
  case CTRL_GAIN:
    return p[0] * x;
  case CTRL_SLEW:
    part->y += clamp(x - part->y, p[0] * dt);
    return part->y;
  case CTRL_WRAP:
  {
    float m = fmodf(x - p[0] + 180.0f, 360.0f);

    return ((m < 0.0f) ? m + 360.0f : m) - 180.0f;
  }
  case CTRL_LOW_PASS:
    part->y += dt / (p[0] + dt) * (x - part->y);
    return part->y;
  default:
    return x;
  }
}

float ctrl_estimate(ctrl_part_t *part, float dt, float measured, float command)
{
  const float *p = part->p;     /* kt j b q r */
  float gain;

  if (part->kind != (uint8_t)CTRL_SPEED_KALMAN)
  {
    return measured;
  }
  if (!part->primed)
  {
    part->primed = true;
    part->y = measured;
    return part->y;
  }
  part->y += dt * (p[0] * command - p[2] * part->y) / p[1];
  part->was += p[3] * dt;
  gain = part->was / (part->was + p[4]);
  part->y += gain * (measured - part->y);
  part->was *= 1.0f - gain;
  return part->y;
}

static float speed_pi(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                      bool held)
{
  const float *p = part->p;     /* hz limit kt j b load_k scale */
  float w0, err, damp, ff, raw, u;

  setpoint *= p[6];
  measured *= p[6];
  if (isnan(accel))
  {
    accel = (dt > 0.0f) ? (setpoint - part->was) / dt : 0.0f;
  }
  else
  {
    accel *= p[6];
  }
  part->was = setpoint;
  w0 = CTRL_TAU * p[0];
  err = setpoint - measured;
  damp = p[4] + 2.0f * p[5] * fabsf(setpoint);
  ff = (p[3] * accel + p[4] * setpoint + p[5] * setpoint * fabsf(setpoint)) / p[2];
  raw = w0 * p[3] / p[2] * err + part->x + ff;
  u = clamp(raw, p[1]);
  if ((u == raw) && !held)
  {
    part->x += w0 * damp / p[2] * err * dt;
  }
  return u;
}

float ctrl_regulate(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                    bool held)
{
  const float *p = part->p;

  switch (part->kind)
  {
  case CTRL_PI:                 /* kp ki limit */
  {
    float e = setpoint - measured;
    float raw = p[0] * e + part->x;
    float u = clamp(raw, p[2]);

    if (u == raw)
    {
      part->x += p[1] * e * dt;
    }
    return u;
  }
  case CTRL_ANGLE_HOLD:         /* poles theta0 ki trim most */
    part->x = clamp(part->x + p[2] * (setpoint - measured) * dt, p[3]);
    part->at += clamp(setpoint + part->x - part->at, p[4]);
    return p[1] + part->at * CTRL_RAD_DEG * p[0];
  case CTRL_DIRECT:
    return clamp(setpoint, p[0]);
  case CTRL_SPEED_PI:
    return speed_pi(part, dt, setpoint, measured, accel, held);
  default:
    return setpoint;
  }
}

void ctrl_feedback_reset(ctrl_feedback_t *f)
{
  ctrl_part_reset(&f->prefilter);
  ctrl_part_reset(&f->measure);
  ctrl_part_reset(&f->estimator);
  ctrl_part_reset(&f->regulator);
  f->ref = f->value = f->estimate = f->command = 0.0f;
}

float ctrl_feedback_step(ctrl_feedback_t *f, float dt, float setpoint, float measured)
{
  f->ref = ctrl_filter(&f->prefilter, dt, setpoint);
  f->value = ctrl_filter(&f->measure, dt, measured);
  f->estimate = ctrl_estimate(&f->estimator, dt, f->value, f->command);
  f->command = ctrl_regulate(&f->regulator, dt, f->ref, f->estimate, NAN, false);
  return f->command;
}
