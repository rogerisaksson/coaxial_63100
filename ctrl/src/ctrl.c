/** ctrl.c - machine.parts and a feedback, step for step (ctrl.h). */
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

/* ---- the steps: one a kind, its part's own state and nothing else ---------------------- */

static float gain(ctrl_part_t *part, float dt, float x)
{
  (void)dt;
  return part->p[0] * x;
}

static float slew(ctrl_part_t *part, float dt, float x)
{
  part->y += clamp(x - part->y, part->p[0] * dt);
  return part->y;
}

static float wrap(ctrl_part_t *part, float dt, float x)
{
  float m = fmodf(x - part->p[0] + 180.0f, 360.0f);

  (void)dt;
  return ((m < 0.0f) ? m + 360.0f : m) - 180.0f;
}

static float low_pass(ctrl_part_t *part, float dt, float x)
{
  part->y += dt / (part->p[0] + dt) * (x - part->y);
  return part->y;
}

static float speed_kalman(ctrl_part_t *part, float dt, float measured, float command)
{
  const float *p = part->p;     /* kt j b q r */
  float gain_k;

  if (!part->primed)
  {
    part->primed = true;
    part->y = measured;
    return part->y;
  }
  part->y += dt * (p[0] * command - p[2] * part->y) / p[1];
  part->was += p[3] * dt;
  gain_k = part->was / (part->was + p[4]);
  part->y += gain_k * (measured - part->y);
  part->was *= 1.0f - gain_k;
  return part->y;
}

static float pi(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                bool held)
{
  const float *p = part->p;     /* kp ki limit */
  const float e = setpoint - measured;
  const float raw = p[0] * e + part->x;
  const float u = clamp(raw, p[2]);

  (void)accel;
  (void)held;
  if (u == raw)
  {
    part->x += p[1] * e * dt;
  }
  return u;
}

static float angle_hold(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                        bool held)
{
  const float *p = part->p;     /* poles theta0 ki trim most */

  (void)accel;
  (void)held;
  part->x = clamp(part->x + p[2] * (setpoint - measured) * dt, p[3]);
  part->at += clamp(setpoint + part->x - part->at, p[4]);
  return p[1] + part->at * CTRL_RAD_DEG * p[0];
}

static float direct(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                    bool held)
{
  (void)dt;
  (void)measured;
  (void)accel;
  (void)held;
  return clamp(setpoint, part->p[0]);
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

/* ---- the tables: a slot's step by kind, NULL where the kind passes its input through ------ */

typedef float (*ctrl_filter_fn)(ctrl_part_t *part, float dt, float x);
typedef float (*ctrl_estimate_fn)(ctrl_part_t *part, float dt, float measured, float command);
typedef float (*ctrl_regulate_fn)(ctrl_part_t *part, float dt, float setpoint, float measured,
                                  float accel, bool held);

static const ctrl_filter_fn FILTERS[CTRL_KINDS] = {
  [CTRL_GAIN] = gain, [CTRL_SLEW] = slew, [CTRL_WRAP] = wrap, [CTRL_LOW_PASS] = low_pass
};

static const ctrl_estimate_fn ESTIMATORS[CTRL_KINDS] = {
  [CTRL_SPEED_KALMAN] = speed_kalman
};

static const ctrl_regulate_fn REGULATORS[CTRL_KINDS] = {
  [CTRL_PI] = pi, [CTRL_ANGLE_HOLD] = angle_hold, [CTRL_DIRECT] = direct,
  [CTRL_SPEED_PI] = speed_pi
};

float ctrl_filter(ctrl_part_t *part, float dt, float x)
{
  const ctrl_filter_fn step = (part->kind < (uint8_t)CTRL_KINDS) ? FILTERS[part->kind] : NULL;

  return (step != NULL) ? step(part, dt, x) : x;
}

float ctrl_estimate(ctrl_part_t *part, float dt, float measured, float command)
{
  const ctrl_estimate_fn step = (part->kind < (uint8_t)CTRL_KINDS) ? ESTIMATORS[part->kind]
                                                                  : NULL;

  return (step != NULL) ? step(part, dt, measured, command) : measured;
}

float ctrl_regulate(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                    bool held)
{
  const ctrl_regulate_fn step = (part->kind < (uint8_t)CTRL_KINDS) ? REGULATORS[part->kind]
                                                                  : NULL;

  return (step != NULL) ? step(part, dt, setpoint, measured, accel, held) : setpoint;
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

uint16_t ctrl_rows_free(const ctrl_runner_t *r)
{
  uint16_t used = (uint16_t)((r->head + CTRL_ROWS - r->tail) % CTRL_ROWS);

  return (uint16_t)(CTRL_ROWS - 1U - used);
}

bool ctrl_rows_push(ctrl_runner_t *r, uint16_t ms, float setpoint)
{
  uint16_t head = r->head;

  if (ctrl_rows_free(r) == 0U)
  {
    return false;
  }
  r->row[head].ms = ms;
  r->row[head].setpoint = setpoint;
  r->head = (uint16_t)((head + 1U) % CTRL_ROWS);   /* the row first, then the index */
  return true;
}

float ctrl_rows_seconds(const ctrl_runner_t *r)
{
  float s = (r->playing && (r->left_s > 0.0f)) ? r->left_s : 0.0f;

  for (uint16_t i = r->tail; i != r->head; i = (uint16_t)((i + 1U) % CTRL_ROWS))
  {
    s += (float)r->row[i].ms * 0.001f;
  }
  return s;
}

void ctrl_rows_clear(ctrl_runner_t *r)
{
  r->tail = r->head;
  r->left_s = 0.0f;
  r->playing = false;
}

float ctrl_runner_step(ctrl_runner_t *r, float dt, float measured)
{
  if (r->left_s <= 0.0f)
  {
    if (r->playing)
    {
      r->played++;
      r->playing = false;
    }
    if (r->tail != r->head)
    {
      const ctrl_row_t *row = &r->row[r->tail];

      r->setpoint = row->setpoint;
      r->left_s += (float)row->ms * 0.001f;
      r->playing = true;
      r->tail = (uint16_t)((r->tail + 1U) % CTRL_ROWS);
    }
    else
    {
      r->left_s = 0.0f;
    }
  }
  if (r->playing)
  {
    r->left_s -= dt;
  }
  else
  {
    r->idle++;
  }
  return ctrl_feedback_step(&r->f, dt, r->setpoint, measured);
}
