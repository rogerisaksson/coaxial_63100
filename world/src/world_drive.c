/** world_drive.c - A stand-in board's drive: the board's law and loop on the motor it models. */
#include "world_drive.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F 6.2831853f
#define TURN_DEG 360.0
#define TURN_RAD 6.283185307179586
#define COUNTS   4096.0     /* a turn as the A1335 counts it */

static world_drive_t s[WORLD_DRIVES];

static world_drive_t *at(int i)
{
  return ((i >= 0) && (i < WORLD_DRIVES)) ? &s[i] : NULL;
}

void world_drive_init(int i, float ts)
{
  world_drive_t *b = at(i);

  if (b == NULL)
  {
    return;
  }
  memset(b, 0, sizeof(*b));
  drive_init(&b->law, ts);
  b->law.vdc = b->law.model.p.vdc;
}

drive_t *world_drive_law(int i)
{
  world_drive_t *b = at(i);

  return (b != NULL) ? &b->law : NULL;
}

ctrl_loop_t *world_drive_loop(int i)
{
  world_drive_t *b = at(i);

  return (b != NULL) ? &b->loop : NULL;
}

/** The shaft as the A1335 reads it: a turn in twelve bits, deg. */
static float shaft_deg(const world_drive_t *b)
{
  const double turns = b->shaft / TURN_RAD;

  return (float)(floor((turns - floor(turns)) * COUNTS) * (TURN_DEG / COUNTS));
}

void world_drive_rest(int i)
{
  world_drive_t *b = at(i);

  if (b != NULL)
  {
    drive_model_init(&b->law.model);
    b->theta_was = b->law.model.theta;
  }
}

/** A period with the converters' bridge open: the law told its MOE, the rotor coasting. */
static bool open_period(drive_t *d, bool moe, drive_out_t *out)
{
  drive_sample_t in;

  drive_model_sample(&d->model, &in);
  const bool trip = drive_step(d, &in, moe, out);

  drive_model_coast(&d->model, d->ts);
  memset(d->model.duty_prev, 0, sizeof(d->model.duty_prev));
  d->model.driven_prev = false;
  return trip;
}

uint32_t world_drive_run(int i, uint32_t periods, bool moe, bool supplied)
{
  world_drive_t *b = at(i);
  uint32_t tripped = 0U;

  if (b == NULL)
  {
    return 0U;
  }
  drive_t *d = &b->law;
  const bool closed = (moe && supplied) || (d->source == DRIVE_SOURCE_MODEL);
  drive_out_t out = { { 0.0f, 0.0f, 0.0f }, false };

  for (uint32_t k = 0U; k < periods; k++)
  {
    if (ctrl_loop_due(&b->loop))
    {
      ctrl_loop_step(&b->loop, d, shaft_deg(b));
    }
    const bool trip = closed ? drive_step_virtual(d, &out) : open_period(d, moe, &out);
    const float turned = d->model.theta - b->theta_was;

    /* A period turns the rotor a fraction of a turn: the rest of a gap is its wrap. */
    b->shaft += (double)((turned - (TWO_PI_F * nearbyintf(turned / TWO_PI_F)))
                         / d->model.p.pole_pairs);
    b->theta_was = d->model.theta;
    tripped += trip ? 1U : 0U;
  }
  if (periods > 0U)
  {
    memcpy(b->duty, out.duty, sizeof(b->duty));
    b->driven = out.driven;
  }
  b->trips += tripped;
  return tripped;
}

int world_drive_read(int i, double *out)
{
  const world_drive_t *b = at(i);
  int n = 0;

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  out[n++] = (double)b->law.periods;
  out[n++] = (double)b->trips;
  out[n++] = b->law.inj_valid ? 1.0 : 0.0;
  out[n++] = b->driven ? 1.0 : 0.0;
  for (int k = 0; k < (int)DRIVE_PHASES; k++)
  {
    out[n++] = (double)b->duty[k];
  }
  for (int k = 0; k < (int)DRIVE_PHASES; k++)
  {
    out[n++] = (double)b->law.model.i_abc[k];
  }
  out[n++] = b->shaft;
  return n;
}

int world_drive_loop_read(int i, double *out)
{
  const world_drive_t *b = at(i);
  ctrl_loop_state_t st;
  int n = 0;

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  ctrl_loop_state(&b->loop, &st);
  out[n++] = st.running ? 1.0 : 0.0;
  out[n++] = st.playing ? 1.0 : 0.0;
  out[n++] = (double)st.measured;
  out[n++] = (double)st.command;
  out[n++] = (double)st.hz;
  out[n++] = (double)st.rows;
  out[n++] = (double)st.free;
  out[n++] = (double)st.queued_s;
  out[n++] = (double)st.played;
  out[n++] = (double)st.idle;
  out[n++] = (double)st.blind;
  out[n++] = (double)st.setpoint;
  out[n++] = (double)st.ref;
  out[n++] = (double)st.value;
  out[n++] = (double)st.estimate;
  out[n++] = (double)st.out;
  return n;
}
