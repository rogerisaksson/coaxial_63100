/** wbc_world.c - her world's steps on the host (wbc_world.h): World.advance's arithmetic between MuJoCo's steps,
    numpy's operation for operation; MuJoCo's contacts read off its data through its own headers. The host's
    alone. */
#include "wbc_world.h"
#include "wbc_np.h"

#include <math.h>
#include <stddef.h>
#include <mujoco/mujoco.h>

void wbc_world_rated(wbc_steps_t *s, double span)
{
  for (int i = 0; i < s->pass->n; i++)
  {
    if (span > 1e-9)
    {
      s->rate[i] = (s->target[i] - s->was[i]) / span;
    }
    s->was[i] = s->target[i];
  }
}

void wbc_world_before(wbc_steps_t *s, double since)
{
  wbc_pass_ctrl(s->pass, since, NULL, NULL);
  wbc_pass_motor(s->pass, s->step_s);
}

/* dclink.step: the link a step on under her drives' torques at their joints' speeds */
static void linked(wbc_steps_t *s)
{
  const wbc_world_t *p = s->pass;
  const int          n = (p->n < WBC_PASS_MAX) ? p->n : WBC_PASS_MAX;
  const double       v = s->link_v[0];
  const double       share = npclip((v - s->onset_v) / (s->clamp_v - s->onset_v), 0.0, 1.0);
  double             speed[WBC_PASS_MAX], bled[WBC_PASS_MAX], squared[WBC_PASS_MAX];
  double             bleed, load, linear, limited;

  for (int i = 0; i < n; i++)
  {
    const double q = p->motor[i] / s->link_kt[i];

    s->link_id[i] = sqrt(npmaximum(s->link_amps[i] * s->link_amps[i] - q * q, 0.0)) * share;
    bled[i] = s->link_id[i] * s->link_id[i];
    speed[i] = p->qvel[p->vadr[i]];
    squared[i] = p->motor[i] * p->motor[i];
  }
  bleed = wbc_dot(s->blas, s->link_ohm, 1, bled, 1, n);
  load = (((wbc_dot(s->blas, p->motor, 1, speed, 1, n) + wbc_dot(s->blas, s->link_loss, 1, squared, 1, n)) + bleed)
          + s->house_w) / v;
  linear = (((s->link_f * v) / s->step_s + s->pack_v / s->pack_ohm) - load) / (s->link_f / s->step_s + 1.0 / s->pack_ohm);
  limited = v + (s->step_s / s->link_f) * (-s->charge_a - load);
  s->link_v[0] = npmaximum(linear, limited);
  s->link_peak[0] = npmaximum(s->link_peak[0], s->link_v[0]);
  s->link_bled[0] += bleed * s->step_s;
}

/* room.touched: the lamp toggled where a hand touches its switch, not twice within its dead time */
static void switched(wbc_steps_t *s)
{
  double since = s->switched + s->step_s;

  if ((since >= s->switch_s) && (s->d->ncon > 0))
  {
    if (wbc_world_paired(s, s->switch_geom, s->hands, INFINITY) != 0)
    {
      s->lamp = (s->lamp != 0) ? 0 : 1;
      since = 0.0;
    }
  }
  s->switched = since;
}

void wbc_world_after(wbc_steps_t *s)
{
  wbc_pass_metered(s->pass, NULL, s->step_s);
  linked(s);
  if (s->switch_geom != NULL)
  {
    switched(s);
  }
}

int wbc_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below)
{
  const mjData    *d = s->d;
  const mjContact *c = d->contact;
  int              paired = 0;

  if (d->ncon > 0)
  {
    paired = wbc_pass_paired((const unsigned char *)c[0].geom, (const unsigned char *)c[0].pos,
                             (ptrdiff_t)sizeof(mjContact), d->ncon, a, b, below);
  }
  return paired;
}
