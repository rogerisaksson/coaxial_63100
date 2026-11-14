/** wbc_world.c - her world's steps on the host (wbc_world.h): World.advance's arithmetic between MuJoCo's steps,
    her gearboxes' with it, numpy's operation for operation and its own tanh; MuJoCo's contacts read off its data
    through its own headers. The host's alone. */
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

/* np.tanh of `n` doubles `x` into `y` through numpy's own loop: its bits */
static void nptanh(const wbc_boxes_t *b, double *x, double *y, int n)
{
  char           *args[2] = {(char *)x, (char *)y};
  const ptrdiff_t dims[1] = {(ptrdiff_t)n};
  const ptrdiff_t steps[2] = {(ptrdiff_t)sizeof(double), (ptrdiff_t)sizeof(double)};

  b->tanh(args, dims, steps, b->tanh_data);
}

/* World.advance's `WOUND * boxes.ahead(motor)` and `WOUND * (boxes.w - qvel) * sprung`: each stepped box's rotor asked
   past its setpoint by its wind-up and the play its last torque takes up, its rotor's speed past its link's */
static void ahead(const wbc_steps_t *s, double *at, double *wrate, int n)
{
  const wbc_boxes_t *b = s->boxes;
  const wbc_world_t *p = s->pass;
  double             x[WBC_PASS_MAX], th[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    x[i] = p->motor[i] / b->play_nm;
  }
  nptanh(b, x, th, n);
  for (int i = 0; i < n; i++)
  {
    const int    on = b->on[i] != 0U;
    const double k = on ? b->k[i] : 1.0;

    at[i] = p->wound * (on ? (b->ahead_wind * p->motor[i]) / k + (b->ahead_play * b->gap[i]) * th[i] : 0.0);
    wrate[i] = (p->wound * (b->w[i] - p->qvel[p->vadr[i]])) * (double)b->on[i];
  }
}

/* Boxes.step on ctrl at the step's middle (the start's speed and half a step of the last acceleration): what each
   stepped box passes on to its link - its wind-up past the play, its damping, its mesh's friction -, its rotor
   stepped `substeps` times on its board's drive - the command, its torque loop on its observer's estimate against
   the feed, within its peak -, braked where its drive is shorted; ctrl the mean each stepped box passed on */
static void stepped(const wbc_steps_t *s, int n)
{
  const wbc_boxes_t *b = s->boxes;
  wbc_world_t       *p = s->pass;
  const double       dt = s->step_s, h = dt / (double)b->substeps, ease = dt / b->rate_s, half = 0.5 * dt;
  double             qd[WBC_PASS_MAX], ctrl[WBC_PASS_MAX], brake[WBC_PASS_MAX], j[WBC_PASS_MAX], k[WBC_PASS_MAX];
  double             lim[WBC_PASS_MAX], mean[WBC_PASS_MAX], rel[WBC_PASS_MAX], x[WBC_PASS_MAX], th[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    qd[i] = p->qvel[p->vadr[i]] + half * p->qacc[p->vadr[i]];
    ctrl[i] = p->ctrl[i];
    brake[i] = (p->shorted[i] != 0U) ? b->damping[i] : 0.0;
    j[i] = (b->on[i] != 0U) ? b->j[i] : 1.0;
    k[i] = npmaximum(b->k[i], 1.0);
    mean[i] = 0.0;
    b->q_step[i] = p->qpos[p->qadr[i]];
    b->went[i] = dt * qd[i];
    lim[i] = b->gap[i] + b->shock[i] / k[i];
    b->rate[i] = b->rate[i] + ease * ((p->feed[i] - b->asked[i]) / dt - b->rate[i]);
    b->asked[i] = p->feed[i];
  }
  for (int sub = 0; sub < b->substeps; sub++)
  {
    for (int i = 0; i < n; i++)
    {
      rel[i] = b->w[i] - qd[i];
      x[i] = rel[i] / b->mesh_rad_s;
    }
    nptanh(b, x, th, n);
    for (int i = 0; i < n; i++)
    {
      const int    on = b->on[i] != 0U;
      const double past = npsign(b->delta[i]) * npmaximum(fabs(b->delta[i]) - b->gap[i], 0.0);
      double       passed = (past != 0.0) ? (b->k[i] * past + b->c[i] * rel[i]) + b->mesh[i] * th[i] : 0.0;
      double       drive;

      passed = on ? npclip(passed, -b->shock[i], b->shock[i]) : passed;
      b->p_int[i] = on ? b->p_int[i] + h * ((b->drive[i] - brake[i] * b->w[i]) - b->hat[i]) : 0.0;
      b->hat[i] = b->observe[i] * (b->p_int[i] - j[i] * b->w[i]);
      drive = ctrl[i] + b->track[i] * ((p->feed[i] - b->hat[i]) + b->c[i] * (b->rate[i] / k[i] - rel[i]));
      b->drive[i] = on ? npclip(drive, -b->peak[i], b->peak[i]) : ctrl[i];
      b->w[i] = on ? (b->w[i] + (h * (b->drive[i] - passed)) / j[i]) / (1.0 + (h * brake[i]) / j[i]) : qd[i];
      b->delta[i] = on ? npclip(b->delta[i] + h * (b->w[i] - qd[i]), -lim[i], lim[i]) : b->delta[i];
      mean[i] = mean[i] + passed / (double)b->substeps;
    }
  }
  for (int i = 0; i < n; i++)
  {
    p->ctrl[i] = (b->on[i] != 0U) ? mean[i] : ctrl[i];
  }
}

/* Boxes.moved: each stepped rotor's angle past its link, its rotor's turn less where its link landed */
static void moved(const wbc_steps_t *s, int n)
{
  const wbc_boxes_t *b = s->boxes;
  const wbc_world_t *p = s->pass;

  for (int i = 0; i < n; i++)
  {
    b->delta[i] = b->delta[i] + (b->went[i] - (p->qpos[p->qadr[i]] - b->q_step[i])) * (double)b->on[i];
  }
}

void wbc_world_before(wbc_steps_t *s, double since)
{
  const int n = (s->pass->n < WBC_PASS_MAX) ? s->pass->n : WBC_PASS_MAX;
  double    at[WBC_PASS_MAX], wrate[WBC_PASS_MAX];

  if (s->boxes != NULL)
  {
    ahead(s, at, wrate, n);
  }
  wbc_pass_ctrl(s->pass, since, (s->boxes != NULL) ? at : NULL, (s->boxes != NULL) ? wrate : NULL);
  wbc_pass_motor(s->pass, s->step_s);
  if (s->boxes != NULL)
  {
    stepped(s, n);
  }
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
  if (s->boxes != NULL)
  {
    moved(s, (s->pass->n < WBC_PASS_MAX) ? s->pass->n : WBC_PASS_MAX);
  }
  wbc_pass_metered(s->pass, (s->boxes != NULL) ? s->boxes->w : NULL, s->step_s);
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

int wbc_world_contacted(const mjData *d, wbc_contacts_t *c)
{
  const mjContact *con = d->contact;
  const double    *applied = d->xfrc_applied;
  int              shoved = 0;

  if (d->ncon > 0)
  {
    const char *pos = (const char *)con[0].pos;

    if (c->stride == 0)
    {
      c->stride = (int)sizeof(mjContact);
      c->off_geom = (int)((const char *)con[0].geom - pos);
      c->off_frame = (int)((const char *)con[0].frame - pos);
      c->off_dim = (int)((const char *)&con[0].dim - pos);
      c->off_efc = (int)((const char *)&con[0].efc_address - pos);
      c->off_friction = (int)((const char *)con[0].friction - pos);
    }
    wbc_pass_contacts(c, d->ncon, pos, d->efc_force, d->efc_R);
  }
  else
  {
    wbc_pass_contacts(c, 0, NULL, NULL, NULL);
  }
  for (int i = 0; i < (6 * c->nbody); i++)
  {
    shoved = (applied[i] != 0.0) ? 1 : shoved;
  }
  return shoved;
}

int wbc_world_held(mjData *d, const wbc_held_t *h)
{
  const mjContact *con = d->contact;
  int              woke = 0;

  for (int k = 0; k < h->nthing; k++)
  {
    h->woken[k] = 0U;
  }
  for (int i = 0; i < d->ncon; i++)
  {
    const int g0 = con[i].geom[0];
    const int g1 = con[i].geom[1];

    if ((g0 >= 0) && (g1 >= 0))
    {
      const int k0 = h->by_geom[g0];
      const int k1 = h->by_geom[g1];

      if ((k0 >= 0) && (k1 < 0) && (h->live[g1] != 0U))
      {
        h->woken[k0] = 1U;
      }
      if ((k1 >= 0) && (k0 < 0) && (h->live[g0] != 0U))
      {
        h->woken[k1] = 1U;
      }
    }
  }
  for (int i = 0; i < h->nvel; i++)
  {
    if (fabs(d->qvel[h->vel[i]]) > h->low)
    {
      h->woken[h->of[i / 3]] = 1U;
    }
  }
  for (int k = 0; k < h->nthing; k++)
  {
    woke += (int)h->woken[k];
  }
  if (woke == 0)
  {
    for (int i = 0; i < h->nat; i++)
    {
      d->qpos[h->at[i]] = h->lie[i];
    }
    for (int i = 0; i < h->nstill; i++)
    {
      d->qvel[h->still[i]] = 0.0;
    }
  }
  return woke;
}
