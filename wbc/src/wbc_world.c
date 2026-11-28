/** wbc_world.c - the robot's world's steps on the host (wbc_world.h): World.advance's arithmetic between MuJoCo's steps,
    its gearboxes' with it, numpy's operation for operation and its own tanh; MuJoCo's contacts read off
    its data through its own headers, their friction rows sprung to where they stuck. The host's alone. */
#include "wbc_world.h"
#include "wbc_np.h"
#include "se3.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
#include <mujoco/mujoco.h>

_Static_assert(sizeof(mjtNum) == sizeof(double), "MuJoCo mjtNum must be IEEE-754 float64");

void wbc_world_rated(wbc_steps_t *s, double span)
{
  const int            n = s->pass->n;
  double *restrict     rate = s->rate;
  double *restrict     was = s->was;
  const double *restrict target = s->target;

  if (span > 1e-9)
  {
    for (int i = 0; i < n; i++)
    {
      rate[i] = (target[i] - was[i]) / span;
    }
  }
  for (int i = 0; i < n; i++)
  {
    was[i] = target[i];
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
static void ahead(const wbc_steps_t *s, double *restrict at, double *restrict wrate, int n)
{
  const wbc_boxes_t             *b = s->boxes;
  const wbc_world_t             *p = s->pass;
  const double *restrict         motor = p->motor, *restrict bk = b->k, *restrict gap = b->gap;
  const double *restrict         bw = b->w, *restrict qvel = p->qvel;
  const unsigned char *restrict  on_arr = b->on;
  const int *restrict            vadr = p->vadr;
  const double                   play_nm = b->play_nm, aw = b->ahead_wind, ap = b->ahead_play, wound = p->wound;
  _Alignas(64) double            x[WBC_PASS_MAX], th[WBC_PASS_MAX], qv[WBC_PASS_MAX];
  double *restrict               xp = x, *restrict thp = th;

  for (int i = 0; i < n; i++)
  {
    qv[i] = qvel[vadr[i]];
  }
  for (int i = 0; i < n; i++)
  {
    xp[i] = motor[i] / play_nm;
  }
  nptanh(b, xp, thp, n);
  for (int i = 0; i < n; i++)
  {
    const int    on = on_arr[i] != 0U;
    const double k = on ? bk[i] : 1.0;

    at[i] = wound * (on ? (aw * motor[i]) / k + (ap * gap[i]) * thp[i] : 0.0);
    wrate[i] = (wound * (bw[i] - qv[i])) * (double)on_arr[i];
  }
}

/* Boxes.step on ctrl at the step's middle (the start's speed and half a step of the last acceleration): what each
   stepped box passes on to its link - its wind-up past the play, its damping, its mesh's friction -, its rotor
   stepped `substeps` times on its board's drive - the command, its torque loop on its observer's estimate against
   the feed, within its peak -, braked where its drive is shorted; ctrl the mean each stepped box passed on */
static void stepped(const wbc_steps_t *s, int n)
{
  const wbc_boxes_t             *b = s->boxes;
  wbc_world_t                   *p = s->pass;
  const int                      subs = b->substeps;
  const double                   dt = s->step_s, h = dt / (double)subs, ease = dt / b->rate_s;
  const double                   half = 0.5 * dt, third = dt / 3.0, play_nm = b->play_nm, inv_subs = (double)subs;
  double *restrict               p_ctrl = p->ctrl, *restrict p_motor = p->motor, *restrict q_step = b->q_step, *restrict went = b->went;
  double *restrict               brate = b->rate, *restrict asked = b->asked, *restrict p_int = b->p_int;
  double *restrict               hat = b->hat, *restrict bdrive = b->drive, *restrict bw = b->w, *restrict delta = b->delta;
  const double *restrict         p_feed = p->feed, *restrict p_limit = p->limit, *restrict p_gains = p->gains;
  const double *restrict         qpos = p->qpos, *restrict qvel = p->qvel, *restrict qacc = p->qacc;
  const double *restrict         damping = b->damping, *restrict bj = b->j, *restrict bk = b->k, *restrict bc = b->c;
  const double *restrict         gap = b->gap, *restrict shock = b->shock, *restrict mesh = b->mesh;
  const double *restrict         v_mesh = b->v_mesh, *restrict inv_m = b->inv_m;
  const double *restrict         observe = b->observe, *restrict track = b->track;
  const int *restrict            qadr = p->qadr, *restrict vadr = p->vadr;
  const unsigned char *restrict  shorted = p->shorted, *restrict cut = p->cut, *restrict on_arr = b->on;
  _Alignas(64) double            qp[WBC_PASS_MAX], qv[WBC_PASS_MAX], qa[WBC_PASS_MAX], pg[WBC_PASS_MAX];
  _Alignas(64) double            qd[WBC_PASS_MAX], ctrl[WBC_PASS_MAX], brake[WBC_PASS_MAX], j[WBC_PASS_MAX], k[WBC_PASS_MAX];
  _Alignas(64) double            top[WBC_PASS_MAX], feed[WBC_PASS_MAX], trk[WBC_PASS_MAX], cd[WBC_PASS_MAX];
  _Alignas(64) double            lim[WBC_PASS_MAX], mean[WBC_PASS_MAX], mdrv[WBC_PASS_MAX], rel[WBC_PASS_MAX];
  _Alignas(64) double            x[WBC_PASS_MAX], th[WBC_PASS_MAX];
  double *restrict               xp = x, *restrict thp = th;

  for (int i = 0; i < n; i++)
  {
    const int vi = vadr[i];

    qp[i] = qpos[qadr[i]];
    qv[i] = qvel[vi];
    qa[i] = qacc[vi];
    pg[i] = p_gains[2 * i];
    xp[i] = (bw[i] - qv[i]) / v_mesh[i];
  }
  nptanh(b, xp, thp, n);
  for (int i = 0; i < n; i++)
  {
    const double b0 = bc[i] + (mesh[i] / v_mesh[i]) * (1.0 - thp[i] * thp[i]);
    const double im = inv_m[i];
    const double h_mid = (im > 0.0) ? (half / (1.0 + (third * b0) * im)) : half;
    const double lim_live = ((shorted[i] | cut[i]) != 0U) ? 0.0 : p_limit[i];
    const int    pd_on = pg[i] > 0.0;
    const double ti = npminimum(shock[i], lim_live);
    const double fi = npclip(p_feed[i], -ti, ti);
    const double ki = npmaximum(bk[i], 1.0);
    const double qi = qv[i] + h_mid * qa[i];
    const double ri = brate[i] + ease * ((fi - asked[i]) / dt - brate[i]);
    const int    act = (ti > 0.0) & (!pd_on) & ((fi != 0.0) | (ri != 0.0));
    const double ji = (on_arr[i] != 0U) ? bj[i] : 1.0;
    const double trki = (act != 0) ? track[i] : 0.0;
    const double c_cl = 2.0 * sqrt((bk[i] * (1.0 + trki)) * ji) - (1.0 + trki) * bc[i];

    top[i] = ti;
    feed[i] = fi;
    qd[i] = qi;
    ctrl[i] = p_ctrl[i];
    brake[i] = (shorted[i] != 0U) ? damping[i] : 0.0;
    j[i] = ji;
    k[i] = ki;
    mean[i] = 0.0;
    mdrv[i] = 0.0;
    q_step[i] = qp[i];
    went[i] = dt * qi;
    lim[i] = gap[i] + shock[i] / ki;
    brate[i] = ri;
    asked[i] = fi;
    trk[i] = trki;
    cd[i] = (trki > 0.0) ? npmaximum(trki * bc[i], c_cl) : 0.0;
  }
  for (int sub = 0; sub < subs; sub++)
  {
    for (int i = 0; i < n; i++)
    {
      rel[i] = bw[i] - qd[i];
      xp[i] = rel[i] / v_mesh[i];
    }
    nptanh(b, xp, thp, n);
    for (int i = 0; i < n; i++)
    {
      const int    on = on_arr[i] != 0U;
      const double d0 = delta[i];
      const double past = npsign(d0) * npmaximum(fabs(d0) - gap[i], 0.0);
      double       passed = (past != 0.0) ? (bk[i] * past + bc[i] * rel[i]) + mesh[i] * thp[i] : 0.0;
      double       pi, ht, cap, drive, wi;

      passed = on ? npclip(passed, -shock[i], shock[i]) : passed;
      pi = on ? p_int[i] + h * ((bdrive[i] - brake[i] * bw[i]) - hat[i]) : 0.0;
      ht = observe[i] * (pi - j[i] * bw[i]);
      cap = npminimum(top[i], npmaximum(fabs(ht), fabs(feed[i])) + play_nm);
      drive = (ctrl[i] + trk[i] * npclip(feed[i] - ht, -cap, cap)) + cd[i] * (brate[i] / k[i] - rel[i]);
      drive = on ? npclip(drive, -top[i], top[i]) : ctrl[i];
      wi = on ? (bw[i] + (h * (drive - passed)) / j[i]) / (1.0 + (h * brake[i]) / j[i]) : qd[i];
      p_int[i] = pi;
      hat[i] = ht;
      bdrive[i] = drive;
      bw[i] = wi;
      delta[i] = on ? npclip(d0 + h * (wi - qd[i]), -lim[i], lim[i]) : d0;
      mean[i] = mean[i] + passed / inv_subs;
      mdrv[i] = mdrv[i] + drive / inv_subs;
    }
  }
  for (int i = 0; i < n; i++)
  {
    const int mv = (on_arr[i] != 0U) && (trk[i] != 0.0);

    p_ctrl[i] = (on_arr[i] != 0U) ? mean[i] : ctrl[i];
    p_motor[i] = mv ? mdrv[i] : p_motor[i];
  }
}

/* Boxes.moved: each stepped rotor's angle past its link, its rotor's turn less where its link landed */
static void moved(const wbc_steps_t *s, int n)
{
  const wbc_boxes_t             *b = s->boxes;
  const wbc_world_t             *p = s->pass;
  double *restrict               delta = b->delta;
  const double *restrict         went = b->went, *restrict q_step = b->q_step, *restrict qpos = p->qpos;
  const int *restrict            qadr = p->qadr;
  const unsigned char *restrict  on = b->on;
  _Alignas(64) double            qp[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    qp[i] = qpos[qadr[i]];
  }
  for (int i = 0; i < n; i++)
  {
    delta[i] = delta[i] + (went[i] - (qp[i] - q_step[i])) * (double)on[i];
  }
}

void wbc_world_drives(wbc_steps_t *s, double since)
{
  const int           n = (s->pass->n < WBC_PASS_MAX) ? s->pass->n : WBC_PASS_MAX;
  _Alignas(64) double at[WBC_PASS_MAX], wrate[WBC_PASS_MAX];

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

void wbc_world_stuck_step(wbc_steps_t *s)
{
  if (s->stuck != NULL)
  {
    wbc_world_stuck(s->d, s->stuck);
  }
}

void wbc_world_before(wbc_steps_t *s, double since)
{
  wbc_world_drives(s, since);
  wbc_world_stuck_step(s);
}

/* dclink.step: the link a step on under the robot's drives' torques at their joints' speeds */
static void linked(wbc_steps_t *s)
{
  const wbc_world_t      *p = s->pass;
  const int               n = (p->n < WBC_PASS_MAX) ? p->n : WBC_PASS_MAX;
  const double            v = s->link_v[0];
  const double            share = npclip((v - s->onset_v) / (s->clamp_v - s->onset_v), 0.0, 1.0);
  double *restrict        link_id = s->link_id;
  const double *restrict  motor = p->motor, *restrict kt = s->link_kt, *restrict amps = s->link_amps;
  const double *restrict  qvel = p->qvel;
  const int *restrict     vadr = p->vadr;
  _Alignas(64) double     speed[WBC_PASS_MAX], bled[WBC_PASS_MAX], squared[WBC_PASS_MAX];
  double                  bleed, load, linear, limited;

  for (int i = 0; i < n; i++)
  {
    speed[i] = qvel[vadr[i]];
  }
  for (int i = 0; i < n; i++)
  {
    const double mi = motor[i];
    const double q = mi / kt[i];
    const double id = sqrt(npmaximum(amps[i] * amps[i] - q * q, 0.0)) * share;

    link_id[i] = id;
    bled[i] = id * id;
    squared[i] = mi * mi;
  }
  bleed = wbc_dot(s->blas, s->link_ohm, 1, bled, 1, n);
  load = (((wbc_dot(s->blas, motor, 1, speed, 1, n) + wbc_dot(s->blas, s->link_loss, 1, squared, 1, n)) + bleed)
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

void wbc_world_contacts_after(wbc_steps_t *s)
{
  if (s->stuck != NULL)
  {
    wbc_world_slipped(s->d, s->stuck);
  }
  if (s->switch_geom != NULL)
  {
    switched(s);
  }
}

void wbc_world_rest_after(wbc_steps_t *s)
{
  if (s->boxes != NULL)
  {
    moved(s, (s->pass->n < WBC_PASS_MAX) ? s->pass->n : WBC_PASS_MAX);
  }
  wbc_pass_metered(s->pass, (s->boxes != NULL) ? s->boxes->w : NULL, s->step_s);
  linked(s);
}

void wbc_world_after(wbc_steps_t *s)
{
  wbc_world_contacts_after(s);
  wbc_world_rest_after(s);
}

/* which of a contact's two slips each of its friction rows reads - along its plane (0) or about (3) - and on which
   of its frame's rows: its two tangents, its normal, its two tangents */
static const int slip_of[5] = {0, 0, 3, 3, 3};
static const int slip_on[5] = {3, 6, 0, 3, 6};

/* whether MuJoCo's constraints hold contact `c`'s friction rows, an elliptic cone's between two geoms */
static int gripped(const mjData *d, const mjContact *c)
{
  return ((c->efc_address >= 0) && (c->dim >= 3) && (c->geom[0] >= 0) && (c->geom[1] >= 0)
          && (d->efc_type[c->efc_address] == (int)mjCNSTR_CONTACT_ELLIPTIC))
             ? 1
             : 0;
}

/* friction row `e`'s spring on its slip: the stiffest its own damper holds without ringing ((B / 2)^2), at its
   impedance - as MuJoCo lays a row's reference on its violation (K I) */
static double sprung(const mjData *d, int e)
{
  const double half = 0.5 * d->efc_KBIP[(4 * e) + 1];

  return (half * half) * d->efc_KBIP[(4 * e) + 2];
}

void wbc_world_stuck(mjData *d, wbc_stuck_t *k)
{
  const int n = (d->ncon < k->cap) ? d->ncon : k->cap;
  int       from = 0;

  for (int i = 0; i < n; i++)
  {
    mjContact *c = &d->contact[i];
    double    *row = &k->row[5 * i];
    double     was[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double     least = INFINITY;
    double     bore = 0.0;   /* the nearest's normal force the last step: a contact new to the step lands unstrained */
    int        j = from;
    int        seen = 0;

    /* the last step's contacts of its two geoms, sought on from where the contact before found its own: they lie
       together, in MuJoCo's order; the nearest's slip its own */
    while ((seen < k->n) && ((k->pair[2 * j] != c->geom[0]) || (k->pair[(2 * j) + 1] != c->geom[1])))
    {
      j = ((j + 1) < k->n) ? (j + 1) : 0;
      seen++;
    }
    if (seen < k->n)
    {
      from = j;
      while ((j < k->n) && (k->pair[2 * j] == c->geom[0]) && (k->pair[(2 * j) + 1] == c->geom[1]))
      {
        const double x = c->pos[0] - k->at[3 * j];
        const double y = c->pos[1] - k->at[(3 * j) + 1];
        const double z = c->pos[2] - k->at[(3 * j) + 2];
        const double far = (x * x + y * y) + z * z;

        if (far < least)
        {
          least = far;
          bore = k->borne[j];
          for (int v = 0; v < 6; v++)
          {
            was[v] = k->slip[(6 * j) + v];
          }
        }
        j++;
      }
    }
    for (int r = 0; r < 5; r++)
    {
      row[r] = 0.0;
    }
    if (gripped(d, c) != 0)
    {
      /* the friction its geoms state at rest, falling to its sliding share - the share of the geom whose friction
         MuJoCo took - by its slip's speed along its plane, the SE(3) core's law (se3_rubs): its cone's every row
         and its regularized cone's by it, their R as laid (a row's R times its friction squared the same for all) */
      const int    g = (k->grip[3 * c->geom[1]] == c->friction[0]) ? c->geom[1] : c->geom[0];
      const int    a = c->efc_address;
      const double u2 = (d->efc_vel[a + 1] * d->efc_vel[a + 1]) + (d->efc_vel[a + 2] * d->efc_vel[a + 2]);
      const double share = se3_rubs(1.0, k->share[g], k->fade, u2);

      for (int v = 0; v < 5; v++)
      {
        c->friction[v] = c->friction[v] * share;
      }
      c->mu = c->mu * share;
      double pulls = 0.0, kept;

      for (int r = 0; r < (c->dim - 1); r++)
      {
        const double *s = &was[slip_of[r]];
        const double *t = &c->frame[slip_on[r]];
        const int     e = (a + 1) + r;
        double        pull;

        row[r] = ((s[0] * t[0] + s[1] * t[1]) + s[2] * t[2]) + d->efc_vel[e] * k->step_s;
        pull = (sprung(d, e) * row[r]) / (d->efc_R[e] * c->friction[r]);
        pulls = pulls + pull * pull;
      }
      /* the strain its rows pull on kept to the reach its load bore the last step: the SE(3) core's radial return
         before the pull (se3_kept), as its friction pass takes it - a row's reference past its cone pressed the
         contact's normal and a block sliding a slope flew 354 of its 1000 steps */
      kept = se3_kept(npmax(bore, 0.0), 0.0, pulls, 1.0, 0.0, 0.0, 1.0);
      for (int r = 0; r < (c->dim - 1); r++)
      {
        const int e = (a + 1) + r;

        d->efc_aref[e] -= sprung(d, e) * (kept * row[r]);
      }
    }
  }
}

void wbc_world_slipped(const mjData *d, wbc_stuck_t *k)
{
  const mjContact *con = d->contact;
  const int        n = (d->ncon < k->cap) ? d->ncon : k->cap;

  for (int i = 0; i < n; i++)
  {
    const mjContact *c = &con[i];
    const double    *f = c->frame;
    const double    *row = &k->row[5 * i];
    double          *slip = &k->slip[6 * i];
    double           held = 1.0;

    k->borne[i] = 0.0;
    if (gripped(d, c) != 0)
    {
      const int a = c->efc_address;
      double    pulls = 0.0;

      k->borne[i] = d->efc_force[a];

      /* its rows' pulls over their friction's, against its normal force: the SE(3) core's radial return
         (se3_kept), the friction at its speed already in the cone they were solved on */
      for (int r = 0; r < (c->dim - 1); r++)
      {
        const int    e = (a + 1) + r;
        const double pull = (sprung(d, e) * row[r]) / (d->efc_R[e] * c->friction[r]);

        pulls = pulls + pull * pull;
      }
      held = se3_kept(npmax(d->efc_force[a], 0.0), 0.0, pulls, 1.0, 0.0, 0.0, 1.0);
    }
    for (int x = 0; x < 3; x++)
    {
      slip[x] = held * (row[0] * f[3 + x] + row[1] * f[6 + x]);
      slip[3 + x] = held * ((row[2] * f[x] + row[3] * f[3 + x]) + row[4] * f[6 + x]);
      k->at[(3 * i) + x] = c->pos[x];
    }
    k->pair[2 * i] = c->geom[0];
    k->pair[(2 * i) + 1] = c->geom[1];
    k->held[i] = held;
  }
  k->n = n;
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

void wbc_world_posed(wbc_kinematics_t kin, const struct mjModel_ *m, struct mjData_ *d, int n, int nq,
                     const double *qpos, int nb, const int *bodies, int nj, const int *jnt,
                     double *xpos, double *xmat, double *axis, double *anchor)
{
  if (n <= 0 || nq <= 0 || nq > m->nq)
  {
    return;
  }
  const size_t qbytes = (size_t)nq * sizeof(double);

  for (int k = 0; k < n; k++)
  {
    memcpy(d->qpos, qpos + (size_t)k * (size_t)nq, qbytes);
    kin(m, d);
    for (int b = 0; b < nb; b++)
    {
      if (bodies[b] < 0 || bodies[b] >= m->nbody)
      {
        continue;
      }
      const size_t       bid = (size_t)bodies[b];
      const size_t       off = (size_t)k * (size_t)nb + (size_t)b;
      const double      *src_p = d->xpos + 3U * bid;
      const double      *src_m = d->xmat + 9U * bid;
      double            *dst_p = xpos + 3U * off;
      double            *dst_m = xmat + 9U * off;

      dst_p[0] = src_p[0];
      dst_p[1] = src_p[1];
      dst_p[2] = src_p[2];
      memcpy(dst_m, src_m, 9U * sizeof(double));
    }
    for (int j = 0; j < nj; j++)
    {
      if (jnt[j] < 0 || jnt[j] >= m->njnt)
      {
        continue;
      }
      const size_t  jid = (size_t)jnt[j];
      const size_t  off = (size_t)k * (size_t)nj + (size_t)j;
      const double *src_ax = d->xaxis + 3U * jid;
      const double *src_an = d->xanchor + 3U * jid;
      double       *dst_ax = axis + 3U * off;
      double       *dst_an = anchor + 3U * off;

      dst_ax[0] = src_ax[0];
      dst_ax[1] = src_ax[1];
      dst_ax[2] = src_ax[2];
      dst_an[0] = src_an[0];
      dst_an[1] = src_an[1];
      dst_an[2] = src_an[2];
    }
  }
}
