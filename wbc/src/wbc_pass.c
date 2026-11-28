/** wbc_pass.c - a pass of the robot's world on the host, its arrays laid once (wbc.h): the robot's drives told as
    machine.law.drives tells them, its world's torques and their meters as
    machine.world.physics steps them, its IMU, encoders, soles and skin as machine.senses.edge
    reads them, its drives' heat as machine.body.heat keeps it - numpy's arithmetic operation
    for operation, its clip and its pairwise sum, so a pass is the same bit for bit. Portable C11. */
#include "wbc_pass.h"
#include "wbc_np.h"
#include "wbc_libm_arc.h"
#include "board_units.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void wbc_pass_drives(wbc_drives_t *p, double dt, double wo, double wo3, double ys, double stood0, double stood1,
                     int in_hop, int wb)
{
  const int    n = p->n;
  const double c3 = 3.0 * wo, c33 = (3.0 * wo) * wo;
  const double leak = npmin(1.0, dt / npmax(1e-6, p->leak_s * ys));
  const double bleed = npmin(1.0, leak * (1.0 - ys));
  const double under = in_hop ? 0.0 : 0.02, pull = in_hop ? 0.0 : 35.0;
  const double stood[2] = {stood0, stood1};
  double      *eso0 = p->eso, *eso1 = p->eso + n, *eso2 = p->eso + 2 * n;

  double *restrict sb = p->sb;

  for (int i = 0; i < n; i++)
  {
    sb[i] = 0.0;
  }
  /* the robot's knees' spring: back from straight, and against folding back past straight (engaged ahead where
     a standing knee's stopping point crosses `under` over a held stride) */
  for (int k = 0; k < 2; k++)
  {
    const int    j = p->knee[k];
    const double qk = p->y[p->q_at + p->joints[j]], qvk = p->y[p->qd_at + p->joints[j]];
    const double qk_stop = qk + (stood[k] * p->stop_reach) * npmin(0.0, qvk);
    const double ramp = (p->stop_width > 0.0) ? npmax((0.20 - qk) / 0.15, (under - qk_stop) / p->stop_width)
                                              : (0.20 - qk) / 0.15;

    p->sb[j] = npclip(((p->knee_nm * npclip(1.0 - qk / p->knee_soft, 0.0, 3.0) + 2400.0 * npmax(0.0, under - qk))
                       - (pull * npclip(ramp, 0.0, 1.5)) * npmin(0.0, qvk)) * ys,
                      -p->clamp_j[j], p->clamp_j[j]);
  }
  for (int i = 0; i < n; i++)
  {
    const int    jo = p->joints[i];
    const double qm = p->y[p->qm_at + jo], qpos = p->y[p->q_at + jo], qvel = p->y[p->qd_at + jo];
    const double b_pas = (1.0 - ys) * 25.0 * npmin(p->jeff[i], 0.6);
    double       qa = p->qacc[p->act[i]] * ys, asked, e;

    if (!in_hop)
    {
      qa = npclip(qa, -250.0, 250.0);
    }
    asked = qa;
    qa = wbc_bus_ask(p->bus, i, dt, qa);
    /* the observer on the motor's encoder (link encoder at SEA closed-loop bandwidth on active SEAs);
       do not integrate ask into a mechanical hard stop where the link cannot accelerate past it */
    const double sw = (p->sea != NULL) ? p->sea[i] : 0.0;
    const double q_obs = (sw > 0.0) ? qpos : qm;
    const double wi = (sw > 0.0) ? (wo * sw) : wo;
    const double c3i = (sw > 0.0) ? (3.0 * wi) : c3;
    const double c33i = (sw > 0.0) ? ((3.0 * wi) * wi) : c33;
    const double wo3i = (sw > 0.0) ? ((wi * wi) * wi) : wo3;
    const int    at_lo = (sw > 0.0) && (qpos <= p->q_lo[i]);
    const int    at_hi = (sw > 0.0) && (qpos >= p->q_hi[i]);
    const int    at_stop = (at_lo && qa < 0.0) || (at_hi && qa > 0.0);
    const double qa_eso = at_stop ? 0.0 : qa;

    e = q_obs - eso0[i];
    eso0[i] += dt * (eso1[i] + c3i * e);
    eso1[i] += dt * (((qa_eso + (p->sb[i] + p->bus->given[i]) / p->jeff[i]) + eso2[i]) + c33i * e);
    if (!at_lo && !at_hi)
    {
      eso2[i] += dt * (wo3i * e);
    }
    if (!in_hop || sw > 0.0)
    {
      const double z_lim = (0.35 * p->clamp[i]) / npmax(0.05, p->jeff[i]);

      eso0[i] = npclip(eso0[i], q_obs - 0.08, q_obs + 0.08);
      eso2[i] = npclip(eso2[i], -z_lim, z_lim);
    }
    eso1[i] *= 1.0 - bleed;
    eso2[i] *= 1.0 - bleed;
    p->hold[i] = p->over[i] ? p->wep_hold_s * ys : npmax(0.0, p->hold[i] * (1.0 - bleed) - dt);
    /* the setpoint carried on by the ask, leaking to the encoder */
    p->ref_v[i] = (1.0 - leak) * (p->ref_v[i] + qa * dt) + leak * ys * qvel;
    if (!in_hop)
    {
      p->ref_v[i] = npclip(p->ref_v[i], -18.0, 18.0);
    }
    p->ref_q[i] += (p->ref_v[i] * dt) + leak * (qpos - p->ref_q[i]);
    if (in_hop)
    {
      if (i == p->knee[0] || i == p->knee[1])
      {
        p->ref_q[i] = npmax(0.0, p->ref_q[i]);
      }
      if (p->cervical[0] >= 0 && p->cervical[1] >= 0)
      {
        if (i == p->cervical[0])
        {
          p->ref_q[i] = npclip(p->ref_q[i], -0.35, 0.45);
        }
        if (i == p->cervical[1])
        {
          p->ref_q[i] = npclip(p->ref_q[i], -0.85, 0.85);
        }
      }
    }
    else
    {
      if ((p->ref_q[i] <= p->q_lo[i] && p->ref_v[i] < 0.0) || (p->ref_q[i] >= p->q_hi[i] && p->ref_v[i] > 0.0))
      {
        p->ref_v[i] = 0.0;
      }
      p->ref_q[i] = npclip(p->ref_q[i], p->q_lo[i], p->q_hi[i]);
    }
    /* less its inertia's share of what the jerk's clip cut: its observer, told the clipped ask,
       pushed against the rest - a jamb's touch yielded the robot's arm 0.24 rad where 0.55 */
    p->feed[i] = (((wbc_bus_slew(p->bus, i, dt, p->tau[i] * ys) + p->sb[i]) - p->jeff[i] * eso2[i]) - b_pas * qvel)
                 - p->jeff[i] * (asked - qa);
    p->limit[i] = (p->hold[i] > 0.0 ? (wb ? p->wep_lim[i] : p->wep[i]) : p->clamp_j[i]) * p->release[i];
  }
  /* the master bus insert (wbc_bus.c): the robot's torque command through the tank, the zero crossing, the
   * slew */
  for (int i = 0; i < n; i++)
  {
    p->feed[i] = wbc_bus_feed(p->bus_k, p->bus, i, dt, p->y[p->qd_at + p->joints[i]], p->feed[i],
                              p->jeff[i] * p->posture_kd);
  }
  wbc_bus_pass(p->bus_k, p->bus, dt);
}

void wbc_pass_tick(wbc_tick_t *t, double dt, double now, double wo, double limp)
{
  wbc_drives_t *p = t->drives;
  const int     n = p->n;
  double        held = 0.0;

  /* the robot's holds read before the step spends them: 1 where any is not none (`hold.any()`) */
  for (int i = 0; i < n; i++)
  {
    held = (p->hold[i] != 0.0) ? 1.0 : held;
  }
  const double ys = wbc_imp_surety(t->imp, t->field, t->surety, dt, NAN, now, t->vec);
  const double drv = 1.0 - wbc_cube(1.0 - ys), wo_s = wo * drv, rest = 1.0 - limp;
  const double k_to = t->field->st[t->field->k_to], k_from = t->field->st[t->field->k_from];
  const double k = (k_from > k_to) ? k_from : k_to;
  const double striding = t->field->a[t->imp->a_strides] * npclip(t->field->out[t->field->held_out], 0.0, 1.0);
  const double st0 = t->field->a[t->imp->a_stance], st1 = t->field->a[t->imp->a_stance + 1];
  const double stood0 = (st0 * (1.0 - st1)) * striding;
  const double stood1 = (st1 * (1.0 - st0)) * striding;
  double       left;

  t->surety = ys;
  wbc_pass_drives(p, dt, wo_s, wbc_cube(wo_s), drv, stood0, stood1, (k > t->hop_k) ? 1 : 0, 1);
  {
    double *restrict       rq = p->ref_q, *restrict lim = t->limit;
    const double *restrict rng = t->rng, *restrict kept = t->kept;

    for (int i = 0; i < n; i++)
    {
      rq[i] = npminimum(npmaximum(rq[i], rng[2 * i]), rng[2 * i + 1]);
      lim[i] = kept[i] * rest;
    }
  }
  left = t->left + dt * (t->wep_fill * (1.0 - held) - held);
  left = (left > 0.0) ? left : 0.0;
  t->left = (left < t->wep_s) ? left : t->wep_s;
}

void wbc_pass_ctrl(wbc_world_t *w, double span, const double *ahead, const double *wrate)
{
  const int                     n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  const double *restrict        target = w->target, *restrict rate = w->rate, *restrict delta = w->delta;
  const double *restrict        zero = w->zero, *restrict gains = w->gains, *restrict feed = w->feed;
  const double *restrict        limit = w->limit, *restrict qpos = w->qpos, *restrict qvel = w->qvel;
  const int *restrict           qadr = w->qadr, *restrict vadr = w->vadr;
  const unsigned char *restrict shorted = w->shorted, *restrict cut = w->cut;
  double *restrict              ctrl = w->ctrl;
  const double                  wound = w->wound;
  _Alignas(64) double           tau[WBC_PASS_MAX], qp[WBC_PASS_MAX], qv[WBC_PASS_MAX];
  _Alignas(64) double           ah[WBC_PASS_MAX], wr[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    qp[i] = qpos[qadr[i]];
    qv[i] = qvel[vadr[i]];
  }
  if (ahead != NULL)
  {
    const double *restrict src = ahead;

    for (int i = 0; i < n; i++)
    {
      ah[i] = src[i];
    }
  }
  else
  {
    for (int i = 0; i < n; i++)
    {
      ah[i] = 0.0;
    }
  }
  if (wrate != NULL)
  {
    const double *restrict src = wrate;

    for (int i = 0; i < n; i++)
    {
      wr[i] = src[i];
    }
  }
  else
  {
    for (int i = 0; i < n; i++)
    {
      wr[i] = 0.0;
    }
  }
  for (int i = 0; i < n; i++)
  {
    const double ref = target[i] + rate[i] * span + ah[i];
    const double seen = (qp[i] + wound * delta[i]) + zero[i];
    const double seen_d = qv[i] + wr[i];
    double       t = (gains[2 * i] * (ref - seen) + gains[2 * i + 1] * (rate[i] - seen_d)) + feed[i];

    t = ((shorted[i] | cut[i]) != 0U) ? 0.0 : t;
    tau[i] = t;
    ctrl[i] = npclip(t, -limit[i], limit[i]);
  }
  /* a parallel pair's two identical drives: each's current the first's share plus or less the second's,
     clamped to each joint's limit and to the pair's symmetric per-motor current ceiling */
  for (int k = 0; k < w->npairs; k++)
  {
    const int    p = w->pairs[2 * k], r = w->pairs[2 * k + 1];
    const double top_p = w->limit[p] / w->kt[p], top_r = w->limit[r] / w->kt[r];
    const double top = (top_p > 0.0 && top_r > 0.0) ? npmax(top_p, top_r) : 0.0;
    const double a = npclip(tau[p] / w->kt[p], -top_p, top_p), b = npclip(tau[r] / w->kt[r], -top_r, top_r);
    const double one = npclip(a + b, -top, top), two = npclip(a - b, -top, top);

    ctrl[p] = w->kt[p] * (one + two) / 2.0;
    ctrl[r] = w->kt[r] * (one - two) / 2.0;
  }
}

void wbc_pass_motor(wbc_world_t *w, double step_s)
{
  const int              n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  double *restrict       motor = w->motor;
  const double *restrict ctrl = w->ctrl;
  _Alignas(64) double    held[WBC_PASS_MAX], squared[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    const double m = ctrl[i];

    motor[i] = m;
    held[i] = fabs(m);
    squared[i] = m * m;
  }
  w->effort += npsum(held, n) * step_s;
  w->heat += wbc_dot(w->blas, w->loss, 1, squared, 1, n) * step_s;
}

void wbc_pass_metered(wbc_world_t *w, const double *speed, double step_s)
{
  const int n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  double    pos[WBC_PASS_MAX], neg[WBC_PASS_MAX];
  int       np_ = 0, nn = 0;

  for (int i = 0; i < n; i++)
  {
    const double sp = speed != NULL ? speed[i] : w->qvel[w->vadr[i]];
    const double power = w->motor[i] * sp;

    if (power > 0.0)
    {
      pos[np_++] = power;
    }
    if (power < 0.0)
    {
      neg[nn++] = power;
    }
    w->geared[i] = npmax(w->geared[i], fabs(w->ctrl[i] - w->armature[i] * w->qacc[w->vadr[i]]));
  }
  w->work += npsum(pos, np_) * step_s;
  w->brake -= npsum(neg, nn) * step_s;
}

int wbc_pass_paired(const unsigned char *geom, const unsigned char *pos, ptrdiff_t stride, int ncon,
                    const unsigned char *a, const unsigned char *b, double below)
{
  int paired = 0;

  for (int i = 0; i < ncon; i++)
  {
    int    g[2];
    double p[3];

    memcpy(g, geom + i * stride, sizeof g);
    memcpy(p, pos + i * stride, sizeof p);
    paired |= ((a[g[0]] & b[g[1]]) | (a[g[1]] & b[g[0]])) & (p[1] < below);
  }
  return paired;
}

void wbc_pass_averaged(const wbc_world_t *w, double k, double *restrict torque, double *restrict power)
{
  const int              n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  const double *restrict ctrl = w->ctrl, *restrict qvel = w->qvel;
  const int *restrict    vadr = w->vadr;

  for (int i = 0; i < n; i++)
  {
    torque[i] += k * (ctrl[i] - torque[i]);
    power[i] += k * (ctrl[i] * qvel[vadr[i]] - power[i]);
  }
}

double wbc_pass_drawn(const wbc_world_t *w, const double *tau, const double *speed, const double *loss,
                      const unsigned char *driven, const double *testing, const double *ohm, double housekeeping_w,
                      double switching_w)
{
  const int n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  double    pos[WBC_PASS_MAX], copper = 0.0, check = 0.0;
  int       driven_n = 0, on_n = 0;

  for (int i = 0; i < n; i++)
  {
    const double sp = speed != NULL ? speed[i] : w->qvel[w->vadr[i]];

    pos[i] = npmax(tau[i] * sp, 0.0);
    copper += loss[i] * (tau[i] * tau[i]);
    check += ohm[i] * (testing[i] * testing[i]);
    driven_n += driven[i] ? 1 : 0;
    on_n += ((driven[i] && !w->shorted[i] && !w->cut[i]) || testing[i] > 0.0) ? 1 : 0;
  }
  return ((npsum(pos, n) + copper) + housekeeping_w * (double)driven_n + switching_w * (double)on_n) + check;
}

void wbc_pass_felt(const wbc_world_t *w, double *y, int quat_at, int gyro_at, int accel_at, int q_at, int qd_at,
                   int qm_at)
{
  static const double    g[3] = {0.0, -WBC_GRAVITY, 0.0};
  const int              n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  const double          *r = w->xmat + 9 * w->pelvis;
  const double *restrict qpos = w->qpos, *restrict qvel = w->qvel, *restrict delta = w->delta, *restrict zero = w->zero;
  const int *restrict    qadr = w->qadr, *restrict vadr = w->vadr;
  double *restrict       y_q = y + q_at, *restrict y_qd = y + qd_at, *restrict y_qm = y + qm_at;
  const double           wound = w->wound;
  _Alignas(64) double    qp[WBC_PASS_MAX], qv[WBC_PASS_MAX];
  double                 v[3];

  for (int i = 0; i < 4; i++)
  {
    y[quat_at + i] = qpos[3 + i];
  }
  for (int i = 0; i < 3; i++)
  {
    y[gyro_at + i] = qvel[3 + i];
    v[i] = w->qacc[i] - g[i];
  }
  for (int i = 0; i < 3; i++)
  {
    y[accel_at + i] = (r[i] * v[0] + r[3 + i] * v[1]) + r[6 + i] * v[2];
  }
  for (int i = 0; i < n; i++)
  {
    qp[i] = qpos[qadr[i]];
    qv[i] = qvel[vadr[i]];
  }
  for (int i = 0; i < n; i++)
  {
    const double q = qp[i];

    y_q[i] = q;
    y_qd[i] = qv[i];
    y_qm[i] = (q + wound * delta[i]) + zero[i];
  }
}

void wbc_pass_contacts(wbc_contacts_t *c, int ncon, const char *pos, const double *efc_force, const double *efc_R)
{
  {
    const int        ns3 = 3 * c->nskin, ns = c->nskin, ni3 = 3 * c->ninsoles, ni = c->ninsoles, nf2 = 2 * c->nfeet;
    double *restrict touch = c->touch, *restrict felt_at = c->felt_at, *restrict felt_nrm = c->felt_nrm;
    double *restrict felt_n = c->felt_n, *restrict felt_give = c->felt_give;
    double *restrict at = c->at, *restrict pressed = c->pressed, *restrict cells = c->cells;

    c->loads[0] = 0.0;
    c->loads[1] = 0.0;
    for (int k = 0; k < ns3; k++)
    {
      touch[k] = 0.0;
      felt_at[k] = 0.0;
      felt_nrm[k] = 0.0;
    }
    for (int k = 0; k < ns; k++)
    {
      felt_n[k] = 0.0;
      felt_give[k] = 0.0;
    }
    for (int k = 0; k < ni3; k++)
    {
      at[k] = 0.0;
    }
    for (int k = 0; k < ni; k++)
    {
      pressed[k] = 0.0;
    }
    for (int k = 0; k < nf2; k++)
    {
      cells[k] = 0.0;
    }
  }
  for (int i = 0; i < ncon; i++)
  {
    const char   *row = pos + (size_t)i * (size_t)c->stride;
    const double *cpos = (const double *)row;
    const int    *geom = (const int *)(row + c->off_geom);
    const double *frame = (const double *)(row + c->off_frame);
    const int     dim = *(const int *)(row + c->off_dim), adr = *(const int *)(row + c->off_efc);
    const int     b0 = c->gb[geom[0]], b1 = c->gb[geom[1]];
    const int     h0 = c->robot_own[b0], h1 = c->robot_own[b1];
    const int     mine = h0 ? b0 : b1;
    const double  sign = h0 ? -1.0 : 1.0;
    const double *r = c->xmat + 9 * mine, *xp = c->xpos + 3 * mine;
    double        force[3], rel[3], f0, af0;
    int           k;

    /* a contact between a body of the robot's own and one not; its force its rows of the solver's
     * */
    if (h0 == h1)
    {
      continue;
    }
    for (k = 0; k < 3; k++)
    {
      force[k] = (adr >= 0 && dim > k) ? efc_force[adr + k] : 0.0;
      rel[k] = cpos[k] - xp[k];
    }
    f0 = force[0];
    af0 = fabs(f0);
    if (c->sole_side[mine] >= 0)
    {
      c->loads[c->sole_side[mine]] += f0;
    }
    k = c->insole[mine];
    if (k >= 0)
    {
      const int f_id = c->foot[mine];

      c->pressed[k] += f0;
      for (int j = 0; j < 3; j++)
      {
        c->at[3 * k + j] += ((f0 * r[j]) * rel[0] + (f0 * r[3 + j]) * rel[1]) + (f0 * r[6 + j]) * rel[2];
      }
      if (f_id >= 0)
      {
        const double along = (r[2] * rel[0] + r[5] * rel[1]) + r[8] * rel[2];

        c->cells[2 * f_id + (along >= c->middle_m ? 1 : 0)] += f0;
      }
    }
    k = c->skin[mine];
    if (k >= 0)
    {
      double push[3];

      for (int j = 0; j < 3; j++)
      {
        push[j] = ((frame[j] * force[0] + frame[3 + j] * force[1]) + frame[6 + j] * force[2]) * sign;
      }
      for (int j = 0; j < 3; j++)
      {
        c->touch[3 * k + j] += (r[j] * push[0] + r[3 + j] * push[1]) + r[6 + j] * push[2];
        c->felt_at[3 * k + j] += ((af0 * r[j]) * rel[0] + (af0 * r[3 + j]) * rel[1]) + (af0 * r[6 + j]) * rel[2];
        /* the contact's normal into the robot (MuJoCo's frame's first, from the first geom to the second), its
         * frame */
        c->felt_nrm[3 * k + j] += af0 * (((r[j] * frame[0] + r[3 + j] * frame[1]) + r[6 + j] * frame[2]) * sign);
      }
      c->felt_n[k] += af0;
      /* its support's give: the solver's impedance on the contact's normal row (a_n = a_ref - R f_n: what a force
         past what is felt costs in acceleration into or out of the support, m/s^2 a N), its force's share - a
         chair's small, never none; at none a plan split a seat's load as it pleased at a_n = 0 */
      c->felt_give[k] += af0 * (adr >= 0 ? efc_R[adr] : 0.0);
    }
  }
}

int wbc_pass_tapped(const wbc_contacts_t *c, int ncon, const char *pos, const double *efc_force,
                    const double *efc_vel, const int *at, int width, int cap, double *out)
{
  int n = 0;

  for (int i = 0; i < ncon; i++)
  {
    const char *row = pos + (size_t)i * (size_t)c->stride;
    const int  *geom = (const int *)(row + c->off_geom);
    const int   dim = *(const int *)(row + c->off_dim), adr = *(const int *)(row + c->off_efc);
    const int   h0 = c->robot_own[c->gb[geom[0]]], h1 = c->robot_own[c->gb[geom[1]]];
    double     *x = out + (size_t)n * (size_t)width;

    if (h0 == h1 || adr < 0)
    {
      continue;
    }
    if (n < cap)
    {
      x[at[0]] = h0 ? 1.0 : 0.0;
      memcpy(x + at[1], row, 3 * sizeof(double));
      memcpy(x + at[2], row + c->off_frame, 9 * sizeof(double));
      for (int k = 0; k < 3; k++)
      {
        x[at[3] + k] = dim > k ? efc_force[adr + k] : 0.0;
      }
      x[at[4]] = *(const double *)(row + c->off_friction);
      for (int k = 0; k < 2; k++)
      {
        x[at[5] + k] = dim > k + 1 ? efc_vel[adr + 1 + k] : 0.0;
      }
    }
    n++;
  }
  return n;
}

void wbc_pass_sensed(const wbc_contacts_t *c, double *y, int load_at, int touch_at, int press_at, int cop_at,
                     int cells_at, int at_at, int give_at, int nrm_at)
{
  const int              ns = c->nskin, ni = c->ninsoles, nf2 = 2 * c->nfeet;
  const double *restrict touch = c->touch, *restrict felt_at = c->felt_at, *restrict felt_nrm = c->felt_nrm;
  const double *restrict felt_n = c->felt_n, *restrict felt_give = c->felt_give;
  const double *restrict at = c->at, *restrict pressed = c->pressed, *restrict cells = c->cells;
  double *restrict       y_touch = y + touch_at, *restrict y_at = y + at_at, *restrict y_nrm = y + nrm_at;
  double *restrict       y_give = y + give_at, *restrict y_press = y + press_at, *restrict y_cop = y + cop_at;
  double *restrict       y_cells = y + cells_at;

  y[load_at] = c->loads[0];
  y[load_at + 1] = c->loads[1];
  for (int k = 0; k < ns; k++)
  {
    const double den = npmax(felt_n[k], 1e-9);

    for (int j = 0; j < 3; j++)
    {
      y_touch[3 * k + j] = touch[3 * k + j];
      y_at[3 * k + j] = felt_at[3 * k + j] / den;
      y_nrm[3 * k + j] = felt_nrm[3 * k + j] / den;
    }
    y_give[k] = felt_give[k] / den;
  }
  for (int k = 0; k < ni; k++)
  {
    const double p = pressed[k], den = npmax(p, 1e-9);

    y_press[k] = p;
    for (int j = 0; j < 3; j++)
    {
      y_cop[3 * k + j] = at[3 * k + j] / den;
    }
  }
  for (int k = 0; k < nf2; k++)
  {
    y_cells[k] = cells[k];
  }
}
