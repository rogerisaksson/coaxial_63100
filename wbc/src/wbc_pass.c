/** wbc_pass.c - a pass of her world on the host, its arrays laid once (wbc.h): her drives told as
    machine.law.drives tells them, her world's torques and their meters as machine.physics steps
    them, her IMU, encoders, soles and skin as machine.edge reads them, her drives' heat as
    machine.heat keeps it - numpy's arithmetic operation for operation, its clip and its pairwise
    sum, so a pass is the same bit for bit. Portable C11. */
#include "wbc_pass.h"
#include "wbc_np.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void wbc_pass_drives(wbc_drives_t *p, double dt, double wo, double wo3, double ys, int in_hop, int wb)
{
  const int    n = p->n;
  const double c3 = 3.0 * wo, c33 = (3.0 * wo) * wo;
  const double leak = npmin(1.0, dt / npmax(1e-6, p->leak_s * ys));
  const double bleed = npmin(1.0, leak * (1.0 - ys));
  const double under = in_hop ? 0.0 : 0.02, pull = in_hop ? 0.0 : 35.0;
  double      *eso0 = p->eso, *eso1 = p->eso + n, *eso2 = p->eso + 2 * n;

  for (int i = 0; i < n; i++)
  {
    p->sb[i] = 0.0;
  }
  /* her knees' spring: back from straight, and against folding back past it */
  for (int k = 0; k < 2; k++)
  {
    const int    j = p->knee[k];
    const double qk = p->y[p->q_at + p->joints[j]], qvk = p->y[p->qd_at + p->joints[j]];

    p->sb[j] = npclip(((p->knee_nm * npclip(1.0 - qk / p->knee_soft, 0.0, 3.0) + 2400.0 * npmax(0.0, under - qk))
                       - (pull * npclip((0.20 - qk) / 0.15, 0.0, 1.5)) * npmin(0.0, qvk)) * ys,
                      -p->clamp_j[j], p->clamp_j[j]);
  }
  for (int i = 0; i < n; i++)
  {
    const int    jo = p->joints[i];
    const double qm = p->y[p->qm_at + jo], qpos = p->y[p->q_at + jo], qvel = p->y[p->qd_at + jo];
    const double b_pas = (1.0 - ys) * 25.0 * npmin(p->jeff[i], 0.6);
    double       qa = p->qacc[p->act[i]] * ys, e;

    if (!in_hop)
    {
      qa = npclip(qa, -250.0, 250.0);
    }
    /* the observer on the motor's encoder: angle, rate, what else moves it */
    e = qm - eso0[i];
    eso0[i] += dt * (eso1[i] + c3 * e);
    eso1[i] += dt * (((qa + p->sb[i] / p->jeff[i]) + eso2[i]) + c33 * e);
    eso2[i] += dt * (wo3 * e);
    if (!in_hop)
    {
      const double z_lim = (0.35 * p->clamp[i]) / npmax(0.05, p->jeff[i]);

      eso0[i] = npclip(eso0[i], qm - 0.08, qm + 0.08);
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
    p->feed[i] = ((p->tau[i] * ys + p->sb[i]) - p->jeff[i] * eso2[i]) - b_pas * qvel;
    p->limit[i] = p->hold[i] > 0.0 ? (wb ? p->wep_lim[i] : p->wep[i]) : p->clamp_j[i];
  }
}

void wbc_pass_ctrl(wbc_world_t *w, double span, const double *ahead, const double *wrate)
{
  const int n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  double    tau[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    double ref = w->target[i] + w->rate[i] * span;
    double seen = (w->qpos[w->qadr[i]] + w->wound * w->delta[i]) + w->zero[i];
    double seen_d = w->qvel[w->vadr[i]];
    double t;

    if (ahead != NULL)
    {
      ref = ref + ahead[i];
    }
    if (wrate != NULL)
    {
      seen_d = seen_d + wrate[i];
    }
    t = (w->gains[2 * i] * (ref - seen) + w->gains[2 * i + 1] * (w->rate[i] - seen_d)) + w->feed[i];
    if (w->shorted[i] || w->cut[i])
    {
      t = 0.0;
    }
    tau[i] = t;
    w->ctrl[i] = npclip(t, -w->limit[i], w->limit[i]);
  }
  /* a parallel pair's two drives: each's current the first's share plus or less the second's */
  for (int k = 0; k < w->npairs; k++)
  {
    const int    p = w->pairs[2 * k], r = w->pairs[2 * k + 1];
    const double a = tau[p] / w->kt[p], b = tau[r] / w->kt[r];
    const double top_p = w->limit[p] / w->kt[p], top_r = w->limit[r] / w->kt[r];
    const double one = npclip(a + b, -top_p, top_p), two = npclip(a - b, -top_r, top_r);

    w->ctrl[p] = w->kt[p] * (one + two) / 2.0;
    w->ctrl[r] = w->kt[r] * (one - two) / 2.0;
  }
}

void wbc_pass_motor(wbc_world_t *w, double step_s)
{
  const int n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;
  double    held[WBC_PASS_MAX], squared[WBC_PASS_MAX];

  for (int i = 0; i < n; i++)
  {
    w->motor[i] = w->ctrl[i];
    held[i] = fabs(w->motor[i]);
    squared[i] = w->motor[i] * w->motor[i];
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

int wbc_pass_paired(const unsigned char *geom, ptrdiff_t stride, int ncon, const unsigned char *a,
                    const unsigned char *b)
{
  int paired = 0;

  for (int i = 0; i < ncon; i++)
  {
    int g[2];

    memcpy(g, geom + i * stride, sizeof g);
    paired |= (a[g[0]] & b[g[1]]) | (a[g[1]] & b[g[0]]);
  }
  return paired;
}

void wbc_pass_averaged(const wbc_world_t *w, double k, double *torque, double *power)
{
  const int n = w->n < WBC_PASS_MAX ? w->n : WBC_PASS_MAX;

  for (int i = 0; i < n; i++)
  {
    torque[i] += k * (w->ctrl[i] - torque[i]);
    power[i] += k * (w->ctrl[i] * w->qvel[w->vadr[i]] - power[i]);
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
  static const double g[3] = {0.0, -9.81, 0.0};
  const double       *r = w->xmat + 9 * w->pelvis;
  double              v[3];

  for (int i = 0; i < 4; i++)
  {
    y[quat_at + i] = w->qpos[3 + i];
  }
  for (int i = 0; i < 3; i++)
  {
    y[gyro_at + i] = w->qvel[3 + i];
    v[i] = w->qacc[i] - g[i];
  }
  for (int i = 0; i < 3; i++)
  {
    y[accel_at + i] = (r[i] * v[0] + r[3 + i] * v[1]) + r[6 + i] * v[2];
  }
  for (int i = 0; i < w->n; i++)
  {
    const double q = w->qpos[w->qadr[i]];

    y[q_at + i] = q;
    y[qd_at + i] = w->qvel[w->vadr[i]];
    y[qm_at + i] = (q + w->wound * w->delta[i]) + w->zero[i];
  }
}

void wbc_pass_contacts(wbc_contacts_t *c, int ncon, const char *pos, const double *efc_force)
{
  for (int s = 0; s < 2; s++)
  {
    c->loads[s] = 0.0;
  }
  for (int k = 0; k < 3 * c->nskin; k++)
  {
    c->touch[k] = c->felt_at[k] = 0.0;
  }
  for (int k = 0; k < c->nskin; k++)
  {
    c->felt_n[k] = 0.0;
  }
  for (int k = 0; k < 3 * c->ninsoles; k++)
  {
    c->at[k] = 0.0;
  }
  for (int k = 0; k < c->ninsoles; k++)
  {
    c->pressed[k] = 0.0;
  }
  for (int k = 0; k < 2 * c->nfeet; k++)
  {
    c->cells[k] = 0.0;
  }
  for (int i = 0; i < ncon; i++)
  {
    const char   *row = pos + (size_t)i * (size_t)c->stride;
    const double *cpos = (const double *)row;
    const int    *geom = (const int *)(row + c->off_geom);
    const double *frame = (const double *)(row + c->off_frame);
    const int     dim = *(const int *)(row + c->off_dim), adr = *(const int *)(row + c->off_efc);
    const int     b0 = c->gb[geom[0]], b1 = c->gb[geom[1]];
    const int     h0 = c->hers[b0], h1 = c->hers[b1];
    const int     mine = h0 ? b0 : b1;
    const double  sign = h0 ? -1.0 : 1.0;
    const double *r = c->xmat + 9 * mine, *xp = c->xpos + 3 * mine;
    double        force[3], rel[3], f0, af0;
    int           k;

    /* a contact between a body of hers and one not; its force its rows of the solver's */
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
      }
      c->felt_n[k] += af0;
    }
  }
}

void wbc_pass_sensed(const wbc_contacts_t *c, double *y, int load_at, int touch_at, int press_at, int cop_at,
                     int cells_at, int at_at)
{
  for (int s = 0; s < 2; s++)
  {
    y[load_at + s] = c->loads[s];
  }
  for (int k = 0; k < c->nskin; k++)
  {
    for (int j = 0; j < 3; j++)
    {
      y[touch_at + 3 * k + j] = c->touch[3 * k + j];
      y[at_at + 3 * k + j] = c->felt_at[3 * k + j] / npmax(c->felt_n[k], 1e-9);
    }
  }
  for (int k = 0; k < c->ninsoles; k++)
  {
    y[press_at + k] = c->pressed[k];
    for (int j = 0; j < 3; j++)
    {
      y[cop_at + 3 * k + j] = c->at[3 * k + j] / npmax(c->pressed[k], 1e-9);
    }
  }
  for (int k = 0; k < 2 * c->nfeet; k++)
  {
    y[cells_at + k] = c->cells[k];
  }
}

void wbc_pass_heat_load(wbc_heat_t *h, const double *torques, const double *qd)
{
  for (int i = 0; i < h->n; i++)
  {
    const double amps = h->active[i] ? torques[i] / h->kt[i] : 0.0, w = h->active[i] ? fabs(qd[i]) : 0.0;

    h->sq[i] = amps * amps;
    h->ticks[i] = h->active[i] ? 1 : 0;
    h->limb_rad_s[i] = w;
    h->krpm[i] = w * h->ratio_k[i];
  }
}

void wbc_pass_heat(wbc_heat_t *h, double dt, const double *air, const double *rds, int envelope)
{
  const double hh = dt * h->haste, amb = h->ambient_c;
  const int    sub_n = (int)(hh / 0.025) + 1 > 1 ? (int)(hh / 0.025) + 1 : 1;
  const double sub_h = hh / (double)sub_n;

  h->at += hh;
  for (int i = 0; i < h->n; i++)
  {
    double      *t = h->t + 4 * i;
    const double ticks = (double)h->ticks[i];
    const double sq = ticks > 0.0 ? h->sq[i] / npmax(1.0, ticks) : 0.0;
    const double board = sq * (h->scale[i] * h->scale[i]);
    const double dt_sw = npmax(0.0, t[0] - amb);
    const double rds_k = rds != NULL ? rds[i] : (1.0 + h->rds_alpha * dt_sw) + (h->rds_beta * dt_sw) * dt_sw;
    const double air_k = air != NULL ? air[i] : (1.0 + 0.35 * npmin(4.0, h->limb_rad_s[i])) / h->cloth_air[i];
    const double rot_for = 1.0 + 0.55 * sqrt(h->krpm[i]);
    const double fet = (0.5 * board) * h->rds_ohm * rds_k;
    const double sw_on = (h->gates[i] && !h->shorted[i]) ? 1.0 : 0.0;
    const double p_sw = 3.0 * fet + h->switching_w * sw_on;
    const double p_lam = (1.5 * board) * h->shunt_ohm + h->house_w[i];
    const double p_win = (h->r[i] * sq) * (1.0 + 0.00393 * npmax(0.0, t[3] - amb));
    const double g_la = (0.35 * air_k) / h->lam_k_w[i];
    const double g_sa = ((0.65 * air_k) * (1.0 + 0.25 * (rot_for - 1.0))) / h->lam_k_w[i];
    const double g_wa = (air_k * rot_for) / h->win_k_w[i];
    const double *cap4 = h->capacity + 4 * i;
    double       net[4] = {p_sw, p_sw, p_sw, p_sw}, die, cap, used[4], spent;
    int          worst = 0;

    h->sq[i] = 0.0;
    h->ticks[i] = 0;
    for (int s = 0; s < sub_n; s++)
    {
      const double into = (t[0] - t[1]) / h->into_k_w;
      const double q_ls = (t[1] - t[2]) * h->g_ls[i], q_ws = (t[3] - t[2]) * h->g_ws[i];

      net[0] = p_sw - into;
      net[1] = ((p_lam + into) - q_ls) - (t[1] - amb) * g_la;
      net[2] = (q_ls + q_ws) - (t[2] - amb) * g_sa;
      net[3] = (p_win - q_ws) - (t[3] - amb) * g_wa;
      for (int k = 0; k < 4; k++)
      {
        t[k] += (sub_h * net[k]) / cap4[k];
      }
    }
    die = t[0] + fet * h->rth_jc;
    h->die[i] = die;
    /* the envelope: each node's span spent, its hold inside the lookahead counted as spent */
    cap = npmin(1.0, h->cap[i] + (h->at - h->cap_at[i]) * h->trip_recover_per_s);
    for (int k = 0; k < 4; k++)
    {
      const double limit = amb + cap * (h->ceiling[k] - amb);
      const int    pos = net[k] > 0.0;
      const double hold = pos ? npmax(0.0, ((limit - t[k]) * cap4[k]) / net[k]) : h->lookahead_s;

      used[k] = npclip((t[k] - amb) / (limit - amb), 0.0, 1.0);
      used[k] = npmax(used[k], pos ? npclip(1.0 - hold / h->lookahead_s, 0.0, 1.0) : 0.0);
      if (used[k] > used[worst])
      {
        worst = k;
      }
    }
    spent = used[worst];
    h->spent[i] = spent;
    h->worst[i] = worst;
    if (envelope)
    {
      const double want = spent <= h->throttle_at ? 1.0 : npmax(0.0, (1.0 - spent) / (1.0 - h->throttle_at));
      int          tripped = die >= h->tj_max_c;

      h->derate[i] = npmin(want, h->derate[i] + h->recover_per_s * hh);
      for (int k = 0; k < 4; k++)
      {
        tripped = tripped || t[k] >= h->ceiling[k];
      }
      if (tripped && h->gates[i])
      {
        h->gates[i] = 0;
        h->trips[i] += 1;
        h->cap[i] = h->trip_margin;
        h->cap_at[i] = h->at;
      }
    }
  }
}

void wbc_pass_heat_peak(const wbc_heat_t *h, double *out)
{
  for (int i = 0; i < h->n; i++)
  {
    const double *t = h->t + 4 * i;
    double        top = t[0];

    for (int k = 1; k < 4; k++)
    {
      if (t[k] > top)
      {
        top = t[k];
      }
    }
    out[i] = h->die[i] >= top ? h->die[i] : top;
  }
}

void wbc_pass_hem(int n, double dt, const double *ap, const double *om, const double *alb, const double *c,
                  double *state, double *q, double *v, double *a, unsigned char *hit)
{
  const double L = c[0], mass = c[1], I0 = c[2], J = c[3], stiff = c[4], damp = c[5], dmin = c[6], dmax = c[7];
  const double width = c[8], k_imp = c[9], b_imp = c[10], invw[2] = {c[11], c[12]}, lo[2] = {c[13], c[14]};
  const double hi[2] = {c[15], c[16]};
  double       qx = state[0], qz = state[1], vx = state[2], vz = state[3];

  for (int t = 0; t < n; t++)
  {
    const double cx = cos(qx), sx = sin(qx), cz = cos(qz), sz = sin(qz);
    const double p0 = ap[3 * t], p1 = ap[3 * t + 1], p2 = ap[3 * t + 2];
    const double o0 = om[3 * t], o1 = om[3 * t + 1], o2 = om[3 * t + 2];
    const double b0 = alb[3 * t], b1 = alb[3 * t + 1], b2 = alb[3 * t + 2];
    const double r0 = L * sz, r1 = ((-L) * cx) * cz, r2 = ((-L) * sx) * cz;
    const double x1 = (L * sx) * cz, x2 = ((-L) * cx) * cz;
    const double z0 = L * cz, z1 = (L * cx) * sz, z2 = (L * sx) * sz;
    const double d0 = o1 * r2 - o2 * r1, d1 = o2 * r0 - o0 * r2, d2 = o0 * r1 - o1 * r0;
    const double v0 = 0.0 * vx + z0 * vz, v1 = x1 * vx + z1 * vz, v2 = x2 * vx + z2 * vz;
    const double af0 = ((p0 + (b1 * r2 - b2 * r1)) + (o1 * d2 - o2 * d1)) + 2.0 * (o1 * v2 - o2 * v1);
    const double af1 = ((p1 + (b2 * r0 - b0 * r2)) + (o2 * d0 - o0 * d2)) + 2.0 * (o2 * v0 - o0 * v2);
    const double af2 = ((p2 + (b0 * r1 - b1 * r0)) + (o0 * d1 - o1 * d0)) + 2.0 * (o0 * v1 - o1 * v0);
    const double tx = (((-mass) * ((0.0 * af0 + x1 * af1) + x2 * af2) - I0 * b0) - stiff * qx) - damp * vx;
    const double tz = (((-mass) * ((z0 * af0 + z1 * af1) + z2 * af2) - I0 * (b2 * cx - b1 * sx)) - stiff * qz)
                      - damp * vz;
    const double was[2] = {qx, qz}, rate[2] = {vx, vz};
    double       qdd[2] = {tx / J, tz / J};

    hit[t] = 0;
    for (int i = 0; i < 2; i++)
    {
      const double r_pen = was[i] < lo[i] ? lo[i] - was[i] : (was[i] > hi[i] ? was[i] - hi[i] : 0.0);
      const double sgn = was[i] < lo[i] ? 1.0 : (was[i] > hi[i] ? -1.0 : 0.0);

      if (r_pen > 0.0)
      {
        const double share = r_pen / width, x = share < 1.0 ? share : 1.0;
        const double imp = dmin + (dmax - dmin) * ((x * x) * (3.0 - 2.0 * x));
        const double aref = sgn * ((k_imp * imp) * r_pen) - b_imp * rate[i];

        hit[t] = 1;
        if (sgn * (aref - qdd[i]) > 0.0)
        {
          qdd[i] += (aref - qdd[i]) / (1.0 + (((1.0 - imp) / imp) * invw[i]) * J);
        }
      }
    }
    q[2 * t] = qx;
    q[2 * t + 1] = qz;
    v[2 * t] = vx;
    v[2 * t + 1] = vz;
    a[2 * t] = qdd[0];
    a[2 * t + 1] = qdd[1];
    vx += qdd[0] * dt;
    vz += qdd[1] * dt;
    qx += vx * dt;
    qz += vz * dt;
  }
  state[0] = qx;
  state[1] = qz;
  state[2] = vx;
  state[3] = vz;
}
