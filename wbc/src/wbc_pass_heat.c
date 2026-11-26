/** wbc_pass_heat.c - a pass's drive heat and hem steps on the host (wbc_pass.h). Portable C11. */
#include "wbc_pass.h"
#include "wbc_np.h"
#include "wbc_libm_arc.h"
#include "board_units.h"

#include <math.h>

void wbc_pass_heat_load(wbc_heat_t *h, const double *restrict torques, const double *restrict qd)
{
  const int                     n = h->n;
  const unsigned char *restrict active = h->active;
  const double *restrict        kt = h->kt, *restrict ratio_k = h->ratio_k;
  double *restrict              sq = h->sq, *restrict limb_rad_s = h->limb_rad_s, *restrict krpm = h->krpm;
  long long *restrict           ticks = h->ticks;

  for (int i = 0; i < n; i++)
  {
    const int    on = active[i] != 0U;
    const double amps = on ? torques[i] / kt[i] : 0.0, w = on ? fabs(qd[i]) : 0.0;

    sq[i] = amps * amps;
    ticks[i] = on ? 1LL : 0LL;
    limb_rad_s[i] = w;
    krpm[i] = w * ratio_k[i];
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
    double       *t = h->t + 4 * i;
    const double  ticks = (double)h->ticks[i];
    const double  sq = ticks > 0.0 ? h->sq[i] / npmax(1.0, ticks) : 0.0;
    const double  board = sq * (h->scale[i] * h->scale[i]);
    const double  dt_sw = npmax(0.0, t[0] - amb);
    const double  rds_k = rds != NULL ? rds[i] : (1.0 + h->rds_alpha * dt_sw) + (h->rds_beta * dt_sw) * dt_sw;
    const double  air_k = air != NULL ? air[i] : (1.0 + 0.35 * npmin(4.0, h->limb_rad_s[i])) / h->cloth_air[i];
    const double  rot_for = 1.0 + 0.55 * sqrt(h->krpm[i]);
    const double  fet = (0.5 * board) * h->rds_ohm * rds_k;
    const double  sw_on = (h->gates[i] && !h->shorted[i]) ? 1.0 : 0.0;
    const double  p_sw = 3.0 * fet + h->switching_w * sw_on;
    const double  p_lam = (1.5 * board) * h->shunt_ohm + h->house_w[i];
    const double  p_win = (h->r[i] * sq) * (1.0 + COPPER_TEMPCO_PER_K * npmax(0.0, t[3] - amb));
    const double  g_la = (0.35 * air_k) / h->lam_k_w[i];
    const double  g_sa = ((0.65 * air_k) * (1.0 + 0.25 * (rot_for - 1.0))) / h->lam_k_w[i];
    const double  g_wa = (air_k * rot_for) / h->win_k_w[i];
    const double *cap4 = h->capacity + 4 * i;
    double        net[4] = {p_sw, p_sw, p_sw, p_sw}, die, cap, used[4], spent;
    int           worst = 0;

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

void wbc_pass_heat_peak(const wbc_heat_t *h, double *restrict out)
{
  const int              n = h->n;
  const double *restrict tt = h->t, *restrict die = h->die;

  for (int i = 0; i < n; i++)
  {
    const double t0 = tt[4 * i], t1 = tt[4 * i + 1], t2 = tt[4 * i + 2], t3 = tt[4 * i + 3], d = die[i];
    double       top = t1 > t0 ? t1 : t0;

    top = t2 > top ? t2 : top;
    top = t3 > top ? t3 : top;
    out[i] = d >= top ? d : top;
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
    const wbc_sc_t ax = wbc_sc(qx), az = wbc_sc(qz);
    const double   cx = ax.c, sx = ax.s, cz = az.c, sz = az.s;
    const double   p0 = ap[3 * t], p1 = ap[3 * t + 1], p2 = ap[3 * t + 2];
    const double   o0 = om[3 * t], o1 = om[3 * t + 1], o2 = om[3 * t + 2];
    const double   b0 = alb[3 * t], b1 = alb[3 * t + 1], b2 = alb[3 * t + 2];
    const double   r0 = L * sz, r1 = ((-L) * cx) * cz, r2 = ((-L) * sx) * cz;
    const double   x1 = (L * sx) * cz, x2 = ((-L) * cx) * cz;
    const double   z0 = L * cz, z1 = (L * cx) * sz, z2 = (L * sx) * sz;
    const double   d0 = o1 * r2 - o2 * r1, d1 = o2 * r0 - o0 * r2, d2 = o0 * r1 - o1 * r0;
    const double   v0 = 0.0 * vx + z0 * vz, v1 = x1 * vx + z1 * vz, v2 = x2 * vx + z2 * vz;
    const double   af0 = ((p0 + (b1 * r2 - b2 * r1)) + (o1 * d2 - o2 * d1)) + 2.0 * (o1 * v2 - o2 * v1);
    const double   af1 = ((p1 + (b2 * r0 - b0 * r2)) + (o2 * d0 - o0 * d2)) + 2.0 * (o2 * v0 - o0 * v2);
    const double   af2 = ((p2 + (b0 * r1 - b1 * r0)) + (o0 * d1 - o1 * d0)) + 2.0 * (o0 * v1 - o1 * v0);
    const double   tx = (((-mass) * ((0.0 * af0 + x1 * af1) + x2 * af2) - I0 * b0) - stiff * qx) - damp * vx;
    const double   tz = (((-mass) * ((z0 * af0 + z1 * af1) + z2 * af2) - I0 * (b2 * cx - b1 * sx)) - stiff * qz)
                        - damp * vz;
    const double   was[2] = {qx, qz}, rate[2] = {vx, vz};
    double         qdd[2] = {tx / J, tz / J};

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
