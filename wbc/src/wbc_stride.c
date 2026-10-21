/** wbc_stride.c - under the ask (wbc_imp.h): machine.modal's reading of her, machine.stride's
    beat, landings, rocks and heels, machine.poise's boxes and stance, flight.standing - numpy's
    arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>
#include <stddef.h>

#define PI 3.14159265358979323846

/** Each sole's share of the soles that stand. */
void wbc_imp_shares(const wbc_imp_t *k, const double stance[2], double bears[2])
{
  const double sum = (0.0 + stance[0]) + stance[1];

  for (int b = 0; b < 2; b++)
  {
    bears[b] = (stance[b] + k->tiny) / (sum + 2.0 * k->tiny);
  }
}

/** machine.modal.shortened: her axial compression and its rate. */
void wbc_imp_shortened(const wbc_imp_t *k, const double len[2], const double rate[2], const double *fr,
                      const double stance[2], double axial[2])
{
  double w[2];

  wbc_imp_shares(k, stance, w);
  axial[0] = (fr[k->f_reach0] - npdot(w, 1, len, 1, 2)) / (k->axial_m * fr[k->f_hips_by]);
  axial[1] = -npdot(w, 1, rate, 1, 2) / (k->axial_m * fr[k->f_hips_by]);
}

/** machine.modal.under: the floor under each sole's middle, its rise along her way. */
double wbc_imp_under(const wbc_imp_t *k, const double *y, const double *fr, const double stance[2], double floors[2])
{
  const double rr[2] = {fr[k->f_rot], fr[k->f_rot + 2]};
  double       bears[2], pitched[2], rise;

  wbc_imp_shares(k, stance, bears);
  wbc_imp_pitched(k, fr + k->f_ground, pitched);
  pitched[0] = -tan(pitched[0]);
  pitched[1] = -tan(pitched[1]);
  rise = npdot(bears, 1, pitched, 1, 2);
  for (int b = 0; b < 2; b++)
  {
    const double d[2] = {y[k->y_sole + 3 * b] - fr[k->f_floor_at], y[k->y_sole + 3 * b + 2] - fr[k->f_floor_at + 1]};

    floors[b] = fr[k->f_floor] + rise * npmv2(d, rr);
  }
  return rise;
}

/** machine.modal.reading: her modes and their rates as she is, her legs (each hip from its
    ankle (2, 3), its length, that's rate), her turns and their rates (3, 2). */
void wbc_imp_reading(const wbc_imp_t *k, const double *y, const double *fr, const double stance[2], double *z,
                    double *zd, double leg[2][3], double len[2], double lrate[2], double turn[3][2],
                    double spun[3][2])
{
  const double *rot = fr + k->f_rot, *unit = fr + k->f_unit;
  double        dd[2][3], ddv[2][3], dp[2][3], dsp[2][3], d[2][3], dv[2][3], place[2][3], speed[2][3];
  double        axial[2], tl, al, rl, arm[2];

  for (int b = 0; b < 2; b++)
  {
    const double *q = y + k->y_quat + 4 * b, *s = y + k->y_spin + 3 * b;
    const double  w = q[0], x = q[1], j = q[2], kk = q[3];
    const double  left[3] = {1.0 - 2.0 * (j * j + kk * kk), 2.0 * (x * j + w * kk), 2.0 * (x * kk - w * j)};
    const double  up[3] = {2.0 * (x * j - w * kk), 1.0 - 2.0 * (x * x + kk * kk), 2.0 * (j * kk + w * x)};
    const double  on[3] = {2.0 * (x * kk + w * j), 2.0 * (j * kk - w * x), 1.0 - 2.0 * (x * x + j * j)};

    turn[0][b] = atan2(npmv3(up, rot), up[1]);
    turn[1][b] = atan2(left[1], npmv3(left, rot + 3));
    turn[2][b] = atan2(npmv3(on, rot + 3), npmv3(on, rot));
    spun[0][b] = npmv3(s, rot + 3);
    spun[1][b] = npmv3(s, rot);
    spun[2][b] = npmv3(s, rot + 6);
    for (int i = 0; i < 3; i++)
    {
      dp[b][i] = y[k->y_sole + 3 * b + i] - y[k->y_com + i];
      dsp[b][i] = y[k->y_vsole + 3 * b + i] - y[k->y_vcom + i];
      dd[b][i] = y[k->y_hip + 3 * b + i] - y[k->y_sole + 3 * b + i];
      ddv[b][i] = y[k->y_vcom + i] - y[k->y_vsole + 3 * b + i];
    }
  }
  npmmt(&dp[0][0], 2, rot, &place[0][0]);
  npmmt(&dsp[0][0], 2, rot, &speed[0][0]);
  npmmt(&dd[0][0], 2, rot, &d[0][0]);
  npmmt(&ddv[0][0], 2, rot, &dv[0][0]);
  for (int b = 0; b < 2; b++)
  {
    const double t = y[k->y_tilt + b];
    const double spoke[3] = {sin(t), 0.0 * t, cos(t)}, inward[3] = {cos(t), 0.0 * t, -sin(t)};
    double       sq[3], pr[3];

    for (int i = 0; i < 3; i++)
    {
      leg[b][i] = (d[b][i] - k->ankle_h * spoke[i]) + k->middle * inward[i];
      sq[i] = leg[b][i] * leg[b][i];
      pr[i] = dv[b][i] * leg[b][i];
    }
    len[b] = sqrt(0.0 + npsum(sq, 3));
    lrate[b] = (0.0 + npsum(pr, 3)) / len[b];
  }
  wbc_imp_shortened(k, len, lrate, fr, stance, axial);
  for (int m = 0; m < k->n; m++)
  {
    z[m] = (0.5 * npein6(k->hands + 6 * m, &place[0][0])) / unit[m] + fr[k->f_z0 + m];
    zd[m] = (0.5 * npein6(k->hands + 6 * m, &speed[0][0])) / unit[m];
  }
  {
    const double za = axial[0] - z[k->axial], zda = axial[1] - zd[k->axial];

    for (int m = 0; m < k->n; m++)
    {
      z[m] += npeye(m, k->axial) * za;
      zd[m] += npeye(m, k->axial) * zda;
    }
  }
  arm[0] = y[k->y_arm] - fr[k->f_arm0];
  arm[1] = y[k->y_arm + 1] - fr[k->f_arm0 + 1];
  tl = npdot(k->limbs, 1, y + k->y_tilt, 1, 2);
  al = npdot(k->limbs, 1, arm, 1, 2);
  rl = npdot(k->limbs, 1, y + k->y_tilt_rate, 1, 2);
  for (int m = 0; m < k->n; m++)
  {
    const double angle = (((npeye(m, k->pitch) * (turn[0][1] - fr[k->f_pitch0 + 1]) + npeye(m, k->list) * turn[1][0])
                           + npeye(m, k->turn) * turn[2][0]) + (npeye(m, k->rock) * 0.5) * tl)
                         + (npeye(m, k->arms) * 0.5) * al;
    const double rate = ((npeye(m, k->pitch) * spun[0][1] + npeye(m, k->list) * spun[1][0]) + npeye(m, k->turn) * spun[2][0])
                        + (npeye(m, k->rock) * 0.5) * rl;

    z[m] = z[m] + angle / unit[m];
    zd[m] = zd[m] + rate / unit[m];
  }
}

void wbc_imp_boxes(const wbc_imp_t *k, const double flat[2][2], const double edge[2], const double tilt[2],
                   double low[2][2], double high[2][2])
{
  for (int b = 0; b < 2; b++)
  {
    for (int j = 0; j < 2; j++)
    {
      const double on_edge = edge[b] * k->along_on[j];
      const double mid = flat[b][j] + (on_edge * k->half_m) * cos(tilt[b]);

      low[b][j] = mid - k->extent[j] * (1.0 - fabs(on_edge));
      high[b][j] = mid + k->extent[j] * (1.0 - fabs(on_edge));
    }
  }
}

void wbc_imp_spanned(const double lo[2], const double hi[2], const double plo[2], const double phi[2], double by,
                     double slo[2], double shi[2])
{
  for (int j = 0; j < 2; j++)
  {
    slo[j] = lo[j] + by * (npmin(lo[j], plo[j]) - lo[j]);
    shi[j] = hi[j] + by * (npmax(hi[j], phi[j]) - hi[j]);
  }
}

/** machine.poise.into: `zmp` in the box lo to hi, `share` of the way to where xi stays in it. */
static void into(const wbc_imp_t *k, const double zmp[2], const double xi[2], const double lo[2], const double hi[2],
                 double om, double share, double got[2])
{
  for (int j = 0; j < 2; j++)
  {
    const double plain = npclip(zmp[j], lo[j], hi[j]);
    const double viable = npclip(npclip(zmp[j], xi[j] - (hi[j] - xi[j]) / (om * k->capture_s),
                                        xi[j] + (xi[j] - lo[j]) / (om * k->capture_s)), lo[j], hi[j]);

    got[j] = plain + share * (viable - plain);
  }
}

/** flight.standing: which soles stand of those `meant`. */
static void standing(const wbc_imp_t *k, const int meant[2], const double loads[2], const double over[2],
                     double rising, double borne, int out[2])
{
  const int rests = (loads[0] >= k->touch_n || loads[1] >= k->touch_n) | (borne >= k->touch_n)
                    | (fabs(rising) <= k->rest_m_s);

  for (int b = 0; b < 2; b++)
  {
    out[b] = meant[b] && ((loads[b] >= k->touch_n) || ((over[b] <= k->touch_m) && rests));
  }
}

/** The box `zmp` is let in: the soles' boxes - each pushed `out_of` off, none NULL -
    spanned over prop, then over where she is told. */
static void spanned_of(const double low[2][2], const double high[2][2], const double *out_of,
                       const double prop[2][2], double by, const double tp[2][2], double trust, double slo[2],
                       double shi[2])
{
  double lo[2], hi[2];

  for (int j = 0; j < 2; j++)
  {
    lo[j] = out_of ? npmin(low[0][j] + out_of[0], low[1][j] + out_of[1]) : npmin(low[0][j], low[1][j]);
    hi[j] = out_of ? npmax(high[0][j] - out_of[0], high[1][j] - out_of[1]) : npmax(high[0][j], high[1][j]);
  }
  wbc_imp_spanned(lo, hi, prop[0], prop[1], by, slo, shi);
  wbc_imp_spanned(slo, shi, tp[0], tp[1], trust, slo, shi);
}

void wbc_imp_stands(const wbc_imp_t *k, double zmp[2], const double xi[2], const double low[2][2],
                    const double high[2][2], double om, double held, const double up[2], int swaying,
                    const double beat[3][2], const double bears[3][2], double rising, double borne,
                    const double stood[2], const double prop[2][2], double by, const double told[2], double trust,
                    double stance[2])
{
  const double *sweep = beat[0], *off = beat[1], *flying = beat[2];
  const double *loads = bears[0], *over = bears[1], *felt = bears[2];
  const double  kept = npclip(held, 0.0, 1.0), foot = npclip(held - 1.0, 0.0, 1.0);
  const double  t_pt[2][2] = {{told[0], told[1]}, {told[0], told[1]}};
  double        level[2], slo[2], shi[2], got[2], left[2], out_of[2];
  int           within[2], landed[2], lifting[2], st[2], meant[2];

  for (int b = 0; b < 2; b++)
  {
    level[b] = npmax(over[b], felt[b]);
  }
  spanned_of(low, high, NULL, prop, by, t_pt, trust, slo, shi);
  into(k, zmp, xi, slo, shi, om, 1.0 - foot, got);
  for (int j = 0; j < 2; j++)
  {
    zmp[j] = zmp[j] + kept * (got[j] - zmp[j]);
  }
  {
    const double p_box[2][2] = {{prop[0][0] - k->extent[0], prop[0][1] - k->extent[1]},
                                {prop[1][0] + k->extent[0], prop[1][1] + k->extent[1]}};
    const double t_box[2][2] = {{told[0] - k->extent[0], told[1] - k->extent[1]},
                                {told[0] + k->extent[0], told[1] + k->extent[1]}};

    for (int b = 0; b < 2; b++)
    {
      const double out_up = k->big * (double)(up[b] > k->lift_eps);
      const double lb[2] = {low[b][0] + out_up, low[b][1] + out_up};
      const double hb[2] = {high[b][0] - out_up, high[b][1] - out_up};
      double       sb_lo[2], sb_hi[2];

      wbc_imp_spanned(lb, hb, p_box[0], p_box[1], by, sb_lo, sb_hi);
      wbc_imp_spanned(sb_lo, sb_hi, t_box[0], t_box[1], trust, sb_lo, sb_hi);
      within[b] = (zmp[0] >= sb_lo[0] - k->gate_m) && (zmp[0] <= sb_hi[0] + k->gate_m)
                  && (zmp[1] >= sb_lo[1] - k->gate_m) && (zmp[1] <= sb_hi[1] + k->gate_m);
      meant[b] = off[b] < 0.5;
    }
    standing(k, meant, loads, level, rising, borne, landed);
    for (int b = 0; b < 2; b++)
    {
      const int early = (sweep[b] > k->early) && ((loads[b] >= k->touch_n) || (stood[b] > 0.5));
      const int gone = (flying[b] > 0.5) && (landed[1 - b] || (over[b] > k->touch_m)) && !early;

      lifting[b] = ((up[b] > k->lift_eps) && ((within[1 - b] && swaying) || (over[b] > k->touch_m))) || gone;
      meant[b] = !lifting[b];
    }
    standing(k, meant, loads, level, rising, borne, st);
    for (int b = 0; b < 2; b++)
    {
      stance[b] = (double)st[b];
      left[b] = stance[b] * (1.0 - off[b] * (double)landed[1 - b]);
    }
    {
      const double most = npmax(left[0], left[1]);

      for (int b = 0; b < 2; b++)
      {
        left[b] = left[b] + (1.0 - most) * stance[b];
        out_of[b] = k->big * (1.0 - left[b]);
      }
    }
    spanned_of(low, high, out_of, p_box, by, t_box, trust, slo, shi);
  }
  into(k, zmp, xi, slo, shi, om, foot * npmin(left[0], left[1]), got);
  for (int j = 0; j < 2; j++)
  {
    zmp[j] = zmp[j] + kept * (got[j] - zmp[j]);
  }
}

void wbc_imp_beat(const wbc_imp_t *k, double drive, double psi, double sway, double lateral, double turns,
                  const double was[2], double need, const double lead[2], double *stepping, double *way,
                  double *swung, double sweep[2], double phi[2], double on[2], double off[2], double u[2],
                  double flying[2])
{
  *stepping = npmax(npclip(fabs(drive) / k->stirs, 0.0, 1.0), need);
  *way = 1.0 - 2.0 * (double)(sin(lateral - psi) * copysign(1.0, sway) < -k->back);
  for (int b = 0; b < 2; b++)
  {
    const double side = ((-k->limbs[b]) * copysign(1.0, drive)) * *way;

    sweep[b] = side * sin(psi);
    phi[b] = atan2(side * sin(psi), side * cos(psi));
    on[b] = (double)((phi[b] > k->phi_w - PI) && (phi[b] < k->phi_w));
    u[b] = npclip(0.5 * (phi[b] / k->phi_w + 1.0) + lead[b], 0.0, 1.0);
    off[b] = (npmax(*stepping, was[b]) * on[b]) * (1.0 - (double)((lead[b] > 0.0) && (u[b] >= 1.0)));
    flying[b] = off[b] * (double)(u[b] > 0.0);
  }
  *swung = (0.5 * turns) / k->phi_w;
}

void wbc_imp_hurried(const wbc_imp_t *k, const double aside[2], double om, const double u[2], double rate,
                     const double kp[2], double hurry[2])
{
  for (int b = 0; b < 2; b++)
  {
    const double due = log(npmax(k->reach_m / npmax(aside[b], k->tiny), 1.0)) / om;

    hurry[b] = npmax(1.0, (1.0 - u[b]) / (rate * npmax(due, sqrt(6.0 / kp[b]))));
  }
}

double wbc_imp_cruises(const wbc_imp_t *k, double way, double drive, double turns, double going)
{
  const double cruise = ((((way * 2.0) / PI) * k->stride_m) * fabs(drive)) * sin(k->phi_w) * turns;

  return npclip(cruise, going - k->ahead_m_s, going + k->ahead_m_s);
}

void wbc_imp_meets(const wbc_imp_t *k, const double xi[2], const double c[3], const double flat[2][2],
                   double cruise, double om, double turns, double wide, double across, double reach,
                   double meet[2][2])
{
  const double step_s = PI / npmax(turns, k->tiny), grown = exp(om * step_s);
  const double led = k->leads * npmax((xi[0] - c[0]) - cruise / om, -0.04), s_ac = npsign(across);
  const double led_lat = s_ac * npclip(s_ac * (xi[1] - c[1]) - (1.35 * fabs(across)) / om, -0.052, 0.042);

  for (int b = 0; b < 2; b++)
  {
    const double m0 = xi[0] - ((((cruise * step_s) / (grown - 1.0)) - led) + 0.0 * k->limbs[b]);
    const double m1 = xi[1] - ((-led_lat) - (k->limbs[b] * wide) / (grown + 1.0));

    meet[b][0] = c[0] + npclip(m0 - c[0], -reach, reach);
    meet[b][1] = flat[1 - b][1] + k->limbs[b] * npmax(k->limbs[b] * (m1 - flat[1 - b][1]), k->apart_m);
  }
}

void wbc_imp_rocks(const wbc_imp_t *k, double rock, double rate, double acc, double tip[2], double tipping[2],
                   double tipped[2])
{
  for (int b = 0; b < 2; b++)
  {
    const double heel = k->rock_rad * (1.0 + k->heel_up * npsign(k->limbs[b] * rock));

    tip[b] = (heel * k->limbs[b]) * rock;
    tipping[b] = (heel * k->limbs[b]) * rate;
    tipped[b] = (heel * k->limbs[b]) * acc;
  }
}

void wbc_imp_folds(const wbc_imp_t *k, const double off[2], const double loads[2], double bent, double fold[2],
                   double shorter[2])
{
  const double sum = (0.0 + loads[0]) + loads[1];

  for (int b = 0; b < 2; b++)
  {
    fold[b] = (k->fold_rad * off[b]) * (1.0 - loads[b] / npmax(sum, k->tiny));
    shorter[b] = k->leg_m * (cos(0.5 * bent) - cos(0.5 * (bent + fold[b])));
  }
}

void wbc_imp_pitched(const wbc_imp_t *k, const double ground[2], double pitched[2])
{
  for (int b = 0; b < 2; b++)
  {
    pitched[b] = ground[b] * npclip((fabs(ground[b]) - 0.5 * k->edge_rad) / (0.5 * k->edge_rad), 0.0, 1.0);
  }
}

void wbc_imp_heels(const wbc_imp_t *k, const double place[2][3], const double tilt[2], const double tilting[2],
                   const double hips[2], const double floors[2], double leg, const double shorter[2],
                   const double off[2], const double tip[2], const double stance[2], double stepping,
                   const double was[2], double dt, const double ground[2], double rocked[2], double flatten[2],
                   double edge[2], double rounds[2][3])
{
  double pitched[2];

  wbc_imp_pitched(k, ground, pitched);
  for (int b = 0; b < 2; b++)
  {
    const double spoke[3] = {sin(tilt[b]), 0.0 * tilt[b], cos(tilt[b])};
    const double inward[3] = {cos(tilt[b]), 0.0 * tilt[b], -sin(tilt[b])};
    const double ball0 = place[b][0] + k->half_m * inward[0], ball1 = place[b][2] + k->half_m * inward[2];
    const double hip = hips[b] - ball0, high = (((floors[b] + k->ankle_h) + leg) - k->dip_m) - ball1;
    const double lever = hypot(hip * k->ball - high * k->ankle_h, hip * k->ankle_h + high * k->ball);
    const double reach = leg - shorter[b];
    const double cosine = ((((reach * reach - hip * hip) - high * high) - k->ball * k->ball) - k->ankle_h * k->ankle_h)
                          / (2.0 * lever);
    const double need = npclip(atan2(-(hip * k->ankle_h + high * k->ball), hip * k->ball - high * k->ankle_h)
                               + acos(npclip(cosine, -1.0, 1.0)), 0.0, k->heel_rad)
                        * npclip(hip / k->ball, 0.0, 1.0);
    const double lean = tilt[b] - pitched[b];
    double       sign;

    rocked[b] = npmax(off[b] * npmax(tip[b], 0.0), need);
    flatten[b] = (stepping * stance[b]) * (k->flat_k * (rocked[b] - lean)
                                           + k->contact_k * npclip((rocked[b] - was[b]) / dt, -k->heel_rad_s,
                                                                   k->heel_rad_s));
    sign = npsign(lean + k->big * rocked[b]);
    edge[b] = ((stepping * stance[b]) * sign) * npclip(npmax(fabs(lean), rocked[b]) / k->edge_rad, 0.0, 1.0);
    for (int j = 0; j < 3; j++)
    {
      rounds[b][j] = (sign * k->half_m) * (flatten[b] * spoke[j] + (tilting[b] * tilting[b]) * inward[j]);
    }
  }
}

void wbc_imp_clears(const wbc_imp_t *k, const double u[2], const double flying[2], const double swung[2],
                    double high, const double tip[2], const double tipping[2], const double tilt[2],
                    const double late[2], double clear[3][2])
{
  for (int b = 0; b < 2; b++)
  {
    const double on = npclip(u[b] / k->rise_u, 0.0, 1.0), by = npclip((1.0 - u[b]) / k->fall_u, 0.0, 1.0);
    const double e00 = (on * on) * (3.0 - 2.0 * on), e01 = (by * by) * (3.0 - 2.0 * by);
    const double e10 = ((6.0 * on) * (1.0 - on)) / k->rise_u, e11 = ((-6.0 * by) * (1.0 - by)) / k->fall_u;
    const double e20 = ((6.0 * (1.0 - 2.0 * on)) * (double)(on < 1.0)) / k->rise_u2;
    const double e21 = ((6.0 * (1.0 - 2.0 * by)) * (double)(by < 1.0)) / k->fall_u2;
    const double down = npmax(tip[b], tilt[b]), toes = k->half_m + k->toe_m;

    clear[0][b] = ((((high * flying[b]) * e00) * e01) - k->reach_m_s * late[b])
                  + flying[b] * (k->half_m * npmax(-sin(tip[b]), 0.0) + toes * npmax(sin(down), 0.0));
    clear[1][b] = ((((high * flying[b]) * (e10 * e01 + e00 * e11)) * swung[b]) - k->reach_m_s * (double)(late[b] > 0.0))
                  + ((flying[b] * ((-k->half_m) * (double)(tip[b] < 0.0) + toes * (double)(tip[b] > 0.0))) * cos(tip[b]))
                    * tipping[b];
    clear[2][b] = ((high * flying[b]) * (e20 * e01 + e00 * e21)) * (swung[b] * swung[b]);
  }
}

void wbc_imp_lates(const wbc_imp_t *k, const double phi[2], double turns, double stepping, double late[2])
{
  for (int b = 0; b < 2; b++)
  {
    late[b] = stepping * npclip((phi[b] - k->phi_w) / npmax(turns, k->tiny), 0.0, k->reach_s);
  }
}

void wbc_imp_goes(const wbc_imp_t *k, const double meet[2][2], const double flat[2][2], const double shod[2][3],
                  const double v[3], const double u[2], const double turns[2], const double kp[2],
                  const double kd[2], double acc[2][2])
{
  for (int b = 0; b < 2; b++)
  {
    const double to_go = npmax((((k->arrive_u - u[b]) * 2.0) * k->phi_w) / npmax(turns[b], k->tiny), k->soon_s);

    for (int j = 0; j < 2; j++)
    {
      acc[b][j] = npmin(6.0 / (to_go * to_go), kp[j]) * (meet[b][j] - flat[b][j])
                  - npmin(4.0 / to_go, kd[j]) * (shod[b][j] - v[j]);
    }
  }
}
