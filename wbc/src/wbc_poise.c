/** wbc_poise.c - under the ask (wbc_imp.h): machine.law.poise's boxes, their span, her capture point
    into them and which soles stand (flight.standing) - numpy's arithmetic operation for operation
    (wbc_np.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>
#include <stddef.h>

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

/** machine.law.poise.into: `zmp` in the box lo to hi, `share` of the way to where xi stays in it. */
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
    spanned over prop. */
static void spanned_of(const double low[2][2], const double high[2][2], const double *out_of,
                       const double prop[2][2], double by, double slo[2], double shi[2])
{
  double lo[2], hi[2];

  for (int j = 0; j < 2; j++)
  {
    lo[j] = out_of ? npmin(low[0][j] + out_of[0], low[1][j] + out_of[1]) : npmin(low[0][j], low[1][j]);
    hi[j] = out_of ? npmax(high[0][j] - out_of[0], high[1][j] - out_of[1]) : npmax(high[0][j], high[1][j]);
  }
  wbc_imp_spanned(lo, hi, prop[0], prop[1], by, slo, shi);
}

void wbc_imp_stands(const wbc_imp_t *k, double zmp[2], const double xi[2], const double low[2][2],
                    const double high[2][2], double om, double held, const double up[2], int swaying,
                    const double beat[4][2], const double bears[3][2], double rising, double borne,
                    const double stood[2], const double prop[2][2], double by, double stance[2])
{
  const double *sweep = beat[0], *off = beat[1], *flying = beat[2], *toes = beat[3];
  const double *loads = bears[0], *over = bears[1], *felt = bears[2];
  const double  kept = npclip(held, 0.0, 1.0), foot = npclip(held - 1.0, 0.0, 1.0);
  double        level[2], slo[2], shi[2], got[2], left[2], out_of[2];
  int           within[2], landed[2], lifting[2], st[2], meant[2];

  for (int b = 0; b < 2; b++)
  {
    level[b] = npmax(over[b], felt[b]);
  }
  spanned_of(low, high, NULL, prop, by, slo, shi);
  into(k, zmp, xi, slo, shi, om, 1.0 - foot, got);
  for (int j = 0; j < 2; j++)
  {
    zmp[j] = zmp[j] + kept * (got[j] - zmp[j]);
  }
  for (int b = 0; b < 2; b++)
  {
    const double out_up = k->big * (double)(up[b] > k->lift_eps);
    const double lb[2] = {low[b][0] + out_up, low[b][1] + out_up};
    const double hb[2] = {high[b][0] - out_up, high[b][1] - out_up};
    double       sb_lo[2], sb_hi[2];

    wbc_imp_spanned(lb, hb, prop[0], prop[1], by, sb_lo, sb_hi);
    within[b] = (zmp[0] >= sb_lo[0] - k->gate_m) && (zmp[0] <= sb_hi[0] + k->gate_m)
                && (zmp[1] >= sb_lo[1] - k->gate_m) && (zmp[1] <= sb_hi[1] + k->gate_m);
    meant[b] = off[b] < 0.5;
  }
  standing(k, meant, loads, level, rising, borne, landed);
  for (int b = 0; b < 2; b++)
  {
    const int early = (sweep[b] > k->early) && ((loads[b] >= k->touch_n) || (stood[b] > 0.5));
    const int gone = (npmax(flying[b], off[b] * toes[b]) > 0.5) && (landed[1 - b] || (over[b] > k->touch_m)) && !early;

    lifting[b] = ((up[b] > k->lift_eps) && ((within[1 - b] && swaying) || (over[b] > k->touch_m))
                  && (loads[b] <= loads[1 - b]))
                 || gone;
    meant[b] = !lifting[b];
  }
  standing(k, meant, loads, level, rising, borne, st);
  for (int b = 0; b < 2; b++)
  {
    stance[b] = (double)st[b];
    left[b] = stance[b] * (1.0 - (double)(off[b] > 0.5) * (double)landed[1 - b]);
  }
  {
    const double most = npmax(left[0], left[1]);

    for (int b = 0; b < 2; b++)
    {
      left[b] = left[b] + (1.0 - most) * stance[b];
      out_of[b] = k->big * (1.0 - left[b]);
    }
  }
  spanned_of(low, high, out_of, prop, by, slo, shi);
  into(k, zmp, xi, slo, shi, om, foot * npmin(left[0], left[1]), got);
  for (int j = 0; j < 2; j++)
  {
    zmp[j] = zmp[j] + kept * (got[j] - zmp[j]);
  }
}
