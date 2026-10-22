/** wbc_floor.c - under the ask (wbc_imp.h): a floor told her, machine.stride's footholds and the height
    her centre keeps over its incline - numpy's arithmetic operation for operation (wbc_np.h). Portable
    C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

#define PI 3.14159265358979323846

/** machine.stride.footholds: each landing moved onto the nearest flat tread of a floor told her its whole sole fits
    within her reach, the tread it left from kept (hold, -1 none); the highest floor between each sole and its landing;
    the landing's height over the floor felt; each tread's run; the tread landed on (-1 none). */
void wbc_imp_footholds(const wbc_imp_t *k, const double *along, const double *rise, int told, const double s_meet[2],
                       const double s_fore[2], const double s_now[2], double s_com, double reach, const double hold[2],
                       double gap, double step, double s_new[2], double top[2], double land[2], double run[2], double cell[2])
{
  enum { MAP_MAX = 16 };
  const int    K = (int)npmin(k->map_k, (double)MAP_MAX), n = (told > 2 ? told : 2);
  double       lo[MAP_MAX], hi[MAP_MAX];
  int          fits[MAP_MAX], any = 0;

  for (int i = 0; i < K - 1; i++)
  {
    const double a = along[i], b = (i == told - 2) ? k->big : along[i + 1];
    const int    flat = (i < told - 1) && (fabs(rise[i + 1] - rise[i]) < k->tiny)
                        && (b - a > k->half_m + k->toe_m + 2.0 * k->margin_m);

    lo[i] = npmax(a + k->margin_m, s_com - reach);
    hi[i] = npmin(b - k->half_m - k->toe_m - k->margin_m, s_com + reach);
    fits[i] = flat && (lo[i] <= hi[i]);
    any = any || fits[i];
  }
  for (int b = 0; b < 2; b++)
  {
    int    best = 0;
    double cost = INFINITY, hi_top, lo_end, hi_end, span_lo, span_hi;

    for (int i = 0; i < K - 1; i++)
    {
      const double c = fabs(npclip(s_fore[b], lo[i], hi[i]) - s_fore[b]) + k->big * (1.0 - (double)fits[i]);

      if (c < cost)
      {
        cost = c;
        best = i;
      }
    }
    {
      const int h = (int)hold[b];
      const int held = (h >= 0) && (h < K - 1) && fits[h];
      const int kk = held ? h : best;

      const double drawn = npclip(s_meet[b], lo[kk], hi[kk]);
      const double pull = npclip(1.0 - (hi[kk] - drawn) / npmax(step, k->tiny), 0.0, 1.0);

      s_new[b] = s_meet[b] + (double)any * ((drawn + (hi[kk] - drawn) * pull) - s_meet[b]);
      span_lo = npmin(s_now[b], s_new[b]) - k->half_m;
      span_hi = npmax(s_now[b], s_new[b]) + k->half_m + k->toe_m;
      hi_top = -k->big;
      for (int i = 0; i < K - 1; i++)
      {
        const int over = (i < told - 1) && (((i == told - 2) ? k->big : along[i + 1]) >= span_lo) && (along[i] <= span_hi);

        hi_top = npmax(hi_top, over ? rise[i] : -k->big);
      }
      lo_end = wbc_imp_interp(span_lo, along, rise, n);
      hi_end = wbc_imp_interp(span_hi, along, rise, n);
      top[b] = (double)any * (npmax(hi_top, npmax(lo_end, hi_end)) + gap) - k->big * (1.0 - (double)any);
      land[b] = (double)any * (rise[kk] + gap);
      run[b] = (double)any * (((kk == told - 2) ? k->big : along[kk + 1]) - along[kk]) + k->big * (1.0 - (double)any);
      cell[b] = (double)any * (double)kk - (1.0 - (double)any);
    }
  }
}

/** numpy.interp over the first n points of an increasing along. */
double wbc_imp_interp(double x, const double *along, const double *rise, int n)
{
  if (x <= along[0])
  {
    return rise[0];
  }
  if (x >= along[n - 1])
  {
    return rise[n - 1];
  }
  for (int i = 0; i < n - 1; i++)
  {
    if (x <= along[i + 1])
    {
      const double w = (x - along[i]) / (along[i + 1] - along[i]);

      return rise[i] + w * (rise[i + 1] - rise[i]);
    }
  }
  return rise[n - 1];
}

/** machine.impedance._landings: her cruise no faster than a tread a step, each landing onto its tread, the highest floor
    between and the landing's height; the tread a swing left from kept in a[cell]. */
void wbc_imp_landings(const wbc_imp_t *k, const double *y, const double *fr, double *a, const double *rot, const double c[3],
                      const double flat[2][2], const double xi[2], double *cruise, double om, double turns, double wide,
                      double across, double reach, const double stood[2], const double u[2], const double flying[2],
                      double meet[2][2], double top[2], double land[2], double adj[2], double over[2])
{
  const double *way = fr + k->f_map_way, *along = fr + k->f_map_along, *rise = fr + k->f_map_rise;
  const int     told = (int)fr[k->f_map_n];
  const double  start = fr[k->f_map_at] * way[0] + fr[k->f_map_at + 1] * way[1];
  const double  s_com = (y[k->y_com] * way[0] + y[k->y_com + 2] * way[1]) - start;
  const double  step_s = PI / npmax(turns, k->tiny);
  double        s_now[2], s_meet[2], s_fore[2], left_s[2], hold[2], s_new[2], run[2], cell[2], t2[2], l2[2], w[2], under[2], gap;

  for (int b = 0; b < 2; b++)
  {
    s_now[b] = (y[k->y_sole + 3 * b] * way[0] + y[k->y_sole + 3 * b + 2] * way[1]) - start;
    hold[b] = stood[b] > 0.5 ? -1.0 : a[k->a_cell + b];
    under[b] = wbc_imp_floor_at(along, rise, told, s_now[b]);
  }
  /* the told floor over the felt one, anchored under her standing soles */
  wbc_imp_shares(k, stood, w);
  gap = -npdot(w, 1, under, 1, 2);
  for (int b = 0; b < 2; b++)
  {
    left_s[b] = step_s * (1.0 - u[b] * flying[b]);
  }
  wbc_imp_meets(k, xi, c, flat, *cruise, om, turns, wide, across, reach, meet);
  for (int b = 0; b < 2; b++)
  {
    s_meet[b] = ((meet[b][0] * rot[0] + meet[b][1] * rot[3]) * way[0] + (meet[b][0] * rot[2] + meet[b][1] * rot[5]) * way[1]) - start;
    s_fore[b] = s_meet[b] + (*cruise) * left_s[b];
  }
  wbc_imp_footholds(k, along, rise, told, s_meet, s_fore, s_now, s_com, reach, hold, gap, (*cruise) * step_s, s_new, t2, l2, run,
                    cell);
  *cruise = npmin(*cruise, (npmin(run[0], run[1]) * turns) / PI);
  wbc_imp_meets(k, xi, c, flat, *cruise, om, turns, wide, across, reach, meet);
  for (int b = 0; b < 2; b++)
  {
    s_meet[b] = ((meet[b][0] * rot[0] + meet[b][1] * rot[3]) * way[0] + (meet[b][0] * rot[2] + meet[b][1] * rot[5]) * way[1]) - start;
    s_fore[b] = s_meet[b] + (*cruise) * left_s[b];
  }
  wbc_imp_footholds(k, along, rise, told, s_meet, s_fore, s_now, s_com, reach, hold, gap, (*cruise) * step_s, s_new, top, land, run,
                    cell);
  for (int b = 0; b < 2; b++)
  {
    const double dx = (s_new[b] - s_meet[b]) * way[0], dz = (s_new[b] - s_meet[b]) * way[1];

    meet[b][0] += dx * rot[0] + dz * rot[2];
    meet[b][1] += dx * rot[3] + dz * rot[5];
    a[k->a_cell + b] = cell[b];
    adj[b] = (double)(told > 0) * (wbc_imp_floor_at(along, rise, told, s_now[b])
                                   - wbc_imp_floor_mean(k, along, rise, told, s_com - k->stride_m, s_com));
    over[b] = npclip((y[k->y_lowest + b] - fr[k->f_floor] - top[b]) / k->clear_m, 0.0, 1.0);
  }
}

/** The floor's integral from its start to `s`, its ends' heights past them. */
static double floor_area(const wbc_imp_t *k, const double *along, const double *rise, int told, double s)
{
  enum { MAP_MAX = 16 };
  const int    K = (int)npmin(k->map_k, (double)MAP_MAX), n = (told > 2 ? told : 2);
  const double x = npclip(s, along[0], along[n - 1]);
  double       terms[MAP_MAX];

  for (int i = 0; i < K - 1; i++)
  {
    terms[i] = (double)(i < told - 1) * rise[i] * (npclip(x, along[i], along[i + 1]) - along[i]);
  }
  return (rise[0] * npmin(s - along[0], 0.0) + rise[n - 1] * npmax(s - along[n - 1], 0.0)) + npsum(terms, K - 1);
}

/** machine.stride.floor_mean: a told floor's mean height from s0 to s1, its ends' past them; 0 with nothing told. */
double wbc_imp_floor_mean(const wbc_imp_t *k, const double *along, const double *rise, int told, double s0, double s1)
{
  return (double)(told > 0) * (floor_area(k, along, rise, told, s1) - floor_area(k, along, rise, told, s0)) / (s1 - s0);
}

/** machine.stride.floor_at: a told floor's height under `s`, the tread's it lies on; its ends' past them. */
double wbc_imp_floor_at(const double *along, const double *rise, int told, double s)
{
  const int    n = (told > 2 ? told : 2);
  const double x = npclip(s, along[0], along[n - 1]);
  double       h = rise[n - 1];

  for (int i = 0; i < n - 1; i++)
  {
    h = ((along[i] <= x) && (x < along[i + 1])) ? rise[i] : h;
  }
  return h;
}
