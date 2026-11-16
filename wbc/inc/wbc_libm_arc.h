/** wbc_libm_arc.h - wbc_libm.h's inverse functions and hypot: arctan, arctan2, arcsin, arccos, hypot, by the same
    rules (basic operations alone, branchless, the Python twin's order), on wbc_libm.h's helpers and tables; a core
    that takes them includes this header for both. */
#ifndef WBC_LIBM_ARC_H
#define WBC_LIBM_ARC_H

#include "wbc_libm.h"

/** arctan(num/den) for 0 <= num <= den <= 2^900, fdlibm's ranges: c = 1 from 11/16 on, 1/2 from 7/16, else 0 (*one,
    *half); u + *ul = (num - c den)/(den + c num), the tangent of arctan(num/den) less arctan(c): the numerator
    exact (c a power of two or 0, num and c den within a factor of two), the denominator a sum of two (Knuth's), the
    quotient's rounding by Dekker's product. */
static inline double lm_atan_u(double num, double den, double *ul, bool *one, bool *half)
{
  double t = num / den, c, cn, n, dh, bb, dl, u, p, pe;

  *one = t >= 0.6875;
  *half = t >= 0.4375;
  c = lm_sel(*one, 1.0, lm_sel(*half, 0.5, 0.0));
  cn = c * num;
  n = num - c * den;
  dh = den + cn;
  bb = dh - den;
  dl = (den - (dh - bb)) + (cn - bb);
  u = n / dh;
  p = lm_two_prod(u, dh, &pe);
  *ul = (((n - p) - pe) - u * dl) / dh;
  return u;
}

/** arctan(c)'s high part on 2^-51's grid, and the rest of arctan(c) + arctan(u + ul) as *l. */
static inline double lm_atan_tail(bool one, bool half, double u, double ul, double *l)
{
  double z = u * u;

  *l = (lm_sel(one, lm_atan_c[1], lm_sel(half, lm_atan_c[3], 0.0)) + ul) + (u * z) * lm_e12(lm_atan, z);
  return lm_sel(one, lm_atan_c[0], lm_sel(half, lm_atan_c[2], 0.0));
}

/** hi + u + l, from pi/2 less when swapped, from pi less when turned: hi on 2^-51's grid as pi/2's and pi's, so
    that those differences are exact; the high part and u by Knuth's sum. Kept apart from lm_atan_tail so that each
    stays under the size a compiler inlines into a loop. */
static inline double lm_atan_turn(double hi, double u, double l, bool swap, bool turn)
{
  double h = lm_sel(swap, lm_grid_pi_2[0] - hi, hi), r = lm_sel(swap, lm_grid_pi_2[1] - l, l), v = lm_sel(swap, -u, u);
  double s;

  h = lm_sel(turn, lm_pi[0] - h, h);
  r = lm_sel(turn, lm_pi[1] - r, r);
  v = lm_sel(turn, -v, v);
  s = h + v;
  return s + (((h - s) + v) + r);
}

/** arctan x: |x| past 1 as pi/2 - arctan(1/|x|), its tangent (1 - c|x|)/(|x| + c), no reciprocal taken; |x| held
    to 2^900 there; the sign that of x (its zeros, a NaN's own). */
static inline double wbc_arctan(double x)
{
  double a = fabs(x), ul, u, l, hi;
  bool   inv = a > 1.0, one, half;

  u = lm_atan_u(lm_sel(inv, 1.0, a), lm_sel(inv, lm_sel(a > 0x1p900, 0x1p900, a), 1.0), &ul, &one, &half);
  hi = lm_atan_tail(one, half, u, ul, &l);
  return lm_copysign(lm_atan_turn(hi, u, l, inv, false), x);
}

/** arctan2(y, x): the smaller magnitude over the larger, both scaled by one power of two into [2^-900, 2^900]; the
    quadrant by the swap, x's sign and y's sign bit; zeros and infinities as Annex F's table. */
static inline double wbc_arctan2(double y, double x)
{
  double ax = fabs(x), ay = fabs(y), lo = lm_sel(ay > ax, ax, ay), hi = lm_sel(ay > ax, ay, ax), ul, u, l, s;
  double sc = lm_sel(hi > 0x1p900, 0x1p-200, lm_sel(hi < 0x1p-900, 0x1p200, 1.0));
  bool   one, half;

  u = lm_atan_u(lm_sel(hi == INFINITY, 0.0, lo * sc), lm_sel(hi == INFINITY, 1.0, hi * sc), &ul, &one, &half);
  s = lm_atan_tail(one, half, u, ul, &l);
  s = lm_atan_turn(s, u, l, ay > ax, x < 0.0);
  s = lm_sel(hi == 0.0, lm_sel(lm_copysign(1.0, x) < 0.0, lm_pi[0], 0.0), s);
  s = lm_copysign(lm_sel(lo == INFINITY, lm_sel(x > 0.0, lm_pi_4[0], lm_pi_3_4[0]), s), y);
  s = lm_sel(lm_nan(x), x + y, s);
  return lm_sel(lm_nan(y), x + y, s);
}

/** asin x, fdlibm's: x + x z R(z) under 1/2 (z = x^2); past it pi/2 - 2 asin s with s = sqrt((1 - |x|)/2) (of |z|: no errno's branch),
    s's high half w and its root's residual c = (z - w^2)/(s + w) carried below 0.975. */
static inline double wbc_arcsin(double x)
{
  double a = fabs(x), z = lm_sel(a < 0.5, x * x, 0.5 * (1.0 - a)), s = sqrt(fabs(z)), r = z * lm_e14(lm_asin, z);
  double g = LM_SPLIT * s, w = g - (g - s), c = (z - w * w) / (s + w);
  double y = lm_sel(a >= 0.975, lm_pi_2[0] - (2.0 * (s + s * r) - lm_pi_2[1]),
                    lm_pi_4[0] - (((2.0 * s) * r - (lm_pi_2[1] - 2.0 * c)) - (lm_pi_4[0] - 2.0 * w)));

  y = lm_sel(x < 0.0, -y, y);
  y = lm_sel(a > 1.0, NAN, y);
  return lm_sel(a < 0.5, x + x * r, y);
}

/** acos x, fdlibm's: pi/2 - (x + x z R(z)) under 1/2, pi - 2 (s + s R) under -1/2, 2 (w + c + s R) past 1/2. */
static inline double wbc_arccos(double x)
{
  double a = fabs(x), z = lm_sel(a < 0.5, x * x, 0.5 * (1.0 - a)), s = sqrt(fabs(z)), r = z * lm_e14(lm_asin, z);
  double g = LM_SPLIT * s, w = g - (g - s), c = (z - w * w) / (s + w);
  double y = lm_sel(x < 0.0, lm_pi[0] - 2.0 * (s + (r * s - lm_pi_2[1])), 2.0 * (w + (r * s + c)));

  y = lm_sel(x == 1.0, 0.0, y);
  y = lm_sel(a > 1.0, NAN, y);
  return lm_sel(a < 0.5, lm_pi_2[0] - (x - (lm_pi_2[1] - x * r)), y);
}

/** sqrt(x^2 + y^2), scaled by a power of two away from overflow and underflow, the root's rounding corrected by
    the residual, exact by Dekker's squares; an infinity wins over a NaN. */
static inline double wbc_hypot(double x, double y)
{
  double ax = fabs(x), ay = fabs(y), hi = lm_sel(ax > ay, ax, ay), lo = lm_sel(ax > ay, ay, ax), he, le, qe, h, hh, ll, qq;
  bool   big = hi > 0x1p500, small = hi < 0x1p-500;

  double sc = lm_sel(big, 0x1p-600, lm_sel(small, 0x1p600, 1.0));

  hi = hi * sc;
  lo = lo * sc;
  hh = lm_two_prod(hi, hi, &he);
  ll = lm_two_prod(lo, lo, &le);
  h = sqrt(hh + ll);
  qq = lm_two_prod(h, h, &qe);
  h = (h + (((hh - qq) + ll) + ((he + le) - qe)) / (2.0 * h)) * lm_sel(big, 0x1p600, lm_sel(small, 0x1p-600, 1.0));
  h = lm_sel(hi == 0.0, lo, h);
  h = lm_sel(ax == INFINITY, INFINITY, h);
  return lm_sel(ay == INFINITY, INFINITY, h);
}

#endif
