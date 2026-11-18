/** wbc_libm.h - the robot's own elementary functions: the same bits on every machine, whatever its libm.

    Each function is IEEE 754's basic operations and square root alone (the build contracts no FMA), in an order
    the Python twin (machine/libm.py) takes step for step, so that the two agree to the bit; the polynomials'
    coefficients and the tables are tools/cores/libm.py's (wbc_libm_tables.h). Every function is branchless over
    its whole range but sin's, cos's and tan's past 2^22, whose argument is reduced by 2/pi's bits in wbc_libm.c
    (lm_reduce_far): a call there, a select everywhere else, so that a loop over an array keeps to vector
    registers (the lm_*_near forms, wbc_libm.c's loops). Errors stay under one unit in the last place; zeros,
    infinities and NaNs answer as C11's Annex F asks.

    A double's bits are read through a union (C11 6.5.2.3, footnote 95): MISRA C:2012's Rule 19.2, advisory, is
    disapplied for lm_word_t alone, the one reinterpretation a compiler keeps in a vector register. */
#ifndef WBC_LIBM_H
#define WBC_LIBM_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "wbc_libm_tables.h"

#define LM_MAGIC 6755399441055744.0 /* 1.5 * 2^52: (x + LM_MAGIC) - LM_MAGIC is x's nearest integer, |x| < 2^51 */
#define LM_SPLIT 134217729.0        /* 2^27 + 1: Veltkamp's split of a double into halves of 26 and 27 bits */
#define LM_FAR 0x1p22               /* from here on sin's, cos's and tan's argument is reduced by 2/pi's bits */
#define WBC_LIBM_LANES 8            /* the arrays' block: two AVX2 vectors, one AVX-512 */

typedef union
{
  double   d;
  uint64_t u;
} lm_word_t;

static inline uint64_t lm_bits(double x)
{
  lm_word_t w;

  w.d = x;
  return w.u;
}

static inline double lm_double(uint64_t u)
{
  lm_word_t w;

  w.u = u;
  return w.d;
}

/** x is a NaN, by comparison alone: some C libraries' isnan and isinf are calls, which no loop vectorizes. */
static inline bool lm_nan(double x)
{
  return x != x;
}

/** c ? a : b by the bits under a mask of all ones or none, c a floating comparison's: no branch for a compiler to
    split, so no select's arms merge into one choice of more than four a vectorizer converts (gcc's limit); no
    integer bit's truth, which a vectorizer takes for a one-bit type it cannot lay; no negated 0 or 1, which a
    compiler turns into a 64-bit multiplication NEON and AVX lack. A blend on every vector machine. */
static inline double lm_sel(bool c, double a, double b)
{
  uint64_t m = c ? 0xFFFFFFFFFFFFFFFFU : 0U;

  return lm_double((lm_bits(a) & m) | (lm_bits(b) & ~m));
}

/** a's magnitude with b's sign, by the bits: some C libraries' copysign works on halves of a word, no vector's. */
static inline double lm_copysign(double a, double b)
{
  return lm_double((lm_bits(a) & 0x7FFFFFFFFFFFFFFFU) | (lm_bits(b) & 0x8000000000000000U));
}

/** a b rounded, and *err that rounding's error, exact (Dekker's product over Veltkamp's halves). */
static inline double lm_two_prod(double a, double b, double *err)
{
  double p = a * b, ga = LM_SPLIT * a, gb = LM_SPLIT * b;
  double ah = ga - (ga - a), bh = gb - (gb - b), al = a - ah, bl = b - bh;

  *err = (((ah * bh - p) + ah * bl) + al * bh) + al * bl;
  return p;
}

/** c[0] + c[1] z + c[2] z^2 + c[3] z^3 by Estrin's pairs; each lm_eN joins its halves by z's powers alike. */
static inline double lm_e4(const double *c, double z)
{
  return (c[0] + c[1] * z) + (z * z) * (c[2] + c[3] * z);
}

static inline double lm_e6(const double *c, double z)
{
  double z2 = z * z;

  return ((c[0] + c[1] * z) + z2 * (c[2] + c[3] * z)) + (z2 * z2) * (c[4] + c[5] * z);
}

static inline double lm_e8(const double *c, double z)
{
  double z2 = z * z;

  return ((c[0] + c[1] * z) + z2 * (c[2] + c[3] * z)) + (z2 * z2) * ((c[4] + c[5] * z) + z2 * (c[6] + c[7] * z));
}

static inline double lm_e11(const double *c, double z)
{
  double z2 = z * z, z4 = z2 * z2;

  return lm_e8(c, z) + (z4 * z4) * ((c[8] + c[9] * z) + z2 * c[10]);
}

static inline double lm_e12(const double *c, double z)
{
  double z2 = z * z, z4 = z2 * z2;

  return lm_e8(c, z) + (z4 * z4) * lm_e4(&c[8], z);
}

static inline double lm_e14(const double *c, double z)
{
  double z2 = z * z, z4 = z2 * z2;

  return lm_e8(c, z) + (z4 * z4) * lm_e6(&c[8], z);
}

/** sin(x + y) for |x| <= pi/4 and y under x's last place, fdlibm's order: x + x^3 S(x^2) + y (1 - x^2/2); the
    sum rounded, *lo its rounding's error. */
static inline double lm_ksin(double x, double y, double *lo)
{
  double z = x * x, v = z * x;
  double t = (z * (0.5 * y - v * lm_e6(&lm_sin[1], z)) - y) - v * lm_sin[0], s = x - t;

  *lo = (x - s) - t;
  return s;
}

/** cos(x + y) likewise: 1 - x^2/2 + x^4 C(x^2) - x y, 1 - x^2/2's rounding carried. */
static inline double lm_kcos(double x, double y, double *lo)
{
  double z = x * x, hz = 0.5 * z, w = 1.0 - hz;
  double t = ((1.0 - w) - hz) + (z * (z * lm_e6(lm_cos, z)) - x * y), c = w + t;

  *lo = (w - c) + t;
  return c;
}

/** x - k pi/2 for |x| < 2^22 as the sum of the value and *lo: k = rint(2x/pi), k mod 4 *q (k less 4 rint(k/4 -
    3/8), each step exact); pi/2's two high parts times k exact, the second difference's rounding kept (Knuth's). */
static inline double lm_reduce(double x, double *lo, double *q)
{
  double t = x * lm_two_over_pi[0] + LM_MAGIC, k = t - LM_MAGIC, a = x - k * lm_pio2[0];
  double w = -(k * lm_pio2[1]), b = a + w, bb = b - a;
  double l = ((a - (b - bb)) + (w - bb)) - k * lm_pio2[2], h = b + l;

  *lo = l - (h - b);
  *q = k - 4.0 * (((0.25 * k - 0.375) + LM_MAGIC) - LM_MAGIC);
  return h;
}

/** The same past 2^22 by 2/pi's bits; an infinity or a NaN gives a NaN (wbc_libm.c). */
double lm_reduce_far(double x, double *lo, double *q);

/** Either reduction, by |x|. */
static inline double lm_reduced(double x, double *lo, double *q)
{
  return (fabs(x) < LM_FAR) ? lm_reduce(x, lo, q) : lm_reduce_far(x, lo, q);
}

/** The quadrant's pick of s or c, signed, for q in 0..3: sin's at q, cos's at q + 1 (mod 4). */
static inline double lm_quadrant(double s, double c, double q)
{
  bool   neg = q >= 2.0;
  double v = lm_sel((q - lm_sel(neg, 2.0, 0.0)) > 0.5, c, s);

  return lm_sel(neg, -v, v);
}

static inline double lm_sin_at(double h, double lo, double q, double x)
{
  double sl, cl, v = lm_quadrant(lm_ksin(h, lo, &sl), lm_kcos(h, lo, &cl), q);

  return lm_sel(x == 0.0, x, v);
}

static inline double lm_cos_at(double h, double lo, double q)
{
  double sl, cl, q1 = q + 1.0;

  return lm_quadrant(lm_ksin(h, lo, &sl), lm_kcos(h, lo, &cl), q1 - lm_sel(q1 >= 4.0, 4.0, 0.0));
}

/** tan as s/c on an even quadrant and -c/s on an odd one, each a sum of two; the quotient's rounding corrected by
    its residual, exact by Dekker's product. */
static inline double lm_tan_at(double h, double lo, double q, double x)
{
  double sl, cl, pe, s = lm_ksin(h, lo, &sl), c = lm_kcos(h, lo, &cl);
  bool   odd = (q - lm_sel(q >= 2.0, 2.0, 0.0)) > 0.5;
  double n = lm_sel(odd, -c, s), nl = lm_sel(odd, -cl, sl), d = lm_sel(odd, s, c), dl = lm_sel(odd, sl, cl);
  double t = n / d;
  double p = lm_two_prod(t, d, &pe);

  t = t + (((n - p) - pe) + (nl - t * dl)) / d;
  return lm_sel(x == 0.0, x, t);
}

static inline double lm_sin_near(double x)
{
  double lo, h, q;

  h = lm_reduce(x, &lo, &q);
  return lm_sin_at(h, lo, q, x);
}

static inline double lm_cos_near(double x)
{
  double lo, h, q;

  h = lm_reduce(x, &lo, &q);
  return lm_cos_at(h, lo, q);
}

static inline double lm_tan_near(double x)
{
  double lo, h, q;

  h = lm_reduce(x, &lo, &q);
  return lm_tan_at(h, lo, q, x);
}

static inline double wbc_sin(double x)
{
  double lo, h, q;

  h = lm_reduced(x, &lo, &q);
  return lm_sin_at(h, lo, q, x);
}

static inline double wbc_cos(double x)
{
  double lo, h, q;

  h = lm_reduced(x, &lo, &q);
  return lm_cos_at(h, lo, q);
}

static inline double wbc_tan(double x)
{
  double lo, h, q;

  h = lm_reduced(x, &lo, &q);
  return lm_tan_at(h, lo, q, x);
}

static inline void wbc_sincos(double x, double *s, double *c)
{
  double lo, h, q;

  h = lm_reduced(x, &lo, &q);
  *s = lm_sin_at(h, lo, q, x);
  *c = lm_cos_at(h, lo, q);
}

/** exp's and expm1's common part for x held in their range (a NaN kept): x = k ln2 + r + *rr with |r| <= ln2/2
    (ln 2's high part times k exact, the sum's rounding kept), e^r = 1 + r + *q; k's bits *k, floor(k/2)'s *k1. */
static inline double lm_exp_core(double xc, double *q, double *rr, uint64_t *k, uint64_t *k1)
{
  double   t = xc * lm_inv_ln2[0] + LM_MAGIC, n = t - LM_MAGIC, rh = xc - n * lm_ln2[0], rl = -(n * lm_ln2[1]);
  double   r = rh + rl;

  *rr = (rh - r) + rl;
  *q = (r * r) * lm_e11(lm_exp, r);
  *k = lm_bits(t);
  *k1 = lm_bits((0.5 * n - 0.25) + LM_MAGIC);
  return r;
}

/** 2^(j + b - 1023) for an integer j in a word's low bits (LM_MAGIC + j's, or a difference of two such): j + b
    laid into the exponent's field, what lies above it shifted out. */
static inline double lm_pow2(uint64_t j, uint64_t b)
{
  return lm_double((j + b) << 52);
}

/** e^x = 2^k (1 + r + q): 1 + r by Knuth's sum, 2^k as 2^floor(k/2) 2^ceil(k/2) so that each factor is normal; |x|
    held to 746, past exp's range either way (a NaN fails the comparison and stays). No branch, no table. */
static inline double wbc_exp(double x)
{
  double   q, rr, r, s;
  uint64_t k, k1;

  r = lm_exp_core(lm_sel(fabs(x) > 746.0, lm_copysign(746.0, x), x), &q, &rr, &k, &k1);
  s = 1.0 + r;
  s = s + (((1.0 - s) + r) + (q + rr * (1.0 + r)));
  return (s * lm_pow2(k1, 1023U)) * lm_pow2(k - k1, 1023U);
}

/** e^x - 1 = (2^k1 (1 + r + q) - 2^-k2) 2^k2, k = k1 + k2: 2^k1 - 2^-k2 and the r term summed exactly (Knuth's), so
    that no rounding precedes the last but q's; the sign that of x (its zeros); |x| held to 40 below 0, 746 above. */
static inline double wbc_expm1(double x)
{
  double   q, rr, r, f, b, h, bb, lo, ar, s;
  uint64_t k, k1;

  r = lm_exp_core(lm_sel(x > 746.0, 746.0, lm_sel(x < -40.0, -40.0, x)), &q, &rr, &k, &k1);
  f = lm_pow2(k1, 1023U);
  b = -lm_pow2(0U - (k - k1), 1023U);
  h = f + b;
  bb = h - f;
  lo = (f - (h - bb)) + (b - bb);
  ar = f * r;
  s = h + ar;
  bb = s - h;
  s = (s + (((h - (s - bb)) + (ar - bb)) + (lo + f * (q + rr * (1.0 + r))))) * lm_pow2(k - k1, 1023U);
  return lm_copysign(s, x);
}

/** tanh x = E / (E + 2), E = e^(2|x|) - 1 summed as wbc_expm1 sums it and kept in two parts, nh + nl, before its last
    rounding; the quotient of nh + nl by nh + nl + 2 (both over 2^ceil(k/2)) one division and its residual by
    Dekker's product. |x| held to 22, past which tanh rounds to 1; the sign that of x (its zeros). */
static inline double wbc_tanh(double x)
{
  double   q, rr, r, f, b, h, bb, lo, ar, s, c, nh, nl, g, dh, dl, qh, p, pe;
  uint64_t k, k1;

  r = lm_exp_core(2.0 * lm_sel(fabs(x) > 22.0, 22.0, fabs(x)), &q, &rr, &k, &k1);
  f = lm_pow2(k1, 1023U);
  b = -lm_pow2(0U - (k - k1), 1023U);
  h = f + b;
  bb = h - f;
  lo = (f - (h - bb)) + (b - bb);
  ar = f * r;
  s = h + ar;
  bb = s - h;
  c = ((h - (s - bb)) + (ar - bb)) + (lo + f * (q + rr * (1.0 + r)));
  nh = s + c;
  bb = nh - s;
  nl = (s - (nh - bb)) + (c - bb);
  g = -2.0 * b;
  dh = nh + g;
  bb = dh - nh;
  dl = ((nh - (dh - bb)) + (g - bb)) + nl;
  qh = nh / dh;
  p = lm_two_prod(qh, dh, &pe);
  qh = qh + ((((nh - p) - pe) + nl) - qh * dl) / dh;
  return lm_sel(lm_nan(x), x, lm_copysign(qh, x));
}

/** log x, fdlibm's: x = 2^k (1 + f), 1 + f in [sqrt(2)/2, sqrt(2)) by x's bits (a subnormal scaled by 2^54 first), s
    = f/(2 + f), log(1 + f) = f - f^2/2 + s (f^2/2 + R(s^2)) or f - s (f - R) as fdlibm picks by f; k ln 2's high
    part exact. */
static inline double wbc_log(double x)
{
  bool     sub = x < 0x1p-1022;
  uint64_t u = lm_bits(lm_sel(sub, x * 0x1p54, x)), hx = (u >> 32) & 0xFFFFFU, i = (hx + 0x95F64U) & 0x100000U;
  double   f = lm_double((u & 0x000FFFFFFFFFFFFFU) | ((i ^ 0x3FF00000U) << 32)) - 1.0;
  double   dk = (lm_double(((u >> 52) + (i >> 20)) | 0x4330000000000000U) - 4503599627371519.0) - lm_sel(sub, 54.0, 0.0);
  double   s = f / (2.0 + f), z = s * s, w = z * z, hf = (0.5 * f) * f;
  double   r = z * (lm_log[0] + w * (lm_log[2] + w * (lm_log[4] + w * lm_log[6]))) +
             w * (lm_log[1] + w * (lm_log[3] + w * lm_log[5]));
  double   m0 = lm_double((u & 0x000FFFFFFFFFFFFFU) | 0x3FF0000000000000U);
  double   y = lm_sel((m0 >= 0x1.6147Bp0) && (m0 < 0x1.6B851p0),
                      dk * lm_ln2[0] - ((hf - (s * (hf + r) + dk * lm_ln2[1])) - f),
                      dk * lm_ln2[0] - ((s * (f - r) - dk * lm_ln2[1]) - f));

  y = lm_sel(x < 0.0, NAN, y);
  y = lm_sel(x == 0.0, -INFINITY, y);
  y = lm_sel(x == INFINITY, x, y);
  return lm_sel(lm_nan(x), x, y);
}

void wbc_libm_sin(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_cos(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_tan(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_sincos(const double *restrict x, double *restrict s, double *restrict c, int32_t n);
void wbc_libm_exp(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_expm1(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_tanh(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_log(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_arctan(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_arcsin(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_arccos(const double *restrict x, double *restrict y, int32_t n);
void wbc_libm_arctan2(const double *restrict y, const double *restrict x, double *restrict out, int32_t n);
void wbc_libm_hypot(const double *restrict x, const double *restrict y, double *restrict out, int32_t n);

#endif
