/** wbc_libm.c - wbc_libm.h's functions over arrays, and the reduction of sin's, cos's and tan's arguments past 2^22.

    Each function's array is a run: one loop that calls the function once, so that a compiler inlines it however
    large and lays the loop in vector registers (sin, cos and tan reduced first, near for every element and far for
    the few past 2^22, their kernels then in a loop of their own), over chunks of LM_CHUNK straight from the
    caller's array, so that a chunk stays in the first cache between the loops; the last elements under a block
    through a block of its own on the stack, its other lanes zero - or, under half a block (LM_FEW), through the run
    as they are, a call each: an array of one to three costs what as many calls do. Portable C11, no allocation.

    This file alone is told by gcc that no floating trap is read (none is, anywhere: no fenv) and to vectorise a loop
    where its cost model finds it pays: without the first, gcc's vectorizer leaves a select the optimizer split as a
    branch (x86: 7 of the 13 loops); without the second, -O2's cheapest model takes none of them on ARMv9 (SVE2,
    SME2), where they run twice as fast as a call at a time. The cores' own flags stay as they were: across the robot's
    loop the cheap model cost 1 to 3 % of its pace. Neither changes a value; another compiler reads no pragma here. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("no-trapping-math", "vect-cost-model=cheap")
#endif

#include "wbc_libm_arc.h"

#define LM_CHUNK 256 /* a run's elements at most: 2 kB of input, of output, of each reduction's part */
/* A tail under half a block goes through its run as it is, a call an element (the loop's own remainder): a block of
   its own costs the whole block's lanes, 40 to 85 ns for one element where a call is 11 to 42. */
#define LM_FEW (WBC_LIBM_LANES / 2)

/** 2^(32k - 190): the weight of the fraction's word k. */
static const double lm_far_weight[6] = {0x1p-190, 0x1p-158, 0x1p-126, 0x1p-94, 0x1p-62, 0x1p-30};

/** 2/pi's word i after two words of zeros, so that an exponent down to -30 finds its window's start. */
static uint32_t lm_g(uint32_t i)
{
  return (i < 2U) ? 0U : lm_two_over_pi_words[i - 2U];
}

/** x - k pi/2 as the value and *lo for |x| >= 2^22, k's low two bits *q. x = m 2^e with m an integer of 53 bits;
    the 192 bits of 2/pi from e's place on (above them m 2/pi 2^e is a multiple of 4) times m, held to 192 bits,
    are k mod 4 (as a double, *q) over a fraction of 190; a fraction past 1/2 is taken less one, k one more; the
    fraction as a sum of two by Knuth's sums, times pi/2 by Dekker's product. An infinity or a NaN gives a NaN. */
double lm_reduce_far(double x, double *lo, double *q)
{
  uint64_t u = lm_bits(x), m = (u & 0x000FFFFFFFFFFFFFU) | 0x0010000000000000U, t, c = 0U;
  uint32_t ue = (uint32_t)(u >> 52) & 0x7FFU, at = (ue > 1013U) ? (ue - 1013U) : 0U, wi = at >> 5, sh = at & 31U;
  uint32_t r[6], f[6], m0 = (uint32_t)m, m1 = (uint32_t)(m >> 32), qq;
  double   h = 0.0, l = 0.0, v, s, bb, p, pe;
  bool     neg, flip;
  int32_t  k;

  for (k = 0; k < 6; k++)
  {
    t = ((((uint64_t)lm_g(wi + (uint32_t)k)) << 32) | lm_g(wi + (uint32_t)k + 1U)) << sh;
    r[k] = (uint32_t)(t >> 32);
  }
  for (k = 0; k < 6; k++)
  {
    t = ((uint64_t)m0 * r[5 - k]) + c;
    f[k] = (uint32_t)t;
    c = t >> 32;
  }
  c = 0U;
  for (k = 1; k < 6; k++)
  {
    t = (((uint64_t)m1 * r[6 - k]) + f[k]) + c;
    f[k] = (uint32_t)t;
    c = t >> 32;
  }
  qq = f[5] >> 30;
  f[5] = f[5] & 0x3FFFFFFFU;
  neg = (f[5] & 0x20000000U) != 0U;
  if (neg)
  {
    c = 1U;
    for (k = 0; k < 6; k++)
    {
      t = (uint64_t)(f[k] ^ 0xFFFFFFFFU) + c;
      f[k] = (uint32_t)t;
      c = t >> 32;
    }
    f[5] = f[5] & 0x3FFFFFFFU;
    qq = qq + 1U;
  }
  for (k = 6; k > 0; k--)
  {
    v = (double)f[k - 1] * lm_far_weight[k - 1];
    s = h + v;
    bb = s - h;
    l = l + ((h - (s - bb)) + (v - bb));
    h = s + l;
    l = l - (h - s);
  }
  p = lm_two_prod(h, lm_pi_2[0], &pe);
  pe = pe + ((h * lm_pi_2[1]) + (l * lm_pi_2[0]));
  h = p + pe;
  l = pe - (h - p);
  flip = neg != (x < 0.0);
  *lo = (fabs(x) < INFINITY) ? (flip ? -l : l) : 0.0;
  *q = (double)(((x < 0.0) ? (0U - qq) : qq) & 3U);
  return (fabs(x) < INFINITY) ? (flip ? -h : h) : (x - x);
}

/** A run of one input: n elements, each written once. */
typedef void (*lm_run1_t)(const double *restrict x, double *restrict y, int32_t n);

/** A run of two inputs. */
typedef void (*lm_run2_t)(const double *restrict a, const double *restrict b, double *restrict y, int32_t n);

/** x's n elements by run: whole blocks a chunk at a time from x itself, the rest through a block of zeros - a few
    (under LM_FEW) through the run itself. */
static void lm_drive1(lm_run1_t run, const double *restrict x, double *restrict y, int32_t n)
{
  double  xb[WBC_LIBM_LANES] = {0.0}, yb[WBC_LIBM_LANES];
  int32_t whole = n - (n % WBC_LIBM_LANES), i, j;

  for (i = 0; i < whole; i += LM_CHUNK)
  {
    run(&x[i], &y[i], ((whole - i) < LM_CHUNK) ? (whole - i) : LM_CHUNK);
  }
  if ((n - whole) > 0 && (n - whole) < LM_FEW)
  {
    run(&x[whole], &y[whole], n - whole);
  }
  else if (whole < n)
  {
    for (j = 0; j < (n - whole); j++)
    {
      xb[j] = x[whole + j];
    }
    run(xb, yb, WBC_LIBM_LANES);
    for (j = 0; j < (n - whole); j++)
    {
      y[whole + j] = yb[j];
    }
  }
}

/** The same for two inputs. */
static void lm_drive2(lm_run2_t run, const double *restrict a, const double *restrict b, double *restrict y, int32_t n)
{
  double  ab[WBC_LIBM_LANES] = {0.0}, bb[WBC_LIBM_LANES] = {0.0}, yb[WBC_LIBM_LANES];
  int32_t whole = n - (n % WBC_LIBM_LANES), i, j;

  for (i = 0; i < whole; i += LM_CHUNK)
  {
    run(&a[i], &b[i], &y[i], ((whole - i) < LM_CHUNK) ? (whole - i) : LM_CHUNK);
  }
  if ((n - whole) > 0 && (n - whole) < LM_FEW)
  {
    run(&a[whole], &b[whole], &y[whole], n - whole);
  }
  else if (whole < n)
  {
    for (j = 0; j < (n - whole); j++)
    {
      ab[j] = a[whole + j];
      bb[j] = b[whole + j];
    }
    run(ab, bb, yb, WBC_LIBM_LANES);
    for (j = 0; j < (n - whole); j++)
    {
      y[whole + j] = yb[j];
    }
  }
}

/** A run's reductions into h (the output's place), lo and q: the near one for every element, then the far one for
    each argument past 2^22 (or not finite), so that each kernel stands once, in one loop. */
static void lm_reduce_run(const double *restrict x, double *restrict h, double *restrict lo, double *restrict q, int32_t n)
{
  int32_t i;

  for (i = 0; i < n; i++)
  {
    h[i] = lm_reduce(x[i], &lo[i], &q[i]);
  }
  for (i = 0; i < n; i++)
  {
    if (!(fabs(x[i]) < LM_FAR))
    {
      h[i] = lm_reduce_far(x[i], &lo[i], &q[i]);
    }
  }
}

static void lm_run_sin(const double *restrict x, double *restrict y, int32_t n)
{
  double lo[LM_CHUNK], q[LM_CHUNK];

  lm_reduce_run(x, y, lo, q, n);
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = lm_sin_at(y[i], lo[i], q[i], x[i]);
  }
}

void wbc_libm_sin(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_sin, x, y, n);
}

static void lm_run_cos(const double *restrict x, double *restrict y, int32_t n)
{
  double lo[LM_CHUNK], q[LM_CHUNK];

  lm_reduce_run(x, y, lo, q, n);
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = lm_cos_at(y[i], lo[i], q[i]);
  }
}

void wbc_libm_cos(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_cos, x, y, n);
}

static void lm_run_tan(const double *restrict x, double *restrict y, int32_t n)
{
  double lo[LM_CHUNK], q[LM_CHUNK];

  lm_reduce_run(x, y, lo, q, n);
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = lm_tan_at(y[i], lo[i], q[i], x[i]);
  }
}

void wbc_libm_tan(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_tan, x, y, n);
}

/** sin and cos of a run from one reduction. */
static void lm_run_sincos(const double *restrict x, double *restrict s, double *restrict c, int32_t n)
{
  double lo[LM_CHUNK], q[LM_CHUNK], h;

  lm_reduce_run(x, s, lo, q, n);
  for (int32_t i = 0; i < n; i++)
  {
    h = s[i];
    s[i] = lm_sin_at(h, lo[i], q[i], x[i]);
    c[i] = lm_cos_at(h, lo[i], q[i]);
  }
}

void wbc_libm_sincos(const double *restrict x, double *restrict s, double *restrict c, int32_t n)
{
  double  xb[WBC_LIBM_LANES] = {0.0}, sb[WBC_LIBM_LANES], cb[WBC_LIBM_LANES];
  int32_t whole = n - (n % WBC_LIBM_LANES), i, j;

  for (i = 0; i < whole; i += LM_CHUNK)
  {
    lm_run_sincos(&x[i], &s[i], &c[i], ((whole - i) < LM_CHUNK) ? (whole - i) : LM_CHUNK);
  }
  if ((n - whole) > 0 && (n - whole) < LM_FEW)
  {
    lm_run_sincos(&x[whole], &s[whole], &c[whole], n - whole);
  }
  else if (whole < n)
  {
    for (j = 0; j < (n - whole); j++)
    {
      xb[j] = x[whole + j];
    }
    lm_run_sincos(xb, sb, cb, WBC_LIBM_LANES);
    for (j = 0; j < (n - whole); j++)
    {
      s[whole + j] = sb[j];
      c[whole + j] = cb[j];
    }
  }
}

static void lm_run_exp(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_exp(x[i]);
  }
}

void wbc_libm_exp(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_exp, x, y, n);
}

static void lm_run_expm1(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_expm1(x[i]);
  }
}

void wbc_libm_expm1(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_expm1, x, y, n);
}

static void lm_run_tanh(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_tanh(x[i]);
  }
}

void wbc_libm_tanh(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_tanh, x, y, n);
}

static void lm_run_log(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_log(x[i]);
  }
}

void wbc_libm_log(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_log, x, y, n);
}

static void lm_run_arctan(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_arctan(x[i]);
  }
}

void wbc_libm_arctan(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_arctan, x, y, n);
}

static void lm_run_arcsin(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_arcsin(x[i]);
  }
}

void wbc_libm_arcsin(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_arcsin, x, y, n);
}

static void lm_run_arccos(const double *restrict x, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_arccos(x[i]);
  }
}

void wbc_libm_arccos(const double *restrict x, double *restrict y, int32_t n)
{
  lm_drive1(lm_run_arccos, x, y, n);
}

static void lm_run_arctan2(const double *restrict a, const double *restrict b, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_arctan2(a[i], b[i]);
  }
}

void wbc_libm_arctan2(const double *restrict y, const double *restrict x, double *restrict out, int32_t n)
{
  lm_drive2(lm_run_arctan2, y, x, out, n);
}

static void lm_run_hypot(const double *restrict a, const double *restrict b, double *restrict y, int32_t n)
{
  for (int32_t i = 0; i < n; i++)
  {
    y[i] = wbc_hypot(a[i], b[i]);
  }
}

void wbc_libm_hypot(const double *restrict x, const double *restrict y, double *restrict out, int32_t n)
{
  lm_drive2(lm_run_hypot, x, y, out, n);
}
