/** wbc_np.h - numpy's arithmetic from C, operation for operation, as numpy 2.5 and this host's
    libm give it: maximum, minimum, clip and sign; the pairwise sum; the orders its einsums add in
    here (its products are its OpenBLAS's own, wbc_blas.h). ISO C contracts nothing: an fma is
    asked by name. Portable C11, the host's alone. */
#ifndef WBC_NP_H
#define WBC_NP_H

#include <float.h>
#include <math.h>

/** The first where it wins, ties too, or is nan: Python's max and min; numpy's but at a tie of
    zeros (npmaximum, npminimum). */
static inline double npmax(double a, double b)
{
  return (a >= b || isnan(a)) ? a : b;
}

static inline double npmin(double a, double b)
{
  return (a <= b || isnan(a)) ? a : b;
}

/** numpy's maximum and minimum: the first where it wins outright or is nan, else the second -
    at a tie of zeros the second, as maxpd and minpd give it. */
static inline double npmaximum(double a, double b)
{
  return (a > b || isnan(a)) ? a : b;
}

static inline double npminimum(double a, double b)
{
  return (a < b || isnan(a)) ? a : b;
}

/** numpy's nan_to_num: nan 0, an infinity the largest finite of its sign. */
static inline double npnan_to_num(double x)
{
  return isnan(x) ? 0.0 : (isinf(x) ? (x > 0.0 ? DBL_MAX : -DBL_MAX) : x);
}

/** numpy's clip: x below lo lo, above hi hi, else x - a zero kept its sign. */
static inline double npclip(double x, double lo, double hi)
{
  return npmin(npmax(x, lo), hi);
}

/** numpy's remainder (np.mod, npy_divmod): fmod's, moved to the divisor's sign, a zero carrying it. */
static inline double npmod(double a, double b)
{
  const double m = fmod(a, b);

  return m != 0.0 ? m + b * (double)((b < 0.0) != (m < 0.0)) : copysign(0.0, b);
}

/** numpy's power where its loop sees the exponent at stride 0 (DOUBLE_power): 1 / x at -1, 1 at 0,
    sqrt at 0.5, x at 1, x * x at 2, else pow - numpy's npy_pow, the CRT's both. */
static inline double nppow(double x, double y)
{
  return y == -1.0 ? 1.0 / x : y == 0.0 ? 1.0 : y == 0.5 ? sqrt(x) : y == 1.0 ? x : y == 2.0 ? x * x : pow(x, y);
}

/** numpy's sign: 1, -1, 0 at either zero, nan at nan. */
static inline double npsign(double x)
{
  return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : (x == 0.0 ? 0.0 : x));
}

/** numpy's sum of a contiguous row: eight running sums under 129, halves over. */
static inline double npsum(const double *a, int n)
{
  double r[8], res;
  int    i, n2;

  if (n < 8)
  {
    res = 0.0;
    for (i = 0; i < n; i++)
    {
      res += a[i];
    }
    return res;
  }
  if (n <= 128)
  {
    for (i = 0; i < 8; i++)
    {
      r[i] = a[i];
    }
    for (i = 8; i < n - (n % 8); i += 8)
    {
      for (int j = 0; j < 8; j++)
      {
        r[j] += a[i + j];
      }
    }
    res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; i++)
    {
      res += a[i];
    }
    return res;
  }
  n2 = n / 2;
  n2 -= n2 % 8;
  return npsum(a, n2) + npsum(a + n2, n - n2);
}

/** numpy's pairwise sum of n at stride s, as npsum. */
static inline double npsum_s(const double *a, long n, long s)
{
  double r[8], res;
  long   i, n2;

  if (n < 8)
  {
    res = 0.0;
    for (i = 0; i < n; i++)
    {
      res += a[i * s];
    }
    return res;
  }
  if (n <= 128)
  {
    for (i = 0; i < 8; i++)
    {
      r[i] = a[i * s];
    }
    for (i = 8; i < n - (n % 8); i += 8)
    {
      for (long j = 0; j < 8; j++)
      {
        r[j] += a[(i + j) * s];
      }
    }
    res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
    for (; i < n; i++)
    {
      res += a[i * s];
    }
    return res;
  }
  n2 = n / 2;
  n2 -= n2 % 8;
  return npsum_s(a, n2, s) + npsum_s(a + n2 * s, n - n2, s);
}

/** Python's math.hypot of `n` finite, normal coordinates, eight at most (CPython's vector_norm):
    scaled, squared and summed losslessly, a differential correction. numpy's hypot is C's. */
static inline double pyhypot(const double *x, int n)
{
  double vec[8], top = 0.0, csum = 1.0, frac1 = 0.0, frac2 = 0.0, s, pr_hi, pr_lo, sm_hi, h, scale;
  int    top_e;

  for (int i = 0; i < n; i++)
  {
    vec[i] = fabs(x[i]);
    top = vec[i] > top ? vec[i] : top;
  }
  if (top == 0.0 || n <= 1)
  {
    return top;
  }
  frexp(top, &top_e);
  scale = ldexp(1.0, -top_e);
  for (int i = 0; i < n; i++)
  {
    s = vec[i] * scale;
    pr_hi = s * s;
    pr_lo = fma(s, s, -pr_hi);
    sm_hi = csum + pr_hi;
    frac2 += (csum - sm_hi) + pr_hi;
    csum = sm_hi;
    frac1 += pr_lo;
  }
  h = sqrt(csum - 1.0 + (frac1 + frac2));
  pr_hi = (-h) * h;
  pr_lo = fma(-h, h, -pr_hi);
  sm_hi = csum + pr_hi;
  frac2 += (csum - sm_hi) + pr_hi;
  csum = sm_hi;
  frac1 += pr_lo;
  s = csum - 1.0 + (frac1 + frac2);
  h += s / (2.0 * h);
  return h / scale;
}

/** A term of a run, a b, added to `acc` - rounded then added, or fused into its add. */
static inline double npterm(double a, double b, double acc, int fused)
{
  return fused != 0 ? fma(a, b, acc) : acc + a * b;
}

/** An einsum's run of n terms a[k sa] b[k sb] summed as numpy's sum_of_products sums it here (`order`, an index of
    tools.cores.orders.ORDERS): from zero in turn; or in 2 or 4 SIMD lanes, four vectors a step - the last first -,
    then one at a time, the lanes past its end adding zero, summed across from zero; each product rounded then
    added, or (an Arm's NEON, from index 3) fused into its add. */
static inline double npsumof(const double *a, int sa, const double *b, int sb, int n, int order)
{
  const int fused = order >= 3 ? 1 : 0;
  const int v = (order == 1 || order == 4) ? 2 : (order == 2 ? 4 : 1);
  double    lanes[4] = {0.0, 0.0, 0.0, 0.0};
  int       t = 0;

  if (v == 1)
  {
    double acc = 0.0;

    for (int k = 0; k < n; k++)
    {
      acc = npterm(a[k * sa], b[k * sb], acc, fused);
    }
    return acc;
  }
  for (; n - t >= 4 * v; t += 4 * v)
  {
    for (int k = 0; k < v; k++)
    {
      for (int u = 3; u >= 0; u--)
      {
        lanes[k] = npterm(a[(t + u * v + k) * sa], b[(t + u * v + k) * sb], lanes[k], fused);
      }
    }
  }
  for (; t < n; t += v)
  {
    for (int k = 0; k < v; k++)
    {
      lanes[k] = t + k < n ? npterm(a[(t + k) * sa], b[(t + k) * sb], lanes[k], fused) : lanes[k] + 0.0;
    }
  }
  return 0.0 + (v == 2 ? lanes[0] + lanes[1] : (lanes[0] + lanes[2]) + (lanes[1] + lanes[3]));
}

/** einsum 'nia,ia->n' of six, contiguous: one run of six in numpy's `order` here (wbc_blas_t's einsum[0]). */
static inline double npein6(const double *h, const double *p, int order)
{
  return npsumof(h, 1, p, 1, 6, order);
}

/** numpy's eye: 1 at mode `m`, else 0. */
static inline double npeye(int n, int m)
{
  return n == m ? 1.0 : 0.0;
}

/** einsum 'nia,n->ia' of `h` (n, 2, 3) and `w`: each of its six a run down n in numpy's `order` here (wbc_blas_t's
    einsum[1]). */
static inline void npein(int n, const double *h, const double *w, double got[6], int order)
{
  for (int i = 0; i < 6; i++)
  {
    got[i] = npsumof(h + i, 6, w, 1, n, order);
  }
}

#endif
