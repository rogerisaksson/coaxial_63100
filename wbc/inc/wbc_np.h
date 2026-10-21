/** wbc_np.h - numpy's arithmetic from C, operation for operation, as numpy 2.5, its OpenBLAS
    and this host's libm give it: maximum, minimum, clip and sign; the pairwise sum; the orders
    its small matmuls, dots and einsums add in. ISO C contracts nothing: an fma is asked by name.
    Portable C11, the host's alone. */
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

/** A 1-d dot of two, a's at stride sa, b's at sb (ddot): the plain sum from zero. A vector @
    a matrix, a transposed one's @ a vector (dgemv_n) adds the same way, k long. */
static inline double npdot(const double *a, int sa, const double *b, int sb, int k)
{
  double t = 0.0;

  for (int i = 0; i < k; i++)
  {
    t += a[i * sa] * b[i * sb];
  }
  return 0.0 + t;
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

/** A row of a matrix @ a vector, three long or two (dgemv_t's tail, contracted). */
static inline double npmv3(const double *a, const double *x)
{
  return 0.0 + fma(a[2], x[2], fma(a[0], x[0], a[1] * x[1]));
}

static inline double npmv2(const double *a, const double *x)
{
  return 0.0 + fma(a[0], x[0], a[1] * x[1]);
}

/** An entry of a small matmul, three long: a's row and b's column at stride s (dgemm's fma
    chain) - 1 against a transposed matrix's row, 3 down a (3, 3)'s column. */
static inline double npmm3(const double *a, const double *b, int s)
{
  return fma(a[2], b[2 * s], fma(a[1], b[s], a[0] * b[0]));
}

/** einsum 'nia,ia->n' of six, contiguous: SSE's two lanes, then their sum. */
static inline double npein6(const double *h, const double *p)
{
  double l0 = 0.0, l1 = 0.0;

  for (int i = 0; i < 6; i += 2)
  {
    l0 = h[i] * p[i] + l0;
    l1 = h[i + 1] * p[i + 1] + l1;
  }
  return 0.0 + (l0 + l1);
}

/** numpy's eye: 1 at mode `m`, else 0. */
static inline double npeye(int n, int m)
{
  return n == m ? 1.0 : 0.0;
}

/** a (r, 3) @ b.T, b (3, 3). */
static inline void npmmt(const double *a, int r, const double *b, double *got)
{
  for (int i = 0; i < r; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      got[3 * i + j] = npmm3(a + 3 * i, b + 3 * j, 1);
    }
  }
}

/** a (2, 3) @ b, b (3, 3). */
static inline void npmm(const double *a, const double *b, double *got)
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      got[3 * i + j] = npmm3(a + 3 * i, b + j, 3);
    }
  }
}

/** einsum 'nia,n->ia' of `h` (n, 2, 3) and `w`, then each of its six. */
static inline void npein(int n, const double *h, const double *w, double got[6])
{
  for (int i = 0; i < 6; i++)
  {
    got[i] = 0.0;
  }
  for (int m = 0; m < n; m++)
  {
    for (int i = 0; i < 6; i++)
    {
      got[i] = h[6 * m + i] * w[m] + got[i];
    }
  }
}

#endif
