/** wbc_gmo_factor.c - the observer's split's factor kept as pushes join and leave. */
#include "wbc_gmo_factor.h"
#include "wbc_libm_arc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* eps top lies on K's diagonal, so no Schur complement of it is smaller than `least`. 53 factorizations of 26 a
 * step from scratch were half the robot's observer's time. */
int wbc_gmo_joined(wbc_gmo_t *restrict o, int *restrict at, int n, int b, double least)
{
  double *restrict       kn = o->k[n];
  const double *restrict hb = o->h[b];
  double                 d = hb[b];

  /* each of the row's entries less its terms along its row of K in their order, mirrored into L^T above the diagonal */
  for (int y = 0; y < n; y++)
  {
    const double *restrict ky = o->k[y];
    double                 s = hb[at[y]];
    int                    x = 0;

    for (; x + 4 <= y; x += 4)
    {
      const double p0 = kn[x] * ky[x];
      const double p1 = kn[x + 1] * ky[x + 1];
      const double p2 = kn[x + 2] * ky[x + 2];
      const double p3 = kn[x + 3] * ky[x + 3];

      s = (((s - p0) - p1) - p2) - p3;
    }
    for (; x < y; x++)
    {
      s -= kn[x] * ky[x];
    }
    const double ky_n = s / ky[y];

    kn[y] = ky_n;
    o->k[y][n] = ky_n;
    d -= ky_n * ky_n;
  }
  at[n] = b;
  kn[n] = sqrt(fmax(d, least));
  return n + 1;
}

/* L L' is K less the push's row and column. */
int wbc_gmo_left(wbc_gmo_t *restrict o, int *restrict at, int n, int p)
{
  for (int r = p; r < n - 1; r++)
  {
    at[r] = at[r + 1];
    memcpy(o->k[r], o->k[r + 1], (size_t)(r + 2) * sizeof(double));
  }
  for (int j = p; j < n - 1; j++)
  {
    const double a = o->k[j][j], b = o->k[j][j + 1], r = wbc_hypot(a, b), c = a / r, s = b / r;

    for (int i = j; i < n - 1; i++)
    {
      const double t1 = o->k[i][j], t2 = o->k[i][j + 1];

      o->k[i][j] = c * t1 + s * t2;
      o->k[i][j + 1] = c * t2 - s * t1;
    }
  }
  for (int r = 1; r < n - 1; r++)
  {
    for (int c_idx = 0; c_idx < r; c_idx++)
    {
      o->k[c_idx][r] = o->k[r][c_idx];
    }
  }
  return n - 1;
}

void wbc_gmo_solve(int n, const wbc_gmo_t *restrict o, const double *restrict b, int from,
                   double *restrict y, double *restrict x)
{
  for (int i = from; i < n; i++)
  {
    const double *restrict row = o->k[i];
    double                 s = b[i];
    int                    k = 0;

    for (; k + 4 <= i; k += 4)
    {
      const double p0 = row[k] * y[k];
      const double p1 = row[k + 1] * y[k + 1];
      const double p2 = row[k + 2] * y[k + 2];
      const double p3 = row[k + 3] * y[k + 3];

      s = (((s - p0) - p1) - p2) - p3;
    }
    for (; k < i; k++)
    {
      s -= row[k] * y[k];
    }
    y[i] = s / row[i];
  }
  for (int i = n - 1; i >= 0; i--)
  {
    const double *restrict row = o->k[i];
    double                 s = y[i];
    int                    k = i + 1;

    for (; k + 4 <= n; k += 4)
    {
      const double p0 = row[k] * x[k];
      const double p1 = row[k + 1] * x[k + 1];
      const double p2 = row[k + 2] * x[k + 2];
      const double p3 = row[k + 3] * x[k + 3];

      s = (((s - p0) - p1) - p2) - p3;
    }
    for (; k < n; k++)
    {
      s -= row[k] * x[k];
    }
    x[i] = s / row[i];
  }
}

void wbc_gmo_added(wbc_gmo_t *restrict o, int x, int m, int weighed, const double *restrict c0,
                   const double *restrict c1)
{
  if (c1 != NULL)
  {
    o->h[x][x] = 0.0;
    for (int k = 0; k < weighed; k++)
    {
      o->h[x][x] += c0[k] * o->ag[k][x];
    }
    for (int y0 = x + 1; y0 < m; y0 += WBC_GMO_LANES)
    {
      const int lanes = (m - y0 < WBC_GMO_LANES) ? (m - y0) : WBC_GMO_LANES;
      double    s0[WBC_GMO_LANES], s1[WBC_GMO_LANES];

      for (int j = 0; j < WBC_GMO_LANES; j++)
      {
        s0[j] = 0.0;
        s1[j] = 0.0;
      }
      for (int k = 0; k < weighed; k++)
      {
        const double           v0 = c0[k], v1 = c1[k];
        const double *restrict agk = &o->ag[k][y0];

        for (int j = 0; j < WBC_GMO_LANES; j++)
        {
          s0[j] += v0 * agk[j];
          s1[j] += v1 * agk[j];
        }
      }
      for (int j = 0; j < lanes; j++)
      {
        o->h[x][y0 + j] = s0[j];
        o->h[x + 1][y0 + j] = s1[j];
      }
    }
    return;
  }
  for (int y0 = x; y0 < m; y0 += WBC_GMO_LANES)
  {
    const int lanes = (m - y0 < WBC_GMO_LANES) ? (m - y0) : WBC_GMO_LANES;
    double    sum[WBC_GMO_LANES];

    for (int j = 0; j < WBC_GMO_LANES; j++)
    {
      sum[j] = 0.0;
    }
    for (int k = 0; k < weighed; k++)
    {
      for (int j = 0; j < WBC_GMO_LANES; j++)
      {
        sum[j] += c0[k] * o->ag[k][y0 + j];
      }
    }
    for (int j = 0; j < lanes; j++)
    {
      o->h[x][y0 + j] = sum[j];
    }
  }
}

void wbc_gmo_lessened(wbc_gmo_t *restrict o, int m, int pushing, const int *restrict on,
                      const double *restrict g, const double *restrict f)
{
  for (int x0 = 0; x0 < m; x0 += WBC_GMO_LANES)
  {
    const int lanes = (m - x0 < WBC_GMO_LANES) ? (m - x0) : WBC_GMO_LANES;
    double    sum[WBC_GMO_LANES];

    for (int j = 0; j < WBC_GMO_LANES; j++)
    {
      sum[j] = g[x0 + j];
    }
    for (int k = 0; k < pushing; k++)
    {
      const double *restrict hc = &o->h[on[k]][x0];
      const double           fc = f[on[k]];

      for (int j = 0; j < WBC_GMO_LANES; j++)
      {
        sum[j] -= hc[j] * fc;
      }
    }
    for (int j = 0; j < lanes; j++)
    {
      o->w[x0 + j] = sum[j];
    }
  }
}
