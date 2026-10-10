/** wbc_solve.c - the loop's numerics: Cholesky, a level's active set in the null space of the
    levels above, the drives' loads through their rods. */
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

static const double DAMP = 1e-6, REG = 1e-6;
/** The active set: at most MAXIT changes a level a tick; a multiplier under -TOL frees its
    bound, a bound over by more than TOL joins. */
#define MAXIT 20
static const double TOL = 1e-7;
/** The levels whose rows are bounds, each scaled to unit norm before the solve. */
static const int LEVEL_UNIT[WBC_LEVELS] = {1, 1, 0, 0, 0};

/** a, n x n by rows of `ld`, into its lower Cholesky factor in place; 0 where not positive. */
int wbc_cholesky(int n, int ld, double *a)
{
  for (int j = 0; j < n; j++)
  {
    double d = a[j * ld + j];

    for (int k = 0; k < j; k++)
    {
      d -= a[j * ld + k] * a[j * ld + k];
    }
    if (d <= 0.0)
    {
      return 0;
    }
    d = sqrt(d);
    a[j * ld + j] = d;
    for (int i = j + 1; i < n; i++)
    {
      double s = a[i * ld + j];

      for (int k = 0; k < j; k++)
      {
        s -= a[i * ld + k] * a[j * ld + k];
      }
      a[i * ld + j] = s / d;
    }
  }
  return 1;
}

/** x = (L L^T)^-1 b. */
void wbc_chol_solve(int n, int ld, const double *l, const double *b, double *x)
{
  for (int i = 0; i < n; i++)
  {
    double s = b[i];

    for (int k = 0; k < i; k++)
    {
      s -= l[i * ld + k] * x[k];
    }
    x[i] = s / l[i * ld + i];
  }
  for (int i = n - 1; i >= 0; i--)
  {
    double s = x[i];

    for (int k = i + 1; k < n; k++)
    {
      s -= l[k * ld + i] * x[k];
    }
    x[i] = s / l[i * ld + i];
  }
}

/** The quaternion (w, x, y, z) of a frame's rotation. */
void wbc_quat_of(const double r[9], double q[4])
{
  const double tr = r[0] + r[4] + r[8];

  if (tr > 0.0)
  {
    const double s = sqrt(tr + 1.0) * 2.0;

    q[0] = 0.25 * s;
    q[1] = (r[7] - r[5]) / s;
    q[2] = (r[2] - r[6]) / s;
    q[3] = (r[3] - r[1]) / s;
  }
  else if ((r[0] > r[4]) && (r[0] > r[8]))
  {
    const double s = sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;

    q[0] = (r[7] - r[5]) / s;
    q[1] = 0.25 * s;
    q[2] = (r[1] + r[3]) / s;
    q[3] = (r[2] + r[6]) / s;
  }
  else if (r[4] > r[8])
  {
    const double s = sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;

    q[0] = (r[2] - r[6]) / s;
    q[1] = (r[1] + r[3]) / s;
    q[2] = 0.25 * s;
    q[3] = (r[5] + r[7]) / s;
  }
  else
  {
    const double s = sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;

    q[0] = (r[3] - r[1]) / s;
    q[1] = (r[2] + r[6]) / s;
    q[2] = (r[5] + r[7]) / s;
    q[3] = 0.25 * s;
  }
}

/** The world-frame rotation vector taking `have` to `want`, rad: machine.wbc.turn_error. */
void wbc_turn_error(const double want[4], const double have[4], double out[3])
{
  const double w0 = have[0], x0 = have[1], y0 = have[2], z0 = have[3];
  const double w1 = want[0], x1 = want[1], y1 = want[2], z1 = want[3];
  double       w = w1 * w0 + x1 * x0 + y1 * y0 + z1 * z0;
  double       v[3] = {-w1 * x0 + x1 * w0 - y1 * z0 + z1 * y0,
                       -w1 * y0 + y1 * w0 - z1 * x0 + x1 * z0,
                       -w1 * z0 + z1 * w0 - x1 * y0 + y1 * x0};
  double       s;

  if (w < 0.0)
  {
    w = -w;
    v[0] = -v[0];
    v[1] = -v[1];
    v[2] = -v[2];
  }
  s = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  for (int k = 0; k < 3; k++)
  {
    out[k] = (s > 1e-12) ? v[k] * (2.0 * atan2(s, w) / s) : 2.0 * v[k];
  }
}

/* ---- the levels --------------------------------------------------------------------------- */

/** A level's rows this tick, least squares in the null space so far under every bound (ADMM),
    its torques into s->tau[level]; then the null space under them for the next. */
void wbc_level_solve(wbc_stack_t *s, int level, int last)
{
  const int rows = s->lrows[level], ineq = s->ineq;
  double    sum[WBC_X], x[WBC_X], x0[WBC_X], rhs[WBC_X], q[WBC_X], srhs[WBC_X], lambda[WBC_X];
  double    big = 0.0, mu;

  for (int k = 0; k < WBC_X; k++)
  {
    sum[k] = 0.0;
    for (int j = 0; j < level; j++)
    {
      sum[k] += s->tau[j][k];
    }
    s->tau[level][k] = 0.0;
  }
  if (rows == 0)
  {
    return;
  }
  /* The rows into the null space, and their residual asks. */
  for (int i = 0; i < rows; i++)
  {
    double got = 0.0, nrm = 0.0;

    for (int k = 0; k < WBC_X; k++)
    {
      double v = 0.0;

      for (int m = 0; m < WBC_X; m++)
      {
        v += s->la[level][i][m] * s->nn[m][k];
      }
      s->ab[i][k] = v;
      got += s->la[level][i][k] * sum[k];
      nrm += v * v;
    }
    s->rho[i] = s->lr[level][i] - got;
    if (LEVEL_UNIT[level])
    {
      nrm = sqrt(nrm);
      for (int k = 0; k < WBC_X; k++)
      {
        s->ab[i][k] = (nrm > 1e-12) ? s->ab[i][k] / nrm : 0.0;
      }
      s->rho[i] = (nrm > 1e-12) ? s->rho[i] / nrm : 0.0;
    }
  }
  /* The bounds into the null space: gb y <= hb, what the torques so far leave. */
  for (int i = 0; i < ineq; i++)
  {
    double got = 0.0;

    for (int k = 0; k < WBC_X; k++)
    {
      double v = 0.0;

      for (int m = 0; m < WBC_X; m++)
      {
        v += s->ga[i][m] * s->nn[m][k];
      }
      s->gb[i][k] = v;
      got += s->ga[i][k] * sum[k];
    }
    s->hb[i] = s->gh[i] - got;
  }
  /* H = ab^T ab, q = -ab^T rho; K = H + mu I, factored. */
  for (int k = 0; k < WBC_X; k++)
  {
    for (int m = 0; m < WBC_X; m++)
    {
      double v = 0.0;

      for (int i = 0; i < rows; i++)
      {
        v += s->ab[i][k] * s->ab[i][m];
      }
      s->hh[k][m] = v;
    }
    big = (s->hh[k][k] > big) ? s->hh[k][k] : big;
    q[k] = 0.0;
    for (int i = 0; i < rows; i++)
    {
      q[k] -= s->ab[i][k] * s->rho[i];
    }
  }
  big = (big > 1e-12) ? big : 1e-12;
  mu = REG * big;
  memcpy(s->kk, s->hh, sizeof(s->kk));
  for (int k = 0; k < WBC_X; k++)
  {
    s->kk[k][k] += mu;
    rhs[k] = -q[k];
  }
  if (!wbc_cholesky(WBC_X, WBC_X, &s->kk[0][0]))
  {
    return;
  }
  wbc_chol_solve(WBC_X, WBC_X, &s->kk[0][0], rhs, x0);
  /* The working set, warm from the last tick where the bounds are as many. */
  if (s->wsized[level] != ineq)
  {
    s->wn[level] = 0;
    s->wsized[level] = ineq;
  }
  memcpy(x, x0, sizeof(x));
  for (int it = 0; it < MAXIT; it++)
  {
    const int k = s->wn[level];
    int       drop = -1, add = -1;
    double    least = -TOL, most = TOL;

    s->iterations++;
    if (k > 0)
    {
      /* v_j = K^-1 g_j; S = G_W K^-1 G_W^T; lambda = S^-1 (G_W x0 - h_W); x = x0 - sum lambda_j v_j. */
      for (int j = 0; j < k; j++)
      {
        wbc_chol_solve(WBC_X, WBC_X, &s->kk[0][0], s->gb[s->wset[level][j]], s->vw[j]);
      }
      for (int i = 0; i < k; i++)
      {
        const double *gi = s->gb[s->wset[level][i]];
        double        v = -s->hb[s->wset[level][i]];

        for (int j = 0; j < k; j++)
        {
          double sij = 0.0;

          for (int m = 0; m < WBC_X; m++)
          {
            sij += gi[m] * s->vw[j][m];
          }
          s->sm[i][j] = sij;
        }
        s->sm[i][i] += 1e-12;
        for (int m = 0; m < WBC_X; m++)
        {
          v += gi[m] * x0[m];
        }
        srhs[i] = v;
      }
      if (!wbc_cholesky(k, WBC_X, &s->sm[0][0]))
      {
        s->wn[level]--;
        continue;
      }
      wbc_chol_solve(k, WBC_X, &s->sm[0][0], srhs, lambda);
      for (int m = 0; m < WBC_X; m++)
      {
        double v = x0[m];

        for (int j = 0; j < k; j++)
        {
          v -= lambda[j] * s->vw[j][m];
        }
        x[m] = v;
      }
      for (int j = 0; j < k; j++)
      {
        if (lambda[j] < least)
        {
          least = lambda[j];
          drop = j;
        }
      }
    }
    if (drop >= 0)
    {
      s->wset[level][drop] = s->wset[level][k - 1];
      s->wn[level]--;
      continue;
    }
    for (int i = 0; i < ineq; i++)
    {
      double v = -s->hb[i];
      int    in = 0;

      for (int j = 0; j < k; j++)
      {
        in |= (s->wset[level][j] == i);
      }
      if (in)
      {
        continue;
      }
      for (int m = 0; m < WBC_X; m++)
      {
        v += s->gb[i][m] * x[m];
      }
      if (v > most)
      {
        most = v;
        add = i;
      }
    }
    if ((add < 0) || (k >= WBC_X))
    {
      break;
    }
    s->wset[level][k] = add;
    s->wn[level]++;
  }
  /* The torques, in the null space; the null space less this level's rows for the next. */
  for (int k = 0; k < WBC_X; k++)
  {
    double v = 0.0;

    for (int m = 0; m < WBC_X; m++)
    {
      v += s->nn[k][m] * x[m];
    }
    s->tau[level][k] = v;
  }
  if (last || (rows > WBC_SMALL))
  {
    return;
  }
  big = 0.0;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < rows; j++)
    {
      double v = 0.0;

      for (int k = 0; k < WBC_X; k++)
      {
        v += s->ab[i][k] * s->ab[j][k];
      }
      s->g[i][j] = v;
    }
    big = (s->g[i][i] > big) ? s->g[i][i] : big;
  }
  for (int i = 0; i < rows; i++)
  {
    s->g[i][i] += DAMP * big + 1e-300;
  }
  if (!wbc_cholesky(rows, WBC_SMALL, &s->g[0][0]))
  {
    return;
  }
  for (int k = 0; k < WBC_X; k++)
  {
    double col[WBC_ROWS] = {0.0}, sol[WBC_ROWS] = {0.0};

    for (int i = 0; i < rows; i++)
    {
      col[i] = s->ab[i][k];
    }
    wbc_chol_solve(rows, WBC_SMALL, &s->g[0][0], col, sol);
    for (int i = 0; i < rows; i++)
    {
      s->qq[i][k] = sol[i];
    }
  }
  for (int k = 0; k < WBC_X; k++)
  {
    for (int m = 0; m < WBC_X; m++)
    {
      double v = 0.0;

      for (int i = 0; i < rows; i++)
      {
        v += s->ab[i][k] * s->qq[i][m];
      }
      s->nn[k][m] -= v;
    }
  }
}

/* ---- the drives' loads -------------------------------------------------------------------- */

/** Each drive's load from the drives' torques: a pair's as its rods share them. */
void wbc_loads(const double tau[WBC_X], double load[WBC_DRIVEN])
{
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    load[j] = tau[j];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    if (pair > 0)
    {
      for (int other = 0; other < WBC_DRIVEN; other++)
      {
        if (wbc_driven[other] == pair - 1)
        {
          const double kj = wbc_kt[wbc_driven[j]], ko = wbc_kt[wbc_driven[other]];
          const double a = tau[j] / kj, c = tau[other] / ko;

          load[j] = (a + c) * kj;
          load[other] = (a - c) * ko;
        }
      }
    }
  }
}

/** The drives' torques from their loads: the inverse of `loads`. */
void wbc_unload(const double load[WBC_DRIVEN], double tau[WBC_X])
{
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    tau[j] = load[j];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    if (pair > 0)
    {
      for (int other = 0; other < WBC_DRIVEN; other++)
      {
        if (wbc_driven[other] == pair - 1)
        {
          const double kj = wbc_kt[wbc_driven[j]], ko = wbc_kt[wbc_driven[other]];
          const double a = 0.5 * (load[j] / kj + load[other] / ko);
          const double c = 0.5 * (load[j] / kj - load[other] / ko);

          tau[j] = a * kj;
          tau[other] = c * ko;
        }
      }
    }
  }
}
