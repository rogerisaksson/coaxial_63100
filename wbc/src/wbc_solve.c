/** wbc_solve.c - the loop's numerics: Cholesky, a level's dual active set in the null space of
    the levels above (Goldfarb and Idnani), the drives' loads through their rods. */
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

/** A level's curvature damped by REG of its largest - level 0, the exact rows, by REG0 only. */
static const double REG = 1e-6, REG0 = 1e-10;
/** The tie-break: the levels below weigh LOWER of a level's curvature over theirs in the
    directions its rows leave flat (machine.qp.stack); not on level 0, the exact rows. */
static const double LOWER = 1e-3;
/** A bound held with a dual over HOLD stays held below its level (machine.qp.stack's rule):
    at 1e-7 a 38 N shove from behind felled her, at 1e-1 the walk fell; 1e-3 and 1e-2 hold
    both, 80 N too (2026-10-11). */
static const double HOLD = 1e-2;
/** The active set: at most MAXIT steps a level a tick; a bound over by more than TOL is broken,
    a dual under TOL is spent. */
#define MAXIT 160
static const double TOL = 1e-7;
/** The levels whose rows are bounds, each scaled to unit norm before the solve. */
static const int LEVEL_UNIT[WBC_LEVELS] = {1, 1, 0, 0, 0};

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

/* ---- the active set ----------------------------------------------------------------------- */

/** g . v over the first m coordinates. */
static double dot(const double *g, const double *v, int m)
{
  double d = 0.0;

  for (int c = 0; c < m; c++)
  {
    d += g[c] * v[c];
  }
  return d;
}

/** Bound i into the set at place k: v_i = K^-1 g_i, and S's new row. */
static void set_add(wbc_stack_t *s, int level, int i, int m, const double *vi, double dual)
{
  const int k = s->wn[level];

  s->wset[level][k] = i;
  memcpy(s->vw[k], vi, (size_t)WBC_V * sizeof(double));
  s->ud[k] = dual;
  for (int j = 0; j <= k; j++)
  {
    const double v = dot(s->gb[s->wset[level][j]], vi, m);

    s->sm[k][j] = v;
    s->sm[j][k] = v;
  }
  s->wn[level]++;
}

/** The bound at place l out of the set: the last takes its place. */
static void set_drop(wbc_stack_t *s, int level, int l)
{
  const int k = s->wn[level] - 1;

  if (l != k)
  {
    s->wset[level][l] = s->wset[level][k];
    memcpy(s->vw[l], s->vw[k], (size_t)WBC_V * sizeof(double));
    s->ud[l] = s->ud[k];
    for (int j = 0; j <= k; j++)
    {
      s->sm[l][j] = s->sm[k][j];
    }
    for (int j = 0; j <= k; j++)
    {
      s->sm[j][l] = s->sm[j][k];
    }
    s->sm[l][l] = s->sm[k][k];
  }
  s->wn[level] = k;
}

/** r = S^-1 b over the k held; 0 where S is not positive. */
static int schur_solve(wbc_stack_t *s, int k, const double *b, double *r)
{
  for (int i = 0; i < k; i++)
  {
    for (int j = 0; j < k; j++)
    {
      s->sfac[i][j] = s->sm[i][j];
    }
    s->sfac[i][i] += 1e-12 * (1.0 + s->sm[i][i]);
  }
  if (!wbc_cholesky(k, WBC_V, &s->sfac[0][0]))
  {
    return 0;
  }
  wbc_chol_solve(k, WBC_V, &s->sfac[0][0], b, r);
  return 1;
}

void wbc_level_solve(wbc_stack_t *s, int level, int last)
{
  const int rows = s->lrows[level], ineq = s->ineq, m = s->zm;
  double    sum[WBC_V], y[WBC_V], y0[WBC_V], rhs[WBC_V], q[WBC_V], nv[WBC_V], r[WBC_V];
  double    vp[WBC_V], z[WBC_V], big = 0.0, mu;
  int       pending = -1, feasible = 1;

  for (int v = 0; v < WBC_V; v++)
  {
    sum[v] = 0.0;
    for (int j = 0; j < level; j++)
    {
      sum[v] += s->tau[j][v];
    }
    s->tau[level][v] = 0.0;
  }
  if ((rows == 0) || (m == 0))
  {
    return;
  }
  /* The rows in the reduced coordinates, ab = A Z, and their residual asks. */
  for (int i = 0; i < rows; i++)
  {
    double got = 0.0, nrm = 0.0;

    for (int c = 0; c < m; c++)
    {
      double v = 0.0;

      for (int w = 0; w < WBC_V; w++)
      {
        v += s->la[level][i][w] * s->zb[w][c];
      }
      s->ab[i][c] = v;
      nrm += v * v;
    }
    for (int w = 0; w < WBC_V; w++)
    {
      got += s->la[level][i][w] * sum[w];
    }
    s->rho[i] = s->lr[level][i] - got;
    if (LEVEL_UNIT[level])
    {
      nrm = sqrt(nrm);
      for (int c = 0; c < m; c++)
      {
        s->ab[i][c] = (nrm > 1e-12) ? s->ab[i][c] / nrm : 0.0;
      }
      s->rho[i] = (nrm > 1e-12) ? s->rho[i] / nrm : 0.0;
    }
  }
  /* The bounds in the reduced coordinates: gb y <= hb, what the variables so far leave; a
     bound this level cannot move (its reduced row all but zero) is the levels' above. */
  for (int i = 0; i < ineq; i++)
  {
    double got = 0.0, nrm = 0.0;

    for (int c = 0; c < m; c++)
    {
      double v = 0.0;

      for (int w = 0; w < WBC_V; w++)
      {
        v += s->ga[i][w] * s->zb[w][c];
      }
      s->gb[i][c] = v;
      nrm += v * v;
    }
    for (int w = 0; w < WBC_V; w++)
    {
      got += s->ga[i][w] * sum[w];
    }
    s->hb[i] = s->gh[i] - got;
    s->gfixed[i] = nrm < 1e-12;
  }
  /* H = ab^T ab, q = -ab^T rho; K = H + mu I, factored; y0 = -K^-1 q. */
  for (int c = 0; c < m; c++)
  {
    for (int d = 0; d < m; d++)
    {
      double v = 0.0;

      for (int i = 0; i < rows; i++)
      {
        v += s->ab[i][c] * s->ab[i][d];
      }
      s->hh[c][d] = v;
    }
    big = (s->hh[c][c] > big) ? s->hh[c][c] : big;
    q[c] = 0.0;
    for (int i = 0; i < rows; i++)
    {
      q[c] -= s->ab[i][c] * s->rho[i];
    }
  }
  big = (big > 1e-12) ? big : 1e-12;
  {
    double top = 0.0, low;

    for (int c = 0; c < m; c++)
    {
      s->lowcol[c] = 0.0;
    }
    for (int l = level + 1; l < WBC_LEVELS; l++)
    {
      for (int i = 0; i < s->lrows[l]; i++)
      {
        double got = 0.0;

        for (int c = 0; c < m; c++)
        {
          double v = 0.0;

          for (int w = 0; w < WBC_V; w++)
          {
            v += s->la[l][i][w] * s->zb[w][c];
          }
          s->lowab[l][i][c] = v;
          s->lowcol[c] += v * v;
        }
        for (int w = 0; w < WBC_V; w++)
        {
          got += s->la[l][i][w] * sum[w];
        }
        s->lowrhs[l][i] = s->lr[l][i] - got;
      }
    }
    for (int c = 0; c < m; c++)
    {
      top = (s->lowcol[c] > top) ? s->lowcol[c] : top;
    }
    low = (level == 0) ? 0.0 : LOWER * ((big > 1.0) ? big : 1.0) / ((top > 1.0) ? top : 1.0);
    for (int l = level + 1; l < WBC_LEVELS; l++)
    {
      for (int i = 0; i < s->lrows[l]; i++)
      {
        const double *row = s->lowab[l][i];

        for (int c = 0; c < m; c++)
        {
          if (row[c] != 0.0)
          {
            for (int d = 0; d < m; d++)
            {
              s->hh[c][d] += low * row[c] * row[d];
            }
            q[c] -= low * row[c] * s->lowrhs[l][i];
          }
        }
      }
    }
  }
  mu = ((level == 0) ? REG0 : REG) * big;
  memcpy(s->kk, s->hh, sizeof(s->kk));
  for (int c = 0; c < m; c++)
  {
    s->kk[c][c] += mu;
    rhs[c] = -q[c];
  }
  if (!wbc_cholesky(m, WBC_V, &s->kk[0][0]))
  {
    return;
  }
  wbc_chol_solve(m, WBC_V, &s->kk[0][0], rhs, y0);
  memcpy(y, y0, sizeof(y));

  /* The set warm from the last tick where the bounds are as many: v_j and S again at this
     pose, the point and the duals on it; a dual turned negative frees its bound. */
  if (s->wsized[level] != ineq)
  {
    s->wn[level] = 0;
    s->wsized[level] = ineq;
  }
  for (int j = 0; j < s->wn[level]; j++)
  {
    if (s->gfixed[s->wset[level][j]])
    {
      s->wset[level][j--] = s->wset[level][--s->wn[level]];
    }
  }
  {
    const int k0 = s->wn[level];

    s->wn[level] = 0;
    for (int j = 0; j < k0; j++)
    {
      const int i = s->wset[level][j];

      wbc_chol_solve(m, WBC_V, &s->kk[0][0], s->gb[i], vp);
      s->wset[level][j] = i;
      set_add(s, level, i, m, vp, 0.0);
    }
  }
  for (int it = 0; it < MAXIT; it++)
  {
    const int k = s->wn[level];
    int       worst = -1;

    for (int j = 0; j < k; j++)
    {
      nv[j] = dot(s->gb[s->wset[level][j]], y0, m) - s->hb[s->wset[level][j]];
    }
    if (k > 0)
    {
      if (!schur_solve(s, k, nv, r))
      {
        set_drop(s, level, k - 1);
        continue;
      }
      for (int c = 0; c < m; c++)
      {
        double v = y0[c];

        for (int j = 0; j < k; j++)
        {
          v -= r[j] * s->vw[j][c];
        }
        y[c] = v;
      }
      for (int j = 0; j < k; j++)
      {
        s->ud[j] = r[j];
        if ((r[j] < -TOL) && ((worst < 0) || (r[j] < r[worst])))
        {
          worst = j;
        }
      }
      if (worst >= 0)
      {
        set_drop(s, level, worst);
        continue;
      }
    }
    break;
  }
  /* Goldfarb and Idnani: the most broken bound in, by the largest step keeping every dual
     non-negative; a dual that would turn negative frees its bound and the step goes on; a
     bound in the span of those held moves the duals alone, and with no dual to spend it
     cannot be met at all (infeasible: the point stays). */
  for (int it = 0; (it < MAXIT) && feasible; it++)
  {
    const int k = s->wn[level];
    double    most = TOL, viol, nz, t1 = 1e300, t2, t;
    int       l = -1;

    s->iterations++;
    if (pending < 0)
    {
      for (int i = 0; i < ineq; i++)
      {
        double v;
        int    in = 0;

        for (int j = 0; j < k; j++)
        {
          in |= (s->wset[level][j] == i);
        }
        if (in || s->gfixed[i])
        {
          continue;
        }
        v = dot(s->gb[i], y, m) - s->hb[i];
        if (v > most)
        {
          most = v;
          pending = i;
        }
      }
      if (pending < 0)
      {
        break;
      }
      wbc_chol_solve(m, WBC_V, &s->kk[0][0], s->gb[pending], vp);
      s->upend = 0.0;
    }
    /* The directions: r = S^-1 N v_p, z = v_p - sum r_j v_j. */
    if (k > 0)
    {
      for (int j = 0; j < k; j++)
      {
        nv[j] = dot(s->gb[s->wset[level][j]], vp, m);
      }
      if (!schur_solve(s, k, nv, r))
      {
        set_drop(s, level, k - 1);
        continue;
      }
    }
    for (int c = 0; c < m; c++)
    {
      double v = vp[c];

      for (int j = 0; j < k; j++)
      {
        v -= r[j] * s->vw[j][c];
      }
      z[c] = v;
    }
    for (int j = 0; j < k; j++)
    {
      if ((r[j] > 1e-12) && (s->ud[j] / r[j] < t1))
      {
        t1 = s->ud[j] / r[j];
        l = j;
      }
    }
    nz = dot(s->gb[pending], z, m);
    viol = dot(s->gb[pending], y, m) - s->hb[pending];
    if (nz <= 1e-12 * dot(s->gb[pending], s->gb[pending], m))
    {
      /* In the span of those held: the duals alone. */
      if (l < 0)
      {
        feasible = 0;
        break;
      }
      for (int j = 0; j < k; j++)
      {
        s->ud[j] -= t1 * r[j];
      }
      s->upend += t1;
      set_drop(s, level, l);
      continue;
    }
    t2 = viol / nz;
    t = (t1 < t2) ? t1 : t2;
    for (int c = 0; c < m; c++)
    {
      y[c] -= t * z[c];
    }
    for (int j = 0; j < k; j++)
    {
      s->ud[j] -= t * r[j];
    }
    s->upend += t;
    if (t2 <= t1)
    {
      if (k < m)
      {
        set_add(s, level, pending, m, vp, s->upend);
      }
      pending = -1;
    }
    else
    {
      set_drop(s, level, l);
    }
    if (it == MAXIT - 1)
    {
      s->stuck++;
    }
  }
  /* The variables, x = Z y. */
  for (int v = 0; v < WBC_V; v++)
  {
    double x = 0.0;

    for (int c = 0; c < m; c++)
    {
      x += s->zb[v][c] * y[c];
    }
    s->tau[level][v] = x;
  }
  if (last)
  {
    return;
  }
  /* Z cut to the null space of the rows, and of the bounds held pressing (a positive dual:
     leaving one would cost the level its optimum, machine.qp.stack's rule): their span
     orthonormalised into s->qq, each old basis direction less that span orthonormalised
     again (Gram-Schmidt, twice for its accuracy). */
  {
    int kept = 0, found = 0, all = rows;

    for (int j = 0; (j < s->wn[level]) && (all < WBC_ROWS); j++)
    {
      if (s->ud[j] > HOLD)
      {
        memcpy(s->ab[all], s->gb[s->wset[level][j]], sizeof(s->ab[all]));
        all++;
      }
    }
    for (int i = 0; i < all; i++)
    {
      double nrm = 0.0;

      memcpy(s->qq[kept], s->ab[i], sizeof(s->qq[kept]));
      for (int pass = 0; pass < 2; pass++)
      {
        for (int j = 0; j < kept; j++)
        {
          const double d = dot(s->qq[kept], s->qq[j], m);

          for (int c = 0; c < m; c++)
          {
            s->qq[kept][c] -= d * s->qq[j][c];
          }
        }
      }
      nrm = sqrt(dot(s->qq[kept], s->qq[kept], m));
      if (nrm > 1e-9)
      {
        for (int c = 0; c < m; c++)
        {
          s->qq[kept][c] /= nrm;
        }
        kept++;
      }
    }
    for (int c = 0; (c < m) && (found < m); c++)
    {
      double col[WBC_V], nrm;

      for (int d = 0; d < m; d++)
      {
        col[d] = (d == c) ? 1.0 : 0.0;
      }
      for (int pass = 0; pass < 2; pass++)
      {
        for (int j = 0; j < kept; j++)
        {
          const double d = dot(col, s->qq[j], m);

          for (int e = 0; e < m; e++)
          {
            col[e] -= d * s->qq[j][e];
          }
        }
        for (int j = 0; j < found; j++)
        {
          const double d = dot(col, s->zn[j], m);

          for (int e = 0; e < m; e++)
          {
            col[e] -= d * s->zn[j][e];
          }
        }
      }
      nrm = sqrt(dot(col, col, m));
      if (nrm > 1e-6)
      {
        for (int d = 0; d < m; d++)
        {
          s->zn[found][d] = col[d] / nrm;
        }
        found++;
      }
    }
    for (int v = 0; v < WBC_V; v++)
    {
      double row[WBC_V];

      for (int j = 0; j < found; j++)
      {
        row[j] = dot(s->zb[v], s->zn[j], m);
      }
      for (int j = 0; j < found; j++)
      {
        s->zb[v][j] = row[j];
      }
    }
    s->zm = found;
  }
}

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
