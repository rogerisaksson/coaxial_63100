/** wbc_qp.c - machine.stack.qp.solve (wbc_qp.h): Goldfarb and Idnani's dual active set on the rows scaled to unit
    norm, the interior point (wbc_qp_interior.c) where it gives up - numpy's arithmetic and its OpenBLAS (wbc_blas.h)
    operation for operation, every product in python's order and layout. Portable C11, the host's alone. */
#include "wbc_np.h"
#include "wbc_qp.h"

#include <math.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})
/** A row over by more than TOL (unit-norm) is added; a direction or a multiplier under TINY is none; a row in the span
    of the held ones over by less than LOOSE is met (machine.stack.qp's TOL, TINY, LOOSE). */
#define TOL 1e-9
#define TINY 1e-12
#define LOOSE 1e-5

/** The dual's state: J0 = inv(L)' the Fortran view of inv(L) (Li), N = -A' the view of -A (NA, m by n), d = -b; the
    rows held in order (act, q of them) and their columns of N gathered as python lays them, F-ordered (Nq: row act[j]
    of NA); J0'N (X), its Q (Qb), J = J0 Q (Jb) and R (Rf, n by q, its first q rows the factor); J the view python
    holds, J0 itself with none held; the qr's workspace (qw, nwork). */
typedef struct
{
  const wbc_blas_t *l;
  int               n, m, q, nwork;
  const double     *Li, *NA, *d;
  double           *Nq, *X, *Qb, *Jb, *Rf, *qw;
  int              *act;
  wbc_view_t        J;
} dual_t;

/** `count` doubles from `*at` on, the next carve 64-byte aligned. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** numpy's argmin of `a` (n): the first least, the first nan before any. */
static int argmin(const double *a, int n)
{
  int best = 0;

  for (int i = 1; i < n; i++)
  {
    if (isnan(a[best]))
    {
      return best;
    }
    if (isnan(a[i]) || a[i] < a[best])
    {
      best = i;
    }
  }
  return best;
}

/** numpy's .min() and .max() of `a` (n). */
static double least(const double *a, int n)
{
  double r = a[0];

  for (int i = 1; i < n; i++)
  {
    r = npminimum(r, a[i]);
  }
  return r;
}

static double most_of(const double *a, int n)
{
  double r = a[0];

  for (int i = 1; i < n; i++)
  {
    r = npmaximum(r, a[i]);
  }
  return r;
}

/** machine.stack.qp._factor: J0'N = Q [R; 0], J = J0 Q - J0 itself with none held. 0, or 2 where numpy raises. */
static int factor(dual_t *s)
{
  const int n = s->n, q = s->q;

  if (!q)
  {
    s->J = VIEW(s->Li, 1, n);
    return 0;
  }
  for (int j = 0; j < q; j++)
  {
    memcpy(s->Nq + j * n, s->NA + s->act[j] * n, (size_t)n * sizeof(double));
  }
  wbc_matmul(s->l, VIEW(s->Li, n, 1), VIEW(s->Nq, 1, n), s->X, q, 1, n, n, q);
  if (wbc_qr(s->l, VIEW(s->X, q, 1), n, q, s->Qb, s->Rf, s->qw, s->nwork))
  {
    return 2;
  }
  wbc_matmul(s->l, VIEW(s->Li, 1, n), VIEW(s->Qb, n, 1), s->Jb, n, 1, n, n, n);
  s->J = VIEW(s->Jb, n, 1);
  return 0;
}

/** The k-th row held let go. */
static void drop(dual_t *s, int k)
{
  for (int j = k; j + 1 < s->q; j++)
  {
    s->act[j] = s->act[j + 1];
  }
  s->q = s->q - 1;
}

/** machine.stack.qp._pair: x and u the minimiser on the rows held as equalities, rows dependent on those before
    them and rows whose multipliers come out negative dropped. 0, or 2 where numpy raises; w, t, dg scratch (n). */
static int pair(dual_t *s, const double *x0, double *x, double *u, double *w, double *t, double *dg, double *gw)
{
  const wbc_blas_t *l = s->l;
  const int         n = s->n;

  s->q = s->q > n ? n : s->q;
  for (;;)
  {
    if (factor(s))
    {
      return 2;
    }
    if (!s->q)
    {
      break;
    }
    for (int i = 0; i < s->q; i++)
    {
      dg[i] = fabs(s->Rf[i * s->q + i]);
    }
    const double hi = most_of(dg, s->q);

    if (least(dg, s->q) > 1e-9 * (hi > 1.0 ? hi : 1.0))
    {
      break;
    }
    drop(s, argmin(dg, s->q));
  }
  while (s->q)
  {
    const int q = s->q;

    wbc_matmul(l, VIEW(s->Nq, n, 1), VIEW(x0, 1, 0), t, 1, 0, q, n, 1);
    for (int j = 0; j < q; j++)
    {
      t[j] = s->d[s->act[j]] - t[j];
    }
    if (wbc_solve(l, VIEW(s->Rf, 1, q), VIEW(t, 1, 0), q, 1, w, gw) || wbc_solve(l, VIEW(s->Rf, q, 1), VIEW(w, 1, 0), q, 1, u, gw))
    {
      return 2;
    }
    if (least(u, q) >= 0.0)
    {
      wbc_matmul(l, VIEW(s->Jb, n, 1), VIEW(w, 1, 0), t, 1, 0, n, q, 1);
      for (int i = 0; i < n; i++)
      {
        x[i] = x0[i] + t[i];
      }
      return 0;
    }
    drop(s, argmin(u, q));
    if (factor(s))
    {
      return 2;
    }
  }
  memcpy(x, x0, (size_t)n * sizeof(double));
  return 0;
}

/** machine.stack.qp._dual on the rows kept: 0 with x, the rows held (s->act, s->q) and their multipliers u, 1 where it
    gives up (python's None), 2 where numpy raises; sv (m), met (m) and the rest (n) scratch. */
static int dual(dual_t *s, const double *x0, int most, double *x, double *u, double *sv, double *dv, double *z, double *r,
                double *t, double *ratio, double *dg, double *met, double *gw)
{
  const wbc_blas_t *l = s->l;
  const int         n = s->n, m = s->m;
  int               steps = 0, got;

  for (int i = 0; i < m; i++)
  {
    met[i] = 0.0;
  }
  if ((got = pair(s, x0, x, u, ratio, t, dg, gw)) != 0)
  {
    return got;
  }
  most = most > 0 ? most : 10 * (n + m);
  for (;;)
  {
    wbc_matmul(l, VIEW(s->NA, n, 1), VIEW(x, 1, 0), sv, 1, 0, m, n, 1);
    for (int i = 0; i < m; i++)
    {
      sv[i] = sv[i] - s->d[i];
    }
    for (int j = 0; j < s->q; j++)
    {
      sv[s->act[j]] = INFINITY;
    }
    for (int i = 0; i < m; i++)
    {
      sv[i] = met[i] != 0.0 && sv[i] > -LOOSE ? INFINITY : sv[i];
    }
    const int     p = argmin(sv, m);
    const double *nv = s->NA + p * n;
    double        up = 0.0;

    if (sv[p] >= -TOL)
    {
      return 0;
    }
    for (;;)
    {
      const int q = s->q;
      double    t1 = INFINITY, t2, zn, nx, tt;
      int       k = -1, any = 0;

      if (++steps > most)
      {
        return 1;
      }
      wbc_matmul(l, VIEW(s->J.at, s->J.s1, s->J.s0), VIEW(nv, 1, 0), dv, 1, 0, n, n, 1);
      wbc_matmul(l, VIEW(s->J.at + q * s->J.s1, s->J.s0, s->J.s1), VIEW(dv + q, 1, 0), z, 1, 0, n, n - q, 1);
      if (q && wbc_solve(l, VIEW(s->Rf, q, 1), VIEW(dv, 1, 0), q, 1, r, gw))
      {
        return 2;
      }
      zn = wbc_dot(l, z, 1, nv, 1, n);
      nx = wbc_dot(l, nv, 1, x, 1, n);
      const double dd = wbc_dot(l, dv, 1, dv, 1, n);

      t2 = zn > TINY * (dd > 1.0 ? dd : 1.0) ? -(nx - s->d[p]) / zn : INFINITY;
      if (t2 == INFINITY && -(nx - s->d[p]) < LOOSE)
      {
        /* a row in the span of the held ones, over by noise: met */
        met[p] = 1.0;
        if ((got = pair(s, x0, x, u, ratio, t, dg, gw)) != 0)
        {
          return got;
        }
        break;
      }
      for (int j = 0; j < q; j++)
      {
        any = any || r[j] > TINY;
      }
      if (any)
      {
        for (int j = 0; j < q; j++)
        {
          ratio[j] = r[j] > TINY ? u[j] / r[j] : INFINITY;
        }
        k = argmin(ratio, q);
        t1 = ratio[k];
      }
      tt = t2 < t1 ? t2 : t1;
      if (tt == INFINITY)
      {
        return 1;
      }
      if (t2 < INFINITY)
      {
        for (int i = 0; i < n; i++)
        {
          x[i] = x[i] + tt * z[i];
        }
      }
      for (int j = 0; j < q; j++)
      {
        u[j] = u[j] - tt * r[j];
      }
      up = up + tt;
      if (t2 <= t1)
      {
        s->act[q] = p;
        s->q = q + 1;
        u[q] = up;
        if (factor(s))
        {
          return 2;
        }
        break;
      }
      drop(s, k);
      for (int j = k; j < q - 1; j++)
      {
        u[j] = u[j + 1];
      }
      if (factor(s))
      {
        return 2;
      }
    }
  }
}

int wbc_qp_solve(const wbc_blas_t *l, const double *H, const double *g, const double *A, const double *b, int n, int m,
                 const int *warm, int nwarm, int most, double *x, int *act, int *nact, double *u, double *work)
{
  double *w = work;
  double *As = carve(&w, m * n), *NA = carve(&w, m * n), *bs = carve(&w, m + 1), *d = carve(&w, m + 1);
  double *norm = carve(&w, m + 1), *met = carve(&w, m + 1), *sv = carve(&w, m + 1);
  double *Lc = carve(&w, n * n), *Li = carve(&w, n * n), *Nq = carve(&w, n * n), *X = carve(&w, n * n);
  double *Qb = carve(&w, n * n), *Jb = carve(&w, n * n), *Rf = carve(&w, n * n);
  double *x0 = carve(&w, n), *dv = carve(&w, n), *z = carve(&w, n), *r = carve(&w, n), *t = carve(&w, n);
  double *ratio = carve(&w, n), *dg = carve(&w, n), *gw = carve(&w, 4 * n * n + 8), *qw = carve(&w, 2 * n * n + 65 * n + 64);
  int     mk = 0, got;
  dual_t  s;

  *nact = 0;
  if (n < 1 || n > WBC_QP_N || m < 0 || m > WBC_QP_M)
  {
    return 2;
  }
  if (!m)
  {
    for (int i = 0; i < n; i++)
    {
      t[i] = -g[i];
    }
    return wbc_solve(l, VIEW(H, n, 1), VIEW(t, 1, 0), n, 1, x, gw) ? 2 : 0;
  }
  /* the rows scaled to unit norm, those too short dropped: one of them asking under -TOL, none meets it */
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < n; j++)
    {
      dv[j] = A[i * n + j] * A[i * n + j];
    }
    norm[i] = sqrt(npsum(dv, n));
    if (norm[i] < TINY && b[i] < -TOL)
    {
      return 1;
    }
  }
  for (int i = 0; i < m; i++)
  {
    if (norm[i] >= TINY)
    {
      for (int j = 0; j < n; j++)
      {
        As[mk * n + j] = A[i * n + j] / norm[i];
        NA[mk * n + j] = -As[mk * n + j];
      }
      bs[mk] = b[i] / norm[i];
      d[mk] = -bs[mk];
      mk++;
    }
  }
  /* the rows the last solve ended on, those kept, each once, the first n of them tried first */
  s.q = 0;
  for (int k = 0; k < nwarm; k++)
  {
    const int wi = warm[k];
    int       fi = 0, seen = 0;

    if (wi < 0 || wi >= m || !(norm[wi] >= TINY))
    {
      continue;
    }
    for (int i = 0; i < wi; i++)
    {
      fi += norm[i] >= TINY;
    }
    for (int j = 0; j < s.q; j++)
    {
      seen = seen || act[j] == fi;
    }
    if (!seen && s.q < n)
    {
      act[s.q++] = fi;
    }
  }
  /* J0 = inv(L)', L the Cholesky factor; x0 = -J0 (J0' g), the minimiser with nothing held */
  if (wbc_potrf(l, VIEW(H, n, 1), n, Lc, gw) || wbc_inv(l, VIEW(Lc, n, 1), n, Li, gw))
  {
    return 2;
  }
  wbc_matmul(l, VIEW(Li, n, 1), VIEW(g, 1, 0), t, 1, 0, n, n, 1);
  wbc_matmul(l, VIEW(Li, 1, n), VIEW(t, 1, 0), x0, 1, 0, n, n, 1);
  for (int i = 0; i < n; i++)
  {
    x0[i] = -x0[i];
  }
  if (!mk)
  {
    memcpy(x, x0, (size_t)n * sizeof(double));
    return 0;
  }
  s.l = l;
  s.n = n;
  s.m = mk;
  s.nwork = 2 * n * n + 65 * n + 64;
  s.Li = Li;
  s.NA = NA;
  s.d = d;
  s.Nq = Nq;
  s.X = X;
  s.Qb = Qb;
  s.Jb = Jb;
  s.Rf = Rf;
  s.qw = qw;
  s.act = act;
  got = dual(&s, x0, most, x, u, sv, dv, z, r, t, ratio, dg, met, gw);
  if (got == 2)
  {
    return 2;
  }
  if (got == 0)
  {
    /* the dual's answer still over a row by more than LOOSE: the interior point's instead */
    wbc_matmul(l, VIEW(As, n, 1), VIEW(x, 1, 0), sv, 1, 0, mk, n, 1);
    for (int i = 0; i < mk; i++)
    {
      sv[i] = sv[i] - bs[i];
    }
    got = most_of(sv, mk) > LOOSE;
  }
  if (got)
  {
    got = wbc_qp_interior(l, H, g, As, bs, n, mk, x, act, nact, u, w);
    if (got)
    {
      *nact = 0;
      return got;
    }
  }
  else
  {
    *nact = s.q;
  }
  /* the rows as given, their multipliers unscaled */
  for (int k = 0; k < *nact; k++)
  {
    int fi = -1, orig = 0;

    for (int i = 0; i < m && fi < act[k]; i++)
    {
      if (norm[i] >= TINY)
      {
        fi++;
        orig = i;
      }
    }
    act[k] = orig;
    u[k] = u[k] / norm[orig];
  }
  return 0;
}
