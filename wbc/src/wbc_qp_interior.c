/** wbc_qp_interior.c - machine.stack.qp.interior (wbc_qp.h): Mehrotra's predictor-corrector where the dual active
    set gives up - numpy's arithmetic and its OpenBLAS (wbc_blas.h) operation for operation, every product in
    python's order and layout. Portable C11, the host's alone. */
#include "wbc_np.h"
#include "wbc_qp.h"
#include "wbc_libm.h"

#include <math.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})
/** The predictor-corrector's steps at most (machine.stack.qp.interior's `most`). */
#define STEPS 60

/** What the Newton step reads: A (m by n, the rows scaled), the residuals rd (n) and rp (m), the slacks s and
    multipliers lam (m), D = lam / s, the Cholesky factor C (n by n); w1 (m), w2 (n), t (n) and gw scratch. */
typedef struct
{
  const wbc_blas_t      *l;
  int                    n, m;
  const double *restrict A, *restrict rd, *restrict rp, *restrict s, *restrict lam, *restrict D, *restrict C;
  double *restrict       w1, *restrict w2, *restrict t, *restrict gw;
} newton_t;

/** `count` doubles from `*at` on, the next carve 64-byte aligned. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** numpy's .max() of `a` (n). */
static double most_of(const double *a, int n)
{
  double r = a[0];

  for (int i = 1; i < n; i++)
  {
    r = npmaximum(r, a[i]);
  }
  return r;
}

/** machine.stack.qp.interior's newton: the step dx (n), dl and ds (m) for the complementarity residual rc (m). 0, or
    2 where numpy raises. */
static int newton(const newton_t *k, const double *restrict rc, double *restrict dx, double *restrict dl,
                  double *restrict ds)
{
  const int              n = k->n, m = k->m;
  const double *restrict rd = k->rd, *restrict rp = k->rp, *restrict s = k->s, *restrict lam = k->lam, *restrict D = k->D;
  double *restrict       w1 = k->w1, *restrict w2 = k->w2;

  for (int i = 0; i < m; i++)
  {
    w1[i] = D[i] * rp[i] - rc[i] / s[i];
  }
  wbc_matmul(k->l, VIEW(k->A, 1, n), VIEW(w1, 1, 0), w2, 1, 0, n, m, 1);
  for (int i = 0; i < n; i++)
  {
    w2[i] = -rd[i] - w2[i];
  }
  if (wbc_solve(k->l, VIEW(k->C, n, 1), VIEW(w2, 1, 0), n, 1, k->t, k->gw)
      || wbc_solve(k->l, VIEW(k->C, 1, n), VIEW(k->t, 1, 0), n, 1, dx, k->gw))
  {
    return 2;
  }
  wbc_matmul(k->l, VIEW(k->A, n, 1), VIEW(dx, 1, 0), dl, 1, 0, m, n, 1);
  for (int i = 0; i < m; i++)
  {
    dl[i] = D[i] * (dl[i] + rp[i]) - rc[i] / s[i];
    ds[i] = -(rc[i] + s[i] * dl[i]) / lam[i];
  }
  return 0;
}

/** machine.stack.qp.interior's reach: how far along dv keeps v (m) positive, 1 at most. */
static double reach(const double *v, const double *dv, int m)
{
  double r = INFINITY;
  int    any = 0;

  for (int i = 0; i < m; i++)
  {
    if (dv[i] < 0.0)
    {
      const double val = -v[i] / dv[i];

      r = any ? npminimum(r, val) : val;
      any = 1;
    }
  }
  return any ? (r < 1.0 ? r : 1.0) : 1.0;
}

int wbc_qp_interior(const wbc_blas_t *l, const double *restrict H, const double *restrict g, const double *restrict A,
                    const double *restrict b, int n, int m, double *restrict x, int *restrict act, int *restrict nact,
                    double *restrict u, double *work)
{
  double          *w = work;
  double *restrict rd = carve(&w, n), *restrict rp = carve(&w, m + 1), *restrict s = carve(&w, m + 1);
  double *restrict lam = carve(&w, m + 1), *restrict D = carve(&w, m + 1), *restrict Ad = carve(&w, m * n + 1);
  double *restrict K = carve(&w, n * n), *restrict Kt = carve(&w, n * n), *restrict C = carve(&w, n * n);
  double *restrict dx = carve(&w, n), *restrict dl = carve(&w, m + 1), *restrict ds = carve(&w, m + 1);
  double *restrict w1 = carve(&w, m + 1), *restrict w2 = carve(&w, n), *restrict t = carve(&w, n);
  double *restrict rc = carve(&w, m + 1), *restrict la = carve(&w, m + 1), *restrict gw = carve(&w, 4 * n * n + 8);
  const double ties[4] = {0.0, 1e-12, 1e-9, 1e-6};
  const newton_t k = {l, n, m, A, rd, rp, s, lam, D, C, w1, w2, t, gw};
  double       scale, ga, ba;

  *nact = 0;
  for (int i = 0; i < n; i++)
  {
    x[i] = 0.0;
    t[i] = fabs(g[i]);
  }
  if (!m)
  {
    for (int i = 0; i < n; i++)
    {
      t[i] = -g[i];
    }
    return wbc_solve(l, VIEW(H, n, 1), VIEW(t, 1, 0), n, 1, x, gw) ? 2 : 0;
  }
  ga = most_of(t, n);
  for (int i = 0; i < m; i++)
  {
    rp[i] = fabs(b[i]);
  }
  ba = most_of(rp, m);
  scale = 1.0 + (ba > ga ? ba : ga);
  wbc_matmul(l, VIEW(A, n, 1), VIEW(x, 1, 0), rp, 1, 0, m, n, 1);
  for (int i = 0; i < m; i++)
  {
    s[i] = npmaximum(b[i] - rp[i], 1.0);
    lam[i] = 1.0;
  }
  for (int it = 0; it < STEPS; it++)
  {
    double mu, dmax, sigma, a, a2;
    int    found = 0;

    /* rd = H x + g + A' lam, rp = A x + s - b */
    wbc_matmul(l, VIEW(H, n, 1), VIEW(x, 1, 0), rd, 1, 0, n, n, 1);
    wbc_matmul(l, VIEW(A, 1, n), VIEW(lam, 1, 0), w2, 1, 0, n, m, 1);
    for (int i = 0; i < n; i++)
    {
      rd[i] = (rd[i] + g[i]) + w2[i];
      t[i] = fabs(rd[i]);
    }
    wbc_matmul(l, VIEW(A, n, 1), VIEW(x, 1, 0), rp, 1, 0, m, n, 1);
    for (int i = 0; i < m; i++)
    {
      rp[i] = (rp[i] + s[i]) - b[i];
      w1[i] = fabs(rp[i]);
    }
    mu = wbc_dot(l, s, 1, lam, 1, m) / (double)m;
    if (most_of(t, n) < 1e-8 * scale && most_of(w1, m) < 1e-8 * scale && mu < 1e-10 * scale)
    {
      for (int i = 0; i < m; i++)
      {
        if (s[i] < 1e-6 * scale)
        {
          act[*nact] = i;
          u[*nact] = lam[i];
          *nact = *nact + 1;
        }
      }
      return 0;
    }
    /* K = H + A' (A D) tied on its diagonal where Cholesky refuses it */
    for (int i = 0; i < m; i++)
    {
      D[i] = lam[i] / s[i];
      for (int j = 0; j < n; j++)
      {
        Ad[i * n + j] = A[i * n + j] * D[i];
      }
    }
    wbc_matmul(l, VIEW(A, 1, n), VIEW(Ad, n, 1), K, n, 1, n, m, n);
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < n; j++)
      {
        K[i * n + j] = H[i * n + j] + K[i * n + j];
      }
      t[i] = K[i * n + i];
    }
    dmax = most_of(t, n);
    for (int c = 0; c < 4 && !found; c++)
    {
      const double tie = ties[c] * dmax;

      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < n; j++)
        {
          Kt[i * n + j] = K[i * n + j] + tie * (i == j ? 1.0 : 0.0);
        }
      }
      found = !wbc_potrf(l, VIEW(Kt, n, 1), n, C, gw);
    }
    if (!found)
    {
      return 1;
    }
    /* the predictor, then the corrector at sigma */
    for (int i = 0; i < m; i++)
    {
      rc[i] = s[i] * lam[i];
    }
    /* a solve through C numpy refuses: none, as where Cholesky refuses K (machine.stack.qp.interior) */
    if (newton(&k, rc, dx, dl, ds))
    {
      return 1;
    }
    a = reach(s, ds, m);
    a2 = reach(lam, dl, m);
    a = a2 < a ? a2 : a;
    for (int i = 0; i < m; i++)
    {
      w1[i] = s[i] + a * ds[i];
      la[i] = lam[i] + a * dl[i];
    }
    sigma = wbc_cube((wbc_dot(l, w1, 1, la, 1, m) / (double)m) / mu);
    for (int i = 0; i < m; i++)
    {
      rc[i] = (s[i] * lam[i] + ds[i] * dl[i]) - sigma * mu;
    }
    if (newton(&k, rc, dx, dl, ds))
    {
      return 1;
    }
    a = reach(s, ds, m);
    a2 = reach(lam, dl, m);
    a = 0.99 * (a2 < a ? a2 : a);
    for (int i = 0; i < n; i++)
    {
      x[i] = x[i] + a * dx[i];
    }
    for (int i = 0; i < m; i++)
    {
      lam[i] = lam[i] + a * dl[i];
      s[i] = s[i] + a * ds[i];
    }
  }
  return 1;
}
