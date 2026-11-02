/** wbc_chain.c - machine.stack.chain.step in C (wbc_step.h): the chain's plan through her dynamics, the contacts'
    forces by complementarity, her torques shrunk - numpy's arithmetic and its OpenBLAS (wbc_blas.h) operation for
    operation; carved 64-byte aligned from the step's workspace. Portable C11. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})
/** Rows above posture at most. */
#define PLAN_ROWS (6 + 12)

/** `count` doubles from `*at` on, the next carve 64-byte aligned. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** machine.stack.chain.frame: Q (3 x 3, rows) the normal (up where none felt) and two tangents. */
static void frame(const double f[3], double Q[9])
{
  const double norm = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  double       n0, n1, n2, e0 = 1.0, e1 = 0.0, e2 = 0.0, t0, t1, t2, tn;

  if (norm > 0.0)
  {
    n0 = f[0] / norm;
    n1 = f[1] / norm;
    n2 = f[2] / norm;
  }
  else
  {
    n0 = 0.0;
    n1 = 1.0;
    n2 = 0.0;
  }
  if (fabs(n0) > 0.9)
  {
    e0 = 0.0;
    e2 = 1.0;
  }
  t0 = n1 * e2 - n2 * e1;
  t1 = n2 * e0 - n0 * e2;
  t2 = n0 * e1 - n1 * e0;
  tn = sqrt(t0 * t0 + t1 * t1 + t2 * t2);
  t0 = t0 / tn;
  t1 = t1 / tn;
  t2 = t2 / tn;
  Q[0] = n0;
  Q[1] = n1;
  Q[2] = n2;
  Q[3] = t0;
  Q[4] = t1;
  Q[5] = t2;
  Q[6] = n1 * t2 - n2 * t1;
  Q[7] = n2 * t0 - n0 * t2;
  Q[8] = n0 * t1 - n1 * t0;
}

/** machine.stack.chain.planned: x (nd) as its python says; carved from `*w` on. */
static int planned(const wbc_step_t *p, int np_, int ng, int k3, const double *held, const double *Minv, const double *Mpg,
                   const double *rhs0, const double *Jp, const double *JpT, const double *Jg, const double *Jv,
                   const double *dJv, const double *Q, const double *RP, const double *RG, const double *r, int m1,
                   const double *want, double *x, double *zwarm, double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         nd = p->nd, nh = ng - nd, nc = k3 / 3, mr = PLAN_ROWS + 3 * WBC_STEP_C;
  const double      lam2 = p->chain_lam * p->chain_lam, task2 = p->chain_task_lam * p->chain_task_lam;
  const double      bleed = p->chain_bleed, mu = p->chain_mu, wf = p->chain_w_force;
  int               rows3[3 * WBC_STEP_C];
  double           *Fk = carve(w, k3), *Jap = carve(w, k3 * np_), *JapT = carve(w, np_ * k3), *Jag = carve(w, k3 * ng);
  double           *Jav = carve(w, k3), *Jad = carve(w, k3), *fa = carve(w, k3), *rhsk = carve(w, np_), *t = carve(w, np_);
  double           *MM = carve(w, np_ * ng), *W1 = carve(w, k3 * np_), *Dl = carve(w, k3 * k3), *Dinv = carve(w, k3 * k3);
  double           *bb0 = carve(w, k3), *WM = carve(w, k3 * ng), *Bg = carve(w, k3 * ng), *c0 = carve(w, k3), *X = carve(w, np_ * k3);
  double           *K = carve(w, np_ * k3), *p0 = carve(w, np_), *KB = carve(w, np_ * ng), *Pg = carve(w, np_ * ng);
  double           *Zx = carve(w, k3 * nd), *d2 = carve(w, k3), *zc = carve(w, k3);
  double           *G0 = carve(w, np_ * np_), *Yi = carve(w, np_ * k3), *Nint = carve(w, k3 * k3), *ZD = carve(w, k3 * nd), *zd = carve(w, k3);
  double           *R1x = carve(w, mr * nd), *t1 = carve(w, m1), *T = carve(w, m1 * nh), *u = carve(w, m1), *r1x = carve(w, mr);
  double           *R1xT = carve(w, nd * mr), *G1 = carve(w, nd * nd), *H1 = carve(w, nd * nd), *g1 = carve(w, nd);
  double           *x1 = carve(w, nd), *Y = carve(w, nd * nd), *N = carve(w, nd * nd), *R2n = carve(w, nd * nd);
  double           *R2nT = carve(w, nd * nd), *H2 = carve(w, nd * nd), *d = carve(w, nd), *g2 = carve(w, nd), *x2 = carve(w, nd);
  double           *Nx = carve(w, nd), *qg = carve(w, ng), *Bq = carve(w, k3), *za = carve(w, k3), *zfull = carve(w, k3);
  double           *rest = *w;

  for (int i = 0; i < nc; i++)
  {
    /* held by what its patch feels - the segments one surface bears, cpatch - not by its own point's share */
    double by = 0.0;

    for (int j = 0; j < nc; j++)
    {
      const double *cf = p->cf[j];

      by = by + (double) (p->cpatch[j] == p->cpatch[i]) * sqrt(cf[0] * cf[0] + cf[1] * cf[1] + cf[2] * cf[2]);
    }
    rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = by >= p->chain_hold_n;
    Fk[3 * i] = Fk[3 * i + 1] = Fk[3 * i + 2] = 0.0;
  }
  wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(Mpg, ng, 1), MM, ng, 1, np_, np_, ng);
  for (int rnd = 0; rnd < p->chain_rounds; rnd++)
  {
    int na = 0, changed = 0, mt, j = 0;

    for (int rw = 0; rw < k3; rw++)
    {
      na = na + rows3[rw];
    }
    for (int rw = 0; rw < k3; rw++)
    {
      if (!rows3[rw])
      {
        continue;
      }
      for (int a = 0; a < np_; a++)
      {
        Jap[j * np_ + a] = Jp[rw * np_ + a];
        JapT[a * na + j] = Jp[rw * np_ + a];
      }
      for (int g = 0; g < ng; g++)
      {
        Jag[j * ng + g] = Jg[rw * ng + g];
      }
      Jav[j] = Jv[rw];
      Jad[j] = dJv[rw];
      fa[j] = p->cf[rw / 3][rw % 3];
      j++;
    }
    if (k3)
    {
      wbc_matmul(l, VIEW(JpT, k3, 1), VIEW(Fk, 1, 0), t, 1, 0, np_, k3, 1);
      for (int a = 0; a < np_; a++)
      {
        rhsk[a] = rhs0[a] + t[a];
      }
    }
    else
    {
      memcpy(rhsk, rhs0, (size_t)np_ * sizeof(double));
    }
    if (na)
    {
      wbc_matmul(l, VIEW(Jap, np_, 1), VIEW(Minv, np_, 1), W1, np_, 1, na, np_, np_);
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(JapT, na, 1), Dl, na, 1, na, np_, na);
      for (int i = 0; i < na; i++)
      {
        Dl[i * na + i] = Dl[i * na + i] + lam2;
      }
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(rhsk, 1, 0), bb0, 1, 0, na, np_, 1);
      for (int i = 0; i < na; i++)
      {
        bb0[i] = bb0[i] + Jad[i];
      }
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(Mpg, ng, 1), WM, ng, 1, na, np_, ng);
      for (int i = 0; i < na * ng; i++)
      {
        Bg[i] = Jag[i] - WM[i];
      }
      for (int i = 0; i < na; i++)
      {
        c0[i] = (-bleed) * Jav[i] - bb0[i];
      }
      if (wbc_inv(l, VIEW(Dl, na, 1), na, Dinv, rest))
      {
        return 1;
      }
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(JapT, na, 1), X, na, 1, np_, np_, na);
      wbc_matmul(l, VIEW(X, na, 1), VIEW(Dinv, na, 1), K, na, 1, np_, na, na);
      wbc_matmul(l, VIEW(K, na, 1), VIEW(c0, 1, 0), p0, 1, 0, np_, na, 1);
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(rhsk, 1, 0), t, 1, 0, np_, np_, 1);
      for (int a = 0; a < np_; a++)
      {
        p0[a] = p0[a] + t[a];
      }
      wbc_matmul(l, VIEW(K, na, 1), VIEW(Bg, ng, 1), KB, ng, 1, np_, na, ng);
      for (int i = 0; i < np_ * ng; i++)
      {
        Pg[i] = -(KB[i] + MM[i]);
      }
      wbc_matmul(l, VIEW(JapT, na, 1), VIEW(Jap, np_, 1), G0, np_, 1, np_, na, np_);
      for (int i = 0; i < np_; i++)
      {
        G0[i * np_ + i] = G0[i * np_ + i] + lam2;
      }
      if (wbc_solve(l, VIEW(G0, np_, 1), VIEW(JapT, na, 1), np_, na, Yi, rest))
      {
        return 1;
      }
      wbc_matmul(l, VIEW(Jap, np_, 1), VIEW(Yi, na, 1), Nint, na, 1, na, np_, na);
      for (int i = 0; i < na; i++)
      {
        for (int c = 0; c < na; c++)
        {
          Nint[i * na + c] = (i == c ? 1.0 : 0.0) - Nint[i * na + c];
        }
      }
      wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(Bg, ng, 1), ZD, nd, 1, na, na, nd);
      wbc_matmul(l, VIEW(Nint, na, 1), VIEW(ZD, nd, 1), Zx, nd, 1, na, na, nd);
      if (nh)
      {
        wbc_matmul(l, VIEW(Bg + nd, ng, 1), VIEW(held, 1, 0), d2, 1, 0, na, nh, 1);
      }
      else
      {
        memset(d2, 0, (size_t)na * sizeof(double));
      }
      for (int i = 0; i < na; i++)
      {
        d2[i] = c0[i] - d2[i];
      }
      wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(d2, 1, 0), zd, 1, 0, na, na, 1);
      for (int i = 0; i < na; i++)
      {
        zd[i] = zd[i] - fa[i];
      }
      wbc_matmul(l, VIEW(Nint, na, 1), VIEW(zd, 1, 0), zc, 1, 0, na, na, 1);
    }
    else
    {
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(rhsk, 1, 0), p0, 1, 0, np_, np_, 1);
      for (int i = 0; i < np_ * ng; i++)
      {
        Pg[i] = -MM[i];
      }
    }
    if (m1)
    {
      wbc_matmul(l, VIEW(RP, np_, 1), VIEW(Pg, ng, 1), R1x, nd, 1, m1, np_, nd);
      for (int i = 0; i < m1; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          R1x[i * nd + c] = R1x[i * nd + c] + RG[i * ng + c];
        }
      }
      wbc_matmul(l, VIEW(RP, np_, 1), VIEW(p0, 1, 0), t1, 1, 0, m1, np_, 1);
      if (nh)
      {
        wbc_matmul(l, VIEW(RP, np_, 1), VIEW(Pg + nd, ng, 1), T, nh, 1, m1, np_, nh);
        for (int i = 0; i < m1; i++)
        {
          for (int c = 0; c < nh; c++)
          {
            T[i * nh + c] = T[i * nh + c] + RG[i * ng + nd + c];
          }
        }
        wbc_matmul(l, VIEW(T, nh, 1), VIEW(held, 1, 0), u, 1, 0, m1, nh, 1);
      }
      else
      {
        memset(u, 0, (size_t)m1 * sizeof(double));
      }
      for (int i = 0; i < m1; i++)
      {
        r1x[i] = (r[i] - t1[i]) - u[i];
      }
    }
    for (int i = 0; i < na; i++)
    {
      for (int c = 0; c < nd; c++)
      {
        R1x[(m1 + i) * nd + c] = wf * Zx[i * nd + c];
      }
      r1x[m1 + i] = wf * zc[i];
    }
    mt = m1 + na;
    if (mt)
    {
      for (int i = 0; i < mt; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          R1xT[c * mt + i] = R1x[i * nd + c];
        }
      }
      wbc_matmul(l, VIEW(R1xT, mt, 1), VIEW(R1x, nd, 1), G1, nd, 1, nd, mt, nd);
      for (int i = 0; i < nd * nd; i++)
      {
        H1[i] = G1[i];
      }
      for (int i = 0; i < nd; i++)
      {
        H1[i * nd + i] = H1[i * nd + i] + task2;
      }
      wbc_matmul(l, VIEW(R1xT, mt, 1), VIEW(r1x, 1, 0), g1, 1, 0, nd, mt, 1);
      if (wbc_solve(l, VIEW(H1, nd, 1), VIEW(g1, 1, 0), nd, 1, x1, rest) || wbc_solve(l, VIEW(H1, nd, 1), VIEW(G1, nd, 1), nd, nd, Y, rest))
      {
        return 1;
      }
      for (int i = 0; i < nd; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          N[i * nd + c] = (i == c ? 1.0 : 0.0) - Y[i * nd + c];
        }
      }
    }
    else
    {
      for (int i = 0; i < nd; i++)
      {
        x1[i] = 0.0;
        for (int c = 0; c < nd; c++)
        {
          N[i * nd + c] = i == c ? 1.0 : 0.0;
        }
      }
    }
    for (int i = 0; i < nd; i++)
    {
      for (int c = 0; c < nd; c++)
      {
        R2n[i * nd + c] = p->chain_w_posture * N[i * nd + c];
        R2nT[c * nd + i] = R2n[i * nd + c];
      }
    }
    wbc_matmul(l, VIEW(R2nT, nd, 1), VIEW(R2n, nd, 1), H2, nd, 1, nd, nd, nd);
    for (int i = 0; i < nd; i++)
    {
      H2[i * nd + i] = H2[i * nd + i] + lam2;
      d[i] = p->chain_w_posture * (want[i] - x1[i]);
    }
    wbc_matmul(l, VIEW(R2nT, nd, 1), VIEW(d, 1, 0), g2, 1, 0, nd, nd, 1);
    if (wbc_solve(l, VIEW(H2, nd, 1), VIEW(g2, 1, 0), nd, 1, x2, rest))
    {
      return 1;
    }
    wbc_matmul(l, VIEW(N, nd, 1), VIEW(x2, 1, 0), Nx, 1, 0, nd, nd, 1);
    for (int i = 0; i < nd; i++)
    {
      x[i] = x1[i] + Nx[i];
      qg[i] = x[i];
    }
    for (int i = 0; i < nh; i++)
    {
      qg[nd + i] = held[i];
    }
    if (!na)
    {
      memcpy(zwarm, Fk, (size_t)k3 * sizeof(double));
      break;
    }
    wbc_matmul(l, VIEW(Bg, ng, 1), VIEW(qg, 1, 0), Bq, 1, 0, na, ng, 1);
    for (int i = 0; i < na; i++)
    {
      Bq[i] = c0[i] - Bq[i];
    }
    wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(Bq, 1, 0), za, 1, 0, na, na, 1);
    j = 0;
    for (int rw = 0; rw < k3; rw++)
    {
      zfull[rw] = rows3[rw] ? Fk[rw] + za[j++] : Fk[rw];
      zwarm[rw] = zfull[rw];
    }
    for (int i = 0; i < nc; i++)
    {
      double loc[3];

      if (!rows3[3 * i])
      {
        continue;
      }
      for (int a = 0; a < 3; a++)
      {
        loc[a] = (Q[9 * i + 3 * a] * zfull[3 * i] + Q[9 * i + 3 * a + 1] * zfull[3 * i + 1]) + Q[9 * i + 3 * a + 2] * zfull[3 * i + 2];
      }
      if (loc[0] < 0.0)
      {
        rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        Fk[3 * i] = Fk[3 * i + 1] = Fk[3 * i + 2] = 0.0;
        changed = 1;
      }
      else if (rows3[3 * i + 1] && npmax(fabs(loc[1]), fabs(loc[2])) > mu * loc[0])
      {
        const double lim = mu * loc[0], tb1 = npclip(loc[1], -lim, lim), tb2 = npclip(loc[2], -lim, lim);

        for (int c = 0; c < 3; c++)
        {
          Fk[3 * i + c] = Q[9 * i + 3 + c] * tb1 + Q[9 * i + 6 + c] * tb2;
        }
        rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        changed = 1;
      }
    }
    if (!changed)
    {
      break;
    }
  }
  return 0;
}

/** machine.stack.chain.pgs: box-projected Gauss-Seidel on w = Dl z + b, z in place; the residual after. */
static double pgs(int m, const double *Dl, const double *b, double *z, double mu, double soft, int sweeps)
{
  double residual = 0.0;

  for (int sweep = 0; sweep < sweeps; sweep++)
  {
    for (int r = 0; r < m; r++)
    {
      double w = b[r], d, x;

      for (int c = 0; c < m; c++)
      {
        w = w + Dl[r * m + c] * z[c];
      }
      d = Dl[r * m + r] + soft;
      x = z[r] - w / d;
      if (r % 3 == 0)
      {
        z[r] = x > 0.0 ? x : 0.0;
      }
      else
      {
        const double lim = mu * z[r - r % 3];

        z[r] = x < -lim ? -lim : (x > lim ? lim : x);
      }
    }
  }
  for (int i = 0; i < m; i += 3)
  {
    double w = b[i], gap;

    for (int c = 0; c < m; c++)
    {
      w = w + Dl[i * m + c] * z[c];
    }
    gap = fabs(w < z[i] ? w : z[i]);
    residual = gap > residual ? gap : residual;
  }
  return residual;
}

/** machine.stack.chain.step's answered: tau and qdd with the forces z, her driven and held joints going qg. */
static void answered(const wbc_step_t *p, int np_, const int *P, int ng, const int *given, int k3, const double *JpT,
                     const double *z, const double *Minv, const double *qg, const double *rhs, const double *Jtz,
                     double *push, double *qp, double *Mq, double *qdd, double *tau)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n;

  memset(qdd, 0, (size_t)n * sizeof(double));
  for (int g = 0; g < ng; g++)
  {
    qdd[given[g]] = qg[g];
  }
  if (k3)
  {
    wbc_matmul(l, VIEW(JpT, k3, 1), VIEW(z, 1, 0), push, 1, 0, np_, k3, 1);
  }
  else
  {
    memset(push, 0, (size_t)np_ * sizeof(double));
  }
  for (int a = 0; a < np_; a++)
  {
    push[a] = push[a] + rhs[a];
  }
  wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(push, 1, 0), qp, 1, 0, np_, np_, 1);
  for (int a = 0; a < np_; a++)
  {
    qdd[P[a]] = qp[a];
  }
  wbc_matmul(l, VIEW(p->M, n, 1), VIEW(qdd, 1, 0), Mq, 1, 0, n, n, 1);
  for (int i = 0; i < p->nd; i++)
  {
    const int a = p->act[i];

    tau[i] = (Mq[a] + p->h[a]) - Jtz[a];
  }
}

int wbc_chain(wbc_step_t *p)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nd = p->nd, nh = p->nheld, nc = p->nc, k3 = 3 * nc, ng = nd + nh;
  int               P[WBC_STEP_N], given[WBC_STEP_D + WBC_STEP_N], hung[WBC_STEP_N] = {0}, np_ = 0, any_over = 0, m1;
  double           *base = (double *)(((uintptr_t)p->work + 63u) & ~(uintptr_t)63u), *w = base;
  double            scale = 1.0;

  if (nc > WBC_STEP_C)
  {
    return 1;
  }
  for (int i = 0; i < p->npass; i++)
  {
    hung[p->passive[i]] = 1;
  }
  for (int i = 0; i < n; i++)
  {
    if (i < 6 || hung[i])
    {
      P[np_++] = i;
    }
  }
  for (int i = 0; i < nd; i++)
  {
    given[i] = p->act[i];
  }
  for (int i = 0; i < nh; i++)
  {
    given[nd + i] = p->held[i];
  }
  double *J = carve(&w, k3 * n), *Jp = carve(&w, k3 * np_), *JpT = carve(&w, np_ * k3), *Jg = carve(&w, k3 * ng);
  double *Jv = carve(&w, k3), *dJv = carve(&w, k3), *Q = carve(&w, 9 * nc), *R = carve(&w, PLAN_ROWS * n);
  double *r = carve(&w, PLAN_ROWS), *want = carve(&w, nd), *RP = carve(&w, PLAN_ROWS * np_), *RG = carve(&w, PLAN_ROWS * ng);
  double *x = carve(&w, nd), *jv6 = carve(&w, 6), *qg = carve(&w, ng), *qg0 = carve(&w, ng), *held = carve(&w, nh);
  double *Mpp = carve(&w, np_ * np_), *Mpg = carve(&w, np_ * ng), *Minv = carve(&w, np_ * np_), *t = carve(&w, np_);
  double *rhsh = carve(&w, np_), *rhs = carve(&w, np_), *rhs0 = carve(&w, np_), *W = carve(&w, k3 * np_);
  double *Dl = carve(&w, k3 * k3), *Dloc = carve(&w, k3 * k3), *a0 = carve(&w, k3), *ag = carve(&w, k3), *bb = carve(&w, k3);
  double *bloc = carve(&w, k3), *zl = carve(&w, k3), *z = carve(&w, k3), *zwarm = carve(&w, k3), *JT = carve(&w, n * k3), *Jtz = carve(&w, n);
  double *qdd1 = carve(&w, n), *qdd0 = carve(&w, n), *Mq = carve(&w, n), *push = carve(&w, np_), *qp = carve(&w, np_);
  double *tau1 = carve(&w, nd), *tau0 = carve(&w, nd), *tau = carve(&w, nd), *limit = carve(&w, nd);
  double *nlimit = carve(&w, nd), *lo = carve(&w, nd), *hi = carve(&w, nd), *zeros = carve(&w, nd), *rest;

  for (int i = 0; i < nd; i++)
  {
    zeros[i] = 0.0;
  }
  for (int i = 0; i < nh; i++)
  {
    held[i] = (-p->held_k) * p->v[p->held[i]];
  }
  for (int q = 0; q < nc; q++)
  {
    memcpy(J + 3 * q * n, p->cJ[q], (size_t)(3 * n) * sizeof(double));
    frame(p->cf[q], Q + 9 * q);
  }
  for (int rw = 0; rw < k3; rw++)
  {
    for (int a = 0; a < np_; a++)
    {
      Jp[rw * np_ + a] = J[rw * n + P[a]];
      JpT[a * k3 + rw] = Jp[rw * np_ + a];
    }
    for (int g = 0; g < ng; g++)
    {
      Jg[rw * ng + g] = J[rw * n + given[g]];
    }
    dJv[rw] = p->cdJv[rw / 3][rw % 3];
  }
  if (k3)
  {
    wbc_matmul(l, VIEW(J, n, 1), VIEW(p->v, 1, 0), Jv, 1, 0, k3, n, 1);
  }
  for (int a = 0; a < np_; a++)
  {
    for (int c = 0; c < np_; c++)
    {
      Mpp[a * np_ + c] = p->M[P[a] * n + P[c]];
    }
    for (int g = 0; g < ng; g++)
    {
      Mpg[a * ng + g] = p->M[P[a] * n + given[g]];
    }
    rhsh[a] = -p->h[P[a]];
  }
  rest = w;
  if (wbc_inv(l, VIEW(Mpp, np_, 1), np_, Minv, rest))
  {
    return 1;
  }
  /* 1. the plan */
  m1 = wbc_chain_tasks(p, R, r, want, jv6);
  for (int i = 0; i < m1; i++)
  {
    for (int a = 0; a < np_; a++)
    {
      RP[i * np_ + a] = R[i * n + P[a]];
    }
    for (int g = 0; g < ng; g++)
    {
      RG[i * ng + g] = R[i * n + given[g]];
    }
  }
  memset(zwarm, 0, (size_t)k3 * sizeof(double));
  if (planned(p, np_, ng, k3, held, Minv, Mpg, rhsh, Jp, JpT, Jg, Jv, dJv, Q, RP, RG, r, m1, want, x, zwarm, &w))
  {
    return 1;
  }
  if ((size_t)(w - base) + 4u * (size_t)WBC_STEP_U * (size_t)WBC_STEP_U > (size_t)WBC_STEP_WORK - 64u)
  {
    return 1; /* the workspace's bound */
  }
  for (int i = 0; i < nd; i++)
  {
    qg[i] = x[i];
    qg0[i] = 0.0;
  }
  for (int i = 0; i < nh; i++)
  {
    qg[nd + i] = qg0[nd + i] = held[i];
  }
  wbc_matmul(l, VIEW(Mpg, ng, 1), VIEW(qg, 1, 0), t, 1, 0, np_, ng, 1);
  for (int a = 0; a < np_; a++)
  {
    rhs[a] = rhsh[a] - t[a];
  }
  wbc_matmul(l, VIEW(Mpg, ng, 1), VIEW(qg0, 1, 0), t, 1, 0, np_, ng, 1);
  for (int a = 0; a < np_; a++)
  {
    rhs0[a] = rhsh[a] - t[a];
  }
  /* 2. forces */
  p->residual = 0.0;
  if (k3)
  {
    wbc_matmul(l, VIEW(Jp, np_, 1), VIEW(Minv, np_, 1), W, np_, 1, k3, np_, np_);
    wbc_matmul(l, VIEW(W, np_, 1), VIEW(JpT, k3, 1), Dl, k3, 1, k3, np_, k3);
    wbc_matmul(l, VIEW(W, np_, 1), VIEW(rhs, 1, 0), a0, 1, 0, k3, np_, 1);
    wbc_matmul(l, VIEW(Jg, ng, 1), VIEW(qg, 1, 0), ag, 1, 0, k3, ng, 1);
    for (int rw = 0; rw < k3; rw++)
    {
      bb[rw] = ((a0[rw] + ag[rw]) + dJv[rw]) + p->chain_bleed * Jv[rw];
    }
    /* framed, z warm from the plan's */
    for (int i = 0; i < nc; i++)
    {
      for (int a = 0; a < 3; a++)
      {
        double s = 0.0, u = 0.0;

        for (int c = 0; c < 3; c++)
        {
          s = s + Q[9 * i + 3 * a + c] * bb[3 * i + c];
          u = u + Q[9 * i + 3 * a + c] * zwarm[3 * i + c];
        }
        bloc[3 * i + a] = s;
        zl[3 * i + a] = u;
        for (int j = 0; j < nc; j++)
        {
          for (int e = 0; e < 3; e++)
          {
            s = 0.0;
            for (int c = 0; c < 3; c++)
            {
              for (int d = 0; d < 3; d++)
              {
                s = s + Q[9 * i + 3 * a + c] * Dl[(3 * i + c) * k3 + 3 * j + d] * Q[9 * j + 3 * e + d];
              }
            }
            Dloc[(3 * i + a) * k3 + 3 * j + e] = s;
          }
        }
      }
    }
    p->residual = pgs(k3, Dloc, bloc, zl, p->chain_mu, p->chain_soft, p->chain_sweeps);
    for (int i = 0; i < nc; i++)
    {
      for (int c = 0; c < 3; c++)
      {
        double s = 0.0;

        for (int a = 0; a < 3; a++)
        {
          s = s + Q[9 * i + 3 * a + c] * zl[3 * i + a];
        }
        z[3 * i + c] = s;
        p->forces[i][c] = s;
      }
    }
    for (int rw = 0; rw < k3; rw++)
    {
      for (int i = 0; i < n; i++)
      {
        JT[i * k3 + rw] = J[rw * n + i];
      }
    }
    wbc_matmul(l, VIEW(JT, k3, 1), VIEW(z, 1, 0), Jtz, 1, 0, n, k3, 1);
  }
  else
  {
    memset(Jtz, 0, (size_t)n * sizeof(double));
  }
  /* 3. torques, shrunk */
  answered(p, np_, P, ng, given, k3, JpT, z, Minv, qg, rhs, Jtz, push, qp, Mq, qdd1, tau1);
  answered(p, np_, P, ng, given, k3, JpT, z, Minv, qg0, rhs0, Jtz, push, qp, Mq, qdd0, tau0);
  for (int i = 0; i < nd; i++)
  {
    limit[i] = (p->wep ? p->wep_nm[i] : p->clamp[i]) * p->derate[i];
    nlimit[i] = -limit[i];
    lo[i] = npmax(-limit[i], p->lo[i]);
    hi[i] = npmin(limit[i], p->hi[i]);
    any_over = any_over || fabs(tau1[i]) > limit[i];
  }
  if (any_over)
  {
    int    over0 = 0;
    double s0 = 1.0;

    scale = wbc_step_shrunk(nd, tau1, tau0, nlimit, limit);
    for (int i = 0; i < nd; i++)
    {
      tau[i] = tau0[i] + scale * (tau1[i] - tau0[i]);
      over0 = over0 || fabs(tau0[i]) > limit[i];
    }
    for (int i = 0; i < n; i++)
    {
      qdd1[i] = qdd0[i] + scale * (qdd1[i] - qdd0[i]);
    }
    if (over0)
    {
      s0 = wbc_step_shrunk(nd, tau, zeros, nlimit, limit);
    }
    if (s0 < 1.0)
    {
      for (int i = 0; i < nd; i++)
      {
        tau[i] = s0 * tau[i];
      }
      for (int i = 0; i < n; i++)
      {
        qdd1[i] = s0 * qdd1[i];
      }
    }
    scale = s0 * scale;
  }
  else
  {
    memcpy(tau, tau1, (size_t)nd * sizeof(double));
  }
  for (int i = 0; i < nd; i++)
  {
    p->over[i] = fabs(tau1[i]) > p->clamp[i] * (1.0 + 1e-6);
    p->load[i] = fabs(tau1[i]) / p->clamp[i];
    p->tau[i] = npclip(tau[i], lo[i], hi[i]);
  }
  for (int q = 0; q < p->npairs; q++)
  {
    const int a = p->pairs[q][0], c = p->pairs[q][1], either = p->over[a] || p->over[c];

    p->over[a] = p->over[c] = either;
  }
  memcpy(p->qacc, qdd1, (size_t)n * sizeof(double));
  p->bears[0] = p->bears[1] = 0.0;
  p->cop_n = 0;
  p->scale = scale;
  return 0;
}
