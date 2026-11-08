/** wbc_chain.c - machine.stack.chain.step in C (wbc_step.h): her tasks and the contacts' forces one QP within
    friction (wbc_chain_qp.c), her torques those forces', shrunk where no answer met the laws - numpy's arithmetic
    and its OpenBLAS (wbc_blas.h) operation for operation; carved 64-byte aligned from the step's workspace. Portable
    C11. */
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

/** machine.stack.chain.frame: Q (3 x 3, rows) the normal of what bears it (up where none) and two tangents. */
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

int wbc_chain(wbc_step_t *p)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nd = p->nd, nh = p->nheld, nc = p->nc, k3 = 3 * nc, ng = nd + nh;
  int               P[WBC_STEP_N], given[WBC_STEP_D + WBC_STEP_N], hung[WBC_STEP_N] = {0}, np_ = 0, any_over = 0, m1, met = 0;
  double           *base = (double *)(((uintptr_t)p->work + 63u) & ~(uintptr_t)63u), *w = base;
  double            scale = 1.0, slide[3] = {0.0, 0.0, 0.0};

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
  double *J = carve(&w, k3 * n + 1), *Jp = carve(&w, k3 * np_ + 1), *Jg = carve(&w, k3 * ng + 1), *JA = carve(&w, k3 * nd + 1);
  double *Jv = carve(&w, k3 + 1), *dJv = carve(&w, k3 + 1), *Q = carve(&w, 9 * nc + 1), *R = carve(&w, PLAN_ROWS * n);
  double *r = carve(&w, PLAN_ROWS), *want = carve(&w, nd), *RP = carve(&w, PLAN_ROWS * np_), *RG = carve(&w, PLAN_ROWS * ng);
  double *x = carve(&w, nd), *jv6 = carve(&w, 6), *held = carve(&w, nh + 1), *Mpp = carve(&w, np_ * np_), *Mpg = carve(&w, np_ * ng);
  double *Minv = carve(&w, np_ * np_), *rhsh = carve(&w, np_), *MA = carve(&w, nd * (np_ + ng)), *hA = carve(&w, nd);
  double *qddP = carve(&w, np_), *qddP0 = carve(&w, np_), *z = carve(&w, k3 + 1), *qdd1 = carve(&w, n), *qdd0 = carve(&w, n);
  double *tau1 = carve(&w, nd), *tau0 = carve(&w, nd), *tau = carve(&w, nd), *limit = carve(&w, nd), *nlimit = carve(&w, nd);
  double *lo = carve(&w, nd), *hi = carve(&w, nd), *zeros = carve(&w, nd), *rest;

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
    frame(p->cn[q], Q + 9 * q);
  }
  for (int rw = 0; rw < k3; rw++)
  {
    for (int a = 0; a < np_; a++)
    {
      Jp[rw * np_ + a] = J[rw * n + P[a]];
    }
    for (int g = 0; g < ng; g++)
    {
      Jg[rw * ng + g] = J[rw * n + given[g]];
    }
    for (int i = 0; i < nd; i++)
    {
      JA[rw * nd + i] = J[rw * n + p->act[i]];
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
  for (int i = 0; i < nd; i++)
  {
    const int a = p->act[i];

    for (int c = 0; c < np_; c++)
    {
      MA[i * (np_ + ng) + c] = p->M[a * n + P[c]];
    }
    for (int g = 0; g < ng; g++)
    {
      MA[i * (np_ + ng) + np_ + g] = p->M[a * n + given[g]];
    }
    hA[i] = p->h[a];
    limit[i] = (p->wep ? p->wep_nm[i] : p->clamp[i]) * p->derate[i];
    nlimit[i] = -limit[i];
    lo[i] = npmax(-limit[i], p->lo[i]);
    hi[i] = npmin(limit[i], p->hi[i]);
  }
  rest = w;
  if (wbc_inv(l, VIEW(Mpp, np_, 1), np_, Minv, rest))
  {
    return 1;
  }
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
  if (p->chain_vx != 0.0)
  {
    /* the twist told her: her sled's way, level (machine.stack.chain.step) */
    const double *q = p->tquat[0];
    const double  hx = 2.0 * (q[1] * q[3] + q[0] * q[2]), hz = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    const double  level = sqrt(hx * hx + hz * hz), sign = p->chain_vx > 0.0 ? 1.0 : -1.0;

    if (level > 0.0)
    {
      slide[0] = sign * (hx / level);
      slide[2] = sign * (hz / level);
    }
  }
  /* 1. the QP: her driven joints, the contacts' forces and her torques as one */
  if (wbc_chain_qp(p, np_, ng, k3, held, Minv, Mpg, rhsh, Jp, Jg, Jv, dJv, Q, RP, RG, r, m1, want, slide, MA, hA, JA, lo, hi, x,
                   qddP, z, tau1, &p->residual, &met, qddP0, tau0, &w))
  {
    return 1;
  }
  if ((size_t)(w - base) > (size_t)WBC_STEP_WORK - 64u)
  {
    return 1; /* the workspace's bound */
  }
  for (int i = 0; i < nc; i++)
  {
    for (int c = 0; c < 3; c++)
    {
      p->forces[i][c] = z[3 * i + c];
    }
  }
  memset(qdd1, 0, (size_t)n * sizeof(double));
  memset(qdd0, 0, (size_t)n * sizeof(double));
  for (int i = 0; i < nd; i++)
  {
    qdd1[p->act[i]] = x[i];
  }
  for (int i = 0; i < nh; i++)
  {
    qdd1[p->held[i]] = qdd0[p->held[i]] = held[i];
  }
  for (int a = 0; a < np_; a++)
  {
    qdd1[P[a]] = qddP[a];
    qdd0[P[a]] = qddP0[a];
  }
  /* 2. torques: the QP's where its laws were met, else shrunk as the upright step's */
  for (int i = 0; i < nd; i++)
  {
    any_over = any_over || fabs(tau1[i]) > limit[i];
  }
  if (!met && any_over)
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
