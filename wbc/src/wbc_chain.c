/** wbc_chain.c - machine.stack.chain.step in C (wbc_step.h): her step in closed chains - the chain's tasks by one
    damped least squares (Levenberg-Marquardt, Cholesky), the contacts' forces by a box-projected Gauss-Seidel over
    her Delassus matrix on the dofs no drive holds, her torques with them, shrunk -, numpy's arithmetic and its
    OpenBLAS (wbc_blas.h) operation for operation; its arrays carved 64-byte aligned from the step's own workspace,
    nothing allocated. Portable C11, the host's alone (closed_chain_complementarity.lisp). */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})
/** The rows the tasks lay at most: each contact's normal and its patch's two tangents, two turns, two swings, the
    driven joints, the base, the held joints. */
#define CHAIN_ROWS (5 * WBC_STEP_C + 6 + 12 + WBC_STEP_D + 6 + WBC_STEP_N)

/** `count` doubles from `*at` on, the next carve 64-byte aligned after them. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** machine.stack.chain.cholesky: L, lower, with L L^T = G (m, m), row by row; 1 where G is not positive. */
static int cholesky(int m, const double *G, double *L)
{
  memset(L, 0, (size_t)(m * m) * sizeof(double));
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j <= i; j++)
    {
      double s = G[i * m + j];

      for (int k = 0; k < j; k++)
      {
        s = s - L[i * m + k] * L[j * m + k];
      }
      if (i == j)
      {
        if (s <= 0.0)
        {
          return 1;
        }
        L[i * m + i] = sqrt(s);
      }
      else
      {
        L[i * m + j] = s / L[j * m + j];
      }
    }
  }
  return 0;
}

/** machine.stack.chain.solved: x with L L^T x = r, forward into y, back into x. */
static void solved(int m, const double *L, const double *r, double *y, double *x)
{
  for (int i = 0; i < m; i++)
  {
    double s = r[i];

    for (int k = 0; k < i; k++)
    {
      s = s - L[i * m + k] * y[k];
    }
    y[i] = s / L[i * m + i];
  }
  for (int i = m - 1; i >= 0; i--)
  {
    double s = y[i];

    for (int k = i + 1; k < m; k++)
    {
      s = s - L[k * m + i] * x[k];
    }
    x[i] = s / L[i * m + i];
  }
}

/** machine.stack.chain.frame: a frame's rows of a felt force - its direction, up where it feels none, two tangents. */
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

/** One row of the tasks: `row` (n) weighed into R, its right-hand side into r. */
static void laid(double *R, double *r, int *m, int n, const double *row, double c, double weight)
{
  for (int i = 0; i < n; i++)
  {
    R[*m * n + i] = weight * row[i];
  }
  r[*m] = weight * c;
  *m = *m + 1;
}

/** machine.stack.chain.tasks: every acceleration (n) the closed chain's tasks ask, one damped least squares: each
    patch's points on its normal and its centroid on its tangents, the turns, the swings, the posture and the base at
    rest, the held joints, each at the regime's weight (chain_w_*); damped chain_lam squared. 1 raised. */
static int tasks(const wbc_step_t *p, double *R, double *r, double *RT, double *H, double *L, double *g, double *y,
                 double *qs, double *Jm, double *row)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nd = p->nd, nc = p->nc;
  int               m = 0, patch_of[WBC_STEP_C], npatch = 0;
  double            Q[9], fsum[3], jv[6], ci[3], cm[3], err[3], a6[6], want[WBC_STEP_D];

  for (int q = 0; q < nc; q++)
  {
    patch_of[q] = -1;
    for (int o = 0; o < q; o++)
    {
      if (p->cbody[o] == p->cbody[q])
      {
        patch_of[q] = patch_of[o];
        break;
      }
    }
    if (patch_of[q] < 0)
    {
      patch_of[q] = npatch++;
    }
  }
  for (int pi = 0; pi < npatch; pi++)
  {
    int count = 0;

    fsum[0] = fsum[1] = fsum[2] = 0.0;
    for (int q = 0; q < nc; q++)
    {
      if (patch_of[q] == pi)
      {
        fsum[0] = fsum[0] + p->cf[q][0];
        fsum[1] = fsum[1] + p->cf[q][1];
        fsum[2] = fsum[2] + p->cf[q][2];
      }
    }
    frame(fsum, Q);
    memset(Jm, 0, (size_t)(3 * n) * sizeof(double));
    cm[0] = cm[1] = cm[2] = 0.0;
    for (int q = 0; q < nc; q++)
    {
      const double *J = p->cJ[q];

      if (patch_of[q] != pi)
      {
        continue;
      }
      wbc_matmul(l, VIEW(J, n, 1), VIEW(p->v, 1, 0), jv, 1, 0, 3, n, 1);
      for (int a = 0; a < 3; a++)
      {
        ci[a] = (-p->cdJv[q][a]) - p->chain_bleed * jv[a];
      }
      for (int i = 0; i < n; i++)
      {
        row[i] = (Q[0] * J[i] + Q[1] * J[n + i]) + Q[2] * J[2 * n + i];
      }
      laid(R, r, &m, n, row, (Q[0] * ci[0] + Q[1] * ci[1]) + Q[2] * ci[2], p->chain_w_stick);
      for (int i = 0; i < 3 * n; i++)
      {
        Jm[i] = Jm[i] + J[i];
      }
      for (int a = 0; a < 3; a++)
      {
        cm[a] = cm[a] + ci[a];
      }
      count++;
    }
    for (int i = 0; i < 3 * n; i++)
    {
      Jm[i] = Jm[i] / (double)count;
    }
    for (int a = 0; a < 3; a++)
    {
      cm[a] = cm[a] / (double)count;
    }
    for (int a = 1; a < 3; a++)
    {
      for (int i = 0; i < n; i++)
      {
        row[i] = (Q[3 * a] * Jm[i] + Q[3 * a + 1] * Jm[n + i]) + Q[3 * a + 2] * Jm[2 * n + i];
      }
      laid(R, r, &m, n, row, (Q[3 * a] * cm[0] + Q[3 * a + 1] * cm[1]) + Q[3 * a + 2] * cm[2], p->chain_w_stick);
    }
  }
  for (int t = 0; t < p->nturns; t++)
  {
    wbc_step_turn_error(l, p->turn[t], p->tquat[t], err);
    for (int a = 0; a < 3; a++)
    {
      laid(R, r, &m, n, p->Jr[t] + a * n, (p->turn_kd[0] * err[a] - p->turn_kd[1] * p->tw[t][a]) - p->dJr[t][a],
           p->chain_w_turn);
    }
  }
  for (int k = 0; k < 2; k++)
  {
    if (!p->swing[k])
    {
      continue;
    }
    wbc_matmul(l, VIEW(p->J6[k], n, 1), VIEW(p->v, 1, 0), jv, 1, 0, 6, n, 1);
    wbc_step_turn_error(l, p->swing_quat[k], p->quat[k], err);
    for (int a = 0; a < 3; a++)
    {
      a6[a] = p->turn_kd[0] * err[a] - p->turn_kd[1] * jv[a];
      a6[3 + a] = (p->swing_acc[k][a] + p->swing_kd[0] * (p->swing_at[k][a] - p->sole[k][a]))
                  + p->swing_kd[1] * (p->swing_speed[k][a] - jv[3 + a]);
    }
    for (int a = 0; a < 6; a++)
    {
      laid(R, r, &m, n, p->J6[k] + a * n, a6[a] - p->dJv[k][a], p->chain_w_swing);
    }
  }
  for (int i = 0; i < nd; i++)
  {
    want[i] = p->posture_kd[0] * (p->posture[i] - p->q[i]) - p->posture_kd[1] * p->v[p->act[i]];
  }
  memset(row, 0, (size_t)n * sizeof(double));
  for (int i = 0; i < nd; i++)
  {
    row[p->act[i]] = 1.0;
    laid(R, r, &m, n, row, want[i], p->chain_w_posture);
    row[p->act[i]] = 0.0;
  }
  for (int j = 0; j < 6; j++)
  {
    row[j] = 1.0;
    laid(R, r, &m, n, row, 0.0, p->chain_w_posture);
    row[j] = 0.0;
  }
  for (int i = 0; i < p->nheld; i++)
  {
    row[p->held[i]] = 1.0;
    laid(R, r, &m, n, row, (-p->held_k) * p->v[p->held[i]], p->chain_w_stick);
    row[p->held[i]] = 0.0;
  }
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < n; j++)
    {
      RT[j * m + i] = R[i * n + j];
    }
  }
  wbc_matmul(l, VIEW(RT, m, 1), VIEW(R, n, 1), H, n, 1, n, m, n);
  for (int i = 0; i < n; i++)
  {
    H[i * n + i] = H[i * n + i] + p->chain_lam * p->chain_lam;
  }
  wbc_matmul(l, VIEW(RT, m, 1), VIEW(r, 1, 0), g, 1, 0, n, m, 1);
  if (cholesky(n, H, L))
  {
    return 1;
  }
  solved(n, L, g, y, qs);
  return 0;
}

/** machine.stack.chain.pgs: box-projected Gauss-Seidel on w = Dl z + b in the contacts' frames, z in place; the
    complementarity's residual after. */
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

/** machine.stack.chain.step's answered: (the drives' torques, every acceleration) with the forces z, her driven and
    held joints going qg. */
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
  int               P[WBC_STEP_N], given[WBC_STEP_D + WBC_STEP_N], hung[WBC_STEP_N] = {0}, np_ = 0, any_over = 0;
  double           *w = (double *)(((uintptr_t)p->work + 63u) & ~(uintptr_t)63u);
  double            scale = 1.0;

  if (nc > WBC_STEP_C)
  {
    return 1; /* machine.stack.chain lays no more: its mirror refuses them before the call */
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
  double *J = carve(&w, k3 * n), *R = carve(&w, CHAIN_ROWS * n), *RT = carve(&w, CHAIN_ROWS * n), *H = carve(&w, n * n);
  double *L = carve(&w, n * n), *r = carve(&w, CHAIN_ROWS), *gv = carve(&w, n), *y = carve(&w, n), *qs = carve(&w, n);
  double *Jm = carve(&w, 3 * n), *row = carve(&w, n), *qg = carve(&w, ng), *qg0 = carve(&w, ng);
  double *Mpp = carve(&w, np_ * np_), *Mpg = carve(&w, np_ * ng), *Minv = carve(&w, np_ * np_), *t = carve(&w, np_);
  double *rhs = carve(&w, np_), *rhs0 = carve(&w, np_), *Jp = carve(&w, k3 * np_), *JpT = carve(&w, np_ * k3);
  double *Jg = carve(&w, k3 * ng), *W = carve(&w, k3 * np_), *Dl = carve(&w, k3 * k3), *Dloc = carve(&w, k3 * k3);
  double *a0 = carve(&w, k3), *ag = carve(&w, k3), *jv = carve(&w, k3), *bb = carve(&w, k3), *bloc = carve(&w, k3);
  double *zl = carve(&w, k3), *z = carve(&w, k3), *Q = carve(&w, 9 * nc), *JT = carve(&w, n * k3), *Jtz = carve(&w, n);
  double *qdd1 = carve(&w, n), *qdd0 = carve(&w, n), *Mq = carve(&w, n), *push = carve(&w, np_), *qp = carve(&w, np_);
  double *tau1 = carve(&w, nd), *tau0 = carve(&w, nd), *tau = carve(&w, nd), *limit = carve(&w, nd);
  double *nlimit = carve(&w, nd), *lo = carve(&w, nd), *hi = carve(&w, nd), *zeros = carve(&w, nd), *rest = w;

  for (int i = 0; i < nd; i++)
  {
    zeros[i] = 0.0;
  }
  for (int q = 0; q < nc; q++)
  {
    memcpy(J + 3 * q * n, p->cJ[q], (size_t)(3 * n) * sizeof(double));
  }
  /* 1. her driven joints as the closed chain's tasks ask them */
  if (tasks(p, R, r, RT, H, L, gv, y, qs, Jm, row))
  {
    return 1;
  }
  for (int i = 0; i < nd; i++)
  {
    qg[i] = qs[p->act[i]];
    qg0[i] = 0.0;
  }
  for (int i = 0; i < nh; i++)
  {
    qg[nd + i] = qg0[nd + i] = (-p->held_k) * p->v[p->held[i]];
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
  }
  if (wbc_inv(l, VIEW(Mpp, np_, 1), np_, Minv, rest))
  {
    return 1;
  }
  wbc_matmul(l, VIEW(Mpg, ng, 1), VIEW(qg, 1, 0), t, 1, 0, np_, ng, 1);
  for (int a = 0; a < np_; a++)
  {
    rhs[a] = (-p->h[P[a]]) - t[a];
  }
  wbc_matmul(l, VIEW(Mpg, ng, 1), VIEW(qg0, 1, 0), t, 1, 0, np_, ng, 1);
  for (int a = 0; a < np_; a++)
  {
    rhs0[a] = (-p->h[P[a]]) - t[a];
  }
  /* 2. the contacts' forces by complementarity over her Delassus matrix on the dofs no drive holds */
  p->residual = 0.0;
  if (k3)
  {
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
    }
    wbc_matmul(l, VIEW(Jp, np_, 1), VIEW(Minv, np_, 1), W, np_, 1, k3, np_, np_);
    wbc_matmul(l, VIEW(W, np_, 1), VIEW(JpT, k3, 1), Dl, k3, 1, k3, np_, k3);
    wbc_matmul(l, VIEW(W, np_, 1), VIEW(rhs, 1, 0), a0, 1, 0, k3, np_, 1);
    wbc_matmul(l, VIEW(Jg, ng, 1), VIEW(qg, 1, 0), ag, 1, 0, k3, ng, 1);
    wbc_matmul(l, VIEW(J, n, 1), VIEW(p->v, 1, 0), jv, 1, 0, k3, n, 1);
    for (int rw = 0; rw < k3; rw++)
    {
      bb[rw] = ((a0[rw] + ag[rw]) + p->cdJv[rw / 3][rw % 3]) + p->chain_bleed * jv[rw];
    }
    for (int q = 0; q < nc; q++)
    {
      frame(p->cf[q], Q + 9 * q);
    }
    /* into the contacts' frames: Dloc = Q Dl Q^T, bloc = Q b, z warm from what is felt */
    for (int i = 0; i < nc; i++)
    {
      for (int a = 0; a < 3; a++)
      {
        double s = 0.0, u = 0.0;

        for (int c = 0; c < 3; c++)
        {
          s = s + Q[9 * i + 3 * a + c] * bb[3 * i + c];
          u = u + Q[9 * i + 3 * a + c] * p->cf[i][c];
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
  /* 3. her torques, shrunk to her clamps as the upright step's: toward nothing asked of her joints, then toward 0 */
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
