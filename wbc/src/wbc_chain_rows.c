/** wbc_chain_rows.c - machine.stack.chain_qp.level1 and solved (wbc_step.h): level 1's least squares and laws over
    u = (x, zeta) and the two levels by the dual active set (wbc_qp.c), her posture projected off her held hands;
    numpy's arithmetic and its OpenBLAS (wbc_blas.h) operation for operation, every product in python's order and
    layout; carved 64-byte aligned from the step's workspace. Portable C11, the host's alone. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** `count` doubles from `*at` on, the next carve 64-byte aligned. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** numpy's `a @ b`, a (m, n) and b (n, p) C-ordered, into o (m, p); `a @ x` into y (m). */
static void mm(const wbc_blas_t *l, const double *a, const double *b, double *o, int m, int n, int p)
{
  wbc_matmul(l, VIEW(a, n, 1), VIEW(b, p, 1), o, p, 1, m, n, p);
}

static void mv(const wbc_blas_t *l, const double *a, const double *x, double *y, int m, int n)
{
  wbc_matmul(l, VIEW(a, n, 1), VIEW(x, 1, 0), y, 1, 0, m, n, 1);
}

/** machine.stack.chain_qp.framed of a vector (3k): each contact's three into its frame. */
static void framed1(const double *Q, int k, const double *v, double *out)
{
  for (int i = 0; i < k; i++)
  {
    for (int a = 0; a < 3; a++)
    {
      out[3 * i + a] = (Q[9 * i + 3 * a] * v[3 * i] + Q[9 * i + 3 * a + 1] * v[3 * i + 1]) + Q[9 * i + 3 * a + 2] * v[3 * i + 2];
    }
  }
}

int wbc_chain_rows(const wbc_step_t *p, const wbc_maps_t *m, const double *S, const int *width, const double *mu,
                   const int *held, const double *Jv, const double *f, const double *Q, const double *lo,
                   const double *hi, double *H1, double *g1, double *G1, double *A, double *b, double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         nd = m->nd, nz = m->nz, nu = nd + nz, k3 = m->k3, k = k3 / 3, m1 = m->m1, most = m1 + k3;
  const double      bleed = p->chain_bleed, stick = p->chain_w_stick, rh = sqrt(0.5), cos8 = sqrt(0.5 + 0.5 * rh);
  const double      oct[8][2] = {{1.0, 0.0}, {rh, rh}, {0.0, 1.0}, {-rh, rh}, {-1.0, 0.0}, {-rh, -rh}, {0.0, -1.0}, {rh, -rh}};
  double           *L = carve(w, most * nu + 1), *e = carve(w, most + 1), *LT = carve(w, nu * most + 1);
  double           *vl = carve(w, k3 + 1), *fl = carve(w, k3 + 1), *normal = carve(w, nu);
  int               ml = m1, ma = 0;

  if (11 * k + 2 * nd > WBC_QP_M)
  {
    return -1;
  }
  for (int i = 0; i < m1; i++)
  {
    for (int c = 0; c < nd; c++)
    {
      L[i * nu + c] = m->Rx[i * nd + c];
    }
    for (int c = 0; c < nz; c++)
    {
      L[i * nu + nd + c] = m->Rz[i * nz + c];
    }
    e[i] = m->rr[i];
  }
  framed1(Q, k, Jv, vl);
  framed1(Q, k, f, fl);
  for (int i = 0; i < k; i++)
  {
    const double give = p->cgive[i];
    const double right = ((-bleed) * vl[3 * i]) + give * fl[3 * i] - m->a0[3 * i];
    const double trust = (fl[3 * i] > 0.0 ? fl[3 * i] : 0.0) + p->chain_hold_n;

    for (int c = 0; c < nd; c++)
    {
      normal[c] = m->Ax[(3 * i) * nd + c];
    }
    for (int c = 0; c < nz; c++)
    {
      normal[nd + c] = m->Az[(3 * i) * nz + c] + give * S[(3 * i) * nz + c];
    }
    if (held[i])
    {
      /* its stick rows: the normal's with its give, the tangents' where unknown (machine.stack.chain_qp.level1) */
      for (int c = 0; c < nu; c++)
      {
        L[ml * nu + c] = stick * normal[c];
      }
      e[ml++] = stick * right;
      for (int a = 1; a < 3 && width[i] == 3; a++)
      {
        for (int c = 0; c < nd; c++)
        {
          L[ml * nu + c] = stick * m->Ax[(3 * i + a) * nd + c];
        }
        for (int c = 0; c < nz; c++)
        {
          L[ml * nu + nd + c] = stick * m->Az[(3 * i + a) * nz + c];
        }
        e[ml++] = stick * (((-bleed) * vl[3 * i + a]) - m->a0[3 * i + a]);
      }
    }
    /* the laws: it presses alone, never sinks faster than its give says, leaves no faster than it is unloaded (still it
       carries no more than it feels and a patch's hold, bearing none it may fall away at g), its friction within its
       pyramid */
    for (int c = 0; c < nu; c++)
    {
      A[ma * nu + c] = c < nd ? 0.0 : -S[(3 * i) * nz + (c - nd)];
      A[(ma + 1) * nu + c] = -normal[c];
      A[(ma + 2) * nu + c] = c < nd ? m->Ax[(3 * i) * nd + c] : m->Az[(3 * i) * nz + (c - nd)] + (give + p->g / trust) * S[(3 * i) * nz + (c - nd)];
    }
    b[ma] = 0.0;
    b[ma + 1] = -right;
    b[ma + 2] = right + p->g;
    ma += 3;
    for (int q = 0; q < 8 && width[i] == 3; q++)
    {
      for (int c = 0; c < nu; c++)
      {
        A[ma * nu + c] = c < nd ? 0.0 : (oct[q][0] * S[(3 * i + 1) * nz + (c - nd)] + oct[q][1] * S[(3 * i + 2) * nz + (c - nd)])
                                          - (mu[i] * cos8) * S[(3 * i) * nz + (c - nd)];
      }
      b[ma++] = 0.0;
    }
  }
  for (int i = 0; i < ml; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      LT[c * ml + i] = L[i * nu + c];
    }
  }
  mm(l, LT, L, G1, nu, ml, nu);
  /* each contact's tangents regularised at chain_reg, its normal at chain_nreg: least shear, the normals physics' */
  for (int i = 0, at = nd; i < k; at += width[i], i++)
  {
    normal[at] = p->chain_nreg * p->chain_nreg;
    for (int c = 1; c < width[i]; c++)
    {
      normal[at + c] = p->chain_reg * p->chain_reg;
    }
  }
  for (int i = 0; i < nu; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      H1[i * nu + c] = G1[i * nu + c] + (i == c ? (i < nd ? p->chain_task_lam * p->chain_task_lam : normal[i]) : 0.0);
    }
  }
  mv(l, LT, e, g1, nu, ml);
  for (int i = 0; i < nu; i++)
  {
    g1[i] = -g1[i];
  }
  /* her torques within [lo, hi] */
  for (int i = 0; i < nd; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      const double t = c < nd ? m->Tx[i * nd + c] : m->Tz[i * nz + (c - nd)];

      A[(ma + i) * nu + c] = t;
      A[(ma + nd + i) * nu + c] = -t;
    }
    b[ma + i] = hi[i] - m->t0[i];
    b[ma + nd + i] = m->t0[i] - lo[i];
  }
  return ma + 2 * nd;
}

int wbc_chain_levels(wbc_step_t *p, const wbc_maps_t *m, const double *H1, const double *g1, const double *G1,
                     const double *A, const double *b, int ma, const double *Jg, int ng, const int *hands, int nh,
                     const double *want, double *u, int *met, double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         nd = m->nd, nu = nd + m->nz, n3 = 3 * nh;
  const double      lam2 = p->chain_lam * p->chain_lam, wp = p->chain_w_posture;
  double           *u1 = carve(w, nu), *Y = carve(w, nu * nu), *N = carve(w, nu * nu), *J = carve(w, n3 * nd + 1);
  double           *JT = carve(w, nd * n3 + 1), *K = carve(w, n3 * n3 + 1), *KS = carve(w, n3 * nd + 1), *P = carve(w, nd * nd);
  double           *L2 = carve(w, nd * nu), *R2n = carve(w, nd * nu), *R2nT = carve(w, nu * nd), *e2 = carve(w, nd);
  double           *d = carve(w, nd), *H2 = carve(w, nu * nu), *g2 = carve(w, nu), *AN = carve(w, ma * nu + 1);
  double           *bN = carve(w, ma + 1), *y = carve(w, nu), *t = carve(w, nu), *mult = carve(w, nu + ma + 1);
  double           *gw = carve(w, 4 * nu * nu + 8), *qw = carve(w, WBC_QP_WORK(nu, ma));
  int               warm[WBC_QP_M], got;

  memcpy(warm, p->chain_held[0], (size_t)p->chain_nheld[0] * sizeof(int));
  got = wbc_qp_solve(l, H1, g1, A, b, nu, ma, warm, p->chain_nheld[0], 0, u1, p->chain_held[0], &p->chain_nheld[0], mult, qw);
  if (got == 2)
  {
    return 1;
  }
  *met = got == 0;
  if (!*met)
  {
    for (int i = 0; i < nu; i++)
    {
      t[i] = -g1[i];
    }
    p->chain_nheld[0] = 0;
    if (wbc_solve(l, VIEW(H1, nu, 1), VIEW(t, 1, 0), nu, 1, u1, gw))
    {
      return 1;
    }
  }
  /* level 1's damped null space, and her posture off her held hands (machine.stack.chain_qp.projector) */
  if (wbc_solve(l, VIEW(H1, nu, 1), VIEW(G1, nu, 1), nu, nu, Y, gw))
  {
    return 1;
  }
  for (int i = 0; i < nu; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      N[i * nu + c] = (i == c ? 1.0 : 0.0) - Y[i * nu + c];
    }
  }
  for (int i = 0; i < nd * nd; i++)
  {
    P[i] = (i % (nd + 1) == 0) ? 1.0 : 0.0;
  }
  if (nh)
  {
    for (int h = 0; h < nh; h++)
    {
      for (int a = 0; a < 3; a++)
      {
        for (int c = 0; c < nd; c++)
        {
          J[(3 * h + a) * nd + c] = Jg[(3 * hands[h] + a) * ng + c];
          JT[c * n3 + 3 * h + a] = J[(3 * h + a) * nd + c];
        }
      }
    }
    mm(l, J, JT, K, n3, nd, n3);
    for (int i = 0; i < n3; i++)
    {
      K[i * n3 + i] = K[i * n3 + i] + lam2;
    }
    if (wbc_solve(l, VIEW(K, n3, 1), VIEW(J, nd, 1), n3, nd, KS, gw))
    {
      return 1;
    }
    mm(l, JT, KS, L2, nd, n3, nd);
    for (int i = 0; i < nd * nd; i++)
    {
      P[i] = P[i] - L2[i];
    }
  }
  for (int i = 0; i < nd; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      L2[i * nu + c] = c < nd ? wp * P[i * nd + c] : 0.0;
    }
    d[i] = want[i] - u1[i];
  }
  mm(l, L2, N, R2n, nd, nu, nu);
  for (int i = 0; i < nd; i++)
  {
    for (int c = 0; c < nu; c++)
    {
      R2nT[c * nd + i] = R2n[i * nu + c];
    }
  }
  mv(l, P, d, e2, nd, nd);
  for (int i = 0; i < nd; i++)
  {
    e2[i] = wp * e2[i];
  }
  mm(l, R2nT, R2n, H2, nu, nd, nu);
  for (int i = 0; i < nu; i++)
  {
    H2[i * nu + i] = H2[i * nu + i] + lam2;
  }
  mv(l, R2nT, e2, g2, nu, nd);
  for (int i = 0; i < nu; i++)
  {
    g2[i] = -g2[i];
  }
  got = 1;
  if (*met)
  {
    mm(l, A, N, AN, ma, nu, nu);
    mv(l, A, u1, bN, ma, nu);
    for (int i = 0; i < ma; i++)
    {
      bN[i] = b[i] - bN[i];
    }
    memcpy(warm, p->chain_held[1], (size_t)p->chain_nheld[1] * sizeof(int));
    got = wbc_qp_solve(l, H2, g2, AN, bN, nu, ma, warm, p->chain_nheld[1], 0, y, p->chain_held[1], &p->chain_nheld[1], mult, qw);
    if (got == 2)
    {
      return 1;
    }
  }
  if (got)
  {
    for (int i = 0; i < nu; i++)
    {
      t[i] = -g2[i];
    }
    p->chain_nheld[1] = 0;
    if (wbc_solve(l, VIEW(H2, nu, 1), VIEW(t, 1, 0), nu, 1, y, gw))
    {
      return 1;
    }
  }
  mv(l, N, y, u, nu, nu);
  for (int i = 0; i < nu; i++)
  {
    u[i] = u1[i] + u[i];
  }
  return 0;
}
