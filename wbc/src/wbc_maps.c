/** wbc_maps.c - machine.stack.wrench._maps in C (wbc_step.h): a leg through its sole's damped inverse, the joints no drive
    holds, each swinging sole's asked acceleration - numpy's arithmetic and its OpenBLAS operation for operation. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** machine.stack.wbc.turn_error: the world-frame rotation vector taking `have` to `want`, rad. */
void wbc_step_turn_error(const wbc_blas_t *l, const double want[4], const double have[4], double out[3])
{
  const double w0 = have[0], x0 = have[1], y0 = have[2], z0 = have[3];
  const double w1 = want[0], x1 = want[1], y1 = want[2], z1 = want[3];
  double       w = ((w1 * w0 + x1 * x0) + y1 * y0) + z1 * z0, s;
  double       v[3] = {((-w1 * x0 + x1 * w0) - y1 * z0) + z1 * y0, ((-w1 * y0 + y1 * w0) - z1 * x0) + x1 * z0,
                       ((-w1 * z0 + z1 * w0) - x1 * y0) + y1 * x0};

  if (w < 0.0)
  {
    w = -w;
    v[0] = -v[0];
    v[1] = -v[1];
    v[2] = -v[2];
  }
  s = sqrt(wbc_dot(l, v, 1, v, 1, 3));
  for (int i = 0; i < 3; i++)
  {
    out[i] = s > 1e-12 ? v[i] * ((2.0 * atan2(s, w)) / s) : 2.0 * v[i];
  }
}

/** machine.stack.wrench._inv: A's damped inverse, `damp2` its lambda^2, A (6, 6) F-ordered as numpy lays a[:, idx]. */
static int inv6(const wbc_step_t *p, const double At[36], double damp2, double out[36], double *work)
{
  double K[36];

  wbc_matmul(&p->blas, VIEW(At, 6, 1), VIEW(At, 1, 6), K, 6, 1, 6, 6, 6);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      K[6 * i + j] = K[6 * i + j] + damp2 * (double)(i == j);
    }
  }
  return wbc_solve(&p->blas, VIEW(K, 6, 1), VIEW(At, 6, 1), 6, 6, out, work);
}

/** machine.stack.wrench._maps: qdd = T base + E u + t0 (E n by nu); each swinging sole's asked a6 and whether asked. */
int wbc_step_maps(const wbc_step_t *p, const double *want, double *T, double *E, double *t0, double *a6,
                double asked[2], int nu, double *work)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nu0 = p->nu0;
  int               hung[WBC_STEP_N] = {0}, pas[WBC_STEP_N], actv[WBC_STEP_N], np_ = 0, na = 0;
  double           *mpp, *mpa, *pull, *Ta, *Ea, *PT, *PE, *ta, *pt, *hp, *y, *rest;

  /* the joints no drive holds */
  for (int i = 0; i < p->npass; i++)
  {
    hung[p->passive[i]] = 1;
  }
  for (int i = 0; i < n; i++)
  {
    if (hung[i])
    {
      pas[np_++] = i;
    }
    else
    {
      actv[na++] = i;
    }
  }
  mpp = work;
  mpa = mpp + np_ * np_;
  pull = mpa + np_ * na;
  Ta = pull + np_ * na;
  Ea = Ta + 6 * na;
  PT = Ea + na * nu0;
  PE = PT + 6 * np_;
  ta = PE + np_ * nu0;
  pt = ta + na;
  hp = pt + np_;
  y = hp + np_;
  rest = y + np_;

  memset(T, 0, (size_t)(6 * n) * sizeof(double));
  memset(E, 0, (size_t)(n * nu) * sizeof(double));
  memset(t0, 0, (size_t)n * sizeof(double));
  memset(a6, 0, (size_t)nu * sizeof(double));
  asked[0] = asked[1] = 0.0;
  for (int i = 0; i < 6; i++)
  {
    T[6 * i + i] = 1.0;
  }
  for (int i = 0; i < p->nheld; i++)
  {
    t0[p->held[i]] = (-p->held_k) * p->v[p->held[i]];
  }
  for (int i = 0; i < p->nupper; i++)
  {
    E[p->act[p->upper[i]] * nu + 12 + i] = 1.0;
  }
  for (int k = 0; k < 2; k++)
  {
    const double *J6 = p->J6[k];
    int           cols[6];
    double        twist[6], A[36], inv[36], neg[36], TT[36], b6[6], t6[6];

    for (int i = 0; i < 6; i++)
    {
      cols[i] = p->act[p->legs[k][i]];
    }
    wbc_matmul(l, VIEW(J6, n, 1), VIEW(p->v, 1, 0), twist, 1, 0, 6, n, 1);
    if (!p->stance[k] && !p->swing[k])
    {
      /* let go: an open chain, each joint an unknown on the swing's row, its foot a contact (machine.stack.wrench._maps) */
      const double loose = (double)(p->let_go[k] != 0);

      for (int i = 0; i < 6; i++)
      {
        const int    j = p->legs[k][i];
        const double opened = p->freed[k] * (p->open_kp * (p->posture[j] - p->q[j]) - p->open_kd * p->v[cols[i]])
                              / p->M[cols[i] * n + cols[i]] + (1.0 - p->freed[k]) * want[j];

        t0[cols[i]] = want[j] * (1.0 - loose);
        E[cols[i] * nu + 6 * k + i] = loose;
        a6[6 * k + i] = loose * opened;
      }
      asked[k] = loose;
      continue;
    }
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        A[6 * j + i] = J6[i * n + cols[j]];
      }
    }
    if (inv6(p, A, p->damp2, inv, rest))
    {
      return 1;
    }
    for (int i = 0; i < 36; i++)
    {
      neg[i] = -inv[i];
    }
    wbc_matmul(l, VIEW(neg, 6, 1), VIEW(J6, n, 1), TT, 6, 1, 6, 6, 6);
    for (int i = 0; i < 6; i++)
    {
      memcpy(T + 6 * cols[i], TT + 6 * i, 6 * sizeof(double));
    }
    if (p->stance[k])
    {
      for (int i = 0; i < 6; i++)
      {
        b6[i] = (p->settle[k][i] - p->dJv[k][i]) - p->contact_k * twist[i];
      }
      wbc_matmul(l, VIEW(inv, 6, 1), VIEW(b6, 1, 0), t6, 1, 0, 6, 6, 1);
      for (int i = 0; i < 6; i++)
      {
        t0[cols[i]] = t6[i];
      }
      continue;
    }
    {
      double te[3];

      wbc_step_turn_error(l, p->swing_quat[k], p->quat[k], te);
      for (int i = 0; i < 3; i++)
      {
        a6[6 * k + i] = p->turn_kd[0] * te[i] - p->turn_kd[1] * twist[i];
        a6[6 * k + 3 + i] = (p->swing_acc[k][i] + p->swing_kd[0] * (p->swing_at[k][i] - p->sole[k][i]))
                            + p->swing_kd[1] * (p->swing_speed[k][i] - twist[3 + i]);
      }
    }
    asked[k] = 1.0;
    wbc_matmul(l, VIEW(neg, 6, 1), VIEW(p->dJv[k], 1, 0), t6, 1, 0, 6, 6, 1);
    for (int i = 0; i < 6; i++)
    {
      t0[cols[i]] = t6[i];
      for (int j = 0; j < 6; j++)
      {
        E[cols[i] * nu + 6 * k + j] = inv[6 * i + j];
      }
    }
  }
  /* the joints no drive holds, by their own rows of her dynamics under all the rest */
  for (int a = 0; a < np_; a++)
  {
    for (int b = 0; b < np_; b++)
    {
      mpp[a * np_ + b] = p->M[pas[a] * n + pas[b]];
    }
    for (int b = 0; b < na; b++)
    {
      mpa[a * na + b] = p->M[pas[a] * n + actv[b]];
    }
    hp[a] = p->h[pas[a]];
  }
  if (wbc_solve(l, VIEW(mpp, np_, 1), VIEW(mpa, na, 1), np_, na, pull, rest))
  {
    return 1;
  }
  for (int i = 0; i < np_ * na; i++)
  {
    pull[i] = -pull[i];
  }
  for (int b = 0; b < na; b++)
  {
    memcpy(Ta + 6 * b, T + 6 * actv[b], 6 * sizeof(double));
    memcpy(Ea + nu0 * b, E + nu * actv[b], (size_t)nu0 * sizeof(double));
    ta[b] = t0[actv[b]];
  }
  wbc_matmul(l, VIEW(pull, na, 1), VIEW(Ta, 6, 1), PT, 6, 1, np_, na, 6);
  wbc_matmul(l, VIEW(pull, na, 1), VIEW(Ea, nu0, 1), PE, nu0, 1, np_, na, nu0);
  wbc_matmul(l, VIEW(pull, na, 1), VIEW(ta, 1, 0), pt, 1, 0, np_, na, 1);
  if (wbc_solve(l, VIEW(mpp, np_, 1), VIEW(hp, 1, 0), np_, 1, y, rest))
  {
    return 1;
  }
  for (int a = 0; a < np_; a++)
  {
    memcpy(T + 6 * pas[a], PT + 6 * a, 6 * sizeof(double));
    memcpy(E + nu * pas[a], PE + nu0 * a, (size_t)nu0 * sizeof(double));
    t0[pas[a]] = pt[a] - y[a];
  }
  return 0;
}
