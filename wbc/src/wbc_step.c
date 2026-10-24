/** wbc_step.c - her whole body's step on the host (wbc_step.h): machine.wrench.step - its maps,
    its rows, its rounds (wbc_rounds.c), its shrink - numpy's arithmetic and numpy's own OpenBLAS
    (wbc_blas.h) operation for operation. Portable C11, the host's alone. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** Python's max and min of two floats: the first unless the second is past it. */
static double pymax(double a, double b)
{
  return b > a ? b : a;
}

static double pymin(double a, double b)
{
  return b < a ? b : a;
}

/** numpy's clip (clip.cpp): nan kept, a bound where the value is not inside it. */
static double clipped(double x, double lo, double hi)
{
  const double m = isnan(x) ? x : (x > lo ? x : lo);

  return isnan(m) ? m : (m < hi ? m : hi);
}

/** machine.wbc.turn_error: the world-frame rotation vector taking `have` to `want`, rad. */
static void turn_error(const wbc_blas_t *l, const double want[4], const double have[4], double out[3])
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

/** machine.wrench._inv: A's damped inverse, `damp2` its lambda^2, A (6, 6) as numpy's a[:, idx]
    lays it - F-ordered, its memory A.T's in C order. */
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

/** machine.wrench._maps: qdd = T base + E u + t0 (E n by nu, its contacts' columns 0); each
    swinging sole's asked acceleration a6 and whether it is asked. */
static int maps(const wbc_step_t *p, const double *want, double *T, double *E, double *t0, double *a6,
                double asked[2], int nu, double *work)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nu0 = p->nu0, np_ = p->npass, na = p->nact;
  double           *mpp = work, *mpa = mpp + np_ * np_, *pull = mpa + np_ * na, *Ta = pull + np_ * na;
  double           *Ea = Ta + 6 * na, *PT = Ea + na * nu0, *PE = PT + 6 * np_, *ta = PE + np_ * nu0;
  double           *pt = ta + na, *hp = pt + np_, *y = hp + np_, *rest = y + np_;

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
      for (int i = 0; i < 6; i++)
      {
        t0[cols[i]] = want[p->legs[k][i]];
      }
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

      turn_error(l, p->swing_quat[k], p->quat[k], te);
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
      mpp[a * np_ + b] = p->M[p->passive[a] * n + p->passive[b]];
    }
    for (int b = 0; b < na; b++)
    {
      mpa[a * na + b] = p->M[p->passive[a] * n + p->active[b]];
    }
    hp[a] = p->h[p->passive[a]];
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
    memcpy(Ta + 6 * b, T + 6 * p->active[b], 6 * sizeof(double));
    memcpy(Ea + nu0 * b, E + nu * p->active[b], (size_t)nu0 * sizeof(double));
    ta[b] = t0[p->active[b]];
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
    memcpy(T + 6 * p->passive[a], PT + 6 * a, 6 * sizeof(double));
    memcpy(E + nu * p->passive[a], PE + nu0 * a, (size_t)nu0 * sizeof(double));
    t0[p->passive[a]] = pt[a] - y[a];
  }
  return 0;
}

/** The drives' torques and every acceleration at the unknowns `u` (step's torques). */
static void torques(const wbc_step_t *p, const double *full, const double *full0, const double stance[2],
                    const double *u, double alpha, int nu, double *tau, double *qdd, double *work)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nd = p->nd;
  double           *q = work, *Mq = q + n, *Ja = Mq + n, *y = Ja + 6 * nd, x6[6];

  wbc_matmul(l, VIEW(full, nu, 1), VIEW(u, 1, 0), q, 1, 0, n, nu, 1);
  for (int i = 0; i < n; i++)
  {
    qdd[i] = q[i] + (alpha == 1.0 ? full0[i] : alpha * full0[i]);
  }
  wbc_matmul(l, VIEW(p->M, n, 1), VIEW(qdd, 1, 0), Mq, 1, 0, n, n, 1);
  for (int i = 0; i < nd; i++)
  {
    const double hi = p->h[p->act[i]];

    tau[i] = Mq[p->act[i]] + (alpha == 1.0 ? hi : alpha * hi);
  }
  for (int c = 0; c < p->nc; c++)
  {
    for (int r = 0; r < 3; r++)
    {
      for (int i = 0; i < nd; i++)
      {
        Ja[i * 3 + r] = p->cJ[c][r * n + p->act[i]];
      }
    }
    wbc_matmul(l, VIEW(Ja, 3, 1), VIEW(u + p->nu0 + 3 * c, 1, 0), y, 1, 0, nd, 3, 1);
    for (int i = 0; i < nd; i++)
    {
      tau[i] = tau[i] - y[i];
    }
  }
  for (int k = 0; k < 2; k++)
  {
    for (int r = 0; r < 6; r++)
    {
      for (int i = 0; i < nd; i++)
      {
        Ja[i * 6 + r] = p->J6[k][r * n + p->act[i]];
      }
      x6[r] = stance[k] * u[6 * k + r];
    }
    wbc_matmul(l, VIEW(Ja, 6, 1), VIEW(x6, 1, 0), y, 1, 0, nd, 6, 1);
    for (int i = 0; i < nd; i++)
    {
      tau[i] = tau[i] - y[i];
    }
  }
}

/** machine.wrench.shrunk: how far from tau0 toward tau every drive stays within its limit. */
static double shrunk(int nd, const double *tau, const double *tau0, const double *lo, const double *hi)
{
  double least = 0.0;

  for (int i = 0; i < nd; i++)
  {
    const double t0 = (tau0[i] >= lo[i] && tau0[i] <= hi[i]) ? tau0[i] : 0.0;
    const double d = tau[i] - t0;
    const double leaves = (d > 0.0 ? hi[i] - t0 : lo[i] - t0) / d;
    const double enters = (d > 0.0 ? lo[i] - t0 : hi[i] - t0) / d;
    double       s = d == 0.0 ? 1.0 * (double)(tau[i] >= lo[i] && tau[i] <= hi[i]) : npmin(1.0, leaves);

    s = s >= npmax(0.0, d == 0.0 ? 0.0 : enters) ? s : 0.0;
    least = i ? npmin(least, s) : s;
  }
  return clipped(least, 0.0, 1.0);
}

/** One row block laid: `rows` of R (at its strides) weighed, its c, its c0 or none. */
static int lay(double *R0, double *rhs, double *rhs0, int at, int nu, wbc_view_t R, int rows, const double *c,
               double weight, const double *c0)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < nu; j++)
    {
      R0[(at + i) * nu + j] = weight * R.at[i * R.s0 + j * R.s1];
    }
    rhs[at + i] = weight * c[i];
    rhs0[at + i] = c0 ? weight * c0[i] : 0.0;
  }
  return at + rows;
}

int wbc_step(wbc_step_t *p)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n, nd = p->nd, nu0 = p->nu0, nc = p->nc, k3 = 3 * nc, nu = nu0 + k3;
  const double      G = p->g, mass = p->mass;
  double           *T = p->work, *E = T + 6 * n, *t0 = E + n * nu, *a6 = t0 + n, *Gm = a6 + nu, *P = Gm + 6 * nu;
  double           *full = P + 6 * nu, *full0 = full + n * nu, *ME = full0 + n, *sumf = ME + 6 * nu;
  double           *about = sumf + 3 * nu, *R0 = about + 3 * nu, *rhs = R0 + WBC_STEP_R * nu, *rhs0 = rhs + WBC_STEP_R;
  double           *H0 = rhs0 + WBC_STEP_R, *g0 = H0 + nu * nu, *kin = g0 + nu, *FM = kin + nu * nu;
  double           *S = FM + n * nu, *r = S + 3 * nu * WBC_STEP_C, *r0 = r + 3 * WBC_STEP_C, *part = r0 + 3 * WBC_STEP_C;
  double           *spare = part + nu, *swung = spare + nu, *moment = swung + nu, *whole = moment + nu;
  double           *stood = whole + nu, *u = stood + nu, *tau = u + nu, *tau0 = tau + nd, *carried = tau0 + nd;
  double           *qdd = carried + nd, *limit = qdd + n, *row = limit + nd, *rest = row + 3 * nu;
  double            stance[2], asked[2], want[WBC_STEP_D], MT[36], Ai[36], nAi[36], mt[6], p0[6], c[16], c0[16];
  double            lo[WBC_STEP_D], hi[WBC_STEP_D], nlimit[WBC_STEP_D];
  double            grounded, turned, felt[WBC_STEP_C], strides = p->strides;
  wbc_geo_t         geo;
  int               at = 0, past = 0;

  stance[0] = (double)(p->stance[0] != 0);
  stance[1] = (double)(p->stance[1] != 0);
  grounded = pymax(npmax(stance[0], stance[1]), (double)(k3 > 0));
  turned = pymax(grounded, p->borne) * (double)(p->nturns > 0);
  for (int i = 0; i < nd; i++)
  {
    want[i] = p->posture_kd[0] * (p->posture[i] - p->q[i]) - p->posture_kd[1] * p->v[p->act[i]];
  }
  if (maps(p, want, T, E, t0, a6, asked, nu, rest))
  {
    return 1;
  }
  /* the base's rows of her dynamics: A base = Gm u - M_b E u - d */
  memset(Gm, 0, (size_t)(6 * nu) * sizeof(double));
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      Gm[i * nu + j] = p->J6[0][j * n + i] * stance[0];
      Gm[i * nu + 6 + j] = p->J6[1][j * n + i] * stance[1];
    }
    for (int q = 0; q < nc; q++)
    {
      for (int j = 0; j < 3; j++)
      {
        Gm[i * nu + nu0 + 3 * q + j] = p->cJ[q][j * n + i];
      }
    }
  }
  wbc_matmul(l, VIEW(p->M, n, 1), VIEW(T, 6, 1), MT, 6, 1, 6, n, 6);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      MT[6 * i + j] = MT[6 * i + j] + 1e-9 * (double)(i == j);
    }
  }
  if (wbc_inv(l, VIEW(MT, 6, 1), 6, Ai, rest))
  {
    return 1;
  }
  wbc_matmul(l, VIEW(p->M, n, 1), VIEW(E, nu, 1), ME, nu, 1, 6, n, nu);
  for (int i = 0; i < 6 * nu; i++)
  {
    ME[i] = Gm[i] - ME[i];
  }
  wbc_matmul(l, VIEW(Ai, 6, 1), VIEW(ME, nu, 1), P, nu, 1, 6, 6, nu);
  wbc_matmul(l, VIEW(p->M, n, 1), VIEW(t0, 1, 0), mt, 1, 0, 6, n, 1);
  for (int i = 0; i < 6; i++)
  {
    mt[i] = mt[i] + p->h[i];
  }
  for (int i = 0; i < 36; i++)
  {
    nAi[i] = -Ai[i];
  }
  wbc_matmul(l, VIEW(nAi, 6, 1), VIEW(mt, 1, 0), p0, 1, 0, 6, 6, 1);
  wbc_matmul(l, VIEW(T, 6, 1), VIEW(P, nu, 1), full, nu, 1, n, 6, nu);
  for (int i = 0; i < n * nu; i++)
  {
    full[i] = full[i] + E[i];
  }
  wbc_matmul(l, VIEW(T, 6, 1), VIEW(p0, 1, 0), full0, 1, 0, n, 6, 1);
  for (int i = 0; i < n; i++)
  {
    full0[i] = full0[i] + t0[i];
  }
  /* her centre of mass, her angular momentum, each contact's point held still */
  memset(sumf, 0, (size_t)(6 * nu) * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double rr[3] = {p->sole[k][0] - p->com[0], p->sole[k][1] - p->com[1], p->sole[k][2] - p->com[2]};
    const double skew[9] = {0.0, -rr[2], rr[1], rr[2], 0.0, -rr[0], -rr[1], rr[0], 0.0};
    const double sk = stance[k];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        sumf[i * nu + 6 * k + 3 + j] = sk * (double)(i == j);
        about[i * nu + 6 * k + j] = sk * (double)(i == j);
        about[i * nu + 6 * k + 3 + j] = sk * skew[3 * i + j];
      }
    }
  }
  for (int q = 0; q < nc; q++)
  {
    const double rr[3] = {p->cp[q][0] - p->com[0], p->cp[q][1] - p->com[1], p->cp[q][2] - p->com[2]};
    const double skew[9] = {0.0, -rr[2], rr[1], rr[2], 0.0, -rr[0], -rr[1], rr[0], 0.0};
    double       left[3], jv[3];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        sumf[i * nu + nu0 + 3 * q + j] = (double)(i == j);
        about[i * nu + nu0 + 3 * q + j] = skew[3 * i + j];
      }
    }
    wbc_matmul(l, VIEW(p->cJ[q], n, 1), VIEW(full0, 1, 0), left, 1, 0, 3, n, 1);
    wbc_matmul(l, VIEW(p->cJ[q], n, 1), VIEW(full, nu, 1), S + 3 * nu * q, nu, 1, 3, n, nu);
    wbc_matmul(l, VIEW(p->cJ[q], n, 1), VIEW(p->v, 1, 0), jv, 1, 0, 3, n, 1);
    for (int j = 0; j < 3 * nu; j++)
    {
      S[3 * nu * q + j] = p->w_still * S[3 * nu * q + j];
    }
    for (int i = 0; i < 3; i++)
    {
      left[i] = (-p->cdJv[q][i]) - left[i];
      r[3 * q + i] = p->w_still * (left[i] - p->contact_k * jv[i]);
      r0[3 * q + i] = p->w_still * left[i];
    }
    felt[q] = p->cf[q][1];
  }
  /* the rows, as machine.wrench lays them */
  for (int j = 0; j < 3 * nu; j++)
  {
    row[j] = sumf[j] / mass;
  }
  c[0] = p->com_acc[0];
  c[1] = p->com_acc[2];
  at = lay(R0, rhs, rhs0, at, nu, VIEW(row, 2 * nu, 1), 2, c, p->w_com * grounded, NULL);
  c[0] = p->com_acc[1] + G;
  c0[0] = G;
  at = lay(R0, rhs, rhs0, at, nu, VIEW(row + nu, nu, 1), 1, c, p->w_height * grounded, c0);
  for (int t = 0; t < p->nturns; t++)
  {
    double left[3], te[3];

    wbc_matmul(l, VIEW(p->Jr[t], n, 1), VIEW(full0, 1, 0), left, 1, 0, 3, n, 1);
    wbc_matmul(l, VIEW(p->Jr[t], n, 1), VIEW(full, nu, 1), row, nu, 1, 3, n, nu);
    turn_error(l, p->turn[t], p->tquat[t], te);
    for (int i = 0; i < 3; i++)
    {
      left[i] = (-p->dJr[t][i]) - left[i];
      c[i] = (p->turn_kd[0] * te[i] - p->turn_kd[1] * p->tw[t][i]) + left[i];
    }
    at = lay(R0, rhs, rhs0, at, nu, VIEW(row, nu, 1), 3, c, p->w_turn * turned, left);
  }
  for (int i = 0; i < p->nupper; i++)
  {
    /* her arms' rows weigh as her text stiffens them over her rest */
    const double w = p->w_posture * (1.0 + (p->hold - 1.0) * p->arms[i]);

    for (int j = 0; j < nu; j++)
    {
      R0[(at + i) * nu + j] = w * (double)(j == 12 + i);
    }
    rhs[at + i] = w * want[p->upper[i]];
    rhs0[at + i] = 0.0;
  }
  at += p->nupper;
  for (int i = 0; i < nu; i++)
  {
    swung[i] = i < 12 ? asked[i / 6] : 0.0;
  }
  /* a sole that does not stand while something else bears her rests: its rows weigh as a pressing
     contact's point */
  for (int i = 0; i < nu; i++)
  {
    const double resting = i < 12 ? p->rests[i / 6] : 0.0;
    const double w = p->w_swing + (p->w_still - p->w_swing) * resting;

    for (int j = 0; j < nu; j++)
    {
      R0[(at + i) * nu + j] = w * (i == j ? swung[i] : 0.0);
    }
    rhs[at + i] = w * (i < nu0 ? a6[i] : 0.0);
    rhs0[at + i] = 0.0;
  }
  at += nu;
  {
    const int    knee[2] = {p->act[p->knees[0]], p->act[p->knees[1]]};
    const double f0k[2] = {full0[knee[0]], full0[knee[1]]};
    const double wk[2] = {want[p->knees[0]] + p->posture_kd[1] * p->knee_rate[0],
                          want[p->knees[1]] + p->posture_kd[1] * p->knee_rate[1]};
    double       left, *fk = row + nu;

    left = -wbc_dot(l, stance, 1, f0k, 1, 2);
    memcpy(fk, full + knee[0] * nu, (size_t)nu * sizeof(double));
    memcpy(fk + nu, full + knee[1] * nu, (size_t)nu * sizeof(double));
    wbc_matmul(l, VIEW(stance, 0, 1), VIEW(fk, nu, 1), row, 0, 1, 1, 2, nu);
    c[0] = wbc_dot(l, stance, 1, wk, 1, 2) + left;
    c0[0] = left;
    at = lay(R0, rhs, rhs0, at, nu, VIEW(row, nu, 1), 1, c, p->w_knee * grounded, c0);
    for (int k = 0; k < 2; k++)
    {
      const int j = knee[k];

      if (!p->fold[k])
      {
        continue;
      }
      c[0] = ((p->fold_acc[k] + p->swing_kd[0] * (p->fold_knee[k] - p->q[p->knees[k]]))
              + p->swing_kd[1] * (p->fold_rate[k] - p->v[j])) - full0[j];
      c0[0] = -full0[j];
      at = lay(R0, rhs, rhs0, at, nu, VIEW(full + j * nu, nu, 1), 1, c, p->w_fold, c0);
    }
  }
  for (int b = 0; b < p->nbuffers; b++)
  {
    const int    j = p->act[p->buffer_at[b]];
    const double depth = p->buffer_side[b] * (p->buffer_stop[b] - p->q[p->buffer_at[b]]) + p->buffer[0];

    c[0] = ((p->buffer[1] * p->buffer_side[b]) * pymax(depth, 0.0) - p->buffer[2] * p->v[j]) - full0[j];
    c0[0] = -full0[j];
    at = lay(R0, rhs, rhs0, at, nu, VIEW(full + j * nu, nu, 1), 1, c,
             p->w_stop * pymin(pymax((depth + p->buffer[3] * pymax(-p->buffer_side[b] * p->v[j], 0.0)) / p->buffer[0],
                                     0.0), 1.0), c0);
  }
  for (int i = 0; i < 3; i++)
  {
    c[i] = p->kdot_given ? p->kdot[i] : (-p->momentum_k) * p->k[i];
  }
  at = lay(R0, rhs, rhs0, at, nu, VIEW(about, nu, 1), 3, c, p->w_moment * grounded, NULL);
  {
    const double inv_mg = 1.0 / (mass * G), inv_mgr = 1.0 / ((mass * G) * p->room[0]);

    for (int j = 0; j < nu; j++)
    {
      const double each = j < 12 ? stance[j / 6] : 0.0, sel = j < 12 ? each : (j < nu0 ? 0.0 : 1.0);

      part[j] = j < 12 ? ((j % 6) < 3 ? inv_mgr : inv_mg) : (j < nu0 ? 0.0 : inv_mg);
      spare[j] = (p->w_part * sel) * part[j] + p->w_air * (j < 12 ? 1.0 - each : 0.0);
      moment[j] = j < nu0 ? p->moment[j] : 0.0;
    }
  }
  for (int i = 0; i < nu; i++)
  {
    for (int j = 0; j < nu; j++)
    {
      R0[(at + i) * nu + j] = 1.0 * (i == j ? spare[i] : 0.0);
    }
    rhs[at + i] = 1.0 * 0.0;
    rhs0[at + i] = 0.0;
  }
  at += nu;
  wbc_matmul(l, VIEW(R0, 1, nu), VIEW(R0, nu, 1), H0, nu, 1, nu, at, nu);
  for (int i = 0; i < nu; i++)
  {
    for (int j = 0; j < nu; j++)
    {
      H0[i * nu + j] = H0[i * nu + j] + 1e-12 * (double)(i == j);
    }
  }
  if (strides > 0.0)
  {
    wbc_matmul(l, VIEW(full, 1, nu), VIEW(p->M, n, 1), FM, n, 1, nu, n, n);
    wbc_matmul(l, VIEW(FM, n, 1), VIEW(full, nu, 1), kin, nu, 1, nu, n, nu);
    for (int j = 0; j < nu; j++)
    {
      const double keep = (p->w_keep * (j < 12 ? stance[j / 6] : 0.0)) * part[j];
      const double sw = p->w_swing * swung[j] + spare[j];

      kin[j * nu + j] = kin[j * nu + j] + (sw * sw + keep * keep);
    }
    for (int i = 0; i < nu; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        kin[i * nu + j] = i == j ? kin[i * nu + j] : kin[i * nu + j] + 0.0;
      }
    }
  }
  geo.nu = nu;
  geo.k3 = k3;
  geo.moment = moment;
  geo.part = part;
  geo.up = row;
  geo.lift = sumf + nu;
  geo.stance = stance;
  geo.kinetic = kin;
  geo.frames[0] = p->R[0];
  geo.frames[1] = p->R[1];
  geo.mass = mass;
  geo.grounded = grounded;
  geo.strides = strides;
  geo.edges[0] = p->edge[0];
  geo.edges[1] = p->edge[1];
  geo.mu[0] = p->mus[0];
  geo.mu[1] = p->mus[1];
  for (int j = 0; j < nu; j++)
  {
    row[j] = sumf[nu + j] / mass;
  }
  /* the ask's answer, and nothing's: every acceleration 0, her weight borne, by the same rows */
  wbc_matmul(l, VIEW(R0, 1, nu), VIEW(rhs, 1, 0), g0, 1, 0, nu, at, 1);
  if (wbc_rounds(p, &geo, H0, g0, S, r, felt, p->com_acc[1] - p->height_band, p->com_acc[1] + p->height_band,
                 whole, rest))
  {
    return 1;
  }
  for (int i = 0; i < nd; i++)
  {
    limit[i] = (p->wep ? p->wep_nm[i] : p->clamp[i]) * p->derate[i];
    nlimit[i] = -limit[i];
    lo[i] = npmax(-limit[i], p->lo[i]);
    hi[i] = npmin(limit[i], p->hi[i]);
  }
  torques(p, full, full0, stance, whole, 1.0, nu, tau, qdd, rest);
  for (int i = 0; i < nd; i++)
  {
    past |= fabs(tau[i]) > limit[i];
  }
  if (past)
  {
    int    past0 = 0;
    double s0 = 1.0;

    wbc_matmul(l, VIEW(R0, 1, nu), VIEW(rhs0, 1, 0), g0, 1, 0, nu, at, 1);
    if (wbc_rounds(p, &geo, H0, g0, S, r0, felt, -p->height_band, p->height_band, stood, rest))
    {
      return 1;
    }
    torques(p, full, full0, stance, stood, 1.0, nu, tau0, qdd, rest);
    p->scale = shrunk(nd, tau, tau0, nlimit, limit);
    for (int j = 0; j < nu; j++)
    {
      u[j] = whole[j] + (p->scale - 1.0) * (whole[j] - stood[j]);
    }
    for (int i = 0; i < nd; i++)
    {
      past0 |= fabs(tau0[i]) > limit[i];
    }
    if (past0)
    {
      double t1[WBC_STEP_D], z0[WBC_STEP_D];

      for (int i = 0; i < nd; i++)
      {
        t1[i] = tau[i] + (p->scale - 1.0) * (tau[i] - tau0[i]);
        z0[i] = 0.0;
      }
      s0 = shrunk(nd, t1, z0, nlimit, limit);
      p->scale = s0 * p->scale;
      if (s0 < 1.0)
      {
        for (int j = 0; j < nu; j++)
        {
          u[j] = s0 * u[j];
        }
      }
    }
    torques(p, full, full0, stance, u, s0, nu, carried, qdd, rest);
  }
  else
  {
    p->scale = 1.0;
    memcpy(u, whole, (size_t)nu * sizeof(double));
    memcpy(carried, tau, (size_t)nd * sizeof(double));
  }
  p->cop_n = 0;
  for (int k = 0; k < 2; k++)
  {
    double wk[6], ml[3];

    for (int j = 0; j < 6; j++)
    {
      wk[j] = stance[k] * u[6 * k + j];
    }
    p->bears[k] = wk[4];
    if (p->stance[k])
    {
      const int at_ = p->cop_n++;

      wbc_matmul(l, VIEW(p->R[k], 1, 3), VIEW(wk, 1, 0), ml, 1, 0, 3, 3, 1);
      p->cop_none[at_] = !(wk[4] > 1e-6);
      p->cop[at_][0] = ml[2] / wk[4];
      p->cop[at_][1] = (-ml[0]) / wk[4];
    }
  }
  for (int i = 0; i < nd; i++)
  {
    p->over[i] = fabs(tau[i]) > p->clamp[i] * (1.0 + 1e-6);
    p->load[i] = fabs(tau[i]) / p->clamp[i];
    p->tau[i] = clipped(carried[i], lo[i], hi[i]);
  }
  for (int q = 0; q < p->npairs; q++)
  {
    const int a = p->pairs[q][0], b = p->pairs[q][1], either = p->over[a] || p->over[b];

    p->over[a] = p->over[b] = either;
  }
  memcpy(p->qacc, qdd, (size_t)n * sizeof(double));
  for (int q = 0; q < nc; q++)
  {
    memcpy(p->forces[q], u + nu0 + 3 * q, 3 * sizeof(double));
  }
  return 0;
}
