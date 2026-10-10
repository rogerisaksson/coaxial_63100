/** wbc_stack.c - the loop: the asks of R^k into the drives' torques, by priority, clipped.

    Unknown the commanded torques, the drives' and the held joints'. The soles standing still
    and the dynamics make every acceleration and each sole's wrench affine in them (M's
    Cholesky, the contacts' Schur complement). Each level's rows are met least squares in the
    null space of the levels above: a small level by its Gram matrix, damped by DAMP of its
    largest diagonal (the redundancy under it cut), the last and longest by its normal
    equations damped by REG. The polytope: each drive's load within its clamp as derated, its
    peak under war emergency power where it ran near its clamp; the lowest level scaled back
    first; a pair's rods shared as machine.wbc._currents has it. machine.wbc's levels, its
    inequalities as tasks and clips (docs/findings/wbc.md). */
#include "wbc.h"

#include <math.h>
#include <string.h>

#define WBC_G 9.81

static const double HELD_K = 50.0, CONTACT_K = 20.0;
static const double TURN_KP = 100.0, TURN_KD = 20.0, POSTURE_KP = 100.0, POSTURE_KD = 20.0;
static const double SWING_KP = 1600.0, SWING_KD = 80.0, MOMENTUM_K = 3.0;
static const double TURN_W = 3.0, MOMENTUM_W = 0.3, POSTURE_W = 1.0, TORQUE_W = 3.0;
static const double FORCE_W = 1.0, HEIGHT_W = 1.0, FOLD_W = 0.3;
static const double CLAMP_SHARE = 0.95, NEAR = 0.6;
static const double DAMP = 1e-6, REG = 1e-6;

/** The dof a commanded torque x drives. */
static int dof_of(int x)
{
  return 5 + ((x < WBC_DRIVEN) ? wbc_driven[x] : wbc_held[x - WBC_DRIVEN]);
}

/** a, n x n by rows of `ld`, into its lower Cholesky factor in place; 0 where not positive. */
static int cholesky(int n, int ld, double *a)
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
static void chol_solve(int n, int ld, const double *l, const double *b, double *x)
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
static void quat_of(const double r[9], double q[4])
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
static void turn_error(const double want[4], const double have[4], double out[3])
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

/** A row over tau from one over udot: coef . (u0 + bb tau) = rhs, weighted. */
static void row_udot(wbc_stack_t *s, const double coef[WBC_N], double rhs, double weight)
{
  double *a = s->a[s->rows];
  double  off = 0.0;

  for (int x = 0; x < WBC_X; x++)
  {
    a[x] = 0.0;
  }
  for (int n = 0; n < WBC_N; n++)
  {
    if (coef[n] != 0.0)
    {
      for (int x = 0; x < WBC_X; x++)
      {
        a[x] += coef[n] * s->bb[n][x];
      }
      off += coef[n] * s->u0[n];
    }
  }
  for (int x = 0; x < WBC_X; x++)
  {
    a[x] *= weight;
  }
  s->r[s->rows] = weight * (rhs - off);
  s->rows++;
}

/** A row over tau from one over the soles' wrenches: coef . (l0 + ll tau) = rhs, weighted. */
static void row_lambda(wbc_stack_t *s, const double coef[WBC_C], double rhs, double weight)
{
  double *a = s->a[s->rows];
  double  off = 0.0;

  for (int x = 0; x < WBC_X; x++)
  {
    a[x] = 0.0;
  }
  for (int i = 0; i < s->nc; i++)
  {
    if (coef[i] != 0.0)
    {
      for (int x = 0; x < WBC_X; x++)
      {
        a[x] += coef[i] * s->ll[i][x];
      }
      off += coef[i] * s->l0[i];
    }
  }
  for (int x = 0; x < WBC_X; x++)
  {
    a[x] *= weight;
  }
  s->r[s->rows] = weight * (rhs - off);
  s->rows++;
}

/** A row on one torque: weight tau_x = weight rhs. */
static void row_tau(wbc_stack_t *s, int x, double rhs, double weight)
{
  for (int k = 0; k < WBC_X; k++)
  {
    s->a[s->rows][k] = 0.0;
  }
  s->a[s->rows][x] = weight;
  s->r[s->rows] = weight * rhs;
  s->rows++;
}

/** The rows laid, least squares in the null space so far: this level's torques, and the null
    space under them for the next. The last level by its damped normal equations. */
static void level_solve(wbc_stack_t *s, int level, int last)
{
  const int rows = s->rows;
  double    sum[WBC_X], z[WBC_ROWS], rhs[WBC_X], y[WBC_X], big;

  for (int x = 0; x < WBC_X; x++)
  {
    sum[x] = 0.0;
    for (int k = 0; k < level; k++)
    {
      sum[x] += s->tau[k][x];
    }
    s->tau[level][x] = 0.0;
  }
  if (rows == 0)
  {
    return;
  }
  for (int i = 0; i < rows; i++)
  {
    double got = 0.0;

    for (int x = 0; x < WBC_X; x++)
    {
      double v = 0.0;

      for (int k = 0; k < WBC_X; k++)
      {
        v += s->a[i][k] * s->nn[k][x];
      }
      s->ab[i][x] = v;
      got += s->a[i][x] * sum[x];
    }
    s->rho[i] = s->r[i] - got;
  }
  if ((rows <= WBC_SMALL) && !last)
  {
    big = 0.0;
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < rows; j++)
      {
        double v = 0.0;

        for (int x = 0; x < WBC_X; x++)
        {
          v += s->ab[i][x] * s->ab[j][x];
        }
        s->g[i][j] = v;
      }
      big = (s->g[i][i] > big) ? s->g[i][i] : big;
    }
    for (int i = 0; i < rows; i++)
    {
      s->g[i][i] += DAMP * big + 1e-300;
    }
    if (!cholesky(rows, WBC_SMALL, &s->g[0][0]))
    {
      return;
    }
    chol_solve(rows, WBC_SMALL, &s->g[0][0], s->rho, z);
    for (int x = 0; x < WBC_X; x++)
    {
      double v = 0.0;

      for (int i = 0; i < rows; i++)
      {
        v += s->ab[i][x] * z[i];
      }
      s->tau[level][x] = v;
    }
    /* The null space less this level's range: nn -= ab^T G^-1 ab. */
    for (int x = 0; x < WBC_X; x++)
    {
      double col[WBC_ROWS] = {0.0}, sol[WBC_ROWS] = {0.0};

      for (int i = 0; i < rows; i++)
      {
        col[i] = s->ab[i][x];
      }
      chol_solve(rows, WBC_SMALL, &s->g[0][0], col, sol);
      for (int i = 0; i < rows; i++)
      {
        s->qq[i][x] = sol[i];
      }
    }
    for (int x = 0; x < WBC_X; x++)
    {
      for (int k = 0; k < WBC_X; k++)
      {
        double v = 0.0;

        for (int i = 0; i < rows; i++)
        {
          v += s->ab[i][x] * s->qq[i][k];
        }
        s->nn[x][k] -= v;
      }
    }
    return;
  }
  big = 0.0;
  for (int x = 0; x < WBC_X; x++)
  {
    for (int k = 0; k < WBC_X; k++)
    {
      double v = 0.0;

      for (int i = 0; i < rows; i++)
      {
        v += s->ab[i][x] * s->ab[i][k];
      }
      s->hh[x][k] = v;
    }
    big = (s->hh[x][x] > big) ? s->hh[x][x] : big;
    rhs[x] = 0.0;
    for (int i = 0; i < rows; i++)
    {
      rhs[x] += s->ab[i][x] * s->rho[i];
    }
  }
  for (int x = 0; x < WBC_X; x++)
  {
    s->hh[x][x] += REG * big + 1e-300;
  }
  if (cholesky(WBC_X, WBC_X, &s->hh[0][0]))
  {
    chol_solve(WBC_X, WBC_X, &s->hh[0][0], rhs, y);
    memcpy(s->tau[level], y, sizeof(y));
  }
}

/** Each drive's load from the drives' torques: a pair's as its rods share them. */
static void loads(const double tau[WBC_X], double load[WBC_DRIVEN])
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
      int other = -1;

      for (int k = 0; k < WBC_DRIVEN; k++)
      {
        if (wbc_driven[k] == pair - 1)
        {
          other = k;
        }
      }
      if (other >= 0)
      {
        const double kj = wbc_kt[wbc_driven[j]], ko = wbc_kt[wbc_driven[other]];
        const double a = tau[j] / kj, c = tau[other] / ko;

        load[j] = (a + c) * kj;
        load[other] = (a - c) * ko;
      }
    }
  }
}

/** The drives' torques from their loads: the inverse of `loads`. */
static void unload(const double load[WBC_DRIVEN], double tau[WBC_X])
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
      int other = -1;

      for (int k = 0; k < WBC_DRIVEN; k++)
      {
        if (wbc_driven[k] == pair - 1)
        {
          other = k;
        }
      }
      if (other >= 0)
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

void wbc_stack_init(wbc_stack_t *s)
{
  memset(s, 0, sizeof(*s));
}

void wbc_stack_step(wbc_body_t *b, wbc_stack_t *s, const wbc_frame_t *base,
                    const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                    wbc_out_t *out)
{
  const double origin[3] = {0.0, 0.0, 0.0};
  double       e[WBC_N], col[WBC_N], coef[WBC_N], lc[WBC_C], tmp[6], have[4], err[3];
  double       ceiling[WBC_DRIVEN], nominal[WBC_DRIVEN], total[WBC_X], before[WBC_DRIVEN];
  double       load[WBC_DRIVEN], sole[2][3];
  int          standing[2], nst = 0;

  wbc_body_pose(b, base, q);
  wbc_body_mass(b);
  wbc_body_bias(b, u);
  wbc_body_com(b, s->com, s->jcom);

  /* M = L L^T; w = M^-1 S^T, w0 = -M^-1 h. */
  memcpy(s->chol, b->m, sizeof(s->chol));
  if (!cholesky(WBC_N, WBC_N, &s->chol[0][0]))
  {
    memset(out, 0, sizeof(*out));
    return;
  }
  for (int x = 0; x < WBC_X; x++)
  {
    memset(e, 0, sizeof(e));
    e[dof_of(x)] = 1.0;
    chol_solve(WBC_N, WBC_N, &s->chol[0][0], e, col);
    for (int n = 0; n < WBC_N; n++)
    {
      s->w[n][x] = col[n];
    }
  }
  for (int n = 0; n < WBC_N; n++)
  {
    e[n] = -b->h[n];
  }
  chol_solve(WBC_N, WBC_N, &s->chol[0][0], e, s->w0);

  /* The standing soles: Jc udot = c, their twists damped to rest. */
  for (int k = 0; k < 2; k++)
  {
    standing[k] = ask->stance[k] ? nst : -1;
    wbc_body_point(b, wbc_sole[k], wbc_sole_at, sole[k]);
    if (ask->stance[k])
    {
      double drift[6];

      wbc_body_jacobian(b, wbc_sole[k], wbc_sole_at, s->jac);
      wbc_body_drift(b, wbc_sole[k], wbc_sole_at, drift);
      for (int i = 0; i < 6; i++)
      {
        double twist = 0.0;

        memcpy(s->jc[6 * nst + i], s->jac[i], sizeof(s->jac[i]));
        for (int n = 0; n < WBC_N; n++)
        {
          twist += s->jac[i][n] * u[n];
        }
        s->c[6 * nst + i] = -drift[i] - CONTACT_K * twist;
      }
      nst++;
    }
  }
  s->nc = 6 * nst;
  if (s->nc)
  {
    double gram[WBC_C][WBC_C], rhs[WBC_C], sol[WBC_C];

    for (int i = 0; i < s->nc; i++)
    {
      chol_solve(WBC_N, WBC_N, &s->chol[0][0], s->jc[i], col);
      for (int n = 0; n < WBC_N; n++)
      {
        s->y[n][i] = col[n];
      }
    }
    for (int i = 0; i < s->nc; i++)
    {
      for (int j = 0; j < s->nc; j++)
      {
        double v = 0.0;

        for (int n = 0; n < WBC_N; n++)
        {
          v += s->jc[i][n] * s->y[n][j];
        }
        gram[i][j] = v;
      }
    }
    for (int i = 0; i < s->nc; i++)
    {
      gram[i][i] += 1e-12;
    }
    if (!cholesky(s->nc, WBC_C, &gram[0][0]))
    {
      memset(out, 0, sizeof(*out));
      return;
    }
    /* lam = gram^-1; ll = -lam Jc w; l0 = lam (c - Jc w0). */
    for (int j = 0; j < s->nc; j++)
    {
      memset(rhs, 0, sizeof(rhs));
      rhs[j] = 1.0;
      chol_solve(s->nc, WBC_C, &gram[0][0], rhs, sol);
      for (int i = 0; i < s->nc; i++)
      {
        s->lam[i][j] = sol[i];
      }
    }
    for (int x = 0; x < WBC_X; x++)
    {
      for (int i = 0; i < s->nc; i++)
      {
        double v = 0.0;

        for (int n = 0; n < WBC_N; n++)
        {
          v += s->jc[i][n] * s->w[n][x];
        }
        rhs[i] = v;
      }
      for (int i = 0; i < s->nc; i++)
      {
        double v = 0.0;

        for (int j = 0; j < s->nc; j++)
        {
          v -= s->lam[i][j] * rhs[j];
        }
        s->ll[i][x] = v;
      }
    }
    for (int i = 0; i < s->nc; i++)
    {
      double v = s->c[i];

      for (int n = 0; n < WBC_N; n++)
      {
        v -= s->jc[i][n] * s->w0[n];
      }
      rhs[i] = v;
    }
    for (int i = 0; i < s->nc; i++)
    {
      double v = 0.0;

      for (int j = 0; j < s->nc; j++)
      {
        v += s->lam[i][j] * rhs[j];
      }
      s->l0[i] = v;
    }
    for (int n = 0; n < WBC_N; n++)
    {
      for (int x = 0; x < WBC_X; x++)
      {
        double v = s->w[n][x];

        for (int i = 0; i < s->nc; i++)
        {
          v += s->y[n][i] * s->ll[i][x];
        }
        s->bb[n][x] = v;
      }
      s->u0[n] = s->w0[n];
      for (int i = 0; i < s->nc; i++)
      {
        s->u0[n] += s->y[n][i] * s->l0[i];
      }
    }
  }
  else
  {
    memcpy(s->bb, s->w, sizeof(s->bb));
    memcpy(s->u0, s->w0, sizeof(s->u0));
  }

  /* The null space whole, and the levels. */
  memset(s->nn, 0, sizeof(s->nn));
  for (int x = 0; x < WBC_X; x++)
  {
    s->nn[x][x] = 1.0;
  }

  /* Level 0: the held joints brought to rest. */
  s->rows = 0;
  for (int k = 0; k < WBC_HELD; k++)
  {
    const int d = 5 + wbc_held[k];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    row_udot(s, coef, -HELD_K * u[d], 1.0);
  }
  level_solve(s, 0, 0);

  /* Level 1: the centre of mass along the floor, by Newton on the soles' forces. */
  s->rows = 0;
  if (s->nc)
  {
    for (int axis = 0; axis < 3; axis += 2)
    {
      memset(lc, 0, sizeof(lc));
      for (int k = 0; k < nst; k++)
      {
        lc[6 * k + 3 + axis] = 1.0 / wbc_total_mass;
      }
      row_lambda(s, lc, ask->com_acc[axis], 1.0);
    }
  }
  level_solve(s, 1, 0);

  /* Level 2: the pelvis's and the trunk's turns, a swinging sole and its knee's fold. */
  s->rows = 0;
  for (int t = 0; t < 2; t++)
  {
    const int link = t ? wbc_trunk : 0;
    double    drift[6], wdot[3];

    wbc_body_jacobian(b, link, origin, s->jac);
    wbc_body_drift(b, link, origin, drift);
    quat_of(b->t[link].r, have);
    turn_error(ask->turn[t], have, err);
    for (int i = 0; i < 3; i++)
    {
      double w = 0.0;

      for (int n = 0; n < WBC_N; n++)
      {
        w += s->jac[i][n] * u[n];
      }
      wdot[i] = TURN_KP * err[i] - TURN_KD * w;
    }
    for (int i = 0; i < 3; i++)
    {
      row_udot(s, s->jac[i], wdot[i] - drift[i], TURN_W);
    }
  }
  for (int k = 0; k < 2; k++)
  {
    if (ask->swing[k])
    {
      double drift[6], twist[6], a6[6];

      wbc_body_jacobian(b, wbc_sole[k], wbc_sole_at, s->jac);
      wbc_body_drift(b, wbc_sole[k], wbc_sole_at, drift);
      for (int i = 0; i < 6; i++)
      {
        twist[i] = 0.0;
        for (int n = 0; n < WBC_N; n++)
        {
          twist[i] += s->jac[i][n] * u[n];
        }
      }
      quat_of(b->t[wbc_sole[k]].r, have);
      turn_error(ask->swing_quat[k], have, err);
      for (int i = 0; i < 3; i++)
      {
        a6[i] = TURN_KP * err[i] - TURN_KD * twist[i];
        a6[3 + i] = ask->swing_acc[k][i] + SWING_KP * (ask->swing_at[k][i] - sole[k][i])
                    + SWING_KD * (ask->swing_speed[k][i] - twist[3 + i]);
      }
      for (int i = 0; i < 6; i++)
      {
        row_udot(s, s->jac[i], a6[i] - drift[i], 1.0);
      }
    }
    if (ask->fold[k])
    {
      const int d = 5 + wbc_knee[k];

      memset(coef, 0, sizeof(coef));
      coef[d] = 1.0;
      row_udot(s, coef, ask->fold_acc[k] + SWING_KP * (ask->fold_knee[k] - b->q[d - 6])
               + SWING_KD * (ask->fold_rate[k] - u[d]), FOLD_W);
    }
  }
  level_solve(s, 2, 0);

  /* Level 3, her form: her height, the angular momentum, the posture, the torques, the forces. */
  s->rows = 0;
  if (s->nc)
  {
    double k3[3], kdot[3];

    memset(lc, 0, sizeof(lc));
    for (int k = 0; k < nst; k++)
    {
      lc[6 * k + 4] = 1.0 / wbc_total_mass;
    }
    row_lambda(s, lc, ask->com_acc[1] + WBC_G, HEIGHT_W);
    wbc_body_momentum(b, s->com, k3);
    for (int i = 0; i < 3; i++)
    {
      kdot[i] = ask->kdot_given ? ask->kdot[i] : -MOMENTUM_K * k3[i];
    }
    /* k-dot = sum of each sole's moment and (sole - com) x its force. */
    for (int i = 0; i < 3; i++)
    {
      memset(lc, 0, sizeof(lc));
      for (int k = 0; k < 2; k++)
      {
        if (standing[k] >= 0)
        {
          const double d[3] = {sole[k][0] - s->com[0], sole[k][1] - s->com[1],
                               sole[k][2] - s->com[2]};
          const int    at = 6 * standing[k];

          lc[at + i] = 1.0;
          /* (d x f)_i */
          lc[at + 3 + (i + 1) % 3] += -d[(i + 2) % 3];
          lc[at + 3 + (i + 2) % 3] += d[(i + 1) % 3];
        }
      }
      row_lambda(s, lc, kdot[i], MOMENTUM_W);
    }
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int d = 5 + wbc_driven[j];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    row_udot(s, coef, POSTURE_KP * (ask->posture[j] - b->q[d - 6]) - POSTURE_KD * u[d],
             POSTURE_W);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    row_tau(s, j, 0.0, TORQUE_W / (CLAMP_SHARE * wbc_clamp[wbc_driven[j]]));
  }
  for (int i = 0; i < s->nc; i++)
  {
    const double fs = wbc_total_mass * WBC_G;
    const double scale = (i % 6 < 3) ? fs * fabs(wbc_sole_corner[0][0]) : fs;

    memset(lc, 0, sizeof(lc));
    lc[i] = 1.0 / scale;
    row_lambda(s, lc, 0.0, FORCE_W);
  }
  level_solve(s, 3, 1);

  /* The polytope: each drive's load within its ceiling, the lowest level scaled back first. */
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    nominal[j] = wbc_clamp[wbc_driven[j]] * ask->derate[j];
    ceiling[j] = (ask->wep && (s->load[j] > NEAR)) ? wbc_peak[wbc_driven[j]] : nominal[j];
    if (ceiling[j] < nominal[j])
    {
      ceiling[j] = nominal[j];
    }
  }
  for (int x = 0; x < WBC_X; x++)
  {
    total[x] = 0.0;
    for (int k = 0; k < WBC_LEVELS; k++)
    {
      total[x] += s->tau[k][x];
    }
  }
  loads(total, before);
  for (int k = 0; k < WBC_LEVELS; k++)
  {
    out->alpha[k] = 1.0;
  }
  for (int level = WBC_LEVELS - 1; level > 0; level--)
  {
    double rest[WBC_X], base_load[WBC_DRIVEN], delta[WBC_DRIVEN], alpha = 1.0;

    for (int x = 0; x < WBC_X; x++)
    {
      rest[x] = 0.0;
      for (int k = 0; k < WBC_LEVELS; k++)
      {
        if (k != level)
        {
          rest[x] += out->alpha[k] * s->tau[k][x];
        }
      }
    }
    loads(rest, base_load);
    loads(s->tau[level], delta);
    for (int j = 0; j < WBC_DRIVEN; j++)
    {
      if (delta[j] != 0.0)
      {
        const double room = (delta[j] > 0.0) ? ceiling[j] - base_load[j]
                                             : -ceiling[j] - base_load[j];
        double       a = room / delta[j];

        if (a < 0.0)
        {
          a = 0.0;
        }
        if (a < alpha)
        {
          alpha = a;
        }
      }
    }
    out->alpha[level] = alpha;
  }
  for (int x = 0; x < WBC_X; x++)
  {
    total[x] = 0.0;
    for (int k = 0; k < WBC_LEVELS; k++)
    {
      total[x] += out->alpha[k] * s->tau[k][x];
    }
  }
  loads(total, load);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    if (load[j] > ceiling[j])
    {
      load[j] = ceiling[j];
    }
    if (load[j] < -ceiling[j])
    {
      load[j] = -ceiling[j];
    }
  }
  unload(load, total);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double gain = s->bb[dof_of(j)][j];

    out->tau[j] = total[j];
    out->over[j] = fabs(before[j]) > nominal[j] * (1.0 + 1e-6);
    out->load[j] = fabs(load[j]) / nominal[j];
    s->load[j] = out->load[j];
    out->jeff[j] = (gain > 1e-9) ? 1.0 / gain : 0.0;
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    if (pair != 0)
    {
      for (int k = 0; k < WBC_DRIVEN; k++)
      {
        if (wbc_driven[k] == ((pair > 0) ? pair : -pair) - 1)
        {
          out->over[j] = out->over[j] || out->over[k];
        }
      }
    }
  }

  /* What the clipped torques give: every acceleration, each sole's wrench. */
  for (int n = 0; n < WBC_N; n++)
  {
    out->udot[n] = s->u0[n];
    for (int x = 0; x < WBC_X; x++)
    {
      out->udot[n] += s->bb[n][x] * total[x];
    }
  }
  for (int k = 0; k < 2; k++)
  {
    out->bears[k] = 0.0;
    memset(out->wrench[k], 0, sizeof(out->wrench[k]));
    if (standing[k] >= 0)
    {
      for (int i = 0; i < 6; i++)
      {
        double v = s->l0[6 * standing[k] + i];

        for (int x = 0; x < WBC_X; x++)
        {
          v += s->ll[6 * standing[k] + i][x] * total[x];
        }
        out->wrench[k][i] = v;
      }
      out->bears[k] = out->wrench[k][4];
    }
  }
  (void)tmp;
}
