/** wbc_stack.c - the loop: the asks of R^k into the drives' torques, by priority, within bounds.

    Unknown the commanded torques, the drives' and the held joints', and each standing sole's
    six internal forces: how its corners share the wrench the dynamics ask of it. The soles
    standing still and the dynamics make every acceleration and each sole's wrench affine in the
    torques (M's Cholesky, the contacts' Schur complement); the corners' forces are the wrench's
    least-norm split plus the internal forces. Each level's rows are met least squares in the
    null space of the levels above, under every bound - each drive's load within its clamp as
    derated (its peak under war emergency power where it ran near its clamp), her vertical
    acceleration within its band, each corner pressing inside its friction pyramid, each sole's
    centre of pressure inside its margin, a joint short of its stop - by an active set
    (wbc_solve.c). A pair's rods share loads as machine.wbc._currents has it. machine.wbc's
    levels, corners and bounds, deterministic (docs/findings/wbc.md). */
#include "wbc.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

#define WBC_G 9.81

static const double HELD_K = 50.0, CONTACT_K = 20.0;
static const double TURN_KP = 100.0, TURN_KD = 20.0, POSTURE_KP = 100.0, POSTURE_KD = 20.0;
static const double SWING_KP = 1600.0, SWING_KD = 80.0, MOMENTUM_K = 3.0;
static const double TURN_W = 3.0, MOMENTUM_W = 0.3, POSTURE_W = 1.0, TORQUE_W = 3.0;
static const double FORCE_W = 1.0, HEIGHT_W = 1.0, FOLD_W = 0.3;
static const double CLAMP_SHARE = 0.95, NEAR = 0.6;
static const double MU = 0.6, MARGIN_M = 0.015, LIMIT_S = 0.05, STOP_RAD = 0.01, HEIGHT_BAND = 3.0;
/** A bound within ACTIVE of its edge is reported at it. */
static const double ACTIVE = 1e-3;
/** The level a swinging sole and its fold are laid on: beside the turns, as the python stack. */
#define SWING_LEVEL 2

/** The dof a commanded torque x drives. */
static int dof_of(int x)
{
  return 5 + ((x < WBC_DRIVEN) ? wbc_driven[x] : wbc_held[x - WBC_DRIVEN]);
}

/* ---- rows over the variables ----------------------------------------------------------- */

/** A row over the variables from one over udot: coef . (u0 + bb tau) = rhs, weighted. */
static void over_udot(const wbc_stack_t *s, double *a, double *r, const double coef[WBC_N],
                      double rhs, double weight)
{
  double off = 0.0;

  memset(a, 0, WBC_V * sizeof(double));
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
  *r = weight * (rhs - off);
}

/** A row over the variables from one over the soles' wrenches: coef . (l0 + ll tau) = rhs. */
static void over_lambda(const wbc_stack_t *s, double *a, double *r, const double coef[WBC_C],
                        double rhs, double weight)
{
  double off = 0.0;

  memset(a, 0, WBC_V * sizeof(double));
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
  *r = weight * (rhs - off);
}

/** A row over the variables from one over sole k's twelve corner force components, world:
    coef . (wp (l0 + ll tau) + bw n) = rhs, weighted. */
static void over_corner(const wbc_stack_t *s, double *a, double *r, int k,
                        const double coef[12], double rhs, double weight)
{
  const int at = 6 * s->standing[k];
  double    lc[WBC_C], off;

  memset(lc, 0, sizeof(lc));
  for (int i = 0; i < 12; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      lc[at + j] += coef[i] * s->wp[k][i][j];
    }
  }
  over_lambda(s, a, &off, lc, rhs, 1.0);
  for (int j = 0; j < 6; j++)
  {
    double v = 0.0;

    for (int i = 0; i < 12; i++)
    {
      v += coef[i] * s->bw[k][i][j];
    }
    a[WBC_X + 6 * k + j] = v;
  }
  for (int v = 0; v < WBC_V; v++)
  {
    a[v] *= weight;
  }
  *r = weight * off;
}

/** The next of `level`'s rows this tick, over udot, over the wrenches, over a sole's corners,
    or on one torque (weight tau_x = weight rhs). */
static void row_udot(wbc_stack_t *s, int level, const double coef[WBC_N], double rhs,
                     double weight)
{
  const int i = s->lrows[level]++;

  over_udot(s, s->la[level][i], &s->lr[level][i], coef, rhs, weight);
}

static void row_lambda(wbc_stack_t *s, int level, const double coef[WBC_C], double rhs,
                       double weight)
{
  const int i = s->lrows[level]++;

  over_lambda(s, s->la[level][i], &s->lr[level][i], coef, rhs, weight);
}

static void row_corner(wbc_stack_t *s, int level, int k, const double coef[12], double rhs,
                       double weight)
{
  const int i = s->lrows[level]++;

  over_corner(s, s->la[level][i], &s->lr[level][i], k, coef, rhs, weight);
}

static void row_tau(wbc_stack_t *s, int level, int x, double rhs, double weight)
{
  const int i = s->lrows[level]++;

  memset(s->la[level][i], 0, sizeof(s->la[level][i]));
  s->la[level][i][x] = weight;
  s->lr[level][i] = weight * rhs;
}

/* ---- the bounds --------------------------------------------------------------------------- */

/** A bound a . v <= h kept this tick, its row scaled to unit norm; its kind, and which. */
static void bound(wbc_stack_t *s, int kind, int id, const double a[WBC_V], double h)
{
  double nrm = 0.0;

  if (s->ineq >= WBC_INEQ)
  {
    return;
  }
  for (int v = 0; v < WBC_V; v++)
  {
    nrm += a[v] * a[v];
  }
  nrm = sqrt(nrm);
  if (nrm < 1e-12)
  {
    return;
  }
  for (int v = 0; v < WBC_V; v++)
  {
    s->ga[s->ineq][v] = a[v] / nrm;
  }
  s->gh[s->ineq] = h / nrm;
  s->gkind[s->ineq] = kind;
  s->gid[s->ineq] = id;
  s->ineq++;
}

/** Every bound this tick: the drives' loads, her vertical acceleration within HEIGHT_BAND of
    its ask, each standing sole's corners pressing inside their pyramids (world axes, the floor
    level) and its centre of pressure inside its margin, each stop LIMIT_S ahead. */
static void lay_bounds(wbc_body_t *b, wbc_stack_t *s, const double u[WBC_N],
                       const wbc_ask_t *ask)
{
  static const double FACE[4][2] = {{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}};
  double a[WBC_V], lc[WBC_C], coef[WBC_N], cc[12], h;

  s->ineq = 0;
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    memset(a, 0, sizeof(a));
    for (int x = 0; x < WBC_X; x++)
    {
      a[x] = s->pmap[j][x];
    }
    bound(s, 0, j * 10 + 1, a, s->ceiling[j]);
    for (int x = 0; x < WBC_X; x++)
    {
      a[x] = -s->pmap[j][x];
    }
    bound(s, 0, j * 10, a, s->ceiling[j]);
  }
  if (s->nc)
  {
    const double ay = ask->com_acc[1] + WBC_G;

    for (int sign = -1; sign <= 1; sign += 2)
    {
      memset(lc, 0, sizeof(lc));
      for (int k = 0; k < s->nst; k++)
      {
        lc[6 * k + 4] = sign / wbc_total_mass;
      }
      over_lambda(s, a, &h, lc, sign * ay + HEIGHT_BAND, 1.0);
      bound(s, 1, 1020 + (sign > 0), a, h);
    }
  }
  for (int k = 0; k < 2; k++)
  {
    if (s->standing[k] < 0)
    {
      continue;
    }
    for (int c = 0; c < 4; c++)
    {
      /* The pyramid's four faces, +-f_x - mu f_y <= 0 and +-f_z - mu f_y <= 0; then pressing. */
      for (int face = 0; face < 4; face++)
      {
        memset(cc, 0, sizeof(cc));
        cc[3 * c] = FACE[face][0];
        cc[3 * c + 2] = FACE[face][1];
        cc[3 * c + 1] = -MU;
        over_corner(s, a, &h, k, cc, 0.0, 1.0);
        bound(s, 2, 2000 + k * 100 + c * 10 + face, a, h);
      }
      memset(cc, 0, sizeof(cc));
      cc[3 * c + 1] = -1.0;
      over_corner(s, a, &h, k, cc, 0.0, 1.0);
      bound(s, 1, 1000 + k * 100 + c * 10, a, h);
    }
    /* The centre of pressure inside its margin: sum_c -sign (corner_c[axis] - edge) f_c,y <= 0. */
    for (int axis = 0; axis < 3; axis += 2)
    {
      for (int side = 0; side < 2; side++)
      {
        const double edge = side ? s->edge_hi[axis / 2] : s->edge_lo[axis / 2];
        const double sign = side ? -1.0 : 1.0;

        memset(cc, 0, sizeof(cc));
        for (int c = 0; c < 4; c++)
        {
          cc[3 * c + 1] = -sign * (wbc_sole_corner[c][axis] - edge);
        }
        over_corner(s, a, &h, k, cc, 0.0, 1.0);
        bound(s, 3, 3000 + k * 10 + axis + side, a, h);
      }
    }
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int    link = wbc_driven[j], d = 5 + link;
    const double lo = wbc_stop[link][0], hi = wbc_stop[link][1];

    if (lo < hi)
    {
      const double ahead = b->q[d - 6] + u[d] * LIMIT_S;
      const double most = 2.0 * (hi - STOP_RAD - ahead) / (LIMIT_S * LIMIT_S);
      const double least = 2.0 * (lo + STOP_RAD - ahead) / (LIMIT_S * LIMIT_S);

      memset(coef, 0, sizeof(coef));
      coef[d] = 1.0;
      over_udot(s, a, &h, coef, most, 1.0);
      bound(s, 4, 4000 + j * 10 + 1, a, h);
      coef[d] = -1.0;
      over_udot(s, a, &h, coef, -least, 1.0);
      bound(s, 4, 4000 + j * 10, a, h);
    }
  }
}

void wbc_stack_init(wbc_stack_t *s)
{
  double unit[WBC_X], load[WBC_DRIVEN];

  memset(s, 0, sizeof(*s));
  for (int x = 0; x < WBC_X; x++)
  {
    memset(unit, 0, sizeof(unit));
    unit[x] = 1.0;
    wbc_loads(unit, load);
    for (int j = 0; j < WBC_DRIVEN; j++)
    {
      s->pmap[j][x] = load[j];
    }
  }
  /* The margin's edges in the foot's frame, as the corners are given. */
  s->edge_lo[0] = s->edge_lo[1] = 1e9;
  s->edge_hi[0] = s->edge_hi[1] = -1e9;
  for (int c = 0; c < 4; c++)
  {
    const double x = wbc_sole_corner[c][0], z = wbc_sole_corner[c][2];

    s->edge_lo[0] = (x < s->edge_lo[0]) ? x : s->edge_lo[0];
    s->edge_hi[0] = (x > s->edge_hi[0]) ? x : s->edge_hi[0];
    s->edge_lo[1] = (z < s->edge_lo[1]) ? z : s->edge_lo[1];
    s->edge_hi[1] = (z > s->edge_hi[1]) ? z : s->edge_hi[1];
  }
  for (int k = 0; k < 2; k++)
  {
    s->edge_lo[k] += MARGIN_M;
    s->edge_hi[k] -= MARGIN_M;
  }
}

/* ---- the tick ----------------------------------------------------------------------------- */

/** The dynamics at the pose: M = L L^T, udot = u0 + bb tau, the soles' wrenches l0 + ll tau,
    each standing sole's corners about its middle and the split of its wrench among them; 0
    where M, the contacts or a split cannot be factored. */
static int prepare(wbc_body_t *b, wbc_stack_t *s, const double u[WBC_N], const wbc_ask_t *ask)
{
  double e[WBC_N], col[WBC_N];
  int    nst = 0;

  memcpy(s->chol, b->m, sizeof(s->chol));
  if (!wbc_cholesky(WBC_N, WBC_N, &s->chol[0][0]))
  {
    return 0;
  }
  for (int x = 0; x < WBC_X; x++)
  {
    memset(e, 0, sizeof(e));
    e[dof_of(x)] = 1.0;
    wbc_chol_solve(WBC_N, WBC_N, &s->chol[0][0], e, col);
    for (int n = 0; n < WBC_N; n++)
    {
      s->w[n][x] = col[n];
    }
  }
  for (int n = 0; n < WBC_N; n++)
  {
    e[n] = -b->h[n];
  }
  wbc_chol_solve(WBC_N, WBC_N, &s->chol[0][0], e, s->w0);
  for (int k = 0; k < 2; k++)
  {
    s->standing[k] = ask->stance[k] ? nst : -1;
    wbc_body_point(b, wbc_sole[k], wbc_sole_at, s->sole_p[k]);
    memcpy(s->sole_r[k], b->t[wbc_sole[k]].r, sizeof(s->sole_r[k]));
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
      for (int c = 0; c < 4; c++)
      {
        const double local[3] = {wbc_sole_corner[c][0] - wbc_sole_at[0],
                                 wbc_sole_corner[c][1] - wbc_sole_at[1],
                                 wbc_sole_corner[c][2] - wbc_sole_at[2]};

        for (int i = 0; i < 3; i++)
        {
          s->corner[k][c][i] = s->sole_r[k][3 * i] * local[0] + s->sole_r[k][3 * i + 1] * local[1]
                               + s->sole_r[k][3 * i + 2] * local[2];
        }
      }
      if (!wbc_corner_split(s->corner[k], s->wp[k], s->bw[k]))
      {
        return 0;
      }
      nst++;
    }
  }
  s->nst = nst;
  s->nc = 6 * nst;
  if (s->nc == 0)
  {
    memcpy(s->bb, s->w, sizeof(s->bb));
    memcpy(s->u0, s->w0, sizeof(s->u0));
    return 1;
  }
  {
    double gram[WBC_C][WBC_C], rhs[WBC_C], sol[WBC_C];

    for (int i = 0; i < s->nc; i++)
    {
      wbc_chol_solve(WBC_N, WBC_N, &s->chol[0][0], s->jc[i], col);
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
      gram[i][i] += 1e-12;
    }
    if (!wbc_cholesky(s->nc, WBC_C, &gram[0][0]))
    {
      return 0;
    }
    for (int j = 0; j < s->nc; j++)
    {
      memset(rhs, 0, sizeof(rhs));
      rhs[j] = 1.0;
      wbc_chol_solve(s->nc, WBC_C, &gram[0][0], rhs, sol);
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
  return 1;
}

/** The levels' rows this tick, from the ask. */
static void lay_levels(wbc_body_t *b, wbc_stack_t *s, const double u[WBC_N],
                       const wbc_ask_t *ask)
{
  const double origin[3] = {0.0, 0.0, 0.0};
  const double fs = wbc_total_mass * WBC_G;
  double       coef[WBC_N], lc[WBC_C], cc[12], have[4], err[3];

  memset(s->lrows, 0, sizeof(s->lrows));

  /* Level 0: the held joints brought to rest. */
  for (int k = 0; k < WBC_HELD; k++)
  {
    const int d = 5 + wbc_held[k];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    row_udot(s, 0, coef, -HELD_K * u[d], 1.0);
  }

  /* Level 1: the centre of mass along the floor, by Newton on the soles' forces. */
  if (s->nc)
  {
    for (int axis = 0; axis < 3; axis += 2)
    {
      memset(lc, 0, sizeof(lc));
      for (int k = 0; k < s->nst; k++)
      {
        lc[6 * k + 3 + axis] = 1.0 / wbc_total_mass;
      }
      row_lambda(s, 1, lc, ask->com_acc[axis], 1.0);
    }
  }

  /* Level 2: the pelvis's and the trunk's turns, a swinging sole and its knee's fold beside. */
  for (int t = 0; t < 2; t++)
  {
    const int link = t ? wbc_trunk : 0;
    double    drift[6], wdot[3];

    wbc_body_jacobian(b, link, origin, s->jac);
    wbc_body_drift(b, link, origin, drift);
    wbc_quat_of(b->t[link].r, have);
    wbc_turn_error(ask->turn[t], have, err);
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
      row_udot(s, 2, s->jac[i], wdot[i] - drift[i], TURN_W);
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
      wbc_quat_of(s->sole_r[k], have);
      wbc_turn_error(ask->swing_quat[k], have, err);
      for (int i = 0; i < 3; i++)
      {
        a6[i] = TURN_KP * err[i] - TURN_KD * twist[i];
        a6[3 + i] = ask->swing_acc[k][i] + SWING_KP * (ask->swing_at[k][i] - s->sole_p[k][i])
                    + SWING_KD * (ask->swing_speed[k][i] - twist[3 + i]);
      }
      for (int i = 0; i < 6; i++)
      {
        row_udot(s, SWING_LEVEL, s->jac[i], a6[i] - drift[i], 1.0);
      }
    }
    if (ask->fold[k])
    {
      const int d = 5 + wbc_knee[k];

      memset(coef, 0, sizeof(coef));
      coef[d] = 1.0;
      row_udot(s, SWING_LEVEL, coef,
               ask->fold_acc[k] + SWING_KP * (ask->fold_knee[k] - b->q[d - 6])
               + SWING_KD * (ask->fold_rate[k] - u[d]), FOLD_W);
    }
  }

  /* Level 4, her form: her height, the angular momentum, the posture, the torques, the forces. */
  if (s->nc)
  {
    double k3[3], kdot[3];

    memset(lc, 0, sizeof(lc));
    for (int k = 0; k < s->nst; k++)
    {
      lc[6 * k + 4] = 1.0 / wbc_total_mass;
    }
    row_lambda(s, 4, lc, ask->com_acc[1] + WBC_G, HEIGHT_W);
    wbc_body_momentum(b, s->com, k3);
    for (int i = 0; i < 3; i++)
    {
      kdot[i] = ask->kdot_given ? ask->kdot[i] : -MOMENTUM_K * k3[i];
    }
    /* k-dot = the sum of each sole's moment and (sole - com) x its force. */
    for (int i = 0; i < 3; i++)
    {
      memset(lc, 0, sizeof(lc));
      for (int k = 0; k < 2; k++)
      {
        if (s->standing[k] >= 0)
        {
          const double d[3] = {s->sole_p[k][0] - s->com[0], s->sole_p[k][1] - s->com[1],
                               s->sole_p[k][2] - s->com[2]};
          const int    at = 6 * s->standing[k];

          lc[at + i] = 1.0;
          lc[at + 3 + (i + 1) % 3] += -d[(i + 2) % 3];
          lc[at + 3 + (i + 2) % 3] += d[(i + 1) % 3];
        }
      }
      row_lambda(s, 4, lc, kdot[i], MOMENTUM_W);
    }
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int d = 5 + wbc_driven[j];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    row_udot(s, 4, coef, POSTURE_KP * (ask->posture[j] - b->q[d - 6]) - POSTURE_KD * u[d],
             POSTURE_W);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    row_tau(s, 4, j, 0.0, TORQUE_W / (CLAMP_SHARE * wbc_clamp[wbc_driven[j]]));
  }
  for (int k = 0; k < 2; k++)
  {
    if (s->standing[k] >= 0)
    {
      for (int i = 0; i < 12; i++)
      {
        memset(cc, 0, sizeof(cc));
        cc[i] = 1.0 / fs;
        row_corner(s, 4, k, cc, 0.0, FORCE_W);
      }
    }
  }
}

/** What the variables s->total give: every acceleration, each sole's wrench and its corners'
    forces, each drive's load. */
static void evaluate(wbc_stack_t *s)
{
  for (int n = 0; n < WBC_N; n++)
  {
    s->udot_now[n] = s->u0[n];
    for (int x = 0; x < WBC_X; x++)
    {
      s->udot_now[n] += s->bb[n][x] * s->total[x];
    }
  }
  for (int i = 0; i < s->nc; i++)
  {
    s->lam_now[i] = s->l0[i];
    for (int x = 0; x < WBC_X; x++)
    {
      s->lam_now[i] += s->ll[i][x] * s->total[x];
    }
  }
  for (int k = 0; k < 2; k++)
  {
    memset(s->force[k], 0, sizeof(s->force[k]));
    if (s->standing[k] >= 0)
    {
      for (int i = 0; i < 12; i++)
      {
        double v = 0.0;

        for (int j = 0; j < 6; j++)
        {
          v += s->wp[k][i][j] * s->lam_now[6 * s->standing[k] + j]
               + s->bw[k][i][j] * s->total[WBC_X + 6 * k + j];
        }
        s->force[k][i] = v;
      }
    }
  }
  wbc_loads(s->total, s->load_now);
}

void wbc_stack_step(wbc_body_t *b, wbc_stack_t *s, const wbc_frame_t *base,
                    const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                    wbc_out_t *out)
{
  double before[WBC_DRIVEN], load[WBC_DRIVEN];
  int    active = 0;

  wbc_body_pose(b, base, q);
  wbc_body_mass(b);
  wbc_body_bias(b, u);
  wbc_body_com(b, s->com, s->jcom);
  if (!prepare(b, s, u, ask))
  {
    memset(out, 0, sizeof(*out));
    return;
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    s->nominal[j] = wbc_clamp[wbc_driven[j]] * ask->derate[j];
    s->ceiling[j] = (ask->wep && (s->load[j] > NEAR)) ? wbc_peak[wbc_driven[j]] : s->nominal[j];
    if (s->ceiling[j] < s->nominal[j])
    {
      s->ceiling[j] = s->nominal[j];
    }
  }
  lay_levels(b, s, u, ask);
  lay_bounds(b, s, u, ask);
  s->iterations = 0;
  memset(s->nn, 0, sizeof(s->nn));
  for (int v = 0; v < WBC_V; v++)
  {
    s->nn[v][v] = 1.0;
  }
  for (int level = 0; level < WBC_LEVELS; level++)
  {
    wbc_level_solve(s, level, level == WBC_LEVELS - 1);
  }
  for (int v = 0; v < WBC_V; v++)
  {
    s->total[v] = 0.0;
    for (int k = 0; k < WBC_LEVELS; k++)
    {
      s->total[v] += s->tau[k][v];
    }
  }
  evaluate(s);
  wbc_loads(s->total, before);

  /* The bounds met, and how: the most any is over, those at their edge. */
  out->residual = 0.0;
  memset(out->guarded, 0, sizeof(out->guarded));
  for (int i = 0; i < WBC_EXTRA; i++)
  {
    out->ids[i] = -1;
  }
  for (int i = 0; i < s->ineq; i++)
  {
    double v = -s->gh[i];

    for (int m = 0; m < WBC_V; m++)
    {
      v += s->ga[i][m] * s->total[m];
    }
    out->residual = (v > out->residual) ? v : out->residual;
    if (v > -ACTIVE)
    {
      out->guarded[s->gkind[i]]++;
      if (active < WBC_EXTRA)
      {
        out->ids[active] = s->gid[i];
      }
      active++;
    }
  }
  out->held = active;
  out->passes = s->iterations;
  out->alpha[0] = out->alpha[1] = out->alpha[2] = 1.0;

  /* Whatever is still over its ceiling is clamped; what the variables give, out. */
  memcpy(load, s->load_now, sizeof(load));
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    if (load[j] > s->ceiling[j])
    {
      load[j] = s->ceiling[j];
    }
    if (load[j] < -s->ceiling[j])
    {
      load[j] = -s->ceiling[j];
    }
  }
  wbc_unload(load, s->total);
  evaluate(s);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double gain = s->bb[dof_of(j)][j];

    out->tau[j] = s->total[j];
    out->over[j] = fabs(before[j]) > s->nominal[j] * (1.0 + 1e-6);
    out->load[j] = fabs(load[j]) / s->nominal[j];
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
  memcpy(out->udot, s->udot_now, sizeof(out->udot));
  memcpy(out->force, s->force, sizeof(out->force));
  for (int k = 0; k < 2; k++)
  {
    out->bears[k] = 0.0;
    memset(out->wrench[k], 0, sizeof(out->wrench[k]));
    if (s->standing[k] >= 0)
    {
      memcpy(out->wrench[k], s->lam_now + 6 * s->standing[k], sizeof(out->wrench[k]));
      out->bears[k] = out->wrench[k][4];
    }
  }
}
