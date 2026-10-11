/** wbc_stack.c - the loop: the asks of R^k into the drives' torques, by priority, within bounds.

    The variables every acceleration and each standing sole's four corner forces, as the python
    stack has them: in this space a task's row has zeros where a joint cannot move it, and the
    joints that cannot help a swing stay the posture's. Exact first: the dynamics' rows for the
    dofs no drive holds, the held joints at rest, the standing soles still. Then by priority,
    least squares in the null space of the levels above and under every bound - each drive's
    load (its torque from the dynamics' row) within its clamp as derated (its peak under war
    emergency power where it ran near its clamp), her vertical acceleration within its band, each
    corner pressing inside its friction pyramid, each sole's centre of pressure inside its
    margin, a joint short of its stop - by an active set (wbc_solve.c). A pair's rods share
    loads as machine.wbc._currents has it. machine.wbc's levels and bounds, deterministic
    (docs/findings/wbc.md). */
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
/** A margin's slack weighed SLACK_W (its square root a row) on the turns' level: hard, the
    turns went unmet by 5-60 rad/s^2 under 100 N and the head took its peak in their stead
    (2026-10-11). */
static const double SLACK_W = 1e3;
static const double MU = 0.6, MARGIN_M = 0.015, LIMIT_S = 0.05, STOP_RAD = 0.01, HEIGHT_BAND = 3.0;
/** A bound within ACTIVE of its edge is reported at it. */
static const double ACTIVE = 1e-3;
/** The level a swinging sole and its fold are laid on: beside the turns, as the python stack. */
#define SWING_LEVEL 2

/** The variable a corner's force component is; the variable a sole's margin's slack is. */
static int fvar(int k, int c, int axis)
{
  return WBC_N + WBC_F * k + 3 * c + axis;
}

static int svar(int k, int axis, int side)
{
  return WBC_N + 2 * WBC_F + 4 * k + axis + side;
}

/* ---- rows over the variables ----------------------------------------------------------- */

/** The next of `level`'s rows this tick: `a` over the variables, = rhs, weighted. */
static void lay(wbc_stack_t *s, int level, const double a[WBC_V], double rhs, double weight)
{
  const int i = s->lrows[level]++;

  for (int v = 0; v < WBC_V; v++)
  {
    s->la[level][i][v] = weight * a[v];
  }
  s->lr[level][i] = weight * rhs;
}

/** A row over the accelerations alone. */
static void lay_udot(wbc_stack_t *s, int level, const double coef[WBC_N], double rhs,
                     double weight)
{
  double a[WBC_V];

  memset(a, 0, sizeof(a));
  memcpy(a, coef, WBC_N * sizeof(double));
  lay(s, level, a, rhs, weight);
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
  double a[WBC_V], h;

  s->ineq = 0;
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    /* The load, a pair's as shared, over the variables: sum_x pmap[j][x] (tc[x] . v + toff[x]). */
    memset(a, 0, sizeof(a));
    h = 0.0;
    for (int x = 0; x < WBC_DRIVEN; x++)
    {
      if (s->pmap[j][x] != 0.0)
      {
        for (int v = 0; v < WBC_V; v++)
        {
          a[v] += s->pmap[j][x] * s->tc[x][v];
        }
        h -= s->pmap[j][x] * s->toff[x];
      }
    }
    bound(s, 0, j * 10 + 1, a, s->ceiling[j] + h);
    for (int v = 0; v < WBC_V; v++)
    {
      a[v] = -a[v];
    }
    bound(s, 0, j * 10, a, s->ceiling[j] - h);
  }
  if (s->nst)
  {
    const double ay = ask->com_acc[1] + WBC_G;

    for (int sign = -1; sign <= 1; sign += 2)
    {
      memset(a, 0, sizeof(a));
      for (int k = 0; k < 2; k++)
      {
        for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
        {
          a[fvar(k, c, 1)] = sign * s->fs / wbc_total_mass;
        }
      }
      bound(s, 1, 1200 + (sign > 0), a, sign * ay + HEIGHT_BAND);
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
      for (int face = 0; face < 4; face++)
      {
        memset(a, 0, sizeof(a));
        a[fvar(k, c, 0)] = FACE[face][0];
        a[fvar(k, c, 2)] = FACE[face][1];
        a[fvar(k, c, 1)] = -MU;
        bound(s, 2, 2000 + k * 100 + c * 10 + face, a, 0.0);
      }
      memset(a, 0, sizeof(a));
      a[fvar(k, c, 1)] = -1.0;
      bound(s, 1, 1000 + k * 100 + c * 10, a, 0.0);
    }
    /* The centre of pressure inside its margin: sum_c -sign (corner_c[axis] - edge) f_c,y <= 0. */
    for (int axis = 0; axis < 3; axis += 2)
    {
      for (int side = 0; side < 2; side++)
      {
        const double edge = side ? s->edge_hi[axis / 2] : s->edge_lo[axis / 2];
        const double sign = side ? -1.0 : 1.0;

        memset(a, 0, sizeof(a));
        for (int c = 0; c < 4; c++)
        {
          a[fvar(k, c, 1)] = -sign * (wbc_sole_corner[c][axis] - edge);
        }
        a[svar(k, axis, side)] = -1.0;
        bound(s, 3, 3000 + k * 10 + axis + side, a, 0.0);
        memset(a, 0, sizeof(a));
        a[svar(k, axis, side)] = -1.0;
        bound(s, 3, 5000 + k * 10 + axis + side, a, 0.0);
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

      memset(a, 0, sizeof(a));
      a[d] = 1.0;
      bound(s, 4, 4000 + j * 10 + 1, a, most);
      a[d] = -1.0;
      bound(s, 4, 4000 + j * 10, a, -least);
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
  /* The dofs no drive and no stop holds: the pelvis's six, and the links of kind free. */
  for (int d = 0; d < 6; d++)
  {
    s->freed[s->nfree++] = d;
  }
  for (int link = 1; link < WBC_LINKS; link++)
  {
    if (wbc_kind[link] == WBC_FREE)
    {
      s->freed[s->nfree++] = 5 + link;
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

/** The contacts at the pose: each standing sole's Jacobian and drift at its middle, each
    corner's place and linear Jacobian; each drive's torque over the variables, from its row
    of the dynamics; M factored for each drive's inertia alone. 0 where M is not positive. */
static int prepare(wbc_body_t *b, wbc_stack_t *s, const wbc_ask_t *ask)
{
  int nst = 0;

  s->fs = wbc_total_mass * WBC_G;
  memcpy(s->chol, b->m, sizeof(s->chol));
  if (!wbc_cholesky(WBC_N, WBC_N, &s->chol[0][0]))
  {
    return 0;
  }
  for (int k = 0; k < 2; k++)
  {
    s->standing[k] = ask->stance[k] ? nst : -1;
    wbc_body_point(b, wbc_sole[k], wbc_sole_at, s->sole_p[k]);
    if (!ask->stance[k])
    {
      continue;
    }
    wbc_body_jacobian(b, wbc_sole[k], wbc_sole_at, s->jsole[k]);
    wbc_body_drift(b, wbc_sole[k], wbc_sole_at, s->dsole[k]);
    for (int c = 0; c < 4; c++)
    {
      wbc_body_point(b, wbc_sole[k], wbc_sole_corner[c], s->pcorner[k][c]);
      wbc_body_jacobian(b, wbc_sole[k], wbc_sole_corner[c], s->jac);
      memcpy(s->jcorner[k][c], s->jac[3], sizeof(s->jcorner[k][c]));
    }
    nst++;
  }
  s->nst = nst;
  /* tau_x = M[d] . udot + h[d] - sum_c Jc_c[d] . f_c, d the drive's dof. */
  for (int x = 0; x < WBC_DRIVEN; x++)
  {
    const int d = 5 + wbc_driven[x];

    memset(s->tc[x], 0, sizeof(s->tc[x]));
    memcpy(s->tc[x], b->m[d], WBC_N * sizeof(double));
    s->toff[x] = b->h[d];
    for (int k = 0; k < 2; k++)
    {
      for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
      {
        for (int axis = 0; axis < 3; axis++)
        {
          s->tc[x][fvar(k, c, axis)] = -s->fs * s->jcorner[k][c][axis][d];
        }
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
  double       a[WBC_V], coef[WBC_N], have[4], err[3];

  memset(s->lrows, 0, sizeof(s->lrows));

  /* Level 0, exact: the dynamics' rows no drive holds; the held joints at rest; the standing
     soles still. */
  for (int i = 0; i < s->nfree; i++)
  {
    const int d = s->freed[i];

    memset(a, 0, sizeof(a));
    memcpy(a, b->m[d], WBC_N * sizeof(double));
    for (int k = 0; k < 2; k++)
    {
      for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
      {
        for (int axis = 0; axis < 3; axis++)
        {
          a[fvar(k, c, axis)] = -s->fs * s->jcorner[k][c][axis][d];
        }
      }
    }
    lay(s, 0, a, -b->h[d], 1.0);
  }
  for (int k = 0; k < WBC_HELD; k++)
  {
    const int d = 5 + wbc_held[k];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    lay_udot(s, 0, coef, -HELD_K * u[d], 1.0);
  }
  for (int k = 0; k < 2; k++)
  {
    if (s->standing[k] < 0)
    {
      continue;
    }
    for (int i = 0; i < 6; i++)
    {
      double twist = 0.0;

      for (int n = 0; n < WBC_N; n++)
      {
        twist += s->jsole[k][i][n] * u[n];
      }
      lay_udot(s, 0, s->jsole[k][i], -s->dsole[k][i] - CONTACT_K * twist, 1.0);
    }
  }

  /* Level 1: the centre of mass along the floor, by Newton on the corners' forces. */
  if (s->nst)
  {
    for (int axis = 0; axis < 3; axis += 2)
    {
      memset(a, 0, sizeof(a));
      for (int k = 0; k < 2; k++)
      {
        for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
        {
          a[fvar(k, c, axis)] = s->fs / wbc_total_mass;
        }
      }
      lay(s, 1, a, ask->com_acc[axis], 1.0);
    }
  }
  for (int i = 0; i < WBC_S; i++)
  {
    memset(a, 0, sizeof(a));
    a[WBC_N + 2 * WBC_F + i] = 1.0;
    lay(s, 2, a, 0.0, sqrt(SLACK_W));
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
      lay_udot(s, 2, s->jac[i], wdot[i] - drift[i], TURN_W);
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
      wbc_quat_of(b->t[wbc_sole[k]].r, have);
      wbc_turn_error(ask->swing_quat[k], have, err);
      for (int i = 0; i < 3; i++)
      {
        a6[i] = TURN_KP * err[i] - TURN_KD * twist[i];
        a6[3 + i] = ask->swing_acc[k][i] + SWING_KP * (ask->swing_at[k][i] - s->sole_p[k][i])
                    + SWING_KD * (ask->swing_speed[k][i] - twist[3 + i]);
      }
      for (int i = 0; i < 6; i++)
      {
        lay_udot(s, SWING_LEVEL, s->jac[i], a6[i] - drift[i], 1.0);
      }
    }
    if (ask->fold[k])
    {
      const int d = 5 + wbc_knee[k];

      memset(coef, 0, sizeof(coef));
      coef[d] = 1.0;
      lay_udot(s, SWING_LEVEL, coef,
               ask->fold_acc[k] + SWING_KP * (ask->fold_knee[k] - b->q[d - 6])
               + SWING_KD * (ask->fold_rate[k] - u[d]), FOLD_W);
    }
  }

  /* Level 4, her form: her height, the angular momentum, the posture, the torques, the forces. */
  if (s->nst)
  {
    double k3[3], kdot[3];

    memset(a, 0, sizeof(a));
    for (int k = 0; k < 2; k++)
    {
      for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
      {
        a[fvar(k, c, 1)] = s->fs / wbc_total_mass;
      }
    }
    lay(s, 4, a, ask->com_acc[1] + WBC_G, HEIGHT_W);
    wbc_body_momentum(b, s->com, k3);
    for (int i = 0; i < 3; i++)
    {
      kdot[i] = ask->kdot_given ? ask->kdot[i] : -MOMENTUM_K * k3[i];
    }
    /* k-dot = the sum over the corners of (corner - com) x its force. */
    for (int i = 0; i < 3; i++)
    {
      memset(a, 0, sizeof(a));
      for (int k = 0; k < 2; k++)
      {
        for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
        {
          const double d[3] = {s->pcorner[k][c][0] - s->com[0], s->pcorner[k][c][1] - s->com[1],
                               s->pcorner[k][c][2] - s->com[2]};

          a[fvar(k, c, (i + 1) % 3)] = -s->fs * d[(i + 2) % 3];
          a[fvar(k, c, (i + 2) % 3)] = s->fs * d[(i + 1) % 3];
        }
      }
      lay(s, 4, a, kdot[i], MOMENTUM_W);
    }
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int d = 5 + wbc_driven[j];

    memset(coef, 0, sizeof(coef));
    coef[d] = 1.0;
    lay_udot(s, 4, coef, POSTURE_KP * (ask->posture[j] - b->q[d - 6]) - POSTURE_KD * u[d],
             POSTURE_W);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    lay(s, 4, s->tc[j], -s->toff[j], TORQUE_W / (CLAMP_SHARE * wbc_clamp[wbc_driven[j]]));
  }
  for (int k = 0; k < 2; k++)
  {
    for (int c = 0; (s->standing[k] >= 0) && (c < 4); c++)
    {
      for (int axis = 0; axis < 3; axis++)
      {
        memset(a, 0, sizeof(a));
        a[fvar(k, c, axis)] = 1.0;
        lay(s, 4, a, 0.0, FORCE_W);
      }
    }
  }
}

/** The levels' contributions summed into s->total; the drives' torques and loads they give. */
static void total_of(wbc_stack_t *s)
{
  for (int v = 0; v < WBC_V; v++)
  {
    s->total[v] = 0.0;
    for (int k = 0; k < WBC_LEVELS; k++)
    {
      s->total[v] += s->tau[k][v];
    }
  }
  memset(s->torque, 0, sizeof(s->torque));
  for (int x = 0; x < WBC_DRIVEN; x++)
  {
    double t = s->toff[x];

    for (int v = 0; v < WBC_V; v++)
    {
      t += s->tc[x][v] * s->total[v];
    }
    s->torque[x] = t;
  }
  wbc_loads(s->torque, s->load_now);
}

void wbc_stack_step(wbc_body_t *b, wbc_stack_t *s, const wbc_frame_t *base,
                    const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                    wbc_out_t *out)
{
  double before[WBC_DRIVEN], load[WBC_DRIVEN], e[WBC_N], col[WBC_N];
  int    active = 0;

  wbc_body_pose(b, base, q);
  wbc_body_mass(b);
  wbc_body_bias(b, u);
  wbc_body_com(b, s->com, s->jcom);
  if (!prepare(b, s, ask))
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
  s->stuck = 0;
  memset(s->zb, 0, sizeof(s->zb));
  for (int v = 0; v < WBC_V; v++)
  {
    s->zb[v][v] = 1.0;
  }
  s->zm = WBC_V;
  for (int level = 0; level < WBC_LEVELS; level++)
  {
    wbc_level_solve(s, level, level == WBC_LEVELS - 1);
  }
  total_of(s);
  memcpy(before, s->load_now, sizeof(before));

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
    if ((v > -ACTIVE) && (s->gid[i] < 5000))
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
  out->stuck = s->stuck;
  /* How far each of the first three levels' rows are from met, unit rows, the worst. */
  for (int level = 0; level < 3; level++)
  {
    double worst = 0.0;

    for (int i = 0; i < s->lrows[level]; i++)
    {
      double got = 0.0, nrm = 0.0;

      for (int v = 0; v < WBC_V; v++)
      {
        got += s->la[level][i][v] * s->total[v];
        nrm += s->la[level][i][v] * s->la[level][i][v];
      }
      got = fabs(got - s->lr[level][i]) / ((nrm > 1e-24) ? sqrt(nrm) : 1.0);
      worst = (got > worst) ? got : worst;
    }
    out->alpha[level] = worst;
  }
  out->clipped[0] = out->clipped[1] = 1.0;

  /* Whatever load is still over its ceiling is clamped, the torques following; out. */
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
  wbc_unload(load, s->torque);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int d = 5 + wbc_driven[j];

    memset(e, 0, sizeof(e));
    e[d] = 1.0;
    wbc_chol_solve(WBC_N, WBC_N, &s->chol[0][0], e, col);
    out->tau[j] = s->torque[j];
    out->over[j] = fabs(before[j]) > s->nominal[j] * (1.0 + 1e-6);
    out->load[j] = fabs(load[j]) / s->nominal[j];
    s->load[j] = out->load[j];
    out->jeff[j] = (col[d] > 1e-9) ? 1.0 / col[d] : 0.0;
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
  memcpy(out->udot, s->total, sizeof(out->udot));
  for (int k = 0; k < 2; k++)
  {
    out->bears[k] = 0.0;
    memset(out->wrench[k], 0, sizeof(out->wrench[k]));
    memset(out->force[k], 0, sizeof(out->force[k]));
    if (s->standing[k] < 0)
    {
      continue;
    }
    for (int c = 0; c < 4; c++)
    {
      const double  f[3] = {s->fs * s->total[fvar(k, c, 0)], s->fs * s->total[fvar(k, c, 1)],
                            s->fs * s->total[fvar(k, c, 2)]};
      const double  r[3] = {s->pcorner[k][c][0] - s->sole_p[k][0],
                            s->pcorner[k][c][1] - s->sole_p[k][1],
                            s->pcorner[k][c][2] - s->sole_p[k][2]};

      memcpy(out->force[k] + 3 * c, f, 3 * sizeof(double));
      out->wrench[k][0] += r[1] * f[2] - r[2] * f[1];
      out->wrench[k][1] += r[2] * f[0] - r[0] * f[2];
      out->wrench[k][2] += r[0] * f[1] - r[1] * f[0];
      for (int i = 0; i < 3; i++)
      {
        out->wrench[k][3 + i] += f[i];
      }
    }
    out->bears[k] = out->wrench[k][4];
  }
}
