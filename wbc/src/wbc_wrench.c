/** wbc_wrench.c - her whole body's step without an active set: machine/wrench.py, row for row.

    WBC_U unknowns: six a sole - the wrench it takes of the floor where it stands (m, f; world,
    about its middle), its own acceleration where it does not - and her upper body's drives'
    accelerations. Every acceleration follows from them and the pelvis's six, udot = T base +
    E x + t0: a leg through its sole's damped inverse, the pelvis by the six base rows of her
    dynamics. One weighted least squares on the normal equations finds x, ROUNDS times, each
    wrench clipped into its sole's bounds and what was clipped pinned for the next; the drives'
    clamps clip the torques after it. No iteration but those rounds
    (docs/findings/wrench.md). */
#include "wbc.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

#define NU WBC_U
#define UPPER (WBC_DRIVEN - 12)
#define ROUNDS 4

static const double G = 9.81;
static const double HELD_K = 50.0, CONTACT_K = 20.0, MOMENTUM_K = 3.0, MU = 0.6, HEIGHT_BAND = 3.0;
static const double TURN_KP = 100.0, TURN_KD = 20.0, POSTURE_KP = 100.0, POSTURE_KD = 20.0;
static const double SWING_KP = 1600.0, SWING_KD = 80.0;
/** machine/wrench.py's: a sole's edge and its yaw's arm, m; a leg's solve's damping; a knee
    nearer straight than KNEE_MIN rad is sprung back; the rows' weights. */
static const double EDGE_M = 0.003, YAW_M = 0.04, DAMP = 0.02, KNEE_MIN = 0.07;
static const double W_COM = 1000.0, W_TURN = 30.0, W_SWING = 30.0, W_KNEE = 3.0, W_POSTURE = 1.0;
static const double W_HEIGHT = 1.0, W_MOMENT = 0.3, W_FOLD = 3.0, W_STOP = 30.0, W_PART = 0.1;
static const double W_AIR = 1e-3, W_PIN = 1e5;

static double clip(double x, double lo, double hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** A row over the unknowns, = c, weighed, onto the normal equations. */
static void lay(wbc_wrench_t *w, const double r[NU], double c, double weight)
{
  const double ww = weight * weight;

  for (int i = 0; i < NU; i++)
  {
    const double ri = ww * r[i];

    w->g[i] += ri * c;
    for (int j = 0; (ri != 0.0) && (j < NU); j++)
    {
      w->h[i][j] += ri * r[j];
    }
  }
}

/** a's inverse by Gauss and Jordan, the largest pivot a column; 0 where it has none. */
static int invert6(double a[6][6], double inv[6][6])
{
  memset(inv, 0, 36U * sizeof(double));
  for (int i = 0; i < 6; i++)
  {
    inv[i][i] = 1.0;
  }
  for (int c = 0; c < 6; c++)
  {
    int    best = c;
    double d;

    for (int r = c + 1; r < 6; r++)
    {
      best = (fabs(a[r][c]) > fabs(a[best][c])) ? r : best;
    }
    if (fabs(a[best][c]) < 1e-300)
    {
      return 0;
    }
    for (int k = 0; k < 6; k++)
    {
      const double x = a[c][k], y = inv[c][k];

      a[c][k] = a[best][k];
      a[best][k] = x;
      inv[c][k] = inv[best][k];
      inv[best][k] = y;
    }
    d = 1.0 / a[c][c];
    for (int k = 0; k < 6; k++)
    {
      a[c][k] *= d;
      inv[c][k] *= d;
    }
    for (int r = 0; r < 6; r++)
    {
      const double f = (r == c) ? 0.0 : a[r][c];

      for (int k = 0; k < 6; k++)
      {
        a[r][k] -= f * a[c][k];
        inv[r][k] -= f * inv[c][k];
      }
    }
  }
  return 1;
}

void wbc_wrench_init(wbc_wrench_t *w)
{
  int upper = 0;

  memset(w, 0, sizeof(*w));
  for (int k = 0; k < 2; k++)
  {
    int at = 5;

    /* her leg's drives from its sole up to the pelvis, kept hip first */
    for (int link = wbc_sole[k]; (link > 0) && (at >= 0); link = wbc_parent[link])
    {
      for (int j = 0; (wbc_kind[link] == WBC_IS_DRIVEN) && (j < WBC_DRIVEN); j++)
      {
        if (wbc_driven[j] == link)
        {
          w->leg[k][at--] = j;
        }
      }
    }
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    int in_leg = 0;

    for (int i = 0; i < 12; i++)
    {
      in_leg |= (w->leg[i / 6][i % 6] == j);
    }
    if (!in_leg && (upper < UPPER))
    {
      w->upper[upper++] = j;
    }
  }
  for (int link = 1; link < WBC_LINKS; link++)
  {
    if (wbc_kind[link] == WBC_FREE)
    {
      w->passive[w->npassive++] = 5 + link;
    }
  }
  for (int c = 0; c < 4; c++)
  {
    const double x = fabs(wbc_sole_corner[c][0] - wbc_sole_at[0]);
    const double z = fabs(wbc_sole_corner[c][2] - wbc_sole_at[2]);

    w->room[0] = (x - EDGE_M > w->room[0]) ? x - EDGE_M : w->room[0];
    w->room[1] = (z - EDGE_M > w->room[1]) ? z - EDGE_M : w->room[1];
  }
}

/** The joints no drive and no stop holds, by their own rows of her dynamics under all the rest:
    their rows of t, e and t0 from the others'. */
static void passive_of(const wbc_body_t *b, wbc_wrench_t *w)
{
  double mpp[WBC_PASSIVE][WBC_PASSIVE], rhs[WBC_PASSIVE], got[WBC_PASSIVE];

  for (int i = 0; i < w->npassive; i++)
  {
    for (int j = 0; j < w->npassive; j++)
    {
      mpp[i][j] = b->m[w->passive[i]][w->passive[j]];
    }
  }
  if (!wbc_cholesky(w->npassive, WBC_PASSIVE, &mpp[0][0]))
  {
    return;
  }
  for (int c = 0; c < 6 + NU + 1; c++)
  {
    for (int i = 0; i < w->npassive; i++)
    {
      const double *m = b->m[w->passive[i]];

      rhs[i] = (c == 6 + NU) ? -b->h[w->passive[i]] : 0.0;
      for (int n = 0; n < WBC_N; n++)
      {
        if (m[n] != 0.0)
        {
          rhs[i] -= m[n] * ((c < 6) ? w->t[n][c] : ((c < 6 + NU) ? w->e[n][c - 6] : w->t0[n]));
        }
      }
    }
    wbc_chol_solve(w->npassive, WBC_PASSIVE, &mpp[0][0], rhs, got);
    for (int i = 0; i < w->npassive; i++)
    {
      if (c < 6)
      {
        w->t[w->passive[i]][c] = got[i];
      }
      else if (c < 6 + NU)
      {
        w->e[w->passive[i]][c - 6] = got[i];
      }
      else
      {
        w->t0[w->passive[i]] = got[i];
      }
    }
  }
}

/** Every acceleration of the pelvis's six and the unknowns, udot = T base + E x + t0; each
    swinging sole's asked acceleration into a6, whether asked into asked. */
static void maps(wbc_body_t *b, wbc_wrench_t *w, const double u[WBC_N], const wbc_ask_t *ask,
                 const double want[WBC_DRIVEN], double a6[12], double asked[2])
{
  memset(w->t, 0, sizeof(w->t));
  memset(w->e, 0, sizeof(w->e));
  memset(w->t0, 0, sizeof(w->t0));
  for (int i = 0; i < 6; i++)
  {
    w->t[i][i] = 1.0;
  }
  for (int k = 0; k < WBC_HELD; k++)
  {
    w->t0[5 + wbc_held[k]] = -HELD_K * u[5 + wbc_held[k]];
  }
  for (int i = 0; i < UPPER; i++)
  {
    w->e[5 + wbc_driven[w->upper[i]]][12 + i] = 1.0;
  }
  for (int k = 0; k < 2; k++)
  {
    double drift[6], twist[6], gram[6][6], inv[6][6], col[6], x[6], have[4], err[3];
    int    dof[6];

    asked[k] = 0.0;
    memset(a6 + 6 * k, 0, 6U * sizeof(double));
    wbc_body_jacobian(b, wbc_sole[k], wbc_sole_at, w->jsole[k]);
    wbc_body_drift(b, wbc_sole[k], wbc_sole_at, drift);
    wbc_body_point(b, wbc_sole[k], wbc_sole_at, w->sole_p[k]);
    for (int i = 0; i < 6; i++)
    {
      dof[i] = 5 + wbc_driven[w->leg[k][i]];
      twist[i] = 0.0;
      for (int n = 0; n < WBC_N; n++)
      {
        twist[i] += w->jsole[k][i][n] * u[n];
      }
    }
    if (!ask->stance[k] && !ask->swing[k])
    {
      for (int i = 0; i < 6; i++)
      {
        w->t0[dof[i]] = want[w->leg[k][i]];
      }
      continue;
    }
    /* the leg's damped inverse: (J^T J + DAMP^2)^-1 J^T, J its six columns */
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 6; j++)
      {
        gram[i][j] = (i == j) ? DAMP * DAMP : 0.0;
        for (int r = 0; r < 6; r++)
        {
          gram[i][j] += w->jsole[k][r][dof[i]] * w->jsole[k][r][dof[j]];
        }
      }
    }
    if (!wbc_cholesky(6, 6, &gram[0][0]))
    {
      continue;
    }
    for (int r = 0; r < 6; r++)
    {
      for (int i = 0; i < 6; i++)
      {
        col[i] = w->jsole[k][r][dof[i]];
      }
      wbc_chol_solve(6, 6, &gram[0][0], col, x);
      for (int i = 0; i < 6; i++)
      {
        inv[i][r] = x[i];
      }
    }
    for (int i = 0; i < 6; i++)
    {
      for (int c = 0; c < 6; c++)
      {
        double v = 0.0;

        for (int r = 0; r < 6; r++)
        {
          v -= inv[i][r] * w->jsole[k][r][c];
        }
        w->t[dof[i]][c] = v;
      }
    }
    if (ask->stance[k])
    {
      for (int i = 0; i < 6; i++)
      {
        double v = 0.0;

        for (int r = 0; r < 6; r++)
        {
          v += inv[i][r] * (ask->settle[k][r] - drift[r] - CONTACT_K * twist[r]);
        }
        w->t0[dof[i]] = v;
      }
      continue;
    }
    wbc_quat_of(b->t[wbc_sole[k]].r, have);
    wbc_turn_error(ask->swing_quat[k], have, err);
    for (int i = 0; i < 3; i++)
    {
      a6[6 * k + i] = TURN_KP * err[i] - TURN_KD * twist[i];
      a6[6 * k + 3 + i] = ask->swing_acc[k][i] + SWING_KP * (ask->swing_at[k][i] - w->sole_p[k][i])
                          + SWING_KD * (ask->swing_speed[k][i] - twist[3 + i]);
    }
    asked[k] = 1.0;
    for (int i = 0; i < 6; i++)
    {
      double v = 0.0;

      for (int r = 0; r < 6; r++)
      {
        v -= inv[i][r] * drift[r];
        w->e[dof[i]][6 * k + r] = inv[i][r];
      }
      w->t0[dof[i]] = v;
    }
  }
  passive_of(b, w);
}

/** x with each standing sole's wrench inside its bounds - pressing, in its friction's pyramid,
    its centre of pressure in its room, toward its ball or heel as `edge` has it - into at;
    which parts moved into moved. */
static void bounded(const wbc_body_t *b, const wbc_wrench_t *w, const wbc_ask_t *ask,
                    const double stance[2], const double x[NU], double at[NU], double moved[NU])
{
  memcpy(at, x, NU * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double *r = b->t[wbc_sole[k]].r, *m = x + 6 * k, *f = x + 6 * k + 3;
    const double  fy = (f[1] > 0.0) ? f[1] : 0.0, e = ask->edge[k];
    const double  cap[3] = {w->room[1] * fy, MU * YAW_M * fy, w->room[0] * fy};
    const double  lo0 = -cap[0] * (1.0 - 2.0 * ((e < 0.0) ? -e : 0.0));
    const double  hi0 = cap[0] * (1.0 - 2.0 * ((e > 0.0) ? e : 0.0));
    double        ml[3], in[6];

    for (int i = 0; i < 3; i++)
    {
      ml[i] = r[i] * m[0] + r[3 + i] * m[1] + r[6 + i] * m[2];
    }
    ml[0] = clip(ml[0], lo0, hi0);
    ml[1] = clip(ml[1], -cap[1], cap[1]);
    ml[2] = clip(ml[2], -cap[2], cap[2]);
    for (int i = 0; i < 3; i++)
    {
      in[i] = r[3 * i] * ml[0] + r[3 * i + 1] * ml[1] + r[3 * i + 2] * ml[2];
    }
    in[3] = clip(f[0], -MU * fy, MU * fy);
    in[4] = fy;
    in[5] = clip(f[2], -MU * fy, MU * fy);
    for (int i = 0; i < 6; i++)
    {
      at[6 * k + i] += stance[k] * (in[i] - x[6 * k + i]);
    }
  }
  for (int c = 0; c < NU; c++)
  {
    moved[c] = (fabs(at[c] - x[c]) > 1e-9) ? 1.0 : 0.0;
  }
}

void wbc_wrench_step(wbc_body_t *b, wbc_wrench_t *w, const wbc_frame_t *base,
                     const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                     wbc_out_t *out)
{
  const double origin[3] = {0.0, 0.0, 0.0};
  const double stance[2] = {ask->stance[0] ? 1.0 : 0.0, ask->stance[1] ? 1.0 : 0.0};
  const double grounded = (stance[0] > stance[1]) ? stance[0] : stance[1];
  const double mass = wbc_total_mass, lo = ask->com_acc[1] - HEIGHT_BAND, hi = ask->com_acc[1] + HEIGHT_BAND;
  double       want[WBC_DRIVEN], a6[12], asked[2], com[3], k3[3], row[NU], part[NU];
  double       a[6][6], ai[6][6], d[6], x[NU], at[NU], pinned[NU], moved[NU], hard[NU], way[NU];
  double       band = 0.0, edge = 0.0;
  int          knee[2];

  memset(out, 0, sizeof(*out));
  wbc_body_pose(b, base, q);
  wbc_body_mass(b);
  wbc_body_bias(b, u);
  wbc_body_com(b, com, w->jcom);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int dof = 5 + wbc_driven[j];

    want[j] = POSTURE_KP * (ask->posture[j] - b->q[dof - 6]) - POSTURE_KD * u[dof];
  }
  maps(b, w, u, ask, want, a6, asked);

  /* the base's rows of her dynamics: A base = (Gm - M_b E) x - d */
  for (int i = 0; i < 6; i++)
  {
    d[i] = b->h[i];
    for (int n = 0; n < WBC_N; n++)
    {
      d[i] += b->m[i][n] * w->t0[n];
    }
    for (int c = 0; c < 6; c++)
    {
      a[i][c] = (i == c) ? 1e-9 : 0.0;
      for (int n = 0; n < WBC_N; n++)
      {
        a[i][c] += b->m[i][n] * w->t[n][c];
      }
    }
    for (int c = 0; c < NU; c++)
    {
      double v = (c < 12) ? stance[c / 6] * w->jsole[c / 6][c % 6][i] : 0.0;

      for (int n = 0; n < WBC_N; n++)
      {
        v -= b->m[i][n] * w->e[n][c];
      }
      w->bm[i][c] = v;
    }
  }
  if (!invert6(a, ai))
  {
    return;
  }
  for (int i = 0; i < 6; i++)
  {
    w->p0[i] = 0.0;
    for (int r = 0; r < 6; r++)
    {
      w->p0[i] -= ai[i][r] * d[r];
    }
    for (int c = 0; c < NU; c++)
    {
      double v = 0.0;

      for (int r = 0; r < 6; r++)
      {
        v += ai[i][r] * w->bm[r][c];
      }
      w->p[i][c] = v;
    }
  }
  for (int n = 0; n < WBC_N; n++)
  {
    w->full0[n] = w->t0[n];
    for (int i = 0; i < 6; i++)
    {
      w->full0[n] += w->t[n][i] * w->p0[i];
    }
    for (int c = 0; c < NU; c++)
    {
      double v = w->e[n][c];

      for (int i = 0; i < 6; i++)
      {
        v += w->t[n][i] * w->p[i][c];
      }
      w->full[n][c] = v;
    }
  }

  /* the rows, onto the normal equations */
  memset(w->h, 0, sizeof(w->h));
  memset(w->g, 0, sizeof(w->g));
  for (int axis = 0; axis < 3; axis++)
  {
    memset(row, 0, sizeof(row));
    row[3 + axis] = stance[0] / mass;
    row[9 + axis] = stance[1] / mass;
    lay(w, row, ask->com_acc[axis] + ((axis == 1) ? G : 0.0), grounded * ((axis == 1) ? W_HEIGHT : W_COM));
  }
  for (int t = 0; t < 2; t++)
  {
    const int    link = t ? wbc_trunk : 0;
    const double asks = ((ask->turn[t][0] != 0.0) || (ask->turn[t][1] != 0.0) || (ask->turn[t][2] != 0.0)
                         || (ask->turn[t][3] != 0.0)) ? 1.0 : 0.0;
    double       drift[6], have[4], err[3];

    wbc_body_jacobian(b, link, origin, w->jac);
    wbc_body_drift(b, link, origin, drift);
    wbc_quat_of(b->t[link].r, have);
    wbc_turn_error(ask->turn[t], have, err);
    for (int i = 0; i < 3; i++)
    {
      double spin = 0.0, rest = 0.0;

      for (int n = 0; n < WBC_N; n++)
      {
        spin += w->jac[i][n] * u[n];
        rest += w->jac[i][n] * w->full0[n];
      }
      memset(row, 0, sizeof(row));
      for (int n = 0; n < WBC_N; n++)
      {
        for (int c = 0; (w->jac[i][n] != 0.0) && (c < NU); c++)
        {
          row[c] += w->jac[i][n] * w->full[n][c];
        }
      }
      lay(w, row, TURN_KP * err[i] - TURN_KD * spin - drift[i] - rest, W_TURN * grounded * asks);
    }
  }
  for (int i = 0; i < UPPER; i++)
  {
    memset(row, 0, sizeof(row));
    row[12 + i] = 1.0;
    lay(w, row, want[w->upper[i]], W_POSTURE);
  }
  for (int c = 0; c < 12; c++)
  {
    memset(row, 0, sizeof(row));
    row[c] = 1.0;
    lay(w, row, a6[c], W_SWING * asked[c / 6]);
  }
  {
    double rhs = 0.0;

    memset(row, 0, sizeof(row));
    for (int k = 0; k < 2; k++)
    {
      knee[k] = 5 + wbc_knee[k];
      rhs += stance[k] * (want[w->leg[k][3]] - w->full0[knee[k]]);
      for (int c = 0; c < NU; c++)
      {
        row[c] += stance[k] * w->full[knee[k]][c];
      }
    }
    lay(w, row, rhs, W_KNEE * grounded);
  }
  for (int k = 0; k < 2; k++)
  {
    const double bend = b->q[knee[k] - 6], shrt = KNEE_MIN - bend;

    lay(w, w->full[knee[k]], ask->fold_acc[k] + SWING_KP * (ask->fold_knee[k] - bend)
        + SWING_KD * (ask->fold_rate[k] - u[knee[k]]) - w->full0[knee[k]], ask->fold[k] ? W_FOLD : 0.0);
    lay(w, w->full[knee[k]], POSTURE_KP * shrt - POSTURE_KD * u[knee[k]] - w->full0[knee[k]],
        W_STOP * clip(shrt / KNEE_MIN, 0.0, 1.0));
  }
  wbc_body_momentum(b, com, k3);
  for (int i = 0; i < 3; i++)
  {
    memset(row, 0, sizeof(row));
    for (int k = 0; k < 2; k++)
    {
      const double r[3] = {w->sole_p[k][0] - com[0], w->sole_p[k][1] - com[1], w->sole_p[k][2] - com[2]};

      /* k-dot = m + r x f */
      row[6 * k + i] = stance[k];
      row[6 * k + 3 + (i + 1) % 3] = -stance[k] * r[(i + 2) % 3];
      row[6 * k + 3 + (i + 2) % 3] = stance[k] * r[(i + 1) % 3];
    }
    lay(w, row, ask->kdot_given ? ask->kdot[i] : -MOMENTUM_K * k3[i], W_MOMENT * grounded);
  }
  for (int c = 0; c < NU; c++)
  {
    const double sole = (c < 12) ? stance[c / 6] : 0.0, air = (c < 12) ? 1.0 - stance[c / 6] : 0.0;
    double       weight;

    part[c] = (c >= 12) ? 0.0 : ((c % 6 < 3) ? 1.0 / (mass * G * w->room[0]) : 1.0 / (mass * G));
    weight = W_PART * sole * part[c] + W_AIR * air;
    w->h[c][c] += weight * weight + 1e-12;
  }

  /* ROUNDS solves: the clipped parts pinned, her height held to its band's edge once past it */
  memset(pinned, 0, sizeof(pinned));
  memset(at, 0, sizeof(at));
  for (int round = 0; round < ROUNDS; round++)
  {
    double ay, past, step;

    memcpy(w->k, w->h, sizeof(w->k));
    memcpy(w->gk, w->g, sizeof(w->gk));
    /* a clipped force is held by a row, a clipped moment is its value and the rest solved
       round it (machine/wrench.py) */
    for (int c = 0; c < NU; c++)
    {
      const double pw = W_PIN * pinned[c] * part[c] * (((c < 12) && (c % 6 < 3)) ? 0.0 : 1.0);

      hard[c] = pinned[c] * (((c < 12) && (c % 6 < 3)) ? 1.0 : 0.0);
      w->k[c][c] += pw * pw;
      w->gk[c] += pw * pw * at[c];
    }
    for (int i = 0; i < NU; i++)
    {
      double moved_to = 0.0;

      for (int j = 0; j < NU; j++)
      {
        moved_to += w->k[i][j] * hard[j] * at[j];
      }
      w->gk[i] = (1.0 - hard[i]) * (w->gk[i] - moved_to) + hard[i] * at[i];
    }
    for (int i = 0; i < NU; i++)
    {
      for (int j = 0; j < NU; j++)
      {
        w->k[i][j] = (1.0 - hard[i]) * (1.0 - hard[j]) * w->k[i][j] + ((i == j) ? hard[i] : 0.0);
      }
    }
    if (!wbc_cholesky(NU, NU, &w->k[0][0]))
    {
      return;
    }
    wbc_chol_solve(NU, NU, &w->k[0][0], w->gk, x);
    /* her height at its band's edge met: x along K^-1 of its row, as far as that takes */
    memset(row, 0, sizeof(row));
    row[4] = stance[0] / mass;
    row[10] = stance[1] / mass;
    wbc_chol_solve(NU, NU, &w->k[0][0], row, way);
    step = band * (edge - row[4] * x[4] - row[10] * x[10])
           / (row[4] * way[4] + row[10] * way[10] + 1.0 - band);
    for (int c = 0; c < NU; c++)
    {
      x[c] += step * way[c];
    }
    ay = (stance[0] * x[4] + stance[1] * x[10]) / mass - G;
    past = clip(ay, lo, hi) + G;
    edge += (1.0 - band) * (past - edge);
    band = ((grounded > 0.0) && (fabs(past - G - ay) > 1e-9)) ? 1.0 : band;
    bounded(b, w, ask, stance, x, at, moved);
    for (int c = 0; c < NU; c++)
    {
      pinned[c] = (moved[c] > pinned[c]) ? moved[c] : pinned[c];
    }
  }

  /* out: the accelerations, the drives' torques inside their clamps, the soles' wrenches */
  for (int n = 0; n < WBC_N; n++)
  {
    out->udot[n] = w->full0[n];
    for (int c = 0; c < NU; c++)
    {
      out->udot[n] += w->full[n][c] * at[c];
    }
  }
  for (int k = 0; k < 2; k++)
  {
    for (int i = 0; i < 6; i++)
    {
      out->wrench[k][i] = stance[k] * at[6 * k + i];
    }
    out->bears[k] = out->wrench[k][4];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int    dof = 5 + wbc_driven[j];
    const double clamp = wbc_clamp[wbc_driven[j]], peak = wbc_peak[wbc_driven[j]];
    const double limit = ((ask->wep && (peak > clamp)) ? peak : clamp) * ask->derate[j];
    double       tau = b->h[dof];

    for (int n = 0; n < WBC_N; n++)
    {
      tau += b->m[dof][n] * out->udot[n];
    }
    for (int k = 0; k < 2; k++)
    {
      for (int i = 0; i < 6; i++)
      {
        tau -= w->jsole[k][i][dof] * out->wrench[k][i];
      }
    }
    out->over[j] = fabs(tau) > clamp * (1.0 + 1e-6);
    out->load[j] = fabs(tau) / clamp;
    out->tau[j] = clip(tau, -limit, limit);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    for (int o = 0; (pair > 0) && (o < WBC_DRIVEN); o++)
    {
      if (wbc_driven[o] == pair - 1)
      {
        out->over[j] = out->over[o] = out->over[j] || out->over[o];
      }
    }
  }
}
