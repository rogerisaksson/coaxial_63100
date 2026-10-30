/** wbc_wrench.c - her whole body's step without an active set: machine/stack/wrench.py, row for row.

    WBC_U unknowns: six a sole - the wrench it takes of the floor where it stands (m, f; world,
    about its middle), its own acceleration where it does not - and her upper body's drives'
    accelerations. Every acceleration follows from them and the pelvis's six, udot = T base +
    E x + t0: a leg through its sole's damped inverse, the pelvis by the six base rows of her
    dynamics. One weighted least squares on the normal equations finds x, WBC_ROUNDS times, each
    wrench clipped into its sole's bounds and what was clipped pinned for the next. Striding,
    what the floor cannot give is not traded among the rows again: her inertia takes it
    (wbc_cone.c). An ask the drives cannot carry is shrunk as one toward nothing asked - the
    same rows solved again with every acceleration 0 and her weight borne - as far as brings
    every torque within its clamp (wbc_shrunk.c). No iteration but those rounds
    (docs/findings/wrench.md). */
#include "wbc.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

#define NU WBC_U
#define UPPER (WBC_DRIVEN - 12)

static const double G = 9.81;
static const double HELD_K = 50.0, CONTACT_K = 20.0, MOMENTUM_K = 3.0, HEIGHT_BAND = 3.0;
static const double TURN_KP = 100.0, TURN_KD = 20.0, POSTURE_KP = 100.0, POSTURE_KD = 20.0;
static const double SWING_KP = 1600.0, SWING_KD = 80.0;
/** machine/stack/wrench.py's: a sole's edge, m; a leg's solve's damping; the rows' weights. */
static const double EDGE_M = 0.003, DAMP = 0.02;
static const double W_COM = 1000.0, W_TURN = 30.0, W_SWING = 30.0, W_KNEE = 3.0, W_POSTURE = 1.0;
static const double W_HEIGHT = 1.0, W_MOMENT = 0.3, W_FOLD = 3.0, W_STOP = 30.0, W_PART = 0.1;
static const double W_AIR = 1e-3, W_PIN = 1e5;
/* striding, a standing sole's wrench kept in the projection's metric, of its share of her weight:
   her soles pushed against each other move her by nothing (machine/stack/wrench.py, W_KEEP) */
static const double W_KEEP = 3.0;

static double clip(double x, double lo, double hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** A row over the unknowns, = c, weighed, onto the normal equations: their lower half, which
    is all a Cholesky reads; = none with nothing asked, onto the second right-hand side. */
static void lay(wbc_wrench_t *w, const double r[NU], double c, double none, double weight)
{
  const double ww = weight * weight;

  for (int i = 0; (ww != 0.0) && (i < NU); i++)
  {
    const double ri = ww * r[i];

    w->g[i] += ri * c;
    w->gn[i] += ri * none;
    for (int j = 0; (ri != 0.0) && (j <= i); j++)
    {
      w->h[i][j] += ri * r[j];
    }
  }
}

/** A row over her accelerations, r, as a row over the unknowns, and what is left of it:
    udot = T (P x + p0) + E x + t0. */
static double over(const wbc_wrench_t *w, const double r[WBC_N], double row[NU])
{
  double rt[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, rest = 0.0;

  memset(row, 0, NU * sizeof(double));
  for (int n = 0; n < WBC_N; n++)
  {
    const double rn = r[n];

    rest += rn * w->t0[n];
    for (int i = 0; (rn != 0.0) && (i < 6); i++)
    {
      rt[i] += rn * w->t[n][i];
    }
    for (int c = w->reach[n][0]; (rn != 0.0) && (c < w->reach[n][1]); c++)
    {
      row[c] += rn * w->e[n][c];
    }
  }
  for (int i = 0; i < 6; i++)
  {
    rest += rt[i] * w->p0[i];
    for (int c = 0; (rt[i] != 0.0) && (c < NU); c++)
    {
      row[c] += rt[i] * w->p[i][c];
    }
  }
  return rest;
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
  /* a leg's dof reaches its sole's six unknowns as it swings (maps), an upper drive its own,
     a passive joint what its row comes to (passive_of) */
  for (int i = 0; i < 12; i++)
  {
    w->led[i] = 5 + wbc_driven[w->leg[i / 6][i % 6]];
  }
  for (int i = 0; i < UPPER; i++)
  {
    w->reach[5 + wbc_driven[w->upper[i]]][0] = 12 + i;
    w->reach[5 + wbc_driven[w->upper[i]]][1] = 13 + i;
  }
  for (int i = 0; i < w->npassive; i++)
  {
    w->led[12 + i] = w->passive[i];
    w->reach[w->passive[i]][1] = NU;
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
  enum { COLS = 6 + NU + 1 };
  double mpp[WBC_PASSIVE][WBC_PASSIVE], inv[WBC_PASSIVE][WBC_PASSIVE], rhs[WBC_PASSIVE][COLS];
  double unit[WBC_PASSIVE], x[WBC_PASSIVE];

  for (int i = 0; i < w->npassive; i++)
  {
    const double *m = b->m[w->passive[i]];

    /* less what her other accelerations put on its row: only the dofs its row of M reaches */
    memset(rhs[i], 0, sizeof(rhs[i]));
    rhs[i][COLS - 1] = -b->h[w->passive[i]];
    for (int n = 0; n < WBC_N; n++)
    {
      if (m[n] != 0.0)
      {
        for (int c = 0; c < 6; c++)
        {
          rhs[i][c] -= m[n] * w->t[n][c];
        }
        for (int c = w->reach[n][0]; c < w->reach[n][1]; c++)
        {
          rhs[i][6 + c] -= m[n] * w->e[n][c];
        }
        rhs[i][COLS - 1] -= m[n] * w->t0[n];
      }
    }
    for (int j = 0; j < w->npassive; j++)
    {
      mpp[i][j] = m[w->passive[j]];
    }
  }
  if (!wbc_cholesky(w->npassive, WBC_PASSIVE, &mpp[0][0]))
  {
    return;
  }
  for (int j = 0; j < w->npassive; j++)
  {
    memset(unit, 0, sizeof(unit));
    unit[j] = 1.0;
    wbc_chol_solve(w->npassive, WBC_PASSIVE, &mpp[0][0], unit, x);
    for (int i = 0; i < w->npassive; i++)
    {
      inv[i][j] = x[i];
    }
  }
  for (int i = 0; i < w->npassive; i++)
  {
    double *t = w->t[w->passive[i]], *e = w->e[w->passive[i]], rest = 0.0;

    for (int j = 0; j < w->npassive; j++)
    {
      const double a = inv[i][j];           /* 0 off its own chain's block */

      for (int c = 0; (a != 0.0) && (c < 6); c++)
      {
        t[c] += a * rhs[j][c];
      }
      for (int c = 0; (a != 0.0) && (c < NU); c++)
      {
        e[c] += a * rhs[j][6 + c];
      }
      rest += a * rhs[j][COLS - 1];
    }
    w->t0[w->passive[i]] = rest;
    /* what its row reaches: its first filled unknown to its last */
    w->reach[w->passive[i]][0] = NU;
    w->reach[w->passive[i]][1] = 0;
    for (int c = NU - 1; c >= 0; c--)
    {
      w->reach[w->passive[i]][0] = (e[c] != 0.0) ? c : w->reach[w->passive[i]][0];
    }
    for (int c = 0; c < NU; c++)
    {
      w->reach[w->passive[i]][1] = (e[c] != 0.0) ? c + 1 : w->reach[w->passive[i]][1];
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
      dof[i] = w->led[6 * k + i];
      w->reach[dof[i]][0] = 6 * k;
      w->reach[dof[i]][1] = 6 * k + ((ask->swing[k] && !ask->stance[k]) ? 6 : 0);
    }
    /* its twist: the pelvis's six and its leg's move it, none else */
    for (int i = 0; i < 6; i++)
    {
      twist[i] = 0.0;
      for (int c = 0; c < 6; c++)
      {
        twist[i] += w->jsole[k][i][c] * u[c] + w->jsole[k][i][dof[c]] * u[dof[c]];
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

/** The unknowns for the rows' right-hand side g, her height's band lo to hi: WBC_ROUNDS
    solves, the clipped parts pinned, her height held to its band's edge once past; striding,
    what the floor cannot give her inertia takes. 0 where a solve has no factor. */
static int solved(wbc_body_t *b, wbc_wrench_t *w, const wbc_ask_t *ask, const double stance[2],
                  double grounded, double lo, double hi, const double g[NU], const double part[NU],
                  const double own[NU], const double mbt[6][6], double at[NU])
{
  const int    n3 = 0;
  const double mass = wbc_total_mass;
  double       x[NU], pinned[NU], moved[NU], hard[NU], way[NU], row[NU], u0[NU];
  double       band = 0.0, edge = 0.0, bent = 0.0;

  memset(pinned, 0, sizeof(pinned));
  memset(at, 0, NU * sizeof(double));
  memset(u0, 0, sizeof(u0));
  for (int round = 0, rounds = WBC_ROUNDS; round < rounds; round++)
  {
    double ay, past, held = 0.0, clipped = 0.0;

    memcpy(w->k, w->h, sizeof(w->k));
    memcpy(w->gk, g, NU * sizeof(double));
    /* a clipped force is held by a row, a clipped moment is its value and the rest solved
       round it (machine/stack/wrench.py) */
    for (int c = 0; c < NU; c++)
    {
      const double pw = W_PIN * pinned[c] * part[c] * (((c < 12) && (c % 6 < 3)) ? 0.0 : 1.0);

      hard[c] = pinned[c] * (((c < 12) && (c % 6 < 3)) ? 1.0 : 0.0);
      held += hard[c];
      w->k[c][c] += pw * pw;
      w->gk[c] += pw * pw * at[c];
    }
    for (int i = 0; (held > 0.0) && (i < NU); i++)
    {
      for (int j = i + 1; j < NU; j++)
      {
        w->k[i][j] = w->k[j][i];
      }
    }
    for (int i = 0; (held > 0.0) && (i < NU); i++)
    {
      double moved_to = 0.0;

      for (int j = 0; j < NU; j++)
      {
        moved_to += w->k[i][j] * hard[j] * at[j];
      }
      w->gk[i] = (1.0 - hard[i]) * (w->gk[i] - moved_to) + hard[i] * at[i];
    }
    for (int i = 0; (held > 0.0) && (i < NU); i++)
    {
      for (int j = 0; j < NU; j++)
      {
        w->k[i][j] = (1.0 - hard[i]) * (1.0 - hard[j]) * w->k[i][j] + ((i == j) ? hard[i] : 0.0);
      }
    }
    if (!wbc_cholesky(NU, NU, &w->k[0][0]))
    {
      return 0;
    }
    wbc_chol_solve(NU, NU, &w->k[0][0], w->gk, x);
    if (round == 0)
    {
      memcpy(u0, x, sizeof(u0));  /* the rows' answer, the floor unbounded */
    }
    if (band > 0.0)
    {
      double step;

      /* her height at its band's edge met: x along K^-1 of its row, as far as that takes */
      memset(row, 0, sizeof(row));
      row[4] = stance[0] / mass;
      row[10] = stance[1] / mass;
      wbc_chol_solve(NU, NU, &w->k[0][0], row, way);
      step = (edge - row[4] * x[4] - row[10] * x[10]) / (row[4] * way[4] + row[10] * way[10]);
      for (int c = 0; c < NU; c++)
      {
        x[c] += step * way[c];
      }
    }
    ay = (stance[0] * x[4] + stance[1] * x[10]) / mass - G;
    past = clip(ay, lo, hi) + G;
    edge += (1.0 - band) * (past - edge);
    band = ((grounded > 0.0) && (fabs(past - G - ay) > 1e-9)) ? 1.0 : band;
    wbc_cone_bounded(b, w, ask, stance, x, at, moved);
    for (int c = 0; c < NU; c++)
    {
      pinned[c] = (moved[c] > pinned[c]) ? moved[c] : pinned[c];
      clipped += pinned[c];
    }
    /* an answer past her height's band after the rounds is held to its edge (machine/stack/wrench.py) */
    ay = (stance[0] * at[4] + stance[1] * at[10]) / mass - G;
    if ((round == rounds - 1) && (rounds < WBC_ROUNDS + WBC_PAST) && (ask->strides == 0.0) && (grounded > 0.0)
        && (fabs(clip(ay, lo, hi) - ay) > 1e-9))
    {
      band = 1.0;
      edge = clip(ay, lo, hi) + G;
      rounds++;
    }
    bent = clipped + band;
    if ((bent == 0.0) || ((ask->strides >= 1.0) && (n3 == 0)))
    {
      break;                      /* nothing clipped: the rounds to come would solve the same;
                                     striding whole, their answer is not hers */
    }
  }
  if ((ask->strides > 0.0) && (bent > 0.0))
  {
    /* striding: what the floor cannot give her inertia takes, as far as she strides */
    if (!wbc_cone_taken(b, w, ask, stance, lo, hi, mbt, own, u0, way))
    {
      return 0;
    }
    for (int c = 0; c < NU; c++)
    {
      at[c] += ask->strides * (way[c] - at[c]);
    }
  }
  return 1;
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
  double       a[6][6], ai[6][6], d[6], mbt[6][6], own[NU], whole[NU], stood[NU];

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
      /* the pelvis's own block, and what its six lead */
      a[i][c] = b->m[i][c] + ((i == c) ? 1e-9 : 0.0);
      for (int n = 0; n < 12 + w->npassive; n++)
      {
        a[i][c] += b->m[i][w->led[n]] * w->t[w->led[n]][c];
      }
    }
    for (int c = 0; c < NU; c++)
    {
      w->bm[i][c] = (c < 12) ? stance[c / 6] * w->jsole[c / 6][c % 6][i] : 0.0;
    }
  }
  for (int n = 0; n < WBC_N; n++)
  {
    for (int c = w->reach[n][0]; c < w->reach[n][1]; c++)
    {
      const double e = w->e[n][c];

      for (int i = 0; (e != 0.0) && (i < 6); i++)
      {
        w->bm[i][c] -= b->m[i][n] * e;
      }
    }
  }
  memcpy(mbt, a, sizeof(mbt));
  for (int i = 0; i < 6; i++)
  {
    mbt[i][i] -= 1e-9;
  }
  if (!wbc_invert6(a, ai))
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

  /* the rows, onto the normal equations: the ask's right-hand side, and nothing's */
  memset(w->h, 0, sizeof(w->h));
  memset(w->g, 0, sizeof(w->g));
  memset(w->gn, 0, sizeof(w->gn));
  for (int axis = 0; axis < 3; axis++)
  {
    const double borne = (axis == 1) ? G : 0.0;

    memset(row, 0, sizeof(row));
    row[3 + axis] = stance[0] / mass;
    row[9 + axis] = stance[1] / mass;
    lay(w, row, ask->com_acc[axis] + borne, borne, grounded * ((axis == 1) ? W_HEIGHT : W_COM));
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
      const double rest = over(w, w->jac[i], row);
      double       spin = 0.0;

      for (int n = 0; n < WBC_N; n++)
      {
        spin += w->jac[i][n] * u[n];
      }
      lay(w, row, TURN_KP * err[i] - TURN_KD * spin - drift[i] - rest, -drift[i] - rest, W_TURN * grounded * asks);
    }
  }
  /* an unknown asked alone: its own weight on the diagonal */
  for (int i = 0; i < UPPER; i++)
  {
    w->h[12 + i][12 + i] += W_POSTURE * W_POSTURE;
    w->g[12 + i] += W_POSTURE * W_POSTURE * want[w->upper[i]];
  }
  for (int c = 0; c < 12; c++)
  {
    const double ww = W_SWING * W_SWING * asked[c / 6];

    w->h[c][c] += ww;
    w->g[c] += ww * a6[c];
  }
  {
    /* each knee's acceleration over the unknowns: her standing knees' posture, a swinging
       knee's fold */
    double pick[WBC_N], bends[2][NU], left[2], rhs = 0.0, none = 0.0;

    memset(row, 0, sizeof(row));
    for (int k = 0; k < 2; k++)
    {
      memset(pick, 0, sizeof(pick));
      pick[5 + wbc_knee[k]] = 1.0;
      left[k] = over(w, pick, bends[k]);
      rhs += stance[k] * ((want[w->leg[k][3]] + POSTURE_KD * ask->knee_rate[k]) - left[k]);
      none -= stance[k] * left[k];
      for (int c = 0; c < NU; c++)
      {
        row[c] += stance[k] * bends[k][c];
      }
    }
    lay(w, row, rhs, none, W_KNEE * grounded);
    for (int k = 0; k < 2; k++)
    {
      const int    knee = 5 + wbc_knee[k];
      const double bend = b->q[knee - 6];

      lay(w, bends[k], ask->fold_acc[k] + SWING_KP * (ask->fold_knee[k] - bend)
          + SWING_KD * (ask->fold_rate[k] - u[knee]) - left[k], -left[k], ask->fold[k] ? W_FOLD : 0.0);
    }
  }
  /* the master insert's buffers: each joint sprung back, critically damped, from its width
     before its stop, engaged as far again as its approach's speed reaches (wbc_model.h) */
  for (int i = 0; i < WBC_BUFFERS; i++)
  {
    const int    dof = 5 + wbc_buffer_link[i];
    const double side = wbc_buffer_stop[i][1], depth = side * (wbc_buffer_stop[i][0] - b->q[dof - 6]) + wbc_buffer[0];
    double       pick[WBC_N], stop, left;

    memset(pick, 0, sizeof(pick));
    pick[dof] = 1.0;
    left = over(w, pick, row);
    stop = (wbc_buffer[1] * side) * fmax(depth, 0.0) - wbc_buffer[2] * u[dof];
    lay(w, row, stop - left, -left,
        W_STOP * clip((depth + wbc_buffer[3] * fmax(-side * u[dof], 0.0)) / wbc_buffer[0], 0.0, 1.0));
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
    lay(w, row, ask->kdot_given ? ask->kdot[i] : -MOMENTUM_K * k3[i], 0.0, W_MOMENT * grounded);
  }
  for (int c = 0; c < NU; c++)
  {
    const double sole = (c < 12) ? stance[c / 6] : 0.0, air = (c < 12) ? 1.0 - stance[c / 6] : 0.0;
    double       weight;

    part[c] = (c >= 12) ? 0.0 : ((c % 6 < 3) ? 1.0 / (mass * G * w->room[0]) : 1.0 / (mass * G));
    weight = W_PART * sole * part[c] + W_AIR * air;
    own[c] = W_SWING * ((c < 12) ? asked[c / 6] : 0.0) + weight;
    own[c] = sqrt(own[c] * own[c] + W_KEEP * W_KEEP * sole * part[c] * part[c]);
    w->h[c][c] += weight * weight + 1e-12;
  }

  /* the ask's answer, and nothing's: every acceleration 0, her weight borne, by the same rows */
  if (!solved(b, w, ask, stance, grounded, lo, hi, w->g, part, own, mbt, whole)
      || !solved(b, w, ask, stance, grounded, -HEIGHT_BAND, HEIGHT_BAND, w->gn, part, own, mbt, stood))
  {
    return;
  }
  wbc_wrench_out(b, w, ask, stance, whole, stood, out);
}
