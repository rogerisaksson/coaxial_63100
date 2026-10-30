/** wbc_cone.c - a standing sole's wrench in its cone, for the step without an active set
    (wbc_wrench.c): clipped a part at a time for its rounds (`wbc_cone_bounded`); striding, met
    a face at a time by her inertia (`wbc_cone_taken`: machine.stack.wrench's `faced` and `_taken`). */
#include "wbc.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

#define NU WBC_U

/** A sole's yaw moment's arm, m. */
static const double YAW_M = 0.04;

static double clip(double x, double lo, double hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** x with each standing sole's wrench inside its bounds - pressing, in its friction's pyramid,
    its centre of pressure in its room, toward its ball or heel as `edge` has it - into at;
    which parts moved into moved. */
void wbc_cone_bounded(const wbc_body_t *b, const wbc_wrench_t *w, const wbc_ask_t *ask,
                      const double stance[2], const double x[NU], double at[NU], double moved[NU])
{
  memcpy(at, x, NU * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double *r = b->t[wbc_sole[k]].r, *m = x + 6 * k, *f = x + 6 * k + 3;
    const double  fy = (f[1] > 0.0) ? f[1] : 0.0, e = ask->edge[k], mu = ask->mu[k];
    const double  cap[3] = {w->room[1] * fy, mu * YAW_M * fy, w->room[0] * fy};
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
    in[3] = clip(f[0], -mu * fy, mu * fy);
    in[4] = fy;
    in[5] = clip(f[2], -mu * fy, mu * fy);
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

/** x with each standing sole's wrench in its cone, as `bounded`, into at; the faces it is past
    - six a sole: its moment about its own axes, then its force - into past, and each one's
    share of the sole's push into to (machine.stack.wrench.faced). */
static void faced(const wbc_body_t *b, const wbc_wrench_t *w, const wbc_ask_t *ask,
                  const double stance[2], const double x[NU], double at[NU], double past[12],
                  double to[12])
{
  memcpy(at, x, NU * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double *r = b->t[wbc_sole[k]].r, *m = x + 6 * k, e = ask->edge[k], mu = ask->mu[k];
    const double  high[6] = {w->room[1] * (1.0 - 2.0 * ((e > 0.0) ? e : 0.0)), mu * YAW_M, w->room[0],
                             mu, 1.0, mu};
    const double  low[6] = {-w->room[1] * (1.0 - 2.0 * ((e < 0.0) ? -e : 0.0)), -mu * YAW_M,
                            -w->room[0], -mu, 0.0, -mu};
    double        was[6], in[6], fy;

    for (int i = 0; i < 3; i++)
    {
      was[i] = r[i] * m[0] + r[3 + i] * m[1] + r[6 + i] * m[2];
      was[3 + i] = m[3 + i];
    }
    fy = (was[4] > 0.0) ? was[4] : 0.0;
    for (int i = 0; i < 6; i++)
    {
      in[i] = clip(was[i], low[i] * fy, high[i] * fy);
      past[6 * k + i] = stance[k] * ((fabs(in[i] - was[i]) > 1e-9) ? 1.0 : 0.0);
      to[6 * k + i] = (was[i] < low[i] * fy) ? low[i] : high[i];
    }
    for (int i = 0; i < 3; i++)
    {
      const double world = r[3 * i] * in[0] + r[3 * i + 1] * in[1] + r[3 * i + 2] * in[2];

      at[6 * k + i] += stance[k] * (world - x[6 * k + i]);
      at[6 * k + 3 + i] += stance[k] * (in[3 + i] - x[6 * k + 3 + i]);
    }
  }
}

/** Her kinetic energy's metric over the unknowns - full^T M full, udot = full x - and each
    unknown's own weight on its diagonal: w->k's lower half. full = T P + E is not formed: M T
    and M E over what T and E reach, then T^T M T, T^T M E and E^T M E. mbt the pelvis's rows
    of M T. */
static void kinetic(const wbc_body_t *b, wbc_wrench_t *w, const double mbt[6][6], const double own[NU])
{
  const int nled = 12 + w->npassive;
  double    tmt[6][6];

  memcpy(tmt, mbt, sizeof(tmt));
  for (int k = 0; k < nled; k++)
  {
    const int     i = w->led[k];
    const double *m = b->m[i], *t = w->t[i];
    double       *mt = w->mt[i];

    for (int c = 0; c < 6; c++)
    {
      mt[c] = m[c];
    }
    for (int j = 0; j < nled; j++)
    {
      const double mn = m[w->led[j]];

      for (int c = 0; (mn != 0.0) && (c < 6); c++)
      {
        mt[c] += mn * w->t[w->led[j]][c];
      }
    }
    for (int a = 0; a < 6; a++)
    {
      for (int c = 0; c < 6; c++)
      {
        tmt[a][c] += t[a] * mt[c];
      }
    }
  }
  /* M E: a dof's row of e reaches few unknowns, a column of M few rows */
  memset(w->me, 0, sizeof(w->me));
  for (int n = 6; n < WBC_N; n++)
  {
    for (int i = 0; (w->reach[n][0] < w->reach[n][1]) && (i < WBC_N); i++)
    {
      const double mn = b->m[i][n];

      for (int c = w->reach[n][0]; (mn != 0.0) && (c < w->reach[n][1]); c++)
      {
        w->me[i][c] += mn * w->e[n][c];
      }
    }
  }
  /* T^T M E, the pelvis's rows of M E its own; then (T^T M T) P + T^T M E */
  memcpy(w->tme, w->me, sizeof(w->tme));
  for (int k = 0; k < nled; k++)
  {
    const double *t = w->t[w->led[k]], *me = w->me[w->led[k]];

    for (int c = 0; c < NU; c++)
    {
      const double v = me[c];

      for (int a = 0; (v != 0.0) && (a < 6); a++)
      {
        w->tme[a][c] += t[a] * v;
      }
    }
  }
  for (int a = 0; a < 6; a++)
  {
    for (int c = 0; c < NU; c++)
    {
      double v = w->tme[a][c];

      for (int i = 0; i < 6; i++)
      {
        v += tmt[a][i] * w->p[i][c];
      }
      w->xk[a][c] = v;
    }
  }
  /* P^T ((T^T M T) P + T^T M E) + (T^T M E)^T P + E^T M E */
  for (int a = 0; a < NU; a++)
  {
    memset(w->k[a], 0, sizeof(w->k[a]));
    w->k[a][a] = own[a] * own[a];
  }
  for (int i = 0; i < 6; i++)
  {
    for (int a = 0; a < NU; a++)
    {
      const double pa = w->p[i][a], ta = w->tme[i][a];

      for (int c = 0; c <= a; c++)
      {
        w->k[a][c] += pa * w->xk[i][c] + ta * w->p[i][c];
      }
    }
  }
  for (int n = 6; n < WBC_N; n++)
  {
    for (int a = w->reach[n][0]; a < w->reach[n][1]; a++)
    {
      const double e = w->e[n][a];

      for (int c = 0; (e != 0.0) && (c <= a); c++)
      {
        w->k[a][c] += e * w->me[n][c];
      }
    }
  }
}

/** Striding, what the floor cannot give her inertia takes: u0, the rows' answer with the floor
    unbounded, brought to the faces her soles' wrenches are past by the least change in the
    metric w->k holds factored - a face a part of a sole's wrench, about its own axes, less its
    share of the sole's push; her height's band decided before the faces, and giving where a
    cone and it cannot both be met. Into at (machine.stack.wrench._taken). */
static void taken(const wbc_body_t *b, wbc_wrench_t *w, const wbc_ask_t *ask, const double stance[2],
                  double lo, double hi, const double u0[NU], double at[NU])
{
  const double mass = wbc_total_mass, grounded = (stance[0] > stance[1]) ? stance[0] : stance[1];
  const double g = -wbc_gravity[1];
  double       met[13], share[13], rhs[13], lam[13], x[NU], unit[NU], past[12], to[12];
  double       level = 0.0, some = 0.0;
  int          have[12], on[13];

  memset(met, 0, sizeof(met));
  memset(share, 0, sizeof(share));
  memset(have, 0, sizeof(have));
  memcpy(x, u0, sizeof(x));
  memcpy(at, u0, sizeof(x));
  for (int round = 0; round < WBC_ROUNDS; round++)
  {
    const double ay = (stance[0] * x[4] + stance[1] * x[10]) / mass - g;
    const double edge = clip(ay, lo, hi) + g;
    const double newly = ((grounded > 0.0) && (fabs(edge - g - ay) > 1e-9)) ? 1.0 - met[12] : 0.0;
    int          n_on = 0;

    level += (1.0 - met[12]) * (edge - level);
    met[12] += newly;
    if ((round > 0) && (some == 0.0) && (newly == 0.0))
    {
      break;                      /* no face met anew: the rounds to come would give the same */
    }
    /* each met face over the soles' unknowns, and the metric's inverse on it: the inverse's
       columns a sole's unknowns have, each solved once */
    for (int j = 0; j < 13; j++)
    {
      const int     k = (j % 12) / 6, i = j % 6;
      const double *r = b->t[wbc_sole[k]].r;
      double       *cf = w->cf[j], *z = w->z[n_on];

      if (met[j] == 0.0)
      {
        continue;
      }
      memset(cf, 0, sizeof(w->cf[j]));
      for (int a = 0; (j < 12) && (i < 3) && (a < 3); a++)
      {
        cf[6 * k + a] = r[3 * a + i];
      }
      cf[6 * k + i] += ((j < 12) && (i >= 3)) ? 1.0 : 0.0;
      cf[6 * k + 4] -= (j < 12) ? share[j] : 0.0;
      cf[4] += (j == 12) ? stance[0] / mass : 0.0;
      cf[10] += (j == 12) ? stance[1] / mass : 0.0;
      memset(z, 0, sizeof(w->z[0]));
      rhs[n_on] = (j == 12) ? -level : 0.0;
      for (int c = 0; c < 12; c++)
      {
        const double f = cf[c];

        if ((f != 0.0) && !have[c])
        {
          memset(unit, 0, sizeof(unit));
          unit[c] = 1.0;
          wbc_chol_solve(NU, NU, &w->k[0][0], unit, w->gi[c]);
          have[c] = 1;
        }
        for (int n = 0; (f != 0.0) && (n < NU); n++)
        {
          z[n] += f * w->gi[c][n];
        }
        rhs[n_on] += f * u0[c];
      }
      on[n_on++] = j;
    }
    for (int a = 0; a < n_on; a++)
    {
      for (int c = 0; c <= a; c++)
      {
        double v = ((a == c) && (on[a] == 12)) ? 1e-9 : 0.0;

        for (int n = 0; n < 12; n++)
        {
          v += w->cf[on[a]][n] * w->z[c][n];
        }
        w->s[a][c] = v;
      }
    }
    if ((n_on > 0) && !wbc_cholesky(n_on, 13, &w->s[0][0]))
    {
      break;
    }
    wbc_chol_solve(n_on, 13, &w->s[0][0], rhs, lam);
    for (int n = 0; n < NU; n++)
    {
      double v = u0[n];

      for (int a = 0; a < n_on; a++)
      {
        v -= lam[a] * w->z[a][n];
      }
      x[n] = v;
    }
    faced(b, w, ask, stance, x, at, past, to);
    some = 0.0;
    for (int j = 0; j < 12; j++)
    {
      share[j] += past[j] * (to[j] - share[j]);
      met[j] = (past[j] > met[j]) ? past[j] : met[j];
      some += past[j];
    }
  }
}

int wbc_cone_taken(const wbc_body_t *b, wbc_wrench_t *w, const wbc_ask_t *ask, const double stance[2],
                   double lo, double hi, const double mbt[6][6], const double own[NU],
                   const double u0[NU], double at[NU])
{
  kinetic(b, w, mbt, own);
  if (!wbc_cholesky(NU, NU, &w->k[0][0]))
  {
    return 0;
  }
  taken(b, w, ask, stance, lo, hi, u0, at);
  return 1;
}
