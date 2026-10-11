/** aero_share.c - the thrusts for a wrench inside each rotor's reach: weighted least squares
    over the box by an active set, a fixed count of passes. No branch: a rotor held or let go
    is a mask from a sign bit. */
#include "aero.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define N ((int)AERO_ROTORS)

/** The set's changes a call, a rotor held or let go a pass. */
#define PASSES 8
/** A rotor with no step along its thrust is this far from its end, of that step. */
#define FAR 1e30
/** A held rotor is let go where its end is pulled the wrong way by more than this, of the
    wrench asked. */
#define NEAR 1e-12

/** All ones where a < b: the sign bit of their difference, no flag read. */
static inline uint64_t lt(double a, double b)
{
  const double d = a - b;
  uint64_t     u;

  memcpy(&u, &d, sizeof(u));
  return (uint64_t)0 - (u >> 63);
}

/** `a` where `k` is none, `b` where it is all: by bits, neither one's value touched. */
static inline double pick(uint64_t k, double a, double b)
{
  uint64_t ua, ub;

  memcpy(&ua, &a, sizeof(ua));
  memcpy(&ub, &b, sizeof(ub));
  ua = (ua & ~k) | (ub & k);
  memcpy(&a, &ua, sizeof(a));
  return a;
}

static inline double one(uint64_t k)
{
  return pick(k, 0.0, 1.0);
}

static inline double least(double a, double b)
{
  return pick(lt(b, a), a, b);
}

static inline double most(double a, double b)
{
  return pick(lt(a, b), a, b);
}

/** What the cost falls by a thrust, at `f`: a^T (W want) - H f. */
static void slope(double h[N][N], const double g[N], const double f[N], double r[N])
{
  for (int i = 0; i < N; i++)
  {
    r[i] = g[i];
    for (int j = 0; j < N; j++)
    {
      r[i] -= h[i][j] * f[j];
    }
  }
}

/** 1 under the diagonal: the factor's own entries. */
static const double UNDER[N][N] = {{0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0},
                                   {1.0, 1.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 0.0}};

/** p of H p = r over the rotors `open`, 0 for the held: L D L^T of the curvature, a held
    rotor's row and column the identity's. Every loop its array's length: an entry not yet
    made is a zero. */
static void solve(double h[N][N], const uint64_t open[N], const double r[N], double p[N])
{
  double m[N][N], l[N][N] = {{0.0}}, d[N] = {0.0}, y[N] = {0.0}, is[N];

  for (int i = 0; i < N; i++)
  {
    is[i] = one(open[i]);
    p[i] = 0.0;
  }
  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      m[i][j] = is[i] * is[j] * h[i][j];
    }
    m[i][i] += 1.0 - is[i];
  }
  for (int j = 0; j < N; j++)
  {
    double pivot = m[j][j];

    for (int k = 0; k < N; k++)
    {
      pivot -= l[j][k] * l[j][k] * d[k];
    }
    d[j] = most(pivot, DBL_MIN);
    for (int i = 0; i < N; i++)
    {
      double below = m[i][j];

      for (int k = 0; k < N; k++)
      {
        below -= l[i][k] * l[j][k] * d[k];
      }
      l[i][j] = UNDER[i][j] * below / d[j];
    }
  }
  for (int i = 0; i < N; i++)
  {
    y[i] = is[i] * r[i];
    for (int k = 0; k < N; k++)
    {
      y[i] -= l[i][k] * y[k];
    }
  }
  for (int i = N - 1; i >= 0; i--)
  {
    p[i] = y[i] / d[i];
    for (int k = 0; k < N; k++)
    {
      p[i] -= l[k][i] * p[k];
    }
  }
}

void aero_share(const aero_t *s, const float weigh[AERO_WRENCH],
                const double want[AERO_WRENCH], const float lo[AERO_ROTORS],
                const float hi[AERO_ROTORS], double f[AERO_ROTORS], uint32_t *bound)
{
  double   a[AERO_WRENCH][N], h[N][N], g[N], side[N], r[N], p[N], reach[N];
  double   low[N], high[N], near = NEAR;
  uint64_t open[N];
  uint32_t ends = 0U;

  for (int i = 0; i < N; i++)
  {
    a[0][i] = (double)weigh[0];
    a[1][i] = (double)(-weigh[1] * s->p.at_z[i]);
    a[2][i] = (double)(weigh[2] * s->p.spin[i] * s->p.kappa);
    a[3][i] = (double)(weigh[3] * s->p.at_x[i]);
  }
  for (int i = 0; i < N; i++)
  {
    low[i] = (double)lo[i];
    high[i] = most(low[i], (double)hi[i]);
    f[i] = 0.5 * (low[i] + high[i]);
    open[i] = lt(low[i], high[i]);
    side[i] = 0.0;
    g[i] = 0.0;
    for (int k = 0; k < (int)AERO_WRENCH; k++)
    {
      g[i] += a[k][i] * (double)weigh[k] * want[k];
    }
    for (int j = 0; j < N; j++)
    {
      h[i][j] = 0.0;
      for (int k = 0; k < (int)AERO_WRENCH; k++)
      {
        h[i][j] += a[k][i] * a[k][j];
      }
    }
    near += NEAR * fabs(g[i]);
  }
  for (int pass = 0; pass < PASSES; pass++)
  {
    double   alpha = 1.0, worst = near;
    uint64_t stopped;

    /* to the open rotors' own best, as far along the way as the first end met */
    slope(h, g, f, r);
    solve(h, open, r, p);
    for (int i = 0; i < N; i++)
    {
      const uint64_t up = lt(0.0, p[i]), down = lt(p[i], 0.0);
      const double   room = pick(up, pick(down, 0.0, f[i] - low[i]), high[i] - f[i]);

      reach[i] = pick(up | down, FAR, room / (fabs(p[i]) + one(~(up | down))));
      alpha = least(alpha, reach[i]);
    }
    stopped = lt(alpha, 1.0);
    for (int i = 0; i < N; i++)
    {
      const uint64_t up = lt(0.0, p[i]), down = lt(p[i], 0.0);
      const uint64_t hit = stopped & open[i] & ~lt(alpha, reach[i]);

      f[i] += alpha * p[i];
      f[i] = pick(hit & up, f[i], high[i]);
      f[i] = pick(hit & down, f[i], low[i]);
      side[i] = pick(hit, side[i], one(up) - one(down));
      open[i] &= ~hit;
    }
    /* there, the held rotor whose end the cost pulls inward the most is let go */
    slope(h, g, f, r);
    for (int i = 0; i < N; i++)
    {
      worst = most(worst, -side[i] * r[i] * one(~open[i]));
    }
    for (int i = 0; i < N; i++)
    {
      const double   inward = -side[i] * r[i] * one(~open[i]);
      const uint64_t go = ~stopped & ~lt(inward, worst);

      open[i] |= go;
      side[i] = pick(go, side[i], 0.0);
    }
  }
  for (int i = 0; i < N; i++)
  {
    f[i] = least(high[i], most(low[i], f[i]));
    ends |= (uint32_t)(~lt(low[i], f[i]) & 1U) << i;
    ends |= (uint32_t)(~lt(f[i], high[i]) & 1U) << (i + N);
  }
  *bound = ends;
}
