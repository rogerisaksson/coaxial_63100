/** wbc_rounds.c - the step's rounds (wbc_step.h): machine.wrench's bounded, pushed, faced, _taken
    and _rounds, numpy's arithmetic and numpy's own OpenBLAS (wbc_blas.h) operation for operation.
    Portable C11, the host's alone. */
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

/** machine.wrench.bounded: each standing sole's wrench inside its bounds, and which parts moved. */
static void bounded(const wbc_step_t *p, const wbc_geo_t *g, const double *u, double *out, double *moved)
{
  memcpy(out, u, (size_t)g->nu * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double *uk = u + 6 * k, *R = g->frames[k];
    const double  fx = uk[3], fy = pymax(uk[4], 0.0), fz = uk[5], ek = g->edges[k], muk = g->mu[k];
    const double  c0 = p->room[1] * fy, c1 = (muk * p->yaw_m) * fy, c2 = p->room[0] * fy, mf = muk * fy;
    const double  lo0 = (-c0) * (1.0 - 2.0 * pymax(-ek, 0.0)), hi0 = c0 * (1.0 - 2.0 * pymax(ek, 0.0));
    double        m[3], mc[3], inside[6];

    wbc_matmul(&p->blas, VIEW(R, 1, 3), VIEW(uk, 1, 0), m, 1, 0, 3, 3, 1);
    mc[0] = pymin(pymax(m[0], lo0), hi0);
    mc[1] = pymin(pymax(m[1], -c1), c1);
    mc[2] = pymin(pymax(m[2], -c2), c2);
    wbc_matmul(&p->blas, VIEW(R, 3, 1), VIEW(mc, 1, 0), inside, 1, 0, 3, 3, 1);
    inside[3] = pymin(pymax(fx, -mf), mf);
    inside[4] = fy;
    inside[5] = pymin(pymax(fz, -mf), mf);
    for (int j = 0; j < 6; j++)
    {
      out[6 * k + j] += g->stance[k] * (inside[j] - uk[j]);
    }
  }
  for (int j = 0; j < g->nu; j++)
  {
    moved[j] = (double)(fabs(out[j] - u[j]) > 1e-9);
  }
}

/** machine.rounds.pushed: each contact's force pressing, no more than GROW_N over what it bears nor
    its cap, in its pyramid - a point sliding (geo->slips: sliding, its slip along the world's x and
    z) has its friction the kinetic one against its slip, given -; which parts moved. */
static void pushed(const wbc_step_t *p, const wbc_geo_t *g, const double *u, const double *bears, double *out,
                   double *kept)
{
  const int nc = g->k3 / 3;

  memcpy(out, u, (size_t)g->nu * sizeof(double));
  for (int i = 0; i < nc; i++)
  {
    double      *f = out + p->nu0 + 3 * i;
    const double sliding = g->slips[3 * i], speed = hypot(g->slips[3 * i + 1], g->slips[3 * i + 2]);
    const double den = pymax(speed, 1e-12), still = 1.0 - sliding;

    f[1] = clipped(f[1], 0.0, fmin(bears[i] + p->grow_n, g->caps[i]));
    f[0] = still * clipped(f[0], (-p->mu) * f[1], p->mu * f[1]) - (((sliding * p->mu) * f[1]) * g->slips[3 * i + 1]) / den;
    f[2] = still * clipped(f[2], (-p->mu) * f[1], p->mu * f[1]) - (((sliding * p->mu) * f[1]) * g->slips[3 * i + 2]) / den;
  }
  for (int j = 0; j < g->nu; j++)
  {
    kept[j] = nc ? (double)(fabs(out[j] - u[j]) > 1e-9) : 0.0;
  }
}

/** machine.wrench.faced: each standing sole's wrench in its cone, the faces it was past, each
    face's share of the sole's push. */
static void faced(const wbc_step_t *p, const wbc_geo_t *g, const double *u, double *out, double past[12],
                  double share[12])
{
  memcpy(out, u, (size_t)g->nu * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    const double *R = g->frames[k];
    const double  ek = g->edges[k], muk = g->mu[k];
    const double  low[6] = {-(p->room[1] * (1.0 - 2.0 * pymax(-ek, 0.0))), -(muk * p->yaw_m), -p->room[0], -muk, -0.0,
                            -muk};
    const double  high[6] = {p->room[1] * (1.0 - 2.0 * pymax(ek, 0.0)), muk * p->yaw_m, p->room[0], muk, 1.0, muk};
    double        was[6], inside[6], w[6], fy;

    wbc_matmul(&p->blas, VIEW(R, 1, 3), VIEW(u + 6 * k, 1, 0), was, 1, 0, 3, 3, 1);
    was[3] = u[6 * k + 3];
    was[4] = u[6 * k + 4];
    was[5] = u[6 * k + 5];
    fy = pymax(was[4], 0.0);
    for (int j = 0; j < 6; j++)
    {
      const double lfy = low[j] * fy, hfy = high[j] * fy;

      inside[j] = clipped(was[j], lfy, hfy);
      past[6 * k + j] = g->stance[k] * (double)(fabs(inside[j] - was[j]) > 1e-9);
      share[6 * k + j] = was[j] < lfy ? low[j] : high[j];
    }
    wbc_matmul(&p->blas, VIEW(R, 3, 1), VIEW(inside, 1, 0), w, 1, 0, 3, 3, 1);
    w[3] = inside[3];
    w[4] = inside[4];
    w[5] = inside[5];
    for (int j = 0; j < 6; j++)
    {
      out[6 * k + j] += g->stance[k] * (w[j] - u[6 * k + j]);
    }
  }
}

/** machine.wrench._solved: K^-1 rhs (rhs a view, nu by m), K ridged on its `free` unknowns where
    it has no inverse; 1 where even that raises. `K` is changed by a ridge. */
static int solved(const wbc_step_t *p, double *K, int nu, wbc_view_t rhs, int m, const double *free, double *x,
                  double *work)
{
  double most;

  if (!wbc_solve(&p->blas, VIEW(K, nu, 1), rhs, nu, m, x, work))
  {
    return 0;
  }
  most = K[0];
  for (int i = 1; i < nu; i++)
  {
    most = npmax(most, K[i * nu + i]);
  }
  for (int i = 0; i < nu; i++)
  {
    for (int j = 0; j < nu; j++)
    {
      K[i * nu + j] = K[i * nu + j] + (i == j ? (free[i] * p->ridge) * most : 0.0);
    }
  }
  return wbc_solve(&p->blas, VIEW(K, nu, 1), rhs, nu, m, x, work);
}

/** machine.wrench._taken: striding, what the floor cannot give her inertia takes - `u0` brought
    to the faces her soles' wrenches are past by the least change in her kinetic energy's metric. */
static int taken(const wbc_step_t *p, const wbc_geo_t *g, const double *u0, double lo, double hi, double *at,
                 double *work)
{
  const int nu = g->nu;
  double   *Gi = work, *F = Gi + nu * nu, *N = F + 13 * nu, *C = N + 13 * nu, *GC = C + 13 * nu, *CG = GC + 13 * nu;
  double   *A = CG + 13 * nu, *u = A + 169, *cu = u + nu, *x = cu + 13, *t = x + 13, *rest = t + nu;
  double    met[13] = {0.0}, share[13] = {0.0}, level[13] = {0.0}, out[12], to[12];

  if (wbc_inv(&p->blas, VIEW(g->kinetic, nu, 1), nu, Gi, rest))
  {
    return 1;
  }
  memset(F, 0, (size_t)(26 * nu) * sizeof(double));
  for (int k = 0; k < 2; k++)
  {
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        F[(6 * k + i) * nu + 6 * k + j] = g->frames[k][3 * j + i];
        F[(6 * k + 3 + i) * nu + 6 * k + 3 + j] = (double)(i == j);
      }
    }
    for (int i = 0; i < 6; i++)
    {
      N[(6 * k + i) * nu + 6 * k + 4] = 1.0;
    }
  }
  memcpy(F + 12 * nu, g->up, (size_t)nu * sizeof(double));
  memcpy(u, u0, (size_t)nu * sizeof(double));
  memcpy(at, u0, (size_t)nu * sizeof(double));
  for (int round = 0; round < p->rounds; round++)
  {
    const double ay = wbc_dot(&p->blas, g->up, 1, u, 1, nu) - p->g, past = pymin(pymax(ay, lo), hi) + p->g;

    level[12] += (1.0 - met[12]) * (past - level[12]);
    met[12] = pymax(met[12], g->grounded * (double)(fabs((past - p->g) - ay) > 1e-9));
    for (int i = 0; i < 13; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        C[i * nu + j] = met[i] * (F[i * nu + j] - share[i] * N[i * nu + j]);
      }
    }
    wbc_matmul(&p->blas, VIEW(Gi, nu, 1), VIEW(C, 1, nu), GC, 13, 1, nu, nu, 13);
    wbc_matmul(&p->blas, VIEW(C, nu, 1), VIEW(Gi, nu, 1), CG, nu, 1, 13, nu, nu);
    wbc_matmul(&p->blas, VIEW(CG, nu, 1), VIEW(C, 1, nu), A, 13, 1, 13, nu, 13);
    for (int i = 0; i < 13; i++)
    {
      for (int j = 0; j < 13; j++)
      {
        A[i * 13 + j] = A[i * 13 + j] + (i == j ? (1.0 - met[i]) + 1e-9 * (double)(i == 12) : 0.0);
      }
    }
    wbc_matmul(&p->blas, VIEW(C, nu, 1), VIEW(u0, 1, 0), cu, 1, 0, 13, nu, 1);
    for (int i = 0; i < 13; i++)
    {
      cu[i] = cu[i] - met[i] * level[i];
    }
    if (wbc_solve(&p->blas, VIEW(A, 13, 1), VIEW(cu, 1, 0), 13, 1, x, rest))
    {
      return 1;
    }
    wbc_matmul(&p->blas, VIEW(GC, 13, 1), VIEW(x, 1, 0), t, 1, 0, nu, 13, 1);
    for (int j = 0; j < nu; j++)
    {
      u[j] = u0[j] - t[j];
    }
    faced(p, g, u, at, out, to);
    for (int i = 0; i < 12; i++)
    {
      share[i] += out[i] * (to[i] - share[i]);
      met[i] = npmax(met[i], out[i]);
    }
  }
  return 0;
}

int wbc_rounds(wbc_step_t *p, const wbc_geo_t *g, const double *H0, const double *g0, const double *S,
               const double *r, const double *bears, double lo, double hi, double *at, double *work)
{
  const int nu = g->nu, nc = g->k3 / 3;
  double   *Hr = work, *gr = Hr + nu * nu, *K = gr + nu, *H = K + nu * nu, *B = H + nu * nu, *X = B + 2 * nu;
  double   *pS = X + 2 * nu, *T = pS + 4 * nu, *ht = T + nu * nu, *sumH = ht + nu, *sumg = sumH + nu * nu;
  double   *pinned = sumg + nu, *hard = pinned + nu, *soft = hard + nu, *free = soft + nu, *un = free + nu;
  double   *moved = un + nu, *kept = moved + nu, *held = kept + nu, *rest = held + nu;
  double    pressed[WBC_STEP_C], band = 0.0, edge = 0.0;
  int       done = 0, rounds = p->rounds, any = 0;

  /* a contact told to bear nothing is let go, as one the step would have pull */
  for (int i = 0; i < nc; i++)
  {
    pressed[i] = (double)(g->caps[i] > 0.0);
  }
  memset(pinned, 0, (size_t)nu * sizeof(double));
  memset(at, 0, (size_t)nu * sizeof(double));
  while (done < rounds)
  {
    if (nc)
    {
      for (int c = 0; c < nc; c++)
      {
        const double *Sc = S + 4 * nu * c;

        for (int j = 0; j < 4 * nu; j++)
        {
          pS[j] = pressed[c] * Sc[j];
        }
        wbc_matmul(&p->blas, VIEW(pS, 1, nu), VIEW(Sc, nu, 1), T, nu, 1, nu, 4, nu);
        wbc_matmul(&p->blas, VIEW(pS, 1, nu), VIEW(r + 4 * c, 1, 0), ht, 1, 0, nu, 4, 1);
        for (int j = 0; j < nu * nu; j++)
        {
          sumH[j] = (c ? sumH[j] : 0.0) + T[j];
        }
        for (int j = 0; j < nu; j++)
        {
          sumg[j] = (c ? sumg[j] : 0.0) + ht[j];
        }
      }
      for (int j = 0; j < nu * nu; j++)
      {
        Hr[j] = H0[j] + sumH[j];
      }
      for (int j = 0; j < nu; j++)
      {
        gr[j] = g0[j] + sumg[j];
      }
    }
    else
    {
      memcpy(Hr, H0, (size_t)(nu * nu) * sizeof(double));
      memcpy(gr, g0, (size_t)nu * sizeof(double));
    }
    /* a clipped force held by a row, a clipped moment its value, the rest solved round them */
    for (int j = 0; j < nu; j++)
    {
      const double s = ((p->w_pin * pinned[j]) * (1.0 - g->moment[j])) * g->part[j];

      hard[j] = pinned[j] * g->moment[j];
      soft[j] = s * s;
      free[j] = 1.0 - hard[j];
      held[j] = hard[j] * at[j];
    }
    for (int i = 0; i < nu; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        H[i * nu + j] = Hr[i * nu + j] + (i == j ? soft[i] : 0.0);
      }
    }
    for (int c = 0; c < nc; c++)
    {
      const int    ix = p->nu0 + 3 * c, ni = ix + 1, iz = ix + 2;
      const double on = (double)(pinned[ni] == 0.0 && at[ni] > 1e-9), den = npmax(at[ni], 1e-9);
      const double rx = on * (at[ix] / den), rz = on * (at[iz] / den);

      H[ix * nu + ni] += (-soft[ix]) * rx;
      H[ni * nu + ix] += (-soft[ix]) * rx;
      H[iz * nu + ni] += (-soft[iz]) * rz;
      H[ni * nu + iz] += (-soft[iz]) * rz;
      H[ni * nu + ni] += soft[ix] * (rx * rx) + soft[iz] * (rz * rz);
    }
    for (int i = 0; i < nu; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        K[i * nu + j] = H[i * nu + j] * (free[i] * free[j]) + (i == j ? hard[i] : 0.0);
      }
    }
    wbc_matmul(&p->blas, VIEW(H, nu, 1), VIEW(held, 1, 0), ht, 1, 0, nu, nu, 1);
    for (int j = 0; j < nu; j++)
    {
      const int    nj = p->nu0 + 3 * ((j - p->nu0) / 3) + 1;
      const double cone = (double)(j >= p->nu0 && j != nj && pinned[nj] == 0.0 && at[nj] > 1e-9);

      B[j] = free[j] * ((gr[j] + (soft[j] * at[j]) * (1.0 - cone)) - ht[j]) + hard[j] * at[j];
      B[nu + j] = g->up[j];
    }
    if (solved(p, K, nu, VIEW(B, 1, nu), 2, free, X, rest))
    {
      return 1;
    }
    {
      const double d1 = wbc_dot(&p->blas, g->up, 1, X, 2, nu), d2 = wbc_dot(&p->blas, g->up, 1, X + 1, 2, nu);
      const double by = (band * (edge - d1)) / ((d2 + 1.0) - band);
      double       ay, past, was;

      for (int j = 0; j < nu; j++)
      {
        un[j] = X[2 * j] + by * X[2 * j + 1];
      }
      /* her height's band: past an edge, held to it */
      ay = wbc_dot(&p->blas, g->lift, 1, un, 1, nu) / g->mass - p->g;
      past = pymin(pymax(ay, lo), hi) + p->g;
      was = band;
      band = pymax(band, g->grounded * (double)(fabs((past - p->g) - ay) > 1e-9));
      edge = edge + (1.0 - was) * (past - edge);
    }
    bounded(p, g, un, at, moved);
    if (nc)
    {
      pushed(p, g, at, bears, rest, kept);
      memcpy(at, rest, (size_t)nu * sizeof(double));
      for (int c = 0; c < nc; c++)
      {
        pressed[c] = pressed[c] * (double)(un[p->nu0 + 1 + 3 * c] >= 0.0);
      }
      for (int j = 0; j < nu; j++)
      {
        pinned[j] = npmax(pinned[j], npmax(moved[j], kept[j]));
      }
    }
    else
    {
      for (int j = 0; j < nu; j++)
      {
        pinned[j] = npmax(pinned[j], moved[j]);
      }
    }
    done += 1;
    {
      const double ay = wbc_dot(&p->blas, g->lift, 1, at, 1, nu) / g->mass - p->g, m = pymin(pymax(ay, lo), hi);

      if (done == rounds && rounds < p->rounds + p->past && !(g->strides != 0.0)
          && g->grounded * fabs(m - ay) > 1e-9)
      {
        band = 1.0;
        edge = m + p->g;
        rounds += 1;
      }
    }
    any = 0;
    for (int j = 0; j < nu; j++)
    {
      any |= pinned[j] != 0.0;
    }
    if ((!any && !(band != 0.0)) || (g->strides >= 1.0 && g->k3 == 0))
    {
      break;
    }
  }
  if (g->strides > 0.0 && (any || band != 0.0))
  {
    double *u0 = rest, *tk = u0 + nu, *ones = tk + nu, *more = ones + nu;

    for (int j = 0; j < nu; j++)
    {
      ones[j] = 1.0;
    }
    if (solved(p, Hr, nu, VIEW(gr, 1, 0), 1, ones, u0, more) || taken(p, g, u0, lo, hi, tk, more))
    {
      return 1;
    }
    for (int j = 0; j < nu; j++)
    {
      at[j] = at[j] + g->strides * (tk[j] - at[j]);
    }
  }
  return 0;
}
