/** wbc_rounds.c - the step's solve (wbc_step.h): machine.stack.wrench's _bounds and _within, machine.stack.rounds'
    faced and taken, numpy's arithmetic and numpy's own OpenBLAS (wbc_blas.h) operation for operation.
    Portable C11, the host's alone. */
#include "wbc_np.h"
#include "wbc_qp.h"
#include "wbc_step.h"
#include "wbc_libm_arc.h"

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
  const double m = (x != x) ? x : (x > lo ? x : lo);

  return (m != m) ? m : (m < hi ? m : hi);
}

/** machine.stack.wrench.faced: each standing sole's wrench in its cone, the faces it was past, each
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

/** machine.stack.wrench._solved: K^-1 rhs (rhs a view, nu by m), K ridged on its `free` unknowns where
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

/** machine.stack.wrench._taken: striding, what the floor cannot give the robot's inertia takes - `u0`
    brought to the faces its soles' wrenches are past by the least change in its kinetic energy's
    metric. */
static int taken(const wbc_step_t *p, const wbc_geo_t *g, const double *restrict u0, double lo, double hi,
                 double *restrict at, double *work)
{
  const int        nu = g->nu;
  double *restrict Gi = work, *restrict F = Gi + nu * nu, *restrict N = F + 13 * nu, *restrict C = N + 13 * nu;
  double *restrict GC = C + 13 * nu, *restrict CG = GC + 13 * nu, *restrict A = CG + 13 * nu, *restrict u = A + 169;
  double *restrict cu = u + nu, *restrict x = cu + 13, *restrict t = x + 13, *restrict rest = t + nu;
  double           met[13] = {0.0}, share[13] = {0.0}, level[13] = {0.0}, out[12], to[12];

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

/** One row of A u <= b laid zeroed, its right-hand side `c`: the row. */
static double *row(double *A, double *b, int *m, int nu, double c)
{
  double *a = A + (*m) * nu;

  memset(a, 0, (size_t)nu * sizeof(double));
  b[*m] = c;
  *m += 1;
  return a;
}

/** machine.stack.wrench._bounds: each standing sole's bounds and each contact's as rows A u <= b, in its order; their
    count. */
static int bounds(const wbc_step_t *p, const wbc_geo_t *g, const double *bears, const double *pressed, double *A,
                  double *b)
{
  const int    nu = g->nu, nc = g->k3 / 3;
  const double rh = sqrt(0.5), c8 = sqrt(0.5 + 0.5 * rh), s8 = sqrt(0.5 - 0.5 * rh), cos16 = sqrt(0.5 + 0.5 * c8);
  const double cone[16][2] = {{1.0, 0.0}, {c8, s8},     {rh, rh},     {s8, c8},     {0.0, 1.0},  {-s8, c8},
                              {-rh, rh},  {-c8, s8},    {-1.0, 0.0},  {-c8, -s8},   {-rh, -rh},  {-s8, -c8},
                              {0.0, -1.0}, {s8, -c8},   {rh, -rh},    {c8, -s8}};
  int          m = 0;
  double      *a;

  for (int k = 0; k < 2; k++)
  {
    if (g->stance[k] > 0.0)
    {
      const int     fy = 6 * k + 4;
      const double  e = g->edges[k], muk = g->mu[k], *R = g->frames[k];
      const double  hi0 = p->room[1] * (1.0 - 2.0 * pymax(e, 0.0)), lo0 = (-p->room[1]) * (1.0 - 2.0 * pymax(-e, 0.0));
      const double  cj[3] = {0.0, muk * p->yaw_m, p->room[0]};

      a = row(A, b, &m, nu, 0.0);
      a[fy] = -1.0;
      for (int f = 6 * k + 3; f <= 6 * k + 5; f += 2)
      {
        a = row(A, b, &m, nu, 0.0);
        a[f] = 1.0;
        a[fy] = -muk;
        a = row(A, b, &m, nu, 0.0);
        a[f] = -1.0;
        a[fy] = -muk;
      }
      a = row(A, b, &m, nu, 0.0);
      for (int i = 0; i < 3; i++)
      {
        a[6 * k + i] = R[3 * i];
      }
      a[fy] = -hi0;
      a = row(A, b, &m, nu, 0.0);
      for (int i = 0; i < 3; i++)
      {
        a[6 * k + i] = -R[3 * i];
      }
      a[fy] = lo0;
      for (int j = 1; j < 3; j++)
      {
        a = row(A, b, &m, nu, 0.0);
        for (int i = 0; i < 3; i++)
        {
          a[6 * k + i] = R[3 * i + j];
        }
        a[fy] = -cj[j];
        a = row(A, b, &m, nu, 0.0);
        for (int i = 0; i < 3; i++)
        {
          a[6 * k + i] = -R[3 * i + j];
        }
        a[fy] = -cj[j];
      }
    }
  }
  for (int i = 0; i < nc; i++)
  {
    const int    fy = p->nu0 + 3 * i + 1, f[2] = {p->nu0 + 3 * i, p->nu0 + 3 * i + 2};
    const double sliding = g->slips[3 * i], sv[2] = {g->slips[3 * i + 1], g->slips[3 * i + 2]};
    const double den = pymax(wbc_hypot(sv[0], sv[1]), 1e-12);
    const double top = pressed[i] != 0.0 ? pymax(pymin(bears[i] + p->grow_n, g->caps[i]) * g->give, 0.0) : 0.0;
    /* sliding, its push no less than grow_n under what it bears: its kinetic friction given, not shed in a pass */
    const double low =
      (pressed[i] != 0.0 && sliding > 0.0) ? pymax(pymin(bears[i] - p->grow_n, g->caps[i]) * g->give, 0.0) : 0.0;

    a = row(A, b, &m, nu, 0.0 - low);
    a[fy] = -1.0;
    a = row(A, b, &m, nu, top);
    a[fy] = 1.0;
    for (int t = 0; sliding > 0.0 && t < 2; t++)
    {
      const double c = (p->mu * sv[t]) / den;

      a = row(A, b, &m, nu, 0.0);
      a[f[t]] = 1.0;
      a[fy] = c;
      a = row(A, b, &m, nu, 0.0);
      a[f[t]] = -1.0;
      a[fy] = -c;
    }
    /* its friction within the sixteen-sided pyramid inscribed in its round cone (machine.stack.rounds.CONE) */
    for (int q = 0; !(sliding > 0.0) && q < 16; q++)
    {
      a = row(A, b, &m, nu, 0.0);
      a[f[0]] = cone[q][0];
      a[f[1]] = cone[q][1];
      a[fy] = -(p->mu * cos16);
    }
  }
  return m;
}

/** The least squares within the rows (m of them, A u <= b) by the stack's dual active set, the height's band - the
    last two - let go where nothing meets them: 0, 1 where none meets it, 2 where numpy would raise. */
static int answered(wbc_step_t *p, const double *H, const double *ng, const double *A, const double *b, int nu, int m,
                    int banded, double *at, int *act, double *mult, double *work)
{
  int nact = 0, got = wbc_qp_solve(&p->blas, H, ng, A, b, nu, m, act, 0, 0, at, act, &nact, mult, work);

  if (got == 1 && banded)
  {
    got = wbc_qp_solve(&p->blas, H, ng, A, b, nu, m - 2, act, 0, 0, at, act, &nact, mult, work);
  }
  return (got == 0 && nact > 0) ? -1 : got;
}

int wbc_within(wbc_step_t *p, const wbc_geo_t *g, const double *H0, const double *g0, const double *S,
               const double *r, const double *bears, double lo, double hi, double *at, double *first, double *work)
{
  const int nu = g->nu, nc = g->k3 / 3;
  double   *Hr = work, *gr = Hr + nu * nu, *K = gr + nu, *A = K + nu * nu, *b = A + WBC_QP_M * nu, *ng = b + WBC_QP_M;
  double   *ones = ng + nu, *mult = ones + nu, *T = mult + (nu + WBC_QP_M + 1), *ht = T + nu * nu, *pS = ht + nu;
  double   *sumH = pS + 4 * nu, *sumg = sumH + nu * nu, *rest = sumg + nu;
  double    pressed[WBC_STEP_C];
  int       act[WBC_QP_M], m, got;

  /* a contact told to bear nothing is let go */
  for (int i = 0; i < nc; i++)
  {
    pressed[i] = (double)(g->caps[i] > 0.0);
  }
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
  for (int j = 0; j < nu; j++)
  {
    ones[j] = 1.0;
  }
  /* the first answer, the floor unbounded (told where asked); of the acceleration it gives the robot, at w_keep
     (machine.stack.within.W_KEEP), a sliding contact's push kept near what it bears and two standing soles' inner
     wrench - what their net says nothing of, past its least in the part metric - least */
  {
    double *own = first != NULL ? first : sumg, *kk = ht, *Hk = sumH;

    memcpy(K, Hr, (size_t)(nu * nu) * sizeof(double));
    if (solved(p, K, nu, VIEW(gr, 1, 0), 1, ones, own, rest))
    {
      return 1;
    }
    for (int j = 0; j < nu; j++)
    {
      const int    c = (j - (nu - g->k3)) / 3, axis = (j - (nu - g->k3)) - 3 * c;
      const double each = j < nu - g->k3 ? 0.0 : (double)(axis == 1) * (double)(g->slips[3 * c] > 0.0);
      const double kp = ((p->w_keep * p->g) * each) * g->part[j];
      const double to = j < nu - g->k3 ? 0.0 : (double)(axis == 1) * bears[c];

      kk[j] = kp * kp;
      ng[j] = -(gr[j] + kk[j] * to);
    }
    for (int i = 0; i < nu; i++)
    {
      for (int j = 0; j < nu; j++)
      {
        Hk[i * nu + j] = Hr[i * nu + j] + (i == j ? kk[i] : 0.0);
      }
    }
    if (g->inner)
    {
      double *net = rest, *wide = net + 72, *M = wide + 72, *X = M + 36, *least = X + 72;
      double *in = least + 144, *inner = in + 144, *more = inner + 144, w2[12];

      for (int j = 0; j < 12; j++)
      {
        w2[j] = g->part[j] * g->part[j];
      }
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          net[i * 12 + j] = g->about[i * nu + j];
          net[(3 + i) * 12 + j] = g->sumf[i * nu + j];
        }
      }
      for (int i = 0; i < 6; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          wide[i * 12 + j] = net[i * 12 + j] / w2[j];
        }
      }
      wbc_matmul(&p->blas, VIEW(wide, 12, 1), VIEW(net, 1, 12), M, 6, 1, 6, 12, 6);
      if (wbc_solve(&p->blas, VIEW(M, 6, 1), VIEW(net, 12, 1), 6, 12, X, more))
      {
        return 1;
      }
      /* numpy's (net.T / w2[:, None]) is Fortran-ordered: read off wide transposed, as numpy's matmul hands it to
         dgemm - laid row by row, an AVX-512 OpenBLAS's small kernels summed it in another order */
      wbc_matmul(&p->blas, VIEW(wide, 1, 12), VIEW(X, 12, 1), least, 12, 1, 12, 6, 12);
      for (int i = 0; i < 12; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          in[i * 12 + j] = (p->w_keep * p->g) * (g->part[i] * ((double)(i == j) - least[i * 12 + j]));
        }
      }
      wbc_matmul(&p->blas, VIEW(in, 1, 12), VIEW(in, 12, 1), inner, 12, 1, 12, 12, 12);
      for (int i = 0; i < 12; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          Hk[i * nu + j] = Hk[i * nu + j] + inner[i * 12 + j];
        }
      }
    }
  }
  m = bounds(p, g, bears, pressed, A, b);
  if (g->grounded > 0.0)
  {
    double *up = row(A, b, &m, nu, hi + p->g), *down = row(A, b, &m, nu, -(lo + p->g));

    for (int j = 0; j < nu; j++)
    {
      up[j] = g->lift[j] / g->mass;
      down[j] = (-g->lift[j]) / g->mass;
    }
  }
  got = answered(p, sumH, ng, A, b, nu, m, g->grounded > 0.0, at, act, mult, rest);
  /* no inverse, or none found where nothing pushing meets every bound - two bearing points 16 mm apart, their split
     said by no row -: ridged as `solved` ridges */
  if (got > 0)
  {
    double most = sumH[0];

    memcpy(K, sumH, (size_t)(nu * nu) * sizeof(double));
    for (int i = 1; i < nu; i++)
    {
      most = npmax(most, K[i * nu + i]);
    }
    for (int i = 0; i < nu; i++)
    {
      K[i * nu + i] = K[i * nu + i] + (ones[i] * p->ridge) * most;
    }
    got = answered(p, K, ng, A, b, nu, m, g->grounded > 0.0, at, act, mult, rest);
  }
  if (got > 0)
  {
    return 1;
  }
  if (g->strides > 0.0 && got < 0)
  {
    double *u0 = rest, *tk = u0 + nu, *more = tk + nu;

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
