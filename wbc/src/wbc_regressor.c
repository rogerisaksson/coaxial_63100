/** wbc_regressor.c - her Newton-Euler regressor on the host (wbc_regressor.h): machine/regressor.py,
    its steps by their names, numpy's arithmetic operation for operation - its einsums' orders
    (wbc_np.h's measure: 'nij,nj->ni' SSE's two lanes, the rest a plain sum), its crosses' formula.
    Portable C11, the host's alone. */
#include "wbc_regressor.h"
#include "wbc_np.h"

#include <stddef.h>
#include <string.h>

/** np.cross of 3-vectors: a1 b2 - a2 b1, a2 b0 - a0 b2, a0 b1 - a1 b0. */
static void cross(const double a[3], const double b[3], double out[3])
{
  const double c0 = a[1] * b[2] - a[2] * b[1], c1 = a[2] * b[0] - a[0] * b[2], c2 = a[0] * b[1] - a[1] * b[0];

  out[0] = c0;
  out[1] = c1;
  out[2] = c2;
}

/** einsum 'nji,nj->ni' of one state: R's columns against v, a plain sum. */
static void tdot(const double R[9], const double v[3], double out[3])
{
  for (int i = 0; i < 3; i++)
  {
    out[i] = ((0.0 + R[i] * v[0]) + R[3 + i] * v[1]) + R[6 + i] * v[2];
  }
}

/** _tensor: L(w) (3, 6), L(w) [Ixx Ixy Ixz Iyy Iyz Izz] = I w. */
static void tensor(const double w[3], double L[3][6])
{
  memset(L, 0, 3 * sizeof L[0]);
  L[0][0] = w[0], L[0][1] = w[1], L[0][2] = w[2];
  L[1][1] = w[0], L[1][3] = w[1], L[1][4] = w[2];
  L[2][2] = w[0], L[2][4] = w[1], L[2][5] = w[2];
}

void wbc_regressor(const wbc_regressor_t *r, int n, const double *xpos, const double *xmat, const double *axis,
                   const double *anchor, const double *qvel, const double *qacc, const double *a0,
                   const double *wacc, double *Y, double *work)
{
  const int nb = r->nb, nh = r->nh, P = r->p, nv = r->nv, rows = nh + 6;
  double   *V = work, *A = V + (ptrdiff_t)6 * nb, *su = A + (ptrdiff_t)6 * nb;

  memset(Y, 0, (size_t)n * (size_t)rows * (size_t)P * sizeof(double));
  for (int s = 0; s < n; s++)
  {
    const double *R = xmat + (ptrdiff_t)9 * nb * s, *p = xpos + (ptrdiff_t)3 * nb * s, *qv = qvel + (ptrdiff_t)nv * s;
    const double *qa = qacc + (ptrdiff_t)nv * s, *ax = axis + (ptrdiff_t)3 * nh * s, *an = anchor + (ptrdiff_t)3 * nh * s;
    double       *Ys = Y + (ptrdiff_t)rows * P * s, w[3], al[3], c1[3], c2[3];

    /* w, al: einsum 'nij,nj->ni', SSE's two lanes; Plucker at the world's origin: (w, u) a body's
       point there and (al, a) its rate, V and A; each hinge's axis (sw, su) */
    for (int i = 0; i < 3; i++)
    {
      w[i] = 0.0 + (((0.0 + R[3 * i] * qv[3]) + R[3 * i + 2] * qv[5]) + (0.0 + R[3 * i + 1] * qv[4]));
      al[i] = 0.0 + (((0.0 + R[3 * i] * qa[3]) + R[3 * i + 2] * qa[5]) + (0.0 + R[3 * i + 1] * qa[4]));
    }
    cross(w, p, c1);
    for (int i = 0; i < 3; i++)
    {
      V[i] = w[i];
      V[3 + i] = qv[i] - c1[i];
    }
    cross(w, qv, c1);
    cross(al, p, c2);
    for (int i = 0; i < 3; i++)
    {
      A[i] = al[i];
      A[3 + i] = (a0[3 * s + i] - c1[i]) - c2[i];
    }
    for (int h = 0; h < nh; h++)
    {
      cross(an + 3 * h, ax + 3 * h, su + 3 * h);
    }
    for (int k = 1; k < nb; k++)
    {
      const double *pv = V + 6 * r->parent[k], *pa = A + 6 * r->parent[k];
      double        wv[3], uv[3], wa[3], ua[3];

      memcpy(wv, pv, sizeof wv);
      memcpy(uv, pv + 3, sizeof uv);
      memcpy(wa, pa, sizeof wa);
      memcpy(ua, pa + 3, sizeof ua);
      for (int h = r->span[2 * k]; h < r->span[2 * k + 1]; h++)
      {
        const int     dof = r->hinge[h];
        const double  qd = qv[dof], qdd = qa[dof], *sw = ax + 3 * h, *u = su + 3 * h;
        double        dsw[3], d1[3], d2[3];

        cross(wv, sw, dsw);
        cross(wv, u, d1);
        cross(uv, sw, d2);
        for (int i = 0; i < 3; i++)
        {
          const double dsu = d1[i] + d2[i];

          wa[i] = (wa[i] + sw[i] * qdd) + dsw[i] * qd;
          ua[i] = (ua[i] + u[i] * qdd) + dsu * qd;
        }
        for (int i = 0; i < 3; i++)
        {
          wv[i] = wv[i] + sw[i] * qd;
          uv[i] = uv[i] + u[i] * qd;
        }
      }
      memcpy(V + 6 * k, wv, sizeof wv);
      memcpy(V + 6 * k + 3, uv, sizeof uv);
      memcpy(A + 6 * k, wa, sizeof wa);
      memcpy(A + 6 * k + 3, ua, sizeof ua);
    }
    for (int k = 0; k < nb; k++)
    {
      const double *Rb = R + 9 * k, *pb = p + 3 * k, *wv = V + 6 * k, *uv = wv + 3, *wa = A + 6 * k, *ua = wa + 3;
      double        om[3], alb[3], ac[3], t1[3], t2[3], t3[3], f[3][10], nn[3][10], fw[3][10], nw[3][10];

      tdot(Rb, wv, om);
      tdot(Rb, wa, alb);
      cross(wa, pb, t1);
      cross(wv, pb, t2);
      for (int i = 0; i < 3; i++)
      {
        t2[i] = uv[i] + t2[i];
      }
      cross(wv, t2, t3);
      for (int i = 0; i < 3; i++)
      {
        t1[i] = (ua[i] + t1[i]) + t3[i];
      }
      tdot(Rb, t1, ac);
      memset(f, 0, sizeof f);
      memset(nn, 0, sizeof nn);
      for (int i = 0; i < 3; i++)
      {
        f[i][0] = ac[i];
      }
      for (int a = 0; a < 3; a++)
      {
        const double e[3] = {(double)(a == 0), (double)(a == 1), (double)(a == 2)};
        double       x1[3], x2[3], x3[3], x4[3];

        cross(alb, e, x1);
        cross(om, e, x2);
        cross(om, x2, x3);
        cross(e, ac, x4);
        for (int i = 0; i < 3; i++)
        {
          f[i][1 + a] = x1[i] + x3[i];
          nn[i][1 + a] = x4[i];
        }
      }
      /* nn[:, :, 4:10] = _tensor(alb) + np.cross(om, _tensor(om), axis=1) */
      {
        double Lt[3][6], Lo[3][6];

        tensor(alb, Lt);
        tensor(om, Lo);
        for (int c = 0; c < 6; c++)
        {
          const double col[3] = {Lo[0][c], Lo[1][c], Lo[2][c]};
          double       x[3];

          cross(om, col, x);
          for (int i = 0; i < 3; i++)
          {
            nn[i][4 + c] = Lt[i][c] + x[i];
          }
        }
      }
      /* to the world, about its origin (einsum 'nij,njc->nic', a plain sum over j); each hinge between
         the body and her pelvis takes its row */
      for (int i = 0; i < 3; i++)
      {
        for (int c = 0; c < 10; c++)
        {
          fw[i][c] = ((0.0 + Rb[3 * i] * f[0][c]) + Rb[3 * i + 1] * f[1][c]) + Rb[3 * i + 2] * f[2][c];
          nw[i][c] = ((0.0 + Rb[3 * i] * nn[0][c]) + Rb[3 * i + 1] * nn[1][c]) + Rb[3 * i + 2] * nn[2][c];
        }
      }
      for (int c = 0; c < 10; c++)
      {
        const double col[3] = {fw[0][c], fw[1][c], fw[2][c]};
        double       x[3], y[3], m[3];

        cross(pb, col, x);
        for (int i = 0; i < 3; i++)
        {
          nw[i][c] = nw[i][c] + x[i];
        }
        cross(p, col, y);
        for (int i = 0; i < 3; i++)
        {
          m[i] = nw[i][c] - y[i];
        }
        for (int i = 0; i < 3; i++)
        {
          Ys[(ptrdiff_t)(nh + i) * P + 10 * k + c] = fw[i][c];
          Ys[(ptrdiff_t)(nh + 3 + i) * P + 10 * k + c] = ((0.0 + R[i] * m[0]) + R[3 + i] * m[1]) + R[6 + i] * m[2];
        }
        for (int h = 0; h < nh; h++)
        {
          if (r->above[nh * k + h])
          {
            const double *sw = ax + 3 * h, *u = su + 3 * h;
            const double  a1 = ((0.0 + sw[0] * nw[0][c]) + sw[1] * nw[1][c]) + sw[2] * nw[2][c];
            const double  a2 = ((0.0 + u[0] * fw[0][c]) + u[1] * fw[1][c]) + u[2] * fw[2][c];

            Ys[(ptrdiff_t)h * P + 10 * k + c] = a1 + a2;
          }
        }
      }
    }
    for (int i = 0; i < r->nd; i++)
    {
      const int    h = r->driven[i], dof = r->dof[i];
      const double q = qv[dof];

      Ys[(ptrdiff_t)h * P + r->drive + 3 * i] = wacc ? wacc[(ptrdiff_t)r->nd * s + i] : qa[dof];
      Ys[(ptrdiff_t)h * P + r->drive + 3 * i + 1] = q;
      Ys[(ptrdiff_t)h * P + r->drive + 3 * i + 2] = npsign(q);
    }
  }
}
