/** wbc_chain_plan.c - the closed chains' plan (wbc_step.h): machine.stack.chain.planned, her tasks answered
    through her dynamics, the contacts held round by round - numpy's arithmetic and its OpenBLAS (wbc_blas.h)
    operation for operation; carved 64-byte aligned from the step's workspace. Portable C11, the host's alone;
    wbc_chain.c steps by it. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>
#include <string.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** `count` doubles from `*at` on, the next carve 64-byte aligned. */
static double *carve(double **at, int count)
{
  double *out = *at;

  *at = out + ((count + 7) & ~7);
  return out;
}

/** Contact `i`'s three rows of Jp (np_ wide), Jg (ng wide), Jv, dJv and fs into their framed copies, turned into its
    frame `q9` - its normal and two tangents -, so a sliding contact holds its normal and lets its tangents go: laid by
    her world's rows, a slide let go of world y, a seat's normal (machine.stack.chain_plan.frame_rows). */
static void frame_rows(const double *q9, int i, int np_, int ng, const double *Jp, const double *Jg, const double *Jv,
                       const double *dJv, const double *fs, double *JpF, double *JgF, double *JvF, double *dJvF,
                       double *fsF)
{
  for (int e = 0; e < 3; e++)
  {
    const double *qe = q9 + 3 * e;

    for (int a = 0; a < np_; a++)
    {
      JpF[(3 * i + e) * np_ + a] = (qe[0] * Jp[(3 * i) * np_ + a] + qe[1] * Jp[(3 * i + 1) * np_ + a]) + qe[2] * Jp[(3 * i + 2) * np_ + a];
    }
    for (int g = 0; g < ng; g++)
    {
      JgF[(3 * i + e) * ng + g] = (qe[0] * Jg[(3 * i) * ng + g] + qe[1] * Jg[(3 * i + 1) * ng + g]) + qe[2] * Jg[(3 * i + 2) * ng + g];
    }
    JvF[3 * i + e] = (qe[0] * Jv[3 * i] + qe[1] * Jv[3 * i + 1]) + qe[2] * Jv[3 * i + 2];
    dJvF[3 * i + e] = (qe[0] * dJv[3 * i] + qe[1] * dJv[3 * i + 1]) + qe[2] * dJv[3 * i + 2];
    fsF[3 * i + e] = (qe[0] * fs[3 * i] + qe[1] * fs[3 * i + 1]) + qe[2] * fs[3 * i + 2];
  }
}

int wbc_chain_planned(const wbc_step_t *p, int np_, int ng, int k3, const double *held, const double *Minv,
                      const double *Mpg, const double *rhs0, const double *Jp, const double *JpT, const double *Jg,
                      const double *Jv, const double *dJv, const double *Q, const double *RP, const double *RG,
                      const double *r, int m1, const double *want, const double *slide, double *x, double *zwarm,
                      double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         nd = p->nd, nh = ng - nd, nc = k3 / 3, mr = m1 + k3 + nc;
  const double      lam2 = p->chain_lam * p->chain_lam, task2 = p->chain_task_lam * p->chain_task_lam;
  const double      bleed = p->chain_bleed, mu = p->chain_mu, wf = p->chain_w_force;
  int               rows3[3 * WBC_STEP_C], yields[WBC_STEP_C], framed[WBC_STEP_C];
  double           *Fk = carve(w, k3), *Jap = carve(w, k3 * np_), *JapT = carve(w, np_ * k3), *Jag = carve(w, k3 * ng);
  double           *Jav = carve(w, k3), *Jad = carve(w, k3), *fa = carve(w, k3), *rhsk = carve(w, np_), *t = carve(w, np_);
  double           *MM = carve(w, np_ * ng), *W1 = carve(w, k3 * np_), *Dl = carve(w, k3 * k3), *Dinv = carve(w, k3 * k3);
  double           *bb0 = carve(w, k3), *WM = carve(w, k3 * ng), *Bg = carve(w, k3 * ng), *c0 = carve(w, k3), *X = carve(w, np_ * k3);
  double           *K = carve(w, np_ * k3), *p0 = carve(w, np_), *KB = carve(w, np_ * ng), *Pg = carve(w, np_ * ng);
  double           *Zx = carve(w, k3 * nd), *d2 = carve(w, k3), *zc = carve(w, k3);
  double           *G0 = carve(w, np_ * np_), *Yi = carve(w, np_ * k3), *Nint = carve(w, k3 * k3), *ZD = carve(w, k3 * nd), *zd = carve(w, k3);
  double           *R1x = carve(w, mr * nd), *t1 = carve(w, m1), *T = carve(w, m1 * nh), *u = carve(w, m1), *r1x = carve(w, mr);
  double           *R1xT = carve(w, nd * mr), *G1 = carve(w, nd * nd), *H1 = carve(w, nd * nd), *g1 = carve(w, nd);
  double           *x1 = carve(w, nd), *Y = carve(w, nd * nd), *N = carve(w, nd * nd), *R2n = carve(w, nd * nd);
  double           *R2nT = carve(w, nd * nd), *H2 = carve(w, nd * nd), *d = carve(w, nd), *g2 = carve(w, nd), *x2 = carve(w, nd);
  double           *Nx = carve(w, nd), *qg = carve(w, ng), *Bq = carve(w, k3), *za = carve(w, k3), *zfull = carve(w, k3);
  double           *Y3 = carve(w, 3 * nd), *q1 = carve(w, ng), *Bq1 = carve(w, k3), *za1 = carve(w, k3), *z1 = carve(w, k3);
  double           *fs = carve(w, k3), *fsF = carve(w, k3), *JpF = carve(w, k3 * np_), *JgF = carve(w, k3 * ng);
  double           *JvF = carve(w, k3), *dJvF = carve(w, k3);
  double           *rest = *w;

  for (int rw = 0; rw < k3; rw++)
  {
    fs[rw] = p->cf[rw / 3][rw % 3];
  }
  if (slide[0] != 0.0 || slide[2] != 0.0)
  {
    /* told a twist, the forces its rows aim at: the ratchet's split of what she feels (machine.stack.chain_plan.said) */
    double whole = 0.0, on = 0.0;

    for (int i = 0; i < nc; i++)
    {
      const double *ci = p->cf[i];
      const double  fe = sqrt(ci[0] * ci[0] + ci[1] * ci[1] + ci[2] * ci[2]);

      whole = whole + fe;
      on = on + fe * (double)(p->cbody[i] >= 0 && p->cpatch[i] < p->carpal);
    }
    for (int i = 0; i < nc; i++)
    {
      const int    sled = p->cbody[i] >= 0 && p->cpatch[i] < p->carpal;
      const double part = sled ? on : whole - on, share = sled ? 1.0 - p->chain_unweight : p->chain_unweight;

      if (part > 0.0)
      {
        for (int c = 0; c < 3; c++)
        {
          fs[3 * i + c] = p->cf[i][c] * (share * whole / part);
        }
      }
    }
  }
  /* the rows the plan holds, a sliding contact's in its frame (frame_rows): her world's as laid, until one slides */
  memcpy(fsF, fs, (size_t)k3 * sizeof(double));
  memcpy(JpF, Jp, (size_t)(k3 * np_) * sizeof(double));
  memcpy(JgF, Jg, (size_t)(k3 * ng) * sizeof(double));
  memcpy(JvF, Jv, (size_t)k3 * sizeof(double));
  memcpy(dJvF, dJv, (size_t)k3 * sizeof(double));
  for (int i = 0; i < nc; i++)
  {
    /* held by what its patch feels - the segments one surface bears, cpatch - not by its own point's share */
    double by = 0.0;
    int    foot;

    for (int j = 0; j < nc; j++)
    {
      const double *cf = p->cf[j];

      by = by + (double) (p->cpatch[j] == p->cpatch[i]) * sqrt(cf[0] * cf[0] + cf[1] * cf[1] + cf[2] * cf[2]);
    }
    rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = by >= p->chain_hold_n;
    /* a foot told free (machine.law.somatic: her heels stepping in turn) starts free, its points with it */
    foot = p->cpatch[i] >= 3 * p->carpal ? p->cpatch[i] / (3 * p->carpal) - 1 : (p->cbody[i] < 0 ? -1 - p->cbody[i] : -1);
    if (foot >= 0 && p->chain_free[foot])
    {
      rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
    }
    Fk[3 * i] = Fk[3 * i + 1] = Fk[3 * i + 2] = 0.0;
    yields[i] = framed[i] = 0;
    if (rows3[3 * i] && (slide[0] != 0.0 || slide[2] != 0.0) && p->cbody[i] >= 0 && p->cpatch[i] < p->carpal)
    {
      /* the twist told her: her sled - what she sits on, no hand or foot - slides along it on its normal, its friction
         mu what it feels (held, her pelvis was locked on its seat) */
      const double  fn = sqrt(fs[3 * i] * fs[3 * i] + fs[3 * i + 1] * fs[3 * i + 1] + fs[3 * i + 2] * fs[3 * i + 2]);
      const double  sn = (Q[9 * i] * slide[0] + Q[9 * i + 1] * slide[1]) + Q[9 * i + 2] * slide[2];
      double        t3[3], nt;

      for (int c = 0; c < 3; c++)
      {
        t3[c] = slide[c] - sn * Q[9 * i + c];
      }
      nt = sqrt(t3[0] * t3[0] + t3[1] * t3[1] + t3[2] * t3[2]);
      if (nt > 0.0)
      {
        for (int c = 0; c < 3; c++)
        {
          Fk[3 * i + c] = -(mu * fn / nt) * t3[c];
        }
        rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        framed[i] = 1;
        frame_rows(Q + 9 * i, i, np_, ng, Jp, Jg, Jv, dJv, fs, JpF, JgF, JvF, dJvF, fsF);
      }
    }
  }
  wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(Mpg, ng, 1), MM, ng, 1, np_, np_, ng);
  for (int rnd = 0; rnd < p->chain_rounds; rnd++)
  {
    int na = 0, changed = 0, mt, j = 0;

    for (int rw = 0; rw < k3; rw++)
    {
      na = na + rows3[rw];
    }
    for (int rw = 0; rw < k3; rw++)
    {
      if (!rows3[rw])
      {
        continue;
      }
      for (int a = 0; a < np_; a++)
      {
        Jap[j * np_ + a] = JpF[rw * np_ + a];
        JapT[a * na + j] = JpF[rw * np_ + a];
      }
      for (int g = 0; g < ng; g++)
      {
        Jag[j * ng + g] = JgF[rw * ng + g];
      }
      Jav[j] = JvF[rw];
      Jad[j] = dJvF[rw];
      fa[j] = fsF[rw];
      j++;
    }
    if (k3)
    {
      wbc_matmul(l, VIEW(JpT, k3, 1), VIEW(Fk, 1, 0), t, 1, 0, np_, k3, 1);
      for (int a = 0; a < np_; a++)
      {
        rhsk[a] = rhs0[a] + t[a];
      }
    }
    else
    {
      memcpy(rhsk, rhs0, (size_t)np_ * sizeof(double));
    }
    if (na)
    {
      wbc_matmul(l, VIEW(Jap, np_, 1), VIEW(Minv, np_, 1), W1, np_, 1, na, np_, np_);
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(JapT, na, 1), Dl, na, 1, na, np_, na);
      for (int i = 0; i < na; i++)
      {
        Dl[i * na + i] = Dl[i * na + i] + lam2;
      }
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(rhsk, 1, 0), bb0, 1, 0, na, np_, 1);
      for (int i = 0; i < na; i++)
      {
        bb0[i] = bb0[i] + Jad[i];
      }
      wbc_matmul(l, VIEW(W1, np_, 1), VIEW(Mpg, ng, 1), WM, ng, 1, na, np_, ng);
      for (int i = 0; i < na * ng; i++)
      {
        Bg[i] = Jag[i] - WM[i];
      }
      for (int i = 0; i < na; i++)
      {
        c0[i] = (-bleed) * Jav[i] - bb0[i];
      }
      if (wbc_inv(l, VIEW(Dl, na, 1), na, Dinv, rest))
      {
        return 1;
      }
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(JapT, na, 1), X, na, 1, np_, np_, na);
      wbc_matmul(l, VIEW(X, na, 1), VIEW(Dinv, na, 1), K, na, 1, np_, na, na);
      wbc_matmul(l, VIEW(K, na, 1), VIEW(c0, 1, 0), p0, 1, 0, np_, na, 1);
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(rhsk, 1, 0), t, 1, 0, np_, np_, 1);
      for (int a = 0; a < np_; a++)
      {
        p0[a] = p0[a] + t[a];
      }
      wbc_matmul(l, VIEW(K, na, 1), VIEW(Bg, ng, 1), KB, ng, 1, np_, na, ng);
      for (int i = 0; i < np_ * ng; i++)
      {
        Pg[i] = -(KB[i] + MM[i]);
      }
      wbc_matmul(l, VIEW(JapT, na, 1), VIEW(Jap, np_, 1), G0, np_, 1, np_, na, np_);
      for (int i = 0; i < np_; i++)
      {
        G0[i * np_ + i] = G0[i * np_ + i] + lam2;
      }
      if (wbc_solve(l, VIEW(G0, np_, 1), VIEW(JapT, na, 1), np_, na, Yi, rest))
      {
        return 1;
      }
      wbc_matmul(l, VIEW(Jap, np_, 1), VIEW(Yi, na, 1), Nint, na, 1, na, np_, na);
      for (int i = 0; i < na; i++)
      {
        for (int c = 0; c < na; c++)
        {
          Nint[i * na + c] = (i == c ? 1.0 : 0.0) - Nint[i * na + c];
        }
      }
      wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(Bg, ng, 1), ZD, nd, 1, na, na, nd);
      wbc_matmul(l, VIEW(Nint, na, 1), VIEW(ZD, nd, 1), Zx, nd, 1, na, na, nd);
      if (nh)
      {
        wbc_matmul(l, VIEW(Bg + nd, ng, 1), VIEW(held, 1, 0), d2, 1, 0, na, nh, 1);
      }
      else
      {
        memset(d2, 0, (size_t)na * sizeof(double));
      }
      for (int i = 0; i < na; i++)
      {
        d2[i] = c0[i] - d2[i];
      }
      wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(d2, 1, 0), zd, 1, 0, na, na, 1);
      for (int i = 0; i < na; i++)
      {
        zd[i] = zd[i] - fa[i];
      }
      wbc_matmul(l, VIEW(Nint, na, 1), VIEW(zd, 1, 0), zc, 1, 0, na, na, 1);
    }
    else
    {
      wbc_matmul(l, VIEW(Minv, np_, 1), VIEW(rhsk, 1, 0), p0, 1, 0, np_, np_, 1);
      for (int i = 0; i < np_ * ng; i++)
      {
        Pg[i] = -MM[i];
      }
    }
    if (m1)
    {
      wbc_matmul(l, VIEW(RP, np_, 1), VIEW(Pg, ng, 1), R1x, nd, 1, m1, np_, nd);
      for (int i = 0; i < m1; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          R1x[i * nd + c] = R1x[i * nd + c] + RG[i * ng + c];
        }
      }
      wbc_matmul(l, VIEW(RP, np_, 1), VIEW(p0, 1, 0), t1, 1, 0, m1, np_, 1);
      if (nh)
      {
        wbc_matmul(l, VIEW(RP, np_, 1), VIEW(Pg + nd, ng, 1), T, nh, 1, m1, np_, nh);
        for (int i = 0; i < m1; i++)
        {
          for (int c = 0; c < nh; c++)
          {
            T[i * nh + c] = T[i * nh + c] + RG[i * ng + nd + c];
          }
        }
        wbc_matmul(l, VIEW(T, nh, 1), VIEW(held, 1, 0), u, 1, 0, m1, nh, 1);
      }
      else
      {
        memset(u, 0, (size_t)m1 * sizeof(double));
      }
      for (int i = 0; i < m1; i++)
      {
        r1x[i] = (r[i] - t1[i]) - u[i];
      }
    }
    for (int i = 0; i < na; i++)
    {
      for (int c = 0; c < nd; c++)
      {
        R1x[(m1 + i) * nd + c] = wf * Zx[i * nd + c];
      }
      r1x[m1 + i] = wf * zc[i];
    }
    mt = m1 + na;
    /* one her posture alone pulled bears what it feels, a row beside the tasks:
       n Dinv_i Bg[:, driven] x = n (Fk_i + Dinv_i (c0 - Bg[:, held] held) - f_i) */
    for (int i = 0, at = 0; i < nc; at += rows3[3 * i] + rows3[3 * i + 1] + rows3[3 * i + 2], i++)
    {
      double e3[3];

      if (!yields[i] || !rows3[3 * i] || !rows3[3 * i + 1] || !rows3[3 * i + 2])
      {
        continue;
      }
      wbc_matmul(l, VIEW(Dinv + at * na, na, 1), VIEW(Bg, ng, 1), Y3, nd, 1, 3, na, nd);
      wbc_matmul(l, VIEW(Q + 9 * i, 0, 1), VIEW(Y3, nd, 1), R1x + mt * nd, 0, 1, 1, 3, nd);
      wbc_matmul(l, VIEW(Dinv + at * na, na, 1), VIEW(d2, 1, 0), e3, 1, 0, 3, na, 1);
      for (int c = 0; c < 3; c++)
      {
        e3[c] = (Fk[3 * i + c] + e3[c]) - fs[3 * i + c];
      }
      r1x[mt++] = wbc_dot(l, Q + 9 * i, 1, e3, 1, 3);
    }
    if (mt)
    {
      for (int i = 0; i < mt; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          R1xT[c * mt + i] = R1x[i * nd + c];
        }
      }
      wbc_matmul(l, VIEW(R1xT, mt, 1), VIEW(R1x, nd, 1), G1, nd, 1, nd, mt, nd);
      for (int i = 0; i < nd * nd; i++)
      {
        H1[i] = G1[i];
      }
      for (int i = 0; i < nd; i++)
      {
        H1[i * nd + i] = H1[i * nd + i] + task2;
      }
      wbc_matmul(l, VIEW(R1xT, mt, 1), VIEW(r1x, 1, 0), g1, 1, 0, nd, mt, 1);
      if (wbc_solve(l, VIEW(H1, nd, 1), VIEW(g1, 1, 0), nd, 1, x1, rest) || wbc_solve(l, VIEW(H1, nd, 1), VIEW(G1, nd, 1), nd, nd, Y, rest))
      {
        return 1;
      }
      for (int i = 0; i < nd; i++)
      {
        for (int c = 0; c < nd; c++)
        {
          N[i * nd + c] = (i == c ? 1.0 : 0.0) - Y[i * nd + c];
        }
      }
    }
    else
    {
      for (int i = 0; i < nd; i++)
      {
        x1[i] = 0.0;
        for (int c = 0; c < nd; c++)
        {
          N[i * nd + c] = i == c ? 1.0 : 0.0;
        }
      }
    }
    for (int i = 0; i < nd; i++)
    {
      for (int c = 0; c < nd; c++)
      {
        R2n[i * nd + c] = p->chain_w_posture * N[i * nd + c];
        R2nT[c * nd + i] = R2n[i * nd + c];
      }
    }
    wbc_matmul(l, VIEW(R2nT, nd, 1), VIEW(R2n, nd, 1), H2, nd, 1, nd, nd, nd);
    for (int i = 0; i < nd; i++)
    {
      H2[i * nd + i] = H2[i * nd + i] + lam2;
      d[i] = p->chain_w_posture * (want[i] - x1[i]);
    }
    wbc_matmul(l, VIEW(R2nT, nd, 1), VIEW(d, 1, 0), g2, 1, 0, nd, nd, 1);
    if (wbc_solve(l, VIEW(H2, nd, 1), VIEW(g2, 1, 0), nd, 1, x2, rest))
    {
      return 1;
    }
    wbc_matmul(l, VIEW(N, nd, 1), VIEW(x2, 1, 0), Nx, 1, 0, nd, nd, 1);
    for (int i = 0; i < nd; i++)
    {
      x[i] = x1[i] + Nx[i];
      qg[i] = x[i];
    }
    for (int i = 0; i < nh; i++)
    {
      qg[nd + i] = held[i];
    }
    if (!na)
    {
      memcpy(zwarm, Fk, (size_t)k3 * sizeof(double));
      break;
    }
    wbc_matmul(l, VIEW(Bg, ng, 1), VIEW(qg, 1, 0), Bq, 1, 0, na, ng, 1);
    for (int i = 0; i < na; i++)
    {
      Bq[i] = c0[i] - Bq[i];
    }
    wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(Bq, 1, 0), za, 1, 0, na, na, 1);
    /* the forces her tasks alone ask (x1, her posture apart): a pull her posture alone makes yields */
    for (int i = 0; i < ng; i++)
    {
      q1[i] = i < nd ? x1[i] : held[i - nd];
    }
    wbc_matmul(l, VIEW(Bg, ng, 1), VIEW(q1, 1, 0), Bq1, 1, 0, na, ng, 1);
    for (int i = 0; i < na; i++)
    {
      Bq1[i] = c0[i] - Bq1[i];
    }
    wbc_matmul(l, VIEW(Dinv, na, 1), VIEW(Bq1, 1, 0), za1, 1, 0, na, na, 1);
    j = 0;
    for (int i = 0; i < nc; i++)
    {
      const int at = j;

      for (int e = 0; e < 3; e++)
      {
        const int rw = 3 * i + e;

        z1[rw] = rows3[rw] ? Fk[rw] + za1[j] : Fk[rw];
        zfull[rw] = rows3[rw] ? Fk[rw] + za[j++] : Fk[rw];
      }
      if (framed[i])
      {
        /* its held rows in its frame: their forces back in the world (machine.stack.chain_plan.unframed) */
        double rf[3], r1[3];
        int    h = at;

        for (int e = 0; e < 3; e++)
        {
          rf[e] = rows3[3 * i + e] ? za[h] : 0.0;
          r1[e] = rows3[3 * i + e] ? za1[h++] : 0.0;
        }
        for (int c = 0; c < 3; c++)
        {
          const double qa = Q[9 * i + c], qb = Q[9 * i + 3 + c], qc = Q[9 * i + 6 + c];

          zfull[3 * i + c] = Fk[3 * i + c] + ((qa * rf[0] + qb * rf[1]) + qc * rf[2]);
          z1[3 * i + c] = Fk[3 * i + c] + ((qa * r1[0] + qb * r1[1]) + qc * r1[2]);
        }
      }
      for (int e = 0; e < 3; e++)
      {
        zwarm[3 * i + e] = zfull[3 * i + e];
      }
    }
    for (int i = 0; i < nc; i++)
    {
      const double *cf = p->cf[i];
      const double  felt = sqrt(cf[0] * cf[0] + cf[1] * cf[1] + cf[2] * cf[2]);
      const int     pushes = p->cbody[i] < 0 || (p->cpatch[i] >= p->carpal && p->cpatch[i] < 3 * p->carpal);
      double        loc[3];

      if (!rows3[3 * i])
      {
        continue;
      }
      for (int a = 0; a < 3; a++)
      {
        loc[a] = (Q[9 * i + 3 * a] * zfull[3 * i] + Q[9 * i + 3 * a + 1] * zfull[3 * i + 1]) + Q[9 * i + 3 * a + 2] * zfull[3 * i + 2];
      }
      if (loc[0] < 0.0 && !yields[i] && rows3[3 * i + 1] && rows3[3 * i + 2] && p->cbody[i] >= 0
          && p->cpatch[i] >= p->carpal && wbc_dot(l, Q + 9 * i, 1, z1 + 3 * i, 1, 3) >= 0.0)
      {
        yields[i] = changed = 1;
      }
      else if (loc[0] < 0.0)
      {
        rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        Fk[3 * i] = Fk[3 * i + 1] = Fk[3 * i + 2] = 0.0;
        changed = 1;
      }
      else if (pushes && loc[0] > felt + p->grow_n)
      {
        /* a hand's or a sole's patch asked past what it feels by grow_n (machine.stack.rounds.GROW_N): held at that
           cap, its friction boxed by it; what she sits or lies on is not */
        const double cap = felt + p->grow_n, lim = mu * cap, tb1 = npclip(loc[1], -lim, lim), tb2 = npclip(loc[2], -lim, lim);

        for (int c = 0; c < 3; c++)
        {
          Fk[3 * i + c] = (Q[9 * i + c] * cap + Q[9 * i + 3 + c] * tb1) + Q[9 * i + 6 + c] * tb2;
        }
        rows3[3 * i] = rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        changed = 1;
      }
      else if (rows3[3 * i + 1] && npmax(fabs(loc[1]), fabs(loc[2])) > mu * loc[0])
      {
        const double lim = mu * loc[0], tb1 = npclip(loc[1], -lim, lim), tb2 = npclip(loc[2], -lim, lim);

        for (int c = 0; c < 3; c++)
        {
          Fk[3 * i + c] = Q[9 * i + 3 + c] * tb1 + Q[9 * i + 6 + c] * tb2;
        }
        rows3[3 * i + 1] = rows3[3 * i + 2] = 0;
        changed = 1;
      }
    }
    if (!changed)
    {
      break;
    }
  }
  return 0;
}
