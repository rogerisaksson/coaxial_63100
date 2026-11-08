/** wbc_chain_qp.c - machine.stack.chain_qp (wbc_step.h): her step in closed chains as one QP - the unknowns u =
    (x, zeta), level 1's affine maps through her dynamics, the answer's parts -, its rows and levels in
    wbc_chain_rows.c; numpy's arithmetic and its OpenBLAS (wbc_blas.h) operation for operation, every product in
    python's order and layout; carved 64-byte aligned from the step's workspace. Portable C11, the host's alone. */
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

/** numpy's `a @ b`, a (m, n) and b (n, p) C-ordered, into o (m, p). */
static void mm(const wbc_blas_t *l, const double *a, const double *b, double *o, int m, int n, int p)
{
  wbc_matmul(l, VIEW(a, n, 1), VIEW(b, p, 1), o, p, 1, m, n, p);
}

/** numpy's `a @ x`, a (m, n) C-ordered, into y (m). */
static void mv(const wbc_blas_t *l, const double *a, const double *x, double *y, int m, int n)
{
  wbc_matmul(l, VIEW(a, n, 1), VIEW(x, 1, 0), y, 1, 0, m, n, 1);
}

/** Columns `from` to `to` of `a` (m rows of `cols`) as a C-ordered copy (numpy's ascontiguousarray of a slice). */
static void sliced(const double *a, int m, int cols, int from, int to, double *out)
{
  for (int i = 0; i < m; i++)
  {
    for (int j = from; j < to; j++)
    {
      out[i * (to - from) + (j - from)] = a[i * cols + j];
    }
  }
}

/** machine.stack.chain_qp.framed: `M` (3k rows of `cols`) with each contact's three rows turned into its frame. */
static void framed(const double *Q, int k, const double *M, int cols, double *out)
{
  for (int i = 0; i < k; i++)
  {
    for (int a = 0; a < 3; a++)
    {
      for (int c = 0; c < cols; c++)
      {
        out[(3 * i + a) * cols + c] = (Q[9 * i + 3 * a] * M[(3 * i) * cols + c] + Q[9 * i + 3 * a + 1] * M[(3 * i + 1) * cols + c])
                                      + Q[9 * i + 3 * a + 2] * M[(3 * i + 2) * cols + c];
      }
    }
  }
}

/** machine.stack.chain_qp.freed: whether contact `i` is on a foot told free. */
static int freed(const wbc_step_t *p, int i)
{
  const int let = 3 * p->carpal, patch = p->cpatch[i], body = p->cbody[i];
  const int foot = patch >= let ? patch / let - 1 : (body < 0 ? -1 - body : -1);

  return foot >= 0 && p->chain_free[foot];
}

/** machine.stack.chain_qp.keyed and the let set's reading: each contact's key (body, patch, ordinal among those of
    both) and whether the plan let it go last pass (p->chain_let). */
static void keyed(const wbc_step_t *p, int k, int key[][3], int *let)
{
  for (int i = 0; i < k; i++)
  {
    key[i][0] = p->cbody[i];
    key[i][1] = p->cpatch[i];
    key[i][2] = 0;
    for (int j = 0; j < i; j++)
    {
      key[i][2] += (p->cbody[j] == p->cbody[i] && p->cpatch[j] == p->cpatch[i]);
    }
    let[i] = 0;
    for (int j = 0; j < p->chain_nlet; j++)
    {
      let[i] = let[i] || (p->chain_let[j][0] == key[i][0] && p->chain_let[j][1] == key[i][1] && p->chain_let[j][2] == key[i][2]);
    }
  }
}

/** machine.stack.chain_qp.letting: the contacts let go for the next pass into p->chain_let - a held one whose row z_n
    >= 0 (`zrow`) level 1 held, a let one (`let`) the plan still lifts (al_n > 0) -; complementarity pass by pass. */
static void letting(wbc_step_t *p, int k, int key[][3], const int *holds, const int *let, const int *zrow, const double *al)
{
  p->chain_nlet = 0;
  for (int i = 0; i < k; i++)
  {
    int active = 0;

    for (int j = 0; j < p->chain_nheld[0]; j++)
    {
      active = active || p->chain_held[0][j] == zrow[i];
    }
    if ((holds[i] && active) || (let[i] && al[3 * i] > 0.0))
    {
      memcpy(p->chain_let[p->chain_nlet], key[i], sizeof(key[i]));
      p->chain_nlet++;
    }
  }
}

/** machine.stack.chain_qp.unknowns: each contact's width in zeta, S (3k x nz) zeta -> z in frames, its friction,
    whether it bears (its patch hold-n, its foot not told free) and whether it is her sled; nz returned. */
static int unknowns(const wbc_step_t *p, const double *Q, const double *slide, int *width, double *S, double *mu, int *bears,
                    int *sleds)
{
  const int    k = p->nc;
  const double rh = sqrt(0.5);
  int          nz = 0;
  double       cols[WBC_STEP_C][3];

  for (int i = 0; i < k; i++)
  {
    const int sled = p->cbody[i] >= 0 && p->cpatch[i] < p->carpal;
    double    by = 0.0;

    sleds[i] = sled;

    for (int j = 0; j < k; j++)
    {
      const double *cj = p->cf[j];

      by = by + (double)(p->cpatch[j] == p->cpatch[i]) * sqrt(cj[0] * cj[0] + cj[1] * cj[1] + cj[2] * cj[2]);
    }
    bears[i] = by >= p->chain_hold_n && !freed(p, i);
    mu[i] = sled ? p->chain_mu : p->chain_anchor_mu;
    width[i] = 3;
    /* pinned against the way told where its normal bears against gravity (up within 45 deg; a back against a
       backrest braces, no pin) */
    if (sled && Q[9 * i + 1] >= rh && (slide[0] != 0.0 || slide[2] != 0.0))
    {
      const double *q = Q + 9 * i;
      const double  sn = (q[0] * slide[0] + q[1] * slide[1]) + q[2] * slide[2];
      double        t[3], nt;

      for (int c = 0; c < 3; c++)
      {
        t[c] = slide[c] - sn * q[c];
      }
      nt = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
      if (nt > 0.0)
      {
        const double st1 = ((q[3] * t[0] + q[4] * t[1]) + q[5] * t[2]) / nt, st2 = ((q[6] * t[0] + q[7] * t[1]) + q[8] * t[2]) / nt;

        cols[i][0] = 1.0;
        cols[i][1] = -(mu[i] * st1);
        cols[i][2] = -(mu[i] * st2);
        width[i] = 1;
      }
    }
    nz += width[i];
  }
  memset(S, 0, (size_t)(3 * k * nz) * sizeof(double));
  for (int i = 0, at = 0; i < k; at += width[i], i++)
  {
    for (int c = 0; c < 3; c++)
    {
      if (width[i] == 1)
      {
        S[(3 * i + c) * nz + at] = cols[i][c];
      }
      else
      {
        S[(3 * i + c) * nz + at + c] = 1.0;
      }
    }
  }
  return nz;
}

/** machine.stack.chain_qp.affine: the maps of u, carved from *w. */
static void affine(const wbc_step_t *p, wbc_maps_t *m, const double *held, const double *Minv, const double *Mpg, int ng,
                   const double *rhs0, const double *Jp, const double *Jg, const double *dJv, const double *Q, const double *S,
                   const double *RP, const double *RG, const double *r, const double *MA, const double *hA, const double *JA,
                   double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         np_ = m->np, nd = m->nd, nz = m->nz, k3 = m->k3, m1 = m->m1, nh = ng - nd, k = k3 / 3;
  double           *TS = carve(w, k3 * nz + 1), *MpgD = carve(w, np_ * nd), *MpgH = carve(w, np_ * nh + 1);
  double           *JgD = carve(w, k3 * nd + 1), *JgH = carve(w, k3 * nh + 1), *JpT = carve(w, np_ * k3 + 1);
  double           *t = carve(w, np_), *t2 = carve(w, np_), *JT = carve(w, np_ * nz + 1), *Cx = carve(w, k3 * nd + 1);
  double           *Cz = carve(w, k3 * nz + 1), *c0 = carve(w, k3 + 1), *c1 = carve(w, k3 + 1), *RGD = carve(w, m1 * nd + 1);
  double           *RGH = carve(w, m1 * nh + 1), *tm = carve(w, m1 + 1), *MAP = carve(w, nd * np_), *MAD = carve(w, nd * nd);
  double           *MAH = carve(w, nd * nh + 1), *JAT = carve(w, nd * k3 + 1), *td = carve(w, nd), *tz = carve(w, nd * nz + 1);
  double            QT[9];

  m->p0 = carve(w, np_);
  m->Px = carve(w, np_ * nd);
  m->Pz = carve(w, np_ * nz + 1);
  m->a0 = carve(w, k3 + 1);
  m->Ax = carve(w, k3 * nd + 1);
  m->Az = carve(w, k3 * nz + 1);
  m->Rx = carve(w, m1 * nd + 1);
  m->Rz = carve(w, m1 * nz + 1);
  m->rr = carve(w, m1 + 1);
  m->t0 = carve(w, nd);
  m->Tx = carve(w, nd * nd);
  m->Tz = carve(w, nd * nz + 1);
  for (int i = 0; i < k; i++)
  {
    for (int a = 0; a < 3; a++)
    {
      for (int c = 0; c < 3; c++)
      {
        QT[3 * a + c] = Q[9 * i + 3 * c + a];
      }
    }
    mm(l, QT, S + 3 * i * nz, TS + 3 * i * nz, 3, 3, nz);
  }
  sliced(Mpg, np_, ng, 0, nd, MpgD);
  sliced(Mpg, np_, ng, nd, ng, MpgH);
  sliced(Jg, k3, ng, 0, nd, JgD);
  sliced(Jg, k3, ng, nd, ng, JgH);
  for (int rw = 0; rw < k3; rw++)
  {
    for (int a = 0; a < np_; a++)
    {
      JpT[a * k3 + rw] = Jp[rw * np_ + a];
    }
  }
  /* qdd_P = p0 + Px x + Pz zeta */
  mv(l, MpgH, held, t, np_, nh);
  for (int a = 0; a < np_; a++)
  {
    t2[a] = rhs0[a] - t[a];
  }
  mv(l, Minv, t2, m->p0, np_, np_);
  mm(l, Minv, MpgD, m->Px, np_, np_, nd);
  for (int i = 0; i < np_ * nd; i++)
  {
    m->Px[i] = -m->Px[i];
  }
  mm(l, JpT, TS, JT, np_, k3, nz);
  mm(l, Minv, JT, m->Pz, np_, np_, nz);
  /* the contacts' accelerations in their frames: a0 + Ax x + Az zeta */
  mv(l, Jp, m->p0, c0, k3, np_);
  mv(l, JgH, held, c1, k3, nh);
  for (int rw = 0; rw < k3; rw++)
  {
    c0[rw] = (c0[rw] + c1[rw]) + dJv[rw];
  }
  framed(Q, k, c0, 1, m->a0);
  mm(l, Jp, m->Px, Cx, k3, np_, nd);
  for (int i = 0; i < k3 * nd; i++)
  {
    Cx[i] = Cx[i] + JgD[i];
  }
  framed(Q, k, Cx, nd, m->Ax);
  mm(l, Jp, m->Pz, Cz, k3, np_, nz);
  framed(Q, k, Cz, nz, m->Az);
  /* her tasks: Rx x + Rz zeta = rr */
  sliced(RG, m1, ng, 0, nd, RGD);
  sliced(RG, m1, ng, nd, ng, RGH);
  mm(l, RP, m->Px, m->Rx, m1, np_, nd);
  for (int i = 0; i < m1 * nd; i++)
  {
    m->Rx[i] = m->Rx[i] + RGD[i];
  }
  mm(l, RP, m->Pz, m->Rz, m1, np_, nz);
  mv(l, RP, m->p0, tm, m1, np_);
  for (int i = 0; i < m1; i++)
  {
    m->rr[i] = r[i] - tm[i];
  }
  mv(l, RGH, held, tm, m1, nh);
  for (int i = 0; i < m1; i++)
  {
    m->rr[i] = m->rr[i] - tm[i];
  }
  /* her torques: t0 + Tx x + Tz zeta */
  sliced(MA, nd, np_ + ng, 0, np_, MAP);
  sliced(MA, nd, np_ + ng, np_, np_ + nd, MAD);
  sliced(MA, nd, np_ + ng, np_ + nd, np_ + ng, MAH);
  for (int rw = 0; rw < k3; rw++)
  {
    for (int a = 0; a < nd; a++)
    {
      JAT[a * k3 + rw] = JA[rw * nd + a];
    }
  }
  mv(l, MAP, m->p0, m->t0, nd, np_);
  mv(l, MAH, held, td, nd, nh);
  for (int i = 0; i < nd; i++)
  {
    m->t0[i] = (m->t0[i] + td[i]) + hA[i];
  }
  mm(l, MAP, m->Px, m->Tx, nd, np_, nd);
  for (int i = 0; i < nd * nd; i++)
  {
    m->Tx[i] = m->Tx[i] + MAD[i];
  }
  mm(l, MAP, m->Pz, m->Tz, nd, np_, nz);
  mm(l, JAT, TS, tz, nd, k3, nz);
  for (int i = 0; i < nd * nz; i++)
  {
    m->Tz[i] = m->Tz[i] - tz[i];
  }
}

int wbc_chain_qp(wbc_step_t *p, int np_, int ng, int k3, const double *held, const double *Minv, const double *Mpg,
                 const double *rhs0, const double *Jp, const double *Jg, const double *Jv, const double *dJv,
                 const double *Q, const double *RP, const double *RG, const double *r, int m1, const double *want,
                 const double *slide, const double *MA, const double *hA, const double *JA, const double *lo,
                 const double *hi, double *x, double *qddP, double *z, double *tau, double *residual, int *met,
                 double *qddP0, double *tau0, double **w)
{
  const wbc_blas_t *l = &p->blas;
  const int         nd = p->nd, k = k3 / 3;
  int               width[WBC_STEP_C], holds[WBC_STEP_C], hands[WBC_STEP_C], nh = 0, ma, nu;
  int               key[WBC_STEP_C][3], let[WBC_STEP_C], zrow[WBC_STEP_C], sled[WBC_STEP_C], bears[WBC_STEP_C];
  int               stick[WBC_STEP_C];
  double            mu[WBC_STEP_C];
  wbc_maps_t        m;
  double           *S = carve(w, k3 * k3 + 1), *H1, *g1, *G1, *A, *b, *u, *zl = carve(w, k3 + 1), *al = carve(w, k3 + 1);
  double           *tp = carve(w, np_), *tn = carve(w, nd), *fv = carve(w, k3 + 1), QT[9];

  m.np = np_;
  m.nd = nd;
  m.k3 = k3;
  m.m1 = m1;
  keyed(p, k, key, let);
  m.nz = unknowns(p, Q, slide, width, S, mu, bears, sled);
  nu = nd + m.nz;
  /* a touching contact's normal is held unless let go or on a foot told free; an anchor's tangents stick where its
     patch bears (machine.stack.chain.step) */
  for (int i = 0, at = 0; i < k; i++)
  {
    const int loose = freed(p, i);

    let[i] = !loose && let[i];
    holds[i] = !loose && !let[i];
    stick[i] = holds[i] && bears[i] && !sled[i];
    zrow[i] = at;
    at += (holds[i] ? 2 : 3) + (width[i] == 3 ? 8 : 1);
  }
  affine(p, &m, held, Minv, Mpg, ng, rhs0, Jp, Jg, dJv, Q, S, RP, RG, r, MA, hA, JA, w);
  H1 = carve(w, nu * nu);
  g1 = carve(w, nu);
  G1 = carve(w, nu * nu);
  A = carve(w, WBC_QP_M * nu);
  b = carve(w, WBC_QP_M);
  u = carve(w, nu);
  for (int rw = 0; rw < k3; rw++)
  {
    fv[rw] = p->cf[rw / 3][rw % 3];
  }
  ma = wbc_chain_rows(p, &m, S, width, mu, holds, stick, Jv, fv, Q, lo, hi, H1, g1, G1, A, b, w);
  if (ma < 0)
  {
    return 1;
  }
  for (int i = 0; i < k; i++)
  {
    if (holds[i] && bears[i] && p->cpatch[i] >= p->carpal && p->cpatch[i] < 3 * p->carpal)
    {
      hands[nh++] = i;
    }
  }
  if (wbc_chain_levels(p, &m, H1, g1, G1, A, b, ma, Jg, ng, hands, nh, want, u, met, w))
  {
    return 1;
  }
  /* the answer's parts (machine.stack.chain_qp.answer) */
  memcpy(x, u, (size_t)nd * sizeof(double));
  mv(l, S, u + nd, zl, k3, m.nz);
  mv(l, m.Ax, x, al, k3, nd);
  for (int rw = 0; rw < k3; rw++)
  {
    al[rw] = m.a0[rw] + al[rw];
  }
  mv(l, m.Az, u + nd, S, k3, m.nz);
  *residual = 0.0;
  for (int i = 0; i < k; i++)
  {
    double gap;

    for (int a = 0; a < 3; a++)
    {
      for (int c = 0; c < 3; c++)
      {
        QT[3 * a + c] = Q[9 * i + 3 * c + a];
      }
    }
    mv(l, QT, zl + 3 * i, z + 3 * i, 3, 3);
    al[3 * i] = al[3 * i] + S[3 * i];
    gap = fabs(zl[3 * i] < al[3 * i] ? zl[3 * i] : al[3 * i]);
    *residual = gap > *residual ? gap : *residual;
  }
  letting(p, k, key, holds, let, zrow, al);
  mv(l, m.Px, x, qddP, np_, nd);
  for (int a = 0; a < np_; a++)
  {
    qddP[a] = m.p0[a] + qddP[a];
  }
  mv(l, m.Pz, u + nd, tp, np_, m.nz);
  for (int a = 0; a < np_; a++)
  {
    qddP[a] = qddP[a] + tp[a];
  }
  mv(l, m.Tx, x, tau, nd, nd);
  for (int i = 0; i < nd; i++)
  {
    tau[i] = m.t0[i] + tau[i];
  }
  mv(l, m.Tz, u + nd, tn, nd, m.nz);
  for (int i = 0; i < nd; i++)
  {
    tau[i] = tau[i] + tn[i];
  }
  /* with nothing asked of her joints, the forces kept: the shrink's other end (machine.stack.chain.step) */
  mv(l, m.Pz, u + nd, qddP0, np_, m.nz);
  for (int a = 0; a < np_; a++)
  {
    qddP0[a] = m.p0[a] + qddP0[a];
  }
  for (int i = 0; i < nd; i++)
  {
    tau0[i] = m.t0[i] + tn[i];
  }
  return 0;
}
