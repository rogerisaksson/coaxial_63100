/** wbc_law.h - her latent loop's step and her upper body's coupling on the host (wbc_law.c):
    machine.latent.step and machine.support.couple, numpy's arithmetic and numpy's own OpenBLAS
    (wbc_blas.h) operation for operation. Portable C11, the host's alone. */
#ifndef WBC_LAW_H
#define WBC_LAW_H

#include "wbc_blas.h"

/** The most modes, her contact manifolds (machine.ortho.MANIFOLDS), row parameters, driven joints. */
#define WBC_LAW_N 16
#define WBC_LAW_M 5
#define WBC_LAW_P (5 * WBC_LAW_N + 7 + WBC_LAW_M + 1)
#define WBC_LAW_D 32

/** What the step and the coupling are laid on, filled once from machine.latent, machine.ortho
    and machine.support: their layouts' offsets and their constants. */
typedef struct
{
  int           n, p, driven;
  int           z, k, d, amp, phase, gain, hz, zmp, yaw;
  int           now, from, to, t0, blend, theta, v0, v1, contig, shape;
  int           vals, said, at, lerp, curve, shaped, table;
  int           ref, rate, acc, ks, ds, psi, sway, turns, held, yawed, bend, duck, ducked, manifold, manifolded;
  int           slide, slid;
  double        edge_s, tau, tau_v, eso_0, eso_s, sigma, bleed;
  const double *pi;    /**< the coupling (n, driven) */
  const double *gamma; /**< each driven joint's admittance */
  wbc_blas_t    blas;
} wbc_law_t;

/** machine.latent.step: `out` this pass, `st` blended, of the table `tab` (ntab rows, contiguous);
    `hit` ntab doubles of work. */
void wbc_law_latent(const wbc_law_t *k, double *st, const double *tab, int ntab, double now, double dt, double *out,
                    double *hit);

/** machine.support.couple: `dq` (driven), out's REF shifted, `eso_z` bled. */
void wbc_law_couple(const wbc_law_t *k, const double *q, const double *qd, const double *ref_q, const double *ref_v,
                    double *eso_z, double *out, double *dq);

#endif
