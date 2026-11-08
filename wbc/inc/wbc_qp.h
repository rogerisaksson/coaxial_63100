/** wbc_qp.h - machine.stack.qp.solve in C (wbc_qp.c, wbc_qp_interior.c): Goldfarb and Idnani's dual active set on
    the rows scaled to unit norm, Mehrotra's interior point where it gives up - numpy's arithmetic and its OpenBLAS
    (wbc_blas.h) operation for operation, in python's layouts. Portable C11, the host's alone. */
#ifndef WBC_QP_H
#define WBC_QP_H

#include "wbc_blas.h"

/** The largest QP laid: unknowns (wbc_blas's solve bounds them) and rows. */
#define WBC_QP_N 128
#define WBC_QP_M 384
/** The doubles `wbc_qp_solve` carves for n unknowns and m rows, each carve rounded up to eight. */
#define WBC_QP_WORK(n, m) (3 * (m) * (n) + 16 * (m) + 20 * (n) * (n) + 96 * (n) + 1024)

/** machine.stack.qp.solve: x (n) minimising 1/2 x'Hx + g'x under A x <= b, H (n, n) and A (m, n) C-ordered, the
    rows scaled to unit norm; `warm` (nwarm) the rows the last solve ended on, tried first; `most` the dual's steps
    (0: 10 (n + m)). Into x, the rows held `act` (their count into *nact) and their multipliers u (each at least n + m
    long): 0, 1 where none meets it (python's None), 2 where numpy would raise. `work` WBC_QP_WORK(n, m) doubles. */
int wbc_qp_solve(const wbc_blas_t *l, const double *H, const double *g, const double *A, const double *b, int n, int m,
                 const int *warm, int nwarm, int most, double *x, int *act, int *nact, double *u, double *work);

/** machine.stack.qp.interior: the same from x = 0 by Mehrotra's predictor-corrector, on the rows as given: 0, 1 where
    it does not converge, 2 where numpy would raise. `work` the solve's past its own carves. */
int wbc_qp_interior(const wbc_blas_t *l, const double *H, const double *g, const double *A, const double *b, int n,
                    int m, double *x, int *act, int *nact, double *u, double *work);

#endif
