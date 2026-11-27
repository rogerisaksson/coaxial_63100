/** wbc_gmo_factor.h - the observer's split's factor kept as pushes join and leave (wbc_gmo.c). */
#ifndef WBC_GMO_FACTOR_H
#define WBC_GMO_FACTOR_H

#include "wbc_gmo.h"

/** The unknowns a row of `h` or `w` is summed for at once: one 64-byte cache line of doubles. */
#define WBC_GMO_LANES 8

/** Push `b` joined to the `n` pushing `at` (in the order they joined): `o->k`'s factor L of K = H over them
    grown a row by forward substitution, the rows before it as they were, its new diagonal's square no less
    than `least`; n + 1. */
int wbc_gmo_joined(wbc_gmo_t *restrict o, int *restrict at, int n, int b, double least);

/** The push at `p` among the `n` pushing `at` left: L's row taken out, its columns from `p` turned back
    lower-triangular by Givens rotations; n - 1. */
int wbc_gmo_left(wbc_gmo_t *restrict o, int *restrict at, int n, int p);

/** Solve `L L^T x = b` from row `from` using symmetric `o->k` (lower `L` + mirrored upper `L^T`),
    reusing `y[0..from-1]` (`L^-1 b`) and streaming contiguous rows in both passes. */
void wbc_gmo_solve(int n, const wbc_gmo_t *restrict o, const double *restrict b, int from,
                   double *restrict y, double *restrict x);

/** Accumulate one or two upper-triangle rows `x` (and `x + 1` when `c1 != NULL`) of `o->h`. */
void wbc_gmo_added(wbc_gmo_t *restrict o, int x, int m, int weighed, const double *restrict c0,
                   const double *restrict c1);

/** Register-blocked `w = g - H[:, on] * f[on]` across `WBC_GMO_LANES` columns at once. */
void wbc_gmo_lessened(wbc_gmo_t *restrict o, int m, int pushing, const int *restrict on,
                      const double *restrict g, const double *restrict f);

#endif
