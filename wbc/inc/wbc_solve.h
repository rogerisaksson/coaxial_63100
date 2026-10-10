/** wbc_solve.h - the loop's numerics (wbc_solve.c), for wbc_stack.c and the tests. */
#ifndef WBC_SOLVE_H
#define WBC_SOLVE_H

#include "wbc.h"

/** a, n x n by rows of `ld`, into its lower Cholesky factor in place; 0 where not positive. */
int wbc_cholesky(int n, int ld, double *a);

/** x = (L L^T)^-1 b. */
void wbc_chol_solve(int n, int ld, const double *l, const double *b, double *x);

/** The quaternion (w, x, y, z) of a frame's rotation, R by rows. */
void wbc_quat_of(const double r[9], double q[4]);

/** The world-frame rotation vector taking `have` to `want`, rad: machine.wbc.turn_error. */
void wbc_turn_error(const double want[4], const double have[4], double out[3]);

/** A level's rows this tick, least squares in the null space so far under every bound (the
    active set), its torques into s->tau[level]; then the null space under them for the next. */
void wbc_level_solve(wbc_stack_t *s, int level, int last);

/** Each drive's load from the drives' torques, a pair's as its rods share them; and back. */
void wbc_loads(const double tau[WBC_X], double load[WBC_DRIVEN]);
void wbc_unload(const double load[WBC_DRIVEN], double tau[WBC_X]);

#endif
