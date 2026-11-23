/** wbc_afford.h - what the robot's things afford it, told anew as a look moves them (wbc_afford.c):
    machine.way.furniture's `_near` and machine.way.affordance's `clear` over a look's rows as registry.Things
    lays them (centres (n, 3), half sizes (n, 3), rotations (n, 3, 3), C-ordered), numpy's arithmetic operation for
    operation (wbc_np.h), its OpenBLAS for each product (wbc_blas.h), the robot's hypot (wbc_libm.h). Portable C11,
    the host's alone. */
#ifndef WBC_AFFORD_H
#define WBC_AFFORD_H

#include <stdint.h>

#include "wbc_blas.h"

/** furniture._near past its rows' identities: whether each of `n` rows is told anew as the rows `now` (centre, half,
    R) follow `was` - a row moved (any of its numbers changed, or `told`, its word changed), or a moved row's circle
    and its own within `cell` m (`root2` the rectangle's reach over its circle) either way -, into `near`: the rows
    moved. */
int wbc_afford_near(int n, const double *c_now, const double *h_now, const double *R_now, const double *c_was,
                    const double *h_was, const double *R_was, const uint8_t *told, double cell, double root2,
                    uint8_t *near);

/** affordance.clear past its box's footprint: the area of a box's top (`box`: its centre (x, z), its floor axes a row
    each, its half sizes along them; `reach` its circle's radius) clear of the `n` things (centre, half, R) whose
    bottoms lie in [lo, hi], each `other`'s alone, m^2 - a thing whose circle misses the box's covers none of it, the
    rest covering the rectangle round them in the box's frame, summed in their order. */
double wbc_afford_clear(const wbc_blas_t *l, int n, const double *centre, const double *half, const double *R,
                        const uint8_t *other, const double box[8], double lo, double hi, double reach, double root2);

#endif
