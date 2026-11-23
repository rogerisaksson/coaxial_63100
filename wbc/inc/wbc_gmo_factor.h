/** wbc_gmo_factor.h - the observer's split's factor kept as pushes join and leave (wbc_gmo.c). */
#ifndef WBC_GMO_FACTOR_H
#define WBC_GMO_FACTOR_H

#include "wbc_gmo.h"

/** Push `b` joined to the `n` pushing `at` (in the order they joined): `o->k`'s factor L of K = H over them
    grown a row by forward substitution, the rows before it as they were, its new diagonal's square no less
    than `least`; n + 1. */
int wbc_gmo_joined(wbc_gmo_t *o, int at[WBC_GMO_F], int n, int b, double least);

/** The push at `p` among the `n` pushing `at` left: L's row taken out, its columns from `p` turned back
    lower-triangular by Givens rotations; n - 1. */
int wbc_gmo_left(wbc_gmo_t *o, int at[WBC_GMO_F], int n, int p);

#endif
