/** wbc_wholes.h - what is one thing, many to one, and the buildings the robot's things make, in C (wbc_wholes.c):
    machine.eyes.wholes.grouped over a look's told boxes, numpy's arithmetic operation for operation (wbc_np.h).
    Portable C11, the host's alone. */
#ifndef WBC_WHOLES_H
#define WBC_WHOLES_H

#include <stddef.h>
#include <stdint.h>

#include "wbc_classes.h"

/** A told name's flags beside its building's (WBC_CLASSES_SHELL, _OPEN, _ROOF, _STOREY): a wall's, one seen through
    (classes.THROUGH), a stair's, and one that is no thing of its own (a piece of furniture, or what none tells). */
#define WBC_WHOLES_WALL    512
#define WBC_WHOLES_THROUGH 1024
#define WBC_WHOLES_STAIRS  2048
#define WBC_WHOLES_NOTONE  4096

/** The scratch `wbc_wholes_grouped` needs for `n` things, bytes. */
size_t wbc_wholes_scratch(int n);

/** wholes.grouped: the one thing each of `n` things is of - their centres and half sizes `c`, `h` (n, 3), turns `R`
    (n, 3, 3), each told name's `code` and its flags (WBC_CLASSES_SHELL..., WBC_WHOLES_*), the thing each is a piece of
    `whole` (`classes.of`'s) -, its lowest index, into `one`; the buildings they make, each a cabin or not (`cabin`),
    its corners into `blo`, `bhi` (3 each) and its members a byte each of `n` into `member`; `w` the scratch. How
    many buildings, or -1 past the boxes held. */
int wbc_wholes_grouped(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                       const int32_t *code, const int32_t *flags, const int32_t *whole, int32_t *one, int32_t *cabin,
                       double *blo, double *bhi, unsigned char *member, unsigned char *w);

#endif
