/** wbc_registry.h - her map's look in C (wbc_registry.c): machine.eyes.registry's passed and seen past its things'
    classes, over its slots' fields as the host lays them (`registry.Rows`: an array a field, each on 64-byte lines),
    numpy's arithmetic operation for operation (wbc_np.h), its OpenBLAS for each product (wbc_blas.h). Portable C11,
    the host's alone. */
#ifndef WBC_REGISTRY_H
#define WBC_REGISTRY_H

#include <stdint.h>

#include "wbc_blas.h"

/** The boxes a look passes at most: its things, its live slots (the text's 256 slots). */
#define WBC_REGISTRY_BOXES 256

/** Her registry: its slots' fields as the host lays them, its constants, numpy's sums here, a look's answer and its
    scratch laid once. */
typedef struct
{
  double           *on, *centre, *half, *R, *var, *salience, *vel, *last, *seen_at, *swell, *drift, *moving, *shown,
                   *clear;                     /* registry.Rows' fields: (slots,), (slots, 3) or (slots, 3, 3) */
  int64_t          *id;
  int32_t          *kind;
  const wbc_blas_t *blas;                      /* numpy's OpenBLAS */
  int               slots;                     /* registry.SLOTS */
  int               lanes[3];                  /* numpy's sum of each einsum, an index of orders.ORDERS: in turn, 2 or 4
                                                  lanes, fused in turn or in 2 lanes - a box's corners, its nearest
                                                  point's turn in and out */
  double            cell2, gate, hit, miss, prune, kept_s, swell_mps, swell_m, diffuse, moving_chi2;
  double            near_m, far_m, tan_up, tan_across;
  double            axis[3], up[3], eyes[3], corners[8][3];
  int               looked[WBC_REGISTRY_BOXES];  /* a look's answer: each thing looked at, */
  int               slot_of[WBC_REGISTRY_BOXES]; /* the slot each looked at took */
  /* a look's scratch */
  double            fc[WBC_REGISTRY_BOXES][3], fh[WBC_REGISTRY_BOXES][3], fR[WBC_REGISTRY_BOXES][9];
  double            fclear[WBC_REGISTRY_BOXES];
  int64_t           fid[WBC_REGISTRY_BOXES], fcode[WBC_REGISTRY_BOXES], lid[WBC_REGISTRY_BOXES];
  int64_t           rest[WBC_REGISTRY_BOXES];
  double            ahead[WBC_REGISTRY_BOXES][3], scale[WBC_REGISTRY_BOXES];
  int               matched[WBC_REGISTRY_BOXES], best[WBC_REGISTRY_BOXES], taken[WBC_REGISTRY_BOXES];
  int               inside[WBC_REGISTRY_BOXES], freed[WBC_REGISTRY_BOXES];
  double            gc[WBC_REGISTRY_BOXES][3], gh[WBC_REGISTRY_BOXES][3], gR[WBC_REGISTRY_BOXES][9];
  double            points[10 * WBC_REGISTRY_BOXES][3], v[10 * WBC_REGISTRY_BOXES][3];
  double            depth[10 * WBC_REGISTRY_BOXES], upv[10 * WBC_REGISTRY_BOXES];
  double            d2[WBC_REGISTRY_BOXES * WBC_REGISTRY_BOXES];
} wbc_registry_t;

/** registry.passed past its slots' kinds: its live slots `live` (nl), each movable or not (`movable`), `dt` s on, her
    head at `pos` turned by `quat` (w, x, y, z): a movable's variance diffused, a slot out of her frustum swollen. */
void wbc_registry_passed(wbc_registry_t *r, const int64_t *live, int nl, const double *movable, const double *pos,
                         const double *quat, double dt);

/** registry.seen past its things' classes: `n` things (centre, half (n, 3), R (n, 3, 3), ids, clear, their classes'
    codes), its live slots `live` (nl) and their classes' codes (-1 none of the look's), at `t` s from her head
    (`pos`, `quat`), the whole room looked at where `everywhere`: each thing looked at (`looked`), the slot each of
    them took (`slot_of`, in their order) - matched or new -, every live slot's salience moved, the pruned let go.
    Those looked at, or -1 past WBC_REGISTRY_BOXES. */
int wbc_registry_seen(wbc_registry_t *r, int n, const double *centre, const double *half, const double *R,
                      const int64_t *ids, const double *clear, const int64_t *codes, const int64_t *live, int nl,
                      const int64_t *slot_codes, double t, int everywhere, const double *pos, const double *quat);

#endif
