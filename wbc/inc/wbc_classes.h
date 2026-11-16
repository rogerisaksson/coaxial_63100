/** wbc_classes.h - what the robot's eyes take a thing for, in C (wbc_classes.c): machine.eyes.classes.of over a
    look's boxes - each one's class by the text's tables in their order, the thing each is a piece of, the buildings
    they make and what lies by them -, numpy's arithmetic operation for operation (wbc_np.h). Portable C11, the
    host's alone. */
#ifndef WBC_CLASSES_H
#define WBC_CLASSES_H

#include <stddef.h>
#include <stdint.h>

/** The boxes a look holds at most. */
#define WBC_CLASSES_BOXES 1024

/** A thing's flags (classes._coded): what its look tells alone is a material (RAW), seen through (SEE), furniture
    (FURN); its told name a building's shell, an opening, a roof, a storey's; its kind a door's or a bed's, told
    by its box (classes.BORNE). */
#define WBC_CLASSES_RAW    1
#define WBC_CLASSES_SEE    2
#define WBC_CLASSES_FURN   4
#define WBC_CLASSES_SHELL  8
#define WBC_CLASSES_OPEN   16
#define WBC_CLASSES_ROOF   32
#define WBC_CLASSES_STOREY 64
#define WBC_CLASSES_DOOR   128
#define WBC_CLASSES_BED    256

/** The classes `of` tells, in classes.CLASSES' order: the tops, the faces, a thing, and what lies by a building. */
enum
{
  WBC_CLASS_FLOOR,
  WBC_CLASS_STAIRS,
  WBC_CLASS_STEP,
  WBC_CLASS_PLATFORM,
  WBC_CLASS_SOFA,
  WBC_CLASS_CHAIR,
  WBC_CLASS_TABLE,
  WBC_CLASS_BENCH,
  WBC_CLASS_STOOL,
  WBC_CLASS_CABINET,
  WBC_CLASS_SHELF,
  WBC_CLASS_WINDOW,
  WBC_CLASS_RAILING,
  WBC_CLASS_CEILING,
  WBC_CLASS_WALL,
  WBC_CLASS_POST,
  WBC_CLASS_THING,
  WBC_CLASS_VERANDA,
  WBC_CLASS_TERRACE,
  WBC_CLASS_GROUND
};

/** The text's heights and reaches as machine.eyes.classes lays them (m), each sum the host summed as numpy sums it;
    numpy's sum of a box's extents (an index of orders.ORDERS); the codes and flags a door's and a bed's look falls
    back on (timber, glass). */
typedef struct
{
  double  flush, step, over, overhead, seat0, seat1, bed0, bed1, brace, lap, lumbar, stance;
  double  thin_eps;       /**< THIN + 1e-9 */
  double  cell, eps;      /**< voxels.CELL_M, 1e-9: added in turn, as numpy adds them */
  double  brace_half;     /**< BRACE + 0.5 CELL_M */
  double  far_eps;        /**< FAR_M + 1e-9 */
  double  half_thin_eps;  /**< 0.5 THIN + 1e-9 */
  double  cell_eps;       /**< CELL_M + 1e-9 */
  int32_t lane, timber, timber_flags, glass, glass_flags, spare;
} wbc_classes_t;

/** The scratch `wbc_classes_of` needs for `n` things, bytes: its per-thing numbers and its pairs. */
size_t wbc_classes_scratch(int n);

/** classes.of: the `n` things' classes - centres and half sizes `c`, `h` (n, 3), turns `R` (n, 3, 3), whether the
    space over each is seen clear `clear` (n,), each one's kind `code` and its flags (WBC_CLASSES_*) - into `cls`
    (a class of the enum, or -1 its whole's told name), `of` (the kind whose told name it carries), `whole` (the
    thing each is a piece of, its own index where it is whole); `w` the scratch. 0, or -1 past the boxes held. */
int wbc_classes_of(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                   const double *clear, const int32_t *code, const int32_t *flags, int32_t *cls, int32_t *of,
                   int32_t *whole, unsigned char *w);

#endif
