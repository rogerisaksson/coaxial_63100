/** wbc_wholes.c - what is one thing, many to one, and the buildings the robot's things make (wbc_wholes.h):
    machine.eyes.wholes.grouped - `_instances` and `classes.houses` -, numpy's arithmetic operation for operation
    (wbc_np.h): the lodge's 93 boxes 1.6 ms of numpy a told state. Portable C11, the host's alone. */
#include "wbc_wholes.h"
#include "wbc_np.h"

#include <math.h>

/** A look's scratch: each thing's extents, corners, middle and half along each axis, its reach along its long
    floor axis (a_lo, a_hi), its labels and the buildings', and the pairs. */
typedef struct
{
  double        *e, *lo, *hi, *mid, *hlf, *alo, *ahi;
  int32_t       *label, *low, *hop, *hlabel, *of;
  unsigned char *beside, *level, *plane, *link, *hlink;
} work_t;

#define WBC_WHOLES_DOUBLES 17
#define WBC_WHOLES_INTS    5
#define WBC_WHOLES_PAIRS   5

size_t wbc_wholes_scratch(int n)
{
  const size_t m = (size_t)(n > 0 ? n : 0);

  return m * (WBC_WHOLES_DOUBLES * sizeof(double) + WBC_WHOLES_INTS * sizeof(int32_t)) + WBC_WHOLES_PAIRS * m * m;
}

/** The next `bytes` of the scratch `s`, taken. */
static unsigned char *taken(unsigned char **s, size_t bytes)
{
  unsigned char *at = *s;

  *s += bytes;
  return at;
}

/** The scratch carved for `n` things. */
static void carve(work_t *w, int n, unsigned char *s)
{
  const size_t   m = (size_t)n, d = sizeof(double), i = sizeof(int32_t);
  unsigned char *at = s;

  w->e = (double *)taken(&at, 3 * m * d);
  w->lo = (double *)taken(&at, 3 * m * d);
  w->hi = (double *)taken(&at, 3 * m * d);
  w->mid = (double *)taken(&at, 3 * m * d);
  w->hlf = (double *)taken(&at, 3 * m * d);
  w->alo = (double *)taken(&at, m * d);
  w->ahi = (double *)taken(&at, m * d);
  w->label = (int32_t *)taken(&at, m * i);
  w->low = (int32_t *)taken(&at, m * i);
  w->hop = (int32_t *)taken(&at, m * i);
  w->hlabel = (int32_t *)taken(&at, m * i);
  w->of = (int32_t *)taken(&at, m * i);
  w->beside = taken(&at, m * m);
  w->level = taken(&at, m * m);
  w->plane = taken(&at, m * m);
  w->link = taken(&at, m * m);
  w->hlink = taken(&at, m * m);
}

/** Whether a thing's half extent along floor axis `j` (0 or 2) is no thicker than a face (classes.planar's flat). */
static int flat(const wbc_classes_t *k, const work_t *w, int a, int j)
{
  return w->hlf[3 * a + j] <= k->half_thin_eps ? 1 : 0;
}

/** Each thing's extents along the room's axes, its corners, middle and half, its reach along its long floor axis
    (`_instances`' u: along x where it is thin across z, else along z). */
static void laid(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R, work_t *w)
{
  for (int a = 0; a < n; a++)
  {
    const double *ca = c + 3 * a, *ha = h + 3 * a, *Ra = R + 9 * a;
    double        absR[9];
    int           u;

    for (int j = 0; j < 9; j++)
    {
      absR[j] = fabs(Ra[j]);
    }
    for (int j = 0; j < 3; j++)
    {
      w->e[3 * a + j] = npsumof(absR + 3 * j, 1, ha, 1, 3, k->lane);
      w->lo[3 * a + j] = ca[j] - w->e[3 * a + j];
      w->hi[3 * a + j] = ca[j] + w->e[3 * a + j];
    }
    for (int j = 0; j < 3; j++)
    {
      w->mid[3 * a + j] = 0.5 * (w->lo[3 * a + j] + w->hi[3 * a + j]);
      w->hlf[3 * a + j] = 0.5 * (w->hi[3 * a + j] - w->lo[3 * a + j]);
    }
    u = flat(k, w, a, 2) != 0 ? 0 : 2;
    w->alo[a] = w->lo[3 * a + u];
    w->ahi[a] = w->hi[3 * a + u];
  }
}

/** The pairs: which meet a cell apart (classes.meet), which stand level within a cell - a stair's treads a rise -
    of each other, which lie in one wall's plane (classes.planar), and what of one class touches, slabs across each
    other apart (`_instances`' first link). */
static void paired(const wbc_classes_t *k, int n, const double *c, const int32_t *code, const int32_t *flags,
                   work_t *w)
{
  for (int a = 0; a < n; a++)
  {
    const double *ea = w->e + 3 * a;
    const double  sa = (flags[a] & WBC_WHOLES_STAIRS) != 0 ? k->step : k->cell;
    const int     xa = flat(k, w, a, 0), za = flat(k, w, a, 2);
    const int     slab_xa = xa != 0 && za == 0, slab_za = za != 0 && xa == 0;

    for (int b = 0; b < n; b++)
    {
      const double *eb = w->e + 3 * b;
      const double  sb = (flags[b] & WBC_WHOLES_STAIRS) != 0 ? k->step : k->cell;
      const double  gx = fabs(c[3 * a] - c[3 * b]), gz = fabs(c[3 * a + 2] - c[3 * b + 2]);
      const int     xb = flat(k, w, b, 0), zb = flat(k, w, b, 2);
      const int     across = (slab_xa && zb != 0 && xb == 0) || (slab_za && xb != 0 && zb == 0);
      const size_t  ab = (size_t)a * (size_t)n + (size_t)b;
      unsigned char plane = 0U;

      w->beside[ab] = (a != b && gx <= ((ea[0] + eb[0]) + k->cell) + k->eps
                       && gz <= ((ea[2] + eb[2]) + k->cell) + k->eps) ? 1U : 0U;
      w->level[ab] = ((fabs(w->mid[3 * a + 1] - w->mid[3 * b + 1]) - w->hlf[3 * a + 1]) - w->hlf[3 * b + 1]
                      <= npmaximum(sa, sb) + k->eps) ? 1U : 0U;
      for (int j = 0; j < 3; j += 2)
      {
        if (flat(k, w, a, j) != 0 && flat(k, w, b, j) != 0 && fabs(w->mid[3 * a + j] - w->mid[3 * b + j]) <= k->cell_eps)
        {
          plane = 1U;
        }
      }
      w->plane[ab] = plane;
      w->link[ab] = (code[a] == code[b] && w->beside[ab] != 0U && w->level[ab] != 0U && !across
                     && (flags[a] & WBC_WHOLES_NOTONE) == 0 && (flags[b] & WBC_WHOLES_NOTONE) == 0) ? 1U : 0U;
    }
  }
}

/** Whether a wall standing from the floor stands between things `a` and `b` along their plane: one of the piers
    (walls whose bottoms lie in the lowest slab) in a's plane, level with both, overlapping the gap from `start` to
    `end` by more than THIN. */
static int stood(const wbc_classes_t *k, int n, const int32_t *flags, const work_t *w, int a, int b, double start,
                 double end)
{
  for (int p = 0; p < n; p++)
  {
    const size_t ap = (size_t)a * (size_t)n + (size_t)p, bp = (size_t)b * (size_t)n + (size_t)p;

    if ((flags[p] & WBC_WHOLES_WALL) != 0 && w->lo[3 * p + 1] <= k->step
        && npminimum(w->ahi[p], end) - npmaximum(w->alo[p], start) > k->thin_eps && w->plane[ap] != 0U
        && w->level[ap] != 0U && w->level[bp] != 0U)
    {
      return 1;
    }
  }
  return 0;
}

/** What the robot sees through in one plane within its map's reach, no wall standing between, one (`_instances`'
    second link). */
static void through(const wbc_classes_t *k, int n, const int32_t *code, const int32_t *flags, work_t *w)
{
  for (int a = 0; a < n; a++)
  {
    if ((flags[a] & WBC_WHOLES_THROUGH) == 0)
    {
      continue;
    }
    for (int b = 0; b < n; b++)
    {
      const size_t ab = (size_t)a * (size_t)n + (size_t)b;
      double       start, end;

      if (code[a] != code[b] || w->plane[ab] == 0U || w->level[ab] == 0U)
      {
        continue;
      }
      start = npminimum(w->ahi[a], w->ahi[b]);
      end = npmaximum(w->alo[a], w->alo[b]);
      if (end - start <= k->far_eps && stood(k, n, flags, w, a, b, start, end) == 0)
      {
        w->link[ab] = 1U;
      }
    }
  }
}

/** The buildings (classes.houses): each a cabin where its storey stands under two of the robot, its members its
    shell's things and what roofs them beside them; their walls on one side one (`_instances`' third link). */
static int housed(const wbc_classes_t *k, int n, const int32_t *flags, work_t *w, int32_t *cabin, double *blo,
                  double *bhi, unsigned char *member)
{
  const int nb = wbc_classes_houses(k, n, w->lo, w->hi, w->mid, w->hlf, flags, w->hlabel, w->low, w->hlink, blo, bhi,
                                    w->of);

  for (int g = 0; g < nb; g++)
  {
    unsigned char *in = member + (size_t)g * (size_t)n;
    double         top = 0.0;

    for (int a = 0; a < n; a++)
    {
      const int part = w->hlabel[a] == w->of[g];
      int       roofed = 0;

      for (int b = 0; b < n && part == 0 && (flags[a] & WBC_CLASSES_ROOF) != 0 && roofed == 0; b++)
      {
        roofed = (w->beside[(size_t)a * (size_t)n + (size_t)b] != 0U && w->hlabel[b] == w->of[g]) ? 1 : 0;
      }
      in[a] = (part != 0 || roofed != 0) ? 1U : 0U;
      if (part != 0 && (flags[a] & WBC_CLASSES_STOREY) != 0)
      {
        top = npmaximum(top, w->hi[3 * a + 1]);
      }
    }
    cabin[g] = top <= 2.0 * k->overhead ? 1 : 0;
    for (int a = 0; a < n; a++)
    {
      for (int b = 0; b < n; b++)
      {
        const size_t ab = (size_t)a * (size_t)n + (size_t)b;

        if (in[a] != 0U && in[b] != 0U && (flags[a] & WBC_WHOLES_WALL) != 0 && (flags[b] & WBC_WHOLES_WALL) != 0
            && w->plane[ab] != 0U)
        {
          w->link[ab] = 1U;
        }
      }
    }
  }
  return nb;
}

/** `_instances`' labels: each thing's lowest linked whole, jumped to its own lowest, till they hold. */
static void labelled(int n, const int32_t *whole, work_t *w, int32_t *one)
{
  int same = 0;

  for (int a = 0; a < n; a++)
  {
    w->label[a] = whole[a];
  }
  while (same == 0)
  {
    for (int a = 0; a < n; a++)
    {
      int32_t least = n;

      for (int b = 0; b < n; b++)
      {
        if (w->link[(size_t)a * (size_t)n + (size_t)b] != 0U && w->label[b] < least)
        {
          least = w->label[b];
        }
      }
      w->low[a] = w->label[a] < least ? w->label[a] : least;
    }
    same = 1;
    for (int a = 0; a < n; a++)
    {
      w->hop[a] = w->low[w->low[a]];
      if (w->hop[a] != w->label[a])
      {
        same = 0;
      }
    }
    for (int a = 0; a < n; a++)
    {
      w->label[a] = w->hop[a];
    }
  }
  for (int a = 0; a < n; a++)
  {
    one[a] = w->label[a];
  }
}

int wbc_wholes_grouped(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                       const int32_t *code, const int32_t *flags, const int32_t *whole, int32_t *one, int32_t *cabin,
                       double *blo, double *bhi, unsigned char *member, unsigned char *w_)
{
  work_t w;
  int    nb;

  if (n <= 0 || n > WBC_CLASSES_BOXES)
  {
    return n == 0 ? 0 : -1;
  }
  carve(&w, n, w_);
  laid(k, n, c, h, R, &w);
  paired(k, n, c, code, flags, &w);
  through(k, n, code, flags, &w);
  nb = housed(k, n, flags, &w, cabin, blo, bhi, member);
  labelled(n, whole, &w, one);
  return nb;
}
