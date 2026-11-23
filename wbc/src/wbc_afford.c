/** wbc_afford.c - what the robot's things afford it, told anew as a look moves them (wbc_afford.h):
    furniture._near and affordance.clear over a look's rows, numpy's arithmetic operation for operation (wbc_np.h),
    its OpenBLAS for each product (wbc_blas.h), the robot's hypot (wbc_libm.h). Portable C11, the host's alone. */
#include "wbc_afford.h"
#include "wbc_libm.h"
#include "wbc_np.h"

#include <math.h>

/** Python's sum of floats from its int zero, as CPython 3.12 sums them: the first added to 0, each after by
    Neumaier's compensation, the compensation added at the end where it is non-zero and finite. */
typedef struct
{
  double sum, c;
  int    n;
} pysum_t;

static void pysum_add(pysum_t *s, double x)
{
  if (s->n == 0)
  {
    s->sum = 0.0 + x;
  }
  else
  {
    const double t = s->sum + x;

    if (fabs(s->sum) >= fabs(x))
    {
      s->c += (s->sum - t) + x;
    }
    else
    {
      s->c += (x - t) + s->sum;
    }
    s->sum = t;
  }
  s->n++;
}

static double pysum_of(const pysum_t *s)
{
  return ((s->c != 0.0) && (isfinite(s->c) != 0)) ? s->sum + s->c : s->sum;
}

/** The robot's hypot of one pair, as its libm's arrays tell each (libm.hypot). */
static double hyp(double x, double y)
{
  double o = 0.0;

  wbc_libm_hypot(&x, &y, &o, 1);
  return o;
}

/** Whether row `i` of `now` holds other numbers than `was`'s, or its word was told changed (`told`): numpy's `!=`, a
    nan never its own. */
static int moved_of(int i, const double *c_now, const double *h_now, const double *R_now, const double *c_was,
                    const double *h_was, const double *R_was, const uint8_t *told)
{
  int moved = (told[i] != 0U) ? 1 : 0;

  for (int j = 0; j < 3; j++)
  {
    if ((c_now[3 * i + j] != c_was[3 * i + j]) || (h_now[3 * i + j] != h_was[3 * i + j]))
    {
      moved = 1;
    }
  }
  for (int j = 0; j < 9; j++)
  {
    if (R_now[9 * i + j] != R_was[9 * i + j])
    {
      moved = 1;
    }
  }
  return moved;
}

/** Whether row `i`'s circle and the moved row `m`'s lie within `cell` m of each other on the floor, one look's rows
    `c`, `h` (centres, half sizes): their centres' gap against their radii, numpy's sum in its order. */
static int beside(int i, int m, const double *c, const double *h, double cell, double root2)
{
  const double gap = hyp(c[3 * i] - c[3 * m], c[3 * i + 2] - c[3 * m + 2]);

  return (gap <= hyp(h[3 * i], h[3 * i + 2]) + root2 * hyp(h[3 * m], h[3 * m + 2]) + cell + 1e-9) ? 1 : 0;
}

int wbc_afford_near(int n, const double *c_now, const double *h_now, const double *R_now, const double *c_was,
                    const double *h_was, const double *R_was, const uint8_t *told, double cell, double root2,
                    uint8_t *near)
{
  int moved = 0;

  /* bit 0 a row moved, bit 1 a moved one beside it: the moved rows held apart while the rest are told */
  for (int i = 0; i < n; i++)
  {
    near[i] = (moved_of(i, c_now, h_now, R_now, c_was, h_was, R_was, told) != 0) ? 1U : 0U;
    moved += (int)near[i];
  }
  if (moved > 0)
  {
    for (int i = 0; i < n; i++)
    {
      for (int m = 0; (m < n) && (near[i] == 0U); m++)
      {
        if (((near[m] & 1U) != 0U)
            && ((beside(i, m, c_now, h_now, cell, root2) != 0) || (beside(i, m, c_was, h_was, cell, root2) != 0)))
        {
          near[i] = 2U;
        }
      }
    }
    for (int i = 0; i < n; i++)
    {
      near[i] = (near[i] != 0U) ? 1U : 0U;
    }
  }
  return moved;
}

double wbc_afford_clear(const wbc_blas_t *l, int n, const double *centre, const double *half, const double *R,
                        const uint8_t *other, const double box[8], double lo, double hi, double reach, double root2)
{
  static const double corners[4][2] = {{-1.0, -1.0}, {-1.0, 1.0}, {1.0, -1.0}, {1.0, 1.0}};
  const wbc_view_t    axes_t = {box + 2, 1, 2};
  pysum_t             covered = {0.0, 0.0, 0};

  for (int i = 0; i < n; i++)
  {
    const double *c = centre + 3 * i, *h = half + 3 * i, *r = R + 9 * i;
    const double  bottom = c[1] - h[1];

    if ((other[i] != 0U) && (bottom >= lo) && (bottom <= hi)
        && (hyp(c[0] - box[0], c[2] - box[1]) <= reach + root2 * hyp(h[0], h[2]) + 1e-9))
    {
      /* the thing's corners by its floor axes (its rotation's x and z, transposed), then along the box's */
      const double     oaxes[4] = {r[0], r[6], r[2], r[8]};
      double           a[8], p[8], q[8], local[8], side[2];
      const wbc_view_t va = {a, 2, 1}, vo = {oaxes, 2, 1}, vq = {q, 2, 1};

      for (int k = 0; k < 4; k++)
      {
        a[2 * k] = corners[k][0] * h[0];
        a[2 * k + 1] = corners[k][1] * h[2];
      }
      wbc_matmul(l, va, vo, p, 2, 1, 4, 2, 2);
      for (int k = 0; k < 4; k++)
      {
        q[2 * k] = (c[0] + p[2 * k]) - box[0];
        q[2 * k + 1] = (c[2] + p[2 * k + 1]) - box[1];
      }
      wbc_matmul(l, vq, axes_t, local, 2, 1, 4, 2, 2);
      for (int d = 0; d < 2; d++)
      {
        double most = local[d], least = local[d];

        for (int k = 1; k < 4; k++)
        {
          most = npmaximum(most, local[2 * k + d]);
          least = npminimum(least, local[2 * k + d]);
        }
        side[d] = npmaximum(npminimum(most, box[6 + d]) - npmaximum(least, -box[6 + d]), 0.0);
      }
      pysum_add(&covered, side[0] * side[1]);
    }
  }
  return 4.0 * box[6] * box[7] - pysum_of(&covered);
}
