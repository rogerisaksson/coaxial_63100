/** wbc_latched.c - under the ask (wbc_imp.h): machine.senses.borne.latched, what bears her off her soles on her
    support's comparator, kept in `a` - numpy's arithmetic operation for operation (wbc_np.h), its own OpenBLAS for
    the prop's corners (wbc_blas.h), the same bit for bit. Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

double wbc_imp_latched(const wbc_imp_t *k, const double *y, const double *fr, const double *rot, double *a,
                       double lo[2], double hi[2])
{
  /* what bears her off her soles, in her way's frame */
  const double xs[2] = {y[k->y_prop_lo], y[k->y_prop_hi]}, zs[2] = {y[k->y_prop_lo + 2], y[k->y_prop_hi + 2]};
  const double corner[4][3] = {{xs[0], 0.0, zs[0]}, {xs[0], 0.0, zs[1]}, {xs[1], 0.0, zs[0]}, {xs[1], 0.0, zs[1]}};
  /* her support's comparator, the master insert's: latched from its engage, let go under its
     release held its dwell, never within its strike's blanking; its box held under the release */
  const double share = y[k->y_prop] + (y[k->y_palms] + y[k->y_palms + 1]), latched = a[k->a_propped];
  const double struck = (1.0 - latched) * (double)(share >= k->prop_engage);
  const double under = latched * (double)(share < k->prop_release);
  double       way4[4][3], freed;

  wbc_mm3(&k->blas, &corner[0][0], 4, rot, 1, &way4[0][0]);
  a[k->a_propped_below] = under * (a[k->a_propped_below] + fr[k->f_dt]);
  a[k->a_propped_since] = (1.0 - struck) * (a[k->a_propped_since] + fr[k->f_dt]);
  freed = (under * (double)(a[k->a_propped_since] >= k->prop_blank_s)) * (double)(a[k->a_propped_below] >= k->prop_dwell_s);
  a[k->a_propped] = (latched + struck) - freed;
  for (int j = 0; j < 2; j++)
  {
    const double lo_j = npmin(npmin(npmin(way4[0][j], way4[1][j]), way4[2][j]), way4[3][j]);
    const double hi_j = npmax(npmax(npmax(way4[0][j], way4[1][j]), way4[2][j]), way4[3][j]);

    a[k->a_propped_box + j] = share >= k->prop_release ? lo_j : a[k->a_propped_box + j];
    a[k->a_propped_box + 2 + j] = share >= k->prop_release ? hi_j : a[k->a_propped_box + 2 + j];
    lo[j] = a[k->a_propped_box + j];
    hi[j] = a[k->a_propped_box + 2 + j];
  }
  return a[k->a_propped];
}
