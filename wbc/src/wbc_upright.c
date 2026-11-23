/** wbc_upright.c - machine.law.motion.control's own arithmetic between its parts (wbc_upright.h), Python's floats
    operation for operation, its two-argument min and max as CPython keeps the first unless the second wins
    outright, numpy's sum and max where control sums or maxes its arrays (wbc_np.h). Portable C11, the host's
    alone. */
#include "wbc_upright.h"
#include "wbc_np.h"

#include <math.h>

/* Python's max(a, b): the first unless the second is greater. */
static double pymax(double a, double b)
{
  return (b > a) ? b : a;
}

/* Python's min(a, b): the first unless the second is less. */
static double pymin(double a, double b)
{
  return (b < a) ? b : a;
}

/* numpy's maximum.reduce of a column from its initial: a nan kept once met. */
static double column_max(const double *tab, int rows, int cols, int col, double initial)
{
  double m = initial;

  for (int r = 0; r < rows; r++)
  {
    const double x = tab[(r * cols) + col];

    m = ((m >= x) || (isnan(m) != 0)) ? m : x;
  }
  return m;
}

void wbc_upright(const wbc_upright_t *u, const wbc_borne_t *b, const double *st, const double *out, const double *a,
                 const double *rest, const double *told, const double *tab, int rows, double load0, double load1,
                 double weight, double reclined, double sw, double w_hop, double got[7])
{
  pysum_t up       = {0.0, 0.0, 0};
  double  pelvic   = 0.0;
  double  ratio    = out[u->out_ks[0]] / rest[u->rest_k[0]];
  int     both     = (load0 >= u->touch_n) && (load1 >= u->touch_n);
  double  isch_w, borne_w, pal_due, upright, w_gait, w_drive, foot, ease, w_base, g_on, brake;

  /* what bears the robot's pelvis: its contacts' push up, Python's sum */
  for (int c = 0; c < b->nc; c++)
  {
    const double on = (b->body[c] == u->pelvis) ? 1.0 : 0.0;

    pysum_add(&up, b->f[c][1] * on);
    pelvic = (on > pelvic) ? on : pelvic;
  }
  isch_w  = pymin(1.0, told[u->told_isch] / pymax(0.5 * npsum(told, u->told_n), u->tiny));
  borne_w = pymin(1.0, pysum_of(&up) / pymax(u->prop_on * weight, u->tiny));
  pal_due = pymax(pymax(told[u->told_palmar], st[u->st_to_pal]), column_max(tab, rows, u->tab_cols, u->tab_pal, 0.0));
  upright = ((both == 0) && (pal_due <= 0.0) && (reclined <= 0.5) && (st[u->st_to_zmp] < 0.5)
             && (fabs(out[u->out_slid]) <= 1e-6)) ? 1.0 : 0.0;
  got[0]  = (isch_w * borne_w) * upright;
  /* its arms' hold: numpy's max of their stiffness over its rest's */
  for (int k = 1; k < 3; k++)
  {
    const double x = out[u->out_ks[k]] / rest[u->rest_k[k]];

    ratio = ((ratio > x) || (isnan(ratio) != 0)) ? ratio : x;
  }
  got[1]  = pymax(1.0, ratio);
  w_gait  = pymin(1.0, pymax(0.0, (0.05 - st[u->st_now_gain]) / 0.05));
  w_drive = pymin(1.0, fabs(st[u->st_to_drive]) / u->stirs);
  foot    = pymin(1.0, pymax(0.0, out[u->out_held] - 1.0));
  ease    = (1.0 - (a[u->a_strides] * (1.0 - a[u->a_borne]))) * w_gait;
  w_base  = sw * pymax(w_hop, ease);
  g_on    = pymax(st[u->st_to_gain], st[u->st_now_gain]);
  brake   = (foot * (1.0 - w_drive)) * g_on;
  got[2]  = sw * pymax(w_hop, pymax(ease, brake));
  got[3]  = w_base;
  got[4]  = w_hop * w_base;
  got[5]  = w_hop * w_base;
  got[6]  = pelvic;
}
