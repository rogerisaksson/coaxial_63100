/** wbc_wait.c - the rows of her table that wait (wbc_imp.h): machine.motion.held_rows and
    risen_rows on her law's arrays, numpy's and Python's arithmetic operation for operation
    (wbc_np.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

/** The rows due this pass of those `halts` marks: whether any, the earliest's `at`. */
static int due_rows(const wbc_wait_t *w, const double *tab, int n, const unsigned char *halts, double t, double dt,
                    double *first)
{
  int any = 0;

  *first = INFINITY;
  for (int r = 0; r < n; r++)
  {
    const double at = tab[r * w->table + w->at];

    if (halts[r] && at <= t + w->edge_s && at > (t - dt) + w->edge_s)
    {
      any = 1;
      *first = npminimum(*first, at);
    }
  }
  return any;
}

/** The rows from `first` on shifted `shift`. */
static void shifted(const wbc_wait_t *w, double *tab, int n, double shift, double first)
{
  for (int r = 0; r < n; r++)
  {
    double *at = tab + r * w->table + w->at;

    *at = *at + shift * (double)(*at >= first);
  }
}

void wbc_wait_held(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *a, const double *out,
                   double t, double dt, double held_s, double got[3])
{
  unsigned char halts[WBC_WAIT_ROWS] = {0};
  const double  stepping = (double)((a[k->a_need] > 0.0) || (npmaximum(a[k->a_swings], a[k->a_swings + 1]) > 0.0));
  const int     rows = n < WBC_WAIT_ROWS ? n : WBC_WAIT_ROWS;
  double        first, active, held, turns, within, shift;

  for (int r = 0; r < rows; r++)
  {
    const double *row = tab + r * w->table;

    halts[r] = (unsigned char)(row[w->said_gain] > 0.5 && row[w->vals_gain] < 0.05);
  }
  active = stepping * (double)due_rows(w, tab, rows, halts, t, dt, &first);
  held = active * held_s;
  turns = out[k->o_turns];
  within = (double)((turns > w->tiny) && (held <= (4.0 * w->phi_w) / npmax(turns, w->tiny)));
  shift = (dt * active) * within;
  first = first - w->edge_s;
  shifted(w, tab, rows, shift, first);
  got[0] = shift;
  got[1] = first;
  got[2] = held + active * dt;
}

void wbc_wait_risen(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *y, const double *fr,
                    const double *a, const double *st, double t, double dt, double prop_until, double got[3])
{
  unsigned char rises[WBC_WAIT_ROWS] = {0};
  const double *rot = fr + k->f_rot;
  const int     rows = n < WBC_WAIT_ROWS ? n : WBC_WAIT_ROWS;
  const double  has_prop = (double)(y[k->y_prop] > 0.0);
  const double  until = has_prop * (t + 0.06) + (1.0 - has_prop) * prop_until;
  const double  propped = (double)((y[k->y_prop] > 0.0) || (t <= until));
  const double  to_ax = st[w->st_to_z + k->axial], to_sag = st[w->st_to_z + k->sagittal];
  double        c[3], v[3], om, xi[2], sr[2][3], flat[2][2], edge[2], tilt[2], lo[2][2], hi[2][2], lmin[2], hmax[2];
  double        first, outside, shift;

  for (int j = 0; j < 3; j++)
  {
    c[j] = npmv3(rot + 3 * j, y + k->y_com);
    v[j] = npmv3(rot + 3 * j, y + k->y_vcom);
  }
  om = sqrt(k->g / npmax(c[2] - fr[k->f_floor], 0.3));
  npmmt(y + k->y_sole, 2, rot, &sr[0][0]);
  for (int b = 0; b < 2; b++)
  {
    xi[b] = c[b] + v[b] / om;
    flat[b][0] = sr[b][0];
    flat[b][1] = sr[b][1];
    edge[b] = a[k->a_edge + b];
    tilt[b] = y[k->y_tilt + b];
  }
  wbc_imp_boxes(k, flat, edge, tilt, lo, hi);
  for (int j = 0; j < 2; j++)
  {
    lmin[j] = npminimum(lo[0][j], lo[1][j]);
    hmax[j] = npmaximum(hi[0][j], hi[1][j]);
  }
  outside = (double)((xi[0] < lmin[0]) || (xi[1] < lmin[1]) || (xi[0] > hmax[0]) || (xi[1] > hmax[1]));
  /* a row raises her over her soles with its centre of mass told from her heels' edge on */
  for (int r = 0; r < rows; r++)
  {
    const double *row = tab + r * w->table;
    const double  sag = row[w->said_z + k->sagittal] > 0.5 ? row[w->vals_z + k->sagittal] : to_sag;
    const int     over = sag >= -1.0;

    rises[r] = (unsigned char)(row[w->said_z + k->axial] > 0.5 && row[w->vals_z + k->axial] < npmin(to_ax, 2.0) - 1e-6
                               && over);
  }
  shift = ((dt * propped) * outside) * (double)due_rows(w, tab, rows, rises, t, dt, &first);
  first = first - w->edge_s;
  shifted(w, tab, rows, shift, first);
  got[0] = shift;
  got[1] = first;
  got[2] = until;
}
