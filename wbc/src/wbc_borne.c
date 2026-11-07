/** wbc_borne.c - what bears her off her soles as her skin feels it (wbc_borne.h): machine.senses.borne.read on her
    law's model, numpy's arithmetic and its OpenBLAS operation for operation (wbc_np.h, wbc_blas.h), MuJoCo's own
    Jacobians by their pointers. Portable C11, the host's alone. */
#include "wbc_borne.h"
#include "wbc_np.h"

#include <math.h>
#include <string.h>

/** numpy's `a @ x` of a C-ordered (3, 3) and a (3,): into o. */
static void mv3(const wbc_blas_t *l, const double *a, const double *x, double o[3])
{
  const wbc_view_t va = {a, 3, 1}, vx = {x, 1, 1};

  wbc_matmul(l, va, vx, o, 1, 1, 3, 3, 1);
}

/** numpy's linalg.norm of a (3,): the square root of its dot with itself. */
static double norm3(const wbc_blas_t *l, const double x[3])
{
  return sqrt(wbc_dot(l, x, 1, x, 1, 3));
}

/** borne._room: segment `k`'s room round a point `local` of it along the floor - its nearest capsule's radius, at a
    tie the smaller (Python's min of (gap, radius)) -; 0 with none. */
static double room_of(const wbc_borne_t *b, int k, const double local[3])
{
  double gap = 0.0, r = 0.0;

  for (int c = 0; c < b->caps[k]; c++)
  {
    const double *a = b->cap_a[k][c], *e = b->cap_b[k][c];
    double        ab[3], la[3], q[3];

    for (int i = 0; i < 3; i++)
    {
      ab[i] = e[i] - a[i];
      la[i] = local[i] - a[i];
    }
    const double t = npclip(wbc_dot(b->blas, la, 1, ab, 1, 3) / npmax(wbc_dot(b->blas, ab, 1, ab, 1, 3), 1e-12), 0.0,
                            1.0);

    for (int i = 0; i < 3; i++)
    {
      q[i] = local[i] - (a[i] + t * ab[i]);
    }
    const double here = norm3(b->blas, q) - b->cap_r[k][c];

    if (c == 0 || here < gap || (here == gap && b->cap_r[k][c] < r))
    {
      gap = here;
      r = b->cap_r[k][c];
    }
  }
  return r;
}

/** numpy's argmax of |f|: the first largest, the first nan where one is. */
static int largest(const double f[3])
{
  int at = 0;

  for (int j = 1; j < 3 && !isnan(fabs(f[at])); j++)
  {
    at = (isnan(fabs(f[j])) || fabs(f[j]) > fabs(f[at])) ? j : at;
  }
  return at;
}

int wbc_borne_read(wbc_borne_t *b, const double *y, const double *v, double weight, double hands, const double *let_go,
                   double dt)
{
  const double     *pushes = y + b->touch_at, *at = y + b->at_at, *gives = y + b->give_at;
  const wbc_blas_t *l = b->blas;
  int               skin[WBC_BORNE_SKIN + 2], soled[2], ns = 0, nk = 0;
  double            fp[3];

  for (int k = 0; k < 2; k++)
  {
    soled[ns] = b->sole[k];
    ns += let_go[k] > 0.5;
  }
  for (int i = 0; i < b->above; i++)
  {
    skin[nk++] = b->order[i];
  }
  for (int i = 0; i < ns; i++)
  {
    skin[nk++] = soled[i];
  }
  b->up = 0.0;
  b->palms[0] = b->palms[1] = 0.0;
  for (int i = 0; i < 3; i++)
  {
    b->low[i] = INFINITY;
    b->high[i] = -INFINITY;
  }
  b->nc = 0;
  mv3(l, b->xmat + 9 * b->ids[b->pelvis], pushes + 3 * b->pelvis, fp);
  const int pivot = ns > 0 && fp[1] > b->bears_n;
  const int ischial = pivot || (hands > 0.0 && fp[1] > b->bears_n);

  for (int i = 0; i < nk; i++)
  {
    const int     k = skin[i];
    const double *pk = pushes + 3 * k, *R = b->xmat + 9 * b->ids[k], *ak = at + 3 * k;
    double        f[3], local[2][3], share[2];
    int           points = 1;

    if (!(pk[0] != 0.0 || pk[1] != 0.0 || pk[2] != 0.0))
    {
      continue;
    }
    mv3(l, R, pk, f);
    if (f[1] <= b->bears_n)
    {
      continue;
    }
    const double give = dt > 0.0 ? gives[k] / (weight / b->g_down * dt * dt) : 0.0;
    const int    let = (ns > 0 && soled[0] == k) || (ns > 1 && soled[1] == k);
    const double prop = let ? 0.0 : 1.0, carpal = (double)b->hand[k];
    const int    side = b->right[k] + let * (k == b->sole[1]);

    b->up += f[1] * (double)!b->hand[k] * prop;
    b->palms[side] += f[1] * carpal;
    if (!(hands > 0.0) && b->hand[k])
    {
      continue;
    }
    const int axis = largest(f);
    const int patch = 2 * axis + (f[axis] < 0.0) + b->carpal * (1 + side) * b->hand[k] + b->let * (1 + side) * let;

    if (k != b->pelvis)
    {
      share[0] = 1.0;
      local[0][0] = ak[0];
      local[0][1] = ak[1];
      local[0][2] = ak[2];
    }
    else
    {
      const double sat = npclip(pk[1] / npmax(norm3(l, pk), 1e-9), 0.0, 1.0);
      const double left = npclip((ak[0] + b->seat_x) / (2.0 * b->seat_x), 0.0, 1.0);
      const double to[2] = {sat * (b->seat_x - ak[0]), sat * (-b->seat_x - ak[0])};

      for (int q = 0; q < 2; q++)
      {
        local[q][0] = ak[0] + to[q];
        local[q][1] = ak[1] + 0.0;
        local[q][2] = ak[2] + 0.0;
      }
      share[0] = left;
      share[1] = 1.0 - left;
      points = 2;
    }
    for (int q = 0; q < points; q++)
    {
      double       p[3], turned[3];
      const double r = room_of(b, k, local[q]);
      const double room[3] = {b->caps[k] ? r * 1.0 : 0.0, b->caps[k] ? r * 0.0 : 0.0, b->caps[k] ? r * 1.0 : 0.0};

      mv3(l, R, local[q], turned);
      for (int j = 0; j < 3; j++)
      {
        p[j] = b->xpos[3 * b->ids[k] + j] + turned[j];
      }
      if (prop != 0.0)
      {
        for (int j = 0; j < 3; j++)
        {
          b->low[j] = npminimum(b->low[j], p[j] - room[j]);
          b->high[j] = npmaximum(b->high[j], p[j] + room[j]);
        }
      }
      if (ischial && b->thigh[k])
      {
        continue;
      }
      if (b->nc >= WBC_BORNE_CONTACTS)
      {
        return -1;
      }
      const int        c = b->nc++;
      const wbc_view_t rows = {b->jp, b->nv, 1}, vel = {v, 1, 1};

      b->jac(b->m, b->d, b->jp, b->jr, p, b->ids[k]);
      for (int row = 0; row < 3; row++)
      {
        for (int j = 0; j < b->n; j++)
        {
          b->J[c][row][j] = b->jp[row * b->nv + j];
        }
      }
      b->jac_dot(b->m, b->d, b->jp, b->jr, p, b->ids[k]);
      wbc_matmul(l, rows, vel, b->dJv[c], 1, 1, 3, b->n, 1);
      for (int j = 0; j < 3; j++)
      {
        b->p[c][j] = p[j];
        b->f[c][j] = share[q] * f[j];
      }
      b->carpal_of[c] = carpal;
      b->side[c] = side;
      b->patch[c] = patch;
      b->room[c] = room[0];
      b->body[c] = b->ids[k];
      b->give[c] = give;
      b->corner[c] = -1;
    }
  }
  return 0;
}

/** Contact `from` laid at `to`: its point, push, drift, Jacobian and the rest. */
static void moved(wbc_borne_t *b, int from, int to)
{
  memcpy(b->p[to], b->p[from], sizeof b->p[to]);
  memcpy(b->f[to], b->f[from], sizeof b->f[to]);
  memcpy(b->dJv[to], b->dJv[from], sizeof b->dJv[to]);
  memcpy(b->J[to], b->J[from], sizeof b->J[to]);
  b->carpal_of[to] = b->carpal_of[from];
  b->room[to]      = b->room[from];
  b->give[to]      = b->give[from];
  b->side[to]      = b->side[from];
  b->patch[to]     = b->patch[from];
  b->body[to]      = b->body[from];
  b->corner[to]    = b->corner[from];
}

/** The seat's lowest end over the floor (its capsules' ends turned, less their radii, Python's min of them in their
    order), m. */
static double lowest(const wbc_borne_t *b, double floor)
{
  const int     k = b->seat_skin;
  const double *R = b->xmat + 9 * b->seat_body, *x = b->xpos + 3 * b->seat_body;
  double        low = 0.0;

  for (int c = 0; c < b->caps[k]; c++)
  {
    for (int e = 0; e < 2; e++)
    {
      double turned[3];

      mv3(b->blas, R, e ? b->cap_b[k][c] : b->cap_a[k][c], turned);
      const double y = (x[1] + turned[1]) - b->cap_r[k][c];

      low = (c == 0 && e == 0) || y < low ? y : low;
    }
  }
  return low - floor;
}

int wbc_borne_seated(wbc_borne_t *b, const double *points, const int64_t *rows, int npoints, int row, double told,
                     double floor, double emancipated, double sat, const double *seat_f, const double *v)
{
  const wbc_blas_t *l = b->blas;
  const int         at = b->seat_body;
  double            on = 0.0, near = 0.0;

  if (told > 0.0)
  {
    const double low = lowest(b, floor);
    int          felt = 0;

    for (int c = 0; c < b->nc; c++)
    {
      felt |= b->body[c] == at;
    }
    const double touching = (double)(low <= b->touch_m), reached = (double)felt * (double)(low <= b->reach_m);

    on   = (touching > on ? touching : on);
    on   = (reached > on ? reached : on) * (double)(emancipated <= b->shed0);
    near = (double)(low <= b->reach_m) > near ? (double)(low <= b->reach_m) : near;
  }
  b->seat_on   = on;
  b->seat_near = near;
  if (sat <= 0.5 || !(told > 0.0))
  {
    return 0;
  }
  const double *P = b->xmat + 9 * b->pelvis_body;
  double        ahead[3] = {P[2] * 1.0, P[5] * 0.0, P[8] * 1.0}, across[3], mid[3] = {0.0, 0.0, 0.0};
  double        felt = 0.0, give = 0.0;
  int           kept = 0, gave = 0, count = 0;
  const double  norm = sqrt(wbc_dot(l, ahead, 1, ahead, 1, 3));

  for (int i = 0; i < 3; i++)
  {
    ahead[i] = ahead[i] / norm;
  }
  across[0] = 1.0 * ahead[2] - 0.0 * ahead[1];
  across[1] = 0.0 * ahead[0] - 0.0 * ahead[2];
  across[2] = 0.0 * ahead[1] - 1.0 * ahead[0];
  for (int c = 0; c < b->nc; c++)
  {
    if (b->body[c] != at)
    {
      continue;
    }
    felt = felt + b->f[c][1];
    give = !gave || b->give[c] > give ? b->give[c] : give;
    gave = 1;
  }
  give = !gave || 0.0 > give ? 0.0 : give;
  for (int c = 0; c < b->nc; c++)
  {
    if (b->body[c] != at)
    {
      if (kept != c)
      {
        moved(b, c, kept);
      }
      kept++;
    }
  }
  b->nc = kept;
  for (int q = 0; q < npoints; q++)
  {
    if (rows[q] != row)
    {
      continue;
    }
    for (int i = 0; i < 3; i++)
    {
      mid[i] = count ? mid[i] + points[3 * q + i] : points[3 * q + i];
    }
    count++;
  }
  for (int i = 0; i < 3; i++)
  {
    mid[i] = mid[i] / (double)count;
  }
  mid[1] = floor;
  for (int corner = 0; corner < 4; corner++)
  {
    const double     dx = b->seat_dx[corner / 2], dz = b->seat_dz[corner % 2], quarter = 0.25 * felt;
    const double     bears = seat_f[corner] > quarter ? seat_f[corner] : quarter;
    const wbc_view_t jrows = {b->jp, b->nv, 1}, vel = {v, 1, 1};
    double           p[3];

    if (b->nc >= WBC_BORNE_CONTACTS)
    {
      return -1;
    }
    const int c = b->nc++;

    for (int i = 0; i < 3; i++)
    {
      p[i] = (mid[i] + dx * ahead[i]) + dz * across[i];
    }
    b->jac(b->m, b->d, b->jp, b->jr, p, at);
    for (int r = 0; r < 3; r++)
    {
      for (int j = 0; j < b->n; j++)
      {
        b->J[c][r][j] = b->jp[r * b->nv + j];
      }
    }
    b->jac_dot(b->m, b->d, b->jp, b->jr, p, at);
    wbc_matmul(l, jrows, vel, b->dJv[c], 1, 1, 3, b->n, 1);
    for (int i = 0; i < 3; i++)
    {
      b->p[c][i] = p[i];
    }
    b->f[c][0] = 0.0;
    b->f[c][1] = bears;
    b->f[c][2] = 0.0;
    b->carpal_of[c] = 0.0;
    b->side[c]      = 0;
    b->patch[c]     = 2;
    b->room[c]      = 0.0;
    b->body[c]      = at;
    b->give[c]      = give;
    b->corner[c]    = corner;
  }
  return 0;
}

int wbc_borne_supports(const wbc_borne_t *b, int np_, const unsigned char *said, const double *points,
                       const double *rooms, const double *corners0, int nc0, const double *corners1, int nc1,
                       const double *loads, const double *legs, double touch_n, int cap, double *pts, double *room,
                       double *soles)
{
  const double *corners[2] = {corners0, corners1};
  const int     ncs[2] = {nc0, nc1};
  int           felt = 0;
  int           kept = 0;
  int           n;
  int           at = 0;

  for (int c = 0; c < b->nc; c++)
  {
    felt += (b->patch[c] < b->let) ? 1 : 0;
  }
  for (int p = 0; p < np_; p++)
  {
    kept += (said[p] != 0U) ? 1 : 0;
  }
  n = ((felt + kept) + nc0) + nc1;
  if (n > cap)
  {
    return -1;
  }
  /* her skin's contacts, her observer's points told, her soles' corners: numpy's concatenation's order */
  for (int c = 0; c < b->nc; c++)
  {
    if (b->patch[c] < b->let)
    {
      pts[2 * at]     = b->p[c][0];
      pts[2 * at + 1] = b->p[c][2];
      room[at]        = b->room[c];
      at++;
    }
  }
  for (int p = 0; p < np_; p++)
  {
    if (said[p] != 0U)
    {
      pts[2 * at]     = points[3 * p];
      pts[2 * at + 1] = points[3 * p + 2];
      room[at]        = rooms[p];
      at++;
    }
  }
  for (int k = 0; k < 2; k++)
  {
    for (int j = 0; j < ncs[k]; j++)
    {
      pts[2 * at]     = corners[k][3 * j];
      pts[2 * at + 1] = corners[k][3 * j + 2];
      room[at]        = 0.0;
      at++;
    }
  }
  for (int i = 0; i < 2 * n; i++)
  {
    soles[i] = 0.0;
  }
  at = felt + kept;
  for (int k = 0; k < 2; k++)
  {
    const double stood = ((loads[k] >= touch_n) && (legs[k] < 1.0)) ? 1.0 : 0.0;

    for (int j = 0; j < ncs[k]; j++)
    {
      soles[k * n + at + j]       = 1.0;
      soles[(1 - k) * n + at + j] = 1.0 - stood;
    }
    at += ncs[k];
  }
  return n;
}
