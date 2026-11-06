/** wbc_registry.c - her map's look (wbc_registry.h): machine.eyes.registry's passed and seen past its things'
    classes over its slots' fields, numpy's arithmetic operation for operation (wbc_np.h), its OpenBLAS for each
    product (wbc_blas.h). Portable C11, the host's alone. */
#include "wbc_registry.h"
#include "wbc_np.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/** An einsum's three terms summed as numpy sums them here (`orders.summed`): from zero in turn, or in `lanes` SIMD
    lanes summed across. */
static double sum3(double p0, double p1, double p2, int lanes)
{
  if (lanes == 1)
  {
    return ((0.0 + p0) + p1) + p2;
  }
  if (lanes == 2)
  {
    return 0.0 + (((0.0 + p0) + p2) + ((0.0 + p1) + 0.0));
  }
  return 0.0 + (((0.0 + p0) + (0.0 + p2)) + ((0.0 + p1) + (0.0 + 0.0)));
}

/** registry._eye: her eye's place `eye` and her head's rotation `turn` of her head at `pos` turned by `quat`. */
static void eye_of(const wbc_registry_t *r, const double *pos, const double *quat, double eye[3], double turn[9])
{
  const double     w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  const wbc_view_t vt = {turn, 3, 1}, ve = {r->eyes, 1, 1};
  double           o[3];

  turn[0] = 1.0 - 2.0 * (y * y + z * z);
  turn[1] = 2.0 * (x * y - w * z);
  turn[2] = 2.0 * (x * z + w * y);
  turn[3] = 2.0 * (x * y + w * z);
  turn[4] = 1.0 - 2.0 * (x * x + z * z);
  turn[5] = 2.0 * (y * z - w * x);
  turn[6] = 2.0 * (x * z - w * y);
  turn[7] = 2.0 * (y * z + w * x);
  turn[8] = 1.0 - 2.0 * (x * x + y * y);
  wbc_matmul(r->blas, vt, ve, o, 1, 1, 3, 3, 1);
  for (int i = 0; i < 3; i++)
  {
    eye[i] = pos[i] + o[i];
  }
}

/** registry.seen_of: whether each of `n` boxes (c, h (n, 3), R (n, 3, 3)) shows in her eyes' frustum - any of its
    corners, its centre or its point nearest her `eye` (`nearest`), her head turned `turn` (`shown`) -, into `got`. */
static void seen_of(wbc_registry_t *r, int n, const double *c, const double *h, const double *R, const double eye[3],
                    const double turn[9], int *got)
{
  const int        m = 10 * n;
  const wbc_view_t vp = {r->points[0], 3, 1}, vt = {turn, 3, 1}, vv = {r->v[0], 3, 1};
  const wbc_view_t va = {r->axis, 1, 1}, vu = {r->up, 1, 1};

  for (int a = 0; a < n; a++)
  {
    const double *ca = c + 3 * a, *ha = h + 3 * a, *Ra = R + 9 * a;
    double (*pa)[3] = r->points + 10 * a;
    double e[3], local[3];

    for (int k = 0; k < 8; k++)
    {
      double x[3];

      for (int j = 0; j < 3; j++)
      {
        x[j] = r->corners[k][j] * ha[j];
      }
      for (int i = 0; i < 3; i++)
      {
        pa[k][i] = ca[i] + sum3(Ra[3 * i] * x[0], Ra[3 * i + 1] * x[1], Ra[3 * i + 2] * x[2], r->lanes[0]);
      }
    }
    for (int j = 0; j < 3; j++)
    {
      pa[8][j] = ca[j];
      e[j]     = eye[j] - ca[j];
    }
    for (int i = 0; i < 3; i++)
    {
      local[i] = npclip(sum3(Ra[i] * e[0], Ra[3 + i] * e[1], Ra[6 + i] * e[2], r->lanes[1]), -ha[i], ha[i]);
    }
    for (int i = 0; i < 3; i++)
    {
      pa[9][i] = ca[i] + sum3(Ra[3 * i] * local[0], Ra[3 * i + 1] * local[1], Ra[3 * i + 2] * local[2], r->lanes[2]);
    }
  }
  for (int p = 0; p < m; p++)
  {
    for (int j = 0; j < 3; j++)
    {
      r->points[p][j] = r->points[p][j] - eye[j];
    }
  }
  wbc_matmul(r->blas, vp, vt, r->v[0], 3, 1, m, 3, 3);
  wbc_matmul(r->blas, vv, va, r->depth, 1, 1, m, 3, 1);
  wbc_matmul(r->blas, vv, vu, r->upv, 1, 1, m, 3, 1);
  for (int a = 0; a < n; a++)
  {
    int any = 0;

    for (int k = 10 * a; k < 10 * a + 10; k++)
    {
      const double d = r->depth[k];

      any |= (d >= r->near_m) & (d <= r->far_m) & (fabs(r->upv[k]) <= d * r->tan_up) &
             (fabs(r->v[k][0]) <= d * r->tan_across);
    }
    got[a] = any;
  }
}

/** The slots `at` (n) of the registry's boxes into its gathered scratch (`gc`, `gh`, `gR`), numpy's fancy index. */
static void gathered(wbc_registry_t *r, const int64_t *at, int n)
{
  for (int k = 0; k < n; k++)
  {
    memcpy(r->gc[k], r->centre + 3 * at[k], sizeof r->gc[k]);
    memcpy(r->gh[k], r->half + 3 * at[k], sizeof r->gh[k]);
    memcpy(r->gR[k], r->R + 9 * at[k], sizeof r->gR[k]);
  }
}

/** registry._w2 of a thing (c, h) to a slot (at, halves) over its variance `scale`. */
static double w2(const double c[3], const double h[3], const double at[3], const double halves[3], double scale)
{
  double d[3], e[3];

  for (int i = 0; i < 3; i++)
  {
    const double dx = at[i] - c[i], dh = halves[i] - h[i];

    d[i] = dx * dx;
    e[i] = dh * dh;
  }
  return (((d[0] + d[1]) + d[2]) + ((e[0] + e[1]) + e[2])) / scale;
}

/** numpy's argmin of `n` doubles: the first least, or the first nan. */
static int argmin(const double *a, int n)
{
  int at = 0;

  for (int j = 0; j < n; j++)
  {
    if (isnan(a[j]))
    {
      return j;
    }
    if (a[j] < a[at])
    {
      at = j;
    }
  }
  return at;
}

/** Thing `k` into live slot `j` (slot `i`): its drift, whether it moves, its velocity, the filter's gain and the
    slot's box, variance and salience - `_matched`'s arithmetic, or the loop's (`one`: its move @ move numpy's dot,
    Python's max). */
static void took(wbc_registry_t *r, int k, int j, int64_t i, double t, int one)
{
  const double since = t - r->seen_at[i];
  const double dt = one ? (1e-9 > since ? 1e-9 : since) : npmaximum(since, 1e-9);
  const double gained = (double)(r->fid[k] > 0 && r->fid[k] == r->lid[j]);
  double       move[3], moving, gain, kept;

  for (int x = 0; x < 3; x++)
  {
    move[x] = r->fc[k][x] - r->centre[3 * i + x];
  }
  r->drift[i] = r->drift[i] + (one ? wbc_dot(r->blas, move, 1, move, 1, 3)
                                   : ((move[0] * move[0] + move[1] * move[1]) + move[2] * move[2])) /
                                  (r->var[i] + r->cell2);
  moving = (double)(r->drift[i] > r->moving_chi2);
  kept   = moving + (1.0 - moving) * r->var[i] / (r->var[i] + r->cell2);
  gain   = one ? (gained > kept ? gained : kept) : npmaximum(kept, gained);
  r->moving[i] = moving;
  for (int x = 0; x < 3; x++)
  {
    r->vel[3 * i + x]    = moving * (r->fc[k][x] - r->last[3 * i + x]) / dt;
    r->centre[3 * i + x] = r->centre[3 * i + x] + gain * move[x];
    r->half[3 * i + x]   = r->half[3 * i + x] + gain * (r->fh[k][x] - r->half[3 * i + x]);
  }
  memcpy(r->R + 9 * i, r->fR[k], sizeof r->fR[k]);
  r->var[i]      = (1.0 - gain) * r->var[i] + moving * r->cell2;
  r->clear[i]    = r->fclear[k];
  r->salience[i] = r->salience[i] + r->hit;
  memcpy(r->last + 3 * i, r->fc[k], sizeof r->fc[k]);
  r->seen_at[i] = t;
  r->swell[i]   = 0.0;
  r->shown[i]   = 1.0;
}

/** Thing `k` laid into slot `i` anew: on, its box, a look's variance, one match's salience, still. */
static void laid(wbc_registry_t *r, int k, int64_t i, double t)
{
  r->on[i]   = 1.0;
  r->kind[i] = 0;
  memcpy(r->centre + 3 * i, r->fc[k], sizeof r->fc[k]);
  memcpy(r->half + 3 * i, r->fh[k], sizeof r->fh[k]);
  memcpy(r->R + 9 * i, r->fR[k], sizeof r->fR[k]);
  r->var[i]      = r->cell2;
  r->salience[i] = r->hit;
  memset(r->vel + 3 * i, 0, 3 * sizeof(double));
  memcpy(r->last + 3 * i, r->fc[k], sizeof r->fc[k]);
  r->seen_at[i] = t;
  r->swell[i]   = 0.0;
  r->drift[i]   = 0.0;
  r->moving[i]  = 0.0;
  r->shown[i]   = 1.0;
  r->id[i]      = r->fid[k];
  r->clear[i]   = r->fclear[k];
}

/** registry._missed: the live slots no thing took shown as her frustum holds them, their salience moved; the
    pruned let go. */
static void missed(wbc_registry_t *r, const int64_t *live, int nl, double t, const double eye[3], const double turn[9])
{
  int m = 0;

  for (int j = 0; j < nl; j++)
  {
    if (!r->matched[j])
    {
      r->rest[m++] = live[j];
    }
  }
  if (m)
  {
    gathered(r, r->rest, m);
    seen_of(r, m, r->gc[0], r->gh[0], r->gR[0], eye, turn, r->inside);
    for (int q = 0; q < m; q++)
    {
      const int64_t i    = r->rest[q];
      const int     kept = t - r->seen_at[i] <= r->kept_s;

      r->shown[i]    = (double)r->inside[q];
      r->salience[i] = r->salience[i] + r->miss * (double)(r->inside[q] | !kept);
    }
  }
  for (int s = 0; s < r->slots; s++)
  {
    if (r->on[s] > 0.0 && r->salience[s] < r->prune)
    {
      r->on[s] = 0.0;
    }
  }
}

void wbc_registry_passed(wbc_registry_t *r, const int64_t *live, int nl, const double *movable, const double *pos,
                         const double *quat, double dt)
{
  double eye[3], turn[9];

  for (int k = 0; k < nl; k++)
  {
    r->var[live[k]] = r->var[live[k]] + r->diffuse * dt * movable[k];
  }
  eye_of(r, pos, quat, eye, turn);
  gathered(r, live, nl);
  seen_of(r, nl, r->gc[0], r->gh[0], r->gR[0], eye, turn, r->inside);
  for (int k = 0; k < nl; k++)
  {
    const int64_t i = live[k];

    r->swell[i] = npminimum(r->swell_m, r->swell[i] + r->swell_mps * dt * (1.0 - (double)r->inside[k]));
  }
}

/** The look's things `seen` (n of them) looked at, laid into the scratch in their order: those laid. */
static int filtered(wbc_registry_t *r, int n, const double *centre, const double *half, const double *R,
                    const int64_t *ids, const double *clear, const int64_t *codes)
{
  int nf = 0;

  for (int k = 0; k < n; k++)
  {
    if (!r->looked[k])
    {
      continue;
    }
    memcpy(r->fc[nf], centre + 3 * k, sizeof r->fc[nf]);
    memcpy(r->fh[nf], half + 3 * k, sizeof r->fh[nf]);
    memcpy(r->fR[nf], R + 9 * k, sizeof r->fR[nf]);
    r->fid[nf]    = ids[k];
    r->fclear[nf] = clear[k];
    r->fcode[nf]  = codes[k];
    nf++;
  }
  return nf;
}

/** Each looked thing's distance to each live slot of its class (`_w2`, gated), 0 to its own id's slot and none to
    another where it has one: `seen`'s d2, row by row over `nl`. */
static void distances(wbc_registry_t *r, int nf, const int64_t *live, int nl, const int64_t *slot_codes)
{
  for (int k = 0; k < nf; k++)
  {
    double *row   = r->d2 + (ptrdiff_t)k * nl;
    int     known = 0;

    for (int j = 0; j < nl; j++)
    {
      known |= r->fid[k] > 0 && r->fid[k] == r->lid[j];
      row[j] = r->fcode[k] == slot_codes[j] ? w2(r->fc[k], r->fh[k], r->ahead[j], r->half + 3 * live[j], r->scale[j])
                                            : INFINITY;
    }
    for (int j = 0; known && j < nl; j++)
    {
      row[j] = r->fid[k] > 0 && r->fid[k] == r->lid[j] ? 0.0 : INFINITY;
    }
  }
}

/** `seen`'s loop where its things' nearest gated slots are not each their own: in turn, the nearest gated live slot
    taken, else a free slot, else the least salient's - its column met by the things after it at their new half
    sizes, its old place -; each new. */
static void looped(wbc_registry_t *r, int nf, const int64_t *live, int nl, const int64_t *slot_codes, double t)
{
  int nfree = 0, next = 0;

  for (int s = 0; s < r->slots; s++)
  {
    if (r->on[s] == 0.0)
    {
      r->freed[nfree++] = s;
    }
  }
  for (int k = 0; k < nf; k++)
  {
    const int b = nl ? argmin(r->d2 + (ptrdiff_t)k * nl, nl) : -1;
    int64_t   i;

    if (b >= 0 && r->d2[(ptrdiff_t)k * nl + b] <= r->gate)
    {
      i = live[b];
      r->matched[b] = 1;
      for (int q = 0; q < nf; q++)
      {
        r->d2[(ptrdiff_t)q * nl + b] = INFINITY;
      }
      r->slot_of[k] = (int)i;
      took(r, k, b, i, t, 1);
      continue;
    }
    if (next < nfree)
    {
      i = r->freed[next++];
    }
    else
    {
      i = 0;
      for (int s = 0; s < r->slots; s++)
      {
        const double here = r->on[s] > 0.0 ? r->salience[s] : INFINITY, least = r->on[i] > 0.0 ? r->salience[i] : INFINITY;

        if (here < least)
        {
          i = s;
        }
      }
      for (int j = 0; j < nl; j++)
      {
        if (live[j] != i || r->matched[j])
        {
          continue;
        }
        for (int q = k + 1; q < nf; q++)
        {
          r->d2[(ptrdiff_t)q * nl + j] =
              r->fcode[q] == slot_codes[j] ? w2(r->fc[q], r->fh[q], r->ahead[j], r->fh[k], r->scale[j]) : INFINITY;
        }
      }
    }
    laid(r, k, i, t);
    r->slot_of[k] = (int)i;
  }
}

int wbc_registry_seen(wbc_registry_t *r, int n, const double *centre, const double *half, const double *R,
                      const int64_t *ids, const double *clear, const int64_t *codes, const int64_t *live, int nl,
                      const int64_t *slot_codes, double t, int everywhere, const double *pos, const double *quat)
{
  double eye[3], turn[9];
  int    nf, fast = 1;

  if (n > WBC_REGISTRY_BOXES || nl > WBC_REGISTRY_BOXES)
  {
    return -1;
  }
  eye_of(r, pos, quat, eye, turn);
  if (everywhere)
  {
    for (int k = 0; k < n; k++)
    {
      r->looked[k] = 1;
    }
  }
  else if (n)
  {
    seen_of(r, n, centre, half, R, eye, turn, r->looked);
  }
  nf = filtered(r, n, centre, half, R, ids, clear, codes);
  for (int j = 0; j < nl; j++)
  {
    const int64_t i = live[j];

    for (int x = 0; x < 3; x++)
    {
      r->ahead[j][x] = r->centre[3 * i + x] + r->vel[3 * i + x] * (t - r->seen_at[i]);
    }
    r->scale[j]   = r->var[i] + r->cell2;
    r->lid[j]     = r->id[i];
    r->matched[j] = 0;
    r->taken[j]   = 0;
  }
  distances(r, nf, live, nl, slot_codes);
  if (nl && nf)
  {
    for (int k = 0; k < nf; k++)
    {
      const int b = argmin(r->d2 + (ptrdiff_t)k * nl, nl);

      r->best[k] = b;
      fast &= r->d2[(ptrdiff_t)k * nl + b] <= r->gate && !r->taken[b];
      r->taken[b] = 1;
    }
    if (fast)
    {
      for (int k = 0; k < nf; k++)
      {
        r->matched[r->best[k]] = 1;
        r->slot_of[k] = (int)live[r->best[k]];
        took(r, k, r->best[k], live[r->best[k]], t, 0);
      }
      missed(r, live, nl, t, eye, turn);
      return nf;
    }
  }
  looped(r, nf, live, nl, slot_codes, t);
  missed(r, live, nl, t, eye, turn);
  return nf;
}
