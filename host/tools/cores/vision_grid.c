/** vision_grid.c - C11 2D convex hull and 1D/2D/3D grid connected-components union-find kernels. */
#include "vision.h"

#include <math.h>
#include <stdbool.h>

static inline bool vs_xy_less(double ax, double ay, double bx, double by)
{
  return (ax < bx) || ((ax == bx) && (ay < by));
}

static void vs_sift_xy(double *xy, int32_t root, int32_t end)
{
  while (2 * root + 1 < end)
  {
    int32_t child = 2 * root + 1;
    if ((child + 1 < end) && vs_xy_less(xy[2 * child], xy[2 * child + 1], xy[2 * (child + 1)], xy[2 * (child + 1) + 1]))
    {
      child++;
    }
    if (!vs_xy_less(xy[2 * root], xy[2 * root + 1], xy[2 * child], xy[2 * child + 1]))
    {
      return;
    }
    const double tx = xy[2 * root], ty = xy[2 * root + 1];
    xy[2 * root] = xy[2 * child];
    xy[2 * root + 1] = xy[2 * child + 1];
    xy[2 * child] = tx;
    xy[2 * child + 1] = ty;
    root = child;
  }
}

static inline double vs_cross_xy(const double *o, const double *a, const double *b)
{
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

int32_t vs_convex_f64(const double *restrict pts_in, int32_t n_pts, double *restrict work_xy, double *restrict out_xy)
{
  if ((n_pts <= 0) || (pts_in == NULL) || (work_xy == NULL) || (out_xy == NULL))
  {
    return 0;
  }
  for (int32_t i = 0; i < n_pts; i++)
  {
    double x = pts_in[2 * i], y = pts_in[2 * i + 1];
    if (!isfinite(x) || !isfinite(y))
    {
      return -1;
    }
    work_xy[2 * i] = (x == 0.0) ? 0.0 : x;
    work_xy[2 * i + 1] = (y == 0.0) ? 0.0 : y;
  }
  for (int32_t start = (n_pts - 2) / 2; start >= 0; start--)
  {
    vs_sift_xy(work_xy, start, n_pts);
  }
  for (int32_t end = n_pts - 1; end > 0; end--)
  {
    const double tx = work_xy[0], ty = work_xy[1];
    work_xy[0] = work_xy[2 * end];
    work_xy[1] = work_xy[2 * end + 1];
    work_xy[2 * end] = tx;
    work_xy[2 * end + 1] = ty;
    vs_sift_xy(work_xy, 0, end);
  }
  int32_t u = 1;
  for (int32_t i = 1; i < n_pts; i++)
  {
    if ((work_xy[2 * i] != work_xy[2 * (u - 1)]) || (work_xy[2 * i + 1] != work_xy[2 * (u - 1) + 1]))
    {
      work_xy[2 * u] = work_xy[2 * i];
      work_xy[2 * u + 1] = work_xy[2 * i + 1];
      u++;
    }
  }
  if (u < 3)
  {
    for (int32_t i = 0; i < 2 * u; i++)
    {
      out_xy[i] = work_xy[i];
    }
    return u;
  }
  int32_t k = 0;
  for (int32_t i = 0; i < u; i++)
  {
    const double *p = &work_xy[2 * i];
    while ((k >= 2) && (vs_cross_xy(&out_xy[2 * (k - 2)], &out_xy[2 * (k - 1)], p) <= 0.0))
    {
      k--;
    }
    out_xy[2 * k] = p[0];
    out_xy[2 * k + 1] = p[1];
    k++;
  }
  const int32_t lower_top = k;
  for (int32_t i = u - 1; i >= 0; i--)
  {
    const double *p = &work_xy[2 * i];
    while ((k >= lower_top + 2) && (vs_cross_xy(&out_xy[2 * (k - 2)], &out_xy[2 * (k - 1)], p) <= 0.0))
    {
      k--;
    }
    out_xy[2 * k] = p[0];
    out_xy[2 * k + 1] = p[1];
    k++;
  }
  const int32_t lower_keep = lower_top - 1;
  const int32_t upper_keep = k - lower_top - 1;
  for (int32_t i = 0; i < upper_keep; i++)
  {
    out_xy[2 * (lower_keep + i)] = out_xy[2 * (lower_top + i)];
    out_xy[2 * (lower_keep + i) + 1] = out_xy[2 * (lower_top + i) + 1];
  }
  return lower_keep + upper_keep;
}

static inline void vs_write_coord(int64_t *coords_out, int32_t ndim, int32_t stride, int32_t dst, int32_t i0,
                                  int32_t i1, int32_t i2)
{
  if (ndim == 3)
  {
    coords_out[0 * stride + dst] = (int64_t)i0;
    coords_out[1 * stride + dst] = (int64_t)i1;
    coords_out[2 * stride + dst] = (int64_t)i2;
  }
  else if (ndim == 2)
  {
    coords_out[0 * stride + dst] = (int64_t)i1;
    coords_out[1 * stride + dst] = (int64_t)i2;
  }
  else
  {
    coords_out[dst] = (int64_t)i2;
  }
}

int32_t vs_grid_cc(const uint8_t *restrict on_u8, const int64_t *restrict codes, int32_t ndim, int32_t d0, int32_t d1,
                   int32_t d2, const int32_t *restrict offs3, int32_t n_off, int32_t *restrict pos_buf,
                   int32_t *restrict parent_buf, int32_t *restrict aux_buf, int64_t *restrict coords_out,
                   int32_t *restrict label_out, int32_t *restrict starts_out)
{
  if ((d0 <= 0) || (d1 <= 0) || (d2 <= 0) || (n_off < 0) || (ndim < 1) || (ndim > 3))
  {
    return 0;
  }
  int32_t k_act = 0;
  size_t  f = 0U;
  for (int32_t i0 = 0; i0 < d0; i0++)
  {
    for (int32_t i1 = 0; i1 < d1; i1++)
    {
      for (int32_t i2 = 0; i2 < d2; i2++, f++)
      {
        const bool active = (codes != NULL) ? (codes[f] >= 0) : (on_u8[f] != 0U);
        if (!active)
        {
          pos_buf[f] = -1;
          continue;
        }
        pos_buf[f] = k_act;
        parent_buf[k_act] = k_act;
        aux_buf[k_act] = (int32_t)f;
        k_act++;
      }
    }
  }
  if (k_act == 0)
  {
    return 0;
  }
  for (int32_t p = 0; p < k_act; p++)
  {
    const int32_t fp = aux_buf[p];
    const int32_t i2 = fp % d2;
    const int32_t t = fp / d2;
    const int32_t i1 = t % d1;
    const int32_t i0 = t / d1;
    for (int32_t o = 0; o < n_off; o++)
    {
      const int32_t n0 = i0 + offs3[3 * o + 0];
      const int32_t n1 = i1 + offs3[3 * o + 1];
      const int32_t n2 = i2 + offs3[3 * o + 2];
      if (((uint32_t)n0 < (uint32_t)d0) && ((uint32_t)n1 < (uint32_t)d1) && ((uint32_t)n2 < (uint32_t)d2))
      {
        const size_t  nf = ((size_t)n0 * (size_t)d1 + (size_t)n1) * (size_t)d2 + (size_t)n2;
        const int32_t q = pos_buf[nf];
        if ((q >= 0) && ((codes == NULL) || (codes[(size_t)fp] == codes[nf])))
        {
          int32_t ra = p, rb = q;
          while (parent_buf[ra] != ra)
          {
            parent_buf[ra] = parent_buf[parent_buf[ra]];
            ra = parent_buf[ra];
          }
          while (parent_buf[rb] != rb)
          {
            parent_buf[rb] = parent_buf[parent_buf[rb]];
            rb = parent_buf[rb];
          }
          if (ra < rb)
          {
            parent_buf[rb] = ra;
          }
          else if (rb < ra)
          {
            parent_buf[ra] = rb;
          }
        }
      }
    }
  }
  for (int32_t p = 0; p < k_act; p++)
  {
    parent_buf[p] = parent_buf[parent_buf[p]];
    label_out[p] = parent_buf[p];
  }
  if (codes != NULL)
  {
    for (int32_t p = 0; p < k_act; p++)
    {
      const int32_t fp = aux_buf[p];
      const int32_t i2 = fp % d2;
      const int32_t t = fp / d2;
      vs_write_coord(coords_out, ndim, k_act, p, t / d1, t % d1, i2);
    }
    return k_act;
  }
  int32_t n_comp = 0;
  for (int32_t p = 0; p < k_act; p++)
  {
    if (parent_buf[p] == p)
    {
      pos_buf[p] = n_comp;
      starts_out[n_comp] = 0;
      n_comp++;
    }
    starts_out[pos_buf[parent_buf[p]]] += 1;
  }
  int32_t acc = 0;
  for (int32_t g = 0; g < n_comp; g++)
  {
    const int32_t cnt = starts_out[g];
    starts_out[g] = acc;
    label_out[g] = acc;
    acc += cnt;
  }
  starts_out[n_comp] = acc;
  for (int32_t p = 0; p < k_act; p++)
  {
    const int32_t fp = aux_buf[p];
    const int32_t i2 = fp % d2;
    const int32_t t = fp / d2;
    const int32_t dst = label_out[pos_buf[parent_buf[p]]]++;
    vs_write_coord(coords_out, ndim, k_act, dst, t / d1, t % d1, i2);
  }
  return n_comp;
}
