/** vision.c - C11 SIMD L2-wavefront stereo SGM disparity and 3D voxel raycast/scatter kernels. */
#include "vision.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

void vs_grey_u8(const uint8_t *restrict rgb, int64_t *restrict out, int32_t npix)
{
  for (int32_t i = 0; i < npix; i++)
  {
    const size_t o = (size_t)i * 3U;
    out[i] = ((int64_t)299 * (int64_t)rgb[o] + (int64_t)587 * (int64_t)rgb[o + 1U] +
              (int64_t)114 * (int64_t)rgb[o + 2U] + (int64_t)500) /
             (int64_t)1000;
  }
}

void vs_box_i64(const int64_t *restrict src, int64_t *restrict dst, int64_t *restrict hsum, int64_t *restrict vsum,
                int32_t height, int32_t width, int32_t rad)
{
  if (height <= 0 || width <= 0)
  {
    return;
  }
  for (int32_t y = 0; y < height; y++)
  {
    const int64_t *row = &src[(size_t)y * (size_t)width];
    int64_t       *hr = &hsum[(size_t)y * (size_t)width];
    int64_t        s = (int64_t)(rad + 1) * row[0];
    for (int32_t k = 1; k <= rad; k++)
    {
      s += row[(k < width) ? k : (width - 1)];
    }
    for (int32_t x = 0; x < width; x++)
    {
      hr[x] = s;
      const int32_t add_x = (x + 1 + rad < width) ? (x + 1 + rad) : (width - 1);
      const int32_t sub_x = (x - rad > 0) ? (x - rad) : 0;
      s += row[add_x] - row[sub_x];
    }
  }
  for (int32_t x = 0; x < width; x++)
  {
    int64_t s = (int64_t)(rad + 1) * hsum[x];
    for (int32_t k = 1; k <= rad; k++)
    {
      const int32_t ky = (k < height) ? k : (height - 1);
      s += hsum[(size_t)ky * (size_t)width + (size_t)x];
    }
    vsum[x] = s;
  }
  for (int32_t y = 0; y < height; y++)
  {
    int64_t       *dr = &dst[(size_t)y * (size_t)width];
    const int32_t  add_y = (y + 1 + rad < height) ? (y + 1 + rad) : (height - 1);
    const int32_t  sub_y = (y - rad > 0) ? (y - rad) : 0;
    const int64_t *ha = &hsum[(size_t)add_y * (size_t)width];
    const int64_t *hs = &hsum[(size_t)sub_y * (size_t)width];
    for (int32_t x = 0; x < width; x++)
    {
      dr[x] = vsum[x];
      vsum[x] += ha[x] - hs[x];
    }
  }
}

static void vs_census_rows(const int64_t *restrict g, uint64_t *restrict dark, uint64_t *restrict bright,
                           int32_t height, int32_t width, int32_t ya, int32_t yb, int32_t rx, int32_t ry,
                           int64_t quantum)
{
  for (int32_t y = ya; y < yb; y++)
  {
    for (int32_t x = 0; x < width; x++)
    {
      const size_t  idx = (size_t)y * (size_t)width + (size_t)x;
      const int64_t c = g[idx];
      uint64_t      dk = 0U, bt = 0U;
      for (int32_t dy = -ry; dy <= ry; dy++)
      {
        const int32_t yy = vs_clamp_i32(y + dy, 0, height - 1);
        const size_t  rbase = (size_t)yy * (size_t)width;
        for (int32_t dx = -rx; dx <= rx; dx++)
        {
          if (dx == 0 && dy == 0)
          {
            continue;
          }
          const int32_t xx = vs_clamp_i32(x + dx, 0, width - 1);
          const int64_t n = g[rbase + (size_t)xx];
          dk = (dk << 1U) | (uint64_t)(n < (c - quantum));
          bt = (bt << 1U) | (uint64_t)(n > (c + quantum));
        }
      }
      dark[idx] = dk;
      bright[idx] = bt;
    }
  }
}

static inline void vs_sgm_step_u8(const uint8_t *restrict c_row, const float *restrict p_pad, float *restrict dst_pad,
                                  float *restrict s_row, int32_t d_count, int32_t d_stride, float p1, float p2)
{
  float least = INFINITY;
  for (int32_t d = 1; d <= d_count; d++)
  {
    least = (p_pad[d] < least) ? p_pad[d] : least;
  }
  const float lp2 = least + p2;
  dst_pad[0] = INFINITY;
  for (int32_t d = 0; d < d_stride; d++)
  {
    const uint8_t cu = c_row[d];
    const float   c = (cu == 255U) ? INFINITY : (float)cu;
    const float   p0 = p_pad[d + 1];
    const float   dn = p_pad[d] + p1;
    const float   up = p_pad[d + 2] + p1;
    const float   m0 = (p0 < dn) ? p0 : dn;
    const float   m1 = (m0 < up) ? m0 : up;
    const float   p = (m1 < lp2) ? m1 : lp2;
    const float   l = c + p - least;
    dst_pad[d + 1] = l;
    s_row[d] += l;
  }
  for (int32_t d = d_count + 1; d <= d_stride + 1; d++)
  {
    dst_pad[d] = INFINITY;
  }
}

static void vs_diff_hsum_row(const int64_t *restrict gl_r, const int64_t *restrict gr_r, int64_t *restrict hrow,
                             int32_t wd, int32_t rad)
{
  const int64_t d0 = gl_r[0] - gr_r[0];
  int64_t       s = (int64_t)(rad + 1) * ((d0 < 0) ? -d0 : d0);
  for (int32_t k = 1; k <= rad; k++)
  {
    const int32_t ku = (k < wd) ? k : (wd - 1);
    const int64_t dk = gl_r[ku] - gr_r[ku];
    s += (dk < 0) ? -dk : dk;
  }
  for (int32_t u = 0; u < wd; u++)
  {
    hrow[u] = s;
    const int32_t au = (u + 1 + rad < wd) ? (u + 1 + rad) : (wd - 1);
    const int32_t su = (u - rad > 0) ? (u - rad) : 0;
    const int64_t da = gl_r[au] - gr_r[au];
    const int64_t ds = gl_r[su] - gr_r[su];
    s += ((da < 0) ? -da : da) - ((ds < 0) ? -ds : ds);
  }
}

typedef struct
{
  const int64_t *gl;
  const int64_t *gr;
  uint64_t      *dl;
  uint64_t      *bl;
  uint64_t      *dr;
  uint64_t      *br;
  uint8_t       *cost_u8;
  float         *total_hwd;
  float         *wave_buf;
  int64_t       *ring_all;
  int64_t       *vsum_all;
  int64_t       *best_out;
  float         *tbest_out;
  float         *second_out;
  int64_t       *seen_out;
  float         *c0_out;
  float         *lo_out;
  float         *hi_out;
  int32_t        height;
  int32_t        width;
  int32_t        d_count;
  int32_t        d_stride;
  int32_t        rx;
  int32_t        ry;
  int32_t        rad;
  int32_t        dx;
  int32_t        dy;
  int64_t        quantum;
  float          p1;
  float          p2;
} vs_sgm_ctx_t;

static void vs_census_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_sgm_ctx_t *c = (const vs_sgm_ctx_t *)raw;
  const int32_t       ya = (tid * c->height) / nt;
  const int32_t       yb = ((tid + 1) * c->height) / nt;
  vs_census_rows(c->gl, c->dl, c->bl, c->height, c->width, ya, yb, c->rx, c->ry, c->quantum);
  vs_census_rows(c->gr, c->dr, c->br, c->height, c->width, ya, yb, c->rx, c->ry, c->quantum);
}

static void vs_cost_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_sgm_ctx_t *c = (const vs_sgm_ctx_t *)raw;
  const int32_t       ya = (tid * c->height) / nt;
  const int32_t       yb = ((tid + 1) * c->height) / nt;
  const size_t        row_elems = (size_t)c->width * (size_t)c->d_stride;
  if (ya < yb)
  {
    memset(&c->total_hwd[(size_t)ya * row_elems], 0, (size_t)(yb - ya) * row_elems * sizeof(float));
  }
  for (int32_t y = ya; y < yb; y++)
  {
    const size_t rbase = (size_t)y * (size_t)c->width;
    for (int32_t x = 0; x < c->width; x++)
    {
      const int32_t  d_lim = (x + 1 < c->d_count) ? (x + 1) : c->d_count;
      const size_t   j = rbase + (size_t)x;
      uint8_t       *crow = &c->cost_u8[j * (size_t)c->d_stride];
      const uint64_t dlj = c->dl[j], blj = c->bl[j];
      for (int32_t d = 0; d < d_lim; d++)
      {
        const size_t k = j - (size_t)d;
        crow[d] = (uint8_t)(vs_popcount64(dlj ^ c->dr[k]) + vs_popcount64(blj ^ c->br[k]));
      }
      for (int32_t d = d_lim; d < c->d_stride; d++)
      {
        crow[d] = 255U;
      }
    }
  }
}

static void vs_horiz_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_sgm_ctx_t *c = (const vs_sgm_ctx_t *)raw;
  const int32_t       ya = (tid * c->height) / nt;
  const int32_t       yb = ((tid + 1) * c->height) / nt;
  float               pad_a[VS_MAX_D_STRIDE + 2], pad_b[VS_MAX_D_STRIDE + 2];

  for (int32_t y = ya; y < yb; y++)
  {
    float       *prev = pad_a, *cur = pad_b;
    int32_t      x = (c->dx > 0) ? 0 : (c->width - 1);
    const size_t b0 = ((size_t)y * (size_t)c->width + (size_t)x) * (size_t)c->d_stride;
    prev[0] = INFINITY;
    for (int32_t d = 0; d < c->d_stride; d++)
    {
      const uint8_t cu = c->cost_u8[b0 + (size_t)d];
      const float   cv = (cu == 255U) ? INFINITY : (float)cu;
      prev[d + 1] = cv;
      c->total_hwd[b0 + (size_t)d] += cv;
    }
    for (int32_t d = c->d_count + 1; d <= c->d_stride + 1; d++)
    {
      prev[d] = INFINITY;
    }
    for (int32_t step = 1; step < c->width; step++)
    {
      x += c->dx;
      const size_t base = ((size_t)y * (size_t)c->width + (size_t)x) * (size_t)c->d_stride;
      vs_sgm_step_u8(&c->cost_u8[base], prev, cur, &c->total_hwd[base], c->d_count, c->d_stride, c->p1, c->p2);
      float *tmp = prev;
      prev = cur;
      cur = tmp;
    }
  }
}

static void vs_chain_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_sgm_ctx_t *c = (const vs_sgm_ctx_t *)raw;
  const int32_t       n_chains = (c->dx == 0) ? c->width : (c->height + c->width - 1);
  const int32_t       ca = (tid * n_chains) / nt;
  const int32_t       cb = ((tid + 1) * n_chains) / nt;
  const size_t        pad_w = (size_t)(c->d_stride + 2);
  float              *chains = &c->wave_buf[(size_t)ca * pad_w];
  float               next_pad[VS_MAX_D_STRIDE + 2];

  if (ca >= cb)
  {
    return;
  }
  for (int32_t step = 0; step < c->height; step++)
  {
    const int32_t y = (c->dy > 0) ? step : (c->height - 1 - step);
    const size_t  rbase = (size_t)y * (size_t)c->width;
    int32_t       xa = ca, xb = cb;
    if (c->dx > 0)
    {
      xa = vs_clamp_i32(ca + step - (c->height - 1), 0, c->width);
      xb = vs_clamp_i32(cb + step - (c->height - 1), 0, c->width);
    }
    else if (c->dx < 0)
    {
      xa = vs_clamp_i32(ca - step, 0, c->width);
      xb = vs_clamp_i32(cb - step, 0, c->width);
    }
    for (int32_t x = xa; x < xb; x++)
    {
      const int32_t cid = (c->dx == 0) ? x : ((c->dx > 0) ? (x - step + (c->height - 1)) : (x + step));
      float        *slot = &chains[(size_t)(cid - ca) * pad_w];
      const size_t  base = (rbase + (size_t)x) * (size_t)c->d_stride;
      const int32_t px = x - c->dx;
      if (step == 0 || px < 0 || px >= c->width)
      {
        slot[0] = INFINITY;
        for (int32_t d = 0; d < c->d_stride; d++)
        {
          const uint8_t cu = c->cost_u8[base + (size_t)d];
          const float   cv = (cu == 255U) ? INFINITY : (float)cu;
          slot[d + 1] = cv;
          c->total_hwd[base + (size_t)d] += cv;
        }
        for (int32_t d = c->d_count + 1; d <= c->d_stride + 1; d++)
        {
          slot[d] = INFINITY;
        }
      }
      else
      {
        vs_sgm_step_u8(&c->cost_u8[base], slot, next_pad, &c->total_hwd[base], c->d_count, c->d_stride, c->p1, c->p2);
        memcpy(slot, next_pad, pad_w * sizeof(float));
      }
    }
  }
}

static void vs_select_window_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_sgm_ctx_t *c = (const vs_sgm_ctx_t *)raw;
  const int32_t       ya = (tid * c->height) / nt;
  const int32_t       yb = ((tid + 1) * c->height) / nt;
  const float         inf = INFINITY;
  if (ya >= yb)
  {
    return;
  }
  for (int32_t y = ya; y < yb; y++)
  {
    const size_t rbase = (size_t)y * (size_t)c->width;
    for (int32_t x = 0; x < c->width; x++)
    {
      const size_t i = rbase + (size_t)x;
      const float *trow = &c->total_hwd[i * (size_t)c->d_stride];
      float        least = inf;
      int32_t      b = 0;
      for (int32_t d = 0; d < c->d_count; d++)
      {
        const float v = trow[d];
        if (v != v)
        {
          b = d;
          break;
        }
        if (v < least)
        {
          least = v;
          b = d;
        }
      }
      c->best_out[i] = (int64_t)b;
      c->tbest_out[i] = trow[b];
      float sec = inf;
      for (int32_t d = 0; d < c->d_count; d++)
      {
        if ((d - b > 2) || (b - d > 2))
        {
          const float v = trow[d];
          if (v != v)
          {
            sec = v;
            break;
          }
          if (v < sec)
          {
            sec = v;
          }
        }
      }
      c->second_out[i] = sec;
      c->c0_out[i] = inf;
      c->lo_out[i] = inf;
      c->hi_out[i] = inf;
      float   s_least = inf;
      int64_t at = 0;
      for (int32_t d = 0; d < c->d_count; d++)
      {
        const float v = (x + d < c->width) ? c->total_hwd[(rbase + (size_t)(x + d)) * (size_t)c->d_stride + (size_t)d] : inf;
        if (v != v)
        {
          at = (int64_t)d;
          break;
        }
        if (v < s_least)
        {
          s_least = v;
          at = (int64_t)d;
        }
      }
      c->seen_out[i] = at;
    }
  }
  const int32_t ring_rows = 2 * c->rad + 2;
  int64_t      *ring_buf = &c->ring_all[(size_t)tid * (size_t)ring_rows * (size_t)c->width];
  int64_t      *vsum_buf = &c->vsum_all[(size_t)tid * (size_t)c->width];
  const int32_t init_lo = vs_clamp_i32(ya - c->rad, 0, c->height - 1);
  const int32_t init_hi = vs_clamp_i32(ya + c->rad, 0, c->height - 1);
  for (int32_t d = 0; d < c->d_count; d++)
  {
    const int32_t wd = c->width - d;
    if (wd <= 0)
    {
      continue;
    }
    for (int32_t row_y = init_lo; row_y <= init_hi; row_y++)
    {
      vs_diff_hsum_row(&c->gl[(size_t)row_y * (size_t)c->width + (size_t)d],
                       &c->gr[(size_t)row_y * (size_t)c->width],
                       &ring_buf[(size_t)(row_y % ring_rows) * (size_t)c->width], wd, c->rad);
    }
    for (int32_t u = 0; u < wd; u++)
    {
      vsum_buf[u] = 0;
    }
    for (int32_t k = -c->rad; k <= c->rad; k++)
    {
      const int32_t  ky = vs_clamp_i32(ya + k, 0, c->height - 1);
      const int64_t *rk = &ring_buf[(size_t)(ky % ring_rows) * (size_t)c->width];
      for (int32_t u = 0; u < wd; u++)
      {
        vsum_buf[u] += rk[u];
      }
    }
    for (int32_t y = ya; y < yb; y++)
    {
      const size_t rbase = (size_t)y * (size_t)c->width;
      for (int32_t u = 0; u < wd; u++)
      {
        const size_t  idx = rbase + (size_t)(u + d);
        const int64_t b = c->best_out[idx];
        const int64_t b_lo = (b > 0) ? (b - 1) : 0;
        const int64_t b_hi = (b + 1 < (int64_t)c->d_count) ? (b + 1) : (int64_t)(c->d_count - 1);
        const float   val = (float)vsum_buf[u];
        if ((int64_t)d == b)
        {
          c->c0_out[idx] = val;
        }
        if ((int64_t)d == b_lo)
        {
          c->lo_out[idx] = val;
        }
        if ((int64_t)d == b_hi)
        {
          c->hi_out[idx] = val;
        }
      }
      if (y + 1 < yb)
      {
        const int32_t next_ry = y + 1 + c->rad;
        if (next_ry < c->height)
        {
          vs_diff_hsum_row(&c->gl[(size_t)next_ry * (size_t)c->width + (size_t)d],
                           &c->gr[(size_t)next_ry * (size_t)c->width],
                           &ring_buf[(size_t)(next_ry % ring_rows) * (size_t)c->width], wd, c->rad);
        }
        const int32_t  add_y = (next_ry < c->height) ? next_ry : (c->height - 1);
        const int32_t  sub_y = (y - c->rad > 0) ? (y - c->rad) : 0;
        const int64_t *ra = &ring_buf[(size_t)(add_y % ring_rows) * (size_t)c->width];
        const int64_t *rs = &ring_buf[(size_t)(sub_y % ring_rows) * (size_t)c->width];
        for (int32_t u = 0; u < wd; u++)
        {
          vsum_buf[u] += ra[u] - rs[u];
        }
      }
    }
  }
}

int32_t vs_stereo_core(const int64_t *restrict gl, const int64_t *restrict gr, int32_t height, int32_t width,
                       int32_t d_count, int32_t d_stride, int32_t rx, int32_t ry, int64_t quantum, int32_t rad,
                       float p1, float p2, uint64_t *restrict census_buf, uint8_t *restrict cost_u8,
                       float *restrict total_hwd, float *restrict wave_buf, int64_t *restrict ring_buf,
                       int64_t *restrict vsum_buf, int64_t *restrict best_out, float *restrict tbest_out,
                       float *restrict second_out, int64_t *restrict seen_out, float *restrict c0_out,
                       float *restrict lo_out, float *restrict hi_out)
{
  static const int32_t paths[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
  const size_t         npix = (size_t)height * (size_t)width;

  if (d_stride > VS_MAX_D_STRIDE || d_count <= 0 || d_count > d_stride || height <= 0 || width <= 0)
  {
    return -1;
  }
  vs_sgm_ctx_t ctx = {
    .gl = gl,
    .gr = gr,
    .dl = census_buf,
    .bl = census_buf + npix,
    .dr = census_buf + 2U * npix,
    .br = census_buf + 3U * npix,
    .cost_u8 = cost_u8,
    .total_hwd = total_hwd,
    .wave_buf = wave_buf,
    .ring_all = ring_buf,
    .vsum_all = vsum_buf,
    .best_out = best_out,
    .tbest_out = tbest_out,
    .second_out = second_out,
    .seen_out = seen_out,
    .c0_out = c0_out,
    .lo_out = lo_out,
    .hi_out = hi_out,
    .height = height,
    .width = width,
    .d_count = d_count,
    .d_stride = d_stride,
    .rx = rx,
    .ry = ry,
    .rad = rad,
    .quantum = quantum,
    .p1 = p1,
    .p2 = p2,
  };
  const int32_t nt = (npix >= 4096U) ? vs_threads_acquire(VS_MAX_THREADS) : 1;
  vs_parallel(nt, vs_census_task, &ctx);
  vs_parallel(nt, vs_cost_task, &ctx);
  for (int32_t p = 0; p < 8; p++)
  {
    ctx.dx = paths[p][0];
    ctx.dy = paths[p][1];
    vs_parallel(nt, (ctx.dy == 0) ? vs_horiz_task : vs_chain_task, &ctx);
  }
  vs_parallel(nt, vs_select_window_task, &ctx);
  vs_threads_release(nt);
  return 0;
}

void vs_voxel_add(const double *restrict xyz, const float *restrict rgb, int32_t n_pts, const double *restrict eye,
                  int32_t nx, int32_t ny, int32_t nz, double ox, double floor_m, double oz, double voxel,
                  double ceil_m, double near_m, double far_m, double step_vox, int32_t hits_min, int32_t told_min,
                  int32_t *restrict hits, int32_t *restrict told, int32_t *restrict free_arr, double *restrict top,
                  double *restrict moments, float *restrict colour, uint8_t *restrict occ_buf,
                  uint8_t *restrict crossed_buf, int32_t *restrict counts_buf, int64_t *restrict out_flat)
{
  const size_t nvox = (size_t)nx * (size_t)ny * (size_t)nz;
  const double ex = eye[0], ey = eye[1], ez = eye[2];
  const double step_m = step_vox * voxel;

  for (size_t f = 0U; f < nvox; f++)
  {
    occ_buf[f] = (uint8_t)((told[f] >= told_min) && (told[f] > free_arr[f]));
  }
  for (int32_t p = 0; p < n_pts; p++)
  {
    const double px = xyz[(size_t)p * 3U + 0U];
    const double py = xyz[(size_t)p * 3U + 1U];
    const double pz = xyz[(size_t)p * 3U + 2U];
    const double dx = px - ex, dy = py - ey, dz = pz - ez;
    const double away = sqrt((dx * dx + dy * dy) + dz * dz);
    if (!((py > floor_m) && (py < ceil_m) && (away > near_m) && (away < far_m)))
    {
      continue;
    }
    const double last = away - voxel;
    const double reach = (away > 1e-6) ? away : 1e-6;
    for (int32_t s = 0;; s++)
    {
      const double t = (double)s * step_m;
      if (!(t < last))
      {
        break;
      }
      const double  sc = t / reach;
      const double  ax = ex + dx * sc, ay = ey + dy * sc, az = ez + dz * sc;
      const int64_t ii = (int64_t)floor((ax - ox) / voxel);
      const int64_t jj = (int64_t)floor((ay - floor_m) / voxel);
      const int64_t kk = (int64_t)floor((az - oz) / voxel);
      if (ii >= 0 && ii < (int64_t)nx && jj >= 0 && jj < (int64_t)ny && kk >= 0 && kk < (int64_t)nz)
      {
        const size_t f = ((size_t)ii * (size_t)ny + (size_t)jj) * (size_t)nz + (size_t)kk;
        if (occ_buf[f] != 0U)
        {
          break;
        }
        const double tp = top[f];
        if ((ay <= tp) || !isfinite(tp))
        {
          crossed_buf[f] = 1U;
        }
      }
    }
  }
  for (size_t f = 0U; f < nvox; f++)
  {
    if (crossed_buf[f] != 0U)
    {
      free_arr[f] += 1;
      crossed_buf[f] = 0U;
    }
  }
  for (int32_t p = 0; p < n_pts; p++)
  {
    const double px = xyz[(size_t)p * 3U + 0U];
    const double py = xyz[(size_t)p * 3U + 1U];
    const double pz = xyz[(size_t)p * 3U + 2U];
    const double dx = px - ex, dy = py - ey, dz = pz - ez;
    const double away = sqrt((dx * dx + dy * dy) + dz * dz);
    const bool   ok = (py > floor_m) && (py < ceil_m) && (away > near_m) && (away < far_m);
    if (!ok)
    {
      out_flat[p] = -1;
      continue;
    }
    const int64_t ii = (int64_t)floor((px - ox) / voxel);
    const int64_t jj = (int64_t)floor((py - floor_m) / voxel);
    const int64_t kk = (int64_t)floor((pz - oz) / voxel);
    const bool    inside = (ii >= 0 && ii < (int64_t)nx && jj >= 0 && jj < (int64_t)ny && kk >= 0 && kk < (int64_t)nz);
    if (inside)
    {
      const size_t f = ((size_t)ii * (size_t)ny + (size_t)jj) * (size_t)nz + (size_t)kk;
      out_flat[p] = (int64_t)f;
      hits[f] += 1;
      counts_buf[f] += 1;
      if (py > top[f])
      {
        top[f] = py;
      }
      double *m = &moments[f * 7U];
      m[0] += 1.0;
      m[1] += px;
      m[2] += px * px;
      m[3] += pz;
      m[4] += pz * pz;
      m[5] += py;
      m[6] += py * py;
      float *col = &colour[f * 3U];
      col[0] += rgb[(size_t)p * 3U + 0U];
      col[1] += rgb[(size_t)p * 3U + 1U];
      col[2] += rgb[(size_t)p * 3U + 2U];
    }
    else
    {
      out_flat[p] = -1;
    }
  }
  for (size_t f = 0U; f < nvox; f++)
  {
    if (counts_buf[f] != 0)
    {
      if (counts_buf[f] >= hits_min)
      {
        told[f] += 1;
      }
      counts_buf[f] = 0;
    }
  }
}
