/** vision.c - C11 SIMD L2-wavefront stereo SGM disparity and 3D voxel raycast/scatter kernels. */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VS_MAX_D_STRIDE 128

static inline int32_t vs_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline int32_t vs_popcount64(uint64_t x)
{
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_popcountll(x);
#else
  x = x - ((x >> 1U) & 0x5555555555555555ULL);
  x = (x & 0x3333333333333333ULL) + ((x >> 2U) & 0x3333333333333333ULL);
  return (int32_t)((((x + (x >> 4U)) & 0x0F0F0F0F0F0F0F0FULL) * 0x0101010101010101ULL) >> 56U);
#endif
}

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

static void vs_census(const int64_t *restrict g, uint64_t *restrict dark, uint64_t *restrict bright, int32_t height,
                      int32_t width, int32_t rx, int32_t ry, int64_t quantum)
{
  for (int32_t y = 0; y < height; y++)
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

static inline void vs_sgm_step(const float *restrict c_row, const float *restrict p_pad, float *restrict dst_pad,
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
    const float p0 = p_pad[d + 1];
    const float dn = p_pad[d] + p1;
    const float up = p_pad[d + 2] + p1;
    const float m0 = (p0 < dn) ? p0 : dn;
    const float m1 = (m0 < up) ? m0 : up;
    const float p = (m1 < lp2) ? m1 : lp2;
    const float l = c_row[d] + p - least;
    dst_pad[d + 1] = l;
    s_row[d] += l;
  }
  for (int32_t d = d_count + 1; d <= d_stride + 1; d++)
  {
    dst_pad[d] = INFINITY;
  }
}

static void vs_sgm_horiz(const float *restrict cost, float *restrict total, int32_t height, int32_t width,
                         int32_t d_count, int32_t d_stride, int32_t dx, float p1, float p2)
{
  float pad_a[VS_MAX_D_STRIDE + 2], pad_b[VS_MAX_D_STRIDE + 2];
  for (int32_t y = 0; y < height; y++)
  {
    float  *prev = pad_a, *cur = pad_b;
    int32_t x = (dx > 0) ? 0 : (width - 1);
    const size_t b0 = ((size_t)y * (size_t)width + (size_t)x) * (size_t)d_stride;
    prev[0] = INFINITY;
    for (int32_t d = 0; d < d_stride; d++)
    {
      const float c = cost[b0 + (size_t)d];
      prev[d + 1] = c;
      total[b0 + (size_t)d] += c;
    }
    for (int32_t d = d_count + 1; d <= d_stride + 1; d++)
    {
      prev[d] = INFINITY;
    }
    for (int32_t step = 1; step < width; step++)
    {
      x += dx;
      const size_t base = ((size_t)y * (size_t)width + (size_t)x) * (size_t)d_stride;
      vs_sgm_step(&cost[base], prev, cur, &total[base], d_count, d_stride, p1, p2);
      float *tmp = prev;
      prev = cur;
      cur = tmp;
    }
  }
}

static void vs_sgm_vert_diag(const float *restrict cost, float *restrict total, float *restrict wave_a,
                             float *restrict wave_b, int32_t height, int32_t width, int32_t d_count, int32_t d_stride,
                             int32_t dx, int32_t dy, float p1, float p2)
{
  const size_t pad_w = (size_t)(d_stride + 2);
  float       *prev_row = wave_a, *cur_row = wave_b;

  for (int32_t step = 0; step < height; step++)
  {
    const int32_t y = (dy > 0) ? step : (height - 1 - step);
    const size_t  rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      const size_t base = (rbase + (size_t)x) * (size_t)d_stride;
      float       *dst = &cur_row[(size_t)x * pad_w];
      const int32_t px = x - dx;
      if (step == 0 || px < 0 || px >= width)
      {
        dst[0] = INFINITY;
        for (int32_t d = 0; d < d_stride; d++)
        {
          const float c = cost[base + (size_t)d];
          dst[d + 1] = c;
          total[base + (size_t)d] += c;
        }
        for (int32_t d = d_count + 1; d <= d_stride + 1; d++)
        {
          dst[d] = INFINITY;
        }
      }
      else
      {
        vs_sgm_step(&cost[base], &prev_row[(size_t)px * pad_w], dst, &total[base], d_count, d_stride, p1, p2);
      }
    }
    float *tmp = prev_row;
    prev_row = cur_row;
    cur_row = tmp;
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

int32_t vs_stereo_core(const int64_t *restrict gl, const int64_t *restrict gr, int32_t height, int32_t width,
                       int32_t d_count, int32_t d_stride, int32_t rx, int32_t ry, int64_t quantum, int32_t rad,
                       float p1, float p2, uint64_t *restrict census_buf, float *restrict cost_hwd,
                       float *restrict total_hwd, float *restrict wave_buf, int64_t *restrict ring_buf,
                       int64_t *restrict vsum_buf, int64_t *restrict best_out, float *restrict tbest_out,
                       float *restrict second_out, int64_t *restrict seen_out, float *restrict c0_out,
                       float *restrict lo_out, float *restrict hi_out)
{
  static const int32_t paths[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
  const size_t         npix = (size_t)height * (size_t)width;
  const float          inf = INFINITY;

  if (d_stride > VS_MAX_D_STRIDE || d_count <= 0 || d_count > d_stride || height <= 0 || width <= 0)
  {
    return -1;
  }
  uint64_t *dl = census_buf;
  uint64_t *bl = dl + npix;
  uint64_t *dr = bl + npix;
  uint64_t *br = dr + npix;
  vs_census(gl, dl, bl, height, width, rx, ry, quantum);
  vs_census(gr, dr, br, height, width, rx, ry, quantum);

  for (size_t i = 0U; i < npix * (size_t)d_stride; i++)
  {
    cost_hwd[i] = inf;
    total_hwd[i] = 0.0f;
  }
  for (int32_t y = 0; y < height; y++)
  {
    const size_t rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      const int32_t  d_lim = (x + 1 < d_count) ? (x + 1) : d_count;
      const size_t   j = rbase + (size_t)x;
      float         *crow = &cost_hwd[j * (size_t)d_stride];
      const uint64_t dlj = dl[j], blj = bl[j];
      for (int32_t d = 0; d < d_lim; d++)
      {
        const size_t k = j - (size_t)d;
        crow[d] = (float)(vs_popcount64(dlj ^ dr[k]) + vs_popcount64(blj ^ br[k]));
      }
    }
  }
  float *wave_a = wave_buf;
  float *wave_b = wave_buf + (size_t)width * (size_t)(d_stride + 2);
  for (int32_t p = 0; p < 8; p++)
  {
    if (paths[p][1] == 0)
    {
      vs_sgm_horiz(cost_hwd, total_hwd, height, width, d_count, d_stride, paths[p][0], p1, p2);
    }
    else
    {
      vs_sgm_vert_diag(cost_hwd, total_hwd, wave_a, wave_b, height, width, d_count, d_stride, paths[p][0], paths[p][1],
                       p1, p2);
    }
  }
  for (size_t i = 0U; i < npix; i++)
  {
    const float *trow = &total_hwd[i * (size_t)d_stride];
    float        least = inf;
    int32_t      b = 0;
    for (int32_t d = 0; d < d_count; d++)
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
    best_out[i] = (int64_t)b;
    tbest_out[i] = trow[b];
    float sec = inf;
    for (int32_t d = 0; d < d_count; d++)
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
    second_out[i] = sec;
    c0_out[i] = inf;
    lo_out[i] = inf;
    hi_out[i] = inf;
  }
  for (int32_t y = 0; y < height; y++)
  {
    const size_t rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      float   least = inf;
      int64_t at = 0;
      for (int32_t d = 0; d < d_count; d++)
      {
        const float v = (x + d < width) ? total_hwd[(rbase + (size_t)(x + d)) * (size_t)d_stride + (size_t)d] : inf;
        if (v != v)
        {
          at = (int64_t)d;
          break;
        }
        if (v < least)
        {
          least = v;
          at = (int64_t)d;
        }
      }
      seen_out[rbase + (size_t)x] = at;
    }
  }
  const int32_t ring_rows = 2 * rad + 2;
  for (int32_t d = 0; d < d_count; d++)
  {
    const int32_t wd = width - d;
    if (wd <= 0)
    {
      continue;
    }
    const int32_t init_max = (rad < height - 1) ? rad : (height - 1);
    for (int32_t ry = 0; ry <= init_max; ry++)
    {
      vs_diff_hsum_row(&gl[(size_t)ry * (size_t)width + (size_t)d], &gr[(size_t)ry * (size_t)width],
                       &ring_buf[(size_t)(ry % ring_rows) * (size_t)width], wd, rad);
    }
    const int64_t *r0 = &ring_buf[0];
    for (int32_t u = 0; u < wd; u++)
    {
      int64_t s = (int64_t)(rad + 1) * r0[u];
      for (int32_t k = 1; k <= rad; k++)
      {
        const int32_t ky = (k < height) ? k : (height - 1);
        s += ring_buf[(size_t)(ky % ring_rows) * (size_t)width + (size_t)u];
      }
      vsum_buf[u] = s;
    }
    for (int32_t y = 0; y < height; y++)
    {
      const size_t rbase = (size_t)y * (size_t)width;
      for (int32_t u = 0; u < wd; u++)
      {
        const size_t  idx = rbase + (size_t)(u + d);
        const int64_t b = best_out[idx];
        const int64_t b_lo = (b > 0) ? (b - 1) : 0;
        const int64_t b_hi = (b + 1 < (int64_t)d_count) ? (b + 1) : (int64_t)(d_count - 1);
        const float   val = (float)vsum_buf[u];
        if ((int64_t)d == b)
        {
          c0_out[idx] = val;
        }
        if ((int64_t)d == b_lo)
        {
          lo_out[idx] = val;
        }
        if ((int64_t)d == b_hi)
        {
          hi_out[idx] = val;
        }
      }
      const int32_t next_ry = y + 1 + rad;
      if (next_ry < height)
      {
        vs_diff_hsum_row(&gl[(size_t)next_ry * (size_t)width + (size_t)d], &gr[(size_t)next_ry * (size_t)width],
                         &ring_buf[(size_t)(next_ry % ring_rows) * (size_t)width], wd, rad);
      }
      const int32_t  add_y = (next_ry < height) ? next_ry : (height - 1);
      const int32_t  sub_y = (y - rad > 0) ? (y - rad) : 0;
      const int64_t *ra = &ring_buf[(size_t)(add_y % ring_rows) * (size_t)width];
      const int64_t *rs = &ring_buf[(size_t)(sub_y % ring_rows) * (size_t)width];
      for (int32_t u = 0; u < wd; u++)
      {
        vsum_buf[u] += ra[u] - rs[u];
      }
    }
  }
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
    const double  px = xyz[(size_t)p * 3U + 0U];
    const double  py = xyz[(size_t)p * 3U + 1U];
    const double  pz = xyz[(size_t)p * 3U + 2U];
    const double  dx = px - ex, dy = py - ey, dz = pz - ez;
    const double  away = sqrt((dx * dx + dy * dy) + dz * dz);
    const bool    ok = (py > floor_m) && (py < ceil_m) && (away > near_m) && (away < far_m);
    const int64_t ii = (int64_t)floor((px - ox) / voxel);
    const int64_t jj = (int64_t)floor((py - floor_m) / voxel);
    const int64_t kk = (int64_t)floor((pz - oz) / voxel);
    const bool inside = (ii >= 0 && ii < (int64_t)nx && jj >= 0 && jj < (int64_t)ny && kk >= 0 && kk < (int64_t)nz);
    if (ok && inside)
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

void vs_warp_u8(const uint8_t *restrict src, const int32_t *restrict at4,
                const int32_t *restrict wt4, const uint8_t *restrict on_mask, int32_t height,
                int32_t width, int32_t bits, uint8_t *restrict dst)
{
  const size_t  npix = (size_t)height * (size_t)width;
  const int32_t fw = 2 * width;
  const int32_t aw = fw - 1;
  const int64_t whole = ((int64_t)1 << (2 * bits)) * 4;
  const int64_t half = whole >> 1;

  for (size_t p = 0U; p < npix; p++)
  {
    const size_t o = p * 3U;
    if (on_mask[p] == 0U)
    {
      dst[o] = 0U;
      dst[o + 1U] = 0U;
      dst[o + 2U] = 0U;
      continue;
    }
    int64_t s0 = 0, s1 = 0, s2 = 0;
    for (size_t k = 0U; k < 4U; k++)
    {
      const int32_t idx = at4[k * npix + p];
      const int64_t w = (int64_t)wt4[k * npix + p];
      const int32_t r = idx / aw;
      const int32_t c = idx - r * aw;
      const size_t  p00 = ((size_t)r * (size_t)fw + (size_t)c) * 3U;
      const size_t  p10 = p00 + (size_t)fw * 3U;
      const int64_t a0 = (int64_t)src[p00] + (int64_t)src[p00 + 3U] + (int64_t)src[p10] + (int64_t)src[p10 + 3U];
      const int64_t a1 =
        (int64_t)src[p00 + 1U] + (int64_t)src[p00 + 4U] + (int64_t)src[p10 + 1U] + (int64_t)src[p10 + 4U];
      const int64_t a2 =
        (int64_t)src[p00 + 2U] + (int64_t)src[p00 + 5U] + (int64_t)src[p10 + 2U] + (int64_t)src[p10 + 5U];
      s0 += a0 * w;
      s1 += a1 * w;
      s2 += a2 * w;
    }
    dst[o] = (uint8_t)((s0 + half) / whole);
    dst[o + 1U] = (uint8_t)((s1 + half) / whole);
    dst[o + 2U] = (uint8_t)((s2 + half) / whole);
  }
}
