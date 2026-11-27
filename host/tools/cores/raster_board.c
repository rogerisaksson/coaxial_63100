/** raster_board.c - C11 board 3D shadow map, shading, glow/halftone/rim, silhouette, and ground. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern void rs_triangles(int nverts, const double *pos, int ntris, const int32_t *idx,
                         const double *nrm, const double *m9, double distance, double scale,
                         double cx, double cy, double aspect, double lx, double ly, double lz,
                         double sun_min, int width, int first, int last, double *sx, double *sy,
                         double *ooz, double *depth, uint8_t *top, uint8_t *sun);
extern void rs_fold(const double *depth, const uint8_t *top, const uint8_t *sun, int width,
                    int height, double *out_depth, uint8_t *out_top, uint8_t *out_sun,
                    double *out_cov, uint8_t *out_reach);

/** Orthographic shadow depth map matching `shadow.cast` and `test_paint._cast` texel-for-texel. */
void rs_shadow_cast(int nverts, const double *restrict pos, int ntris, const int32_t *restrict idx,
                    const double *restrict m9, const double *restrict right3,
                    const double *restrict up3, const double *restrict light3, int size,
                    double extent, double *restrict la, double *restrict lb, double *restrict lc,
                    double *restrict sbuf)
{
  const size_t npix = (size_t)size * (size_t)size;
  for (size_t i = 0u; i < npix; i++)
  {
    sbuf[i] = -9.0;
  }
  if (nverts <= 0 || ntris <= 0 || size <= 0)
  {
    return;
  }
  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  const double r0 = right3[0], r1 = right3[1], r2 = right3[2];
  const double u0 = up3[0], u1 = up3[1], u2 = up3[2];
  const double l0 = light3[0], l1 = light3[1], l2 = light3[2];
  const double half = (double)(size - 1) * 0.5;
  const double s_lim = (double)size;

  for (int i = 0; i < nverts; i++)
  {
    const size_t o = (size_t)i * 3u;
    const double x = pos[o], y = pos[o + 1u], z = pos[o + 2u];
    const double tx = (m0 * x + m1 * y) + m2 * z;
    const double ty = (m3 * x + m4 * y) + m5 * z;
    const double tz = (m6 * x + m7 * y) + m8 * z;
    la[i] = (((tx * r0 + ty * r1) + tz * r2) / extent + 1.0) * half;
    lb[i] = (((tx * u0 + ty * u1) + tz * u2) / extent + 1.0) * half;
    lc[i] = (tx * l0 + ty * l1) + tz * l2;
  }
  for (int t = 0; t < ntris; t++)
  {
    const size_t o = (size_t)t * 3u;
    const int    a = (int)idx[o], b = (int)idx[o + 1u], c = (int)idx[o + 2u];
    double       x0 = la[a], y0 = lb[a], x1 = la[b], y1 = lb[b], x2 = la[c], y2 = lb[c];
    double       da = lc[a], db = lc[b], dc = lc[c];
    double       area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    if (area == 0.0)
    {
      continue;
    }
    if (area < 0.0)
    {
      double tx_ = x1, ty_ = y1, td_ = db;
      x1 = x2;
      y1 = y2;
      db = dc;
      x2 = tx_;
      y2 = ty_;
      dc = td_;
      area = -area;
    }
    const double min_x = (x0 < x1) ? ((x0 < x2) ? x0 : x2) : ((x1 < x2) ? x1 : x2);
    const double max_x = (x0 > x1) ? ((x0 > x2) ? x0 : x2) : ((x1 > x2) ? x1 : x2);
    const double min_y = (y0 < y1) ? ((y0 < y2) ? y0 : y2) : ((y1 < y2) ? y1 : y2);
    const double max_y = (y0 > y1) ? ((y0 > y2) ? y0 : y2) : ((y1 > y2) ? y1 : y2);
    const double cx_hi = (max_x < -2.0) ? -2.0 : ((max_x > s_lim) ? s_lim : max_x);
    const double cy_hi = (max_y < -2.0) ? -2.0 : ((max_y > s_lim) ? s_lim : max_y);
    const int    lo_x = (min_x <= 0.0) ? 0 : ((min_x >= s_lim) ? size : (int)min_x);
    const int    hi_x = ((int)cx_hi + 1 < size) ? ((int)cx_hi + 1) : (size - 1);
    const int    lo_y = (min_y <= 0.0) ? 0 : ((min_y >= s_lim) ? size : (int)min_y);
    const int    hi_y = ((int)cy_hi + 1 < size) ? ((int)cy_hi + 1) : (size - 1);
    const double inv = 1.0 / area;
    const double e0x = x2 - x1, e0y = y2 - y1, e1x = x0 - x2, e1y = y0 - y2;
    for (int py = lo_y; py <= hi_y; py++)
    {
      const size_t row = (size_t)py * (size_t)size;
      const double r_0 = e0x * ((double)py - y1), r_1 = e1x * ((double)py - y2);
      for (int px = lo_x; px <= hi_x; px++)
      {
        const double w0 = (r_0 - e0y * ((double)px - x1)) * inv;
        const double w1 = (r_1 - e1y * ((double)px - x2)) * inv;
        const double w2 = (1.0 - w0) - w1;
        if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0)
        {
          const double here = (w0 * da + w1 * db) + w2 * dc;
          if (here > sbuf[row + (size_t)px])
          {
            sbuf[row + (size_t)px] = here;
          }
        }
      }
    }
  }
}

/** Fused fine triangle raster + 2x4 cell fold without Python list boxing of fine buffers. */
void rs_raster_folded(int nverts, const double *pos, int ntris, const int32_t *idx,
                      const double *nrm, const double *m9, double distance, double f_scale,
                      double f_cx, double f_cy, double f_aspect, double lx, double ly, double lz,
                      double sun_min, int width, int height, double *sx, double *sy, double *ooz,
                      double *f_depth, uint8_t *f_top, uint8_t *f_sun, double *out_depth,
                      uint8_t *out_top, uint8_t *out_sun, double *out_cov, uint8_t *out_reach)
{
  rs_triangles(nverts, pos, ntris, idx, nrm, m9, distance, f_scale, f_cx, f_cy, f_aspect, lx, ly,
               lz, sun_min, width * 2, 0, height * 4, sx, sy, ooz, f_depth, f_top, f_sun);
  rs_fold(f_depth, f_top, f_sun, width, height, out_depth, out_top, out_sun, out_cov, out_reach);
}

/** Per-cell depth/shadow/art class and level shader matching `engine.shade` bit-for-bit. */
void rs_shade(int width, int height, const double *restrict depth, const uint8_t *restrict top,
              const uint8_t *restrict sun, const double *restrict m9, const double *restrict cam5,
              const double *restrict pars7, const int32_t *restrict art_dense, int art_w, int art_h,
              const double *restrict sbuf, int s_n, double s_ext, const double *restrict s_vecs9,
              uint8_t *restrict out_cls, double *restrict levels, double *restrict bare,
              double *restrict seed)
{
  const double distance = cam5[0], scale = cam5[1], cx = cam5[2], cy = cam5[3], reach = cam5[4];
  const double pivot = pars7[0], slope = pars7[1], floor_v = pars7[2], sh_step = pars7[3];
  const double bias = pars7[4], plane = pars7[5], tone_depth = pars7[6];
  const int    back = (m9[8] < 0.0);
  const int    use_art = (art_dense != NULL) && (art_w > 0) && (art_h > 0) && (!back);
  const double lean_art =
    (m9[8] >= 0.05) ? m9[8] : ((m9[8] <= -0.05) ? m9[8] : (!back ? 0.05 : -0.05));
  const double lean_abs = (m9[8] >= 0.0) ? m9[8] : -m9[8];
  const size_t ncells = (size_t)width * (size_t)height;
  memset(out_cls, 0, ncells);

  for (int py = 0; py < height; py++)
  {
    const double v = (cy - ((double)py + 0.5)) / (scale * 0.5);
    for (int px = 0; px < width; px++)
    {
      const size_t at = (size_t)py * (size_t)width + (size_t)px;
      const double w = depth[at];
      if (w == 0.0)
      {
        continue;
      }
      const int    lid = (top[at] != 0u);
      const double u = (((double)px + 0.5) - cx) / scale;
      const double tz = distance - 1.0 / w, tx = u / w, ty = v / w;
      double       shaded = 0.0;
      if (sbuf != NULL && s_n > 0 && lid && sun[at] != 0u)
      {
        const double si_f =
          ((((tx * s_vecs9[0] + ty * s_vecs9[1]) + tz * s_vecs9[2]) / s_ext) * 0.5 + 0.5) *
          (double)(s_n - 1);
        const double sj_f =
          ((((tx * s_vecs9[3] + ty * s_vecs9[4]) + tz * s_vecs9[5]) / s_ext) * 0.5 + 0.5) *
          (double)(s_n - 1);
        if (si_f > -1.0 && si_f < (double)s_n && sj_f > -1.0 && sj_f < (double)s_n)
        {
          const int si = (int)si_f, sj = (int)sj_f;
          if (si >= 0 && si < s_n && sj >= 0 && sj < s_n)
          {
            const double occ = sbuf[sj * s_n + si];
            if (occ > ((tx * s_vecs9[6] + ty * s_vecs9[7]) + tz * s_vecs9[8]) + bias)
            {
              shaded = sh_step;
            }
          }
        }
      }
      int    ink = -1, ix = px, iy = py;
      double rise = 0.0;
      if (use_art && lid)
      {
        const double d = distance - tz, atx = u * d, aty = v * d;
        const double hx = (m9[0] * atx + m9[3] * aty) + m9[6] * tz;
        const double hy = (m9[1] * atx + m9[4] * aty) + m9[7] * tz;
        const double hz = (m9[2] * atx + m9[5] * aty) + m9[8] * tz;
        if (!(hx * hx + hy * hy > 0.96 * 0.96))
        {
          rise = (hz - plane) / lean_art;
          const double hxf = back ? -hx : hx;
          int aiy = (int)((1.0 - (hy + 1.0) * 0.5) * (double)art_h);
          int aix = (int)(((hxf + 1.0) * 0.5) * (double)art_w);
          aiy = (aiy < 0) ? 0 : ((aiy >= art_h) ? (art_h - 1) : aiy);
          aix = (aix < 0) ? 0 : ((aix >= art_w) ? (art_w - 1) : aix);
          ink = (int)art_dense[aiy * art_w + aix];
          ix = aix;
          iy = aiy;
        }
      }
      const double lv0 = pivot + (slope * tz) / reach;
      if (bare != NULL)
      {
        bare[at] = lv0;
      }
      double level;
      if (ink >= 0)
      {
        double inked =
          (((pivot + (double)ink - 2.0) - 3.4 * (1.0 - lean_abs)) + (slope * rise) / reach) -
          shaded;
        if (ink > 0 && inked < floor_v)
        {
          inked = floor_v;
        }
        level = inked;
      }
      else
      {
        level = (lv0 - shaded > floor_v) ? (lv0 - shaded) : floor_v;
      }
      if (seed != NULL)
      {
        seed[at] =
          (double)((((int64_t)ix * 73856093LL) ^ ((int64_t)iy * 19349663LL)) & 255LL) / 255.0;
      }
      if (levels != NULL)
      {
        int cls_l = (int)(level + 0.5);
        cls_l = (cls_l < 0) ? 0 : ((cls_l > 2) ? 2 : cls_l);
        levels[at] = (double)cls_l + tone_depth * (level - (double)cls_l);
      }
      const double cl_lv = (level < 0.0) ? 0.0 : ((level > 2.0) ? 2.0 : level);
      out_cls[at] = (uint8_t)((int)(cl_lv + 0.5));
    }
  }
}

static inline int64_t rs_blend_glow(double heat, const int64_t *restrict rgb, int max_lo)
{
  const double clamped = (heat < 0.0) ? 0.0 : heat;
  int          lo = (int)clamped;
  if (lo < 0)
  {
    lo = 0;
  }
  else if (lo > max_lo)
  {
    lo = max_lo;
  }
  const double   f = clamped - (double)lo;
  const int64_t *a = rgb + (size_t)lo * 3u, *b = a + 3u;
  const int64_t  r = (int64_t)((double)a[0] + (double)(b[0] - a[0]) * f + 0.5);
  const int64_t  g = (int64_t)((double)a[1] + (double)(b[1] - a[1]) * f + 0.5);
  const int64_t  bl = (int64_t)((double)a[2] + (double)(b[2] - a[2]) * f + 0.5);
  return (r << 16) | (g << 8) | bl;
}

/** Glow pass matching `paint._glow` with C `libm` `tanh`, `pow`, and `exp` (0-ULP to Python). */
void rs_paint_glow(int width, int height, const uint8_t *restrict cls,
                   const double *restrict levels, const double *restrict bare,
                   const double *restrict seed, const double *restrict coverage,
                   const double *restrict buf, int colour, int has_key,
                   const double *restrict rig6, const double *restrict colf,
                   const double *restrict rowf, const int64_t *restrict rung_codes,
                   const int64_t *restrict glow_rgb, int n_glow, int64_t *restrict glyph,
                   int64_t *restrict tone, double *restrict heat_out)
{
  const int    steps = n_glow - 1;
  const double top_heat = (double)(steps - 1);
  for (int py = 0; py < height; py++)
  {
    const double ny = ((double)py + 0.5) / (double)height - (-0.08);
    for (int px = 0; px < width; px++)
    {
      const size_t  at = (size_t)py * (size_t)width + (size_t)px;
      const uint8_t c = cls[at];
      if (c == 0u)
      {
        continue;
      }
      const double lv = (levels != NULL) ? levels[at] : (double)c;
      const double hs = (seed != NULL) ? seed[at] : 0.0;
      const int    low = (lv < 1.0) ? 0 : 1;
      double       st = lv - (double)low;
      st = (st < 0.0) ? 0.0 : ((st > 1.0) ? 1.0 : st);
      const double rung = (double)(low * 2) + st * 2.0;
      int          ir = (int)(rung + 0.5);
      ir = (ir < 1) ? 1 : ((ir > 8) ? 8 : ir);
      glyph[at] = rung_codes[ir];
      if (!colour)
      {
        continue;
      }
      const double nx = ((double)px + 0.5) / (double)width - 1.05;
      const double pool = 1.0 - (nx * nx + ny * ny) / (3.45 * 3.45);
      double       t = (lv - (-0.47)) / 7.0 - 0.24;
      if (pool > 0.0)
      {
        t += 0.30 * pool * pool;
      }
      double rel = 0.0;
      if (py > 1 && px < width - 2 && cls[at - (size_t)width + 1u] != 0u &&
          cls[at - 2u * (size_t)width + 2u] != 0u)
      {
        rel = tanh(9.0 * ((bare[at] + bare[at - 2u * (size_t)width + 2u]) -
                          2.0 * bare[at - (size_t)width + 1u]));
      }
      t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
      const double hot0 = pow(t, 0.87);
      const double hot = hot0 / (hot0 + pow(1.0 - t, 0.87));
      const double grain = (c == 1u) ? 1.45 : 0.0;
      double       heat = (hot * (double)steps + (0.11 * (double)steps) * rel) + grain * (hs - 0.5);
      if (has_key)
      {
        double klit = 0.77;
        if (px > 0 && px < width - 1 && py > 0 && py < height - 1 && cls[at - 1u] != 0u &&
            cls[at + 1u] != 0u && cls[at - (size_t)width] != 0u && cls[at + (size_t)width] != 0u)
        {
          const double gx = (bare[at + 1u] - bare[at - 1u]) * rig6[0];
          const double gy = (bare[at - (size_t)width] - bare[at + (size_t)width]) * rig6[1];
          double       dx = 0.60, dy = 0.20, dz = 0.77;
          if (buf[at] != 0.0)
          {
            const double inv = 1.0 / buf[at];
            dx = rig6[2] - colf[px] * inv;
            dy = rig6[3] - rowf[py] * inv;
            dz = (rig6[4] - rig6[5]) + inv;
            double dn = sqrt((dx * dx + dy * dy) + dz * dz);
            if (dn == 0.0)
            {
              dn = 1.0;
            }
            dx /= dn;
            dy /= dn;
            dz /= dn;
          }
          klit = ((dz - gx * dx) - gy * dy) / sqrt((gx * gx + gy * gy) + 1.0);
          if (klit < 0.0)
          {
            klit = 0.0;
          }
        }
        heat += 3.5 * (klit - 0.77);
      }
      heat -= 1.5 * (1.0 - coverage[at]);
      if (heat > top_heat)
      {
        heat = top_heat;
      }
      else if (heat < 1.4)
      {
        heat = 0.4 + 1.0 * exp(((heat - 0.4) - 1.0) / 1.0);
      }
      heat_out[at] = heat;
      tone[at] = rs_blend_glow(heat, glow_rgb, n_glow - 2);
    }
  }
}

/** Halftone scanline dots and silhouette rim matching `paint._dots` and `paint._rim`. */
void rs_paint_dots_rim(int width, int height, const uint8_t *restrict cls,
                       const uint8_t *restrict reached, const double *restrict heat, int colour,
                       double lo, double hi, const int64_t *restrict noise64,
                       const int64_t *restrict rim_glyph, const int64_t *restrict glow_rgb,
                       int n_glow, int64_t *restrict glyph, int64_t *restrict tone)
{
  static const double  OX[4] = {-0.25, -0.25, 0.25, 0.25};
  static const double  OY[4] = {-0.375, 0.125, -0.375, 0.125};
  static const int64_t BIT[4] = {0x01, 0x04, 0x08, 0x20};
  static const int     LANE[4] = {0, 0, 1, 1};
  static const int     ROW[4] = {0, 2, 0, 2};
  const double         gain = (hi > lo) ? (1.0 / (hi - lo)) : 0.0;
  const double         top_heat = (double)(n_glow - 2);

  for (int py = 0; py < height; py++)
  {
    for (int px = 0; px < width; px++)
    {
      const size_t at = (size_t)py * (size_t)width + (size_t)px;
      if (cls[at] == 0u)
      {
        continue;
      }
      const int64_t reach = (reached[at] != 0u) ? (int64_t)reached[at] : 0xFFLL;
      if (colour)
      {
        const double h0 = heat[at];
        const int    lok = (px > 0) && (cls[at - 1u] != 0u);
        const int    rok = (px < width - 1) && (cls[at + 1u] != 0u);
        const int    uok = (py > 0) && (cls[at - (size_t)width] != 0u);
        const int    dok = (py < height - 1) && (cls[at + (size_t)width] != 0u);
        const double gx =
          (lok && rok) ? (0.5 * (heat[at + 1u] - heat[at - 1u]))
                       : (rok ? (heat[at + 1u] - h0) : (lok ? (h0 - heat[at - 1u]) : 0.0));
        const double gy = (uok && dok)
                            ? (0.5 * (heat[at + (size_t)width] - heat[at - (size_t)width]))
                            : (dok ? (heat[at + (size_t)width] - h0)
                                   : (uok ? (h0 - heat[at - (size_t)width]) : 0.0));
        const double  base = (h0 - lo) * gain;
        int64_t       mask = 0;
        for (int k = 0; k < 4; k++)
        {
          if ((reach & BIT[k]) == 0)
          {
            continue;
          }
          double sh = base + (gx * OX[k] + gy * OY[k]) * gain;
          sh = (sh < 0.0) ? 0.0 : ((sh > 1.0) ? 1.0 : sh);
          sh = (0.42 + 0.08 * sh) * 2.0;
          if (sh > 1.0)
          {
            sh = 1.0;
          }
          const size_t ny = (size_t)((py * 4 + ROW[k]) & 63);
          const size_t nx = (size_t)((px * 2 + LANE[k]) & 63);
          if (sh * 4096.0 > (double)noise64[ny * 64u + nx] + 0.5)
          {
            mask |= BIT[k];
          }
        }
        glyph[at] = 0x2800LL + ((mask != 0) ? mask : 0x01LL);
      }
      if (reach != 0xFFLL)
      {
        int64_t was = glyph[at] - 0x2800LL;
        if (was < 0 || was > 0xFFLL)
        {
          was = 0;
        }
        glyph[at] = 0x2800LL + ((was & reach) | rim_glyph[reach]);
        if (colour)
        {
          const double base = (heat != NULL && heat[at] != 0.0) ? heat[at] : 3.0;
          double       eh = base + 3.0 * (0.5 + 0.5 * base / top_heat);
          if (eh > (double)(n_glow - 1))
          {
            eh = (double)(n_glow - 1);
          }
          tone[at] = rs_blend_glow(eh, glow_rgb, n_glow - 2);
        }
      }
    }
  }
}

/** O(N) flood-fill + 8-way silhouette edge matching `strokes.edge` and `lines._edge`. */
void rs_edge(int width, int height, const uint8_t *restrict reached_in, const double *restrict heat,
             int colour, const int64_t *restrict edge_tbl8, const int64_t *restrict glow_rgb,
             int n_glow, int32_t *restrict region, int32_t *restrict queue,
             int64_t *restrict glyph, int64_t *restrict tone)
{
  const int n = width * height;
  if (n <= 0)
  {
    return;
  }
  for (int i = 0; i < n; i++)
  {
    const uint8_t r = (heat != NULL && heat[i] == 0.0) ? 0u : reached_in[i];
    region[i] = (r != 0u) ? 0 : 1;
  }
  for (int pass = 0; pass < 2; pass++)
  {
    for (int i = 0; i < n; i++)
    {
      if (region[i] != 1)
      {
        continue;
      }
      const int r0 = i / width, c0 = i - r0 * width;
      if (pass == 0 && r0 != 0 && r0 != height - 1 && c0 != 0 && c0 != width - 1)
      {
        continue;
      }
      int head = 0, tail = 0;
      region[i] = 2;
      queue[tail++] = i;
      while (head < tail)
      {
        const int cur = queue[head++];
        const int cr = cur / width, cc = cur - cr * width;
        const int nb[4] = {(cc > 0) ? (cur - 1) : -1, (cc < width - 1) ? (cur + 1) : -1,
                           (cr > 0) ? (cur - width) : -1, (cr < height - 1) ? (cur + width) : -1};
        for (int d = 0; d < 4; d++)
        {
          if (nb[d] >= 0 && region[nb[d]] == 1)
          {
            region[nb[d]] = 2;
            queue[tail++] = nb[d];
          }
        }
      }
      if (pass == 1 && tail < 3)
      {
        for (int k = 0; k < tail; k++)
        {
          region[queue[k]] = 3;
        }
      }
    }
  }
  const double top_heat = (double)(n_glow - 2);
  for (int i = 0; i < n; i++)
  {
    const uint8_t bits_u = (heat != NULL && heat[i] == 0.0) ? 0u : reached_in[i];
    if (bits_u == 0u)
    {
      continue;
    }
    const int r = i / width, c = i - r * width;
    int       near_ext = 0;
    for (int rr = r - 1; rr <= r + 1 && !near_ext; rr++)
    {
      if (rr >= 0 && rr < height)
      {
        for (int cc = c - 1; cc <= c + 1; cc++)
        {
          if (cc >= 0 && cc < width && region[rr * width + cc] == 2)
          {
            near_ext = 1;
            break;
          }
        }
      }
    }
    if (!near_ext)
    {
      continue;
    }
    const int nb_ok[4] = {c > 0, c < width - 1, r > 0, r < height - 1};
    const int nb_j[4] = {i - 1, i + 1, i - width, i + width};
    int       beside[4] = {255, 255, 255, 255};
    for (int k = 0; k < 4; k++)
    {
      if (nb_ok[k] && region[nb_j[k]] != 3)
      {
        const int j = nb_j[k];
        beside[k] = (int)((heat != NULL && heat[j] == 0.0) ? 0u : reached_in[j]);
      }
    }
    const int     b = (int)bits_u;
    const int64_t mask = (int64_t)b & (edge_tbl8[b] | edge_tbl8[256 + b] | edge_tbl8[512 + b] |
                                       edge_tbl8[768 + b] | edge_tbl8[1024 + beside[0]] |
                                       edge_tbl8[1280 + beside[1]] | edge_tbl8[1536 + beside[2]] |
                                       edge_tbl8[1792 + beside[3]]);
    if (mask != 0)
    {
      const int64_t was = glyph[i];
      const int64_t m_or = (was >= 0x2800LL && was < 0x2900LL) ? (mask | (was - 0x2800LL)) : mask;
      glyph[i] = 0x2800LL + m_or;
      if (colour)
      {
        const double base = (heat != NULL && heat[i] != 0.0) ? heat[i] : 3.0;
        double       eh = base + 3.0 * (0.5 + 0.5 * base / top_heat);
        if (eh > (double)(n_glow - 1))
        {
          eh = (double)(n_glow - 1);
        }
        tone[i] = rs_blend_glow(eh, glow_rgb, n_glow - 2);
      }
    }
  }
}
