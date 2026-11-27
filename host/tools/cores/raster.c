/** raster.c - C11 3D mesh transform, projection, 2x2 dot-splat, and triangle rasterizer kernels. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/** Transform `(n, 3)` vertices and normals by per-part `(turn, spot)` spans in one pass. */
void rs_moved(int n, const double *restrict pts, const double *restrict nrm, int nspans,
              const int32_t *restrict spans, const double *restrict turns,
              const double *restrict spots, double *restrict out_p, double *restrict out_n)
{
  if (n <= 0 || nspans <= 0)
  {
    return;
  }
  for (int s = 0; s < nspans; s++)
  {
    const int lo = (int)spans[2 * s];
    const int hi = (int)spans[2 * s + 1];
    if (hi <= lo || lo < 0 || hi > n)
    {
      continue;
    }
    const double *t = turns + 9 * s;
    const double *sp = spots + 3 * s;
    const double  t0 = t[0], t1 = t[1], t2 = t[2];
    const double  t3 = t[3], t4 = t[4], t5 = t[5];
    const double  t6 = t[6], t7 = t[7], t8 = t[8];
    const double  sx = sp[0], sy = sp[1], sz = sp[2];

    for (int i = lo; i < hi; i++)
    {
      const size_t o = (size_t)i * 3u;
      const double px = pts[o], py = pts[o + 1u], pz = pts[o + 2u];
      const double nx = nrm[o], ny = nrm[o + 1u], nz = nrm[o + 2u];

      out_p[o] = ((px * t0 + py * t1) + pz * t2) + sx;
      out_p[o + 1u] = ((px * t3 + py * t4) + pz * t5) + sy;
      out_p[o + 2u] = ((px * t6 + py * t7) + pz * t8) + sz;
      out_n[o] = (nx * t0 + ny * t1) + nz * t2;
      out_n[o + 1u] = (nx * t3 + ny * t4) + nz * t5;
      out_n[o + 2u] = (nx * t6 + ny * t7) + nz * t8;
    }
  }
}

/** Project `(n, 3)` world points into screen dot coordinates `(sx, sy, w)`. */
void rs_project(int n, const double *restrict pts, const double *restrict m9,
                const double *restrict centre, double distance, double scale, double cx, double cy,
                double aspect, double *restrict sx, double *restrict sy, double *restrict w)
{
  if (n <= 0)
  {
    return;
  }
  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  const double c0 = centre[0], c1 = centre[1], c2 = centre[2];
  const double cr0 = (c0 * m0 + c1 * m1) + c2 * m2;
  const double cr1 = (c0 * m3 + c1 * m4) + c2 * m5;
  const double cr2 = (c0 * m6 + c1 * m7) + c2 * m8;
  const int    large = (n > 64);

  for (int i = 0; i < n; i++)
  {
    const size_t o = (size_t)i * 3u;
    double       qx, qy, qz;
    if (large)
    {
      const double px = pts[o], py = pts[o + 1u], pz = pts[o + 2u];
      qx = ((px * m0 + py * m1) + pz * m2) - cr0;
      qy = ((px * m3 + py * m4) + pz * m5) - cr1;
      qz = ((px * m6 + py * m7) + pz * m8) - cr2;
    }
    else
    {
      const double dx = pts[o] - c0, dy = pts[o + 1u] - c1, dz0 = pts[o + 2u] - c2;
      qx = (dx * m0 + dy * m1) + dz0 * m2;
      qy = (dx * m3 + dy * m4) + dz0 * m5;
      qz = (dx * m6 + dy * m7) + dz0 * m8;
    }
    const double dz = distance - qz;
    const double wk = (dz != 0.0) ? (1.0 / dz) : 0.0;
    const double sw = scale * wk;
    sx[i] = cx + sw * qx;
    sy[i] = cy - sw * aspect * qy;
    w[i] = wk;
  }
}

/** Single-pass O(N) z-buffered 2x2 dot splat matching `lit.splat` without O(N log N) sorting. */
void rs_splat(int n, const double *restrict pos, const uint32_t *restrict materials,
              const double *restrict m9, const double *restrict centre, double distance,
              double scale, double cx, double cy, double aspect, int width, int height,
              uint32_t under_bit, double layer_m, double *restrict best_sc,
              double *restrict best_w, int32_t *restrict best_idx, int32_t *restrict best_ix,
              int32_t *restrict best_iy, double *restrict depth, int32_t *restrict win)
{
  if (width <= 0 || height <= 0)
  {
    return;
  }
  const int    stride = width + 1;
  const int    ncells = (height + 1) * stride;
  const size_t npix = (size_t)width * (size_t)height;

  for (int c = 0; c < ncells; c++)
  {
    best_idx[c] = -1;
  }
  memset(depth, 0, npix * sizeof(double));
  for (size_t p = 0; p < npix; p++)
  {
    win[p] = -1;
  }
  if (n <= 0)
  {
    return;
  }

  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  const double c0 = centre[0], c1 = centre[1], c2 = centre[2];
  const double cr0 = (c0 * m0 + c1 * m1) + c2 * m2;
  const double cr1 = (c0 * m3 + c1 * m4) + c2 * m5;
  const double cr2 = (c0 * m6 + c1 * m7) + c2 * m8;
  const double w_max = (double)width;
  const double h_max = (double)height;
  const int    large = (n > 64);

  for (int k = 0; k < n; k++)
  {
    const size_t o = (size_t)k * 3u;
    double       qx, qy, qz;
    if (large)
    {
      const double px = pos[o], py = pos[o + 1u], pz = pos[o + 2u];
      qx = ((px * m0 + py * m1) + pz * m2) - cr0;
      qy = ((px * m3 + py * m4) + pz * m5) - cr1;
      qz = ((px * m6 + py * m7) + pz * m8) - cr2;
    }
    else
    {
      const double dx = pos[o] - c0, dy = pos[o + 1u] - c1, dz0 = pos[o + 2u] - c2;
      qx = (dx * m0 + dy * m1) + dz0 * m2;
      qy = (dx * m3 + dy * m4) + dz0 * m5;
      qz = (dx * m6 + dy * m7) + dz0 * m8;
    }
    const double dz = distance - qz;
    const double wk = (dz != 0.0) ? (1.0 / dz) : 0.0;
    if (!(wk > 0.0))
    {
      continue;
    }
    const double  sw = scale * wk;
    const double  sx_raw = cx + sw * qx;
    const double  sy_raw = cy - sw * aspect * qy;
    const double  sx_clp = (sx_raw < -2.0) ? -2.0 : ((sx_raw > w_max) ? w_max : sx_raw);
    const double  sy_clp = (sy_raw < -2.0) ? -2.0 : ((sy_raw > h_max) ? h_max : sy_raw);
    const int32_t ix = (int32_t)rint(sx_clp);
    const int32_t iy = (int32_t)rint(sy_clp);
    if (ix < -1 || ix >= width || iy < -1 || iy >= height)
    {
      continue;
    }
    const int    cell = ((int)iy + 1) * stride + ((int)ix + 1);
    const double under = ((materials[k] & under_bit) != 0u) ? 1.0 : 0.0;
    const double sc = wk / (1.0 + layer_m * wk * under);
    if (best_idx[cell] < 0 || sc >= best_sc[cell])
    {
      best_sc[cell] = sc;
      best_w[cell] = wk;
      best_idx[cell] = (int32_t)k;
      best_ix[cell] = ix;
      best_iy[cell] = iy;
    }
  }

  int ntop = 0;
  for (int c = 0; c < ncells; c++)
  {
    if (best_idx[c] >= 0)
    {
      best_idx[ntop] = best_idx[c];
      best_w[ntop] = best_w[c];
      best_ix[ntop] = best_ix[c];
      best_iy[ntop] = best_iy[c];
      ntop++;
    }
  }

  for (int pass = 0; pass < 4; pass++)
  {
    const int dx = pass & 1;
    const int dy = pass >> 1;
    for (int t = 0; t < ntop; t++)
    {
      const int x = (int)best_ix[t] + dx;
      const int y = (int)best_iy[t] + dy;
      if (x >= 0 && x < width && y >= 0 && y < height)
      {
        const size_t pix = (size_t)y * (size_t)width + (size_t)x;
        depth[pix] = best_w[t];
        win[pix] = best_idx[t];
      }
    }
  }
}

static void rs_project_solid(int nverts, const double *restrict pos, const double *restrict m9,
                             double distance, double scale, double cx, double cy, double aspect,
                             double *restrict sx, double *restrict sy, double *restrict ooz)
{
  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  for (int i = 0; i < nverts; i++)
  {
    const size_t o = (size_t)i * 3u;
    const double x = pos[o], y = pos[o + 1u], z = pos[o + 2u];
    const double tx = (m0 * x + m1 * y) + m2 * z;
    const double ty = (m3 * x + m4 * y) + m5 * z;
    const double tz = (m6 * x + m7 * y) + m8 * z;
    const double w = 1.0 / (distance - tz);
    sx[i] = cx + (scale * w) * tx;
    sy[i] = cy - ((scale * aspect) * w) * ty;
    ooz[i] = w;
  }
}

static void rs_scan_triangle(double x0, double y0, double x1, double y1, double x2, double y2,
                             double oa, double ob, double og, double area, uint8_t flat,
                             uint8_t lit, int width, int first, int last, double *restrict depth,
                             uint8_t *restrict top, uint8_t *restrict sun)
{
  const double w_lim = (double)width;
  const double y_lo_lim = (double)first - 2.0;
  const double y_hi_lim = (double)last;
  const double min_x = (x0 < x1) ? ((x0 < x2) ? x0 : x2) : ((x1 < x2) ? x1 : x2);
  const double max_x = (x0 > x1) ? ((x0 > x2) ? x0 : x2) : ((x1 > x2) ? x1 : x2);
  const double min_y = (y0 < y1) ? ((y0 < y2) ? y0 : y2) : ((y1 < y2) ? y1 : y2);
  const double max_y = (y0 > y1) ? ((y0 > y2) ? y0 : y2) : ((y1 > y2) ? y1 : y2);
  const double cx_max = (max_x < -2.0) ? -2.0 : ((max_x > w_lim) ? w_lim : max_x);
  const double cy_max = (max_y < y_lo_lim) ? y_lo_lim : ((max_y > y_hi_lim) ? y_hi_lim : max_y);
  const int    lo_x = (min_x <= 0.0) ? 0 : ((min_x >= w_lim) ? width : (int)min_x);
  const int    rx_hi = (int)cx_max + 1;
  const int    hi_x = (rx_hi < width) ? rx_hi : (width - 1);
  const int    lo_y = (min_y <= (double)first) ? first : ((min_y >= y_hi_lim) ? last : (int)min_y);
  const int    ry_hi = (int)cy_max + 1;
  const int    hi_y = (ry_hi < last) ? ry_hi : (last - 1);
  if (hi_x < lo_x || hi_y < lo_y)
  {
    return;
  }
  const double inv = 1.0 / area;
  const double e0x = x2 - x1, e0y = y2 - y1;
  const double e1x = x0 - x2, e1y = y0 - y2;
  for (int py = lo_y; py <= hi_y; py++)
  {
    const size_t row = (size_t)(py - first) * (size_t)width;
    const double r0 = e0x * ((double)py - y1);
    const double r1 = e1x * ((double)py - y2);
    int          inside = 0;
    for (int px = lo_x; px <= hi_x; px++)
    {
      const double w0 = (r0 - e0y * ((double)px - x1)) * inv;
      const double w1 = (r1 - e1y * ((double)px - x2)) * inv;
      const double w2 = (1.0 - w0) - w1;
      if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0)
      {
        if (inside)
        {
          break;
        }
        continue;
      }
      inside = 1;
      const double here = (w0 * oa + w1 * ob) + w2 * og;
      const size_t at = row + (size_t)px;
      if (here > depth[at])
      {
        depth[at] = here;
        top[at] = flat;
        sun[at] = lit;
      }
    }
  }
}

/** Rasterize indexed triangles onto `[first, last)` rows matching `engine.raster` bit-for-bit. */
void rs_triangles(int nverts, const double *restrict pos, int ntris, const int32_t *restrict idx,
                  const double *restrict nrm, const double *restrict m9, double distance,
                  double scale, double cx, double cy, double aspect, double lx, double ly,
                  double lz, double sun_min, int width, int first, int last, double *restrict sx,
                  double *restrict sy, double *restrict ooz, double *restrict depth,
                  uint8_t *restrict top, uint8_t *restrict sun)
{
  const int rows = last - first;
  if (width <= 0 || rows <= 0)
  {
    return;
  }
  const size_t npix = (size_t)width * (size_t)rows;
  memset(depth, 0, npix * sizeof(double));
  memset(top, 0, npix);
  memset(sun, 0, npix);
  if (nverts <= 0 || ntris <= 0)
  {
    return;
  }
  rs_project_solid(nverts, pos, m9, distance, scale, cx, cy, aspect, sx, sy, ooz);

  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  for (int t = 0; t < ntris; t++)
  {
    const size_t to = (size_t)t * 3u;
    const int    a = (int)idx[to], b = (int)idx[to + 1u], c = (int)idx[to + 2u];
    const double bx = nrm[to], by = nrm[to + 1u], bz = nrm[to + 2u];
    double       nx = (m0 * bx + m1 * by) + m2 * bz;
    double       ny = (m3 * bx + m4 * by) + m5 * bz;
    double       nz = (m6 * bx + m7 * by) + m8 * bz;
    if (nz < 0.0)
    {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
    const uint8_t lit = ((bz > 0.0) && (((nx * lx + ny * ly) + nz * lz) > sun_min)) ? 1u : 0u;
    const uint8_t flat = ((bz > 0.9) || (bz < -0.9)) ? 1u : 0u;
    double        x0 = sx[a], y0 = sy[a], x1 = sx[b], y1 = sy[b], x2 = sx[c], y2 = sy[c];
    double        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    if (area == 0.0)
    {
      continue;
    }
    double oa = ooz[a], ob = ooz[b], og = ooz[c];
    if (area < 0.0)
    {
      rs_scan_triangle(x0, y0, x2, y2, x1, y1, oa, og, ob, -area, flat, lit, width, first, last,
                       depth, top, sun);
    }
    else
    {
      rs_scan_triangle(x0, y0, x1, y1, x2, y2, oa, ob, og, area, flat, lit, width, first, last,
                       depth, top, sun);
    }
  }
}
