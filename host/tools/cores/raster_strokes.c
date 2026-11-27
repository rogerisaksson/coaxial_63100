/** raster_strokes.c - C11 2D/3D clipped line strokes, ground grid, and multi-part mesh pose. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
  double  x0, y0, w0, x1, y1, w1, lo, hi;
  int32_t count, which;
} rs_stroke_t;

static inline int rs_clip_2d(double ax, double ay, double bx, double by, double w_lim, double h_lim,
                             double *x0, double *y0, double *x1, double *y1)
{
  const double dx = bx - ax, dy = by - ay;
  const double ps[4] = {-dx, dx, -dy, dy};
  const double qs[4] = {ax, w_lim - ax, ay, h_lim - ay};
  double       t0 = 0.0, t1 = 1.0;

  for (int e = 0; e < 4; e++)
  {
    const double p = ps[e], q = qs[e];
    if (p == 0.0)
    {
      if (q < 0.0)
      {
        return 0;
      }
    }
    else
    {
      const double r = q / p;
      if (p < 0.0)
      {
        if (r > t1)
        {
          return 0;
        }
        if (r > t0)
        {
          t0 = r;
        }
      }
      else
      {
        if (r < t0)
        {
          return 0;
        }
        if (r < t1)
        {
          t1 = r;
        }
      }
    }
  }
  *x0 = ax + t0 * dx;
  *y0 = ay + t0 * dy;
  *x1 = ax + t1 * dx;
  *y1 = ay + t1 * dy;
  return 1;
}

/** Rasterize `n` 2D segments into `(height, width)` `dots` matching `callouts.lines`. */
void rs_lines(uint8_t *restrict dots, int height, int width, int n, const double *restrict ax,
              const double *restrict ay, const double *restrict bx, const double *restrict by)
{
  if (height <= 0 || width <= 0 || n <= 0)
  {
    return;
  }
  const double w_lim = (double)(width - 1), h_lim = (double)(height - 1);
  for (int i = 0; i < n; i++)
  {
    double x0, y0, x1, y1;
    if (!rs_clip_2d(ax[i], ay[i], bx[i], by[i], w_lim, h_lim, &x0, &y0, &x1, &y1))
    {
      continue;
    }
    const double adx = fabs(x1 - x0), ady = fabs(y1 - y0);
    const int    cnt = (int)((adx >= ady) ? adx : ady) + 1;
    const double div = (cnt - 1 > 1) ? (double)(cnt - 1) : 1.0;
    const double sx = (x1 - x0) / div, sy = (y1 - y0) / div;
    for (int k = 0; k < cnt; k++)
    {
      const int    end = (k == cnt - 1) && (cnt > 1);
      const double px = end ? x1 : ((double)k * sx + x0);
      const double py = end ? y1 : ((double)k * sy + y0);
      int          ix = (int)rint(px), iy = (int)rint(py);
      ix = (ix < 0) ? 0 : ((ix >= width) ? (width - 1) : ix);
      iy = (iy < 0) ? 0 : ((iy >= height) ? (height - 1) : iy);
      dots[(size_t)iy * (size_t)width + (size_t)ix] = 1u;
    }
  }
}

static inline int rs_clip_stroke(const double *restrict ends6, int32_t idx,
                                 const double *restrict m9, const double *restrict c3,
                                 double distance, double near_m, double scale, double cx, double cy,
                                 double aspect, double w_top, double h_top, rs_stroke_t *out)
{
  const double d0x = ends6[0] - c3[0], d0y = ends6[1] - c3[1], d0z = ends6[2] - c3[2];
  const double d1x = ends6[3] - c3[0], d1y = ends6[4] - c3[1], d1z = ends6[5] - c3[2];
  double       ax = (d0x * m9[0] + d0y * m9[1]) + d0z * m9[2];
  double       ay = (d0x * m9[3] + d0y * m9[4]) + d0z * m9[5];
  double       az = (d0x * m9[6] + d0y * m9[7]) + d0z * m9[8];
  double       bx = (d1x * m9[0] + d1y * m9[1]) + d1z * m9[2];
  double       by = (d1x * m9[3] + d1y * m9[4]) + d1z * m9[5];
  double       bz = (d1x * m9[6] + d1y * m9[7]) + d1z * m9[8];
  const double limit = distance - near_m;

  if (!(az < limit || bz < limit))
  {
    return 0;
  }
  if (az > limit)
  {
    const double t = (limit - az) / (bz - az);
    ax += (bx - ax) * t;
    ay += (by - ay) * t;
    az += (bz - az) * t;
  }
  if (bz > limit)
  {
    const double t = (limit - bz) / (az - bz);
    bx += (ax - bx) * t;
    by += (ay - by) * t;
    bz += (az - bz) * t;
  }
  const double w0 = 1.0 / (distance - az), w1 = 1.0 / (distance - bz);
  const double x0 = cx + (scale * w0) * ax, y0 = cy - ((scale * aspect) * w0) * ay;
  const double x1 = cx + (scale * w1) * bx, y1 = cy - ((scale * aspect) * w1) * by;
  const double ps[2] = {x0, y0}, ds[2] = {x1 - x0, y1 - y0}, tops[2] = {w_top, h_top};
  double       lo = 0.0, hi = 1.0;

  for (int a = 0; a < 2; a++)
  {
    const double p = ps[a], d = ds[a], top = tops[a];
    if (fabs(d) < 1e-12)
    {
      const int inside = (p >= 0.0) && (p <= top);
      const double cl = inside ? 0.0 : 1.0, ch = inside ? 1.0 : 0.0;
      lo = (cl > lo) ? cl : lo;
      hi = (ch < hi) ? ch : hi;
    }
    else
    {
      const double t0 = (0.0 - p) / d, t1 = (top - p) / d;
      const double tmin = (t0 < t1) ? t0 : t1, tmax = (t0 > t1) ? t0 : t1;
      lo = (tmin > lo) ? tmin : lo;
      hi = (tmax < hi) ? tmax : hi;
    }
  }
  if (!(lo < hi))
  {
    return 0;
  }
  const double adx = fabs(x1 - x0), ady = fabs(y1 - y0);
  out->x0 = x0;
  out->y0 = y0;
  out->w0 = w0;
  out->x1 = x1;
  out->y1 = y1;
  out->w1 = w1;
  out->lo = lo;
  out->hi = hi;
  out->count = (int32_t)(((adx >= ady) ? adx : ady) * (hi - lo)) + 2;
  out->which = idx;
  return 1;
}

/** Clip `nseg` 3D segments into `scratch` (`sizeof(rs_stroke_t) * nseg`), returning `[kept, dots]`. */
void rs_strokes_prep(int nseg, const double *restrict ends, const double *restrict m9,
                     const double *restrict c3, double distance, double near_m, double scale,
                     double cx, double cy, double aspect, int width, int height,
                     void *restrict scratch, int64_t *restrict out_counts)
{
  rs_stroke_t *st = (rs_stroke_t *)scratch;
  int64_t      kept = 0, total = 0;
  const double w_top = (double)width - 1.0, h_top = (double)height - 1.0;

  for (int i = 0; i < nseg; i++)
  {
    if (rs_clip_stroke(ends + (size_t)i * 6u, (int32_t)i, m9, c3, distance, near_m, scale, cx, cy,
                       aspect, w_top, h_top, &st[kept]))
    {
      total += (int64_t)st[kept].count;
      kept++;
    }
  }
  out_counts[0] = kept;
  out_counts[1] = total;
}

/** Emit `kept` prepared strokes from `scratch` into `(ys, xs, w, which)`. */
void rs_strokes_emit(int64_t kept, const void *restrict scratch, int width, int height,
                     int64_t *restrict ys, int64_t *restrict xs, double *restrict w_out,
                     int64_t *restrict which_out)
{
  const rs_stroke_t *st = (const rs_stroke_t *)scratch;
  size_t             at = 0u;

  for (int64_t s = 0; s < kept; s++)
  {
    const rs_stroke_t g = st[s];
    const double      dx = g.x1 - g.x0, dy = g.y1 - g.y0, dw = g.w1 - g.w0;
    const double      span = g.hi - g.lo, denom = (double)(g.count - 1);
    for (int32_t step = 0; step < g.count; step++)
    {
      const double t = g.lo + (span * (double)step) / denom;
      int          ix = (int)rint(g.x0 + dx * t), iy = (int)rint(g.y0 + dy * t);
      ix = (ix < 0) ? 0 : ((ix >= width) ? (width - 1) : ix);
      iy = (iy < 0) ? 0 : ((iy >= height) ? (height - 1) : iy);
      xs[at] = (int64_t)ix;
      ys[at] = (int64_t)iy;
      w_out[at] = g.w0 + dw * t;
      which_out[at] = (int64_t)g.which;
      at++;
    }
  }
}

/** Fused `scenery.ground` brightness rasterizer without intermediate dot arrays. */
void rs_ground(int nseg, const double *restrict ends, const double *restrict m9,
               const double *restrict c3, double distance, double near_m, double scale, double cx,
               double cy, double aspect, int width, int height, double *restrict out)
{
  if (width <= 0 || height <= 0)
  {
    return;
  }
  memset(out, 0, (size_t)width * (size_t)height * sizeof(double));
  const double w_top = (double)width - 1.0, h_top = (double)height - 1.0;

  for (int i = 0; i < nseg; i++)
  {
    rs_stroke_t g;
    if (!rs_clip_stroke(ends + (size_t)i * 6u, (int32_t)i, m9, c3, distance, near_m, scale, cx, cy,
                        aspect, w_top, h_top, &g))
    {
      continue;
    }
    const double dx = g.x1 - g.x0, dy = g.y1 - g.y0, dw = g.w1 - g.w0;
    const double span = g.hi - g.lo, denom = (double)(g.count - 1);
    for (int32_t step = 0; step < g.count; step++)
    {
      const double t = g.lo + (span * (double)step) / denom;
      int          ix = (int)rint(g.x0 + dx * t), iy = (int)rint(g.y0 + dy * t);
      ix = (ix < 0) ? 0 : ((ix >= width) ? (width - 1) : ix);
      iy = (iy < 0) ? 0 : ((iy >= height) ? (height - 1) : iy);
      const double wd = (g.w0 + dw * t) * distance;
      const double cl = (wd < 0.0) ? 0.0 : ((wd > 1.0) ? 1.0 : wd);
      const double val = 0.2 + 0.8 * cl;
      const size_t pix = (size_t)iy * (size_t)width + (size_t)ix;
      if (val > out[pix])
      {
        out[pix] = val;
      }
    }
  }
}

/** Transform `nparts` separate `(points, normals)` arrays into contiguous `(out_p, out_n)`. */
void rs_pose_parts(int nparts, const uintptr_t *restrict pts_ptrs,
                   const uintptr_t *restrict nrm_ptrs, const int32_t *restrict counts,
                   const double *restrict turns9, const double *restrict spots3,
                   double *restrict out_p, double *restrict out_n)
{
  size_t base = 0u;
  for (int p = 0; p < nparts; p++)
  {
    const int n = (int)counts[p];
    if (n <= 0)
    {
      continue;
    }
    const double *restrict pts = (const double *)pts_ptrs[p];
    const double *restrict nrm = (const double *)nrm_ptrs[p];
    const double *t = turns9 + (size_t)p * 9u;
    const double *sp = spots3 + (size_t)p * 3u;
    const double  t0 = t[0], t1 = t[1], t2 = t[2];
    const double  t3 = t[3], t4 = t[4], t5 = t[5];
    const double  t6 = t[6], t7 = t[7], t8 = t[8];
    const double  sx = sp[0], sy = sp[1], sz = sp[2];

    for (int i = 0; i < n; i++)
    {
      const size_t src = (size_t)i * 3u, dst = (base + (size_t)i) * 3u;
      const double px = pts[src], py = pts[src + 1u], pz = pts[src + 2u];
      const double nx = nrm[src], ny = nrm[src + 1u], nz = nrm[src + 2u];
      out_p[dst] = ((px * t0 + py * t1) + pz * t2) + sx;
      out_p[dst + 1u] = ((px * t3 + py * t4) + pz * t5) + sy;
      out_p[dst + 2u] = ((px * t6 + py * t7) + pz * t8) + sz;
      out_n[dst] = (nx * t0 + ny * t1) + nz * t2;
      out_n[dst + 1u] = (nx * t3 + ny * t4) + nz * t5;
      out_n[dst + 2u] = (nx * t6 + ny * t7) + nz * t8;
    }
    base += (size_t)n;
  }
}

static void rs_sort_unique(double *ts, int *cnt)
{
  const int n = *cnt;
  for (int i = 1; i < n; i++)
  {
    const double v = ts[i];
    int          j = i - 1;
    while (j >= 0 && ts[j] > v)
    {
      ts[j + 1] = ts[j];
      j--;
    }
    ts[j + 1] = v;
  }
  int m = (n > 0) ? 1 : 0;
  for (int i = 1; i < n; i++)
  {
    if (ts[i] != ts[m - 1])
    {
      ts[m++] = ts[i];
    }
  }
  *cnt = m;
}

/** Direct polyline raster + haze shading matching `ground._laid` + `ground._shaded` (0-ULP). */
int rs_ground_shaded(int nseg, const double *restrict fx, const double *restrict fy,
                     const double *restrict fd, const int32_t *restrict seg_s,
                     const int32_t *restrict seg_line, const double *restrict seg_fade,
                     int horizon_id, int width, int height, double near_depth, double reach,
                     double *restrict ts_buf, int ts_cap, int32_t *restrict cell_slot,
                     uint64_t *restrict line_bits, int64_t *restrict out_cell,
                     int64_t *restrict out_mask, int64_t *restrict out_grey,
                     double *restrict acc_depth, double *restrict acc_fade,
                     int32_t *restrict acc_dots, uint8_t *restrict acc_hz)
{
  static const int64_t BBITS[2][4] = {{0x01, 0x02, 0x04, 0x40}, {0x08, 0x10, 0x20, 0x80}};
  const int            ncells = width * height;
  for (int i = 0; i < ncells; i++)
  {
    cell_slot[i] = -1;
  }
  int m = 0;
  for (int g = 0; g < nseg; g++)
  {
    const int    s = (int)seg_s[g];
    const double x0 = fx[s], y0 = fy[s], d0 = fd[s];
    const double x1 = fx[s + 1], y1 = fy[s + 1], d1 = fd[s + 1];
    const double dx = x1 - x0, dy = y1 - y0, dd = d1 - d0;
    int          nt = 0;
    ts_buf[nt++] = 0.0;
    ts_buf[nt++] = 1.0;
    if (dx != 0.0)
    {
      const double a = (x0 * 2.0 < x1 * 2.0) ? (x0 * 2.0) : (x1 * 2.0);
      const double b = (x0 * 2.0 > x1 * 2.0) ? (x0 * 2.0) : (x1 * 2.0);
      const double lim = (double)(2 * width) + 1.0;
      double       lo = floor(a) + 1.0;
      double       hi = ceil(b);
      if (lo < 0.0)
      {
        lo = 0.0;
      }
      if (hi > lim)
      {
        hi = lim;
      }
      for (double i = lo; i < hi && nt < ts_cap; i += 1.0)
      {
        ts_buf[nt++] = (i / 2.0 - x0) / dx;
      }
    }
    if (dy != 0.0)
    {
      const double a = (y0 * 4.0 < y1 * 4.0) ? (y0 * 4.0) : (y1 * 4.0);
      const double b = (y0 * 4.0 > y1 * 4.0) ? (y0 * 4.0) : (y1 * 4.0);
      const double lim = (double)(4 * height) + 1.0;
      double       lo = floor(a) + 1.0;
      double       hi = ceil(b);
      if (lo < 0.0)
      {
        lo = 0.0;
      }
      if (hi > lim)
      {
        hi = lim;
      }
      for (double i = lo; i < hi && nt < ts_cap; i += 1.0)
      {
        ts_buf[nt++] = (i / 4.0 - y0) / dy;
      }
    }
    rs_sort_unique(ts_buf, &nt);
    const int32_t lid = seg_line[g];
    const double  fade = seg_fade[g];
    for (int k = 0; k + 1 < nt; k++)
    {
      const double t = 0.5 * (ts_buf[k] + ts_buf[k + 1]);
      const double gx = x0 + dx * t, gy = y0 + dy * t, gd = d0 + dd * t;
      if (!(gx >= 0.0 && gx < (double)width && gy >= 0.0 && gy < (double)height))
      {
        continue;
      }
      const int px = (int)gx, py = (int)gy;
      if (px < 0 || px >= width || py < 0 || py >= height)
      {
        continue;
      }
      const int at = py * width + px;
      int       slot = (int)cell_slot[at];
      if (slot < 0)
      {
        slot = m++;
        cell_slot[at] = (int32_t)slot;
        out_cell[slot] = (int64_t)at;
        out_mask[slot] = 0;
        acc_hz[slot] = 0u;
        acc_depth[slot] = 0.0;
        acc_dots[slot] = 0;
        line_bits[slot] = 0ull;
        acc_fade[slot] = fade;
      }
      else if (fade < 0.5)
      {
        continue;
      }
      const int col = (gx - (double)px >= 0.5) ? 1 : 0;
      int       row = (int)((gy - (double)py) * 4.0);
      if (row > 3)
      {
        row = 3;
      }
      out_mask[slot] |= BBITS[col][row];
      if (lid == horizon_id)
      {
        acc_hz[slot] = 1u;
      }
      acc_depth[slot] += gd;
      acc_dots[slot] += 1;
      line_bits[slot] |= (1ull << (uint32_t)lid);
      if (fade > acc_fade[slot])
      {
        acc_fade[slot] = fade;
      }
    }
  }
  for (int s = 0; s < m; s++)
  {
    uint64_t bits = line_bits[s];
    int      nlines = 0;
    while (bits != 0ull)
    {
      bits &= (bits - 1ull);
      nlines++;
    }
    const double g = acc_hz[s]
                       ? 128.0
                       : (118.0 * pow(0.4, ((acc_depth[s] / (double)acc_dots[s]) - near_depth) / reach));
    out_grey[s] = (int64_t)((g * acc_fade[s]) / sqrt((double)nlines) + 0.5);
  }
  return m;
}
