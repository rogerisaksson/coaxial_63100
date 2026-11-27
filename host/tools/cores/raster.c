/** raster.c - O(N) C11 3D mesh transform, projection, and 2x2 dot-splat rasterizer kernels. */
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
