/** raster_fsaa.c - C11 4x RGSS (4-rooks) FSAA indexed lit triangle rasterizer and shader. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const double RS_ROOK_X[4] = {-0.375, 0.125, -0.125, 0.375};
static const double RS_ROOK_Y[4] = {-0.125, -0.375, 0.375, 0.125};

static void rf_project_lit(int nverts, const double *restrict pos, const double *restrict nrm,
                           const uint32_t *restrict materials, const double *restrict m9,
                           const double *restrict centre, double distance, double scale, double cx,
                           double cy, double aspect, uint32_t under_bit, double layer_m,
                           double *restrict vbuf)
{
  const double m0 = m9[0], m1 = m9[1], m2 = m9[2];
  const double m3 = m9[3], m4 = m9[4], m5 = m9[5];
  const double m6 = m9[6], m7 = m9[7], m8 = m9[8];
  const double c0 = centre[0], c1 = centre[1], c2 = centre[2];

  for (int i = 0; i < nverts; i++)
  {
    const size_t o = (size_t)i * 3u;
    const size_t vo = (size_t)i * 7u;
    const double qx = pos[o] - c0, qy = pos[o + 1u] - c1, qz = pos[o + 2u] - c2;
    const double tx = (m0 * qx + m1 * qy) + m2 * qz;
    const double ty = (m3 * qx + m4 * qy) + m5 * qz;
    const double tz = (m6 * qx + m7 * qy) + m8 * qz;
    const double dz = distance - tz;
    if (!(dz > 1e-6) || !isfinite(tx) || !isfinite(ty))
    {
      vbuf[vo] = 0.0;
      vbuf[vo + 1u] = 0.0;
      vbuf[vo + 2u] = 0.0;
      vbuf[vo + 3u] = 0.0;
      vbuf[vo + 4u] = 0.0;
      vbuf[vo + 5u] = 0.0;
      vbuf[vo + 6u] = 0.0;
      continue;
    }
    const double w = 1.0 / dz;
    const double u_add = ((materials[i] & under_bit) != 0u) ? layer_m : 0.0;
    const double uw = 1.0 / (dz + u_add);
    const double nx = nrm[o], ny = nrm[o + 1u], nz = nrm[o + 2u];
    vbuf[vo] = cx + (scale * w) * tx;
    vbuf[vo + 1u] = cy - ((scale * aspect) * w) * ty;
    vbuf[vo + 2u] = w;
    vbuf[vo + 3u] = uw;
    vbuf[vo + 4u] = (m0 * nx + m1 * ny) + m2 * nz;
    vbuf[vo + 5u] = (m3 * nx + m4 * ny) + m5 * nz;
    vbuf[vo + 6u] = (m6 * nx + m7 * ny) + m8 * nz;
  }
}

static void rf_scan_tri(int tri_id, const double *va, const double *vb, const double *vc,
                        double area, int width, int height, int fsaa, float *restrict zbuf,
                        float *restrict obuf, int32_t *restrict tbuf, float *restrict bbuf)
{
  const double x0 = va[0], py0 = va[1], x1 = vb[0], py1 = vb[1], x2 = vc[0], py2 = vc[1];
  const double oa = va[2], ob = vb[2], oc = vc[2];
  const double ua = va[3], ub = vb[3], uc = vc[3];
  const double pad = fsaa ? 0.375 : 0.0;
  const double w_lim = (double)(width - 1);
  const double h_lim = (double)(height - 1);
  double       min_x = (x0 < x1) ? ((x0 < x2) ? x0 : x2) : ((x1 < x2) ? x1 : x2);
  double       max_x = (x0 > x1) ? ((x0 > x2) ? x0 : x2) : ((x1 > x2) ? x1 : x2);
  double       min_y = (py0 < py1) ? ((py0 < py2) ? py0 : py2) : ((py1 < py2) ? py1 : py2);
  double       max_y = (py0 > py1) ? ((py0 > py2) ? py0 : py2) : ((py1 > py2) ? py1 : py2);
  min_x -= pad;
  max_x += pad;
  min_y -= pad;
  max_y += pad;
  if (!(max_x >= 0.0 && min_x <= w_lim && max_y >= 0.0 && min_y <= h_lim))
  {
    return;
  }
  const int    lo_x = (min_x <= 0.0) ? 0 : (int)ceil(min_x);
  const int    hi_x = (max_x >= w_lim) ? (width - 1) : (int)floor(max_x);
  const int    lo_y = (min_y <= 0.0) ? 0 : (int)ceil(min_y);
  const int    hi_y = (max_y >= h_lim) ? (height - 1) : (int)floor(max_y);
  const double inv = 1.0 / area;
  const double e0x = x2 - x1, e0y = py2 - py1;
  const double e1x = x0 - x2, e1y = py0 - py2;
  const int    ns = fsaa ? 4 : 1;

  for (int py = lo_y; py <= hi_y; py++)
  {
    const size_t row = (size_t)py * (size_t)width;
    for (int px = lo_x; px <= hi_x; px++)
    {
      const size_t base = (row + (size_t)px) * (size_t)ns;
      for (int s = 0; s < ns; s++)
      {
        const double qx = (double)px + (fsaa ? RS_ROOK_X[s] : 0.0);
        const double qy = (double)py + (fsaa ? RS_ROOK_Y[s] : 0.0);
        const double w0 = (e0x * (qy - py1) - e0y * (qx - x1)) * inv;
        const double w1 = (e1x * (qy - py2) - e1y * (qx - x2)) * inv;
        const double w2 = (1.0 - w0) - w1;
        if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0)
        {
          const float zt = (float)((w0 * ua + w1 * ub) + w2 * uc);
          const size_t si = base + (size_t)s;
          if (zt > zbuf[si])
          {
            zbuf[si] = zt;
            obuf[si] = (float)((w0 * oa + w1 * ob) + w2 * oc);
            tbuf[si] = (int32_t)tri_id;
            bbuf[si * 2u] = (float)w0;
            bbuf[si * 2u + 1u] = (float)w1;
          }
        }
      }
    }
  }
}

static void rf_shade_sample(double nx, double ny, double nz, double u, double v, uint32_t mat,
                            const double *restrict key3, const double *restrict fill3,
                            const double *restrict half3, const double *restrict pal12,
                            uint8_t *restrict rgba4)
{
  double nlen = sqrt((nx * nx + ny * ny) + nz * nz);
  if (nlen < 1e-9)
  {
    nlen = 1e-9;
  }
  nx /= nlen;
  ny /= nlen;
  nz /= nlen;
  if (nz < 0.0)
  {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
  double lam = (nx * key3[0] + ny * key3[1]) + nz * key3[2];
  lam = (lam < 0.0) ? 0.0 : lam;
  double fd = (nx * fill3[0] + ny * fill3[1]) + nz * fill3[2];
  fd = (fd < 0.0) ? 0.0 : fd;
  const double   fil = 0.35 * fd;
  const double   cz = (nz < 0.0) ? 0.0 : ((nz > 1.0) ? 1.0 : nz);
  const double   rim = (1.0 - cz) * (1.0 - cz) * (1.0 - cz);
  double         lobe = (nx * half3[0] + ny * half3[1]) + nz * half3[2];
  lobe = (lobe < 0.0) ? 0.0 : lobe;
  const int      painted = (mat >= (1u << 24));
  const int      woven = (mat & (1u << 25)) != 0u;
  const int      twill = (mat & (1u << 27)) != 0u;
  const int      hard = (mat & (1u << 28)) != 0u;
  const double   spec = pow(lobe, twill ? 160.0 : 40.0) +
                        (twill ? ((0.6 / 0.9) * pow(lobe, 28.0)) : 0.0);
  double b0, b1, b2;
  if (painted)
  {
    b0 = (double)((mat >> 16) & 255u) / 255.0;
    b1 = (double)((mat >> 8) & 255u) / 255.0;
    b2 = (double)(mat & 255u) / 255.0;
  }
  else
  {
    const uint32_t pi = (mat < 3u) ? mat : 3u;
    b0 = pal12[pi * 3u];
    b1 = pal12[pi * 3u + 1u];
    b2 = pal12[pi * 3u + 2u];
  }
  if (mat == 0u)
  {
    double       ip0, ip1;
    double       fu = modf(u * 22.0, &ip0), fv = modf(v * 30.0, &ip1);
    if (fu < 0.0)
    {
      fu += 1.0;
    }
    if (fv < 0.0)
    {
      fv += 1.0;
    }
    const double g0 = fabs(fu - 0.5), g1 = fabs(fv - 0.5);
    const double gm = (g0 > g1) ? g0 : g1;
    double       st = (gm - 0.36) * 10.0;
    st = (st < 0.0) ? 0.0 : ((st > 1.0) ? 1.0 : st);
    const double att = 1.0 - 0.5 * (st * st * (3.0 - 2.0 * st));
    b0 *= att;
    b1 *= att;
    b2 *= att;
  }
  double c0, c1, c2;
  if (mat == 3u)
  {
    const double cf = 0.75 + 0.25 * lam;
    c0 = b0 * cf;
    c1 = b1 * cf;
    c2 = b2 * cf;
  }
  else
  {
    const double base_sh = painted ? 0.3 : 0.55;
    const double shine = (mat == 1u) ? 0.12 : (woven ? 0.0 : (twill ? 0.9 : base_sh));
    const double df = 0.10 + 0.85 * lam + fil, sp = spec * shine;
    const double rf = rim * (woven ? 0.15 : (hard ? 0.3 : 0.8));
    c0 = b0 * df + sp + rf * 0.55;
    c1 = b1 * df + sp + rf * 0.75;
    c2 = b2 * df + sp + rf * 1.0;
  }
  c0 = (c0 < 0.0) ? 0.0 : ((c0 > 1.0) ? 1.0 : c0);
  c1 = (c1 < 0.0) ? 0.0 : ((c1 > 1.0) ? 1.0 : c1);
  c2 = (c2 < 0.0) ? 0.0 : ((c2 > 1.0) ? 1.0 : c2);
  rgba4[0] = (uint8_t)(c0 * 255.0 + 0.5);
  rgba4[1] = (uint8_t)(c1 * 255.0 + 0.5);
  rgba4[2] = (uint8_t)(c2 * 255.0 + 0.5);
  rgba4[3] = twill ? (uint8_t)(((nx > 0.0) ? 0 : 128) + (hard ? 64 : 0)) : (hard ? 224u : 255u);
}

static void rf_resolve_pixel(size_t pix, int ns, const int32_t *restrict idx,
                             const uint32_t *restrict materials, const double *restrict uv,
                             const double *restrict vbuf, const float *restrict obuf,
                             const int32_t *restrict tbuf, const float *restrict bbuf,
                             const double *restrict key3, const double *restrict fill3,
                             const double *restrict half3, const double *restrict pal12,
                             double *restrict out_depth, uint8_t *restrict out_rgba)
{
  const size_t base = pix * (size_t)ns;
  int          any_hits = 0, best_s = -1, sec_s = -1;
  float        best_o = 0.0f, sec_o = 0.0f;
  int          sum_r = 0, sum_g = 0, sum_b = 0;
  uint8_t      h_arr[4] = {255u, 255u, 255u, 255u};

  for (int s = 0; s < ns; s++)
  {
    const size_t  si = base + (size_t)s;
    const int32_t tid = tbuf[si];
    if (tid < 0)
    {
      continue;
    }
    any_hits++;
    const float oo = obuf[si];
    if (oo > best_o)
    {
      sec_o = best_o;
      sec_s = best_s;
      best_o = oo;
      best_s = s;
    }
    else if (oo > sec_o)
    {
      sec_o = oo;
      sec_s = s;
    }
    const uint32_t raw = (uint32_t)tid;
    const int      swapped = (int)(raw & 1u);
    const size_t   to = (size_t)(raw >> 1u) * 3u;
    const int    ia = (int)idx[to];
    const int    ib = (int)idx[to + (swapped ? 2u : 1u)];
    const int    ic = (int)idx[to + (swapped ? 1u : 2u)];
    const double w0 = (double)bbuf[si * 2u], w1 = (double)bbuf[si * 2u + 1u];
    const double w2 = (1.0 - w0) - w1;
    const double *va = vbuf + (size_t)ia * 7u;
    const double *vb = vbuf + (size_t)ib * 7u;
    const double *vc = vbuf + (size_t)ic * 7u;
    const double  nx = (w0 * va[4] + w1 * vb[4]) + w2 * vc[4];
    const double  ny = (w0 * va[5] + w1 * vb[5]) + w2 * vc[5];
    const double  nz = (w0 * va[6] + w1 * vb[6]) + w2 * vc[6];
    const double  u = (w0 * uv[(size_t)ia * 2u] + w1 * uv[(size_t)ib * 2u]) +
                      w2 * uv[(size_t)ic * 2u];
    const double  v = (w0 * uv[(size_t)ia * 2u + 1u] + w1 * uv[(size_t)ib * 2u + 1u]) +
                      w2 * uv[(size_t)ic * 2u + 1u];
    uint8_t       rgba4[4];
    rf_shade_sample(nx, ny, nz, u, v, materials[ia], key3, fill3, half3, pal12, rgba4);
    sum_r += (int)rgba4[0];
    sum_g += (int)rgba4[1];
    sum_b += (int)rgba4[2];
    h_arr[s] = rgba4[3];
  }

  if (any_hits == 0)
  {
    out_depth[pix] = 0.0;
    memset(out_rgba + pix * 4u, 0, 4u);
    return;
  }
  int fg_hits = 0;
  for (int s = 0; s < ns; s++)
  {
    if (tbuf[base + (size_t)s] >= 0 && obuf[base + (size_t)s] >= best_o * 0.96f)
    {
      fg_hits++;
    }
  }
  const int     rep_s = (ns == 4 && fg_hits == 1 && any_hits >= 2 && sec_s >= 0) ? sec_s : best_s;
  const uint8_t h_rep = h_arr[rep_s];
  const int     miss = (ns == 4) ? (4 - any_hits) : 0;
  out_depth[pix] = (double)obuf[base + (size_t)rep_s];
  out_rgba[pix * 4u] = (uint8_t)((sum_r * 2 + any_hits) / (2 * any_hits));
  out_rgba[pix * 4u + 1u] = (uint8_t)((sum_g * 2 + any_hits) / (2 * any_hits));
  out_rgba[pix * 4u + 2u] = (uint8_t)((sum_b * 2 + any_hits) / (2 * any_hits));
  out_rgba[pix * 4u + 3u] = (h_rep >= 248u) ? (uint8_t)(255 - miss) : (uint8_t)((int)h_rep + miss);
}

/** Rasterize indexed lit mesh with 4x RGSS (4-rooks) FSAA matching `gpu_lit.LitRaster`. */
void rs_lit_mesh_fsaa(int nverts, const double *restrict pos, const double *restrict nrm,
                      const double *restrict uv, const uint32_t *restrict materials, int ntris,
                      const int32_t *restrict idx, const double *restrict m9,
                      const double *restrict centre, const double *restrict cam7,
                      uint32_t under_bit, int width, int height, int fsaa,
                      const double *restrict key3, const double *restrict fill3,
                      const double *restrict half3, const double *restrict pal12,
                      double *restrict vbuf, float *restrict zbuf, float *restrict obuf,
                      int32_t *restrict tbuf, float *restrict bbuf, double *restrict out_depth,
                      uint8_t *restrict out_rgba)
{
  if (width <= 0 || height <= 0)
  {
    return;
  }
  const int    ns = fsaa ? 4 : 1;
  const size_t npix = (size_t)width * (size_t)height;
  const size_t nsub = npix * (size_t)ns;
  memset(zbuf, 0, nsub * sizeof(float));
  for (size_t i = 0u; i < nsub; i++)
  {
    tbuf[i] = -1;
  }
  if (nverts <= 0 || ntris <= 0)
  {
    memset(out_depth, 0, npix * sizeof(double));
    memset(out_rgba, 0, npix * 4u);
    return;
  }
  rf_project_lit(nverts, pos, nrm, materials, m9, centre, cam7[0], cam7[1], cam7[2], cam7[3],
                 cam7[4], under_bit, cam7[5], vbuf);

  for (int t = 0; t < ntris; t++)
  {
    if ((uint32_t)t >= 0x40000000u)
    {
      break;
    }
    const size_t to = (size_t)t * 3u;
    const int    ia = (int)idx[to], ib = (int)idx[to + 1u], ic = (int)idx[to + 2u];
    if (ia < 0 || ia >= nverts || ib < 0 || ib >= nverts || ic < 0 || ic >= nverts)
    {
      continue;
    }
    const double *va = vbuf + (size_t)ia * 7u;
    const double *vb = vbuf + (size_t)ib * 7u;
    const double *vc = vbuf + (size_t)ic * 7u;
    if (va[2] == 0.0 || vb[2] == 0.0 || vc[2] == 0.0)
    {
      continue;
    }
    const double area = (vb[0] - va[0]) * (vc[1] - va[1]) - (vb[1] - va[1]) * (vc[0] - va[0]);
    if (!isfinite(area) || area == 0.0)
    {
      continue;
    }
    const uint32_t ut = (uint32_t)t;
    if (area < 0.0)
    {
      rf_scan_tri((int)((ut << 1u) | 1u), va, vc, vb, -area, width, height, fsaa, zbuf, obuf, tbuf,
                  bbuf);
    }
    else
    {
      rf_scan_tri((int)(ut << 1u), va, vb, vc, area, width, height, fsaa, zbuf, obuf, tbuf, bbuf);
    }
  }

  for (size_t pix = 0u; pix < npix; pix++)
  {
    rf_resolve_pixel(pix, ns, idx, materials, uv, vbuf, obuf, tbuf, bbuf, key3, fill3, half3,
                     pal12, out_depth, out_rgba);
  }
}
