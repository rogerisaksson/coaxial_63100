/** raster_cells.c - C11 2x4 Braille dot-to-cell fold, lit Braille screen, and cloth drape kernels. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>

static const uint8_t RS_DOT_BITS[8] = {0x01u, 0x08u, 0x02u, 0x10u, 0x04u, 0x20u, 0x40u, 0x80u};

/** Fold 2x4 fine dot rasters `(4*height, 2*width)` down to cells matching `engine.fold`. */
void rs_fold(const double *restrict depth, const uint8_t *restrict top, const uint8_t *restrict sun,
             int width, int height, double *restrict out_depth, uint8_t *restrict out_top,
             uint8_t *restrict out_sun, double *restrict out_cov, uint8_t *restrict out_reach)
{
  if (width <= 0 || height <= 0)
  {
    return;
  }
  const size_t fine_w = (size_t)width * 2u;
  for (int cy = 0; cy < height; cy++)
  {
    const size_t base_y = (size_t)cy * 4u;
    const size_t cell_row = (size_t)cy * (size_t)width;
    for (int cx = 0; cx < width; cx++)
    {
      const size_t base_x = (size_t)cx * 2u;
      const size_t cell = cell_row + (size_t)cx;
      uint8_t      mask = 0u;
      int          hits = 0;
      int          best_k = 0;
      double       best_d = depth[base_y * fine_w + base_x];
      for (int k = 0; k < 8; k++)
      {
        const size_t dy = (size_t)(k >> 1);
        const size_t dx = (size_t)(k & 1);
        const double d = depth[(base_y + dy) * fine_w + (base_x + dx)];
        if (d != 0.0)
        {
          hits++;
          mask = (uint8_t)(mask | RS_DOT_BITS[k]);
        }
        if (k > 0 && d > best_d)
        {
          best_d = d;
          best_k = k;
        }
      }
      if (hits > 0)
      {
        const size_t near_idx =
          (base_y + (size_t)(best_k >> 1)) * fine_w + (base_x + (size_t)(best_k & 1));
        out_depth[cell] = best_d;
        out_top[cell] = top[near_idx];
        out_sun[cell] = sun[near_idx];
        out_cov[cell] = (double)hits * 0.125;
        out_reach[cell] = mask;
      }
      else
      {
        out_depth[cell] = 0.0;
        out_top[cell] = 0u;
        out_sun[cell] = 0u;
        out_cov[cell] = 0.0;
        out_reach[cell] = 0u;
      }
    }
  }
}

static inline double rs_screen_thresh(int x, int y, uint8_t hand, const double *restrict noise64)
{
  if (hand >= 248u)
  {
    return noise64[(((size_t)y & 63u) << 6u) + ((size_t)x & 63u)];
  }
  if (hand >= 208u)
  {
    return 0.0;
  }
  if (hand < 96u)
  {
    const double r0 = (double)(((x + y) & 3) + 0.5) * 0.25;
    return (hand < 32u) ? r0 : ((r0 > 0.75) ? r0 : 0.0);
  }
  const double r1 = (double)(((x - y) & 3) + 0.5) * 0.25;
  return (hand < 160u) ? r1 : ((r1 > 0.75) ? r1 : 0.0);
}

static void rs_braille_cell(int cx, int cy, int W, int H, const double *restrict depth,
                            const uint8_t *restrict rgb, int channels,
                            const double *restrict floor_b, const double *restrict noise64,
                            const uint8_t *restrict over, const uint8_t *restrict alone,
                            const uint8_t *restrict leaders, const double *restrict inks6,
                            uint8_t *restrict out_mask, uint8_t *restrict out_hits,
                            double *restrict out_rgb)
{
  const int    x0 = cx * 2, y0 = cy * 4;
  const size_t stride = (size_t)W, ch = (size_t)channels;
  int          cell_alone = 0;
  if (alone != NULL)
  {
    for (int dy = 0; dy < 4 && !cell_alone; dy++)
    {
      const size_t r_off = (size_t)(y0 + dy) * stride + (size_t)x0;
      cell_alone = (alone[r_off] != 0u) || (alone[r_off + 1u] != 0u);
    }
  }
  uint8_t mask = 0u, hits = 0u;
  int     inked_w = 0, right_n = 0, left_n = 0, any_lead = 0, mr = 0, mg = 0, mb = 0;
  double  shine = floor_b[(size_t)y0 * stride + (size_t)x0];

  for (int k = 0; k < 8; k++)
  {
    const int     y = y0 + (k >> 1), x = x0 + (k & 1);
    const size_t  p = (size_t)y * stride + (size_t)x, co = p * ch;
    const double  d = depth[p], fl = floor_b[p];
    const uint8_t cr = rgb[co], cg = rgb[co + 1u], cb = rgb[co + 2u];
    const uint8_t hand = (channels > 3) ? rgb[co + 3u] : 255u;
    const int     miss = (hand >= 248u) ? (int)(255u - hand) : (int)(hand & 3u);
    const int     sub_h = (d > 0.0) ? (4 - miss) : 4;
    mr += sub_h * (int)cr;
    mg += sub_h * (int)cg;
    mb += sub_h * (int)cb;
    if (fl > shine)
    {
      shine = fl;
    }
    const double lum = ((double)cr * (0.2126 / 255.0) + (double)cg * (0.7152 / 255.0)) +
                       (double)cb * (0.0722 / 255.0);
    int on_dot = 0;
    if (d > 0.0)
    {
      hits++;
      inked_w += (lum > 0.004) ? sub_h : 0;
      if (channels > 3 && hand < 208u)
      {
        right_n += (hand < 96u) ? 1 : 0;
        left_n += (hand >= 96u) ? 1 : 0;
      }
      const double th = 0.04 * d;
      const double s0 = (x > 0) ? depth[p - 1u] : 0.0;
      const double s1 = (x + 1 < W) ? depth[p + 1u] : 0.0;
      const double s2 = (y > 0) ? depth[p - stride] : 0.0;
      const double s3 = (y + 1 < H) ? depth[p + stride] : 0.0;
      const int    e0 = (s0 == 0.0 || (d - s0) > th), e1 = (s1 == 0.0 || (d - s1) > th);
      const int    e2 = (s2 == 0.0 || (d - s2) > th), e3 = (s3 == 0.0 || (d - s3) > th);
      const int    edge = (sub_h >= 2) ? (e0 || e1 || e2 || e3) : ((e0 && e1) || (e2 && e3));
      const double lum_c = (lum < 0.0) ? 0.0 : ((lum > 1.0) ? 1.0 : lum);
      const double keep = ((lum > 0.004) ? (0.40 + 0.60 * lum_c) : 0.0) * ((double)sub_h * 0.25);
      const int    lit_d = (!cell_alone) && (keep > rs_screen_thresh(x, y, hand, noise64));
      on_dot = lit_d || edge || (over != NULL && over[p] != 0u);
    }
    else
    {
      const int ld = (leaders != NULL && leaders[p] != 0u);
      any_lead |= ld;
      on_dot = (fl > 0.0) || ld || (over != NULL && over[p] != 0u);
    }
    mask = on_dot ? (uint8_t)(mask | RS_DOT_BITS[k]) : mask;
  }
  *out_mask = mask;
  *out_hits = hits;
  if (hits == 0u)
  {
    out_rgb[0] = any_lead ? inks6[3] : (inks6[0] * shine);
    out_rgb[1] = any_lead ? inks6[4] : (inks6[1] * shine);
    out_rgb[2] = any_lead ? inks6[5] : (inks6[2] * shine);
    return;
  }
  const double inv_i = 1.0 / (double)((inked_w > 4) ? inked_w : 4);
  double       br = ((double)mr * inv_i) * 1.2 + 18.0;
  double       bg = ((double)mg * inv_i) * 1.2 + 18.0;
  double       bb = ((double)mb * inv_i) * 1.2 + 18.0;
  br = (br < 0.0) ? 0.0 : ((br > 255.0) ? 255.0 : br);
  bg = (bg < 0.0) ? 0.0 : ((bg > 255.0) ? 255.0 : bg);
  bb = (bb < 0.0) ? 0.0 : ((bb > 255.0) ? 255.0 : bb);
  if (right_n + left_n > 0)
  {
    const int    way = (right_n >= left_n) ? 1 : -1;
    const double tow = (((cx + way * cy) & 3) < 2) ? 1.0 : 0.0;
    const double k_tow = 1.0 - 0.6 * tow * (double)(right_n + left_n) / (double)hits;
    br *= k_tow;
    bg *= k_tow;
    bb *= k_tow;
  }
  out_rgb[0] = br;
  out_rgb[1] = bg;
  out_rgb[2] = bb;
}

/** Fold `(4*height, 2*width)` lit dot rasters into Braille bitmasks, hit counts, and cell RGB. */
void rs_braille(const double *restrict depth, const uint8_t *restrict rgb, int channels,
                const double *restrict floor_b, const double *restrict noise64,
                const uint8_t *restrict over, const uint8_t *restrict alone,
                const uint8_t *restrict leaders, const double *restrict inks6, int width,
                int height, uint8_t *restrict out_cells, uint8_t *restrict out_hits,
                double *restrict out_ink)
{
  if (width <= 0 || height <= 0)
  {
    return;
  }
  const int W = width * 2, H = height * 4;
  for (int cy = 0; cy < height; cy++)
  {
    for (int cx = 0; cx < width; cx++)
    {
      const size_t cell = (size_t)cy * (size_t)width + (size_t)cx;
      rs_braille_cell(cx, cy, W, H, depth, rgb, channels, floor_b, noise64, over, alone, leaders,
                      inks6, &out_cells[cell], &out_hits[cell], &out_ink[cell * 3u]);
    }
  }
}

static inline void rs_drape_store(int64_t dst_i, int64_t slot_i, double w, double qx, double qy,
                                  double qz, double fx, double fy, double fz,
                                  const double *restrict hinge3, double *restrict pos,
                                  double *restrict norm, double *restrict sums)
{
  if (sums == NULL)
  {
    const size_t dst = (size_t)dst_i * 3u;
    pos[dst] = qx;
    pos[dst + 1u] = qy;
    pos[dst + 2u] = qz;
    norm[dst] = fx;
    norm[dst + 1u] = fy;
    norm[dst + 2u] = fz;
  }
  else
  {
    const size_t so = (size_t)slot_i * 10u;
    sums[so] += qx * w;
    sums[so + 1u] += qy * w;
    sums[so + 2u] += qz * w;
    sums[so + 3u] += fx * w;
    sums[so + 4u] += fy * w;
    sums[so + 5u] += fz * w;
    sums[so + 6u] += hinge3[0] * w;
    sums[so + 7u] += hinge3[1] * w;
    sums[so + 8u] += hinge3[2] * w;
    sums[so + 9u] += w;
  }
}

/** Skin one garment row's `full` and `mid` vertices directly into `(pos, norm)` or `sums`. */
void rs_drape_row(int nfull, const int64_t *restrict full_idx, const double *restrict there,
                  const double *restrict facing, const double *restrict turn9,
                  const double *restrict spot3, int nmid, const int64_t *restrict mid_idx,
                  const double *restrict share, const double *restrict at_pts,
                  const double *restrict out_nrm, const double *restrict about3, int nrot,
                  const int32_t *restrict rot_axes, const double *restrict rot_rads,
                  const double *restrict base9, const double *restrict hinge3,
                  const int64_t *restrict slot_full, const double *restrict w_full,
                  const int64_t *restrict slot_mid, const double *restrict w_mid,
                  double *restrict pos, double *restrict norm, double *restrict sums)
{
  const double t0 = turn9[0], t1 = turn9[1], t2 = turn9[2];
  const double t3 = turn9[3], t4 = turn9[4], t5 = turn9[5];
  const double t6 = turn9[6], t7 = turn9[7], t8 = turn9[8];
  const double sx = spot3[0], sy = spot3[1], sz = spot3[2];

  for (int k = 0; k < nfull; k++)
  {
    const size_t o = (size_t)k * 3u;
    const double px = there[o], py = there[o + 1u], pz = there[o + 2u];
    const double nx = facing[o], ny = facing[o + 1u], nz = facing[o + 2u];
    rs_drape_store(full_idx[k], (sums != NULL) ? slot_full[k] : 0, (sums != NULL) ? w_full[k] : 0.0,
                   ((px * t0 + py * t1) + pz * t2) + sx, ((px * t3 + py * t4) + pz * t5) + sy,
                   ((px * t6 + py * t7) + pz * t8) + sz, (nx * t0 + ny * t1) + nz * t2,
                   (nx * t3 + ny * t4) + nz * t5, (nx * t6 + ny * t7) + nz * t8, hinge3, pos, norm,
                   sums);
  }

  const double b0 = base9[0], b1 = base9[1], b2 = base9[2];
  const double b3 = base9[3], b4 = base9[4], b5 = base9[5];
  const double b6 = base9[6], b7 = base9[7], b8 = base9[8];
  const double hx = hinge3[0], hy = hinge3[1], hz = hinge3[2];
  const double ax = (about3 != NULL) ? about3[0] : 0.0;
  const double ay = (about3 != NULL) ? about3[1] : 0.0;
  const double az = (about3 != NULL) ? about3[2] : 0.0;

  for (int k = 0; k < nmid; k++)
  {
    const size_t o = (size_t)k * 3u;
    double       p[3] = {at_pts[o] - ax, at_pts[o + 1u] - ay, at_pts[o + 2u] - az};
    double       n[3] = {out_nrm[o], out_nrm[o + 1u], out_nrm[o + 2u]};
    const double sh = share[k];
    for (int r = 0; r < nrot; r++)
    {
      const int    ia = (int)rot_axes[2 * r], ib = (int)rot_axes[2 * r + 1];
      const double ang = rot_rads[r] * sh;
      const double c = cos(ang), s = sin(ang);
      const double pi = p[ia], pj = p[ib], ni = n[ia], nj = n[ib];
      p[ia] = c * pi - s * pj;
      p[ib] = s * pi + c * pj;
      n[ia] = c * ni - s * nj;
      n[ib] = s * ni + c * nj;
    }
    p[0] += ax;
    p[1] += ay;
    p[2] += az;
    rs_drape_store(mid_idx[k], (sums != NULL) ? slot_mid[k] : 0, (sums != NULL) ? w_mid[k] : 0.0,
                   ((p[0] * b0 + p[1] * b1) + p[2] * b2) + hx,
                   ((p[0] * b3 + p[1] * b4) + p[2] * b5) + hy,
                   ((p[0] * b6 + p[1] * b7) + p[2] * b8) + hz, (n[0] * b0 + n[1] * b1) + n[2] * b2,
                   (n[0] * b3 + n[1] * b4) + n[2] * b5, (n[0] * b6 + n[1] * b7) + n[2] * b8, hinge3,
                   pos, norm, sums);
  }
}

/** Finalize two-sided weighted garment `sums` into `(pos, norm)` at `held_idx` preserving reach. */
void rs_drape_finish(int nheld, const int64_t *restrict held_idx, const double *restrict sums,
                     double *restrict pos, double *restrict norm)
{
  for (int k = 0; k < nheld; k++)
  {
    const size_t so = (size_t)k * 10u;
    const size_t dst = (size_t)held_idx[k] * 3u;
    const double inv_w = 1.0 / sums[so + 9u];
    const double px = sums[so] * inv_w, py = sums[so + 1u] * inv_w, pz = sums[so + 2u] * inv_w;
    const double fx = sums[so + 3u] * inv_w, fy = sums[so + 4u] * inv_w, fz = sums[so + 5u] * inv_w;
    const double jx = sums[so + 6u] * inv_w, jy = sums[so + 7u] * inv_w, jz = sums[so + 8u] * inv_w;
    const double rx = pos[dst] - jx, ry = pos[dst + 1u] - jy, rz = pos[dst + 2u] - jz;
    const double reach = sqrt((rx * rx + ry * ry) + rz * rz);
    const double ax = px - jx, ay = py - jy, az = pz - jz;
    const double anrm = sqrt((ax * ax + ay * ay) + az * az);
    const double scale = reach / ((anrm > 1e-12) ? anrm : 1e-12);
    pos[dst] = jx + ax * scale;
    pos[dst + 1u] = jy + ay * scale;
    pos[dst + 2u] = jz + az * scale;
    norm[dst] = fx;
    norm[dst + 1u] = fy;
    norm[dst + 2u] = fz;
  }
}
