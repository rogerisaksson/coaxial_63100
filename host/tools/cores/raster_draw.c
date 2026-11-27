/* SIMD C11 kernels for ascii3d triangle/cell rendering and lit splat shading. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline int rd_clamp_int(double v, int lo, int hi)
{
  if (!(v >= (double)lo))
  {
    return lo;
  }
  if (v >= (double)hi)
  {
    return hi;
  }
  return (int)v;
}

static void rd_ascii3d_fill(double *restrict depth, double *restrict value, int32_t *restrict who,
                            int32_t wear, double lit, const double *tri9, double area,
                            int first, int last, int left, int right, int top, int cols)
{
  const double x0 = tri9[0], py0 = tri9[1], x1 = tri9[2], py1 = tri9[3], x2 = tri9[4], py2 = tri9[5];
  const double oa = tri9[6], ob = tri9[7], og = tri9[8];
  const double inv = 1.0 / area;
  const double e0x = x2 - x1, e0y = py2 - py1;
  const double e1x = x0 - x2, e1y = py0 - py2;
  const double e2x = x1 - x0, e2y = py1 - py0;

  for (int py = first; py < last; py++)
  {
    const size_t row = (size_t)(py - top) * (size_t)cols;
    const double dy1 = (double)py - py1, dy2 = (double)py - py2, dy0 = (double)py - py0;
    for (int px = left; px < right; px++)
    {
      const double w0 = (e0x * dy1 - e0y * ((double)px - x1)) * inv;
      if (w0 < 0.0)
      {
        continue;
      }
      const double w1 = (e1x * dy2 - e1y * ((double)px - x2)) * inv;
      if (w1 < 0.0)
      {
        continue;
      }
      const double w2 = (e2x * dy0 - e2y * ((double)px - x0)) * inv;
      if (w2 < 0.0)
      {
        continue;
      }
      const double here = (w0 * oa + w1 * ob) + w2 * og;
      const size_t at = row + (size_t)px;
      if (here <= depth[at])
      {
        continue;
      }
      depth[at] = here;
      value[at] = lit;
      if (who != NULL)
      {
        who[at] = wear;
      }
    }
  }
}

/** Rasterise `n_tris` triangles into `(depth, value, who)` matching `ascii3d.rasterise` (0-ULP). */
void rs_ascii3d_raster(int n_verts, const double *restrict pos, int n_tris,
                       const int32_t *restrict idx, const double *restrict nrm,
                       const double *restrict m, const double *restrict cam8,
                       int cols, int top, int bottom, int cull, const int32_t *restrict tints,
                       double *restrict sx, double *restrict sy, double *restrict ooz,
                       double *restrict vx, double *restrict vy, double *restrict vz,
                       double *restrict depth, double *restrict value, int32_t *restrict who)
{
  const double distance = cam8[0], scale = cam8[1], cx = cam8[2], cy = cam8[3];
  const double lamp_x = cam8[4], lamp_y = cam8[5], lamp_z = cam8[6];
  const size_t deep = (size_t)(bottom - top) * (size_t)cols;
  memset(depth, 0, deep * sizeof(double));
  memset(value, 0, deep * sizeof(double));
  if (who != NULL)
  {
    memset(who, 0, deep * sizeof(int32_t));
  }
  for (int v = 0; v < n_verts; v++)
  {
    const size_t i = (size_t)v * 3u;
    const double px = pos[i], py = pos[i + 1u], pz = pos[i + 2u];
    const double z = (m[6] * px + m[7] * py) + m[8] * pz;
    const double behind = distance - z;
    if (behind <= 1e-6)
    {
      sx[v] = 0.0;
      sy[v] = 0.0;
      ooz[v] = 0.0;
      vx[v] = 0.0;
      vy[v] = 0.0;
      vz[v] = 0.0;
      continue;
    }
    const double x = (m[0] * px + m[1] * py) + m[2] * pz;
    const double y = (m[3] * px + m[4] * py) + m[5] * pz;
    const double near_v = 1.0 / behind;
    ooz[v] = near_v;
    vx[v] = x;
    vy[v] = y;
    vz[v] = z;
    sx[v] = cx + (scale * near_v) * x;
    sy[v] = cy - (scale * near_v) * y;
  }

  for (int tri = 0; tri < n_tris; tri++)
  {
    const size_t t = (size_t)tri * 3u;
    const int    a = (int)idx[t], b = (int)idx[t + 1u], c = (int)idx[t + 2u];
    double       oa = ooz[a], ob = ooz[b], og = ooz[c];
    if (oa == 0.0 || ob == 0.0 || og == 0.0)
    {
      continue;
    }
    double py0 = sy[a], py1 = sy[b], py2 = sy[c];
    double low = (py0 < py1) ? py0 : py1;
    if (py2 < low)
    {
      low = py2;
    }
    int first = rd_clamp_int(low, top - 2, bottom + 2);
    if (first >= bottom)
    {
      continue;
    }
    double high = (py0 > py1) ? py0 : py1;
    if (py2 > high)
    {
      high = py2;
    }
    int last = rd_clamp_int(high, top - 2, bottom + 2) + 1;
    if (last <= top)
    {
      continue;
    }
    double x0 = sx[a], x1 = sx[b], x2 = sx[c];
    double area = (x1 - x0) * (py2 - py0) - (py1 - py0) * (x2 - x0);
    if (area <= 0.0 && (cull || area == 0.0))
    {
      continue;
    }
    if (area <= 0.0)
    {
      const double tx = x1, ty = py1, to = ob;
      x1 = x2;
      py1 = py2;
      x2 = tx;
      py2 = ty;
      ob = og;
      og = to;
      area = -area;
    }
    const double min_x = (x0 < x1) ? ((x0 < x2) ? x0 : x2) : ((x1 < x2) ? x1 : x2);
    const double max_x = (x0 > x1) ? ((x0 > x2) ? x0 : x2) : ((x1 > x2) ? x1 : x2);
    int          left = rd_clamp_int(min_x, -2, cols + 2);
    int          right = rd_clamp_int(max_x, -2, cols + 2) + 1;
    const int tiny = (right - left == 1) && (last - first == 1);
    first = (first < top) ? top : first;
    last = (last > bottom) ? bottom : last;
    if (right <= 0 || left >= cols)
    {
      continue;
    }
    left = (left < 0) ? 0 : left;
    right = (right > cols) ? cols : right;

    double near_z = (oa > ob) ? oa : ob;
    if (og > near_z)
    {
      near_z = og;
    }
    const double n0 = nrm[t], n1 = nrm[t + 1u], n2 = nrm[t + 2u];
    double       nx = (m[0] * n0 + m[1] * n1) + m[2] * n2;
    double       ny = (m[3] * n0 + m[4] * n1) + m[5] * n2;
    double       nz = (m[6] * n0 + m[7] * n1) + m[8] * n2;
    const double ptx = ((vx[a] + vx[b]) + vx[c]) / 3.0;
    const double pty = ((vy[a] + vy[b]) + vy[c]) / 3.0;
    const double ptz = ((vz[a] + vz[b]) + vz[c]) / 3.0;
    double       ex = -ptx, ey = -pty, ez = distance - ptz;
    double       elen = sqrt((ex * ex + ey * ey) + ez * ez);
    if (elen == 0.0)
    {
      elen = 1.0;
    }
    ex /= elen;
    ey /= elen;
    ez /= elen;
    if ((nx * ex + ny * ey) + nz * ez < 0.0)
    {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
    double lx = lamp_x - ptx, ly = lamp_y - pty, lz = lamp_z - ptz;
    double llen = sqrt((lx * lx + ly * ly) + lz * lz);
    if (llen == 0.0)
    {
      llen = 1.0;
    }
    lx /= llen;
    ly /= llen;
    lz /= llen;
    const double  lam = (nx * lx + ny * ly) + nz * lz;
    const double  lit = (lam > 0.0) ? lam : 0.0;
    const int32_t wear = (who != NULL && tints != NULL) ? (tints[tri] + 1) : 0;
    const size_t  at = (size_t)(first - top) * (size_t)cols + (size_t)left;
    if (tiny)
    {
      if (near_z > depth[at])
      {
        if (who != NULL && tints != NULL)
        {
          who[at] = wear;
        }
        depth[at] = near_z;
        value[at] = lit;
      }
      continue;
    }
    const double tri9[9] = {x0, py0, x1, py1, x2, py2, oa, ob, og};
    rd_ascii3d_fill(depth, value, who, wear, lit, tri9, area, first, last, left, right, top, cols);
  }
}

static int rd_ascii3d_inked(const double *near_b, const int32_t *band, int at, int c, int r,
                            int width, int height)
{
  const double  here = near_b[at];
  const int32_t mine = band[at];
  const int     nb[4] = {(c > 0) ? (at - 1) : -1, (c + 1 < width) ? (at + 1) : -1,
                         (r > 0) ? (at - width) : -1, (r + 1 < height) ? (at + width) : -1};
  for (int k = 0; k < 4; k++)
  {
    const int other = nb[k];
    if (other < 0)
    {
      continue;
    }
    const double there = near_b[other];
    if (there == 0.0 || (here - there) > 0.10 * here)
    {
      return 1;
    }
    const int32_t jump = mine - band[other];
    if (jump >= 2 || jump <= -2)
    {
      return 1;
    }
  }
  return 0;
}

static void rd_ascii3d_wire(const double *near_b, const int32_t *paintbox, int has_shades,
                            const int32_t *shades_lut, int32_t ink_colour, int at, int c, int r,
                            int width, int height, double z_lo, double z_hi,
                            int32_t *out_code, int32_t *out_tone)
{
  const double here = near_b[at];
  int          horiz = 0, vert = 0;
  const int    nb[4] = {(c > 0) ? (at - 1) : -1, (c + 1 < width) ? (at + 1) : -1,
                        (r > 0) ? (at - width) : -1, (r + 1 < height) ? (at + width) : -1};
  for (int k = 0; k < 4; k++)
  {
    const int other = nb[k];
    if (other >= 0)
    {
      const double there = near_b[other];
      if (there == 0.0 || (here - there) > 0.018 * here)
      {
        if (k < 2)
        {
          horiz = 1;
        }
        else
        {
          vert = 1;
        }
      }
    }
  }
  if (!horiz && !vert)
  {
    *out_code = 0x20;
    *out_tone = -1;
    return;
  }
  *out_code = (horiz && vert) ? (int32_t)'+' : (horiz ? (int32_t)'|' : (int32_t)'-');
  int32_t wearing = (paintbox != NULL && paintbox[at] != 0) ? (paintbox[at] - 1) : ink_colour;
  if (wearing >= 0 && has_shades)
  {
    const double span = (z_hi - z_lo) != 0.0 ? (z_hi - z_lo) : 1.0;
    int          third = rd_clamp_int(3.0 * (here - z_lo) / span, 0, 2);
    if (wearing < 256)
    {
      wearing = shades_lut[wearing * 3 + third];
    }
  }
  *out_tone = wearing;
}

/** Downsample framebuffer into per-cell `(code, tone)` and trimmed `row_len` (`ascii3d.resolve`). */
void rs_ascii3d_resolve(const double *restrict depth, const double *restrict value,
                        const int32_t *restrict who, int width, int height, int cols,
                        int cell_rows, int supersample, const int32_t *restrict ramp_codes,
                        int ramp_len, int invert, int has_ink, int32_t ink_code,
                        int32_t ink_colour, int has_shades, const int32_t *restrict shades_lut,
                        int wire, double *restrict shade, double *restrict near_b,
                        int32_t *restrict paintbox, int32_t *restrict band,
                        int32_t *restrict out_code, int32_t *restrict out_tone,
                        int32_t *restrict out_len)
{
  const int32_t bg_code = invert ? ramp_codes[0] : ramp_codes[ramp_len - 1];
  const int     steps = ramp_len - 1;
  double        z_lo = 1e30, z_hi = 0.0;

  for (int r = 0; r < height; r++)
  {
    const int row = r * width;
    for (int c = 0; c < width; c++)
    {
      double  total = 0.0, deep = 0.0;
      int     seen = 0;
      int32_t wore = 0;
      for (int dr = 0; dr < cell_rows; dr++)
      {
        const int base = (r * cell_rows + dr) * cols + c * supersample;
        for (int dc = 0; dc < supersample; dc++)
        {
          const double d = depth[base + dc];
          if (d != 0.0)
          {
            total += value[base + dc];
            seen++;
          }
          if (d > deep)
          {
            deep = d;
            if (who != NULL)
            {
              wore = who[base + dc];
            }
          }
        }
      }
      const int at = row + c;
      shade[at] = (seen > 0) ? (total / (double)seen) : 0.0;
      near_b[at] = (seen > 0) ? deep : 0.0;
      if (paintbox != NULL)
      {
        paintbox[at] = (seen > 0) ? wore : 0;
      }
      if (seen > 0)
      {
        if (deep < z_lo)
        {
          z_lo = deep;
        }
        if (deep > z_hi)
        {
          z_hi = deep;
        }
      }
      if (has_ink && seen > 0)
      {
        const double lit = invert ? (1.0 - shade[at]) : shade[at];
        band[at] = (int32_t)((1.0 - lit) * (double)steps);
      }
      else
      {
        band[at] = 0;
      }
    }
  }

  for (int r = 0; r < height; r++)
  {
    const int row = r * width;
    for (int c = 0; c < width; c++)
    {
      const int at = row + c;
      if (near_b[at] == 0.0)
      {
        out_code[at] = bg_code;
        out_tone[at] = -1;
      }
      else if (wire)
      {
        rd_ascii3d_wire(near_b, paintbox, has_shades, shades_lut, ink_colour, at, c, r, width,
                        height, z_lo, z_hi, &out_code[at], &out_tone[at]);
      }
      else if (has_ink && rd_ascii3d_inked(near_b, band, at, c, r, width, height))
      {
        out_code[at] = ink_code;
        out_tone[at] = ink_colour;
      }
      else
      {
        const double cl = (shade[at] < 0.0) ? 0.0 : ((shade[at] > 1.0) ? 1.0 : shade[at]);
        int          ridx = rd_clamp_int((1.0 - cl) * (double)steps, 0, steps);
        if (invert)
        {
          ridx = steps - ridx;
        }
        out_code[at] = ramp_codes[ridx];
        int32_t wear = (paintbox != NULL && paintbox[at] != 0) ? (paintbox[at] - 1) : -1;
        if (wear >= 0 && has_shades && wear < 256)
        {
          const int denom = (steps > 1) ? steps : 1;
          int       third = (band[at] * 3) / denom;
          third = (third < 0) ? 0 : ((third > 2) ? 2 : third);
          wear = shades_lut[wear * 3 + third];
        }
        out_tone[at] = wear;
      }
    }
    int end = width;
    while (end > 0 && out_code[row + end - 1] == bg_code && out_tone[row + end - 1] == -1)
    {
      end--;
    }
    out_len[r] = (int32_t)end;
  }
}

/** Shade `win` splat buffer into `out_rgba` (`(height, width, 4)` uint8) matching `lit._light`. */
void rs_lit_shade_win(int npix, const int32_t *restrict win, const double *restrict normals,
                      const uint32_t *restrict materials, const double *restrict uv,
                      const double *restrict m9, const double *restrict key3,
                      const double *restrict fill3, const double *restrict half3,
                      const double *restrict pal12, uint8_t *restrict out_rgba)
{
  int32_t   seen_pt[512];
  uint32_t  seen_rgba[512];
  uint32_t *dst32 = (uint32_t *)out_rgba;
  for (int i = 0; i < 512; i++)
  {
    seen_pt[i] = -1;
    seen_rgba[i] = 0u;
  }

  for (int i = 0; i < npix; i++)
  {
    const int32_t pt = win[i];
    if (pt < 0)
    {
      dst32[i] = 0u;
      continue;
    }
    const int slot = (int)((uint32_t)pt & 511u);
    if (seen_pt[slot] == pt)
    {
      dst32[i] = seen_rgba[slot];
      continue;
    }
    const size_t n_off = (size_t)pt * 3u, u_off = (size_t)pt * 2u;
    const double nx0 = normals[n_off], ny0 = normals[n_off + 1u], nz0 = normals[n_off + 2u];
    double       nx = (nx0 * m9[0] + ny0 * m9[1]) + nz0 * m9[2];
    double       ny = (nx0 * m9[3] + ny0 * m9[4]) + nz0 * m9[5];
    double       nz = (nx0 * m9[6] + ny0 * m9[7]) + nz0 * m9[8];
    double       nlen = sqrt((nx * nx + ny * ny) + nz * nz);
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
    double fdot = (nx * fill3[0] + ny * fill3[1]) + nz * fill3[2];
    fdot = (fdot < 0.0) ? 0.0 : fdot;
    const double   fil = 0.35 * fdot;
    const double   cz = (nz < 0.0) ? 0.0 : ((nz > 1.0) ? 1.0 : nz);
    const double   rim = pow(1.0 - cz, 3.0);
    double         lobe = (nx * half3[0] + ny * half3[1]) + nz * half3[2];
    lobe = (lobe < 0.0) ? 0.0 : lobe;
    const uint32_t mat = materials[pt];
    const int      woven = (mat & (1u << 25)) != 0u;
    const int      twill = (mat & (1u << 27)) != 0u;
    const int      hard = (mat & (1u << 28)) != 0u;
    const double   spec = pow(lobe, twill ? 160.0 : 40.0)
                        + (twill ? ((0.6 / 0.9) * pow(lobe, 28.0)) : 0.0);
    double b0, b1, b2;
    if (mat >= (1u << 24))
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
      const double g0 = fabs(modf(uv[u_off] * 22.0, &ip0) - 0.5);
      const double g1 = fabs(modf(uv[u_off + 1u] * 30.0, &ip1) - 0.5);
      if (((g0 > g1) ? g0 : g1) > 0.4)
      {
        b0 *= 0.5;
        b1 *= 0.5;
        b2 *= 0.5;
      }
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
      const double shine = (mat == 1u) ? 0.12 : (woven ? 0.0 : (twill ? 0.9 : 0.55));
      const double df = 0.10 + 0.85 * lam + fil, sp = spec * shine;
      const double rf = rim * (woven ? 0.15 : (hard ? 0.3 : 0.8));
      c0 = b0 * df + sp + rf * 0.55;
      c1 = b1 * df + sp + rf * 0.75;
      c2 = b2 * df + sp + rf * 1.0;
    }
    c0 = (c0 < 0.0) ? 0.0 : ((c0 > 1.0) ? 1.0 : c0);
    c1 = (c1 < 0.0) ? 0.0 : ((c1 > 1.0) ? 1.0 : c1);
    c2 = (c2 < 0.0) ? 0.0 : ((c2 > 1.0) ? 1.0 : c2);
    const uint8_t r8 = (uint8_t)(c0 * 255.0);
    const uint8_t g8 = (uint8_t)(c1 * 255.0);
    const uint8_t b8 = (uint8_t)(c2 * 255.0);
    const uint8_t a8 = twill ? (uint8_t)(((nx > 0.0) ? 0 : 128) + (hard ? 64 : 0))
                             : (hard ? 224u : 255u);
    uint8_t       bytes4[4] = {r8, g8, b8, a8};
    uint32_t      packed;
    memcpy(&packed, bytes4, 4u);
    seen_pt[slot] = pt;
    seen_rgba[slot] = packed;
    dst32[i] = packed;
  }
}
