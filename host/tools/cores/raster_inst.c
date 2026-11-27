/* SIMD C11 2D instrument kernels for motor cross_section, shaft-angle dial, and thermalmap. */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const uint8_t RI_BBITS[2][4] = {{0x01u, 0x02u, 0x04u, 0x40u},
                                       {0x08u, 0x10u, 0x20u, 0x80u}};

static const double RI_SUBDOT_X[4] = {-0.375, 0.125, -0.125, 0.375};
static const double RI_SUBDOT_Y[4] = {-0.125, -0.375, 0.375, 0.125};

static const int RI_BAYER[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};

static inline double ri_mod_pos(double x, double y)
{
  double r = fmod(x, y);
  if (r == 0.0)
  {
    return 0.0;
  }
  if (r < 0.0)
  {
    r += y;
  }
  return r;
}

static inline double ri_ring(double radius, double at, double line, double weight)
{
  const double edge = line * weight + 0.5 - fabs(radius - at);
  return (edge <= 0.0) ? 0.0 : ((edge >= 1.0) ? 1.0 : edge);
}

/** Build `cross_section._seat_arrayed` sample tables in C, returning `n_dots`. */
int rs_csec_seat(int width, int height, const double *restrict g11, int64_t *restrict out_kind,
                 double *restrict out_a, double *restrict out_b, int32_t *restrict out_x,
                 int32_t *restrict out_y)
{
  const double cx = g11[0], cy = g11[1], stretch = g11[2], can = g11[3];
  const double mag_out = g11[4], mag_in = g11[5], tooth_out = g11[6], tooth_in = g11[7];
  const double bore = g11[8], can_in = g11[9], line = g11[10];
  const int    dot_h = height * 4, dot_w = width * 2;
  int          m = 0;

  for (int y = 0; y < dot_h; y++)
  {
    for (int x = 0; x < dot_w; x++)
    {
      int64_t k4[4] = {-1, -1, -1, -1};
      double  a4[4] = {0.0, 0.0, 0.0, 0.0};
      double  b4[4] = {0.0, 0.0, 0.0, 0.0};
      int     cnt = 0;
      for (int s = 0; s < 4; s++)
      {
        const double dx = (double)x + RI_SUBDOT_X[s] - cx;
        const double dy = (cy - (double)y - RI_SUBDOT_Y[s]) * stretch;
        const double radius = hypot(dx, dy);
        const double phi = atan2(dy, dx);
        double       cov = ri_ring(radius, bore, line, 1.0);
        if (cov != 0.0)
        {
          k4[cnt] = 0;
          a4[cnt] = 1.0;
          b4[cnt] = cov;
          cnt++;
          continue;
        }
        cov = ri_ring(radius, tooth_in, line, 1.0);
        if (cov != 0.0)
        {
          k4[cnt] = 0;
          a4[cnt] = 2.0;
          b4[cnt] = cov;
          cnt++;
          continue;
        }
        if (radius <= tooth_out)
        {
          if (tooth_in <= radius)
          {
            k4[cnt] = 1;
            a4[cnt] = radius;
            b4[cnt] = phi;
            cnt++;
          }
          continue;
        }
        if (mag_in <= radius && radius <= mag_out)
        {
          k4[cnt] = 2;
          a4[cnt] = radius;
          b4[cnt] = phi;
          cnt++;
          continue;
        }
        const double c1 = ri_ring(radius, can, line, 1.0);
        const double c2 = ri_ring(radius, can_in, line, 1.0);
        cov = (c1 > c2) ? c1 : c2;
        if (cov != 0.0)
        {
          k4[cnt] = 0;
          a4[cnt] = 8.0;
          b4[cnt] = cov;
          cnt++;
        }
      }
      if (cnt > 0)
      {
        const size_t off = (size_t)m * 4u;
        for (int j = 0; j < 4; j++)
        {
          out_kind[off + (size_t)j] = k4[j];
          out_a[off + (size_t)j] = a4[j];
          out_b[off + (size_t)j] = b4[j];
        }
        out_x[m] = (int32_t)x;
        out_y[m] = (int32_t)y;
        m++;
      }
    }
  }
  return m;
}

static inline double ri_exposed(double place, double blur, int pole)
{
  const double width = 0.9 - 0.1;
  if (fabs(blur) < 1e-9)
  {
    const double x = place - (double)pole;
    const double at = x - 2.0 * floor(x / 2.0);
    return (0.1 <= at && at < 0.9) ? 1.0 : 0.0;
  }
  const double x1 = (place + blur) - (double)pole, k1 = floor(x1 / 2.0);
  double       c1 = x1 - 2.0 * k1 - 0.1;
  c1 = (c1 < 0.0) ? 0.0 : ((c1 > width) ? width : c1);
  const double x0 = place - (double)pole, k0 = floor(x0 / 2.0);
  double       c0 = x0 - 2.0 * k0 - 0.1;
  c0 = (c0 < 0.0) ? 0.0 : ((c0 > width) ? width : c0);
  return ((k1 * width + c1) - (k0 * width + c0)) / blur;
}

/** Run `cross_section._body` vote reduction, hollow north outline, and cell tally in C. */
void rs_csec_body(int n_dots, const int64_t *restrict kind, const double *restrict a,
                  const double *restrict b, const int32_t *restrict xs, const int32_t *restrict ys,
                  int width, int height, const double *restrict pars8, int slots, int poles,
                  int coast, uint8_t *restrict north_grid, int32_t *restrict north_idx,
                  int32_t *restrict track_idx, int32_t *restrict tally9,
                  int32_t *restrict out_dots, int32_t *restrict out_owner)
{
  const double tooth_in = pars8[0], tooth_out = pars8[1], mag_in = pars8[2], mag_out = pars8[3];
  const double line = pars8[4], rotor = pars8[5], blur = pars8[6], tau = pars8[7];
  const double slot_step = tau / (double)slots, pole_step = tau / (double)poles;
  const double mag_mid = (mag_in + mag_out) / 2.0;
  const int    dot_w = width * 2, dot_h = height * 4, ncells = width * height;
  int          n_north = 0, n_track = 0;

  memset(north_grid, 0, (size_t)dot_w * (size_t)dot_h);
  memset(tally9, 0, (size_t)ncells * 9u * sizeof(int32_t));
  memset(out_dots, 0, (size_t)ncells * sizeof(int32_t));
  for (int i = 0; i < ncells; i++)
  {
    out_owner[i] = -1;
  }

  for (int i = 0; i < n_dots; i++)
  {
    const size_t base = (size_t)i * 4u;
    const int    x = (int)xs[i], y = (int)ys[i];
    const double thresh = ((double)RI_BAYER[(y & 3) * 4 + (x & 3)] + 0.5) / 16.0;
    int          cls4[4];
    double       sh4[4];
    for (int j = 0; j < 4; j++)
    {
      const int64_t kj = kind[base + (size_t)j];
      const double  aj = a[base + (size_t)j], bj = b[base + (size_t)j];
      if (kj == 0)
      {
        cls4[j] = (int)aj;
        sh4[j] = bj;
      }
      else if (kj == 1)
      {
        const double place = ri_mod_pos(bj, tau) / slot_step;
        const double whole = floor(place);
        const int    inside = (!coast) && (tooth_in <= aj) && (aj <= tooth_out)
                           && !((place - whole) > 0.5);
        cls4[j] = inside ? (3 + (((int)whole) % 3)) : -1;
        sh4[j] = inside ? 1.0 : 0.0;
      }
      else if (kj == 2)
      {
        const double place = ri_mod_pos(bj - rotor, tau) / pole_step;
        const int    is_n = ri_exposed(place, blur, 0) > thresh;
        double       cov = line + 0.5 - fabs(aj - mag_mid);
        cov = (cov < 0.0) ? 0.0 : ((cov > 1.0) ? 1.0 : cov);
        const int is_s = (!is_n) && (cov != 0.0) && (ri_exposed(place, blur, 1) > thresh);
        cls4[j] = is_n ? 7 : (is_s ? 6 : -1);
        sh4[j] = is_n ? 1.0 : (is_s ? cov : 0.0);
      }
      else
      {
        cls4[j] = -1;
        sh4[j] = 0.0;
      }
    }
    double total = 0.0, best = -1e300;
    int    owner = -1;
    for (int j = 0; j < 4; j++)
    {
      const int cj = cls4[j];
      if (cj < 0)
      {
        continue;
      }
      int    first = 1;
      double mine = sh4[j];
      for (int k = 0; k < 4; k++)
      {
        if (k < j && cls4[k] == cj)
        {
          first = 0;
        }
        else if (k > j && cls4[k] == cj)
        {
          mine += sh4[k];
        }
      }
      if (first)
      {
        total += mine;
        if (mine > best || (mine == best && cj > owner))
        {
          best = mine;
          owner = cj;
        }
      }
    }
    if (owner < 0 || total * 2.0 < 4.0)
    {
      continue;
    }
    if (owner == 7)
    {
      north_grid[y * dot_w + x] = 1u;
      north_idx[n_north++] = i;
    }
    else if (owner == 0)
    {
      track_idx[n_track++] = i;
    }
    else
    {
      const int col = x / 2, row = y / 4;
      if (row >= 0 && row < height && col >= 0 && col < width)
      {
        const int cell = row * width + col;
        out_dots[cell] |= (int32_t)RI_BBITS[x & 1][y & 3];
        tally9[cell * 9 + owner] += 1;
      }
    }
  }
  for (int k = 0; k < n_north; k++)
  {
    const int i = north_idx[k];
    const int x = (int)xs[i], y = (int)ys[i];
    if (x == 0 || x + 1 >= dot_w || y == 0 || y + 1 >= dot_h
        || !north_grid[y * dot_w + x + 1] || !north_grid[y * dot_w + x - 1]
        || !north_grid[(y + 1) * dot_w + x] || !north_grid[(y - 1) * dot_w + x])
    {
      const int col = x / 2, row = y / 4;
      if (row >= 0 && row < height && col >= 0 && col < width)
      {
        const int cell = row * width + col;
        out_dots[cell] |= (int32_t)RI_BBITS[x & 1][y & 3];
        tally9[cell * 9 + 7] += 1;
      }
    }
  }
  for (int k = 0; k < n_track; k++)
  {
    const int i = track_idx[k];
    const int x = (int)xs[i], y = (int)ys[i];
    const int col = x / 2, row = y / 4;
    if (row >= 0 && row < height && col >= 0 && col < width)
    {
      const int      cell = row * width + col;
      const int32_t *t9 = tally9 + cell * 9;
      if ((t9[3] | t9[4] | t9[5] | t9[6] | t9[7]) == 0)
      {
        out_dots[cell] |= (int32_t)RI_BBITS[x & 1][y & 3];
        tally9[cell * 9 + 0] += 1;
      }
    }
  }
  for (int cell = 0; cell < ncells; cell++)
  {
    if (out_dots[cell] == 0)
    {
      continue;
    }
    const int32_t *t9 = tally9 + cell * 9;
    int            best_line = -1, best_cnt = -1, best_c = -1;
    for (int c = 0; c < 9; c++)
    {
      if (t9[c] <= 0)
      {
        continue;
      }
      const int is_line = (c == 1 || c == 2 || c == 8) ? 1 : 0;
      if (is_line > best_line || (is_line == best_line && (t9[c] > best_cnt
                                                           || (t9[c] == best_cnt && c > best_c))))
      {
        best_line = is_line;
        best_cnt = (int)t9[c];
        best_c = c;
      }
    }
    out_owner[cell] = (int32_t)best_c;
  }
}

static int ri_dial_sub_fixed(double sx, double sy, double r, const double *notch2, double rad2deg,
                             double deg2rad, int which)
{
  const double sr = hypot(sx, sy);
  if (sr > r + 2.5)
  {
    return -10;
  }
  const double turned = ri_mod_pos(225.0 - atan2(sy, sx) * rad2deg, 360.0);
  if (sr < r - 3.0 || turned > 270.0)
  {
    return -2;
  }
  if (sr > r)
  {
    for (int b = 0; b < 2; b++)
    {
      const double off = fabs(turned - notch2[b]) * deg2rad;
      if (off * sr <= 0.6 && sr >= r + 1.0)
      {
        return 7;
      }
    }
    return -2;
  }
  int k = (int)(turned / 11.25);
  if (k > 23)
  {
    k = 23;
  }
  return (turned - (double)k * 11.25 <= 0.68 * 11.25) ? (100 + which * 24 + k) : -2;
}

/** Build `dial._arrayed` sample and dot arrays in C, returning `[n_dots, n_samples]`. */
void rs_dial_arrays(int width, int height, const double *restrict pars12,
                    int64_t *restrict out_dot, double *restrict out_dx, double *restrict out_dy,
                    double *restrict out_phi, int64_t *restrict out_base,
                    uint8_t *restrict out_needle, uint8_t *restrict out_sweep,
                    int64_t *restrict out_seg, int64_t *restrict out_cell,
                    int64_t *restrict out_bit, int64_t *restrict out_counts)
{
  const double cx = pars12[0], cy = pars12[1], rim = pars12[2], line = pars12[3];
  const double stretch = pars12[4], tau = pars12[5], rad2deg = pars12[6], deg2rad = pars12[7];
  const double p30 = pars12[8], p6 = pars12[9], p3 = pars12[10];
  const double notches[2][2] = {{270.0 * 0.25, 270.0 * (1000.0 / 1200.0)},
                                {270.0 * (55.0 / 190.0), 270.0 * (105.0 / 190.0)}};
  const int    has_subs = rim >= 30.0, has_micro = rim >= 60.0;
  const double sub_r = rim * 0.27, sub_off = rim * 0.44;
  const int    dot_h = height * 4, dot_w = width * 2;
  int64_t      nd = 0, ns = 0;

  for (int y = 0; y < dot_h; y++)
  {
    for (int x = 0; x < dot_w; x++)
    {
      int dot_has = 0;
      for (int s = 0; s < 4; s++)
      {
        const double dx = (double)x + RI_SUBDOT_X[s] - cx;
        const double dy = (cy - (double)y - RI_SUBDOT_Y[s]) * stretch;
        const double radius = hypot(dx, dy);
        if (radius > rim + 3.0 + line + 1.0)
        {
          continue;
        }
        dot_has = 1;
        const double phi = ri_mod_pos(atan2(dy, dx), tau);
        int          fcode = -10;
        if (radius <= 2.6)
        {
          fcode = (radius >= 2.6 - 0.7) ? 21 : -10;
        }
        else
        {
          const double t_p[3] = {p30, p6, p3};
          const double t_d[3] = {6.0, 2.5, 1.2};
          const double t_w[3] = {1.2, 0.0, 0.0};
          const int    t_c[3] = {5, 4, 3};
          const int    nt = has_micro ? 3 : 2;
          for (int ti = 0; ti < nt; ti++)
          {
            const double pitch = t_p[ti];
            const double off = fabs(ri_mod_pos(phi + pitch / 2.0, pitch) - pitch / 2.0);
            const double hw = ((t_w[ti] > line) ? t_w[ti] : line) / 2.0;
            if (off * radius <= hw && (rim - t_d[ti]) <= radius && radius <= rim)
            {
              fcode = t_c[ti];
              break;
            }
          }
          if (fcode == -10)
          {
            if (fabs(radius - rim) <= line)
            {
              fcode = 2;
            }
            else if (fabs(radius - rim - 3.0) <= line * 0.6)
            {
              fcode = 1;
            }
            else if (rim - 11.0 <= radius && radius <= rim - 9.0)
            {
              fcode = -3;
            }
            else
            {
              const double dxp = radius * cos(phi), dyp = radius * sin(phi);
              if (has_subs)
              {
                for (int w = 0; w < 2; w++)
                {
                  const double oy = (w == 0) ? sub_off : -sub_off;
                  const int    sc = ri_dial_sub_fixed(dxp, dyp - oy, sub_r, notches[w], rad2deg,
                                                      deg2rad, w);
                  if (sc != -10)
                  {
                    fcode = sc;
                    break;
                  }
                }
              }
              if (fcode == -10 && (2.6 + 2.0) < radius && radius < (rim - 11.0 - 2.0)
                  && (((int)radius) % 3 == 0))
              {
                if (fabs(dyp) <= 0.5 || (!has_subs && fabs(dxp) <= 0.5))
                {
                  fcode = 0;
                }
              }
            }
          }
        }
        const int64_t seg = (fcode >= 100) ? (int64_t)(fcode - 100) : -1;
        const int     plain = !(seg >= 0 || fcode == -2 || fcode == -3 || fcode == -10);
        out_dot[ns] = nd;
        out_dx[ns] = dx;
        out_dy[ns] = dy;
        out_phi[ns] = phi;
        out_seg[ns] = seg;
        out_base[ns] = plain ? (int64_t)fcode : -1;
        out_needle[ns] = (uint8_t)(seg < 0 && fcode != -2 && fcode != 21 && fcode != 7);
        out_sweep[ns] = (uint8_t)(fcode == -3);
        ns++;
      }
      if (dot_has)
      {
        const int col = x / 2, row = y / 4;
        out_cell[nd] = (row >= 0 && row < height && col >= 0 && col < width)
                         ? (int64_t)(row * width + col)
                         : -1;
        out_bit[nd] = (int64_t)RI_BBITS[x & 1][y & 3];
        nd++;
      }
    }
  }
  out_counts[0] = nd;
  out_counts[1] = ns;
}

/** Classify `dial` samples and scatter covered dots into `(out_bits, out_owners)` in C. */
void rs_dial_frame(int n_samples, int n_dots, int n_cells, const int64_t *restrict dot,
                   const double *restrict dx, const double *restrict dy,
                   const double *restrict phi, const int64_t *restrict base,
                   const uint8_t *restrict needle_ok, const uint8_t *restrict sweep_ok,
                   const int64_t *restrict seg, const int64_t *restrict cell,
                   const int64_t *restrict bit, int has_needle, double nc, double ns_v,
                   double needle_len, int has_span, double span, double sweep_fade, double tau,
                   const double *restrict glow, const int64_t *restrict seg_lut,
                   int32_t *restrict dot_cnt, int64_t *restrict dot_top,
                   int64_t *restrict out_bits, int64_t *restrict out_owners)
{
  memset(dot_cnt, 0, (size_t)n_dots * sizeof(int32_t));
  memset(out_bits, 0, (size_t)n_cells * sizeof(int64_t));
  for (int i = 0; i < n_dots; i++)
  {
    dot_top[i] = -1;
  }
  for (int i = 0; i < n_cells; i++)
  {
    out_owners[i] = -1;
  }
  const double inv_len = 1.0 / ((needle_len > 1e-6) ? needle_len : 1e-6);

  for (int i = 0; i < n_samples; i++)
  {
    int64_t c = base[i];
    int     hit = 0;
    if (has_needle && needle_ok[i])
    {
      const double along = dx[i] * nc + dy[i] * ns_v;
      const double across = fabs(-dx[i] * ns_v + dy[i] * nc);
      if (0.0 <= along && along <= needle_len
          && across <= 0.75 + (0.35 - 0.75) * (along * inv_len))
      {
        hit = 1;
        c = 16;
      }
    }
    if (has_span && sweep_ok[i] && !hit)
    {
      const double p = phi[i];
      if (glow == NULL)
      {
        if (0.0 < p && p <= span && (span - p) < sweep_fade)
        {
          const double bright = 1.0 - (span - p) / sweep_fade;
          c = 8 + (int64_t)(bright * 7.0 + 0.5);
        }
      }
      else
      {
        const int    bin = ((int)(p / tau * 720.0)) % 720;
        const double g = glow[bin];
        if (g > 0.0)
        {
          c = 8 + (int64_t)(g * 7.0 + 0.5);
        }
      }
    }
    if (seg[i] >= 0 && seg_lut != NULL)
    {
      c = seg_lut[seg[i]];
    }
    if (c >= 0)
    {
      const int d = (int)dot[i];
      dot_cnt[d] += 1;
      if (c > dot_top[d])
      {
        dot_top[d] = c;
      }
    }
  }
  for (int d = 0; d < n_dots; d++)
  {
    const int64_t cl = cell[d];
    if (dot_cnt[d] >= 2 && cl >= 0)
    {
      out_bits[cl] |= bit[d];
      if (dot_top[d] > out_owners[cl])
      {
        out_owners[cl] = dot_top[d];
      }
    }
  }
}

static int32_t ri_thermal_rgb(double celsius, const double *stops_c, const int32_t *stops_rgb,
                              int n_stops)
{
  if (celsius <= stops_c[0])
  {
    return (stops_rgb[0] << 16) | (stops_rgb[1] << 8) | stops_rgb[2];
  }
  for (int i = 0; i < n_stops - 1; i++)
  {
    if (celsius <= stops_c[i + 1])
    {
      const double  f = (celsius - stops_c[i]) / (stops_c[i + 1] - stops_c[i]);
      const int32_t r = (int32_t)((double)stops_rgb[i * 3] + (double)(stops_rgb[(i + 1) * 3] - stops_rgb[i * 3]) * f + 0.5);
      const int32_t g = (int32_t)((double)stops_rgb[i * 3 + 1] + (double)(stops_rgb[(i + 1) * 3 + 1] - stops_rgb[i * 3 + 1]) * f + 0.5);
      const int32_t b = (int32_t)((double)stops_rgb[i * 3 + 2] + (double)(stops_rgb[(i + 1) * 3 + 2] - stops_rgb[i * 3 + 2]) * f + 0.5);
      return (r << 16) | (g << 8) | b;
    }
  }
  const int last = (n_stops - 1) * 3;
  return (stops_rgb[last] << 16) | (stops_rgb[last + 1] << 8) | stops_rgb[last + 2];
}

/** Rasterise `thermalmap._braille_rows` halftone and ramp colours in C. */
void rs_thermal_braille(int down, int cells, const double *restrict temps,
                        const uint8_t *restrict on, const uint8_t *restrict mask,
                        const uint16_t *restrict noise, const double *restrict stops_c,
                        const int32_t *restrict stops_rgb, int n_stops,
                        int32_t *restrict out_codes, int32_t *restrict out_rgb)
{
  const int     rows = down / 2, wide = 2 * cells;
  const double  t_min = stops_c[0], inv_span = 1.0 / (stops_c[n_stops - 1] - t_min);
  const int32_t mark_rgb = (60 << 16) | (175 << 8) | 185;

  for (int r = 0; r < rows; r++)
  {
    const int r0 = 2 * r, r1 = 2 * r + 1;
    for (int c = 0; c < cells; c++)
    {
      const uint8_t o0 = on[r0 * cells + c];
      const uint8_t o1 = (r1 < down) ? on[r1 * cells + c] : 0u;
      double        share0 = -1.0, share1 = -1.0;
      int           mid = 0;
      if (o0 || o1)
      {
        const double t0 = o0 ? temps[r0 * cells + c] : temps[r1 * cells + c];
        const double t1 = o1 ? temps[r1 * cells + c] : t0;
        double       f0 = (t0 - t_min) * inv_span;
        double       f1 = (t1 - t_min) * inv_span;
        f0 = (f0 < 0.0) ? 0.0 : ((f0 > 1.0) ? 1.0 : f0);
        f1 = (f1 < 0.0) ? 0.0 : ((f1 > 1.0) ? 1.0 : f1);
        share0 = (0.40 + 0.60 * f0) * 4096.0;
        share1 = (0.40 + 0.60 * f1) * 4096.0;
        mid = (int)((t0 + t1) / 2.0 + 0.5);
      }
      int marks_b = 0, dots_b = 0;
      for (int lane = 0; lane < 2; lane++)
      {
        const int i = 2 * c + lane;
        const int ni = i & 63;
        for (int y = 0; y < 4; y++)
        {
          const uint8_t mv = mask[(4 * r + y) * wide + i];
          if (mv == 0u)
          {
            continue;
          }
          if (mv == 2u)
          {
            marks_b |= (int)RI_BBITS[lane][y];
          }
          else if (((y < 2) ? share0 : share1) > (double)noise[((4 * r + y) & 63) * 64 + ni] + 0.5)
          {
            dots_b |= (int)RI_BBITS[lane][y];
          }
        }
      }
      const int at = r * cells + c;
      if (marks_b != 0)
      {
        out_codes[at] = 0x2800 + marks_b;
        out_rgb[at] = mark_rgb;
      }
      else if (dots_b != 0)
      {
        out_codes[at] = 0x2800 + dots_b;
        out_rgb[at] = ri_thermal_rgb((double)mid, stops_c, stops_rgb, n_stops);
      }
      else
      {
        out_codes[at] = 0x20;
        out_rgb[at] = -1;
      }
    }
  }
}
