/** se3_sums.c - the points' sums about their centroid: two passes of sixteen lanes, no branch */
#include "se3_bits.h"

/** A pass's sums, a row of lanes each: a lane takes every sixteenth point, so a target's vector
    adds them as any other's does and the sums are the same bits on each. */
enum { W, X, Y, Z, SPREAD, SX, SY, SZ, COST, FIRST };
enum { TX = SE3_SECOND, TY, TZ, MOMENTS };
/** What the first pass keeps a point for the second, a plane each. */
enum { MX, MY, MZ, WE, KEPT };

/** The lanes' sum, by halves: each half onto the other, a vector's add at a time. */
static inline se3_real_t tree(const se3_real_t lane[SE3_LANES])
{
  se3_real_t eight[8], four[4];

  for (size_t i = 0U; i < 8U; i++)
  {
    eight[i] = lane[i] + lane[i + 8U];
  }
  for (size_t i = 0U; i < 4U; i++)
  {
    four[i] = eight[i] + eight[i + 4U];
  }
  return (four[0] + four[2]) + (four[1] + four[3]);
}

void se3_summed(se3_real_t *planes, size_t stride, se3_real_t soft, const se3_real_t r[9],
                const se3_real_t t[3], se3_sums_t *sums)
{
  const se3_real_t *px = &planes[SE3_PX * stride], *py = &planes[SE3_PY * stride];
  const se3_real_t *pz = &planes[SE3_PZ * stride], *gx = &planes[SE3_GX * stride];
  const se3_real_t *gy = &planes[SE3_GY * stride], *gz = &planes[SE3_GZ * stride];
  const se3_real_t *pw = &planes[SE3_W * stride];
  se3_real_t       *kept = &planes[SE3_MX * stride];
  const se3_real_t r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3], r4 = r[4], r5 = r[5];
  const se3_real_t r6 = r[6], r7 = r[7], r8 = r[8], t0 = t[0], t1 = t[1], t2 = t[2];
  se3_real_t       first[FIRST][SE3_LANES] = {{ZERO}};
  se3_real_t       moment[MOMENTS][SE3_LANES] = {{ZERO}};
  se3_real_t       lane[KEPT][SE3_LANES];
  se3_real_t       weight, inverse, cx, cy, cz;

  /* the weights as the misses leave them, the centroid, and what no centroid enters; a point
     past the last weighs zero. Each point's miss and weight go to their planes a block at a
     time: the second pass makes neither again, and divides nothing */
  for (size_t base = 0U; base < stride; base += SE3_LANES)
  {
    for (size_t k = 0U; k < SE3_LANES; k++)
    {
      const size_t     i = base + k;
      const se3_real_t x = px[i], y = py[i], z = pz[i];
      const se3_real_t mx = ((((r0 * x) + (r1 * y)) + (r2 * z)) + t0) - gx[i];
      const se3_real_t my = ((((r3 * x) + (r4 * y)) + (r5 * z)) + t1) - gy[i];
      const se3_real_t mz = ((((r6 * x) + (r7 * y)) + (r8 * z)) + t2) - gz[i];
      const se3_real_t m2 = ((mx * mx) + (my * my)) + (mz * mz);
      const se3_real_t w = over(pw[i]) / (ONE + (m2 * soft));

      first[W][k] += w;
      first[X][k] += w * x;
      first[Y][k] += w * y;
      first[Z][k] += w * z;
      first[SPREAD][k] += w * (((x * x) + (y * y)) + (z * z));
      first[SX][k] += w * mx;
      first[SY][k] += w * my;
      first[SZ][k] += w * mz;
      first[COST][k] += w * m2;
      lane[MX][k] = mx;
      lane[MY][k] = my;
      lane[MZ][k] = mz;
      lane[WE][k] = w;
    }
    for (size_t n = 0U; n < (size_t)KEPT; n++)
    {
      memcpy(&kept[(n * stride) + base], lane[n], sizeof(lane[n]));
    }
  }
  weight = tree(first[W]);
  inverse = weight / ((weight * weight) + SE3_TINY);
  cx = tree(first[X]) * inverse;
  cy = tree(first[Y]) * inverse;
  cz = tree(first[Z]) * inverse;

  /* the moments about it: each lever the turn of a point less the centroid, never a place in
     the world */
  for (size_t base = 0U; base < stride; base += SE3_LANES)
  {
    for (size_t k = 0U; k < SE3_LANES; k++)
    {
      const size_t     i = base + k;
      const se3_real_t mx = kept[((size_t)MX * stride) + i], my = kept[((size_t)MY * stride) + i];
      const se3_real_t mz = kept[((size_t)MZ * stride) + i], w = kept[((size_t)WE * stride) + i];
      const se3_real_t lx = px[i] - cx, ly = py[i] - cy, lz = pz[i] - cz;
      const se3_real_t dx = ((r0 * lx) + (r1 * ly)) + (r2 * lz);
      const se3_real_t dy = ((r3 * lx) + (r4 * ly)) + (r5 * lz);
      const se3_real_t dz = ((r6 * lx) + (r7 * ly)) + (r8 * lz);
      const se3_real_t wx = w * dx, wy = w * dy, wz = w * dz;

      moment[SE3_XX][k] += wx * dx;
      moment[SE3_XY][k] += wx * dy;
      moment[SE3_XZ][k] += wx * dz;
      moment[SE3_YY][k] += wy * dy;
      moment[SE3_YZ][k] += wy * dz;
      moment[SE3_ZZ][k] += wz * dz;
      moment[TX][k] += (wy * mz) - (wz * my);
      moment[TY][k] += (wz * mx) - (wx * mz);
      moment[TZ][k] += (wx * my) - (wy * mx);
    }
  }
  sums->weight = weight;
  sums->spread = tree(first[SPREAD]);
  sums->centre[0] = cx;
  sums->centre[1] = cy;
  sums->centre[2] = cz;
  for (size_t n = 0U; n < (size_t)SE3_SECOND; n++)
  {
    sums->second[n] = tree(moment[n]);
  }
  for (size_t n = 0U; n < 3U; n++)
  {
    sums->turn[n] = tree(moment[(size_t)TX + n]);
    sums->shift[n] = tree(first[(size_t)SX + n]);
  }
  sums->cost = HALF * tree(first[COST]);
}
