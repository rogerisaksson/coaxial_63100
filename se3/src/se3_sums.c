/** se3_sums.c - the points' sums about their centroid, fitted or under friction: no branch */
#include "se3_bits.h"

/** A pass's sums, a row of lanes each: a lane takes every sixteenth point, so a target's vector
    adds them as any other's does and the sums are the same bits on each. */
enum { W, X, Y, Z, SPREAD, SX, SY, SZ, COST, FIRST };
enum { TX = SE3_SECOND, TY, TZ, MOMENTS };
/** What the first pass keeps a point for the second, a plane each. */
enum { MX, MY, MZ, WE, KEPT };
/** And under friction the block's anchors beside them, a lane each. */
enum { AX = KEPT, AY, AZ, LAID };

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

/** The passes' lanes summed into `sums`, about the centroid (cx, cy, cz) of the weights' sum. */
static void told(se3_real_t first[FIRST][SE3_LANES], se3_real_t moment[MOMENTS][SE3_LANES],
                 se3_real_t weight, se3_real_t cx, se3_real_t cy, se3_real_t cz, se3_sums_t *sums)
{
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
  told(first, moment, weight, cx, cy, cz, sums);
}

/** The friction's first pass over the planes, its sums into `first` - summed in lanes of its own,
    which no plane can be. Each point's bristle: its stiffness as its miss and speed leave it,
    its pull that on the miss and the drag on the speed; a point with no load weighs nothing and
    pulls nothing. The pull and the stiffness go to their planes a block at a time, as the fit's
    miss and weight, and with them the block's anchors. `taken` all ones, an anchor goes to
    q - share (q - g): the share one, the anchor's own bits; under one, the anchor the reach
    behind its point; none, the point's place. `taken` none, to its own. */
static inline void bristled(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                            const se3_real_t r[9], const se3_real_t t[3], se3_word_t taken,
                            se3_real_t first[FIRST][SE3_LANES])
{
  const se3_real_t *px = &planes[SE3_PX * stride], *py = &planes[SE3_PY * stride];
  const se3_real_t *pz = &planes[SE3_PZ * stride], *gx = &planes[SE3_GX * stride];
  const se3_real_t *gy = &planes[SE3_GY * stride], *gz = &planes[SE3_GZ * stride];
  const se3_real_t *pw = &planes[SE3_W * stride];
  se3_real_t       *kept = &planes[SE3_MX * stride], *anchors = &planes[SE3_GX * stride];
  const se3_real_t r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3], r4 = r[4], r5 = r[5];
  const se3_real_t r6 = r[6], r7 = r[7], r8 = r[8], t0 = t[0], t1 = t[1], t2 = t[2];
  const se3_real_t s0 = friction->spun[0], s1 = friction->spun[1], s2 = friction->spun[2];
  const se3_real_t s3 = friction->spun[3], s4 = friction->spun[4], s5 = friction->spun[5];
  const se3_real_t s6 = friction->spun[6], s7 = friction->spun[7], s8 = friction->spun[8];
  const se3_real_t v0 = friction->sped[0], v1 = friction->sped[1], v2 = friction->sped[2];
  const se3_real_t firm = over(friction->stiff) + SE3_TINY, yield = ONE / firm;
  const se3_real_t drag = friction->drag, slide = friction->slide;
  const se3_real_t fall = friction->rest - slide, fade = friction->fade;
  se3_real_t       sum[FIRST][SE3_LANES] = {{ZERO}};
  se3_real_t       lane[LAID][SE3_LANES];

  for (size_t base = 0U; base < stride; base += SE3_LANES)
  {
    for (size_t k = 0U; k < SE3_LANES; k++)
    {
      const size_t     i = base + k;
      const se3_real_t x = px[i], y = py[i], z = pz[i], load = over(pw[i]);
      const se3_real_t qx = (((r0 * x) + (r1 * y)) + (r2 * z)) + t0;
      const se3_real_t qy = (((r3 * x) + (r4 * y)) + (r5 * z)) + t1;
      const se3_real_t qz = (((r6 * x) + (r7 * y)) + (r8 * z)) + t2;
      const se3_real_t mx = qx - gx[i], my = qy - gy[i], mz = qz - gz[i];
      const se3_real_t ux = (((s0 * x) + (s1 * y)) + (s2 * z)) + v0;
      const se3_real_t uy = (((s3 * x) + (s4 * y)) + (s5 * z)) + v1;
      const se3_real_t uz = (((s6 * x) + (s7 * y)) + (s8 * z)) + v2;
      const se3_real_t m2 = ((mx * mx) + (my * my)) + (mz * mz);
      const se3_real_t u2 = ((ux * ux) + (uy * uy)) + (uz * uz);
      const se3_real_t held = share(load, u2, m2, slide, fall, fade, yield);
      const se3_real_t spring = firm * held;
      const se3_real_t damp = pick(under(ZERO - load), ZERO, drag);
      const se3_real_t w = spring + damp;
      const se3_real_t fx = (spring * mx) + (damp * ux);
      const se3_real_t fy = (spring * my) + (damp * uy);
      const se3_real_t fz = (spring * mz) + (damp * uz);

      sum[W][k] += w;
      sum[X][k] += w * x;
      sum[Y][k] += w * y;
      sum[Z][k] += w * z;
      sum[SPREAD][k] += w * (((x * x) + (y * y)) + (z * z));
      sum[SX][k] += fx;
      sum[SY][k] += fy;
      sum[SZ][k] += fz;
      sum[COST][k] += spring * m2;
      lane[MX][k] = fx;
      lane[MY][k] = fy;
      lane[MZ][k] = fz;
      lane[WE][k] = w;
      lane[AX][k] = pick(taken, gx[i], qx - (held * mx));
      lane[AY][k] = pick(taken, gy[i], qy - (held * my));
      lane[AZ][k] = pick(taken, gz[i], qz - (held * mz));
    }
    for (size_t n = 0U; n < (size_t)KEPT; n++)
    {
      memcpy(&kept[(n * stride) + base], lane[n], sizeof(lane[n]));
    }
    for (size_t n = 0U; n < 3U; n++)
    {
      memcpy(&anchors[(n * stride) + base], lane[(size_t)AX + n], sizeof(lane[n]));
    }
  }
  memcpy(first, sum, sizeof(sum));
}

void se3_rubbed(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                const se3_real_t r[9], const se3_real_t t[3], unsigned take, se3_sums_t *sums)
{
  const se3_real_t *px = &planes[SE3_PX * stride], *py = &planes[SE3_PY * stride];
  const se3_real_t *pz = &planes[SE3_PZ * stride], *kept = &planes[SE3_MX * stride];
  const se3_real_t r0 = r[0], r1 = r[1], r2 = r[2], r3 = r[3], r4 = r[4], r5 = r[5];
  const se3_real_t r6 = r[6], r7 = r[7], r8 = r[8];
  se3_real_t       first[FIRST][SE3_LANES];
  se3_real_t       moment[MOMENTS][SE3_LANES] = {{ZERO}};
  se3_real_t       weight, inverse, cx, cy, cz;

  bristled(planes, stride, friction, r, t, (se3_word_t)0 - (se3_word_t)(take & 1U), first);
  weight = tree(first[W]);
  inverse = weight / ((weight * weight) + SE3_TINY);
  cx = tree(first[X]) * inverse;
  cy = tree(first[Y]) * inverse;
  cz = tree(first[Z]) * inverse;

  /* the moments about the centroid of the stiffnesses, the torque each lever's on its pull */
  for (size_t base = 0U; base < stride; base += SE3_LANES)
  {
    for (size_t k = 0U; k < SE3_LANES; k++)
    {
      const size_t     i = base + k;
      const se3_real_t fx = kept[((size_t)MX * stride) + i], fy = kept[((size_t)MY * stride) + i];
      const se3_real_t fz = kept[((size_t)MZ * stride) + i], w = kept[((size_t)WE * stride) + i];
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
      moment[TX][k] += (dy * fz) - (dz * fy);
      moment[TY][k] += (dz * fx) - (dx * fz);
      moment[TZ][k] += (dx * fy) - (dy * fx);
    }
  }
  told(first, moment, weight, cx, cy, cz, sums);
}

void se3_anchored(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                  const se3_real_t r[9], const se3_real_t t[3])
{
  se3_real_t first[FIRST][SE3_LANES];

  bristled(planes, stride, friction, r, t, (se3_word_t)0 - (se3_word_t)1U, first);
}
