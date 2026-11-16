/** se3_step.c - the step off the sums: the turn's 3 by 3, its caps, the pose taken on SO(3) */
#include "se3_bits.h"

/** The least a floor is: its square a double's, over its least. */
#define LEAST 1e-150
/** The series' terms, in the turn's square: sin a / a and (1 - cos a) / a^2 by their terms'
    ratios, each laid as its reciprocal - a product a term, no division. */
#define TERMS 7U

static const se3_real_t SINE[TERMS] = {ONE / 6.0f,   ONE / 20.0f,  ONE / 42.0f, ONE / 72.0f,
                                       ONE / 110.0f, ONE / 156.0f, ONE / 210.0f};
static const se3_real_t COSINE[TERMS] = {ONE / 12.0f,  ONE / 30.0f,  ONE / 56.0f, ONE / 90.0f,
                                         ONE / 132.0f, ONE / 182.0f, ONE / 240.0f};
static const se3_real_t EYE[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

unsigned se3_size(unsigned which)
{
  static const unsigned SIZES[8] = {(unsigned)sizeof(se3_real_t), (unsigned)sizeof(se3_cfg_t),
                                    (unsigned)sizeof(se3_sums_t), (unsigned)sizeof(se3_out_t),
                                    (unsigned)sizeof(se3_friction_t), 0U, 0U, 0U};

  return SIZES[which & 7U];
}

/** a b, 3 by 3 row-major. */
static void times(const se3_real_t a[9], const se3_real_t b[9], se3_real_t out[9])
{
  for (size_t i = 0U; i < 3U; i++)
  {
    for (size_t j = 0U; j < 3U; j++)
    {
      out[(3U * i) + j] = ((a[3U * i] * b[j]) + (a[(3U * i) + 1U] * b[3U + j]))
                          + (a[(3U * i) + 2U] * b[6U + j]);
    }
  }
}

/** a x = b for a symmetric, positive a (xx xy xz yy yz zz): its L D L', no root, a division a
    pivot. */
static void solved(const double a[6], const double b[3], double x[3])
{
  const double i0 = 1.0 / a[0];
  const double l10 = a[1] * i0, l20 = a[2] * i0;
  const double i1 = 1.0 / (a[3] - (l10 * a[1]));
  const double e = a[4] - (l20 * a[1]);
  const double l21 = e * i1;
  const double i2 = 1.0 / ((a[5] - (l20 * a[2])) - (l21 * e));
  const double f1 = b[1] - (l10 * b[0]);
  const double f2 = (b[2] - (l20 * b[0])) - (l21 * f1);

  x[2] = f2 * i2;
  x[1] = (f1 * i1) - (l21 * x[2]);
  x[0] = ((b[0] * i0) - (l10 * x[1])) - (l20 * x[2]);
}

/** The turn that answers the gradient b on the curvature h: h (h h + floor^2)^-1 (-b), each of
    h's own directions by its curvature c / (c^2 + floor^2) - the inverse where the points tell
    the turn, none where they do not, whatever way those directions lie: pivots cut at a floor
    kept the turn about a line of points wherever the line left the axes. The floor a share of
    h's trace and the rounding of the points at their own scale - one point's lever is its
    centroid's last bit, a curvature of rounding alone. In double whatever the lanes are: the
    square of h stands over its rounding there, and h last, so what the solve leaves along a
    direction of no curvature is multiplied by none. */
static void turn(const se3_real_t h[9], const se3_real_t b[3], se3_real_t spread, se3_real_t out[3])
{
  const double xx = (double)h[0], xy = (double)h[1], xz = (double)h[2];
  const double yy = (double)h[4], yz = (double)h[5], zz = (double)h[8];
  const double ground = ((SE3_SHARE * fabs((xx + yy) + zz)) + ((double)SE3_EPS * (double)spread))
                        + LEAST;
  const double f2 = ground * ground;
  const double a[6] = {(((xx * xx) + (xy * xy)) + (xz * xz)) + f2,
                       ((xx * xy) + (xy * yy)) + (xz * yz),
                       ((xx * xz) + (xy * yz)) + (xz * zz),
                       (((xy * xy) + (yy * yy)) + (yz * yz)) + f2,
                       ((xy * xz) + (yy * yz)) + (yz * zz),
                       (((xz * xz) + (yz * yz)) + (zz * zz)) + f2};
  const double g[3] = {0.0 - (double)b[0], 0.0 - (double)b[1], 0.0 - (double)b[2]};
  double       y[3];

  solved(a, g, y);
  out[0] = (se3_real_t)(((xx * y[0]) + (xy * y[1])) + (xz * y[2]));
  out[1] = (se3_real_t)(((xy * y[0]) + (yy * y[1])) + (yz * y[2]));
  out[2] = (se3_real_t)(((xz * y[0]) + (yz * y[1])) + (zz * y[2]));
}

/** `v` no longer than `cap`: scaled whole, its own bits where it is inside. */
static void capped(se3_real_t v[3], se3_real_t cap)
{
  const se3_real_t size = SE3_ROOT(((v[0] * v[0]) + (v[1] * v[1])) + (v[2] * v[2]));
  const se3_real_t held = over(cap) + SE3_TINY;
  const se3_real_t scale = held / most(held, size);

  v[0] *= scale;
  v[1] *= scale;
  v[2] *= scale;
}

/** The pose after the turn w about the centroid and its shift v: Rodrigues' turn by its series
    in w's square - no angle taken, no division by one -, laid back on SO(3) by a step of
    Newton's and Schulz's, r (3 - r'r) / 2; the place t + (r - r_out) centre + v, so the
    centroid moves by v alone and nothing multiplies a place by a turn. */
static void taken(const se3_real_t r[9], const se3_real_t t[3], const se3_real_t centre[3],
                  const se3_real_t w[3], const se3_real_t v[3], se3_real_t r_out[9],
                  se3_real_t t_out[3])
{
  const se3_real_t xx = w[0] * w[0], yy = w[1] * w[1], zz = w[2] * w[2];
  const se3_real_t xy = w[0] * w[1], xz = w[0] * w[2], yz = w[1] * w[2];
  const se3_real_t u = (xx + yy) + zz;
  se3_real_t       a = ONE, b = ONE, by[9], raw[9], gram[9];

  for (size_t k = TERMS; k > 0U; k--)
  {
    a = ONE - ((u * SINE[k - 1U]) * a);
    b = ONE - ((u * COSINE[k - 1U]) * b);
  }
  b *= HALF;
  by[0] = ONE - (b * (yy + zz));
  by[1] = (b * xy) - (a * w[2]);
  by[2] = (b * xz) + (a * w[1]);
  by[3] = (b * xy) + (a * w[2]);
  by[4] = ONE - (b * (xx + zz));
  by[5] = (b * yz) - (a * w[0]);
  by[6] = (b * xz) - (a * w[1]);
  by[7] = (b * yz) + (a * w[0]);
  by[8] = ONE - (b * (xx + yy));
  times(by, r, raw);
  for (size_t i = 0U; i < 3U; i++)
  {
    for (size_t j = 0U; j < 3U; j++)
    {
      const se3_real_t dot = ((raw[i] * raw[j]) + (raw[3U + i] * raw[3U + j]))
                             + (raw[6U + i] * raw[6U + j]);

      gram[(3U * i) + j] = HALF * ((THREE * EYE[(3U * i) + j]) - dot);
    }
  }
  times(raw, gram, r_out);
  for (size_t i = 0U; i < 3U; i++)
  {
    const se3_real_t lever = (((r[3U * i] - r_out[3U * i]) * centre[0])
                              + ((r[(3U * i) + 1U] - r_out[(3U * i) + 1U]) * centre[1]))
                             + ((r[(3U * i) + 2U] - r_out[(3U * i) + 2U]) * centre[2]);

    t_out[i] = (t[i] + lever) + v[i];
  }
}

/** The step off the sums `out` holds, from the pose (r, t): the fit's and the friction's alike. */
static void stepped(const se3_cfg_t *cfg, const se3_real_t r[9], const se3_real_t t[3],
                    se3_out_t *out)
{
  const se3_sums_t *s = &out->sums;
  se3_real_t        held, inverse;

  for (size_t i = 0U; i < 3U; i++)
  {
    out->at[i] = (((r[3U * i] * s->centre[0]) + (r[(3U * i) + 1U] * s->centre[1]))
                  + (r[(3U * i) + 2U] * s->centre[2]))
                 + t[i];
  }
  out->curve[0] = (s->second[SE3_YY] + s->second[SE3_ZZ]) + cfg->damp_turn;
  out->curve[4] = (s->second[SE3_XX] + s->second[SE3_ZZ]) + cfg->damp_turn;
  out->curve[8] = (s->second[SE3_XX] + s->second[SE3_YY]) + cfg->damp_turn;
  out->curve[1] = ZERO - s->second[SE3_XY];
  out->curve[2] = ZERO - s->second[SE3_XZ];
  out->curve[5] = ZERO - s->second[SE3_YZ];
  out->curve[3] = out->curve[1];
  out->curve[6] = out->curve[2];
  out->curve[7] = out->curve[5];
  turn(out->curve, s->turn, s->spread, out->turned);
  held = s->weight + cfg->damp_shift;
  inverse = held / ((held * held) + SE3_TINY);
  for (size_t i = 0U; i < 3U; i++)
  {
    out->shifted[i] = ZERO - (s->shift[i] * inverse);
  }
  capped(out->turned, least(cfg->most_turn, (se3_real_t)SE3_TURN));
  capped(out->shifted, cfg->most_shift);
  taken(r, t, s->centre, out->turned, out->shifted, out->r, out->t);
}

void se3_step(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg, const se3_real_t r[9],
              const se3_real_t t[3], se3_out_t *out)
{
  se3_summed(planes, stride, cfg->soft, r, t, &out->sums);
  stepped(cfg, r, t, out);
}

void se3_grip(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg,
              const se3_friction_t *friction, const se3_real_t r[9], const se3_real_t t[3],
              unsigned take, se3_out_t *out)
{
  se3_rubbed(planes, stride, friction, r, t, take, &out->sums);
  stepped(cfg, r, t, out);
}
