/** se3.h - a pose fitted to weighted points in SE(3): a Gauss-Newton step about their centroid */
#ifndef SE3_H
#define SE3_H

#include <float.h>
#include <stddef.h>

/* A pose (r, t) lays a body's points p on their goals g: q = r p + t, its miss q - g. A step
 * lessens the weighted squares of the misses about the points' own weighted centroid: there the
 * turn and the shift part, a 3 by 3 for the turn and a division for the shift, and no place in
 * the world enters the turn's curvature. What the points cannot tell - the turn about a line of
 * them, any turn of one point - gets no step: the curvature's inverse is filtered where it is
 * under its floor, turned with the curvature itself, whatever axis the line lies along. No
 * jump the data decides: a pass is the same instructions bearing or in the air. Built for
 * double, or for float with SE3_SINGLE; host/tools/cores/se3.py holds its numpy twin, the same
 * bits. The firmware builds none of it.
 *
 * Under friction (se3_grip) the same planes are a body's contact: a goal is its point's anchor
 * in the ground, where the point's bristle stands unstrained, the weight the load pressing the
 * point there, the miss the bristle's strain. The anchors are what the contact remembers: a
 * pose alone tells no pull, the way the body came there does (se3_anchored). */

#ifdef SE3_SINGLE
typedef float se3_real_t;
/** The data's rounding, and the least a sum is divided by. */
#define SE3_EPS  FLT_EPSILON
#define SE3_TINY 1e-30f
/** The floor's share of the curvature's trace: over the sums' own noise, 1e-7 of it. */
#define SE3_SHARE 1e-4
#endif
#ifndef SE3_SINGLE
typedef double se3_real_t;
#define SE3_EPS  DBL_EPSILON
#define SE3_TINY 1e-60
/** The floor's share of the curvature's trace: its square stands 1e4 over a double's rounding. */
#define SE3_SHARE 1e-6
#endif

/** The lanes a sum is kept in, the widest target's vector: a stride is a multiple of them. */
#define SE3_LANES 16U
/** A step's turn at most, rad: the series' eight terms hold 3e-15 to it. */
#define SE3_TURN 1.0

/** The arena's planes in this order, each a stride long, one block: the points in their body,
    their goals in the world's frame about the caller's anchor - the pose's t about it too, so
    no place far off costs a float its grain -, their weights, past the last point zeros; then
    the step's own, each point's miss and its weight as the first pass leaves them the second -
    under friction each point's pull and its bristle's stiffness. */
enum {
  SE3_PX, SE3_PY, SE3_PZ, SE3_GX, SE3_GY, SE3_GZ, SE3_W, SE3_MX, SE3_MY, SE3_MZ, SE3_WE, SE3_PLANES
};

/** The second moments' order in se3_sums_t. */
enum { SE3_XX, SE3_XY, SE3_XZ, SE3_YY, SE3_YZ, SE3_ZZ, SE3_SECOND };

typedef struct
{
  se3_real_t damp_turn;  /* added to the turn's curvature, a weight's m^2 */
  se3_real_t damp_shift; /* added to the weights' sum */
  se3_real_t soft;       /* a miss r weighs w / (1 + soft r^2), 1/m^2: 0 the squares alone */
  se3_real_t most_turn;  /* a step's turn at most, rad; SE3_TURN where over it */
  se3_real_t most_shift; /* a step's shift at most, m */
} se3_cfg_t;

/** A bearing point's friction, a bristle from its anchor. Inside the reach the point's load
    holds, mu load / stiff, the bristle is a spring of the whole stiffness; past the reach the
    bristle keeps the reach's share of the miss, so the pull stands on the cone, mu load,
    whatever the miss; a point with no load pulls nothing. mu falls from `rest` to `slide` with
    the point's own speed, spun p + sped: the body's rate, twelve numbers for every point of the
    body. The rate's time is the caller's, and `drag` and `fade` are in the same: with the rate a
    tick's change, `drag` is a stiffness as `stiff` is. */
typedef struct
{
  se3_real_t stiff;   /* a bristle's stiffness, N/m */
  se3_real_t drag;    /* a bearing point's viscous stiffness: on its speed as `stiff` on its miss */
  se3_real_t rest;    /* the friction's coefficient at rest */
  se3_real_t slide;   /* and sliding */
  se3_real_t fade;    /* 1 / v^2, v the speed where the friction stands half way between them */
  se3_real_t spun[9]; /* the body's rate: (r - the r before) / dt, row-major */
  se3_real_t sped[3]; /* and (t - the t before) / dt */
} se3_friction_t;

/** The points' sums at a pose: about their centroid, the lever d = r (p - centre). Under
    friction `turn` is the bristles' torque about the centroid, `shift` their force, `cost` their
    strain's energy, and a weight a bristle's stiffness as its miss and speed leave it. */
typedef struct
{
  se3_real_t weight;              /* the weights' sum, each as its miss leaves it */
  se3_real_t spread;              /* the sum of w |p|^2: the scale the floor's rounding has */
  se3_real_t centre[3];           /* the weighted centroid, in the body */
  se3_real_t second[SE3_SECOND];  /* the sums of w d d' */
  se3_real_t turn[3];             /* the sum of w d x miss */
  se3_real_t shift[3];            /* the sum of w miss */
  se3_real_t cost;                /* half the sum of w |miss|^2 */
} se3_sums_t;

typedef struct
{
  se3_sums_t sums;
  se3_real_t at[3];      /* the centroid's place in the world's frame */
  se3_real_t curve[9];   /* the turn's curvature, damped */
  se3_real_t turned[3];  /* the step's turn about the centroid, rad */
  se3_real_t shifted[3]; /* the step's shift of the centroid, m */
  se3_real_t r[9];       /* the pose after the step: its turn, on SO(3) */
  se3_real_t t[3];       /* and its place */
} se3_out_t;

/** The size of se3_real_t, se3_cfg_t, se3_sums_t, se3_out_t, se3_friction_t by `which` 0 to 4. */
unsigned se3_size(unsigned which);

/** The sums of `stride` points (a multiple of SE3_LANES, 16 at least) at the pose (r row-major,
    t): two passes of the planes, the weights and the centroid, then the moments about it. */
void se3_summed(se3_real_t *planes, size_t stride, se3_real_t soft, const se3_real_t r[9],
                const se3_real_t t[3], se3_sums_t *sums);

/** A step from the pose (r, t), into `out` - apart from r and t. */
void se3_step(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg, const se3_real_t r[9],
              const se3_real_t t[3], se3_out_t *out);

/** The sums of `stride` points under `friction` at the pose (r, t), se3_summed's two passes:
    each point's pull and its bristle's stiffness left in the step's own planes. With `take` 1
    the first pass takes the anchors as well, se3_anchored's own bits and no pass of its own: a
    tick's sums and its memory at once; with 0 the anchors keep theirs. */
void se3_rubbed(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                const se3_real_t r[9], const se3_real_t t[3], unsigned take, se3_sums_t *sums);

/** A step from the pose (r, t) under `friction`, into `out`: towards where the body rests on
    its bristles, by `cfg`'s damping and caps - `soft` unread, the cone lets a miss go. `take`
    as se3_rubbed's: 1 where the pose is the one the body has, 0 where a pose is tried. */
void se3_grip(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg,
              const se3_friction_t *friction, const se3_real_t r[9], const se3_real_t t[3],
              unsigned take, se3_out_t *out);

/** The anchors taken to where the pose (r, t) leaves them, in their own planes: g = q - share
    (q - g), q the point's place and the share se3_rubbed's. An anchor inside its reach stays,
    one past it follows its point at the reach, one with no load stands under its point - where
    the point lands, the bristle holds at once. se3_rubbed's first pass alone, the anchors taken:
    the step's own planes are left as that pass leaves them. */
void se3_anchored(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                  const se3_real_t r[9], const se3_real_t t[3]);

#endif /* SE3_H */
