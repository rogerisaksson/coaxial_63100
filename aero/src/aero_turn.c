/** aero_turn.c - the frame's attitude: where its rotors' axis is asked to point, and the
    torque that takes it there as far as the rotors render the payload's impedance. No branch. */
#include "aero_bits.h"

#include <math.h>

#define PI       3.14159265358979f
/** The rotors' reach a pass is the two tilts' together: each plans on this share of it. */
#define SHARED   0.5f
/** Two directions are opposite from here: 1 + their dot, the least a least turn divides by. */
#define OPPOSITE 1e-3f
/** The terms of cos and sinc in the tilt's square: 4e-6 at half a turn about each axis. */
#define TERMS    10

/** Each axis of the frame - x its wing, y its rotors', z its nose -: the attitude mode about
    it and the rate mode, and the way a mode's sign turns the frame about it. */
static const uint8_t ATTITUDE[AERO_AXES] = {AERO_PITCH, NONE, AERO_ROLL};
static const uint8_t RATE[AERO_AXES] = {AERO_PITCH_RATE, AERO_YAW_RATE, AERO_ROLL_RATE};
static const float   WAY[AERO_AXES] = {-1.0f, 1.0f, -1.0f};
static const float   UP[AERO_AXES] = {0.0f, 1.0f, 0.0f};
static const float   AHEAD[AERO_AXES] = {0.0f, 0.0f, 1.0f};

/** cos sqrt q and sin sqrt q / sqrt q, their series' terms in q. */
static const float COS[TERMS] = {
  1.0f, -1.0f / 2.0f, 1.0f / 24.0f, -1.0f / 720.0f, 1.0f / 40320.0f, -1.0f / 3628800.0f,
  1.0f / 479001600.0f, -1.0f / 87178291200.0f, 1.0f / 20922789888000.0f,
  -1.0f / 6402373705728000.0f,
};
static const float SINC[TERMS] = {
  1.0f, -1.0f / 6.0f, 1.0f / 120.0f, -1.0f / 5040.0f, 1.0f / 362880.0f, -1.0f / 39916800.0f,
  1.0f / 6227020800.0f, -1.0f / 1307674368000.0f, 1.0f / 355687428096000.0f,
  -1.0f / 121645100408832000.0f,
};

/** The cube root of x >= 0: its bits' third, three of Newton's steps; no library's. */
static inline float root3(float x)
{
  uint32_t u;
  float    y;

  memcpy(&u, &x, sizeof(u));
  u = u / 3U + 709921077U;
  memcpy(&y, &u, sizeof(y));
  for (int k = 0; k < 3; k++)
  {
    y -= (y * y * y - x) / (3.0f * y * y);
  }
  return y;
}

/** `v` turned by the least turn that takes the unit `from` onto the unit `to`: Rodrigues' in
    their cross and dot, no angle taken; opposite, the turn is any and the answer bounded. */
static void least_turn(const float from[AERO_AXES], const float to[AERO_AXES],
                       const float v[AERO_AXES], float out[AERO_AXES])
{
  const float w[AERO_AXES] = {from[1] * to[2] - from[2] * to[1], from[2] * to[0] - from[0] * to[2],
                              from[0] * to[1] - from[1] * to[0]};
  const float u[AERO_AXES] = {w[1] * v[2] - w[2] * v[1], w[2] * v[0] - w[0] * v[2],
                              w[0] * v[1] - w[1] * v[0]};
  const float deep = 1.0f / most(1.0f + from[0] * to[0] + from[1] * to[1] + from[2] * to[2],
                                 OPPOSITE);

  out[0] = v[0] + u[0] + (w[1] * u[2] - w[2] * u[1]) * deep;
  out[1] = v[1] + u[1] + (w[2] * u[0] - w[0] * u[2]) * deep;
  out[2] = v[2] + u[2] + (w[0] * u[1] - w[1] * u[0]) * deep;
}

/** Where the rotors' axis is asked to point, and the tilt that takes it there. The payload's
    attitude is the axis's own tilt from up - the nose up, the axis back; the right wing down,
    the axis right - in the level frame of the heading the core holds: the frame's own as it
    first stood, its tilt taken out, turned on by the yaw rate asked. In the world that is a
    point on the sphere: the tilt's exponential (its series, no angle taken), come to along
    the sphere from where the axis was asked as the payload came. The thrust that makes the
    collective along it in this air is that less the air's push, across it as far as `lean`
    lets: where that points the axis goes, its size the lift. The miss is the tilt that
    takes the axis there, its sine, a unit past a quarter turn: no turn about the axis in
    it, whatever the nose does, and none it cannot make on its edge or its back. A dropped
    mode leaves its axis of the frame free: the other's miss is what its own turn closes of
    the axis's offset across it. */
void aero_refer(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float *w = s->now[AERO_WEIGHT];
  const float  wp = w[AERO_PITCH], wr = w[AERO_ROLL], wc = w[AERO_THRUST];
  const float  both = wp * wr, dt = most(in->dt, TINY);
  const float  tx = -least(PI, most(-PI, s->ask[AERO_PITCH]));
  const float  tz = -least(PI, most(-PI, s->ask[AERO_ROLL]));
  const float  q = tx * tx + tz * tz;
  const float  half = 0.5f * w[AERO_YAW_RATE] * s->ask[AERO_YAW_RATE] * dt;
  const float  seen[AERO_AXES] = {in->r[3], in->r[4], in->r[5]};
  float        c = 0.0f, sc = 0.0f, level[AERO_AXES], want[AERO_AXES], turn[AERO_AXES];
  float        f[AERO_AXES], n[AERO_AXES], g[AERO_AXES], e[AERO_AXES], across[AERO_AXES];
  float        hc, hs, norm, along, pushed, size, lean, unit, trim;

  (void)p;
  (void)out;
  for (int k = TERMS - 1; k >= 0; k--)
  {
    c = COS[k] + q * c;
    sc = SINC[k] + q * sc;
  }
  /* the heading held: the nose's as the frame stands level - at the first reading -, turned
     on by Cayley's map */
  least_turn(UP, seen, AHEAD, level);
  hs = in->r[0] * level[0] + in->r[1] * level[1] + in->r[2] * level[2];
  hc = in->r[6] * level[0] + in->r[7] * level[1] + in->r[8] * level[2];
  norm = most(sqrtf(hc * hc + hs * hs), TINY);
  hc = pick(s->headed, hc / norm, s->head[0]);
  hs = pick(s->headed, hs / norm, s->head[1]);
  s->head[0] = ((1.0f - half * half) * hc - 2.0f * half * hs) / (1.0f + half * half);
  s->head[1] = (2.0f * half * hc + (1.0f - half * half) * hs) / (1.0f + half * half);
  trim = 1.5f - 0.5f * (s->head[0] * s->head[0] + s->head[1] * s->head[1]);
  s->head[0] *= trim;
  s->head[1] *= trim;
  s->headed = ~0U;
  /* the axis asked: in that heading's level frame (tz sc, c, -tx sc) mirrored, then the
     world's; come to along the sphere */
  want[0] = -tz * sc * s->head[0] + tx * sc * s->head[1];
  want[1] = c;
  want[2] = tz * sc * s->head[1] + tx * sc * s->head[0];
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    s->began[j] = pick(s->came, s->began[j], s->stood[j]);
    want[j] = s->began[j] + s->share * (want[j] - s->began[j]);
  }
  norm = most(sqrtf(want[0] * want[0] + want[1] * want[1] + want[2] * want[2]), TINY);
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    want[j] /= norm;
  }
  /* the point's own rate about the frame's axes: what turned it from the last pass's, its
     blend and its oscillator; a step at a payload's coming is none, nor has a mode one that
     was dropped either side */
  turn[0] = (s->point[1] * want[2] - s->point[2] * want[1]) / dt;
  turn[1] = (s->point[2] * want[0] - s->point[0] * want[2]) / dt;
  turn[2] = (s->point[0] * want[1] - s->point[1] * want[0]) / dt;
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    const uint8_t att = ATTITUDE[j];

    s->turns[j] = (1.0f - one(s->came)) * s->from[AERO_WEIGHT][att] * s->to[AERO_WEIGHT][att]
                  * (in->r[j] * turn[0] + in->r[3 + j] * turn[1] + in->r[6 + j] * turn[2]);
    s->point[j] = want[j];
    g[j] = in->r[j] * want[0] + in->r[3 + j] * want[1] + in->r[6 + j] * want[2];
  }
  /* where the lift acts: the axis asked, a dropped mode's part of it the frame's own */
  e[0] = wr * g[0];
  e[2] = wp * g[2];
  e[1] = copysignf(sqrtf(most(1.0f - e[0] * e[0] - e[2] * e[2], 0.0f)), both * g[1] + 1.0f - both);
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    s->stood[j] = in->r[3 * j] * e[0] + in->r[3 * j + 1] * e[1] + in->r[3 * j + 2] * e[2];
  }
  /* the thrust for the collective along it in this air: the air's push taken out, across
     the axis as far as `lean` lets */
  pushed = s->air[0] * s->stood[0] + s->air[1] * s->stood[1] + s->air[2] * s->stood[2];
  along = most(wc * s->ask[AERO_THRUST] - pushed, TINY);
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    across[j] = pushed * s->stood[j] - s->air[j];
  }
  size = sqrtf(across[0] * across[0] + across[1] * across[1] + across[2] * across[2]);
  lean = both * wc * least(1.0f, s->p.lean * along / most(size, TINY));
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    f[j] = along * s->stood[j] + lean * across[j];
    /* the axis leant as for a lift of the frame's weight at the least: with next to none
       asked a lean answers nothing, and the axis stood a tangent's angle off on any push */
    n[j] = most(along, s->p.gravity) * want[j] + lean * across[j];
  }
  s->lift = s->p.mass * wc * sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  norm = most(sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]), TINY);
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    g[j] = (in->r[j] * n[0] + in->r[3 + j] * n[1] + in->r[6 + j] * n[2]) / norm;
  }
  /* the tilt that takes the axis there */
  unit = pick(neg(g[1]), 1.0f, 1.0f / most(sqrtf(g[0] * g[0] + g[2] * g[2]), TINY));
  s->miss[0] = wp * g[2] * (wr * unit + (1.0f - wr) * g[1]);
  s->miss[1] = 0.0f;
  s->miss[2] = -wr * g[0] * (wp * unit + (1.0f - wp) * g[1]);
  s->aim = both * most(g[1], 0.0f)
           + (1.0f - both) * sqrtf(most(1.0f - wr * g[0] * g[0] - wp * g[2] * g[2], 0.0f));
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    s->axis[j] = g[j];
  }
}

/** The torque asked about each axis, the payload's impedance as far as the rotors render it.
    Its attitude mode's stiffness on the miss and damping on the rate the axis asked itself
    has not (`turns`); its rate mode's damping on its rate's miss and stiffness on that miss
    summed, where no attitude is tracked there: one rate asked, d of its miss the
    acceleration. Past what the rotors' reach this pass turns - a jerk, j - the miss is
    closed at the fastest rate they stop from, (j e^2)^(1/3), and that rate come to at
    sqrt(2 j x): each curve met where its slope is the payload's. A tilt's torque the frame's
    turn about the rotors' axis carries round is asked no larger than the reach follows it
    round, j over that turn's rate, and its miss closed no faster than that stops. A rate's
    miss is summed inside the payload's own slopes and the rotors' reach alone. Less what the
    observer holds. */
void aero_turn(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float dt = most(in->dt, TINY);
  const float *n = s->p.inertia;
  /* its own inertia's turn, less what its turn about the rotors' axis carries its tilt's
     rates round by: a tilt is closed in the world, whatever the frame's heading does */
  const float carried[AERO_AXES] = {(n[2] - n[1] + n[0]) * in->spin[1] * in->spin[2],
                                    (n[0] - n[2]) * in->spin[2] * in->spin[0],
                                    (n[1] - n[0] - n[2]) * in->spin[0] * in->spin[1]};
  const float swept[AERO_AXES] = {fabsf(in->spin[1]), 0.0f, fabsf(in->spin[1])};
  float       reach[AERO_AXES] = {0.0f, 0.0f, 0.0f};

  (void)p;
  (void)out;
  for (int i = 0; i < (int)AERO_ROTORS; i++)
  {
    const float room = 0.5f * most(in->hi[i] - in->lo[i], 0.0f);

    reach[0] += SHARED * fabsf(s->p.at_z[i]) * room;
    reach[1] += s->p.kappa * room;
    reach[2] += SHARED * fabsf(s->p.at_x[i]) * room;
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    const uint8_t att = ATTITUDE[a], rat = RATE[a];
    const float   wa = s->now[AERO_WEIGHT][att], wr = s->now[AERO_WEIGHT][rat];
    const float   own = wr * (1.0f - wa);
    const float   miss = WAY[a] * s->ask[rat] - in->spin[a];
    const float   j = most(reach[a] / (n[a] * dt), TINY);
    const float   d = wa * s->now[AERO_DAMP][att] + wr * s->now[AERO_DAMP][rat];
    const float   k = wa * s->now[AERO_STIFF][att], kr = own * s->now[AERO_STIFF][rat];
    const float   slope = k / most(d, TINY);
    const float   knee = j / (slope * slope * slope), linear = j / most(d * d, TINY);
    const float   e = fabsf(s->miss[a]);
    const float   bend = root3(most(e - 4.0f / 27.0f * knee, 0.0f));
    const float   cap = j / most(swept[a], TINY);
    const float   near = least(pick(lt(4.0f / 9.0f * knee, e), slope * e,
                                    root3(j) * bend * bend), sqrtf(2.0f * cap * e));
    const float   x = copysignf(near, s->miss[a]) - in->spin[a]
                      + (wa * s->now[AERO_DAMP][att] * s->turns[a]
                         + wr * s->now[AERO_DAMP][rat] * WAY[a] * s->ask[rat]
                         + kr * s->wound[a]) / most(d, TINY);
    const float   far = sqrtf(2.0f * j * most(fabsf(x) - 0.5f * linear, 0.0f));
    const float   damped = copysignf(least(cap, pick(lt(linear, fabsf(x)), d * fabsf(x), far)),
                                     x);
    const float   bare = k * s->miss[a] + kr * s->wound[a];

    s->torque[a] = n[a] * (pick(lt(0.0f, d), bare, damped) - s->turn_hat[a]) + carried[a];
    s->wound[a] = one(lt(0.0f, own)) * (s->wound[a] + own * miss * dt * (1.0f - s->held[a])
                                        * one(~lt(linear, fabsf(x))));
  }
}
