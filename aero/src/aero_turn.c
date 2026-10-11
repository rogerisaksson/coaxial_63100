/** aero_turn.c - the frame's attitude: where up is asked to stand in it, and the torque that
    takes it there as far as the rotors render the payload's impedance. No branch. */
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

/** Where up is asked to stand in the frame, a point on the sphere: the tilt's exponential
    (its series, no angle taken), leant against what the air pushes across the rotors' axis;
    along a dropped mode it stands where the frame has it. The turn from where it stands to
    there is the miss; the lift is the collective and the air's part along the axis. */
void aero_refer(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float *w = s->now[AERO_WEIGHT];
  const float  wp = w[AERO_PITCH], wr = w[AERO_ROLL], wc = w[AERO_THRUST];
  const float  tx = -least(PI, most(-PI, s->ask[AERO_PITCH]));
  const float  tz = -least(PI, most(-PI, s->ask[AERO_ROLL]));
  const float  q = tx * tx + tz * tz;
  const float  seen[AERO_AXES] = {in->r[3], in->r[4], in->r[5]};
  float        c = 0.0f, sc = 0.0f, mine[AERO_AXES], d[AERO_AXES], want[AERO_AXES];
  float        f[AERO_AXES], n[AERO_AXES], leant[AERO_AXES];
  float        along, size, lean, norm, a, b, y, dot;

  (void)p;
  (void)out;
  for (int k = TERMS - 1; k >= 0; k--)
  {
    c = COS[k] + q * c;
    sc = SINC[k] + q * sc;
  }
  want[0] = tz * sc;
  want[1] = c;
  want[2] = -tx * sc;
  /* the air's push in the frame as asked - the frame's own, turned the least turn that takes
     up as it stands to up as asked -, and the thrust that answers it with the collective */
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    mine[j] = in->r[j] * s->push_hat[0] + in->r[3 + j] * s->push_hat[1]
              + in->r[6 + j] * s->push_hat[2];
  }
  least_turn(seen, want, mine, d);
  along = most(wc * s->ask[AERO_THRUST] - d[1], TINY);
  size = sqrtf(d[0] * d[0] + d[2] * d[2]);
  lean = wp * wr * wc * least(1.0f, s->p.lean * along / most(size, TINY));
  f[0] = -lean * d[0];
  f[1] = along;
  f[2] = -lean * d[2];
  norm = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    n[j] = f[j] / norm;
  }
  s->lift = s->p.mass * wc * norm;
  /* up as asked, turned as the rotors' axis is from n: the least turn, n onto the axis */
  least_turn(n, UP, want, leant);
  /* a dropped mode's part of it is the frame's own: the wing's rise is the roll's, the
     nose's the pitch's */
  a = wr * leant[0] + (1.0f - wr) * seen[0];
  b = wp * leant[2] + (1.0f - wp) * seen[2];
  y = copysignf(sqrtf(most(1.0f - a * a - b * b, 0.0f)),
                wp * wr * leant[1] + (1.0f - wp * wr) * seen[1]);
  norm = most(sqrtf(a * a + y * y + b * b), TINY);
  s->up[0] = a / norm;
  s->up[1] = y / norm;
  s->up[2] = b / norm;
  s->miss[0] = s->up[1] * seen[2] - s->up[2] * seen[1];
  s->miss[1] = s->up[2] * seen[0] - s->up[0] * seen[2];
  s->miss[2] = s->up[0] * seen[1] - s->up[1] * seen[0];
  /* past a quarter turn its sine falls: a unit there */
  dot = s->up[0] * seen[0] + s->up[1] * seen[1] + s->up[2] * seen[2];
  s->aim = most(dot, 0.0f);
  norm = sqrtf(s->miss[0] * s->miss[0] + s->miss[1] * s->miss[1] + s->miss[2] * s->miss[2]);
  norm = pick(neg(dot), 1.0f, 1.0f / most(norm, TINY));
  for (int j = 0; j < (int)AERO_AXES; j++)
  {
    s->miss[j] *= norm;
  }
}

/** The torque asked about each axis, the payload's impedance as far as the rotors render it.
    Its attitude mode's stiffness on the miss and damping on the rate the target itself has
    not; its rate mode's damping on its rate's miss and stiffness on that miss summed, where
    no attitude is tracked there: one rate asked, d of its miss the acceleration. Past what
    the rotors' reach this pass turns - a jerk, j - the miss is closed at the fastest rate
    they stop from, (j e^2)^(1/3), and that rate come to at sqrt(2 j x): each curve met where
    its slope is the payload's. A tilt's torque the frame's turn about the rotors' axis
    carries round is asked no larger than the reach follows it round, j over that turn's
    rate, and its miss closed no faster than that stops. A rate's miss is summed inside the
    payload's own slopes and the rotors' reach alone. Less what the observer holds. */
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
                      + (wa * s->now[AERO_DAMP][att] * WAY[a] * s->rate[att]
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
