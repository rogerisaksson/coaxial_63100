/** se3_bits.h - se3/src's own: its precision's functions, a choice by a sign bit and no flag */
#ifndef SE3_BITS_H
#define SE3_BITS_H

#include "se3.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef SE3_SINGLE
typedef uint32_t se3_word_t;
#define SE3_SIGN    31U
#define SE3_ROOT(x) sqrtf(x)
#define SE3_SIZE(x) fabsf(x)
#endif
#ifndef SE3_SINGLE
typedef uint64_t se3_word_t;
#define SE3_SIGN    63U
#define SE3_ROOT(x) sqrt(x)
#define SE3_SIZE(x) fabs(x)
#endif

#define ZERO  ((se3_real_t)0.0)
#define HALF  ((se3_real_t)0.5)
#define ONE   ((se3_real_t)1.0)
#define THREE ((se3_real_t)3.0)

/** x where it is over zero, else zero: exact either way, a mask and a sum in a lane. */
static inline se3_real_t over(se3_real_t x)
{
  return HALF * (x + SE3_SIZE(x));
}

/** All ones where x is under zero: its sign bit. */
static inline se3_word_t under(se3_real_t x)
{
  se3_word_t u;

  memcpy(&u, &x, sizeof(u));
  return (se3_word_t)0 - (u >> SE3_SIGN);
}

/** `a` where `k` is none, `b` where it is all: by bits, neither one's value touched. */
static inline se3_real_t pick(se3_word_t k, se3_real_t a, se3_real_t b)
{
  se3_word_t ua, ub;

  memcpy(&ua, &a, sizeof(ua));
  memcpy(&ub, &b, sizeof(ub));
  ua = (ua & ~k) | (ub & k);
  memcpy(&a, &ua, sizeof(a));
  return a;
}

static inline se3_real_t most(se3_real_t a, se3_real_t b)
{
  return pick(under(a - b), a, b);
}

static inline se3_real_t least(se3_real_t a, se3_real_t b)
{
  return pick(under(b - a), a, b);
}

/** The share of its bristle's stiffness a point keeps. The point's reach is the strain its load
    holds at its speed: the friction between sliding and rest by the speed's square, slide + fall
    / pace, times the load, over the stiffness (`yield` its inverse). Inside the reach the whole,
    past it the reach over the miss - the pull stays on the cone -, with no load none. The reach
    and the miss are both taken times the pace, 1 + speed^2 fade: one division a point. The core's
    friction pass inlines it on its lanes (se3_sums.c), se3_kept is it alone (se3_law.c). */
static inline se3_real_t share(se3_real_t load, se3_real_t speed2, se3_real_t miss2,
                               se3_real_t slide, se3_real_t fall, se3_real_t fade,
                               se3_real_t yield)
{
  const se3_real_t pace = ONE + (speed2 * fade);
  const se3_real_t reach = (((slide * pace) + fall) * load) * yield;

  return reach / (most(reach, SE3_ROOT(miss2) * pace) + SE3_TINY);
}

#endif /* SE3_BITS_H */
