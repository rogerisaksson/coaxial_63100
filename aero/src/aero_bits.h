/** aero_bits.h - a choice without a jump, for aero/src alone: a mask from a sign bit - no flag
    is read, so no compiler makes a branch of it -, a value picked by its bits; and the steps
    one source lays for another. */
#ifndef AERO_BITS_H
#define AERO_BITS_H

#include "aero.h"

#include <string.h>

/** The column of a mode an axis has not: zeros. */
#define NONE AERO_MODES
#define TINY 1e-12f

/** All ones where `v` is not zero. */
static inline uint32_t set(uint32_t v)
{
  return 0U - ((v | (0U - v)) >> 31);
}

/** All ones where x is under zero: its sign bit. */
static inline uint32_t neg(float x)
{
  uint32_t u;

  memcpy(&u, &x, sizeof(u));
  return 0U - (u >> 31);
}

/** All ones where a < b. */
static inline uint32_t lt(float a, float b)
{
  return neg(a - b);
}

/** `a` where `k` is none, `b` where it is all: by bits, neither one's value touched. */
static inline float pick(uint32_t k, float a, float b)
{
  uint32_t ua, ub;

  memcpy(&ua, &a, sizeof(ua));
  memcpy(&ub, &b, sizeof(ub));
  ua = (ua & ~k) | (ub & k);
  memcpy(&a, &ua, sizeof(a));
  return a;
}

/** 1 where `k` is all, 0 where none. */
static inline float one(uint32_t k)
{
  return pick(k, 0.0f, 1.0f);
}

static inline float least(float a, float b)
{
  return pick(lt(b, a), a, b);
}

static inline float most(float a, float b)
{
  return pick(lt(a, b), a, b);
}

/** All ones for a number, none for a NaN: by its bits. */
static inline uint32_t known(float x)
{
  uint32_t u;

  memcpy(&u, &x, sizeof(u));
  return ~(0U - ((0x7F800000U - (u & 0x7FFFFFFFU)) >> 31));
}

/** A NaN as 0. */
static inline float clean(float x)
{
  return pick(known(x), 0.0f, x);
}

/** A pass's steps (aero.c's table), each its own arguments alone. */
void aero_refer(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out);
void aero_turn(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out);

#endif /* AERO_BITS_H */
