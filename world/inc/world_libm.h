/** world_libm.h - ucrt's fminf and fmaxf inline, bit for bit: signed zeros as ucrt orders them,
    two NaNs the second; forced into the stand-in's Windows build (tools.cores.stand), where the
    calls took 30 % of the drive's period. gcc keeps a NaN-honouring fminf a call; glibc's
    differs on zeros (fmaxf(+0, -0) is -0 there). */
#ifndef WORLD_LIBM_H
#define WORLD_LIBM_H

#include <math.h>
#include <stdint.h>
#include <string.h>

static inline float world_libm_bits(float a, float b, int both)
{
  uint32_t x, y;

  memcpy(&x, &a, sizeof x);
  memcpy(&y, &b, sizeof y);
  x = both ? (x & y) : (x | y);
  memcpy(&a, &x, sizeof a);
  return a;
}

static inline float world_fmaxf(float a, float b)
{
  return (a > b) ? a : (a < b) ? b : (a != a) ? b : (b != b) ? a : world_libm_bits(a, b, 1);
}

static inline float world_fminf(float a, float b)
{
  return (a < b) ? a : (a > b) ? b : (a != a) ? b : (b != b) ? a : world_libm_bits(a, b, 0);
}

#define fmaxf(a, b) world_fmaxf((a), (b))
#define fminf(a, b) world_fminf((a), (b))

#endif /* WORLD_LIBM_H */
