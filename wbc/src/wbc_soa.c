/** wbc_soa.c - her loop's side of the kernel's SoA bank (wbc_soa.h, machine.texts.lowering). */
#include "wbc_soa.h"

float wbc_soa_biquad(wbc_soa_t *b, int k, float x)
{
  const float y = b->b0[k] * x + b->s1[k];

  b->s1[k] = b->b1[k] * x - b->a1[k] * y + b->s2[k];
  b->s2[k] = b->b2[k] * x - b->a2[k] * y;
  return y;
}
