/**
  ******************************************************************************
  * @file    filter.c
  * @brief   The decimating anti-alias chain. See filter.h for why it is two
  *          stages and why the host designs the second one.
  ******************************************************************************
  */
#include "filter.h"

#include <string.h>


void filter_reset(filter_channel_t *ch)
{
  if (ch == NULL)
  {
    return;
  }
  memset(ch, 0, sizeof(*ch));
}


void filter_prime(const filter_design_t *design, filter_channel_t *ch,
                  float value)
{
  if ((design == NULL) || (ch == NULL))
  {
    return;
  }

  ch->box_sum = 0;
  ch->box_n = 0U;
  ch->out_n = 0U;
  ch->taken = 0U;

  float x = value;

  for (uint8_t i = 0U; i < design->sections; i++)
  {
    const filter_biquad_t *s = &design->section[i];
    const float den = 1.0f + s->a1 + s->a2;
    /* A section whose poles sum to -1 has no DC gain to solve for; it
       cannot be primed, and zero is what it would settle to anyway. */
    const float y = (den != 0.0f)
                    ? (x * ((s->b0 + s->b1 + s->b2) / den)) : 0.0f;

    ch->s2[i] = (s->b2 * x) - (s->a2 * y);
    ch->s1[i] = (s->b1 * x) - (s->a1 * y) + ch->s2[i];
    x = y;
  }
}


void filter_pass_through(filter_design_t *design)
{
  if (design == NULL)
  {
    return;
  }
  memset(design, 0, sizeof(*design));
  design->boxcar = 1U;
  design->decimate = 1U;
  design->sections = 0U;
}


bool filter_valid(const filter_design_t *design)
{
  return (design != NULL) && (design->boxcar > 0U) &&
         (design->decimate > 0U) && (design->sections <= FILTER_MAX_SECTIONS);
}


uint32_t filter_ratio(const filter_design_t *design)
{
  if (!filter_valid(design))
  {
    return 1U;
  }
  return (uint32_t)design->boxcar * (uint32_t)design->decimate;
}


/** One section, transposed direct form II.
  *
  *   y  = b0*x + s1
  *   s1 = b1*x - a1*y + s2
  *   s2 = b2*x - a2*y
  *
  * Two multiply-adds more than direct form I costs, and worth it: the state
  * variables carry the signal's own magnitude rather than its square, so a
  * 16-bit code through an 8th-order cascade does not walk into the float's
  * tail.
  */
static float section_run(const filter_biquad_t *s, float *s1, float *s2,
                         float x)
{
  const float y = (s->b0 * x) + *s1;

  *s1 = (s->b1 * x) - (s->a1 * y) + *s2;
  *s2 = (s->b2 * x) - (s->a2 * y);
  return y;
}


bool filter_push_value(const filter_design_t *design, filter_channel_t *ch,
                       float value, float *out)
{
  if ((ch == NULL) || (out == NULL) || !filter_valid(design))
  {
    return false;
  }

  float x = value;

  for (uint8_t i = 0U; i < design->sections; i++)
  {
    x = section_run(&design->section[i], &ch->s1[i], &ch->s2[i], x);
  }

  ch->out_n++;
  if (ch->out_n < design->decimate)
  {
    return false;
  }
  ch->out_n = 0U;

  *out = x;
  return true;
}


bool filter_push(const filter_design_t *design, filter_channel_t *ch,
                 int32_t sample, float *out)
{
  if ((ch == NULL) || (out == NULL) || !filter_valid(design))
  {
    return false;
  }

  ch->taken++;

  /* STAGE 1, the only thing that can run at the converter's rate: one add.
     Summing rather than averaging keeps the bits an average throws away,
     and the divide happens once per dump instead of once per sample. */
  ch->box_sum += sample;
  ch->box_n++;
  if (ch->box_n < design->boxcar)
  {
    return false;
  }

  float x = (float)ch->box_sum / (float)design->boxcar;

  ch->box_sum = 0;
  ch->box_n = 0U;

  /* STAGE 2, on the thinned stream where there is time for it. */
  for (uint8_t i = 0U; i < design->sections; i++)
  {
    x = section_run(&design->section[i], &ch->s1[i], &ch->s2[i], x);
  }

  /* And only now is a sample thrown away - after something shaped what
     would otherwise have folded on top of the answer. */
  ch->out_n++;
  if (ch->out_n < design->decimate)
  {
    return false;
  }
  ch->out_n = 0U;

  *out = x;
  return true;
}
