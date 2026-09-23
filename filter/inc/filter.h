/** filter.h - The decimating anti-alias chain: read a fast converter over a
    slow link without folding what it saw into the answer. */
#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdint.h>

/** Biquads in the cascade. */
#define FILTER_MAX_SECTIONS 4U

/** One second-order section, `a0` normalised out by the host. */
typedef struct
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
} filter_biquad_t;

/** What the host designed. Shared by every channel; the state is not. */
typedef struct
{
  uint16_t boxcar;    /**< stage 1: sum this many input samples, 1 is off  */
  uint16_t decimate;  /**< stage 2: emit every Nth filtered sample, 1 is off */
  uint8_t  sections;  /**< biquads in use, 0 leaves the boxcar alone       */
  filter_biquad_t section[FILTER_MAX_SECTIONS];
} filter_design_t;

/** One channel's running state. */
typedef struct
{
  float    s1[FILTER_MAX_SECTIONS];
  float    s2[FILTER_MAX_SECTIONS];
  int32_t  box_sum;
  uint16_t box_n;
  uint16_t out_n;
  uint32_t taken;     /**< input samples seen since the last reset         */
} filter_channel_t;

/** Forget everything a channel accumulated. Call on reconfigure: */
void filter_reset(filter_channel_t *ch);

/** Put a channel's state where it would be if `value` had been on */
void filter_prime(const filter_design_t *design, filter_channel_t *ch,
                  float value);

/** A design that changes nothing - no boxcar, no sections, no */
void filter_pass_through(filter_design_t *design);

/** True when `design` can be run: the rates are non-zero and the */
bool filter_valid(const filter_design_t *design);

/** One input sample in; true when an output sample came out.
    @param  design  the host's, unchanged by this call
    @param  ch      this channel's state, advanced
    @param  sample  a raw ADC code, whatever the converter gave
    @param  out     the filtered, decimated value - written only on true */
bool filter_push(const filter_design_t *design, filter_channel_t *ch,
                 int32_t sample, float *out);

/** The chain's decimation, input samples per output sample. */
uint32_t filter_ratio(const filter_design_t *design);

/** A value that is ALREADY the boxcar's answer, in; true when an */
bool filter_push_value(const filter_design_t *design, filter_channel_t *ch,
                       float value, float *out);

#endif /* FILTER_H */
