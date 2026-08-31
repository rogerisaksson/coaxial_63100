/**
  ******************************************************************************
  * @file    filter.h
  * @brief   The decimating anti-alias chain: read a fast converter over a slow
  *          link without folding what it saw into the answer.
  *
  * A converter running at megasamples and a link carrying a few hundred
  * records a second are three orders of magnitude apart. Throwing samples
  * away closes the gap and ALIASES: everything above half the output rate
  * comes back as something that was never there, and no field in the record
  * says so. Averaging N and dumping is better and still not enough - a
  * boxcar's first sidelobe is only 13 dB down.
  *
  * So the chain is two stages, cheap first:
  *
  *   x[fs] -> boxcar M1 (integer accumulate and dump) -> biquads at fs/M1
  *         -> decimate M2 -> y[fs/(M1*M2)]
  *
  * The boxcar costs one add per sample and is the only thing that can run at
  * the converter's rate; the biquads cost ~10 cycles each and run on the
  * thinned stream, where there is time for them. The HOST designs them: it
  * knows the output rate it asked for, so it is the only place that can pick
  * a cutoff against the sampling theorem - see `coaxial/bessel.py`, which
  * also reports what the chain actually attenuates at the folding
  * frequencies rather than asserting it is enough.
  *
  * BESSEL, and the trade is deliberate: its group delay is maximally flat,
  * so a current waveform arrives its shape rather than smeared - which is
  * what a drive wants. It buys that with a gentle rolloff, so it needs more
  * order or a lower cutoff than a Butterworth for the same stopband. The
  * design reports both numbers; nothing here decides for the operator.
  *
  * Portable C11 like Modbus/, Thermal/ and Drive/ - no HAL, no CMSIS - so
  * `test_filter_core.py` runs it on the host against a reference written in
  * Python and an analytic oracle.
  ******************************************************************************
  */
#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdint.h>

/** Biquads in the cascade. Four is an 8th-order Bessel, which is past what
  * the rolloff is worth: the design's own attenuation report is the thing to
  * read, not this ceiling. */
#define FILTER_MAX_SECTIONS 4U

/** One second-order section, `a0` normalised out by the host. Transposed
  * direct form II, which is the numerically kind one in float: the state
  * holds sums of the same magnitude as the signal rather than its square. */
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

/**
  * @brief  Forget everything a channel accumulated. Call on reconfigure:
  *         state carried across a coefficient change is a transient nobody
  *         asked for and nothing in the record would explain.
  */
void filter_reset(filter_channel_t *ch);

/**
  * @brief  A design that changes nothing - no boxcar, no sections, no
  *         decimation. What a task gets before the host has designed one.
  */
void filter_pass_through(filter_design_t *design);

/**
  * @brief  True when `design` can be run: the rates are non-zero and the
  *         section count fits. A refusal belongs to the caller that can say
  *         which field was wrong, so this only answers whether it is usable.
  */
bool filter_valid(const filter_design_t *design);

/**
  * @brief  One input sample in; true when an output sample came out.
  *
  * @param  design  the host's, unchanged by this call
  * @param  ch      this channel's state, advanced
  * @param  sample  a raw ADC code, whatever the converter gave
  * @param  out     the filtered, decimated value - written only on true
  *
  * The boxcar SUMS and divides at the dump, so the intermediate keeps the
  * bits an average would throw away. `int32_t` against a 65535 code bounds
  * the boxcar at 32767 samples, which the host's design honours.
  */
bool filter_push(const filter_design_t *design, filter_channel_t *ch,
                 int32_t sample, float *out);

/**
  * @brief  The chain's decimation, input samples per output sample.
  */
uint32_t filter_ratio(const filter_design_t *design);

/**
  * @brief  A value that is ALREADY the boxcar's answer, in; true when an
  *         output came out.
  *
  * For a caller that accumulates for its own reasons and would otherwise
  * accumulate twice - the acquisition task sums into a record and its
  * `accumulate` IS this chain's first stage, so it has the mean already
  * and at a precision an int sample would throw away. Skips stage one
  * and runs the biquads and the decimation.
  */
bool filter_push_value(const filter_design_t *design, filter_channel_t *ch,
                       float value, float *out);

#endif /* FILTER_H */
