/** board/adc.h - what the comms stack needs from board_adc.c; included by board.h. */
#ifndef COMMS_BOARD_ADC_H
#define COMMS_BOARD_ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "board/sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Physical quantity a channel can be converted to, 0 meaning none is defined. */
#define BOARD_UNIT_NONE      0U
#define BOARD_UNIT_MILLIVOLT 1U
#define BOARD_UNIT_CENTIDEGC 2U
#define BOARD_UNIT_MILLIAMP  3U

typedef struct
{
  uint8_t     adc_index;      /**< 1, 2 or 3                          */
  uint8_t     channel;        /**< ADC channel number, decimal        */
  const char *pin;            /**< e.g. "PC3_C/PC2_C"                 */
  bool        differential;
  const char *signal;         /**< "" where the pin has no assignment */
  uint8_t     unit;           /**< BOARD_UNIT_*                       */
} board_chan_t;

/** A differential code as the converter gives it: offset binary, 32768 is 0
    V. */
int32_t Board_AdcDifferential(uint32_t raw);

/** ADC sampling time, as an index 0..7 into the H7's eight, shortest first. */
bool    Board_AdcSetSampleTime(uint8_t index);
uint8_t Board_AdcSampleTime(void);

/** Is this channel one the injected group converts? Only those three can be
    clocked from TIM1; everything else has to come through the meter. */
bool    Board_AdcIsPhase(uint8_t index);

/** Whether the injected sequence converts this channel at all - the three
    phases and, on rank 2, the DC link and the NTC. */
bool    Board_AdcInjected(uint8_t index);

/** One channel's value out of a latched injected sample. */
int32_t Board_AdcInjectedSlot(uint8_t index,
                              const board_sync_sample_t *sample);
int32_t Board_AdcPhaseSlot(uint8_t index, const int16_t *phase);

uint8_t Board_AdcCount(void);
bool    Board_AdcChan(uint8_t index, board_chan_t *info);

/** Read one channel.
    @param  microvolts  Voltage at the ADC pin. Not the sensed quantity for the
    @param  scaled      Physical quantity in the channel's unit, 0 when the */
bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled);

/* False from any of these four means no reading was taken - a bad index, or
   a conversion that did not complete. */

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w);
bool Board_DcBus(int32_t *raw, int32_t *millivolts);
bool Board_Ntc(int32_t *raw, int32_t *centidegc);

/** The MCU die, centi-degrees C. */
bool Board_McuDie(int32_t *raw, int32_t *centidegc);

/** Amperes from a centred phase code - what Board_AdcDifferential returns. */
float Board_PhaseAmps(uint8_t leg, int32_t centred);

/** The affine form of the two conversions the drive needs at 50 kHz:
    quantity = (code - offset) * per_code, with the record's trim folded into
    the factor. */
void Board_PhaseScale(uint8_t leg, int32_t *offset_raw,
                      float *amps_per_code);
void Board_DcBusScale(int32_t *offset_raw, float *volts_per_code);

/** What the converters are actually clocked at, after the prescaler. */
uint32_t Board_AdcClockHz(void);
uint8_t  Board_SysClkSource(void);   /**< 0 HSI, 1 CSI, 2 HSE, 3 PLL1, 4 other */
uint32_t Board_Cycles(void);

/** True when SYSCLK comes from the HSE crystal, directly or through PLL1. */
bool Board_SysClkOnCrystal(void);

/** Per-channel result of a burst. */
typedef struct
{
  uint8_t  index;
  int32_t  mean_milliraw;
  int32_t  min_raw;
  int32_t  max_raw;
  uint32_t sd_milliraw;
} board_burst_t;

/** Longest burst the firmware will accept, so a request cannot outlive the
    master's patience or wedge the link. */
#define BOARD_BURST_MAX_US 5000000UL

/** Most passes one burst may make. */
#define BOARD_BURST_MAX_SAMPLES 10000U

/** Channels one burst can cover, and the size every caller's `out` array
    must have. */
#define BOARD_BURST_MAX_CHAN 16U

/** The most samples one burst takes: the reply is a summary, so this bounds
    the time the link waits, not a buffer. */
#define BOARD_ADC_BURST_MAX 1000U

/** Sample a set of channels repeatedly and return per-channel statistics.
    @param  mask         Bit i selects channel i of the channel table.
    @param  samples      1..10000 passes over the selected set.
    @param  interval_us  Requested spacing between passes; 0 means as fast as
    @param  out          At least Board_AdcCount() entries.
    @param  count        Channels actually measured, in ascending index order.
    @param  elapsed_us   Wall time the burst took, so the host can see the rate
    @return False if the mask is empty, the count is out of range, the burst */
bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us,
                    board_burst_t *out, uint8_t *count, uint32_t *elapsed_us);

/** Sample one ADC back to back and return basic noise statistics.
    @param  adc_index  1..3; the differential phase channel on that ADC.
    @param  samples    1..1000.
    @return False if either argument is out of range, or a conversion failed. */
bool Board_AdcNoise(uint8_t adc_index, uint16_t samples,
                    int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw,
                    uint32_t *span_raw, uint32_t *stddev_uv);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_ADC_H */
