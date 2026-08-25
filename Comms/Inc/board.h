/**
  ******************************************************************************
  * @file    board.h
  * @brief   Everything the comms stack needs from this board, and nothing more.
  *
  * The ADC helpers in main.c are static and stay static: they are the reporting
  * code's own business. This is the whole surface the command handlers may use.
  * The dependency runs one way - the comms stack asks, main.c answers.
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Physical quantity a channel can be converted to, 0 meaning none is defined. */
#define BOARD_UNIT_NONE      0U
#define BOARD_UNIT_MILLIVOLT 1U
#define BOARD_UNIT_CENTIDEGC 2U

typedef struct
{
  uint8_t     adc_index;      /**< 1, 2 or 3                          */
  uint8_t     channel;        /**< ADC channel number, decimal        */
  const char *pin;            /**< e.g. "PC3_C/PC2_C"                 */
  bool        differential;
  const char *signal;         /**< "" where the pin has no assignment */
  uint8_t     unit;           /**< BOARD_UNIT_*                       */
} board_chan_t;

const char *Board_Name(void);

/* ---- discrete I/O ------------------------------------------------------- */

/** Which way a pin's signal runs, seen from the MCU. */
#define BOARD_DIR_IN    0U
#define BOARD_DIR_OUT   1U
#define BOARD_DIR_INOUT 2U

/**
  * @brief One digital channel: a pin this board actually uses for something.
  *
  * `usable` is false for the pins raw access is refused on - the link and the
  * debug port. They are listed rather than hidden, because "PB10 is USART3_TX
  * and you may not drive it" is the answer a fixture needs; leaving them out
  * only means someone asks again with a pin write.
  */
typedef struct
{
  const char *pin;      /**< "PB2"                                  */
  uint8_t     dir;      /**< BOARD_DIR_*                            */
  const char *signal;   /**< what the pin carries on this board     */
  bool        usable;   /**< false where raw pin access is refused  */
} board_dchan_t;

uint8_t Board_DigitalCount(void);
bool    Board_DigitalChan(uint8_t index, board_dchan_t *info);

/**
  * @brief  Whether a fixture may drive this pin at all.
  *
  * The reserved list is the pin table, not a second list beside it: testrig.c
  * used to keep its own, and two lists of what PB10 is are one edit away from
  * disagreeing.
  */
bool Board_PinUsable(char port, uint8_t pin);

uint8_t Board_AdcCount(void);
bool    Board_AdcChan(uint8_t index, board_chan_t *info);

/**
  * @brief  Read one channel.
  * @param  microvolts  Voltage at the ADC pin. Not the sensed quantity for the
  *                     phase inputs, which sit behind unknown AFE gain.
  * @param  scaled      Physical quantity in the channel's unit, 0 when the
  *                     channel has no defined unit.
  */
bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled);

/* False from any of these four means no reading was taken - a bad index, or a
   conversion that did not complete. It is never a measurement of zero: on a
   differential channel code 0 is 0 V, so a failure reported as data would be
   indistinguishable from a signal. */

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w);
bool Board_DcBus(int32_t *raw, int32_t *millivolts);
bool Board_Ntc(int32_t *raw, int32_t *centidegc);

bool Board_AfeOn(void);
void Board_SetAfeOn(bool on);
bool Board_Pe15(void);

uint32_t Board_SysClkHz(void);
uint32_t Board_HclkHz(void);
uint8_t  Board_SysClkSource(void);   /**< 0 HSI, 1 CSI, 2 HSE, 3 PLL1, 4 other */
uint32_t Board_Cycles(void);

/** True when SYSCLK comes from the HSE crystal, directly or through PLL1. */
bool Board_SysClkOnCrystal(void);

/** Enable the cycle counter the comms stack uses as its timebase. */
void Board_TimebaseInit(void);

/** Per-channel result of a burst. Raw codes only: scaling is the host's job,
    so a different divider or thermistor needs no firmware change. Means and
    deviations are in milli-codes (raw x 1000) to keep fractions without
    putting a float on the wire. */
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

/** Most passes one burst may make. Named because the command handler checks
    it too: see h_adc_burst for why the limits live in both places. */
#define BOARD_BURST_MAX_SAMPLES 10000U

/**
  * @brief  Sample a set of channels repeatedly and return per-channel statistics.
  * @param  mask         Bit i selects channel i of the channel table.
  * @param  samples      1..10000 passes over the selected set.
  * @param  interval_us  Requested spacing between passes; 0 means as fast as
  *                      the conversions allow.
  * @param  out          At least Board_AdcCount() entries.
  * @param  count        Channels actually measured, in ascending index order.
  * @param  elapsed_us   Wall time the burst took, so the host can see the rate
  *                      it really got rather than the one it asked for.
  * @return False if the mask is empty, the count is out of range, the burst
  *         would exceed BOARD_BURST_MAX_US, or a conversion failed.
  */
bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us,
                    board_burst_t *out, uint8_t *count, uint32_t *elapsed_us);

/**
  * @brief  Sample one ADC back to back and return basic noise statistics.
  * @param  adc_index  1..3; the differential phase channel on that ADC.
  * @param  samples    1..1000.
  * @return False if either argument is out of range, or a conversion failed.
  */
bool Board_AdcNoise(uint8_t adc_index, uint16_t samples,
                    int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw,
                    uint32_t *span_raw, uint32_t *stddev_uv);

/* ---- self test ---------------------------------------------------------- */

/**
  * @brief One result from the board's self test.
  *
  * status is PASS or FAIL only where the board can genuinely PROVE the answer
  * from its own registers - a clock that is not locked, a calibration that never
  * ran, a checksum that does not match. Anything that would need a calibrated
  * instrument to judge is reported as INFO with its value, and the decision
  * belongs to whatever test executive is driving the line.
  *
  * That split is deliberate. This board is a dumb slave: it measures and
  * reports. It does not know what "good" is, and a limit compiled into firmware
  * is a limit nobody on the line can see or change.
  */
#define BOARD_CHECK_PASS 0U
#define BOARD_CHECK_FAIL 1U
#define BOARD_CHECK_INFO 2U

#define BOARD_SELFTEST_MAX 16U

typedef struct
{
  const char *name;
  uint8_t     status;
  int32_t     value;   /**< meaning is per check; 0 where there is none */
} board_check_t;

/**
  * @brief  Run every self check and fill @p out.
  * @return Number of checks written, never more than @p capacity.
  */
uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity);

/** Leave the binary link and resume the ASCII console, once the reply is out. */
void Board_RequestConsoleMode(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
