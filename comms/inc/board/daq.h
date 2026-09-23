/** board/daq.h - what the comms stack needs from board_daq.c; included by board.h. */
#ifndef COMMS_BOARD_DAQ_H
#define COMMS_BOARD_DAQ_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** As many channels as the ADC table has rows. */
#define BOARD_DAQ_MAX_CHANNELS 10U

#define BOARD_DAQ_CLOCK_SOFTWARE 0U  /**< the main loop, as fast as it gets round */
#define BOARD_DAQ_CLOCK_TIM1     1U  /**< the injected group, one per PWM period  */

/** The sensor fields a record may append - SNAPSHOTS, never sums: a summed
    quaternion means nothing. */
#define BOARD_DAQ_SENSOR_ORIENTATION (1U << 0)  /**< i, j, k, real - Q14  */
#define BOARD_DAQ_SENSOR_ACCEL       (1U << 1)  /**< x, y, z, status - Q8 */
#define BOARD_DAQ_SENSOR_GYRO        (1U << 2)  /**< x, y, z, status - Q9 */
#define BOARD_DAQ_SENSOR_MAG         (1U << 3)  /**< x, y, z, status - Q4 */
#define BOARD_DAQ_SENSOR_SHAFT       (1U << 4)  /**< value, crc, reg, have */
#define BOARD_DAQ_MAX_SENSORS 5U

/** What a task is. Every field is the caller's; nothing is inferred. */
typedef struct
{
  uint16_t channels;     /**< bitmask over the ADC table's rows. */
  uint8_t  clock;        /**< BOARD_DAQ_CLOCK_*                          */
  uint8_t  sample_time;  /**< 0..7, the converter's own sampling window  */
  uint16_t decimate;     /**< keep one trigger in N; 1 keeps every one   */
  uint16_t accumulate;   /**< sum N samples per record; 1 sums nothing, 0 closes the record on
      interval_us instead */
  uint32_t records;      /**< stop after this many, or 0 to run on       */
  uint8_t  digital;      /**< append the digital pins to every record    */
  uint32_t interval_us;  /**< software clock: minimum gap between samples*/
  uint8_t  adapt;        /**< climb the ladder when the ring fills      */
  uint16_t sensors;      /**< BOARD_DAQ_SENSOR_* mask; software clock only - the poll records are
      the main loop's, and a TIM1-clocked record closes in ADC3's interrupt,
      which would read them torn */
} board_daq_config_t;

typedef struct
{
  bool     running;
  bool     done;         /**< a finite task reached its record count     */
  bool     lost_power;   /**< stopped because AFE_ON went off, and the buffers were emptied with it
      - invariant 9 */
  uint16_t stride;       /**< bytes per record: 4 (stamp) + 4 per channel + 1 per sampled pin + 8
      per sensor field + 2 (count). */
  uint8_t  fields;
  uint32_t available;    /**< whole records waiting to be taken          */
  uint32_t produced;
  uint32_t dropped;      /**< records the buffer had no room for         */
  /* THE BUFFER LEVEL, and it takes both numbers to be one: `available` alone
     is a count nobody can read as full or empty without knowing what the
     ring holds at THIS stride, which changes with the channel count. */
  uint32_t capacity;     /**< whole records the ring holds at `stride`   */
  uint32_t worst;        /**< the fullest it has been since the start    */
  uint8_t  rung;         /**< which rung of the ladder is running        */
  uint8_t  rungs;        /**< how many the host sent                     */
  uint32_t rung_changes; /**< how often it has climbed or fallen         */
  /* SWEEPS, not records: what the acquisition loop is actually managing
     underneath the decimation. */
  uint32_t triggers;
  board_daq_config_t config;
} board_daq_state_t;

/** The always-available accumulator: every trigger adds to it and a read
    takes it away. */
typedef struct
{
  int32_t  sum;
  uint32_t additions;                  /**< how many went into this one     */
  int32_t  lowest;                     /**< what the channel did in the     */
  int32_t  highest;                    /**< window, not inferred from a mean*/
} board_daq_slot_t;

typedef struct
{
  bool     fresh;                      /**< anything arrived since the last */
  uint32_t first;                      /**< Board_Cycles(), raw ticks       */
  uint32_t last;
  uint32_t digital;                    /**< pins at `last`                  */
  board_daq_slot_t slot[BOARD_DAQ_MAX_CHANNELS];
} board_daq_live_t;

/** Copy the accumulator out and reset it. */
void Board_DaqTakeLive(board_daq_live_t *out);

/** NULL when it took, and the reason in the board's own words when it did
    not. */
const char *Board_DaqConfigure(const board_daq_config_t *cfg);
/** Override the software clock's interval after configuring. */
void Board_DaqSetInterval(uint32_t interval_us);

const char *Board_DaqStart(void);
void Board_DaqStop(void);
void Board_DaqState(board_daq_state_t *out);

/** Load the anti-alias chain the host designed, or clear it.
    @return NULL, or the board's own words for what is wrong. */
const char *Board_DaqSetFilter(const void *sections, uint8_t count,
                               uint16_t decimate);

/** One rung of the ladder: a whole design and its boxcar. */
const char *Board_DaqSetRung(uint8_t rung, uint16_t boxcar,
                             const void *sections, uint8_t count,
                             uint16_t decimate);

/** A known tone in place of the converter, for proving the path. */
/** What the generator makes. */
#define BOARD_DAQ_TONE_SINE 0U
#define BOARD_DAQ_TONE_RAMP 1U

const char *Board_DaqSetTone(uint32_t hz, uint32_t rate_hz,
                             int32_t amplitude, int32_t offset,
                             uint8_t kind);

/** Advance the tone generator. */
void Board_DaqTonePoll(void);

/** Whether a tone is standing in for the converter. */
bool Board_DaqToneOn(void);

/** Triggers one record costs: the decimation, the accumulate AND the
    filter's own decimation. */
uint32_t Board_DaqTriggersPerRecord(void);

/** Whether the task asked for no rate and had one chosen for it. */
bool Board_DaqRateIsAuto(void);

/** Which channel field `n` of a record carries. */
bool Board_DaqField(uint8_t field, uint8_t *channel);

/** Advanced by the main loop for a software-clocked task. */
void Board_DaqPoll(void);

/** Fed from the injected end-of-sequence for a TIM1-clocked one. */
void Board_DaqOnInjected(const board_sync_sample_t *sample);

uint32_t Board_DaqAvailable(void);
uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_DAQ_H */
