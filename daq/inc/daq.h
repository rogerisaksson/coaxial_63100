/**
  ******************************************************************************
  * @file    daq.h
  * @brief   The acquisition engine, hardware-free: a byte ring of records,
  *          the summing window that makes them, the anti-alias chain and
  *          the ladder of designs it climbs when the ring fills, the tone
  *          generator that stands in for the converter, and the live
  *          accumulator a host reads at its leisure.
  ******************************************************************************
  */
#ifndef DAQ_H
#define DAQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "filter.h"

#define DAQ_MAX_CHANNELS 10U   /**< fields a record can carry - board.h's count */
#define DAQ_MAX_PINS     16U   /**< digital pins a record can carry a duty for */
#define DAQ_MAX_SENSORS  5U    /**< sensor snapshot fields a record can carry  */
#define DAQ_LADDER       4U    /**< rungs of designs a task can climb          */
#define DAQ_SENSOR_WORDS 4U    /**< a sensor field is four 16-bit words        */

/** Most additions a summing path takes before it stops widening: INT32_MAX /
    65535, the largest a single-ended code can be, so one more can never
    carry an int32 sum past the end. */
#define DAQ_MAX_ADDITIONS 32767U

#define DAQ_TONE_SINE 0U
#define DAQ_TONE_RAMP 1U

/** A record on the wire: the start time, a 32-bit sum a field, a byte a
    sampled pin, four 16-bit words a sensor, and the count last. */
#define DAQ_RECORD_BYTES(fields, pins, sensors) \
  (4U + (4U * (fields)) + (pins) + (8U * (sensors)) + 2U)
#define DAQ_RECORD_MAX \
  DAQ_RECORD_BYTES(DAQ_MAX_CHANNELS, DAQ_MAX_PINS, DAQ_MAX_SENSORS)

/** One task, as the engine needs it: the glue has already checked it against
    the converter and turned the wire's microseconds into cycles. */
typedef struct
{
  uint8_t  fields;                    /**< how many of `order` are in use    */
  uint8_t  order[DAQ_MAX_CHANNELS];   /**< channel index per field           */
  uint16_t accumulate;                /**< sum N a record; 0 closes by clock */
  uint16_t decimate;                  /**< keep one trigger in N             */
  uint32_t interval_cycles;           /**< the count's gate, or the clock's  */
  uint8_t  pins;                      /**< digital pins carried, 0 for none  */
  uint16_t sensors;                   /**< sensor field mask                 */
  uint32_t records;                   /**< stop after this many, 0 to run on */
  bool     adapt;                     /**< climb the ladder as the ring fills*/
} daq_task_t;

/** Where the ring is when a task climbs and when it comes back down, in
    `eighths` of capacity, the climb mark's ceiling in records, and how many
    polls at the low mark before it steps down. */
typedef struct
{
  uint32_t climb_at;
  uint32_t fall_at;
  uint32_t eighths;
  uint32_t climb_max;
  uint32_t fall_after;
} daq_ladder_t;

/** The interrupt hold around what a feed on the timer's interrupt and a take
    on the main loop both touch. */
typedef struct
{
  uint32_t (*hold)(void);
  void (*release)(uint32_t masked);
} daq_guard_t;

/** The sensor snapshot at a record's close: every field the mask names, four
    words each, read once for the whole record so they are one instant. */
typedef void (*daq_snapshot_fn)(void *ctx, uint16_t sensors,
                                int16_t words[DAQ_MAX_SENSORS][DAQ_SENSOR_WORDS]);

/** The always-available accumulator, one slot a field: every sample adds to
    it and a take empties it. */
typedef struct
{
  int32_t  sum;
  uint32_t additions;
  int32_t  lowest;
  int32_t  highest;
} daq_slot_t;

typedef struct
{
  bool       fresh;
  uint32_t   first;
  uint32_t   last;
  uint32_t   digital;
  daq_slot_t slot[DAQ_MAX_CHANNELS];
} daq_live_t;

/** The generator: a rotating unit vector for a sine, integer arithmetic for
    a ramp, and the samples the elapsed cycles owe. */
typedef struct
{
  bool     on;
  uint8_t  kind;
  uint32_t step;        /**< the ramp's increment a sample                 */
  int32_t  mod;         /**< and what it counts up to                      */
  float    cos_step;    /**< per-sample rotation                           */
  float    sin_step;
  float    x;           /**< the rotating unit vector                      */
  float    y;
  float    amp;
  float    offset;
  uint32_t cycles;      /**< cycles per tone sample                        */
  uint32_t at;          /**< when the last sample was made                 */
  uint32_t owed;        /**< fractional cycles carried over                */
  uint32_t n;
} daq_tone_t;

/** The engine. */
typedef struct
{
  /* the ring of records */
  uint8_t          *buf;
  uint32_t          bytes;
  volatile uint32_t head;          /**< byte offset of the next write      */
  volatile uint32_t tail;          /**< byte offset of the next read       */
  volatile uint32_t dropped;
  volatile uint32_t produced;
  volatile uint32_t worst;         /**< the fullest it has been, in records*/
  volatile uint32_t triggers;      /**< sweeps kept after decimation       */
  daq_guard_t       guard;
  daq_snapshot_fn   snapshot;
  void             *ctx;

  /* the task */
  daq_task_t        task;
  uint16_t          stride;
  volatile bool     running;
  volatile bool     done;

  /* the window */
  int32_t           acc[DAQ_MAX_CHANNELS];
  uint16_t          acc_n;
  uint16_t          skip;
  uint32_t          first_at;
  uint16_t          dacc[DAQ_MAX_PINS];
  uint32_t          first_digital;
  uint32_t          last_trigger;

  /* the chain and the ladder */
  filter_design_t   chain;
  filter_channel_t  filter[DAQ_MAX_CHANNELS];
  bool              filtering;
  filter_design_t   rungs[DAQ_LADDER];
  uint16_t          rung_boxcar[DAQ_LADDER];
  uint8_t           rungs_held;
  uint8_t           rung;
  uint32_t          rung_changes;
  uint32_t          low_for;
  daq_ladder_t      ladder;

  /* the generator */
  daq_tone_t        tone;

  /* the live accumulator */
  daq_slot_t        live[DAQ_MAX_CHANNELS];
  uint8_t           live_any;
  uint32_t          live_first;
  uint32_t          live_last;
  uint32_t          live_digital;

  /* the sweep in progress, one field a turn */
  uint8_t           next_field;
  int32_t           pending[DAQ_MAX_CHANNELS];
  uint32_t          pending_at;
  uint32_t          pending_digital;
} daq_t;

/* ---- set-up ------------------------------------------------------------ */

/** Everything zero, the ring over `buf`, the guard and the snapshot the
    glue's (`guard` NULL: no interrupts to hold). */
void daq_init(daq_t *d, uint8_t *buf, uint32_t bytes, const daq_guard_t *guard,
              daq_snapshot_fn snapshot, void *ctx);

/** The ladder's marks. */
void daq_set_ladder(daq_t *d, const daq_ladder_t *ladder);

/** A task that passed the glue's checks becomes the engine's: the stride,
    the buffers empty, the ladder at the bottom. */
void daq_begin(daq_t *d, const daq_task_t *task);

/** The counters and the window back to nothing, and the task running. */
bool daq_start(daq_t *d, uint32_t now);
void daq_stop(daq_t *d);

/** What the glue does when the converter's reference went away: the task
    stops and every buffer empties, because an accumulator holding half real
    samples and half mid-scale divides out to something plausible. */
void daq_lose_power(daq_t *d);

/* ---- the chain, the ladder and the generator --------------------------- */

/** NULL, or the reason in the board's own words. */
const char *daq_set_filter(daq_t *d, const filter_biquad_t *sections,
                           uint8_t count, uint16_t decimate);
const char *daq_set_rung(daq_t *d, uint8_t rung, uint16_t boxcar,
                         const filter_biquad_t *sections, uint8_t count,
                         uint16_t decimate);
/** `cycles_per_second` is the clock the cycle counts are in. */
const char *daq_set_tone(daq_t *d, uint32_t hz, uint32_t rate_hz,
                         int32_t amplitude, int32_t offset, uint8_t kind,
                         uint32_t cycles_per_second, uint32_t now);
/** The samples the elapsed cycles owe, at most `burst`, each fed as a sweep. */
void daq_tone_poll(daq_t *d, uint32_t now, uint32_t burst);

/** Triggers a record costs: the accumulate, the decimation and the chain's. */
uint32_t daq_triggers_per_record(const daq_t *d);

/* ---- samples in -------------------------------------------------------- */

/** One trigger's worth of samples, already read. Accumulates and may push. */
void daq_feed(daq_t *d, const int32_t *values, uint32_t at, uint32_t digital);

/** Closed by a count: the interval gates the triggers. */
bool daq_trigger_due(daq_t *d, uint32_t now);

/** A sweep read one field a turn: begun at `at` with the pins as they stood,
    each field put in turn - true when the last one landed - and closed
    through the count's gate into the feed. */
void daq_sweep_begin(daq_t *d, uint32_t at, uint32_t digital);
bool daq_sweep_put(daq_t *d, int32_t raw);
void daq_sweep_close(daq_t *d, uint32_t now);

/** One sample into the live accumulator, for one field. */
void daq_live_insert(daq_t *d, uint8_t field, int32_t value, uint32_t at,
                     uint32_t digital);

/* ---- records out ------------------------------------------------------- */

uint32_t daq_available(const daq_t *d);   /**< whole records waiting      */
uint32_t daq_capacity(const daq_t *d);    /**< whole records the ring holds*/
uint16_t daq_take(daq_t *d, uint8_t *out, uint16_t max_records);
/** The live accumulator copied out and emptied. */
void daq_take_live(daq_t *d, daq_live_t *out);

#endif /* DAQ_H */
