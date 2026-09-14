/**
  ******************************************************************************
  * @file    harness.c
  * @brief   A flat C API over daq/, so test_daq_core.py can run the
  *          acquisition engine on the host through ctypes.
  *
  * Built by the Python suite with the host gcc, never by the firmware build.
  * Test scaffolding; it must not appear in the root CMakeLists.
  *
  * One engine over a static ring, a snapshot that answers with whatever
  * words the test last set, and a guard that counts how often it was
  * taken - so a test can say the record path and the live path hold the
  * interrupt, without there being one. NOTHING CROSSES AS A STRUCT: the
  * live accumulator comes back as four flat arrays and the span, a record
  * as the bytes the wire would carry, which the test decodes exactly as a
  * host does.
  ******************************************************************************
  */
#include "daq.h"

#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

#define RING_MAX 65536U

static daq_t   s_d;
static uint8_t s_buf[RING_MAX];
static int16_t s_words[DAQ_MAX_SENSORS][DAQ_SENSOR_WORDS];
static int     s_holds;

static uint32_t hold(void)
{
  s_holds++;
  return 1U;
}

static void release(uint32_t masked)
{
  (void)masked;
}

static void snapshot(void *ctx, uint16_t sensors,
                     int16_t words[DAQ_MAX_SENSORS][DAQ_SENSOR_WORDS])
{
  (void)ctx;
  (void)sensors;
  memcpy(words, s_words, sizeof(s_words));
}

/* ---- set-up ------------------------------------------------------------- */

API void daq_h_init(uint32_t bytes)
{
  static const daq_guard_t guard = { hold, release };

  daq_init(&s_d, s_buf, (bytes > RING_MAX) ? RING_MAX : bytes, &guard,
           snapshot, NULL);
  memset(s_words, 0, sizeof(s_words));
  s_holds = 0;
}

API void daq_h_ladder(uint32_t climb_at, uint32_t fall_at, uint32_t eighths,
                      uint32_t climb_max, uint32_t fall_after)
{
  const daq_ladder_t ladder = { climb_at, fall_at, eighths, climb_max,
                                fall_after };

  daq_set_ladder(&s_d, &ladder);
}

API void daq_h_begin(uint8_t fields, uint16_t accumulate, uint16_t decimate,
                     uint32_t interval_cycles, uint8_t pins, uint16_t sensors,
                     uint32_t records, int adapt)
{
  daq_task_t task;

  memset(&task, 0, sizeof(task));
  task.fields = fields;
  for (uint8_t f = 0U; f < DAQ_MAX_CHANNELS; f++)
  {
    task.order[f] = f;
  }
  task.accumulate = accumulate;
  task.decimate = decimate;
  task.interval_cycles = interval_cycles;
  task.pins = pins;
  task.sensors = sensors;
  task.records = records;
  task.adapt = (adapt != 0);
  daq_begin(&s_d, &task);
}

API int  daq_h_start(uint32_t now)  { return daq_start(&s_d, now) ? 1 : 0; }
API void daq_h_stop(void)           { daq_stop(&s_d); }
API void daq_h_lose_power(void)     { daq_lose_power(&s_d); }
API void daq_h_words(const int16_t *words)
{
  memcpy(s_words, words, sizeof(s_words));
}

/* ---- samples in --------------------------------------------------------- */

API void daq_h_feed(const int32_t *values, uint32_t at, uint32_t digital)
{
  daq_feed(&s_d, values, at, digital);
}

API int  daq_h_trigger_due(uint32_t now) { return daq_trigger_due(&s_d, now) ? 1 : 0; }
API void daq_h_sweep_begin(uint32_t at, uint32_t digital) { daq_sweep_begin(&s_d, at, digital); }
API int  daq_h_sweep_put(int32_t raw)  { return daq_sweep_put(&s_d, raw) ? 1 : 0; }
API void daq_h_sweep_close(uint32_t now) { daq_sweep_close(&s_d, now); }

API void daq_h_live_insert(uint8_t field, int32_t value, uint32_t at,
                           uint32_t digital)
{
  daq_live_insert(&s_d, field, value, at, digital);
}

/* LIVE_ORDER: sum, additions, lowest, highest per field; span is first,
   last, digital. Returns `fresh`. */
API int daq_h_take_live(int32_t *sum, uint32_t *additions, int32_t *lowest,
                        int32_t *highest, uint32_t *span)
{
  daq_live_t live;

  daq_take_live(&s_d, &live);
  for (uint8_t f = 0U; f < DAQ_MAX_CHANNELS; f++)
  {
    sum[f] = live.slot[f].sum;
    additions[f] = live.slot[f].additions;
    lowest[f] = live.slot[f].lowest;
    highest[f] = live.slot[f].highest;
  }
  span[0] = live.first;
  span[1] = live.last;
  span[2] = live.digital;
  return live.fresh ? 1 : 0;
}

/* ---- the chain, the ladder and the generator ---------------------------- */

/* COEFF_ORDER per section: b0, b1, b2, a1, a2. */
static void sections_of(const float *coeffs, uint8_t count,
                        filter_biquad_t out[FILTER_MAX_SECTIONS])
{
  for (uint8_t s = 0U; (s < count) && (s < FILTER_MAX_SECTIONS); s++)
  {
    out[s].b0 = coeffs[(s * 5U) + 0U];
    out[s].b1 = coeffs[(s * 5U) + 1U];
    out[s].b2 = coeffs[(s * 5U) + 2U];
    out[s].a1 = coeffs[(s * 5U) + 3U];
    out[s].a2 = coeffs[(s * 5U) + 4U];
  }
}

API const char *daq_h_set_filter(const float *coeffs, uint8_t count,
                                 uint16_t decimate)
{
  filter_biquad_t sections[FILTER_MAX_SECTIONS] = {{0}};

  sections_of(coeffs, count, sections);
  return daq_set_filter(&s_d, sections, count, decimate);
}

API const char *daq_h_set_rung(uint8_t rung, uint16_t boxcar,
                               const float *coeffs, uint8_t count,
                               uint16_t decimate)
{
  filter_biquad_t sections[FILTER_MAX_SECTIONS] = {{0}};

  sections_of(coeffs, count, sections);
  return daq_set_rung(&s_d, rung, boxcar, sections, count, decimate);
}

API const char *daq_h_set_tone(uint32_t hz, uint32_t rate_hz, int32_t amplitude,
                               int32_t offset, uint8_t kind,
                               uint32_t cycles_per_second, uint32_t now)
{
  return daq_set_tone(&s_d, hz, rate_hz, amplitude, offset, kind,
                      cycles_per_second, now);
}

API void daq_h_tone_poll(uint32_t now, uint32_t burst)
{
  daq_tone_poll(&s_d, now, burst);
}

/* ---- records out and the counters --------------------------------------- */

API uint16_t daq_h_take(uint8_t *out, uint16_t max_records)
{
  return daq_take(&s_d, out, max_records);
}

API uint32_t daq_h_available(void)     { return daq_available(&s_d); }
API uint32_t daq_h_capacity(void)      { return daq_capacity(&s_d); }
API uint32_t daq_h_dropped(void)       { return s_d.dropped; }
API uint32_t daq_h_produced(void)      { return s_d.produced; }
API uint32_t daq_h_worst(void)         { return s_d.worst; }
API uint32_t daq_h_triggers(void)      { return s_d.triggers; }
API uint32_t daq_h_rung(void)          { return s_d.rung; }
API uint32_t daq_h_rung_changes(void)  { return s_d.rung_changes; }
API uint32_t daq_h_accumulate(void)    { return s_d.task.accumulate; }
API uint32_t daq_h_stride(void)        { return s_d.stride; }
API int      daq_h_filtering(void)     { return s_d.filtering ? 1 : 0; }
API int      daq_h_running(void)       { return s_d.running ? 1 : 0; }
API int      daq_h_done(void)          { return s_d.done ? 1 : 0; }
API int      daq_h_tone_on(void)       { return s_d.tone.on ? 1 : 0; }
API int      daq_h_holds(void)         { return s_holds; }
API uint32_t daq_h_triggers_per_record(void) { return daq_triggers_per_record(&s_d); }
API uint32_t daq_h_max_channels(void)  { return DAQ_MAX_CHANNELS; }
API uint32_t daq_h_max_sensors(void)   { return DAQ_MAX_SENSORS; }
API uint32_t daq_h_max_additions(void) { return DAQ_MAX_ADDITIONS; }
API uint32_t daq_h_record_max(void)    { return DAQ_RECORD_MAX; }
