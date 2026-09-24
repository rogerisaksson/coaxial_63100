/** daq.c - The acquisition engine - see daq.h. */
#include "daq.h"

#include <math.h>
#include <string.h>

/* The rotating unit vector drifts off length by rounding: every
   TONE_RENORM_MASK + 1 samples it is put back, unless it has collapsed. */
#define TONE_RENORM_MASK 1023U
#define TONE_TINY        1e-6f
#define TWO_PI           6.28318530717958647692f
#define DUTY_FULL        255U      /* a pin high for the whole window */
#define IMU_SENSORS      0x0FU     /* the four sensor bits the IMU serves */

/* ---- the guard ---------------------------------------------------------- */

static uint32_t held(const daq_t *d)
{
  return (d->guard.hold != NULL) ? d->guard.hold() : 0U;
}

static void released(const daq_t *d, uint32_t masked)
{
  if (d->guard.release != NULL)
  {
    d->guard.release(masked);
  }
}

/* ---- set-up ------------------------------------------------------------- */

void daq_init(daq_t *d, uint8_t *buf, uint32_t bytes, const daq_guard_t *guard,
              daq_snapshot_fn snapshot, void *ctx)
{
  memset(d, 0, sizeof(*d));
  d->buf = buf;
  d->bytes = bytes;
  if (guard != NULL)
  {
    d->guard = *guard;
  }
  d->snapshot = snapshot;
  d->ctx = ctx;
  d->tone.x = 1.0f;
}

void daq_set_ladder(daq_t *d, const daq_ladder_t *ladder)
{
  d->ladder = *ladder;
}

/* ---- the ring ----------------------------------------------------------- */

static uint32_t room(const daq_t *d)
{
  const uint32_t head = d->head;
  const uint32_t tail = d->tail;

  return (head >= tail) ? (d->bytes - head + tail) : (tail - head);
}

uint32_t daq_available(const daq_t *d)
{
  const uint32_t head = d->head;
  const uint32_t tail = d->tail;
  const uint32_t used = (head >= tail) ? (head - tail)
                                       : (d->bytes - tail + head);

  return (d->stride != 0U) ? (used / d->stride) : 0U;
}

uint32_t daq_capacity(const daq_t *d)
{
  return (d->stride != 0U) ? (d->bytes / d->stride) : 0U;
}

/* Wraps at the end of the buffer, not at a record boundary: the stride
   divides nothing in particular and rounding the buffer down to whole
   records for every possible stride wastes more than it saves. */
static void put(daq_t *d, const uint8_t *src, uint16_t len)
{
  const uint32_t whole = len;
  const uint32_t to_end = d->bytes - d->head;
  const uint32_t first = (whole < to_end) ? whole : to_end;

  memcpy(d->buf + d->head, src, first);
  memcpy(d->buf, src + first, whole - first);
  d->head = (d->head + whole) % d->bytes;
}

/* The mirror of put: a record out of the ring, in at most two runs. */
static void get(const daq_t *d, uint32_t from, uint8_t *dst, uint16_t len)
{
  const uint32_t whole = len;
  const uint32_t to_end = d->bytes - from;
  const uint32_t first = (whole < to_end) ? whole : to_end;

  memcpy(dst, d->buf + from, first);
  memcpy(dst + first, d->buf, whole - first);
}

/* Big endian, like every other u32 this board puts on the wire. */
static uint16_t put_be32(uint8_t *dst, uint16_t at, uint32_t v)
{
  dst[at++] = (uint8_t)((v >> 24) & 0xFFU);
  dst[at++] = (uint8_t)((v >> 16) & 0xFFU);
  dst[at++] = (uint8_t)((v >> 8) & 0xFFU);
  dst[at++] = (uint8_t)(v & 0xFFU);
  return at;
}

static uint16_t put_be16(uint8_t *dst, uint16_t at, uint16_t v)
{
  dst[at++] = (uint8_t)((v >> 8) & 0xFFU);
  dst[at++] = (uint8_t)(v & 0xFFU);
  return at;
}

uint16_t daq_take(daq_t *d, uint8_t *out, uint16_t max_records)
{
  uint16_t taken = 0U;

  if ((out == NULL) || (d->stride == 0U))
  {
    return 0U;
  }
  while ((taken < max_records) && (daq_available(d) > 0U))
  {
    get(d, d->tail, out + ((uint32_t)taken * d->stride), d->stride);
    d->tail = (d->tail + d->stride) % d->bytes;
    taken++;
  }
  return taken;
}

/* ---- the window --------------------------------------------------------- */

/** The accumulators back to empty: the next window starts here. */
static void clear_window(daq_t *d)
{
  memset(d->acc, 0, sizeof(d->acc));
  memset(d->dacc, 0, sizeof(d->dacc));
  d->acc_n = 0U;
}

static uint8_t sensor_count(uint16_t mask)
{
  uint8_t n = 0U;

  for (uint8_t b = 0U; b < DAQ_MAX_SENSORS; b++)
  {
    n = (uint8_t)(n + ((mask >> b) & 1U));
  }
  return n;
}

/* ---- the ladder --------------------------------------------------------- */

/** Take rung `n`: its whole design, and the accumulate that goes with it. */
static void take_rung(daq_t *d, uint8_t n)
{
  if ((n >= d->rungs_held) || (n == d->rung))
  {
    return;
  }
  d->rung = n;
  d->chain = d->rungs[n];
  d->task.accumulate = d->rung_boxcar[n];
  d->filtering = (d->chain.sections > 0U) || (d->chain.decimate > 1U);
  d->rung_changes++;
  d->low_for = 0U;

  /* Primed, not zeroed. */
  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    filter_prime(&d->chain, &d->filter[f], (float)d->pending[f]);
  }
  clear_window(d);
}

/** One record's worth of pressure on the ring, and what it costs. */
static void ladder_step(daq_t *d)
{
  if ((d->rungs_held < 2U) || !d->task.adapt || (d->ladder.eighths == 0U))
  {
    return;
  }

  const uint32_t capacity = daq_capacity(d);

  if (capacity == 0U)
  {
    return;
  }

  const uint32_t held_records = daq_available(d);
  /* The lower of the two: a fraction of a small ring, a fixed backlog of a
     large one. */
  uint32_t climb = (capacity * d->ladder.climb_at) / d->ladder.eighths;
  uint32_t fall = (capacity * d->ladder.fall_at) / d->ladder.eighths;

  if (climb > d->ladder.climb_max)
  {
    climb = d->ladder.climb_max;
  }
  if (fall > (d->ladder.climb_max / d->ladder.eighths))
  {
    fall = d->ladder.climb_max / d->ladder.eighths;
  }

  if (held_records >= climb)
  {
    take_rung(d, (uint8_t)(d->rung + 1U));
    return;
  }
  if (held_records > fall)
  {
    d->low_for = 0U;
    return;
  }
  if (++d->low_for < d->ladder.fall_after)
  {
    return;
  }
  d->low_for = 0U;
  if (d->rung > 0U)
  {
    take_rung(d, (uint8_t)(d->rung - 1U));
  }
}

/* ---- records ------------------------------------------------------------ */

/* The record into the ring, or counted as dropped when it does not fit - and
   the high-water mark of what the host has yet to take. */
static void put_record(daq_t *d, const uint8_t *rec)
{
  if (room(d) <= d->stride)
  {
    d->dropped++;
    return;
  }
  put(d, rec, d->stride);
  d->produced++;

  const uint32_t waiting = daq_available(d);

  if (waiting > d->worst)
  {
    d->worst = waiting;
  }
}

static void push_record(daq_t *d)
{
  uint8_t rec[DAQ_RECORD_MAX];
  uint16_t at = put_be32(rec, 0U, d->first_at);

  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    at = put_be32(rec, at, (uint32_t)d->acc[f]);
  }

  /* 0..255 of the window, so a host divides by 255 and gets the fraction of
     it the pin was high for. */
  const uint32_t n = (d->acc_n > 0U) ? d->acc_n : 1U;

  for (uint8_t p = 0U; (p < d->task.pins) && (p < DAQ_MAX_PINS); p++)
  {
    rec[at++] = (uint8_t)(((uint32_t)d->dacc[p] * DUTY_FULL + (n / 2U)) / n);
  }

  /* The sensor snapshots, after the pins and before the count: an appended
     field, seen only by a host that asked for it, read once so the fields
     are one instant. */
  int16_t words[DAQ_MAX_SENSORS][DAQ_SENSOR_WORDS] = {{0}};

  if ((d->task.sensors != 0U) && (d->snapshot != NULL))
  {
    d->snapshot(d->ctx, d->task.sensors, words);
  }
  for (uint8_t b = 0U; b < DAQ_MAX_SENSORS; b++)
  {
    if ((d->task.sensors & (1U << b)) == 0U)
    {
      continue;
    }
    for (uint8_t w = 0U; w < DAQ_SENSOR_WORDS; w++)
    {
      at = put_be16(rec, at, (uint16_t)words[b][w]);
    }
  }

  /* The divisor travels with the sum. */
  at = put_be16(rec, at, d->acc_n);

  const uint32_t masked = held(d);

  put_record(d, rec);
  released(d, masked);

  clear_window(d);

  if ((d->task.records != 0U) && (d->produced >= d->task.records))
  {
    d->running = false;
    d->done = true;
  }
  ladder_step(d);
}

/* ---- the chain ---------------------------------------------------------- */

/** The filter's answer for one field, or the sum unchanged. */
static bool filtered(daq_t *d, uint8_t field, int32_t sum, uint16_t count,
                     int32_t *out)
{
  float y = 0.0f;

  /* The mean, not the sum: the task's accumulate is the chain's first stage
     and has already run, so what goes into the biquads is what came out of
     it. */
  if (!filter_push_value(&d->chain, &d->filter[field],
                         (float)sum / (float)count, &y))
  {
    return false;
  }
  *out = (int32_t)lrintf(y * (float)count);
  return true;
}

/* One sweep into the sums - the digital word's bits too, when the task
   carries pins. */
static void accumulate(daq_t *d, const int32_t *values, uint32_t digital)
{
  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    d->acc[f] += values[f];
  }
  for (uint8_t p = 0U; (p < d->task.pins) && (p < DAQ_MAX_PINS); p++)
  {
    d->dacc[p] = (uint16_t)(d->dacc[p] + (uint16_t)((digital >> p) & 1U));
  }
  d->acc_n++;
}

/** The boxcar has dumped; the shaping and the decimation happen here, and
    only what comes out of them becomes a record. */
static bool shaped_ready(daq_t *d)
{
  bool ready = false;

  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    int32_t shaped = 0;

    if (filtered(d, f, d->acc[f], d->task.accumulate, &shaped))
    {
      d->acc[f] = shaped;
      ready = true;
    }
  }
  return ready;
}

void daq_feed(daq_t *d, const int32_t *values, uint32_t at, uint32_t digital)
{
  if (!d->running)
  {
    return;                        /* a finite task done, or stopped */
  }
  if (d->skip != 0U)
  {
    d->skip--;
    return;                        /* decimated away */
  }
  d->skip = (uint16_t)(d->task.decimate - 1U);
  d->triggers++;

  if (d->acc_n == 0U)
  {
    d->first_at = at;

    /* The pins as they stood at `at`, not summed and not OR-ed across the
       window. */
    d->first_digital = digital;
  }

  /* Saturate, do not wrap. */
  if (d->acc_n < DAQ_MAX_ADDITIONS)
  {
    accumulate(d, values, digital);
  }

  /* Closed by the clock. */
  if ((d->task.accumulate == 0U)
      && ((uint32_t)(at - d->first_at) >= d->task.interval_cycles))
  {
    push_record(d);
  }
  if (d->task.accumulate == 0U)
  {
    return;
  }
  if (d->acc_n < d->task.accumulate)
  {
    return;
  }
  if (!d->filtering || shaped_ready(d))
  {
    push_record(d);
    return;
  }

  /* Swallowed by the decimation: the window is over, so the accumulator
     starts the next one. */
  clear_window(d);
}

bool daq_trigger_due(daq_t *d, uint32_t now)
{
  if (d->task.interval_cycles == 0U)
  {
    return true;
  }
  if ((uint32_t)(now - d->last_trigger) < d->task.interval_cycles)
  {
    return false;
  }
  d->last_trigger = now;
  return true;
}

void daq_sweep_begin(daq_t *d, uint32_t at, uint32_t digital)
{
  d->pending_at = at;
  d->pending_digital = digital;
}

bool daq_sweep_put(daq_t *d, int32_t raw)
{
  d->pending[d->next_field] = raw;
  if (++d->next_field < d->task.fields)
  {
    return false;
  }
  d->next_field = 0U;
  return true;
}

void daq_sweep_close(daq_t *d, uint32_t now)
{
  /* Closed by the clock: nothing gates the triggers. */
  if ((d->task.accumulate != 0U) && !daq_trigger_due(d, now))
  {
    return;
  }
  daq_feed(d, d->pending, d->pending_at, d->pending_digital);
}

/* ---- the task ----------------------------------------------------------- */

void daq_begin(daq_t *d, const daq_task_t *task)
{
  d->task = *task;
  d->stride = (uint16_t)DAQ_RECORD_BYTES(task->fields, task->pins,
                                         sensor_count(task->sensors));
  d->head = 0U;
  d->tail = 0U;
  d->dropped = 0U;
  d->produced = 0U;
  d->done = false;
  d->skip = 0U;
  d->next_field = 0U;
  d->live_any = 0U;
  memset(d->live, 0, sizeof(d->live));
  memset(d->filter, 0, sizeof(d->filter));
  clear_window(d);
  /* A new task starts at the bottom: the ladder was designed against a
     stride, and the stride is what just changed. */
  d->rung = 0U;
  d->low_for = 0U;
  d->rung_changes = 0U;
}

bool daq_start(daq_t *d, uint32_t now)
{
  if ((d->stride == 0U) || d->running)
  {
    return false;
  }
  d->head = 0U;
  d->tail = 0U;
  d->dropped = 0U;
  d->produced = 0U;
  d->worst = 0U;
  d->done = false;
  d->tone.at = now;
  d->tone.owed = 0U;
  memset(d->filter, 0, sizeof(d->filter));
  d->skip = 0U;
  d->next_field = 0U;
  d->live_any = 0U;
  memset(d->live, 0, sizeof(d->live));
  clear_window(d);
  d->running = true;
  return true;
}

void daq_stop(daq_t *d)
{
  d->running = false;
}

void daq_lose_power(daq_t *d)
{
  d->running = false;
  d->head = 0U;
  d->tail = 0U;
  d->next_field = 0U;
  d->acc_n = 0U;
  memset(d->acc, 0, sizeof(d->acc));
  d->live_any = 0U;
  memset(d->live, 0, sizeof(d->live));
}

/* ---- the chain, the ladder and the generator, as the host sets them ----- */

const char *daq_set_filter(daq_t *d, const filter_biquad_t *sections,
                           uint8_t count, uint16_t decimate)
{
  if (d->running)
  {
    return "a task is running - stop it first, because coefficients "
           "changing under a half-drained buffer hand out records of "
           "two filters with nothing to say which was which";
  }
  if (count > FILTER_MAX_SECTIONS)
  {
    return "the board runs four biquads - an eighth-order Bessel. Ask "
           "the design for a lower order";
  }
  if (decimate == 0U)
  {
    return "decimate counts filtered samples per record, so the "
           "smallest is 1";
  }

  filter_pass_through(&d->chain);
  d->chain.decimate = decimate;
  d->chain.sections = count;
  if (count > 0U)
  {
    memcpy(d->chain.section, sections,
           (size_t)count * sizeof(d->chain.section[0]));
  }
  /* The task's accumulate is the boxcar. One first stage. */
  d->chain.boxcar = 1U;
  d->filtering = (count > 0U) || (decimate > 1U);
  memset(d->filter, 0, sizeof(d->filter));
  return NULL;
}

const char *daq_set_rung(daq_t *d, uint8_t rung, uint16_t boxcar,
                         const filter_biquad_t *sections, uint8_t count,
                         uint16_t decimate)
{
  if (d->running)
  {
    return "a task is running - stop it first, because a ladder that "
           "changed under a half-drained buffer hands out records of "
           "two designs with nothing to say which was which";
  }
  if (rung >= DAQ_LADDER)
  {
    return "the board holds four rungs - ask the ladder for fewer, or "
           "a wider step between them";
  }
  if (rung > d->rungs_held)
  {
    return "rungs are built from the bottom: send 0, then 1, and so on, "
           "so there is never a gap the board would climb into";
  }
  if (count > FILTER_MAX_SECTIONS)
  {
    return "the board runs four biquads - an eighth-order Bessel";
  }
  if ((decimate == 0U) || (boxcar == 0U))
  {
    return "a rung needs a boxcar and a decimation of at least 1 each";
  }

  filter_pass_through(&d->rungs[rung]);
  d->rungs[rung].decimate = decimate;
  d->rungs[rung].sections = count;
  if (count > 0U)
  {
    memcpy(d->rungs[rung].section, sections,
           (size_t)count * sizeof(d->rungs[rung].section[0]));
  }
  d->rung_boxcar[rung] = boxcar;

  /* Rung 0 forgets what was above it, so a host that rebuilds the ladder
     cannot leave a stale rung for the board to climb into. */
  d->rungs_held = (rung == 0U) ? 1U : (uint8_t)(rung + 1U);
  if (rung == 0U)
  {
    d->rung = 0U;
    d->chain = d->rungs[0];
    d->task.accumulate = boxcar;
    d->filtering = (count > 0U) || (decimate > 1U);
    memset(d->filter, 0, sizeof(d->filter));
  }
  return NULL;
}

uint32_t daq_triggers_per_record(const daq_t *d)
{
  const uint32_t accumulate = (d->task.accumulate != 0U)
                              ? (uint32_t)d->task.accumulate : 1U;
  const uint32_t decimate = (d->task.decimate != 0U)
                            ? (uint32_t)d->task.decimate : 1U;
  const uint32_t shaped = d->filtering ? (uint32_t)d->chain.decimate : 1U;

  return accumulate * decimate * shaped;
}

const char *daq_set_tone(daq_t *d, uint32_t hz, uint32_t rate_hz,
                         int32_t amplitude, int32_t offset, uint8_t kind,
                         uint32_t cycles_per_second, uint32_t now)
{
  daq_tone_t *t = &d->tone;

  if (hz == 0U)
  {
    t->on = false;
    return NULL;
  }
  if (rate_hz == 0U)
  {
    return "a generator needs a sample rate to be anything at all";
  }
  if (kind > DAQ_TONE_RAMP)
  {
    return "kind is 0 for a sine or 1 for a ramp";
  }
  if ((kind == DAQ_TONE_RAMP) && (amplitude < 2))
  {
    return "a ramp counts up to `amplitude`, so it needs at least 2";
  }
  if ((kind == DAQ_TONE_SINE) && ((hz * 2U) > rate_hz))
  {
    return "a tone at or past half its own sample rate is an alias of "
           "something else - ask for a higher rate or a lower tone";
  }

  t->kind = kind;
  if (kind == DAQ_TONE_RAMP)
  {
    t->step = hz;
    t->mod = amplitude;
  }
  else
  {
    const float step = TWO_PI * (float)hz / (float)rate_hz;

    t->cos_step = cosf(step);
    t->sin_step = sinf(step);
    t->x = 1.0f;
    t->y = 0.0f;
    t->amp = (float)amplitude;
  }
  t->offset = (float)offset;
  t->cycles = cycles_per_second / rate_hz;
  t->at = now;
  t->owed = 0U;
  t->n = 0U;
  t->on = true;
  return NULL;
}

static void tone_renormalise(daq_tone_t *t, float x, float y)
{
  const float size = sqrtf((x * x) + (y * y));

  if (size > TONE_TINY)
  {
    t->x = x / size;
    t->y = y / size;
  }
}

/** One tone sample: the unit vector turned by one step, renormalised every
    1024 so the rotation's magnitude cannot creep; or the ramp, integer all
    the way so a host computes the same value rather than something within a
    tolerance of it. */
static int32_t tone_next(daq_tone_t *t)
{
  if (t->kind == DAQ_TONE_RAMP)
  {
    const uint32_t n = t->n++;

    return (int32_t)t->offset + (int32_t)((n * t->step) % (uint32_t)t->mod);
  }

  const float x = (t->x * t->cos_step) - (t->y * t->sin_step);
  const float y = (t->x * t->sin_step) + (t->y * t->cos_step);

  t->x = x;
  t->y = y;
  if ((++t->n & TONE_RENORM_MASK) == 0U)
  {
    tone_renormalise(t, x, y);
  }
  return (int32_t)lrintf(t->offset + (t->amp * t->y));
}

void daq_tone_poll(daq_t *d, uint32_t now, uint32_t burst)
{
  daq_tone_t *t = &d->tone;

  if (!t->on || !d->running || (t->cycles == 0U))
  {
    return;
  }

  /* Exactly the samples the elapsed time owed, and the remainder is carried
     rather than dropped: a generator that rounded down every turn would run
     slow by a fraction of a sample per poll, and a host checking phase would
     see the drift and call it a lost record. */
  const uint32_t elapsed = (uint32_t)(now - t->at) + t->owed;
  uint32_t owed = elapsed / t->cycles;

  t->owed = elapsed - (owed * t->cycles);
  t->at = now;

  /* A burst is bounded so one long gap cannot hold the caller's loop past
     what it can afford; what the clamp drops is dropped rather than owed, or
     the debt would burst again next turn. */
  if (owed > burst)
  {
    owed = burst;
  }
  for (uint32_t i = 0U; i < owed; i++)
  {
    const int32_t sample = tone_next(t);

    for (uint8_t f = 0U; f < d->task.fields; f++)
    {
      d->pending[f] = sample;
    }
    daq_feed(d, d->pending, now, 0U);
  }
}

/* ---- the live accumulator ----------------------------------------------- */

/** One sample into the live accumulator, for one field. */
void daq_live_insert(daq_t *d, uint8_t field, int32_t value, uint32_t at,
                     uint32_t digital)
{
  daq_slot_t *slot = &d->live[field];
  const uint32_t masked = held(d);

  if (d->live_any == 0U)
  {
    d->live_first = at;
  }
  if (slot->additions == 0U)
  {
    slot->lowest = value;
    slot->highest = value;
  }
  else if (value < slot->lowest)
  {
    slot->lowest = value;
  }
  else if (value > slot->highest)
  {
    slot->highest = value;
  }

  /* Saturate. */
  if (slot->additions < DAQ_MAX_ADDITIONS)
  {
    slot->sum += value;
    slot->additions++;
  }
  d->live_any = 1U;
  d->live_last = at;
  d->live_digital = digital;

  released(d, masked);
}

void daq_take_live(daq_t *d, daq_live_t *out)
{
  /* Under the guard because a feed on the timer's interrupt writes these: a
     reader that caught the count from one trigger and a sum from the next
     would report a mean that was never taken. */
  const uint32_t masked = held(d);

  out->fresh = (d->live_any != 0U);
  out->first = d->live_first;
  out->last = d->live_last;
  out->digital = d->live_digital;
  memcpy(out->slot, d->live, sizeof(out->slot));
  memset(d->live, 0, sizeof(d->live));
  d->live_any = 0U;

  released(d, masked);
}
