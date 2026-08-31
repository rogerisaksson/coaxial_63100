/**
  ******************************************************************************
  * @file    board_daq.c
  * @brief   One acquisition task: configure, start, read. DAQmx's shape, cut
  *          down to what this board has.
  *
  * One task, not many: three converters and one timer, and pretending
  * otherwise puts the arbitration where it cannot be honoured.
  *
  *   channels     which ADC rows, as a bitmask
  *   clock        SOFTWARE (the main loop) or TIM1 (the injected group)
  *   sample_time  0..7, the converter's own window
  *   decimate     keep one trigger in N
  *   accumulate   sum N samples per record; 0 closes the record on
  *                `interval_us` instead, with the converter free
  *   records      stop after this many, 0 to run until stopped
  *
  * The buffer is bytes and the stride comes from the config, so no host holds
  * a copy of the record shape - it asks for the layout and the board names
  * every field. A shape written here and mirrored in a decoder is two answers
  * to one question, and the mirror goes stale.
  *
  * Accumulation SUMS rather than averages: it keeps the bits an average
  * throws away, and the count rides in the record, so a host divides by
  * what actually went in rather than by what it asked for. The digital
  * pins go the same way and come out as a DUTY - a byte each, 0..255 of
  * the window - because a level sampled once and decimated by two
  * thousand is aliased by construction.
  *
  * TWO WAYS TO CLOSE A RECORD, and `accumulate` picks which. A COUNT
  * (accumulate >= 1) is the old one: N samples make a record, and
  * `interval_us` gates the triggers. A CLOCK (accumulate == 0) is the
  * other: the converter runs at whatever the loop manages - megasamples
  * a second is the whole reason to sum on the target - and `interval_us`
  * closes the record, so the host gets the rate it can drain and every
  * sample taken in between is in the sum rather than thrown away.
  ******************************************************************************
  */
#include "board_limits.h"
#include "board.h"
#include "board_hw.h"
#include "filter.h"

#include <math.h>
#include <string.h>

/* `.buffers` is the AXI SRAM section in STM32H753xx_FLASH.ld. NOLOAD,
   so a quarter of a megabyte of zeroes is not carried in the image. */
static uint8_t  s_buf[DAQ_BYTES] __attribute__((section(".buffers")));
static volatile uint32_t s_head;        /* byte offset of the next write */
static volatile uint32_t s_tail;        /* byte offset of the next read  */
static volatile uint32_t s_dropped;
static volatile uint32_t s_produced;
/* The fullest the ring has been, in records. Taken where a record is
   pushed rather than where a host asks: a level read at the host's
   leisure is a level between the peaks, and the peak is what says
   whether the next one drops. */
static volatile uint32_t s_worst;
/* Every sweep that reached the accumulator, gated or not. */
static volatile uint32_t s_triggers;

static board_daq_config_t s_cfg;
static volatile bool s_running;
static volatile bool s_done;
static volatile bool s_lost_power;

static uint16_t s_stride;
static uint8_t  s_order[BOARD_DAQ_MAX_CHANNELS];  /* channel index per field */
static uint8_t  s_fields;

/* Accumulator, reset every time a record is pushed. */
static int32_t  s_acc[BOARD_DAQ_MAX_CHANNELS];
static uint16_t s_acc_n;
static uint16_t s_skip;
static uint32_t s_first_at;
/* THE PINS AS A DUTY, not as a snapshot. A digital line decimated by
   2000 and reported as the level it happened to have at one instant is
   aliased by construction - KEEPALIVE toggles at ~100 kHz and would
   read as a coin toss. Counted high over the window instead, which is
   what a low-pass does to a square wave and is the only answer that
   survives the decimation. */
static uint16_t s_dacc[BOARD_DAQ_MAX_PINS];
static uint32_t s_first_digital;
static uint32_t s_last_trigger;
static uint32_t s_interval_cycles;
static bool     s_rate_auto;        /* no rate asked for: one was chosen */

/* The live accumulator, kept beside the ring and fed by the same triggers.
   The ring is a capture and drops when it is full; this is the freshest
   average and cannot drop, because a reader that is late just gets a wider
   window. Separate totals so the two do not consume each other. */
/* One sum AND one count per channel, not one count for the lot. The
   software poll reads one channel per turn of the main loop, so over any
   window the channels have had different numbers of samples - a single
   count would divide most of them by the wrong number. */
static board_daq_slot_t s_live[BOARD_DAQ_MAX_CHANNELS];
/* A FLAG, not a count. It was `s_live_any++` per sample, which at 50 kHz
   wraps through zero every 23.9 hours - and zero is what `fresh` and the
   window-start test both read as "nothing has arrived". Nothing needs the
   number; only whether anything came. */
static uint8_t  s_live_any;
static uint32_t s_live_first;
static uint32_t s_live_last;
static uint32_t s_live_digital;

/* The host's anti-alias chain, and one running state per field. The
   boxcar in `s_chain` is the task's own `accumulate`, set when the
   filter is loaded and again when the task is configured - one first
   stage, not two. */
static filter_design_t  s_chain;
static filter_channel_t s_filter[BOARD_DAQ_MAX_CHANNELS];
static bool             s_filtering;

/* THE LADDER. Each rung is a whole design and its own boxcar, so a
   record says which one made it without a field for it: `samples` IS
   the rung's accumulate. The board climbs when the ring fills and comes
   back down when it has been empty a while - it cannot design anything
   itself, so what it does is choose between designs the host sent. */
static filter_design_t s_rungs[BOARD_DAQ_LADDER];
static uint16_t        s_rung_boxcar[BOARD_DAQ_LADDER];
static uint8_t         s_rungs_held;
static uint8_t         s_rung;
static uint32_t        s_rung_changes;
static uint32_t        s_low_for;

/* The tone generator: a complex rotation, two multiplies a sample, so a
   burst of thousands costs microseconds where as many sinf() calls
   would cost a millisecond. Renormalised every so often - the rotation
   is stable in phase and creeps in magnitude. */
static bool     s_tone_on;
static uint8_t  s_tone_kind;
static uint32_t s_tone_step;        /* the ramp's increment a sample   */
static int32_t  s_tone_mod;         /* and what it counts up to        */
static float    s_tone_cos;         /* per-sample rotation             */
static float    s_tone_sin;
static float    s_tone_x = 1.0f;    /* the rotating unit vector        */
static float    s_tone_y;
static float    s_tone_amp;
static float    s_tone_offset;
static uint32_t s_tone_cycles;      /* CYCCNT ticks per tone sample    */
static uint32_t s_tone_at;          /* when the last sample was made   */
static uint32_t s_tone_owed;        /* fractional ticks carried over   */
static uint32_t s_tone_n;

static uint8_t  s_next_field;
static int32_t  s_pending[BOARD_DAQ_MAX_CHANNELS];
static uint32_t s_pending_at;
static uint32_t s_pending_digital;


/** Microseconds to CYCCNT ticks, saturating rather than wrapping.
  *
  * DWT->CYCCNT is 32 bits at 475 MHz, so it comes round every 9.04 s and an
  * interval longer than that cannot be expressed at all. The multiply used
  * to be done in uint32: asking for one record every 30 s produced
  * 30e6 * 475 mod 2^32, about 1.5 s, so a run left alone overnight filled
  * the ring and dropped instead of ticking over slowly. `Board_DaqConfigure`
  * refuses it outright; this is the belt for the paths that cannot.
  */
static uint32_t interval_cycles(uint32_t interval_us)
{
  const uint64_t cycles = (uint64_t)interval_us
                          * (uint64_t)(SystemCoreClock / 1000000U);

  return (cycles > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;
}


static uint32_t room(void)
{
  const uint32_t head = s_head;
  const uint32_t tail = s_tail;

  return (head >= tail) ? (DAQ_BYTES - head + tail) : (tail - head);
}


uint32_t Board_DaqAvailable(void)
{
  const uint32_t head = s_head;
  const uint32_t tail = s_tail;
  const uint32_t used = (head >= tail) ? (head - tail)
                                       : (DAQ_BYTES - tail + head);

  return (s_stride != 0U) ? (used / s_stride) : 0U;
}


static void put(const uint8_t *src, uint16_t len)
{
  /* Wraps at the end of the buffer, not at a record boundary: the stride
     divides nothing in particular and rounding the buffer down to whole
     records for every possible stride wastes more than it saves. */
  for (uint16_t i = 0U; i < len; i++)
  {
    s_buf[s_head] = src[i];
    s_head = (s_head + 1U) % DAQ_BYTES;
  }
}


/* Big endian, like every other u32 this board puts on the wire. Writing it
   LSB first here was a real bug: the values came back as huge negatives and
   the timestamps ran backwards, and CLAUDE.md already carried the same
   mistake from the IMU's report interval. */
static uint16_t put_be32(uint8_t *dst, uint16_t at, uint32_t v)
{
  dst[at++] = (uint8_t)((v >> 24) & 0xFFU);
  dst[at++] = (uint8_t)((v >> 16) & 0xFFU);
  dst[at++] = (uint8_t)((v >> 8) & 0xFFU);
  dst[at++] = (uint8_t)(v & 0xFFU);
  return at;
}


/** Take rung `n`: its whole design, and the accumulate that goes with
  * it. The filter state goes with them - coefficients changing under a
  * running biquad is a transient nothing in the record would explain,
  * and the settling is the price of the step. */
static void take_rung(uint8_t n)
{
  if ((n >= s_rungs_held) || (n == s_rung))
  {
    return;
  }
  s_rung = n;
  s_chain = s_rungs[n];
  s_cfg.accumulate = s_rung_boxcar[n];
  s_filtering = (s_chain.sections > 0U) || (s_chain.decimate > 1U);
  s_rung_changes++;
  s_low_for = 0U;
  memset(s_filter, 0, sizeof(s_filter));
  memset(s_acc, 0, sizeof(s_acc));
  memset(s_dacc, 0, sizeof(s_dacc));
  s_acc_n = 0U;
}


/** One record's worth of pressure on the ring, and what it costs.
  *
  * Climbing means filtering and decimating harder, which is the only
  * answer a board has to a link that cannot keep up: dropping records
  * loses what happened, and a slower converter loses it too. Coming back
  * down needs the ring to have been empty a WHILE - it empties the
  * instant a host reads it, so one look says only that it just drained.
  */
static void ladder_step(void)
{
  if ((s_rungs_held < 2U) || !s_cfg.adapt)
  {
    return;
  }

  const uint32_t capacity = (s_stride != 0U) ? (DAQ_BYTES / s_stride) : 0U;

  if (capacity == 0U)
  {
    return;
  }

  const uint32_t held = Board_DaqAvailable();
  /* The lower of the two: a fraction of a small ring, a fixed backlog of
     a large one. A rung answers how far BEHIND the link is, and that is
     a count of records rather than a share of whatever was allocated. */
  uint32_t climb = (capacity * BOARD_DAQ_CLIMB_AT) / 8U;
  uint32_t fall = (capacity * BOARD_DAQ_FALL_AT) / 8U;

  if (climb > BOARD_DAQ_CLIMB_MAX)
  {
    climb = BOARD_DAQ_CLIMB_MAX;
  }
  if (fall > (BOARD_DAQ_CLIMB_MAX / 8U))
  {
    fall = BOARD_DAQ_CLIMB_MAX / 8U;
  }

  if (held >= climb)
  {
    take_rung((uint8_t)(s_rung + 1U));
    return;
  }

  if (held > fall)
  {
    s_low_for = 0U;
    return;
  }
  if (++s_low_for >= BOARD_DAQ_FALL_AFTER)
  {
    s_low_for = 0U;
    if (s_rung > 0U)
    {
      take_rung((uint8_t)(s_rung - 1U));
    }
  }
}


static void push_record(void)
{
  uint8_t rec[4U + (4U * BOARD_DAQ_MAX_CHANNELS) + 4U + 2U];
  uint16_t at = put_be32(rec, 0U, s_first_at);

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    at = put_be32(rec, at, (uint32_t)s_acc[f]);
  }

  if (s_cfg.digital != 0U)
  {
    /* 0..255 of the window, so a host divides by 255 and gets the
       fraction of it the pin was high for. A byte is 0.4 % of a
       window, which is finer than anything a decimated pin means. */
    const uint8_t pins = Board_DigitalIoCount();
    const uint32_t n = (s_acc_n > 0U) ? s_acc_n : 1U;

    for (uint8_t p = 0U; (p < pins) && (p < BOARD_DAQ_MAX_PINS); p++)
    {
      rec[at++] = (uint8_t)(((uint32_t)s_dacc[p] * 255U + (n / 2U)) / n);
    }
  }

  /* THE DIVISOR TRAVELS WITH THE SUM. Closed by the clock the count is
     whatever the window held, and a host that took it from the config
     would divide by a number the board never used. Last in the record
     so the fields keep the offsets a reader already knows. */
  rec[at++] = (uint8_t)((s_acc_n >> 8) & 0xFFU);
  rec[at++] = (uint8_t)(s_acc_n & 0xFFU);

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();

  if (room() > s_stride)
  {
    put(rec, s_stride);
    s_produced++;

    const uint32_t held = Board_DaqAvailable();

    if (held > s_worst)
    {
      s_worst = held;
    }
  }
  else
  {
    s_dropped++;
  }

  if (!masked)
  {
    __enable_irq();
  }

  memset(s_acc, 0, sizeof(s_acc));
  memset(s_dacc, 0, sizeof(s_dacc));
  s_acc_n = 0U;

  if ((s_cfg.records != 0U) && (s_produced >= s_cfg.records))
  {
    s_running = false;
    s_done = true;
  }

  ladder_step();
}


/** The filter's answer for one field, or the sum unchanged.
  *
  * The record's field is a SUM and `samples` its divisor, and that does
  * not change here: the chain's output is a mean, so it goes back on the
  * wire multiplied by the divisor the record carries. A host divides as
  * it always did and never learns there was a filter - which is the
  * point, because the alternative was a second record shape.
  */
static bool filtered(uint8_t field, int32_t sum, uint16_t count,
                     int32_t *out)
{
  float y = 0.0f;

  /* THE MEAN, not the sum: the task's accumulate is the chain's first
     stage and has already run, so what goes into the biquads is what
     came out of it. Pushing the sum instead multiplied every reading by
     the count - measured on the board, a 32768-code tone arrived as
     8.2 million. Divided here rather than in an int, so the precision
     the accumulate bought is not rounded away first. */
  if (!filter_push_value(&s_chain, &s_filter[field],
                         (float)sum / (float)count, &y))
  {
    return false;
  }
  *out = (int32_t)lrintf(y * (float)count);
  return true;
}


/** One trigger's worth of samples, already read. Accumulates and may push. */
static void feed(const int32_t *values, uint32_t at, uint32_t digital)
{
  if (s_skip != 0U)
  {
    s_skip--;
    return;                        /* decimated away */
  }
  s_skip = (uint16_t)(s_cfg.decimate - 1U);
  s_triggers++;

  if (s_acc_n == 0U)
  {
    s_first_at = at;

    /* The pins as they stood at `at`, not summed and not OR-ed across the
       window. Summing a bitmask means nothing, and an OR would report a
       pin as high that was high for one sample in fifty with no field
       saying which. With accumulate at 1 this is every sample. */
    s_first_digital = digital;
  }

  /* SATURATE, do not wrap. A window closed by the clock has no bound on
     how many samples it holds, and `s_acc` is int32 against a
     single-ended code of 65535 - so past LIVE_MAX_ADDITIONS the sum
     would go negative and the host would divide the wreck by the count
     and report it as a mean. Stopping instead keeps the mean over what
     did go in true, and the count says how many that was.

     The same bound and the same reasoning as the live accumulator's:
     INT32_MAX / 65535, so the next addition cannot carry it past the
     end whatever the channel read. */
  if (s_acc_n < LIVE_MAX_ADDITIONS)
  {
    for (uint8_t f = 0U; f < s_fields; f++)
    {
      s_acc[f] += values[f];
    }
    if (s_cfg.digital != 0U)
    {
      const uint8_t pins = Board_DigitalIoCount();

      for (uint8_t p = 0U; (p < pins) && (p < BOARD_DAQ_MAX_PINS); p++)
      {
        s_dacc[p] = (uint16_t)(s_dacc[p] +
                               (uint16_t)((digital >> p) & 1U));
      }
    }
    s_acc_n++;
  }

  if (s_cfg.accumulate == 0U)
  {
    /* Closed by the clock. Unsigned elapsed arithmetic, so the CYCCNT
       wrap costs nothing (invariant 2). A clock-closed window and a
       filter are alternatives: a fixed-rate filter needs a fixed
       decimation, and this window's length is whatever the loop
       managed. Board_DaqConfigure refuses the pair. */
    if ((uint32_t)(at - s_first_at) >= s_interval_cycles)
    {
      push_record();
    }
    return;
  }

  if (s_acc_n < s_cfg.accumulate)
  {
    return;
  }

  if (!s_filtering)
  {
    push_record();
    return;
  }

  /* The boxcar has dumped; the shaping and the decimation happen here,
     and only what comes out of them becomes a record. Every field is
     pushed so their states stay in step - they share a decimation
     counter's worth of history, and one field skipped would put the
     record's channels a sample apart. */
  bool ready = false;

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    int32_t shaped = 0;

    if (filtered(f, s_acc[f], s_cfg.accumulate, &shaped))
    {
      s_acc[f] = shaped;
      ready = true;
    }
  }

  if (ready)
  {
    push_record();
    return;
  }

  /* Swallowed by the decimation: the window is over, so the accumulator
     starts the next one. */
  memset(s_acc, 0, sizeof(s_acc));
  memset(s_dacc, 0, sizeof(s_dacc));
  s_acc_n = 0U;
}


/** True when the injected sequence converts every selected field: the
  * three phases, and the DC link and NTC that ride rank 2. */
static bool only_injected(void)
{
  for (uint8_t f = 0U; f < s_fields; f++)
  {
    if (!Board_AdcInjected(s_order[f]))
    {
      return false;
    }
  }
  return true;
}

const char *Board_DaqConfigure(const board_daq_config_t *cfg)
{
  /* Every refusal says which check failed, in the board's own words. The
     board is the only thing that knows which one it was; a host listing
     possible causes is the second answer this codebase keeps deleting.
     Each one says what is wrong AND what to do about it - a refusal that
     leaves the caller guessing has done half a job. */
  if (cfg == NULL)
  {
    return "no configuration given - pass one";
  }
  if (s_running)
  {
    return "a task is running - stop it first, because a stride that "
           "changed under a half-drained buffer would hand out records of "
           "two shapes with nothing to say which was which";
  }
  if ((cfg->clock != BOARD_DAQ_CLOCK_SOFTWARE) &&
      (cfg->clock != BOARD_DAQ_CLOCK_TIM1))
  {
    return "clock is 0 for the main loop or 1 for the injected group";
  }
  if (cfg->decimate == 0U)
  {
    return "decimate counts triggers, so the smallest is 1";
  }
  if ((uint64_t)cfg->interval_us * (uint64_t)(SystemCoreClock / 1000000U)
      > (uint64_t)UINT32_MAX)
  {
    return "interval_us is more than the cycle counter can express - it is "
           "32 bits at 475 MHz and comes round every 9.04 s. Sample faster "
           "and decimate, or drive it from the host";
  }
  if (cfg->accumulate > LIVE_MAX_ADDITIONS)
  {
    /* The record's sum is int32 and a single-ended code reaches 65535, so
       beyond this the sum wraps and the host divides a negative by the
       count and calls it a mean. */
    return "accumulate is at most 32767 - beyond that the record's sum "
           "overflows a signed 32-bit total and stops being a measurement";
  }
  if (!Board_AdcSetSampleTime(cfg->sample_time))
  {
    return "sample_time is 0 to 7, shortest window first";
  }

  /* Field order is the channel table's order, so a host reading the layout
     and a host reading `0x6D` get the same answer in the same sequence. */
  const uint8_t rows = Board_AdcCount();
  s_fields = 0U;
  for (uint8_t i = 0U; (i < rows) && (i < BOARD_DAQ_MAX_CHANNELS); i++)
  {
    if ((cfg->channels & (1U << i)) != 0U)
    {
      s_order[s_fields++] = i;
    }
  }
  if (s_fields == 0U)
  {
    return "no channels selected - the mask is over the rows of 0x6D kind 0";
  }

  /* TIM1 clock means the injected group, and that group converts the three
     phases and nothing else. Asking for a channel it does not carry is a
     configuration that cannot be honoured, so it is refused here rather
     than answered with zeros. */
  if ((cfg->clock == BOARD_DAQ_CLOCK_TIM1) && !only_injected())
  {
    return "the TIM1 clock carries what the injected sequence converts - "
           "the three phases, and the DC link and the NTC on rank 2. Any "
           "other channel has to come through the meter on the software "
           "clock";
  }

  if ((cfg->accumulate == 0U) && s_filtering)
  {
    return "a clock-closed record and a filter are alternatives: a "
           "fixed-rate filter needs a fixed decimation, and a window's "
           "length is whatever the loop managed. Ask for an accumulate, "
           "or clear the filter";
  }

  s_cfg = *cfg;
  /* Remembered here, where the ASK is still visible. */
  s_rate_auto = (cfg->interval_us == 0U) && (cfg->records == 0U);
  s_interval_cycles = interval_cycles(cfg->interval_us);
  /* One byte of duty per pin where there used to be one snapshot word,
     and + 2 for the sample count every record carries. */
  s_stride = (uint16_t)(4U + (4U * s_fields)
                        + ((cfg->digital != 0U)
                           ? Board_DigitalIoCount() : 0U) + 2U);
  s_head = 0U;
  s_tail = 0U;
  s_dropped = 0U;
  s_produced = 0U;
  s_done = false;
  s_skip = 0U;
  s_next_field = 0U;
  s_acc_n = 0U;
  s_live_any = 0U;
  memset(s_live, 0, sizeof(s_live));
  memset(s_acc, 0, sizeof(s_acc));
  memset(s_dacc, 0, sizeof(s_dacc));
  memset(s_filter, 0, sizeof(s_filter));
  /* A new task starts at the bottom: the ladder was designed against a
     stride, and the stride is what just changed. */
  s_rung = 0U;
  s_low_for = 0U;
  s_rung_changes = 0U;
  return NULL;
}


const char *Board_DaqSetFilter(const void *sections, uint8_t count,
                               uint16_t decimate)
{
  if (s_running)
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

  filter_pass_through(&s_chain);
  s_chain.decimate = decimate;
  s_chain.sections = count;
  if (count > 0U)
  {
    memcpy(s_chain.section, sections,
           (size_t)count * sizeof(s_chain.section[0]));
  }
  /* The task's accumulate IS the boxcar. One first stage. */
  s_chain.boxcar = 1U;
  s_filtering = (count > 0U) || (decimate > 1U);
  memset(s_filter, 0, sizeof(s_filter));
  return NULL;
}


const char *Board_DaqSetRung(uint8_t rung, uint16_t boxcar,
                             const void *sections, uint8_t count,
                             uint16_t decimate)
{
  if (s_running)
  {
    return "a task is running - stop it first, because a ladder that "
           "changed under a half-drained buffer hands out records of "
           "two designs with nothing to say which was which";
  }
  if (rung >= BOARD_DAQ_LADDER)
  {
    return "the board holds four rungs - ask the ladder for fewer, or "
           "a wider step between them";
  }
  if (rung > s_rungs_held)
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

  filter_pass_through(&s_rungs[rung]);
  s_rungs[rung].decimate = decimate;
  s_rungs[rung].sections = count;
  if (count > 0U)
  {
    memcpy(s_rungs[rung].section, sections,
           (size_t)count * sizeof(s_rungs[rung].section[0]));
  }
  s_rung_boxcar[rung] = boxcar;

  /* Rung 0 forgets what was above it, so a host that rebuilds the
     ladder cannot leave a stale rung for the board to climb into. */
  s_rungs_held = (rung == 0U) ? 1U : (uint8_t)(rung + 1U);
  if (rung == 0U)
  {
    s_rung = 0U;
    s_chain = s_rungs[0];
    s_cfg.accumulate = boxcar;
    s_filtering = (count > 0U) || (decimate > 1U);
    memset(s_filter, 0, sizeof(s_filter));
  }
  return NULL;
}


const char *Board_DaqSetTone(uint32_t hz, uint32_t rate_hz,
                             int32_t amplitude, int32_t offset,
                             uint8_t kind)
{
  if (hz == 0U)
  {
    s_tone_on = false;
    return NULL;
  }
  if (rate_hz == 0U)
  {
    return "a generator needs a sample rate to be anything at all";
  }
  if (kind > BOARD_DAQ_TONE_RAMP)
  {
    return "kind is 0 for a sine or 1 for a ramp";
  }
  if (kind == BOARD_DAQ_TONE_RAMP)
  {
    if (amplitude < 2)
    {
      return "a ramp counts up to `amplitude`, so it needs at least 2";
    }
    s_tone_kind = kind;
    s_tone_step = hz;
    s_tone_mod = amplitude;
    s_tone_offset = (float)offset;
    s_tone_cycles = SystemCoreClock / rate_hz;
    s_tone_at = Board_Cycles();
    s_tone_owed = 0U;
    s_tone_n = 0U;
    s_tone_on = true;
    return NULL;
  }
  if ((hz * 2U) > rate_hz)
  {
    return "a tone at or past half its own sample rate is an alias of "
           "something else - ask for a higher rate or a lower tone";
  }
  s_tone_kind = BOARD_DAQ_TONE_SINE;

  const float step = 2.0f * 3.14159265358979f * (float)hz / (float)rate_hz;

  s_tone_cos = cosf(step);
  s_tone_sin = sinf(step);
  s_tone_x = 1.0f;
  s_tone_y = 0.0f;
  s_tone_amp = (float)amplitude;
  s_tone_offset = (float)offset;
  s_tone_cycles = SystemCoreClock / rate_hz;
  s_tone_at = Board_Cycles();
  s_tone_owed = 0U;
  s_tone_n = 0U;
  s_tone_on = true;
  return NULL;
}


/** One tone sample: the unit vector turned by one step, renormalised
  * every 1024 so the rotation's magnitude cannot creep. */
static int32_t tone_next(void)
{
  if (s_tone_kind == BOARD_DAQ_TONE_RAMP)
  {
    /* Integer all the way, so a host computes the same value rather
       than something within a tolerance of it. */
    const uint32_t n = s_tone_n++;

    return (int32_t)s_tone_offset +
           (int32_t)((n * s_tone_step) % (uint32_t)s_tone_mod);
  }

  const float x = (s_tone_x * s_tone_cos) - (s_tone_y * s_tone_sin);
  const float y = (s_tone_x * s_tone_sin) + (s_tone_y * s_tone_cos);

  s_tone_x = x;
  s_tone_y = y;
  if ((++s_tone_n & 1023U) == 0U)
  {
    const float size = sqrtf((x * x) + (y * y));

    if (size > 1e-6f)
    {
      s_tone_x = x / size;
      s_tone_y = y / size;
    }
  }
  return (int32_t)lrintf(s_tone_offset + (s_tone_amp * s_tone_y));
}


void Board_DaqTonePoll(void)
{
  if (!s_tone_on || !s_running || (s_tone_cycles == 0U) ||
      (s_cfg.clock != BOARD_DAQ_CLOCK_SOFTWARE))
  {
    return;
  }

  /* EXACTLY the samples the elapsed time owed, and the remainder is
     carried rather than dropped: a generator that rounded down every
     turn would run slow by a fraction of a sample per poll, and a host
     checking phase would see the drift and call it a lost record. */
  const uint32_t now = Board_Cycles();
  const uint32_t elapsed = (uint32_t)(now - s_tone_at) + s_tone_owed;
  uint32_t owed = elapsed / s_tone_cycles;

  s_tone_owed = elapsed - (owed * s_tone_cycles);
  s_tone_at = now;

  /* A burst is bounded so one long gap cannot hold the main loop past a
     frame boundary - RTU discards a frame whose characters arrive more
     than t1.5 apart, and this runs beside the link. */
  if (owed > BOARD_DAQ_TONE_BURST)
  {
    owed = BOARD_DAQ_TONE_BURST;
  }

  for (uint32_t i = 0U; i < owed; i++)
  {
    const int32_t sample = tone_next();

    for (uint8_t f = 0U; f < s_fields; f++)
    {
      s_pending[f] = sample;
    }
    feed(s_pending, now, 0U);
  }
}


bool Board_DaqToneOn(void)
{
  return s_tone_on;
}


bool Board_DaqRateIsAuto(void)
{
  return s_rate_auto;
}


uint32_t Board_DaqTriggersPerRecord(void)
{
  const uint32_t accumulate = (s_cfg.accumulate != 0U)
                              ? (uint32_t)s_cfg.accumulate : 1U;
  const uint32_t decimate = (s_cfg.decimate != 0U)
                            ? (uint32_t)s_cfg.decimate : 1U;
  const uint32_t shaped = s_filtering ? (uint32_t)s_chain.decimate : 1U;

  return accumulate * decimate * shaped;
}


void Board_DaqSetInterval(uint32_t interval_us)
{
  s_cfg.interval_us = interval_us;
  s_interval_cycles = interval_cycles(interval_us);
}


const char *Board_DaqStart(void)
{
  if (s_stride == 0U)
  {
    return "nothing configured to start - configure the task first";
  }
  if (s_running)
  {
    return "already running - stop it first, or leave it be";
  }
  if (!Board_AfeOn())
  {
    return "AFE_ON is off, and it powers the converter's reference - every "
           "channel would read exact mid-scale, which is not a measurement";
  }
  s_head = 0U;
  s_tail = 0U;
  s_dropped = 0U;
  s_produced = 0U;
  s_worst = 0U;
  s_done = false;
  s_lost_power = false;
  s_tone_at = Board_Cycles();
  s_tone_owed = 0U;
  memset(s_dacc, 0, sizeof(s_dacc));
  memset(s_filter, 0, sizeof(s_filter));
  s_skip = 0U;
  s_next_field = 0U;
  s_acc_n = 0U;
  s_live_any = 0U;
  memset(s_live, 0, sizeof(s_live));
  memset(s_acc, 0, sizeof(s_acc));
  s_running = true;
  return NULL;
}


void Board_DaqStop(void)
{
  s_running = false;
}


/** One sample into the live accumulator, for one channel.
  *
  * Per sample and not per record: the loop runs at whatever the converter
  * and the SPI buses manage, and holding a channel back until its
  * neighbours caught up would throw away the samples that make the average
  * worth having.
  */
static void live_insert(uint8_t field, int32_t value, uint32_t at,
                        uint32_t digital)
{
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();

  if (s_live_any == 0U)
  {
    s_live_first = at;
  }
  if (s_live[field].additions == 0U)
  {
    s_live[field].lowest = value;
    s_live[field].highest = value;
  }
  else if (value < s_live[field].lowest)
  {
    s_live[field].lowest = value;
  }
  else if (value > s_live[field].highest)
  {
    s_live[field].highest = value;
  }

  /* SATURATE. `sum` is int32 and a single-ended channel reads up to 65535,
     so at 50 kHz the sum passes INT32_MAX in 0.66 s - signed overflow, and
     the host divides the wrapped negative by `additions` and reports it as a
     mean. Stop widening the window instead: the mean over what did go in
     stays true, which a wrapped sum does not.

     LIVE_MAX_ADDITIONS is INT32_MAX / 65535, the worst a single sample can
     be, so the next addition can never carry it past the end. */
  if (s_live[field].additions < LIVE_MAX_ADDITIONS)
  {
    s_live[field].sum += value;
    s_live[field].additions++;
  }
  s_live_any = 1U;
  s_live_last = at;
  s_live_digital = digital;

  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DaqTakeLive(board_daq_live_t *out)
{
  if (out == NULL)
  {
    return;
  }

  /* Under PRIMASK because feed() runs in ADC3's interrupt on the TIM1
     clock: a reader that caught the count from one trigger and a sum from
     the next would report a mean that was never taken. */
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();

  out->fresh = (s_live_any != 0U);
  out->first = s_live_first;
  out->last = s_live_last;
  out->digital = s_live_digital;
  for (uint8_t f = 0U; f < BOARD_DAQ_MAX_CHANNELS; f++)
  {
    out->slot[f] = s_live[f];
    s_live[f].sum = 0;
    s_live[f].additions = 0U;
    s_live[f].lowest = 0;
    s_live[f].highest = 0;
  }
  s_live_any = 0U;

  if (!masked)
  {
    __enable_irq();
  }
}


void Board_DaqState(board_daq_state_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->running = s_running;
  out->done = s_done;
  out->lost_power = s_lost_power;
  out->stride = s_stride;
  out->fields = s_fields;
  out->available = Board_DaqAvailable();
  out->produced = s_produced;
  out->dropped = s_dropped;
  /* What the ring holds at this stride, not what it holds in bytes: a
     level is a fraction of something, and DAQ_BYTES is not what a host
     counts records against. */
  out->capacity = (s_stride != 0U) ? (DAQ_BYTES / s_stride) : 0U;
  out->worst = s_worst;
  out->rung = s_rung;
  out->rungs = s_rungs_held;
  out->rung_changes = s_rung_changes;
  out->triggers = s_triggers;
  out->config = s_cfg;
}


bool Board_DaqField(uint8_t field, uint8_t *channel)
{
  if ((field >= s_fields) || (channel == NULL))
  {
    return false;
  }
  *channel = s_order[field];
  return true;
}


/** AFE_ON off means every channel reads exact mid-scale - it powers the ADC
  * reference, not just the signal path (invariant 9). The task stops and the
  * buffers empty: an accumulator holding half real samples and half
  * mid-scale divides out to something plausible with no field to say so.
  */
static bool powered(void)
{
  if (Board_AfeOn())
  {
    return true;
  }

  if (s_running)
  {
    s_running = false;
    s_lost_power = true;
    s_head = 0U;
    s_tail = 0U;
    s_next_field = 0U;
    s_acc_n = 0U;
    memset(s_acc, 0, sizeof(s_acc));
    s_live_any = 0U;
    memset(s_live, 0, sizeof(s_live));
  }
  return false;
}


void Board_DaqPoll(void)
{
  int32_t raw;
  int32_t uv;
  int32_t scaled;

  if (!s_running || (s_cfg.clock != BOARD_DAQ_CLOCK_SOFTWARE) || !powered())
  {
    return;
  }

  /* The generator is a SOURCE, in the converter's place: with a tone on,
     the meter is not read at all, so what the ring holds is arithmetic
     with a known answer and nothing of the board's analog front end. */
  if (s_tone_on)
  {
    Board_DaqTonePoll();
    return;
  }

  if (s_next_field == 0U)
  {
    s_pending_at = Board_Cycles();
    s_pending_digital = (s_cfg.digital != 0U) ? Board_DigitalMask() : 0U;
  }

  /* Not throttled. The loop runs at whatever the converter and the main
     loop manage, because that is what makes the accumulator's window worth
     having - and it is safe now for a reason that has nothing to do with
     rate: ONE poll used to read seven channels and take about 190 us, and
     RTU discards a frame whose characters arrive more than t1.5 apart,
     143 us at 115200. One channel per turn fixed that. Rate limiting never
     did - 200 Hz survived and 1000 Hz did not, because a single poll still
     overran t1.5 whenever it landed inside a frame. */
  if (!Board_AdcRead(s_order[s_next_field], &raw, &uv, &scaled))
  {
    return;                        /* the meter is busy; try again next turn */
  }
  s_pending[s_next_field] = raw;
  live_insert(s_next_field, raw, s_pending_at, s_pending_digital);

  if (++s_next_field >= s_fields)
  {
    s_next_field = 0U;

    /* CLOSED BY THE CLOCK: nothing gates the triggers. Every sweep the
       loop manages goes into the sum, and `interval_us` decides when the
       record is finished rather than when the next sample may be taken -
       which is the whole point of summing on the target. */
    if (s_cfg.accumulate == 0U)
    {
      feed(s_pending, s_pending_at, s_pending_digital);
      return;
    }

    /* Closed by a count: `interval_us` gates RECORDS by gating the
       triggers that make one. The ring is a capture and its rate is the
       link's business; the accumulator's is not. */
    if (s_interval_cycles != 0U)
    {
      const uint32_t now = Board_Cycles();

      if ((uint32_t)(now - s_last_trigger) < s_interval_cycles)
      {
        return;
      }
      s_last_trigger = now;
    }
    feed(s_pending, s_pending_at, s_pending_digital);
  }
}


void Board_DaqOnInjected(const board_sync_sample_t *sample)
{
  int32_t values[BOARD_DAQ_MAX_CHANNELS];

  if (!s_running || (s_cfg.clock != BOARD_DAQ_CLOCK_TIM1) ||
      (sample == NULL) || !powered())
  {
    return;
  }

  const uint32_t at = Board_Cycles();
  const uint32_t digital = (s_cfg.digital != 0U) ? Board_DigitalMask() : 0U;

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    values[f] = Board_AdcInjectedSlot(s_order[f], sample);
    live_insert(f, values[f], at, digital);
  }
  feed(values, at, digital);
}


uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records)
{
  uint16_t taken = 0U;

  if ((out == NULL) || (s_stride == 0U))
  {
    return 0U;
  }

  while ((taken < max_records) && (Board_DaqAvailable() > 0U))
  {
    for (uint16_t i = 0U; i < s_stride; i++)
    {
      out[(taken * s_stride) + i] = s_buf[s_tail];
      s_tail = (s_tail + 1U) % DAQ_BYTES;
    }
    taken++;
  }
  return taken;
}
