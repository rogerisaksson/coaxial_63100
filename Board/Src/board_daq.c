/**
  ******************************************************************************
  * @file    board_daq.c
  * @brief   One acquisition task: configure, start, read. DAQmx's shape, cut
  *          down to what this board actually has.
  *
  * There is one task, not many. A card with its own sequencer can run several;
  * this is one MCU with three converters and one timer, and pretending
  * otherwise would put the arbitration somewhere it cannot be honoured.
  *
  * What a task owns:
  *
  *   channels     which of the ADC table's rows, as a bitmask
  *   clock        SOFTWARE - the main loop, as fast as it gets round
  *                TIM1     - the injected group, one record per PWM period
  *   sample_time  0..7, the converter's own sampling window
  *   decimate     keep one trigger in N
  *   accumulate   sum N samples into each record before it is pushed
  *   records      stop after this many, or 0 to run until stopped
  *
  * The buffer is bytes, not a struct, and the stride is computed from the
  * config: `u32 at` then one `i32` per enabled channel. That is why a host
  * needs no copy of the record shape - it asks for the layout and the board
  * names every field, the same way `0x6D` names the channels. A record laid
  * out in a header here and mirrored in a decoder there is two answers to
  * one question, and the mirror is the one that goes stale.
  *
  * Accumulation sums, it does not average. Summing keeps the bits an average
  * would throw away, and a host that wants the mean has the count.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include <string.h>

/** 16 KB of DTCM. At one channel that is 2048 records, at all seven 512. */
#define DAQ_BYTES 16384U

static uint8_t  s_buf[DAQ_BYTES];
static volatile uint32_t s_head;        /* byte offset of the next write */
static volatile uint32_t s_tail;        /* byte offset of the next read  */
static volatile uint32_t s_dropped;
static volatile uint32_t s_produced;

static board_daq_config_t s_cfg;
static volatile bool s_running;
static volatile bool s_done;

static uint16_t s_stride;
static uint8_t  s_order[BOARD_DAQ_MAX_CHANNELS];  /* channel index per field */
static uint8_t  s_fields;

/* Accumulator, reset every time a record is pushed. */
static int32_t  s_acc[BOARD_DAQ_MAX_CHANNELS];
static uint16_t s_acc_n;
static uint16_t s_skip;
static uint32_t s_first_at;
static uint32_t s_first_digital;
static uint32_t s_last_trigger;
static uint32_t s_interval_cycles;

/* The software poll reads ONE channel per turn of the main loop, so a
   record is assembled across several. That is not a compromise on
   simultaneity - a software clock reads the channels one after another
   whatever it does - and it is what keeps the loop responsive. */
/* The live accumulator, kept beside the ring and fed by the same triggers.
   The ring is a capture and drops when it is full; this is the freshest
   average and cannot drop, because a reader that is late just gets a wider
   window. Separate totals so the two do not consume each other. */
static int32_t  s_live[BOARD_DAQ_MAX_CHANNELS];
static uint32_t s_live_n;
static uint32_t s_live_first;
static uint32_t s_live_last;
static uint32_t s_live_digital;

static uint8_t  s_next_field;
static int32_t  s_pending[BOARD_DAQ_MAX_CHANNELS];
static uint32_t s_pending_at;
static uint32_t s_pending_digital;


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


static void push_record(void)
{
  uint8_t rec[4U + (4U * BOARD_DAQ_MAX_CHANNELS) + 4U];
  uint16_t at = put_be32(rec, 0U, s_first_at);

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    at = put_be32(rec, at, (uint32_t)s_acc[f]);
  }

  if (s_cfg.digital != 0U)
  {
    at = put_be32(rec, at, s_first_digital);
  }

  const uint32_t masked = __get_PRIMASK();
  __disable_irq();

  if (room() > s_stride)
  {
    put(rec, s_stride);
    s_produced++;
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
  s_acc_n = 0U;

  if ((s_cfg.records != 0U) && (s_produced >= s_cfg.records))
  {
    s_running = false;
    s_done = true;
  }
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

  if (s_acc_n == 0U)
  {
    s_first_at = at;

    /* The pins as they stood at `at`, not summed and not OR-ed across the
       window. Summing a bitmask means nothing, and an OR would report a
       pin as high that was high for one sample in fifty with no field
       saying which. With accumulate at 1 this is every sample. */
    s_first_digital = digital;
  }

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    s_acc[f] += values[f];
    s_live[f] += values[f];
  }

  if (s_live_n == 0U)
  {
    s_live_first = at;
  }
  s_live_n++;
  s_live_last = at;
  s_live_digital = digital;

  if (++s_acc_n >= s_cfg.accumulate)
  {
    push_record();
  }
}


bool Board_DaqConfigure(const board_daq_config_t *cfg)
{
  if ((cfg == NULL) || s_running)
  {
    return false;
  }
  if ((cfg->clock != BOARD_DAQ_CLOCK_SOFTWARE) &&
      (cfg->clock != BOARD_DAQ_CLOCK_TIM1))
  {
    return false;
  }
  if ((cfg->decimate == 0U) || (cfg->accumulate == 0U))
  {
    return false;
  }
  if (!Board_AdcSetSampleTime(cfg->sample_time))
  {
    return false;
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
    return false;
  }

  /* TIM1 clock means the injected group, and that group converts the three
     phases and nothing else. Asking for a channel it does not carry is a
     configuration that cannot be honoured, so it is refused here rather
     than answered with zeros. */
  if (cfg->clock == BOARD_DAQ_CLOCK_TIM1)
  {
    for (uint8_t f = 0U; f < s_fields; f++)
    {
      if (!Board_AdcIsPhase(s_order[f]))
      {
        return false;
      }
    }
  }

  s_cfg = *cfg;
  s_interval_cycles = cfg->interval_us * (SystemCoreClock / 1000000U);
  s_stride = (uint16_t)(4U + (4U * s_fields) + ((cfg->digital != 0U) ? 4U : 0U));
  s_head = 0U;
  s_tail = 0U;
  s_dropped = 0U;
  s_produced = 0U;
  s_done = false;
  s_skip = 0U;
  s_next_field = 0U;
  s_acc_n = 0U;
  s_live_n = 0U;
  memset(s_live, 0, sizeof(s_live));
  memset(s_acc, 0, sizeof(s_acc));
  return true;
}


void Board_DaqSetInterval(uint32_t interval_us)
{
  s_cfg.interval_us = interval_us;
  s_interval_cycles = interval_us * (SystemCoreClock / 1000000U);
}


bool Board_DaqStart(void)
{
  if ((s_stride == 0U) || s_running)
  {
    return false;
  }
  s_head = 0U;
  s_tail = 0U;
  s_dropped = 0U;
  s_produced = 0U;
  s_done = false;
  s_skip = 0U;
  s_next_field = 0U;
  s_acc_n = 0U;
  s_live_n = 0U;
  memset(s_live, 0, sizeof(s_live));
  memset(s_acc, 0, sizeof(s_acc));
  s_running = true;
  return true;
}


void Board_DaqStop(void)
{
  s_running = false;
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

  out->fresh = (s_live_n != 0U);
  out->count = s_live_n;
  out->first = s_live_first;
  out->last = s_live_last;
  out->digital = s_live_digital;
  for (uint8_t f = 0U; f < BOARD_DAQ_MAX_CHANNELS; f++)
  {
    out->sum[f] = s_live[f];
    s_live[f] = 0;
  }
  s_live_n = 0U;

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
  out->stride = s_stride;
  out->fields = s_fields;
  out->available = Board_DaqAvailable();
  out->produced = s_produced;
  out->dropped = s_dropped;
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


void Board_DaqPoll(void)
{
  int32_t raw;
  int32_t uv;
  int32_t scaled;

  if (!s_running || (s_cfg.clock != BOARD_DAQ_CLOCK_SOFTWARE))
  {
    return;
  }

  if (s_next_field == 0U)
  {
    /* A software clock has to BE a clock. Left to run at whatever the loop
       has spare it took the link down, and rate limiting alone did not fix
       it: ONE poll of seven channels is about 190 us of converter work, and
       RTU discards a frame whose characters arrive more than t1.5 apart -
       143 us at 115200. Hence one channel per turn below, and a stated
       interval here. Zero is unlimited, which is only safe for a short
       finite run. */
    const uint32_t now = Board_Cycles();

    if ((s_interval_cycles != 0U) &&
        ((uint32_t)(now - s_last_trigger) < s_interval_cycles))
    {
      return;
    }
    s_last_trigger = now;
    s_pending_at = now;
    s_pending_digital = (s_cfg.digital != 0U) ? Board_DigitalMask() : 0U;
  }

  if (!Board_AdcRead(s_order[s_next_field], &raw, &uv, &scaled))
  {
    return;                        /* the meter is busy; try again next turn */
  }
  s_pending[s_next_field] = raw;

  if (++s_next_field >= s_fields)
  {
    s_next_field = 0U;
    feed(s_pending, s_pending_at, s_pending_digital);
  }
}


void Board_DaqOnInjected(const int16_t *phase)
{
  int32_t values[BOARD_DAQ_MAX_CHANNELS];

  if (!s_running || (s_cfg.clock != BOARD_DAQ_CLOCK_TIM1) || (phase == NULL))
  {
    return;
  }

  for (uint8_t f = 0U; f < s_fields; f++)
  {
    values[f] = Board_AdcPhaseSlot(s_order[f], phase);
  }
  feed(values, Board_Cycles(),
       (s_cfg.digital != 0U) ? Board_DigitalMask() : 0U);
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
