/**
  ******************************************************************************
  * @file    board_daq.c
  * @brief   One acquisition task: configure, start, read. DAQmx's shape, cut
  *          down to what this board has.
  ******************************************************************************
  */
#include "board_limits.h"
#include "board.h"
#include "board_irq.h"
#include "board_hw.h"
#include "board_units.h"
#include "daq.h"

#include <string.h>

/* The engine's sizes are the wire's, and the wire's are board.h's. */
_Static_assert(DAQ_MAX_CHANNELS == BOARD_DAQ_MAX_CHANNELS,
               "daq.h's channel count and board.h's disagree");
_Static_assert(DAQ_MAX_PINS == BOARD_DAQ_MAX_PINS,
               "daq.h's pin count and board_limits.h's disagree");
_Static_assert(DAQ_MAX_SENSORS == BOARD_DAQ_MAX_SENSORS,
               "daq.h's sensor count and board.h's disagree");
_Static_assert(DAQ_LADDER == BOARD_DAQ_LADDER,
               "daq.h's ladder and board_limits.h's disagree");
_Static_assert(DAQ_MAX_ADDITIONS == LIVE_MAX_ADDITIONS,
               "daq.h's saturation bound and board_limits.h's disagree");
_Static_assert((DAQ_TONE_SINE == BOARD_DAQ_TONE_SINE)
               && (DAQ_TONE_RAMP == BOARD_DAQ_TONE_RAMP),
               "daq.h's tone kinds and board.h's disagree");

/* `.buffers` is the AXI SRAM section in STM32H753xx_FLASH.ld. */
static uint8_t s_buf[DAQ_BYTES] __attribute__((section(".buffers")));

/** The acquisition glue's state: the engine, whether it is built yet, the
    task as the wire gave it, and whether the rate was chosen or the
    reference lost. */
static struct
{
  daq_t daq;
  bool built;
  board_daq_config_t cfg;         /**< the task as the wire gave it     */
  bool rate_auto;                 /**< no rate asked for: one was chosen*/
  volatile bool lost_power;
} s;

#define IMU_SENSORS  0x0FU             /* the four sensor bits the IMU serves */
#define SHAFT_SENSOR 4U

/** One sensor field's four words, SNAPSHOT at the record's close - raw and
    source-defined, the scale stays the host's as everywhere else. */
static void snapshot(void *ctx, uint16_t sensors,
                     int16_t words[DAQ_MAX_SENSORS][DAQ_SENSOR_WORDS])
{
  board_imu_state_t imu = {0};
  board_angle_state_t angle;

  (void)ctx;
  if ((sensors & IMU_SENSORS) != 0U)
  {
    Board_ImuState(&imu);
  }
  if ((sensors & (1U << SHAFT_SENSOR)) != 0U)
  {
    Board_AngleState(&angle);
    words[SHAFT_SENSOR][0] = (int16_t)angle.value;
    words[SHAFT_SENSOR][1] = (int16_t)angle.crc;
    words[SHAFT_SENSOR][2] = (int16_t)angle.reg;
    words[SHAFT_SENSOR][3] = angle.have ? 1 : 0;
  }
  words[0][0] = imu.i;        words[0][1] = imu.j;
  words[0][2] = imu.k;        words[0][3] = imu.real;
  words[1][0] = imu.accel[0]; words[1][1] = imu.accel[1];
  words[1][2] = imu.accel[2]; words[1][3] = (int16_t)imu.accel_status;
  words[2][0] = imu.gyro[0];  words[2][1] = imu.gyro[1];
  words[2][2] = imu.gyro[2];  words[2][3] = (int16_t)imu.gyro_status;
  words[3][0] = imu.mag[0];   words[3][1] = imu.mag[1];
  words[3][2] = imu.mag[2];   words[3][3] = (int16_t)imu.mag_status;
}

/** The engine, built on first use - the lazy shape the rest of board/ uses,
    since any of this file's entry points may be the first. */
static daq_t *engine(void)
{
  if (!s.built)
  {
    static const daq_guard_t guard = { Board_IrqHold, Board_IrqRelease };
    static const daq_ladder_t ladder = {
      BOARD_DAQ_CLIMB_AT, BOARD_DAQ_FALL_AT, BOARD_DAQ_RUNG_EIGHTHS,
      BOARD_DAQ_CLIMB_MAX, BOARD_DAQ_FALL_AFTER };

    daq_init(&s.daq, s_buf, DAQ_BYTES, &guard, snapshot, NULL);
    daq_set_ladder(&s.daq, &ladder);
    s.built = true;
  }
  return &s.daq;
}

/** Cycles for a microsecond interval, saturated. */
static uint32_t interval_cycles(uint32_t interval_us)
{
  const uint64_t cycles = (uint64_t)interval_us
                          * (uint64_t)(SystemCoreClock / US_PER_S);

  return (cycles > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;
}

static uint32_t digital_now(void)
{
  return (s.cfg.digital != 0U) ? Board_DigitalMask() : 0U;
}

uint32_t Board_DaqAvailable(void)
{
  return daq_available(engine());
}

/** True when the injected sequence converts every selected field: the three
    phases, and the DC link and NTC that ride rank 2. */
static bool only_injected(const daq_t *d)
{
  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    if (!Board_AdcInjected(d->task.order[f]))
    {
      return false;
    }
  }
  return true;
}

/** The checks that need no field list. */
static const char *refused_before_fields(const board_daq_config_t *cfg)
{
  if (cfg == NULL)
  {
    return "no configuration given - pass one";
  }
  if (engine()->running)
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
  if ((uint64_t)cfg->interval_us * (uint64_t)(SystemCoreClock / US_PER_S)
      > (uint64_t)UINT32_MAX)
  {
    return "interval_us is more than the cycle counter can express - it is "
           "32 bits at 475 MHz and comes round every 9.04 s. Sample faster "
           "and decimate, or drive it from the host";
  }
  if (cfg->accumulate > DAQ_MAX_ADDITIONS)
  {
    /* The record's sum is int32 and a single-ended code reaches 65535, so
       beyond this the sum wraps and the host divides a negative by the count
       and calls it a mean. */
    return "accumulate is at most 32767 - beyond that the record's sum "
           "overflows a signed 32-bit total and stops being a measurement";
  }
  if (!Board_AdcSetSampleTime(cfg->sample_time))
  {
    return "sample_time is 0 to 7, shortest window first";
  }
  return NULL;
}

/** Field order is the channel table's order, so a host reading the layout
    and a host reading `0x6D` get the same answer in the same sequence. */
static uint8_t select_fields(uint16_t mask, uint8_t *order)
{
  const uint8_t rows = Board_AdcCount();
  uint8_t count = 0U;

  for (uint8_t i = 0U; (i < rows) && (i < DAQ_MAX_CHANNELS); i++)
  {
    if ((mask & (1U << i)) != 0U)
    {
      order[count++] = i;
    }
  }
  return count;
}

/** The checks over the fields just selected. NULL when they all pass. */
static const char *refused_with_fields(const board_daq_config_t *cfg,
                                       const daq_t *d)
{
  if (d->task.fields == 0U)
  {
    return "no channels selected - the mask is over the rows of 0x6D kind 0";
  }

  /* TIM1 clock means the injected group, and that group converts the three
     phases and nothing else. */
  if ((cfg->clock == BOARD_DAQ_CLOCK_TIM1) && !only_injected(d))
  {
    return "the TIM1 clock carries what the injected sequence converts - "
           "the three phases, and the DC link and the NTC on rank 2. Any "
           "other channel has to come through the meter on the software "
           "clock";
  }
  if ((cfg->accumulate == 0U) && d->filtering)
  {
    return "a clock-closed record and a filter are alternatives: a "
           "fixed-rate filter needs a fixed decimation, and a window's "
           "length is whatever the loop managed. Ask for an accumulate, "
           "or clear the filter";
  }
  if ((cfg->sensors >> DAQ_MAX_SENSORS) != 0U)
  {
    return "the sensor mask has five bits: orientation, acceleration, "
           "rotation rate, magnetic field, shaft angle";
  }
  if ((cfg->sensors != 0U) && (cfg->clock == BOARD_DAQ_CLOCK_TIM1))
  {
    return "sensor fields ride the software clock only - the poll "
           "records belong to the main loop, and a TIM1-clocked record "
           "closes inside ADC3's interrupt, which would read them torn";
  }
  return NULL;
}

const char *Board_DaqConfigure(const board_daq_config_t *cfg)
{
  const char *refusal = refused_before_fields(cfg);

  if (refusal != NULL)
  {
    return refusal;
  }

  daq_t *d = engine();
  daq_task_t task;

  memset(&task, 0, sizeof(task));
  task.fields = select_fields(cfg->channels, task.order);
  /* The refusals over the fields read them off the engine's task, so the
     order is set before they run and the rest only once they pass. */
  d->task.fields = task.fields;
  memcpy(d->task.order, task.order, sizeof(task.order));
  refusal = refused_with_fields(cfg, d);
  if (refusal != NULL)
  {
    return refusal;
  }

  task.accumulate = cfg->accumulate;
  task.decimate = cfg->decimate;
  task.interval_cycles = interval_cycles(cfg->interval_us);
  task.pins = (cfg->digital != 0U) ? Board_DigitalSampledCount() : 0U;
  task.sensors = cfg->sensors;
  task.records = cfg->records;
  task.adapt = (cfg->adapt != 0U);
  daq_begin(d, &task);

  s.cfg = *cfg;
  /* Remembered here, where the ASK is still visible. */
  s.rate_auto = (cfg->interval_us == 0U) && (cfg->records == 0U);
  s.lost_power = false;
  return NULL;
}

const char *Board_DaqSetFilter(const void *sections, uint8_t count,
                               uint16_t decimate)
{
  return daq_set_filter(engine(), (const filter_biquad_t *)sections, count,
                        decimate);
}

const char *Board_DaqSetRung(uint8_t rung, uint16_t boxcar,
                             const void *sections, uint8_t count,
                             uint16_t decimate)
{
  return daq_set_rung(engine(), rung, boxcar,
                      (const filter_biquad_t *)sections, count, decimate);
}

const char *Board_DaqSetTone(uint32_t hz, uint32_t rate_hz,
                             int32_t amplitude, int32_t offset,
                             uint8_t kind)
{
  return daq_set_tone(engine(), hz, rate_hz, amplitude, offset, kind,
                      SystemCoreClock, Board_Cycles());
}

void Board_DaqTonePoll(void)
{
  if (s.cfg.clock != BOARD_DAQ_CLOCK_SOFTWARE)
  {
    return;
  }
  /* The burst is bounded because this runs beside the link: RTU discards a
     frame whose characters arrive more than t1.5 apart, and the bound was
     measured against exactly that (board_limits.h). */
  daq_tone_poll(engine(), Board_Cycles(), BOARD_DAQ_TONE_BURST);
}

bool Board_DaqToneOn(void)
{
  return engine()->tone.on;
}

bool Board_DaqRateIsAuto(void)
{
  return s.rate_auto;
}

uint32_t Board_DaqTriggersPerRecord(void)
{
  return daq_triggers_per_record(engine());
}

void Board_DaqSetInterval(uint32_t interval_us)
{
  s.cfg.interval_us = interval_us;
  engine()->task.interval_cycles = interval_cycles(interval_us);
}

const char *Board_DaqStart(void)
{
  daq_t *d = engine();

  if (d->stride == 0U)
  {
    return "nothing configured to start - configure the task first";
  }
  if (d->running)
  {
    return "already running - stop it first, or leave it be";
  }
  if (!Board_AfeOn())
  {
    return "AFE_ON is off, and it powers the converter's reference - every "
           "channel would read exact mid-scale, which is not a measurement";
  }
  s.lost_power = false;
  (void)daq_start(d, Board_Cycles());
  return NULL;
}

void Board_DaqStop(void)
{
  daq_stop(engine());
}

void Board_DaqTakeLive(board_daq_live_t *out)
{
  daq_live_t live;

  if (out == NULL)
  {
    return;
  }
  daq_take_live(engine(), &live);
  out->fresh = live.fresh;
  out->first = live.first;
  out->last = live.last;
  out->digital = live.digital;
  for (uint8_t f = 0U; f < BOARD_DAQ_MAX_CHANNELS; f++)
  {
    out->slot[f].sum = live.slot[f].sum;
    out->slot[f].additions = live.slot[f].additions;
    out->slot[f].lowest = live.slot[f].lowest;
    out->slot[f].highest = live.slot[f].highest;
  }
}

void Board_DaqState(board_daq_state_t *out)
{
  if (out == NULL)
  {
    return;
  }

  const daq_t *d = engine();

  out->running = d->running;
  out->done = d->done;
  out->lost_power = s.lost_power;
  out->stride = d->stride;
  out->fields = d->task.fields;
  out->available = daq_available(d);
  out->produced = d->produced;
  out->dropped = d->dropped;
  /* What the ring holds at this stride, not what it holds in bytes: a level
     is a fraction of something, and DAQ_BYTES is not what a host counts
     records against. */
  out->capacity = daq_capacity(d);
  out->worst = d->worst;
  out->rung = d->rung;
  out->rungs = d->rungs_held;
  out->rung_changes = d->rung_changes;
  out->triggers = d->triggers;
  out->config = s.cfg;
  /* The accumulate is the rung's while a ladder runs: what the engine is
     summing, not what the wire first asked for. */
  out->config.accumulate = d->task.accumulate;
}

bool Board_DaqField(uint8_t field, uint8_t *channel)
{
  const daq_t *d = engine();

  if ((field >= d->task.fields) || (channel == NULL))
  {
    return false;
  }
  *channel = d->task.order[field];
  return true;
}

/** AFE_ON off means every channel reads exact mid-scale - it powers the ADC
    reference, not just the signal path (invariant 9). */
static bool powered(daq_t *d)
{
  if (Board_AfeOn())
  {
    return true;
  }
  if (d->running)
  {
    daq_lose_power(d);
    s.lost_power = true;
  }
  return false;
}

void Board_DaqPoll(void)
{
  daq_t *d = engine();
  int32_t raw;
  int32_t uv;
  int32_t scaled;

  if (!d->running || (s.cfg.clock != BOARD_DAQ_CLOCK_SOFTWARE) || !powered(d))
  {
    return;
  }

  /* The generator is a SOURCE, in the converter's place: with a tone on, the
     meter is not read at all, so what the ring holds is arithmetic with a
     known answer and nothing of the board's analog front end. */
  if (d->tone.on)
  {
    Board_DaqTonePoll();
    return;
  }

  if (d->next_field == 0U)
  {
    daq_sweep_begin(d, Board_Cycles(), digital_now());
  }

  /* Not throttled. */
  if (!Board_AdcRead(d->task.order[d->next_field], &raw, &uv, &scaled))
  {
    return;                        /* the meter is busy; try again next turn */
  }
  daq_live_insert(d, d->next_field, raw, d->pending_at, d->pending_digital);
  if (!daq_sweep_put(d, raw))
  {
    return;                        /* more fields to read this sweep */
  }
  daq_sweep_close(d, Board_Cycles());
}

void Board_DaqOnInjected(const board_sync_sample_t *sample)
{
  daq_t *d = engine();
  int32_t values[DAQ_MAX_CHANNELS];

  if (!d->running || (s.cfg.clock != BOARD_DAQ_CLOCK_TIM1) ||
      (sample == NULL) || !powered(d))
  {
    return;
  }

  const uint32_t at = Board_Cycles();
  const uint32_t digital = digital_now();

  for (uint8_t f = 0U; f < d->task.fields; f++)
  {
    values[f] = Board_AdcInjectedSlot(d->task.order[f], sample);
    daq_live_insert(d, f, values[f], at, digital);
  }
  daq_feed(d, values, at, digital);
}

uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records)
{
  return daq_take(engine(), out, max_records);
}
