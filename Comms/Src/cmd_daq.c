/**
  ******************************************************************************
  * @file    cmd_daq.c
  * @brief   The acquisition task behind command 0x6E, device 6.
  *
  * Configure, start, read - and `layout`, which is the one that matters.
  * A record is bytes with a stride the config decides, and the host decodes
  * it from what op 5 says rather than from a copy of the shape. Add a
  * channel to `board_adc.c` and a task that selects it describes itself; a
  * decoder written against a header would have needed telling.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "filter.h"
#include "dev_serial.h"
#include "wire.h"

/** What is left of MB_MAX_PDU once the count byte is spent. The reply is
    whole records only - half of one is not a short read, it is a corrupt
    one, and the host has no way to tell the difference. */
#define DAQ_REPLY_ROOM 240U


/** Coefficients cross as Q28: the wire has no floating point (PROTOCOL,
  * the header), and a biquad's a1 reaches -2, so a scale of 2^28 leaves
  * a range of +/-8 and a resolution of 4e-9 - three orders inside what
  * a float carries anyway. */
#define DAQ_COEFF_SHIFT 28
#define DAQ_COEFF_SCALE 268435456.0f


/** The link's own answer where a task asked for no rate.
  *
  * Free-running with no rate is the one combination that took the link
  * down, so it gets what the link can carry rather than "as fast as the
  * loop can". A finite run is left alone: it stops on its own, and a
  * short burst at full speed is the whole point of one.
  *
  * The ceiling is on RECORDS, and a record costs `decimate x accumulate
  * x the chain's decimation` triggers. Gating at the record rate instead
  * would sample sixteen times slower at accumulate 16 rather than
  * averaging sixteen samples - the same output, every sample but one
  * thrown away. Reduce on the target, do not slow it down.
  *
  * THE BOARD COUNTS THE TRIGGERS, because the filter's decimation is not
  * in the config - and this runs from BOTH configure and the filter op,
  * so the order they arrive in does not matter. Measured with it only in
  * configure, where the filter is not loaded yet: ten channels asked for
  * 62.8 records a second and made 8, the missing factor of 9 being the
  * chain's.
  */
static void daq_substitute_interval(void)
{
  board_daq_state_t st;

  Board_DaqState(&st);

  if (!Board_DaqRateIsAuto())
  {
    return;
  }

  const uint32_t rps = cmd_link_records_per_second(st.stride);
  const uint32_t per_record = Board_DaqTriggersPerRecord();

  if ((rps != 0U) && (per_record != 0U) && (rps < (1000000U / per_record)))
  {
    Board_DaqSetInterval(1000000U / (rps * per_record));
  }
  else
  {
    Board_DaqSetInterval(0U);      /* faster than the loop can go anyway */
  }
}


/** op 7 - the anti-alias chain the host designed. */
static cmd_status_t h_daq_filter(rd_t *in, wr_t *out)
{
  const uint8_t count = rd_u8(in);
  const uint16_t decimate = rd_u16(in);
  filter_biquad_t sections[FILTER_MAX_SECTIONS];

  if (count > FILTER_MAX_SECTIONS)
  {
    cmd_took(out, "the board runs four biquads - an eighth-order "
                  "Bessel. Ask the design for a lower order");
    return CMD_OK;
  }

  for (uint8_t i = 0U; i < count; i++)
  {
    sections[i].b0 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
  }

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = Board_DaqSetFilter(sections, count, decimate);

  if (refusal == NULL)
  {
    /* The chain changed what a record costs, so the rate the link was
       promised is worked out again. */
    daq_substitute_interval();
  }
  cmd_took(out, refusal);
  return CMD_OK;
}


/** op 9 - one rung of the ladder. */
static cmd_status_t h_daq_rung(rd_t *in, wr_t *out)
{
  const uint8_t rung = rd_u8(in);
  const uint16_t boxcar = rd_u16(in);
  const uint8_t count = rd_u8(in);
  const uint16_t decimate = rd_u16(in);
  filter_biquad_t sections[FILTER_MAX_SECTIONS];

  if (count > FILTER_MAX_SECTIONS)
  {
    cmd_took(out, "the board runs four biquads - an eighth-order "
                  "Bessel");
    return CMD_OK;
  }

  for (uint8_t i = 0U; i < count; i++)
  {
    sections[i].b0 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
  }

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = Board_DaqSetRung(rung, boxcar, sections, count,
                                         decimate);

  if ((refusal == NULL) && (rung == 0U))
  {
    daq_substitute_interval();
  }
  cmd_took(out, refusal);
  return CMD_OK;
}


/** op 8 - a known tone in the converter's place. */
static cmd_status_t h_daq_tone(rd_t *in, wr_t *out)
{
  const uint32_t hz = rd_u32(in);
  const uint32_t rate = rd_u32(in);
  const int32_t amplitude = rd_i32(in);
  const int32_t offset = rd_i32(in);
  /* Appended: a request without it is a sine, which is what the op
     meant before there was anything else to be. */
  const uint8_t kind = (rd_left(in) > 0U) ? rd_u8(in)
                                          : (uint8_t)BOARD_DAQ_TONE_SINE;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  cmd_took(out, Board_DaqSetTone(hz, rate, amplitude, offset, kind));
  return CMD_OK;
}


static cmd_status_t h_daq_state(wr_t *out)
{
  board_daq_state_t st;

  Board_DaqState(&st);

  wr_u8(out, (uint8_t)((st.running ? 0x01U : 0U) | (st.done ? 0x02U : 0U)
                     | (st.lost_power ? 0x04U : 0U)));
  wr_u16(out, st.stride);
  wr_u8(out, st.fields);
  wr_u32(out, st.available);
  wr_u32(out, st.produced);
  wr_u32(out, st.dropped);
  wr_u16(out, st.config.channels);
  wr_u8(out, st.config.clock);
  /* What the CONVERTER has, not what the task asked for. They are the same
     today because the task is the only caller of Board_AdcSetSampleTime, and
     that is exactly why the copy was worth removing: two records of one fact
     agree right up until they do not, and the stale one is the one on the
     wire. */
  wr_u8(out, Board_AdcSampleTime());
  wr_u16(out, st.config.decimate);
  wr_u16(out, st.config.accumulate);
  wr_u32(out, st.config.records);
  wr_u8(out, st.config.digital);
  wr_u32(out, st.config.interval_us);
  wr_u32(out, cmd_link_records_per_second(st.stride));
  /* Appended (MINOR 4): the buffer level as the board measures it -
     what the ring holds at this stride, and the fullest it has been. */
  wr_u32(out, st.capacity);
  wr_u32(out, st.worst);
  /* Which rung is running, how many there are, and how often it has
     moved - a host that sees `samples` change needs to know whether the
     board climbed or the task was reconfigured. */
  wr_u8(out, st.rung);
  wr_u8(out, st.rungs);
  wr_u32(out, st.rung_changes);
  wr_u32(out, st.triggers);
  return CMD_OK;
}


/** op 1 - configure. Refused while running: a task whose stride changed
  * under a half-drained buffer would hand out records of two shapes with
  * nothing in them to say which was which. */
static cmd_status_t h_daq_configure(rd_t *in, wr_t *out)
{
  board_daq_config_t cfg;

  cfg.channels = rd_u16(in);
  cfg.clock = rd_u8(in);
  cfg.sample_time = rd_u8(in);
  cfg.decimate = rd_u16(in);
  cfg.accumulate = rd_u16(in);
  cfg.records = rd_u32(in);
  cfg.digital = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  cfg.interval_us = (rd_left(in) > 0U) ? rd_u32(in) : 0U;
  /* Appended: a request without it does not adapt, which is what the
     op meant before there was a ladder to climb. */
  cfg.adapt = (rd_left(in) > 0U) ? rd_u8(in) : 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = Board_DaqConfigure(&cfg);

  if (refusal != NULL)
  {
    cmd_took(out, refusal);
    return CMD_OK;
  }

  daq_substitute_interval();

  cmd_took(out, NULL);
  return CMD_OK;
}


static cmd_status_t h_daq_start(wr_t *out)
{
  cmd_took(out, Board_DaqStart());
  return CMD_OK;
}


static cmd_status_t h_daq_stop(wr_t *out)
{
  Board_DaqStop();
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 4 - take whole records, oldest first. */
static cmd_status_t h_daq_read(rd_t *in, wr_t *out)
{
  board_daq_state_t st;

  Board_DaqState(&st);

  if (st.stride == 0U)
  {
    return CMD_ERR_DEVICE;         /* nothing configured to have a shape */
  }

  uint16_t fits = (uint16_t)(DAQ_REPLY_ROOM / st.stride);

  if (rd_left(in) > 0U)
  {
    const uint8_t want = rd_u8(in);

    if (!rd_ok(in))
    {
      return CMD_ERR_LENGTH;
    }
    if ((want != 0U) && (want < fits))
    {
      fits = want;
    }
  }

  uint8_t batch[DAQ_REPLY_ROOM];
  const uint16_t got = Board_DaqTake(batch, fits);

  wr_u8(out, (uint8_t)got);
  wr_bytes(out, batch, (uint16_t)(got * st.stride));

  /* THE BACKLOG, THE WAY A DAQ CARD ANSWERS ONE: what is still in the
     ring after this read, in the same transaction that took the
     records. A host pacing itself to the link needs the level it had
     AFTER its own read, and asking separately both costs a round trip
     and answers about a different moment.

     Appended, so a host that slices `got` records by the stride and
     stops sees exactly what it saw before. Worst case 1 + 240 + 4 =
     245 against the 252 the PDU leaves after the function code, so
     DAQ_REPLY_ROOM does not move and neither does the link's rate. */
  wr_u32(out, Board_DaqAvailable());
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 5 - what each field of a record is, named by the board.
  *
  * Field order is the channel table's order, so this and `0x6D` kind 0
  * agree by construction rather than by anyone keeping them in step.
  */
/** The digital word, named bit by bit. Without this the host would be
  * counting rows of a table it does not hold, which is the copy this
  * whole layout exists to avoid. Only the drivable pins: all twenty-three
  * came to 312 bytes against MB_MAX_PDU's 253 and the reply failed
  * outright. */
static cmd_status_t digital_rows(wr_t *out)
{
  const uint8_t pins = Board_DigitalSampledCount();

  wr_u8(out, pins);
  for (uint8_t i = 0U; i < pins; i++)
  {
    board_dchan_t d;

    if (!Board_DigitalSampledChan(i, &d))
    {
      return CMD_ERR_DEVICE;
    }
    wr_u8(out, d.dir);
    wr_str(out, d.signal);
  }
  return CMD_OK;
}

static cmd_status_t h_daq_layout(wr_t *out)
{
  board_daq_state_t st;

  Board_DaqState(&st);

  wr_u8(out, st.fields);
  wr_u16(out, st.stride);

  for (uint8_t f = 0U; f < st.fields; f++)
  {
    uint8_t index;
    board_chan_t info;

    if (!Board_DaqField(f, &index) || !Board_AdcChan(index, &info))
    {
      return CMD_ERR_DEVICE;
    }
    wr_u8(out, index);
    wr_u8(out, info.unit);
    wr_u8(out, (uint8_t)(info.differential ? 1U : 0U));
    wr_str(out, info.signal);
  }

  wr_u8(out, st.config.digital);
  if ((st.config.digital != 0U) && (digital_rows(out) != CMD_OK))
  {
    return CMD_ERR_DEVICE;
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 6 - the live accumulator, taken and reset.
  *
  * One reply whatever the rate: each channel carries its own count and the
  * span says over what. A late reader gets a wider window, not a backlog.
  *
  * `fresh` 0 means nothing arrived and the reply stops there. Blocking is the
  * caller's own business - a slave sitting on a reply would break RTU
  * framing for everyone on the segment.
  */
static cmd_status_t h_daq_live(wr_t *out)
{
  board_daq_state_t st;
  board_daq_live_t live;

  Board_DaqState(&st);
  Board_DaqTakeLive(&live);

  wr_u8(out, live.fresh ? 1U : 0U);

  if (!live.fresh)
  {
    return CMD_OK;
  }

  wr_u32(out, live.first);
  wr_u32(out, live.last);

  /* One sum AND one count per channel. The software poll reads one channel
     per turn of the main loop, so over any window they have had different
     numbers of samples and a single count would divide most of them by the
     wrong number. */
  for (uint8_t f = 0U; f < st.fields; f++)
  {
    wr_i32(out, live.slot[f].sum);
    wr_u32(out, live.slot[f].additions);
    /* What the channel did in the window, measured. A mean and a count
       cannot tell you a spike happened; these can, and it is the same two
       comparisons a meter would make anyway. */
    wr_i32(out, live.slot[f].lowest);
    wr_i32(out, live.slot[f].highest);
  }
  if (st.config.digital != 0U)
  {
    wr_u32(out, live.digital);
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


cmd_status_t cmd_daq_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case DAQ_OP_STATE:     return h_daq_state(out);
    case DAQ_OP_CONFIGURE: return h_daq_configure(in, out);
    case DAQ_OP_START:     return h_daq_start(out);
    case DAQ_OP_STOP:      return h_daq_stop(out);
    case DAQ_OP_READ:      return h_daq_read(in, out);
    case DAQ_OP_LAYOUT:    return h_daq_layout(out);
    case DAQ_OP_LIVE:      return h_daq_live(out);
    case DAQ_OP_FILTER:    return h_daq_filter(in, out);
    case DAQ_OP_TONE:      return h_daq_tone(in, out);
    case DAQ_OP_RUNG:      return h_daq_rung(in, out);
    default:               return CMD_ERR_VALUE;
  }
}
