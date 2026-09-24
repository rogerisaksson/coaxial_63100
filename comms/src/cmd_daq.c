/** cmd_daq.c - The acquisition task behind command 0x6E, device 6. */
#include "cmd.h"
#include "board.h"
#include "filter.h"
#include "dev_serial.h"
#include "wire.h"
#include "board_units.h"

/** What is left of MB_MAX_PDU once the count byte is spent. */
#define DAQ_REPLY_ROOM 240U

/** Coefficients cross as Q28: the wire has no floating point (PROTOCOL, the
    header), and a biquad's a1 reaches -2, so a scale of 2^28 leaves a range
    of +/-8 and a resolution of 4e-9 - three orders inside what a float
    carries anyway. */
#define DAQ_COEFF_SHIFT 28
#define DAQ_COEFF_SCALE 268435456.0f

/** The link's own answer where a task asked for no rate. */
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

  if ((rps != 0U) && (per_record != 0U) && (rps < (US_PER_S / per_record)))
  {
    Board_DaqSetInterval(US_PER_S / (rps * per_record));
  }
  else
  {
    Board_DaqSetInterval(0U);      /* faster than the loop can go anyway */
  }
}

/* `count` biquads off the wire, each coefficient an i32 over
   DAQ_COEFF_SCALE. */
static void rd_sections(rd_t *in, filter_biquad_t *sections, uint8_t count)
{
  for (uint8_t i = 0U; i < count; i++)
  {
    sections[i].b0 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].b2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a1 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
    sections[i].a2 = (float)rd_i32(in) / DAQ_COEFF_SCALE;
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
    wr_took(out, "the board runs four biquads - an eighth-order "
                  "Bessel. Ask the design for a lower order");
    return CMD_OK;
  }

  rd_sections(in, sections, count);

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
  wr_took(out, refusal);
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
    wr_took(out, "the board runs four biquads - an eighth-order "
                  "Bessel");
    return CMD_OK;
  }

  rd_sections(in, sections, count);

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
  wr_took(out, refusal);
  return CMD_OK;
}

/** op 8 - a known tone in the converter's place. */
static cmd_status_t h_daq_tone(rd_t *in, wr_t *out)
{
  const uint32_t hz = rd_u32(in);
  const uint32_t rate = rd_u32(in);
  const int32_t amplitude = rd_i32(in);
  const int32_t offset = rd_i32(in);
  /* Appended: a request without it is a sine, which is what the op meant
     before there was anything else to be. */
  const uint8_t kind = (rd_left(in) > 0U) ? rd_u8(in)
                                          : (uint8_t)BOARD_DAQ_TONE_SINE;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  wr_took(out, Board_DaqSetTone(hz, rate, amplitude, offset, kind));
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
  /* What the converter has, not what the task asked for. */
  wr_u8(out, Board_AdcSampleTime());
  wr_u16(out, st.config.decimate);
  wr_u16(out, st.config.accumulate);
  wr_u32(out, st.config.records);
  wr_u8(out, st.config.digital);
  wr_u32(out, st.config.interval_us);
  wr_u32(out, cmd_link_records_per_second(st.stride));
  /* Appended (MINOR 4): the buffer level as the board measures it - what the
     ring holds at this stride, and the fullest it has been. */
  wr_u32(out, st.capacity);
  wr_u32(out, st.worst);
  /* Which rung is running, how many there are, and how often it has moved -
     a host that sees `samples` change needs to know whether the board
     climbed or the task was reconfigured. */
  wr_u8(out, st.rung);
  wr_u8(out, st.rungs);
  wr_u32(out, st.rung_changes);
  wr_u32(out, st.triggers);
  /* Appended, MINOR 7: what the task carries and what this build can carry -
     the mask the catalogue's `selectable` answers from. */
  wr_u16(out, st.config.sensors);
  wr_u16(out, (uint16_t)((1U << BOARD_DAQ_MAX_SENSORS) - 1U));
  return CMD_OK;
}

/** op 1 - configure. */
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
  /* Appended: a request without it does not adapt, which is what the op
     meant before there was a ladder to climb. */
  cfg.adapt = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  /* Appended, MINOR 7: the sensor snapshot mask. */
  cfg.sensors = (rd_left(in) > 0U) ? rd_u16(in) : 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = Board_DaqConfigure(&cfg);

  if (refusal != NULL)
  {
    wr_took(out, refusal);
    return CMD_OK;
  }

  daq_substitute_interval();

  wr_took(out, NULL);
  return CMD_OK;
}

static cmd_status_t h_daq_start(wr_t *out)
{
  wr_took(out, Board_DaqStart());
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
  const bool given = rd_left(in) > 0U;
  const uint8_t want = given ? rd_u8(in) : 0U;

  if (given && !rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((want != 0U) && (want < fits))
  {
    fits = want;
  }

  uint8_t batch[DAQ_REPLY_ROOM];
  const uint16_t got = Board_DaqTake(batch, fits);

  wr_u8(out, (uint8_t)got);
  wr_bytes(out, batch, (uint16_t)(got * st.stride));

  /* The backlog, the way a DAQ card answers one: what is still in the ring
     after this read, in the same transaction that took the records. */
  wr_u32(out, Board_DaqAvailable());
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}

/** op 5 - what each field of a record is, named by the board. */
/** The digital word, named bit by bit. */
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

  /* Appended, MINOR 7: the sensor rows, in bit order. */
  {
    static const char *const names[BOARD_DAQ_MAX_SENSORS] = {
      "orientation", "acceleration", "rotation rate", "magnetic field",
      "shaft angle",
    };
    uint8_t count = 0U;

    for (uint8_t b = 0U; b < BOARD_DAQ_MAX_SENSORS; b++)
    {
      count = (uint8_t)(count + ((st.config.sensors >> b) & 1U));
    }
    wr_u8(out, count);
    for (uint8_t b = 0U; b < BOARD_DAQ_MAX_SENSORS; b++)
    {
      if ((st.config.sensors & (1U << b)) != 0U)
      {
        wr_u8(out, b);
        wr_u8(out, 4U);              /* words per field, all of them */
        wr_str(out, names[b]);
      }
    }
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}

/** op 6 - the live accumulator, taken and reset. */
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

  /* One sum and one count per channel. */
  for (uint8_t f = 0U; f < st.fields; f++)
  {
    wr_i32(out, live.slot[f].sum);
    wr_u32(out, live.slot[f].additions);
    /* What the channel did in the window, measured. */
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
