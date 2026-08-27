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
#include "dev_serial.h"
#include "wire.h"

/** What is left of MB_MAX_PDU once the count byte is spent. The reply is
    whole records only - half of one is not a short read, it is a corrupt
    one, and the host has no way to tell the difference. */
#define DAQ_REPLY_ROOM 240U


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
  wr_u8(out, st.config.channels);
  wr_u8(out, st.config.clock);
  wr_u8(out, st.config.sample_time);
  wr_u16(out, st.config.decimate);
  wr_u16(out, st.config.accumulate);
  wr_u32(out, st.config.records);
  wr_u8(out, st.config.digital);
  wr_u32(out, st.config.interval_us);
  wr_u32(out, cmd_link_records_per_second(st.stride));
  return CMD_OK;
}


/** op 1 - configure. Refused while running: a task whose stride changed
  * under a half-drained buffer would hand out records of two shapes with
  * nothing in them to say which was which. */
static cmd_status_t h_daq_configure(rd_t *in, wr_t *out)
{
  board_daq_config_t cfg;

  cfg.channels = rd_u8(in);
  cfg.clock = rd_u8(in);
  cfg.sample_time = rd_u8(in);
  cfg.decimate = rd_u16(in);
  cfg.accumulate = rd_u16(in);
  cfg.records = rd_u32(in);
  cfg.digital = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  cfg.interval_us = (rd_left(in) > 0U) ? rd_u32(in) : 0U;

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

  /* Free-running and no rate asked for is the one combination that took the
     link down, so it gets the link's own answer rather than "as fast as the
     loop can". A finite run is left alone: it stops on its own, and a short
     burst at full speed is the whole point of one. */
  if ((cfg.interval_us == 0U) && (cfg.records == 0U))
  {
    board_daq_state_t st;

    Board_DaqState(&st);

    /* The link's ceiling is on RECORDS, and a record is decimate x
       accumulate triggers. Gating the triggers at the record rate would
       have sampled sixteen times slower with accumulate at 16 instead of
       averaging sixteen samples - the same output rate, and every sample
       but one thrown away. Reduce on the target, do not slow it down. */
    const uint32_t rps = cmd_link_records_per_second(st.stride);
    const uint32_t per_record = (uint32_t)cfg.decimate *
                                (uint32_t)cfg.accumulate;

    if ((rps != 0U) && (per_record != 0U) && (rps < (1000000U / per_record)))
    {
      Board_DaqSetInterval(1000000U / (rps * per_record));
    }
    else
    {
      Board_DaqSetInterval(0U);    /* faster than the loop can go anyway */
    }
  }

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
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 5 - what each field of a record is, named by the board.
  *
  * Field order is the channel table's order, so this and `0x6D` kind 0
  * agree by construction rather than by anyone keeping them in step.
  */
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

  /* The digital word, named bit by bit. Without this the host would be
     counting rows of a table it does not hold, which is the copy this whole
     layout exists to avoid. Only the drivable pins: all twenty-three came
     to 312 bytes against MB_MAX_PDU's 253 and the reply failed outright. */
  wr_u8(out, st.config.digital);

  if (st.config.digital != 0U)
  {
    const uint8_t pins = Board_DigitalIoCount();

    wr_u8(out, pins);
    for (uint8_t i = 0U; i < pins; i++)
    {
      board_dchan_t d;

      if (!Board_DigitalIoChan(i, &d))
      {
        return CMD_ERR_DEVICE;
      }
      wr_u8(out, d.dir);
      wr_str(out, d.signal);
    }
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 6 - the live accumulator, taken and reset.
  *
  * One reply whatever the sampling rate: each channel carries its own count
  * of how many went into its sum, and the span says over what. A late reader gets a wider window rather
  * than a backlog, so this path cannot overflow and has nothing to drop -
  * which is the difference between it and the ring.
  *
  * `fresh` is 0 when nothing has arrived since the last take, and the reply
  * stops there. A caller that wants to block does it on its own side: this
  * is a request/response protocol, and a slave that sat on a reply waiting
  * for a sample would break RTU framing for everyone on the segment.
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
    default:               return CMD_ERR_VALUE;
  }
}
