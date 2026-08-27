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
#include "wire.h"

/** What is left of MB_MAX_PDU once the count byte is spent. The reply is
    whole records only - half of one is not a short read, it is a corrupt
    one, and the host has no way to tell the difference. */
#define DAQ_REPLY_ROOM 240U


static cmd_status_t h_daq_state(wr_t *out)
{
  board_daq_state_t st;

  Board_DaqState(&st);

  wr_u8(out, (uint8_t)((st.running ? 0x01U : 0U) | (st.done ? 0x02U : 0U)));
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

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  wr_u8(out, Board_DaqConfigure(&cfg) ? 1U : 0U);
  return CMD_OK;
}


static cmd_status_t h_daq_start(wr_t *out)
{
  wr_u8(out, Board_DaqStart() ? 1U : 0U);
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
    default:               return CMD_ERR_VALUE;
  }
}
