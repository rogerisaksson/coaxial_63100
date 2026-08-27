/**
  ******************************************************************************
  * @file    cmd_log.c
  * @brief   The measurement ring's operations behind command 0x6E, device 5.
  *
  * Buffered reads, and that is the whole point. One sample per round trip
  * caps a host at a couple of hundred samples a second whatever the board
  * managed; fifteen per reply and the board's own rate is the only limit
  * left.
  *
  * A record goes out as 14 bytes - `u32 at, u8 source, u8 seq, i16 v[4]` -
  * rather than the 16 it occupies in RAM, because the padding is the
  * compiler's business and not the wire's.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

/** Wire size of one record. 15 of them plus the count is 211 bytes, inside
    MB_MAX_PDU's 253 with room for the function code and the unit. */
#define LOG_RECORD_BYTES 14U
#define LOG_MAX_BURST    15U


static cmd_status_t h_log_state(wr_t *out)
{
  wr_u8(out, Board_LogSources());
  wr_u16(out, Board_LogCount());
  wr_u16(out, (uint16_t)BOARD_LOG_DEPTH);
  wr_u32(out, Board_LogDropped());
  /* Appended, so an older host reads everything before it unchanged.
     Separate from `dropped` because they mean opposite things: dropped is
     a sample the ring had no room for, thinned is one it declined to take
     because the link could not have carried it anyway. */
  wr_u32(out, Board_LogThinned());
  return CMD_OK;
}


/** op 1 - arm a bitmask of sources and empty the ring.
  *
  * Emptying is not optional: a burst whose first records predate the run is
  * worse than an empty one, and no field in the record would say so.
  */
static cmd_status_t h_log_arm(rd_t *in, wr_t *out)
{
  const uint8_t sources = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  /* Each armed source gets an equal share of what the link can drain. The
     IMU reports at 50 Hz and never reaches its share; the angle loop polls
     at about 24 kHz and is held to it, which is the whole fix - measured,
     the IMU went from 1 record a second to its full rate. */
  uint8_t armed = 0U;
  for (uint8_t i = 0U; i < BOARD_LOG_SOURCES; i++)
  {
    armed = (uint8_t)(armed + (((sources >> i) & 1U) ? 1U : 0U));
  }

  const uint32_t share = (armed != 0U)
                       ? (cmd_link_records_per_second(LOG_RECORD_BYTES) / armed)
                       : 0U;

  Board_LogEnable(sources, (share != 0U) ? (Board_SysClkHz() / share) : 0U);
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 2 - take up to `want` records, oldest first, and free their slots. */
static cmd_status_t h_log_take(rd_t *in, wr_t *out)
{
  board_sample_t batch[LOG_MAX_BURST];
  uint8_t want = LOG_MAX_BURST;

  if (rd_left(in) > 0U)
  {
    want = rd_u8(in);
    if (!rd_ok(in))
    {
      return CMD_ERR_LENGTH;
    }
    if ((want == 0U) || (want > LOG_MAX_BURST))
    {
      want = LOG_MAX_BURST;
    }
  }

  const uint16_t got = Board_LogTake(batch, want);

  wr_u8(out, (uint8_t)got);
  for (uint16_t i = 0U; i < got; i++)
  {
    wr_u32(out, batch[i].at);
    wr_u8(out, batch[i].source);
    wr_u8(out, batch[i].seq);
    for (uint8_t k = 0U; k < 4U; k++)
    {
      wr_i16(out, batch[i].v[k]);
    }
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


cmd_status_t cmd_log_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case LOG_OP_STATE: return h_log_state(out);
    case LOG_OP_ARM:   return h_log_arm(in, out);
    case LOG_OP_TAKE:  return h_log_take(in, out);
    default:           return CMD_ERR_VALUE;
  }
}
