/** cmd_link.c - The serial ports' operations behind command 0x6E, device 2. */
#include "cmd.h"
#include "link.h"
#include "dev_serial.h"
#include "wire.h"

/** op 0 - four patterns out on one port, and what came back. */
static cmd_status_t h_link_echo(rd_t *in, wr_t *out)
{
  const uint8_t index = rd_u8(in);
  uint8_t seen = 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (index >= LINK_COUNT)
  {
    return CMD_ERR_VALUE;
  }

  /* Not the port this request came in on: its patterns would land in front of the reply. */
  if (index == link_current())
  {
    wr_took(out, "this port carries the request - its own patterns would land in front of "
                 "the reply; ask on another");
    return CMD_OK;
  }

  const uint8_t matched = dev_uart_echo(index, &seen);

  wr_u8(out, index);
  wr_u8(out, link_is_rs485(index) ? 1U : 0U);
  wr_u8(out, matched);              /* one bit per pattern, 0x0F is all four */
  wr_u8(out, seen);
  wr_str(out, link_name(index));

  return CMD_OK;
}

/** op 1 - one port's framing state and counters. */
static cmd_status_t h_link_stats(rd_t *in, wr_t *out)
{
  const uint8_t index = rd_u8(in);
  link_stats_t st;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (index >= LINK_COUNT)
  {
    return CMD_ERR_VALUE;
  }

  link_stats_of(index, &st);

  wr_u8(out, index);
  wr_u8(out, st.unit_id);
  wr_u8(out, link_is_rs485(index) ? 1U : 0U);
  wr_u8(out, link_port_open(index) ? 1U : 0U);
  wr_u32(out, link_baud(index));
  wr_u32(out, st.t15_ticks);
  wr_u32(out, st.t35_ticks);
  wr_u32(out, st.bus_message);
  wr_u32(out, st.bus_comm_error);
  wr_u32(out, st.server_message);
  wr_u32(out, st.server_exception);
  wr_u32(out, st.server_no_response);
  wr_u32(out, st.char_overrun);
  wr_u32(out, dev_uart_dropped(index));
  wr_str(out, link_name(index));

  return CMD_OK;
}

cmd_status_t cmd_link_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case LINK_OP_ECHO:  return h_link_echo(in, out);
    case LINK_OP_STATS: return h_link_stats(in, out);
    default:            return CMD_ERR_VALUE;
  }
}
