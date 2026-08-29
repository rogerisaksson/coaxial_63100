/**
  ******************************************************************************
  * @file    cmd_link.c
  * @brief   The serial ports' operations behind command 0x6E, device 2.
  *
  * A port is as much a device as the IMU is: it has state worth reading and a
  * self-check worth running, and 0x47 answers the console port's counters
  * only because that is what it was written for.
  ******************************************************************************
  */
#include "cmd.h"
#include "link.h"
#include "dev_serial.h"
#include "wire.h"

/**
  * @brief op 0 - four patterns out on one port, and what came back.
  *
  * RE is tied to GND on both transceivers, so those two ports hear
  * themselves and all four must return; one that does not is the driver, the
  * receiver or the wiring. USART3 is not RS485 and returns nothing, which is
  * the right answer there. Four bytes on the segment, and nothing times it.
  */
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

  /* Not the port this request came in on. Measured: asking USART3 to test
     itself put 00 ff 5a a5 in front of the reply and the master saw a
     checksum failure. A port cannot check its own loopback while it is
     carrying the conversation. */
  if (index == link_current())
  {
    return CMD_ERR_VALUE;
  }

  const uint8_t matched = dev_uart_echo(index, &seen);

  wr_u8(out, index);
  wr_u8(out, link_is_rs485(index) ? 1U : 0U);
  wr_u8(out, matched);              /* one bit per pattern, 0x0F is all four */
  wr_u8(out, seen);
  wr_str(out, link_name(index));

  return CMD_OK;
}

/**
  * @brief op 1 - one port's framing state and counters.
  *
  * `bus_message` counts every frame seen on the segment and `server_message`
  * only the ones addressed to this unit - on a multidrop bus the difference
  * is the traffic meant for somebody else, which is the evidence that the
  * address filter is doing its job rather than that the wire is quiet.
  */
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
