/**
  ******************************************************************************
  * @file    link.c
  * @brief   Assembles the comms stack and pumps it from the main loop.
  ******************************************************************************
  */
#include "comms_limits.h"
#include "link.h"
#include "cmd_length.h"
#include "cmd.h"
#include "dev_serial.h"
#include "modbus_map.h"
#include "modbus_rtu.h"
#include "modbus_slave.h"

#include <stddef.h>

/* One port, whole. The console link is index 0 and can be handed back to the
   ASCII console; the two RS485 links have no console to hand it back to, so
   they are open from boot and never close.

   Each carries its own model copy: modbus_map_model() returns a pointer to
   one static whose ctx it overwrites per call, so three links sharing it
   would all reach the last one's counters. */
typedef struct
{
  const dev_serial_t *dev;
  mb_data_model_t     model;
  mb_slave_t          slave;
  mb_rtu_t            rtu;
  bool                open;
  bool                close_pending;
} link_port_t;

static link_port_t s_links[LINK_COUNT];

/* Which port is inside mb_rtu_service, and so which one a command handler is
   answering on. A handler that puts bytes on the wire needs to know: the
   loopback check transmitted its four patterns on the port carrying the
   request, and the reply came back with 00 ff 5a a5 in front of it and a
   failed checksum. */
static uint8_t s_current = LINK_CONSOLE;

/* GateDrivers from the protocol's user-defined function space into the command
   table. The mapping of failures onto Modbus exceptions is the only judgement
   here: a bad length or a bad field are both "the request was wrong", which is
   ILLEGAL DATA VALUE, while an unknown code is ILLEGAL FUNCTION. */
static mb_exception_t user_function(void *ctx, uint8_t fc,
                                   const uint8_t *req, size_t req_len,
                                   uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
  (void)ctx;

  uint16_t n = 0U;

  const cmd_status_t st = cmd_dispatch(fc, req, (uint16_t)req_len,
                                       rsp, (uint16_t)rsp_cap, &n);

  *rsp_len = (size_t)n;

  if (st == CMD_OK)
  {
    return MB_EX_NONE;
  }

  if (st == CMD_ERR_UNKNOWN)
  {
    return MB_EX_ILLEGAL_FUNCTION;
  }

  if (st == CMD_ERR_DEVICE)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }

  return MB_EX_ILLEGAL_DATA_VALUE;
}

static void build(link_port_t *l)
{
  /* mb_rtu_init memsets the whole mb_rtu_t, counters included - right for the
     very first call from link_init(), where the port is still its static
     zero-initialised self, but not for a later call from link_open(): the
     counters are this run's diagnostic history, not framing state, and
     link_close() already leaves them alone on the way OUT of binary mode. A
     console round trip - 'm' to enter, 0x0001 to leave, 'm' again - silently
     zeroed them on the way back in, with nothing in link_open()'s own comment
     saying so. Saved and restored here rather than in mb_rtu_init itself,
     since that file has no notion of "this is a reopen, not a cold start". */
  const mb_rtu_counters_t saved = l->rtu.counters;

  l->model = *modbus_map_model(&l->rtu, user_function);
  mb_slave_init(&l->slave, &l->model);
  mb_rtu_init(&l->rtu, &l->slave, modbus_map_unit_id(),
              dev_uart_port_baud((uint8_t)(l - s_links)),
              LINK_BITS_PER_CHAR,
              l->dev->ticks_per_us(l->dev->ctx));

  /* The early path: a request whose shape the oracle can prove is
     dispatched on its own CRC instead of after t3.5 of silence - 1.75 ms
     off every proven transaction (MINOR 9). Anything unproven waits the
     silence exactly as before. */
  mb_rtu_set_length_hint(&l->rtu, cmd_request_length);

  l->rtu.counters = saved;
}

void link_init(void)
{
  for (uint8_t i = 0U; i < LINK_COUNT; i++)
  {
    s_links[i].dev = dev_uart(i);
    build(&s_links[i]);

    /* The RS485 pair answers from boot. There is no console on a bus with
       other devices on it, and a port that has to be opened by a console
       command cannot be opened at all from the far end of one. */
    s_links[i].open          = dev_uart_rs485(i);
    s_links[i].close_pending = false;
  }
}

const char *link_proto_name(void)
{
  return "modbus-rtu";
}

bool link_active(void)
{
  return s_links[LINK_CONSOLE].open;
}

bool link_busy(void)
{
  for (uint8_t i = 0U; i < LINK_COUNT; i++)
  {
    if (s_links[i].open && mb_rtu_busy(&s_links[i].rtu))
    {
      return true;
    }
  }

  return false;
}

void link_open(void)
{
  link_port_t *l = &s_links[LINK_CONSOLE];

  l->dev->purge(l->dev->ctx);
  build(l);

  l->close_pending = false;
  l->open          = true;
}

void link_close(void)
{
  link_port_t *l = &s_links[LINK_CONSOLE];

  l->open          = false;
  l->close_pending = false;
  l->dev->purge(l->dev->ctx);
}

void link_request_close(void)
{
  s_links[LINK_CONSOLE].close_pending = true;
}

uint32_t link_ticks_per_us(void)
{
  const link_port_t *l = &s_links[LINK_CONSOLE];

  return l->dev->ticks_per_us(l->dev->ctx);
}

/* One port's pump. Four steps, no nesting beyond a guard each. */
/* Bytes in, from any port. A COUNT and not a timestamp: this file gets its
   clock injected through the device and has none of its own, so whoever
   wants to know how long ago keeps the time themselves. */
static uint32_t s_rx_count;

uint32_t link_rx_count(void)
{
  return s_rx_count;
}


static void pump(link_port_t *l)
{
  if (!l->open)
  {
    return;
  }

  uint8_t  byte;
  uint32_t at = 0U;

  if (l->dev->fault(l->dev->ctx))
  {
    mb_rtu_on_error(&l->rtu, l->dev->ticks(l->dev->ctx));
  }
  else if (l->dev->get(l->dev->ctx, &byte, &at))
  {
    /* `at` is when the character arrived, which on the interrupt-driven
       ports is not when this loop reached it. Framing is silence, so the
       difference is the whole measurement. */
    mb_rtu_on_byte(&l->rtu, byte, at);
    s_rx_count++;
  }

  const uint8_t *frame = NULL;

  s_current = (uint8_t)(l - s_links);

  const size_t n = mb_rtu_service(&l->rtu, l->dev->ticks(l->dev->ctx),
                                  &frame);

  if (n > 0U)
  {
    l->dev->put(l->dev->ctx, frame, (uint16_t)n);
    /* A request may have just rewritten the unit address. The reply above
       correctly used the old one; adopt the new one now - on every port,
       because the address belongs to the board and not to the wire it was
       changed over. */
    for (uint8_t i = 0U; i < LINK_COUNT; i++)
    {
      s_links[i].rtu.unit_id = modbus_map_unit_id();
    }
  }

  if (l->close_pending && !mb_rtu_busy(&l->rtu))
  {
    link_close();
  }
}

void link_poll(void)
{
  for (uint8_t i = 0U; i < LINK_COUNT; i++)
  {
    pump(&s_links[i]);
  }
}

uint8_t link_unit_id(void)
{
  return modbus_map_unit_id();
}

void link_stats(link_stats_t *out)
{
  link_stats_of(LINK_CONSOLE, out);
}

void link_stats_of(uint8_t index, link_stats_t *out)
{
  if ((index >= LINK_COUNT) || (out == NULL))
  {
    return;
  }

  const mb_rtu_t *rtu = &s_links[index].rtu;

  out->unit_id            = modbus_map_unit_id();
  out->t15_ticks          = rtu->t15_ticks;
  out->t35_ticks          = rtu->t35_ticks;
  out->bus_message        = rtu->counters.bus_message;
  out->bus_comm_error     = rtu->counters.bus_comm_error;
  out->server_message     = rtu->counters.server_message;
  out->server_exception   = rtu->counters.server_exception;
  out->server_no_response = rtu->counters.server_no_response;
  out->char_overrun       = rtu->counters.char_overrun;
}

uint8_t link_current(void)
{
  return s_current;
}

bool link_port_open(uint8_t index)
{
  return (index < LINK_COUNT) && s_links[index].open;
}

const char *link_name(uint8_t index)
{
  return dev_uart_name(index);
}

bool link_is_rs485(uint8_t index)
{
  return dev_uart_rs485(index);
}

uint32_t link_baud(uint8_t index)
{
  return dev_uart_port_baud(index);
}
