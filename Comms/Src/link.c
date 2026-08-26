/**
  ******************************************************************************
  * @file    link.c
  * @brief   Assembles the comms stack and pumps it from the main loop.
  ******************************************************************************
  */
#include "link.h"
#include "cmd.h"
#include "dev_serial.h"
#include "modbus_map.h"
#include "modbus_rtu.h"
#include "modbus_slave.h"

#include <stddef.h>

#define LINK_BITS_PER_CHAR 11U

static const dev_serial_t *s_dev;
static mb_slave_t          s_slave;
static mb_rtu_t            s_rtu;
static bool                s_open;
static bool                s_close_pending;

/* Bridge from the protocol's user-defined function space into the command
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

static void build(void)
{
  /* mb_rtu_init memsets the whole mb_rtu_t, counters included - right for the
     very first call from link_init(), where s_rtu is still its static
     zero-initialised self, but not for a later call from link_open(): the
     counters are this run's diagnostic history, not framing state, and
     link_close() already leaves them alone on the way OUT of binary mode. A
     console round trip - 'm' to enter, 0x0001 to leave, 'm' again - silently
     zeroed them on the way back in, with nothing in link_open()'s own comment
     saying so. Saved and restored here rather than in mb_rtu_init itself,
     since that file has no notion of "this is a reopen, not a cold start". */
  const mb_rtu_counters_t saved = s_rtu.counters;

  mb_slave_init(&s_slave, modbus_map_model(&s_rtu, user_function));
  mb_rtu_init(&s_rtu, &s_slave, modbus_map_unit_id(),
              dev_usart3_baud(), LINK_BITS_PER_CHAR,
              s_dev->ticks_per_us(s_dev->ctx));

  s_rtu.counters = saved;
}

void link_init(void)
{
  s_dev = dev_usart3();
  build();

  s_open          = false;
  s_close_pending = false;
}

const char *link_proto_name(void)
{
  return "modbus-rtu";
}

bool link_active(void)
{
  return s_open;
}

bool link_busy(void)
{
  return s_open && mb_rtu_busy(&s_rtu);
}

void link_open(void)
{
  s_dev->purge(s_dev->ctx);
  build();

  s_close_pending = false;
  s_open          = true;
}

void link_close(void)
{
  s_open          = false;
  s_close_pending = false;
  s_dev->purge(s_dev->ctx);
}

void link_request_close(void)
{
  s_close_pending = true;
}

uint32_t link_ticks_per_us(void)
{
  return s_dev->ticks_per_us(s_dev->ctx);
}

/* The whole pump. Four steps, no nesting beyond a guard each. */
void link_poll(void)
{
  if (!s_open)
  {
    return;
  }

  const uint32_t now = s_dev->ticks(s_dev->ctx);
  uint8_t        byte;

  if (s_dev->fault(s_dev->ctx))
  {
    mb_rtu_on_error(&s_rtu, now);
  }
  else if (s_dev->get(s_dev->ctx, &byte))
  {
    mb_rtu_on_byte(&s_rtu, byte, now);
  }

  const uint8_t *frame = NULL;
  const size_t   n     = mb_rtu_service(&s_rtu, s_dev->ticks(s_dev->ctx), &frame);

  if (n > 0U)
  {
    s_dev->put(s_dev->ctx, frame, (uint16_t)n);
    /* A request may have just rewritten the unit address. The reply above
       correctly used the old one; adopt the new one now. */
    s_rtu.unit_id = modbus_map_unit_id();
  }

  if (s_close_pending && !mb_rtu_busy(&s_rtu))
  {
    link_close();
  }
}

uint8_t link_unit_id(void)
{
  return modbus_map_unit_id();
}

void link_stats(link_stats_t *out)
{
  out->unit_id            = modbus_map_unit_id();
  out->t15_ticks          = s_rtu.t15_ticks;
  out->t35_ticks          = s_rtu.t35_ticks;
  out->bus_message        = s_rtu.counters.bus_message;
  out->bus_comm_error     = s_rtu.counters.bus_comm_error;
  out->server_message     = s_rtu.counters.server_message;
  out->server_exception   = s_rtu.counters.server_exception;
  out->server_no_response = s_rtu.counters.server_no_response;
  out->char_overrun       = s_rtu.counters.char_overrun;
}
