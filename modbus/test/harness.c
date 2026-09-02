/**
  ******************************************************************************
  * @file    harness.c
  * @brief   A data model and a flat C API, so the portable core can be driven
  *          from host/tests/test_modbus_core.py through ctypes.
  *
  * Invariant 1 says modbus_crc/slave/rtu are hardware-free so they can be
  * tested on a host. Nothing did, until this: their only verification was
  * test_conformance.py, which needs a board on the other end of a cable. This
  * file is the application half those three expect - a small register bank and
  * the vtable over it - plus setters the tests use to make the model refuse
  * things on purpose.
  *
  * Built by the Python suite with the host gcc, never by the firmware build.
  * It is test scaffolding and must not appear in the root CMakeLists.
  ******************************************************************************
  */
#include "modbus_crc.h"
#include "modbus_rtu.h"
#include "modbus_slave.h"

#include <stdlib.h>
#include <string.h>

#define BANK 64U

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

typedef struct
{
  uint16_t hold[BANK];
  uint16_t input[BANK];
  uint8_t  coil[BANK];
  uint8_t  discrete[BANK];

  /* write_reg refuses this address outright, which is how a multi-register
     write is made to fail part way through. */
  int      fail_write_at;      /* -1 for never */
  /* validate_reg_value refuses this value, before anything is applied. */
  int      bad_value;          /* -1 for never */
  int      have_validate;      /* wire validate_reg_value at all */
  /* validate_range says yes to every address. Without this the harness's own
     32-bit span check stands in front of the engine's, and a test aimed at
     span_overflows passes whatever span_overflows does - measured: the engine
     rewritten to check in 16 bits was caught only by a compiler warning. */
  int      accept_all;

  char     id[64];
  int      run_indicator;

  mb_data_model_t model;
  mb_slave_t      slave;
  mb_rtu_t        rtu;
} harness_t;

/* ---- the data model ----------------------------------------------------- */

static mb_exception_t h_validate_range(void *ctx, mb_table_t table, uint16_t addr,
                                       uint16_t qty, bool for_write)
{
  const harness_t *h = (const harness_t *)ctx;
  if (h->accept_all)
  {
    return MB_EX_NONE;
  }
  if (for_write && (table == MB_TABLE_DISCRETE_INPUT || table == MB_TABLE_INPUT_REG))
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
  return ((uint32_t)addr + (uint32_t)qty <= BANK) ? MB_EX_NONE
                                                  : MB_EX_ILLEGAL_DATA_ADDRESS;
}

static mb_exception_t h_read_reg(void *ctx, mb_table_t table, uint16_t addr,
                                 uint16_t *out)
{
  harness_t *h = (harness_t *)ctx;
  *out = (table == MB_TABLE_INPUT_REG) ? h->input[addr] : h->hold[addr];
  return MB_EX_NONE;
}

static mb_exception_t h_write_reg(void *ctx, uint16_t addr, uint16_t value)
{
  harness_t *h = (harness_t *)ctx;
  if (h->fail_write_at >= 0 && (int)addr == h->fail_write_at)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  h->hold[addr] = value;
  return MB_EX_NONE;
}

static mb_exception_t h_read_bit(void *ctx, mb_table_t table, uint16_t addr,
                                 bool *out)
{
  harness_t *h = (harness_t *)ctx;
  *out = (table == MB_TABLE_DISCRETE_INPUT) ? (h->discrete[addr] != 0U)
                                            : (h->coil[addr] != 0U);
  return MB_EX_NONE;
}

static mb_exception_t h_write_bit(void *ctx, uint16_t addr, bool value)
{
  harness_t *h = (harness_t *)ctx;
  h->coil[addr] = value ? 1U : 0U;
  return MB_EX_NONE;
}

static mb_exception_t h_validate_reg_value(void *ctx, uint16_t addr, uint16_t value)
{
  harness_t *h = (harness_t *)ctx;
  (void)addr;
  if (h->bad_value >= 0 && (int)value == h->bad_value)
  {
    return MB_EX_ILLEGAL_DATA_VALUE;
  }
  return MB_EX_NONE;
}

static const char *h_server_id(void *ctx, uint8_t *run)
{
  harness_t *h = (harness_t *)ctx;
  *run = (uint8_t)h->run_indicator;
  return h->id[0] == '\0' ? NULL : h->id;
}

/* Echoes its payload back, so a test can tell a reached handler from a
   refused function code without reading any state. The engine writes the
   function code itself and hands this the byte after it, so the payload is
   all there is to write. */
static mb_exception_t h_user_function(void *ctx, uint8_t fc,
                                      const uint8_t *req, size_t req_len,
                                      uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
  (void)ctx;
  (void)fc;
  if (req_len > rsp_cap)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  memcpy(rsp, req, req_len);
  *rsp_len = req_len;
  return MB_EX_NONE;
}

/* ---- the flat API ctypes drives ----------------------------------------- */

API harness_t *mbh_new(void)
{
  harness_t *h = (harness_t *)calloc(1U, sizeof(*h));
  if (h == NULL)
  {
    return NULL;
  }

  h->fail_write_at = -1;
  h->bad_value     = -1;
  h->have_validate = 1;
  h->run_indicator = 0xFF;
  strcpy(h->id, "coaxial_63100");

  for (uint16_t i = 0U; i < BANK; i++)
  {
    h->hold[i]  = (uint16_t)(0x1000U + i);
    h->input[i] = (uint16_t)(0x2000U + i);
    h->coil[i]     = (uint8_t)(i & 1U);
    h->discrete[i] = (uint8_t)((i >> 1) & 1U);
  }

  h->model.validate_range = h_validate_range;
  h->model.read_reg       = h_read_reg;
  h->model.write_reg      = h_write_reg;
  h->model.read_bit       = h_read_bit;
  h->model.write_bit      = h_write_bit;
  h->model.validate_reg_value = h_validate_reg_value;
  h->model.server_id      = h_server_id;
  h->model.user_function  = h_user_function;
  h->model.ctx            = h;

  mb_slave_init(&h->slave, &h->model);
  return h;
}

API void mbh_free(harness_t *h) { free(h); }

/* Any callback can be unwired, which is how "answers ILLEGAL FUNCTION when the
   model cannot do it" is tested rather than assumed. */
API void mbh_drop(harness_t *h, int which)
{
  switch (which)
  {
    case 0: h->model.read_reg = NULL; break;
    case 1: h->model.write_reg = NULL; break;
    case 2: h->model.read_bit = NULL; break;
    case 3: h->model.write_bit = NULL; break;
    case 4: h->model.validate_reg_value = NULL; break;
    case 5: h->model.server_id = NULL; break;
    case 6: h->model.user_function = NULL; break;
    case 7: h->model.validate_range = NULL; break;
    default: break;
  }
}

API void mbh_accept_all(harness_t *h, int on)       { h->accept_all = on; }
API void mbh_fail_write_at(harness_t *h, int addr)  { h->fail_write_at = addr; }
API void mbh_bad_value(harness_t *h, int value)     { h->bad_value = value; }
API void mbh_set_id(harness_t *h, const char *s)
{
  strncpy(h->id, s, sizeof(h->id) - 1U);
  h->id[sizeof(h->id) - 1U] = '\0';
}

API uint16_t mbh_hold(harness_t *h, uint16_t addr) { return h->hold[addr]; }
API uint8_t  mbh_coil(harness_t *h, uint16_t addr) { return h->coil[addr]; }
API void     mbh_set_hold(harness_t *h, uint16_t addr, uint16_t v) { h->hold[addr] = v; }

API size_t mbh_execute(harness_t *h, const uint8_t *req, size_t req_len,
                       uint8_t *rsp, size_t rsp_cap)
{
  return mb_slave_execute(&h->slave, req, req_len, rsp, rsp_cap);
}

/* ---- RTU, with the clock injected ---------------------------------------- */

API void mbh_rtu_init(harness_t *h, uint8_t unit, uint32_t baud,
                      uint8_t bits, uint32_t ticks_per_us)
{
  mb_rtu_init(&h->rtu, &h->slave, unit, baud, bits, ticks_per_us);
}

API void mbh_rtu_byte(harness_t *h, uint8_t b, uint32_t ticks)
{
  mb_rtu_on_byte(&h->rtu, b, ticks);
}

API void mbh_rtu_error(harness_t *h, uint32_t ticks)
{
  mb_rtu_on_error(&h->rtu, ticks);
}

API int mbh_rtu_busy(harness_t *h) { return mb_rtu_busy(&h->rtu) ? 1 : 0; }

/* The real oracle, linked in by the suite's build beside a cmd_find stub
   (comms/test/cmd_find_stub.c): the 0x6E arm and the standard-FC arm are
   the hand-maintained code under test; the dispatch-table arm is bound to
   its tables by the suite's source parse instead. */
extern uint16_t cmd_request_length(const uint8_t *pdu, uint16_t have);

API void mbh_rtu_hint(harness_t *h, int on)
{
  mb_rtu_set_length_hint(&h->rtu, on ? cmd_request_length : NULL);
}

API uint16_t mbh_request_length(const uint8_t *pdu, uint16_t have)
{
  return cmd_request_length(pdu, have);
}

API uint32_t mbh_rtu_t35(harness_t *h) { return h->rtu.t35_ticks; }
API uint32_t mbh_rtu_t15(harness_t *h) { return h->rtu.t15_ticks; }

API size_t mbh_rtu_service(harness_t *h, uint32_t ticks, uint8_t *out, size_t cap)
{
  const uint8_t *frame = NULL;
  const size_t n = mb_rtu_service(&h->rtu, ticks, &frame);
  if (n > 0U && frame != NULL && n <= cap)
  {
    memcpy(out, frame, n);
  }
  return n;
}

/** Six counters in declaration order: bus_message, bus_comm_error,
    server_message, server_exception, server_no_response, char_overrun. */
API void mbh_rtu_counters(harness_t *h, uint32_t *out)
{
  out[0] = h->rtu.counters.bus_message;
  out[1] = h->rtu.counters.bus_comm_error;
  out[2] = h->rtu.counters.server_message;
  out[3] = h->rtu.counters.server_exception;
  out[4] = h->rtu.counters.server_no_response;
  out[5] = h->rtu.counters.char_overrun;
}

/* ---- CRC, straight through ---------------------------------------------- */

API uint16_t mbh_crc16(const uint8_t *d, size_t n)      { return modbus_crc16(d, n); }
API size_t   mbh_crc_append(uint8_t *b, size_t n)       { return modbus_crc_append(b, n); }
API int      mbh_crc_check(const uint8_t *f, size_t n)  { return modbus_crc_check(f, n); }
