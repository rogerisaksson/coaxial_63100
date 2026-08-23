/**
  ******************************************************************************
  * @file    modbus_slave.c
  * @brief   Portable Modbus server PDU engine. No hardware dependencies.
  ******************************************************************************
  */
#include "modbus_slave.h"

#include <string.h>

/* Quantity limits from MODBUS Application Protocol V1.1b3. They are not
   arbitrary: each is the largest count whose response still fits in a 253-byte
   PDU. Exceeding them is ILLEGAL DATA VALUE (0x03), never ILLEGAL DATA ADDRESS
   (0x02) - the request is badly formed, the addresses were never consulted. */
#define MB_MAX_READ_BITS   2000U
#define MB_MAX_READ_REGS    125U
#define MB_MAX_WRITE_BITS  1968U
#define MB_MAX_WRITE_REGS   123U

static uint16_t rd_u16(const uint8_t *p)
{
  /* Every 16-bit field in a PDU is big-endian. Only the CRC is not, and the
     CRC never reaches this file. */
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void wr_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFFU);
}

static size_t make_exception(uint8_t *rsp, uint8_t fc, mb_exception_t ex)
{
  rsp[0] = (uint8_t)(fc | 0x80U);
  rsp[1] = (uint8_t)ex;
  return 2U;
}

/* True if [addr, addr+qty) would run past the end of the address space.
   Computed in 32 bits on purpose: doing it in 16 would wrap and let a request
   straddling the top of the space slip past the range check entirely. */
static bool span_overflows(uint16_t addr, uint16_t qty)
{
  return ((uint32_t)addr + (uint32_t)qty) > 0x10000UL;
}

static mb_exception_t check_span(const mb_slave_t *s, mb_table_t table,
                                 uint16_t addr, uint16_t qty, bool for_write)
{
  if (span_overflows(addr, qty))
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }

  if (s->model->validate_range == NULL)
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }

  return s->model->validate_range(s->model->ctx, table, addr, qty, for_write);
}

/* ---- bit reads: FC 0x01, 0x02 ------------------------------------------ */

static size_t do_read_bits(mb_slave_t *s, mb_table_t table, uint8_t fc,
                           const uint8_t *req, uint8_t *rsp)
{
  const uint16_t addr = rd_u16(&req[1]);
  const uint16_t qty  = rd_u16(&req[3]);

  if ((qty < 1U) || (qty > MB_MAX_READ_BITS))
  {
    return make_exception(rsp, fc, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (s->model->read_bit == NULL)
  {
    return make_exception(rsp, fc, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, table, addr, qty, false);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, fc, ex);
  }

  const uint8_t nbytes = (uint8_t)((qty + 7U) / 8U);

  rsp[0] = fc;
  rsp[1] = nbytes;
  memset(&rsp[2], 0, nbytes);

  for (uint16_t i = 0U; i < qty; i++)
  {
    bool bit = false;

    ex = s->model->read_bit(s->model->ctx, table, (uint16_t)(addr + i), &bit);
    if (ex != MB_EX_NONE)
    {
      return make_exception(rsp, fc, ex);
    }

    if (bit)
    {
      /* Bits pack LSB-first within each byte: the item at the starting
         address is bit 0 of the first data byte. Unused high bits of the
         last byte stay zero, which the memset above guaranteed. */
      rsp[2U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
    }
  }

  return (size_t)(2U + nbytes);
}

/* ---- register reads: FC 0x03, 0x04 ------------------------------------- */

static size_t do_read_regs(mb_slave_t *s, mb_table_t table, uint8_t fc,
                           const uint8_t *req, uint8_t *rsp)
{
  const uint16_t addr = rd_u16(&req[1]);
  const uint16_t qty  = rd_u16(&req[3]);

  if ((qty < 1U) || (qty > MB_MAX_READ_REGS))
  {
    return make_exception(rsp, fc, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (s->model->read_reg == NULL)
  {
    return make_exception(rsp, fc, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, table, addr, qty, false);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, fc, ex);
  }

  rsp[0] = fc;
  rsp[1] = (uint8_t)(qty * 2U);

  for (uint16_t i = 0U; i < qty; i++)
  {
    uint16_t v = 0U;

    ex = s->model->read_reg(s->model->ctx, table, (uint16_t)(addr + i), &v);
    if (ex != MB_EX_NONE)
    {
      return make_exception(rsp, fc, ex);
    }

    wr_u16(&rsp[2U + (i * 2U)], v);
  }

  return (size_t)(2U + (qty * 2U));
}

/* ---- FC 0x05 write single coil ----------------------------------------- */

static size_t do_write_single_coil(mb_slave_t *s, const uint8_t *req, uint8_t *rsp)
{
  const uint16_t addr = rd_u16(&req[1]);
  const uint16_t val  = rd_u16(&req[3]);

  /* The spec allows exactly two values here. Anything else is a malformed
     value, not an address problem. */
  if ((val != 0xFF00U) && (val != 0x0000U))
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (s->model->write_bit == NULL)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, MB_TABLE_COIL, addr, 1U, true);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_COIL, ex);
  }

  ex = s->model->write_bit(s->model->ctx, addr, (val == 0xFF00U));
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_COIL, ex);
  }

  /* A successful write echoes the request verbatim. */
  memcpy(rsp, req, 5U);
  return 5U;
}

/* ---- FC 0x06 write single register ------------------------------------- */

static size_t do_write_single_reg(mb_slave_t *s, const uint8_t *req, uint8_t *rsp)
{
  const uint16_t addr = rd_u16(&req[1]);
  const uint16_t val  = rd_u16(&req[3]);

  if (s->model->write_reg == NULL)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, MB_TABLE_HOLDING_REG, addr, 1U, true);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_REG, ex);
  }

  ex = s->model->write_reg(s->model->ctx, addr, val);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_SINGLE_REG, ex);
  }

  memcpy(rsp, req, 5U);
  return 5U;
}

/* ---- FC 0x0F write multiple coils -------------------------------------- */

static size_t do_write_multi_coils(mb_slave_t *s, const uint8_t *req, size_t req_len,
                                   uint8_t *rsp)
{
  const uint16_t addr   = rd_u16(&req[1]);
  const uint16_t qty    = rd_u16(&req[3]);
  const uint8_t  nbytes = req[5];

  if ((qty < 1U) || (qty > MB_MAX_WRITE_BITS) ||
      (nbytes != (uint8_t)((qty + 7U) / 8U)))
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_COILS, MB_EX_ILLEGAL_DATA_VALUE);
  }

  /* The byte count must also agree with what actually arrived, or the loop
     below would read past the end of the frame. */
  if (req_len != (size_t)(6U + nbytes))
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_COILS, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (s->model->write_bit == NULL)
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_COILS, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, MB_TABLE_COIL, addr, qty, true);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_COILS, ex);
  }

  for (uint16_t i = 0U; i < qty; i++)
  {
    const bool bit = ((req[6U + (i / 8U)] >> (i % 8U)) & 1U) != 0U;

    ex = s->model->write_bit(s->model->ctx, (uint16_t)(addr + i), bit);
    if (ex != MB_EX_NONE)
    {
      return make_exception(rsp, MB_FC_WRITE_MULTIPLE_COILS, ex);
    }
  }

  rsp[0] = MB_FC_WRITE_MULTIPLE_COILS;
  wr_u16(&rsp[1], addr);
  wr_u16(&rsp[3], qty);
  return 5U;
}

/* ---- FC 0x10 write multiple registers ---------------------------------- */

static size_t do_write_multi_regs(mb_slave_t *s, const uint8_t *req, size_t req_len,
                                  uint8_t *rsp)
{
  const uint16_t addr   = rd_u16(&req[1]);
  const uint16_t qty    = rd_u16(&req[3]);
  const uint8_t  nbytes = req[5];

  if ((qty < 1U) || (qty > MB_MAX_WRITE_REGS) || (nbytes != (uint8_t)(qty * 2U)))
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (req_len != (size_t)(6U + nbytes))
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, MB_EX_ILLEGAL_DATA_VALUE);
  }

  if (s->model->write_reg == NULL)
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, MB_EX_ILLEGAL_FUNCTION);
  }

  mb_exception_t ex = check_span(s, MB_TABLE_HOLDING_REG, addr, qty, true);
  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, ex);
  }

  /* check_span only proves every address in the span is writable, not that
     every VALUE in the request is. Without this pass, a span covering a
     register this model rejects by value - not by address - would already
     have applied write_reg() to the registers before it, leaving the device
     half written under a response that reports the whole request failed. */
  if (s->model->validate_reg_value != NULL)
  {
    for (uint16_t i = 0U; i < qty; i++)
    {
      ex = s->model->validate_reg_value(s->model->ctx, (uint16_t)(addr + i),
                                        rd_u16(&req[6U + (i * 2U)]));
      if (ex != MB_EX_NONE)
      {
        return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, ex);
      }
    }
  }

  for (uint16_t i = 0U; i < qty; i++)
  {
    ex = s->model->write_reg(s->model->ctx, (uint16_t)(addr + i),
                             rd_u16(&req[6U + (i * 2U)]));
    if (ex != MB_EX_NONE)
    {
      return make_exception(rsp, MB_FC_WRITE_MULTIPLE_REGS, ex);
    }
  }

  rsp[0] = MB_FC_WRITE_MULTIPLE_REGS;
  wr_u16(&rsp[1], addr);
  wr_u16(&rsp[3], qty);
  return 5U;
}

/* ---- FC 0x11 report server id ------------------------------------------ */

static size_t do_report_server_id(mb_slave_t *s, uint8_t *rsp, size_t rsp_cap)
{
  if (s->model->server_id == NULL)
  {
    return make_exception(rsp, MB_FC_REPORT_SERVER_ID, MB_EX_ILLEGAL_FUNCTION);
  }

  uint8_t run = 0xFFU;
  const char *id = s->model->server_id(s->model->ctx, &run);
  if (id == NULL)
  {
    return make_exception(rsp, MB_FC_REPORT_SERVER_ID, MB_EX_SERVER_DEVICE_FAILURE);
  }

  size_t idlen = strlen(id);

  /* Layout is fc, byte count, run indicator, then the id. Clamp so a long id
     string can never overrun the response buffer, and so the byte count field
     cannot exceed what one octet can express. */
  const size_t room = (rsp_cap > 3U) ? (rsp_cap - 3U) : 0U;
  if (idlen > room)
  {
    idlen = room;
  }
  if (idlen > 249U)
  {
    idlen = 249U;
  }

  rsp[0] = MB_FC_REPORT_SERVER_ID;
  rsp[1] = (uint8_t)(1U + idlen);
  rsp[2] = run;
  memcpy(&rsp[3], id, idlen);

  return 3U + idlen;
}

/* ---- dispatch ---------------------------------------------------------- */

/* One uniform signature so the function codes can live in a table instead of a
   switch. The wrappers are one line each; the tested handlers above keep their
   own natural arguments. */
typedef size_t (*mb_fc_fn)(mb_slave_t *s, const uint8_t *req, size_t len,
                           uint8_t *rsp, size_t cap);

static size_t fc_read_coils(mb_slave_t *s, const uint8_t *req, size_t len,
                            uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_read_bits(s, MB_TABLE_COIL, MB_FC_READ_COILS, req, rsp);
}

static size_t fc_read_discrete(mb_slave_t *s, const uint8_t *req, size_t len,
                               uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_read_bits(s, MB_TABLE_DISCRETE_INPUT, MB_FC_READ_DISCRETE_INPUTS, req, rsp);
}

static size_t fc_read_holding(mb_slave_t *s, const uint8_t *req, size_t len,
                              uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_read_regs(s, MB_TABLE_HOLDING_REG, MB_FC_READ_HOLDING_REGS, req, rsp);
}

static size_t fc_read_input(mb_slave_t *s, const uint8_t *req, size_t len,
                            uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_read_regs(s, MB_TABLE_INPUT_REG, MB_FC_READ_INPUT_REGS, req, rsp);
}

static size_t fc_write_coil(mb_slave_t *s, const uint8_t *req, size_t len,
                            uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_write_single_coil(s, req, rsp);
}

static size_t fc_write_reg(mb_slave_t *s, const uint8_t *req, size_t len,
                           uint8_t *rsp, size_t cap)
{
  (void)len; (void)cap;
  return do_write_single_reg(s, req, rsp);
}

static size_t fc_write_coils(mb_slave_t *s, const uint8_t *req, size_t len,
                             uint8_t *rsp, size_t cap)
{
  (void)cap;
  return do_write_multi_coils(s, req, len, rsp);
}

static size_t fc_write_regs(mb_slave_t *s, const uint8_t *req, size_t len,
                            uint8_t *rsp, size_t cap)
{
  (void)cap;
  return do_write_multi_regs(s, req, len, rsp);
}

static size_t fc_server_id(mb_slave_t *s, const uint8_t *req, size_t len,
                           uint8_t *rsp, size_t cap)
{
  (void)req; (void)len;
  return do_report_server_id(s, rsp, cap);
}

typedef struct
{
  uint8_t  fc;
  uint8_t  len_min;    /**< shortest acceptable request PDU            */
  uint8_t  len_exact;  /**< exact length required, or 0 if variable    */
  mb_fc_fn fn;
} mb_fc_desc_t;

/* len_min 6 for the two block writes is deliberate: a request for zero items
   carries a zero byte count and no data, which is a well-formed 6-byte PDU
   declaring an illegal quantity. It must be answered with ILLEGAL DATA VALUE,
   not with silence, so it has to reach its handler. Six bytes is exactly
   enough to index the byte count, and the handler rechecks the length against
   it before touching any data. */
static const mb_fc_desc_t FC_TABLE[] =
{
  { MB_FC_READ_COILS,           5U, 5U, fc_read_coils    },
  { MB_FC_READ_DISCRETE_INPUTS, 5U, 5U, fc_read_discrete },
  { MB_FC_READ_HOLDING_REGS,    5U, 5U, fc_read_holding  },
  { MB_FC_READ_INPUT_REGS,      5U, 5U, fc_read_input    },
  { MB_FC_WRITE_SINGLE_COIL,    5U, 5U, fc_write_coil    },
  { MB_FC_WRITE_SINGLE_REG,     5U, 5U, fc_write_reg     },
  { MB_FC_WRITE_MULTIPLE_COILS, 6U, 0U, fc_write_coils   },
  { MB_FC_WRITE_MULTIPLE_REGS,  6U, 0U, fc_write_regs    },
  { MB_FC_REPORT_SERVER_ID,     1U, 1U, fc_server_id     },
};

static const mb_fc_desc_t *fc_find(uint8_t fc)
{
  for (size_t i = 0U; i < (sizeof(FC_TABLE) / sizeof(FC_TABLE[0])); i++)
  {
    if (FC_TABLE[i].fc == fc)
    {
      return &FC_TABLE[i];
    }
  }

  return NULL;
}

/* The ranges the specification reserves for user-defined functions. Anything
   here is handed to the application rather than refused, which is where this
   board's own binary commands live. */
static bool fc_is_user_defined(uint8_t fc)
{
  return ((fc >= 65U) && (fc <= 72U)) || ((fc >= 100U) && (fc <= 110U));
}

static size_t run_user_function(mb_slave_t *s, uint8_t fc, const uint8_t *req,
                                size_t req_len, uint8_t *rsp, size_t rsp_cap)
{
  size_t n = 0U;

  const mb_exception_t ex = s->model->user_function(s->model->ctx, fc,
                                                    &req[1], req_len - 1U,
                                                    &rsp[1], rsp_cap - 1U, &n);

  if (ex != MB_EX_NONE)
  {
    return make_exception(rsp, fc, ex);
  }

  rsp[0] = fc;
  return n + 1U;
}

void mb_slave_init(mb_slave_t *slave, const mb_data_model_t *model)
{
  slave->model = model;
}

/* Guard clauses, one lookup, no switch. A length that does not match the
   function code yields no response at all rather than an exception: a frame
   whose length contradicts its own function code cannot be trusted to have
   been parsed correctly, and the remedy for an unintelligible frame is
   silence. */
size_t mb_slave_execute(mb_slave_t *slave, const uint8_t *req, size_t req_len,
                        uint8_t *rsp, size_t rsp_cap)
{
  if ((slave == NULL) || (slave->model == NULL) || (req_len < 1U) || (rsp_cap < 5U))
  {
    return 0U;
  }

  const uint8_t           fc = req[0];
  const mb_fc_desc_t     *d  = fc_find(fc);

  if (d == NULL)
  {
    if (fc_is_user_defined(fc) && (slave->model->user_function != NULL))
    {
      return run_user_function(slave, fc, req, req_len, rsp, rsp_cap);
    }

    /* Intelligible, but not something this server can do. */
    return make_exception(rsp, fc, MB_EX_ILLEGAL_FUNCTION);
  }

  if (req_len < (size_t)d->len_min)
  {
    return 0U;
  }

  if ((d->len_exact != 0U) && (req_len != (size_t)d->len_exact))
  {
    return 0U;
  }

  return d->fn(slave, req, req_len, rsp, rsp_cap);
}
