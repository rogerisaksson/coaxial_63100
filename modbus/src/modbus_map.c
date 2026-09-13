/**
  ******************************************************************************
  * @file    modbus_map.c
  * @brief   This board as a Modbus data model. See modbus_map.h for the map.
  ******************************************************************************
  */
#include "modbus_map.h"
#include "board.h"
#include "board_power.h"
#include "modbus_rtu.h"

#include <stddef.h>
#include <stdint.h>

/* Live unit address. Defaults to 1: this is the board on the end of the link
   the developer is already using, and 1 is the conventional first server. */
static uint8_t s_unit_id = 1U;

uint8_t modbus_map_unit_id(void)
{
  return s_unit_id;
}

bool modbus_map_set_unit_id(uint8_t id)
{
  /* 0 is broadcast and 248..255 are reserved, so neither may be an address. */
  if ((id < 1U) || (id > 247U))
  {
    return false;
  }

  s_unit_id = id;
  return true;
}

/* ---- the input register space ------------------------------------------ */

/* One row per span of input registers: where it starts, how many words it
   holds, and the reader that produces one of them from its offset. Mapping,
   the extent and reading all walk this table, so a register added here is
   mapped, readable and counted at once - the map used to say its layout in
   two places and a span added to one was a hole in the other. The ADC codes
   come first and their count is the board's, asked at run time, which is why
   a span's width is a call rather than a number. */
typedef mb_exception_t (*ireg_reader_t)(const mb_rtu_t *rtu, uint16_t offset,
                                        uint16_t *out);

typedef struct
{
  uint16_t base;
  uint16_t (*words)(void);
  ireg_reader_t read;
} ireg_span_t;

/* The high word first, matching the big-endian convention every other
   multi-register value on the wire follows. */
static uint16_t word_of(uint32_t value, uint16_t which)
{
  return (which == 0U) ? (uint16_t)(value >> 16) : (uint16_t)(value & 0xFFFFU);
}

static uint16_t clamp_u16(int32_t value)
{
  const int32_t held = (value < 0) ? 0 : ((value > UINT16_MAX) ? UINT16_MAX : value);
  return (uint16_t)held;
}

static int16_t clamp_i16(int32_t value)
{
  const int32_t held = (value < INT16_MIN) ? INT16_MIN
                       : ((value > INT16_MAX) ? INT16_MAX : value);
  return (int16_t)held;
}

static uint16_t adc_words(void)
{
  return (uint16_t)Board_AdcCount();
}

static uint16_t one_word(void)
{
  return 1U;
}

static uint16_t two_words(void)
{
  return 2U;
}

static uint16_t counter_word_count(void)
{
  return (uint16_t)MB_IREG_COUNTERS_WORDS;
}

static mb_exception_t read_adc(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  int32_t raw = 0;
  int32_t uv = 0;
  int32_t scaled = 0;

  (void)rtu;
  if (!Board_AdcRead((uint8_t)offset, &raw, &uv, &scaled))
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  /* Truncating to 16 bits is lossless for both cases: single-ended codes are
     0..65535 and differential codes are -32768..32767, and the master knows
     from the map which reading to interpret as signed. */
  *out = word_of((uint32_t)raw, 1U);
  return MB_EX_NONE;
}

static mb_exception_t read_dcbus(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  int32_t dc_raw = 0;
  int32_t mv = 0;

  (void)rtu;
  (void)offset;
  if (!Board_DcBus(&dc_raw, &mv))
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  *out = clamp_u16(mv);
  return MB_EX_NONE;
}

static mb_exception_t read_ntc(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  int32_t ntc_raw = 0;
  int32_t cc = 0;

  (void)rtu;
  (void)offset;
  if (!Board_Ntc(&ntc_raw, &cc))
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  *out = word_of((uint32_t)clamp_i16(cc), 1U);
  return MB_EX_NONE;
}

static mb_exception_t read_sysclk(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  (void)rtu;
  *out = word_of(Board_SysClkHz(), offset);
  return MB_EX_NONE;
}

static mb_exception_t read_hclk(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  (void)rtu;
  *out = word_of(Board_HclkHz(), offset);
  return MB_EX_NONE;
}

/* The counters live in the transport, not here, so the model carries the
   mb_rtu_t as its context. Two words a counter, high first. */
static mb_exception_t read_counter(const mb_rtu_t *rtu, uint16_t offset, uint16_t *out)
{
  const uint16_t idx = (uint16_t)(offset / 2U);

  if (rtu == NULL)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }

  const uint32_t counters[] = {
    rtu->counters.bus_message,
    rtu->counters.bus_comm_error,
    rtu->counters.server_message,
    rtu->counters.server_exception,
    rtu->counters.server_no_response,
    rtu->counters.char_overrun,
  };
  if (idx >= (uint16_t)(sizeof counters / sizeof counters[0]))
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
  *out = word_of(counters[idx], (uint16_t)(offset & 1U));
  return MB_EX_NONE;
}

static const ireg_span_t s_input_spans[] = {
  { 0U,                    adc_words,          read_adc     },
  { MB_IREG_DCBUS_MV,      one_word,           read_dcbus   },
  { MB_IREG_NTC_CENTI_C,   one_word,           read_ntc     },
  { MB_IREG_SYSCLK_HI,     two_words,          read_sysclk  },
  { MB_IREG_HCLK_HI,       two_words,          read_hclk    },
  { MB_IREG_COUNTERS_BASE, counter_word_count, read_counter },
};

/* The span an address falls in, its offset within it written back; NULL for
   a hole. The input register space has holes by design - the map is grouped
   for legibility rather than packed - and a hole is ILLEGAL DATA ADDRESS. */
static const ireg_span_t *span_of(uint16_t addr, uint16_t *offset)
{
  for (size_t i = 0U; i < (sizeof s_input_spans / sizeof s_input_spans[0]); i++)
  {
    const ireg_span_t *span = &s_input_spans[i];
    const uint16_t words = span->words();
    if ((addr >= span->base) && (addr < (uint16_t)(span->base + words)))
    {
      *offset = (uint16_t)(addr - span->base);
      return span;
    }
  }
  return NULL;
}

static bool input_reg_mapped(uint16_t addr)
{
  uint16_t offset = 0U;
  return span_of(addr, &offset) != NULL;
}

/* A read of several registers must either succeed wholly or fail wholly, so
   every address in the span is checked before any value is produced. */
static mb_exception_t validate_range(void *ctx, mb_table_t table, uint16_t addr,
                                     uint16_t qty, bool for_write)
{
  const uint32_t end = (uint32_t)addr + (uint32_t)qty;

  (void)ctx;
  if (for_write && ((table == MB_TABLE_INPUT_REG) || (table == MB_TABLE_DISCRETE_INPUT)))
  {
    return MB_EX_ILLEGAL_FUNCTION;
  }

  switch (table)
  {
    case MB_TABLE_INPUT_REG:
      for (uint16_t i = 0U; i < qty; i++)
      {
        if (!input_reg_mapped((uint16_t)(addr + i)))
        {
          return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
      }
      return MB_EX_NONE;

    case MB_TABLE_HOLDING_REG:
      return (end > (uint32_t)MB_HREG_COUNT) ? MB_EX_ILLEGAL_DATA_ADDRESS : MB_EX_NONE;

    case MB_TABLE_COIL:
      return (end > (uint32_t)MB_COIL_COUNT) ? MB_EX_ILLEGAL_DATA_ADDRESS : MB_EX_NONE;

    case MB_TABLE_DISCRETE_INPUT:
      return (end > (uint32_t)MB_DIN_COUNT) ? MB_EX_ILLEGAL_DATA_ADDRESS : MB_EX_NONE;

    default:
      return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
}

/* ---- reads ------------------------------------------------------------- */

static mb_exception_t read_hreg(uint16_t addr, uint16_t *out)
{
  switch (addr)
  {
    case MB_HREG_UNIT_ID:
      *out = (uint16_t)s_unit_id;
      return MB_EX_NONE;

    case MB_HREG_COMMAND:
      /* Write-only in effect: reading it back as 0 makes it obvious that no
         command is pending, rather than echoing a stale one. */
      *out = 0U;
      return MB_EX_NONE;

    default:
      return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
}

static mb_exception_t read_reg(void *ctx, mb_table_t table, uint16_t addr, uint16_t *out)
{
  uint16_t offset = 0U;
  const ireg_span_t *span = NULL;

  if (table == MB_TABLE_HOLDING_REG)
  {
    return read_hreg(addr, out);
  }
  if (table != MB_TABLE_INPUT_REG)
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
  span = span_of(addr, &offset);
  return (span != NULL) ? span->read((const mb_rtu_t *)ctx, offset, out)
                        : MB_EX_ILLEGAL_DATA_ADDRESS;
}

static mb_exception_t read_bit(void *ctx, mb_table_t table, uint16_t addr, bool *out)
{
  (void)ctx;

  if ((table == MB_TABLE_COIL) && (addr == MB_COIL_AFE_ON))
  {
    *out = Board_AfeOn();
    return MB_EX_NONE;
  }
  if ((table == MB_TABLE_DISCRETE_INPUT) && (addr == MB_DIN_PE15))
  {
    *out = Board_Pe15();
    return MB_EX_NONE;
  }
  return MB_EX_ILLEGAL_DATA_ADDRESS;
}

/* ---- writes ------------------------------------------------------------ */

/* Whether write_reg would accept this value, with no side effect - shared by
   the actual write and by validate_reg_value, so a multi-register write (FC
   0x10) can check every value in its span before applying any of them. Keeping
   one copy of the legality rule is the point: two copies are two places for
   the unit-id range or the command enum to drift apart. */
static mb_exception_t check_hreg_value(uint16_t addr, uint16_t value)
{
  switch (addr)
  {
    case MB_HREG_UNIT_ID:
      /* 0 and 248..255 are not addresses. The register exists and is
         writable, so this is a bad value rather than a bad address. */
      return ((value >= 1U) && (value <= 247U))
             ? MB_EX_NONE : MB_EX_ILLEGAL_DATA_VALUE;

    case MB_HREG_COMMAND:
      switch (value)
      {
        case 0U:                    /* no-op, so a block write spanning this
                                        register does not have to invent one */
        case MB_CMD_CONSOLE_MODE:
        case MB_CMD_CLEAR_COUNTERS:
          return MB_EX_NONE;
        default:
          return MB_EX_ILLEGAL_DATA_VALUE;
      }

    default:
      return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
}

static mb_exception_t validate_reg_value(void *ctx, uint16_t addr, uint16_t value)
{
  (void)ctx;
  return check_hreg_value(addr, value);
}

/** Holding register 1: a command. 0 is nothing to do; anything else
  * check_hreg_value has already refused. */
static mb_exception_t run_command(mb_rtu_t *rtu, uint16_t value)
{
  if (value == 0U)
  {
    return MB_EX_NONE;
  }
  if (value == MB_CMD_CONSOLE_MODE)
  {
    Board_RequestConsoleMode();
    return MB_EX_NONE;
  }
  if (value != MB_CMD_CLEAR_COUNTERS)
  {
    return MB_EX_ILLEGAL_DATA_VALUE;         /* unreachable, see above */
  }
  if (rtu == NULL)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  rtu->counters.bus_message        = 0U;
  rtu->counters.bus_comm_error     = 0U;
  rtu->counters.server_message     = 0U;
  rtu->counters.server_exception   = 0U;
  rtu->counters.server_no_response = 0U;
  rtu->counters.char_overrun       = 0U;
  return MB_EX_NONE;
}

static mb_exception_t write_reg(void *ctx, uint16_t addr, uint16_t value)
{
  const mb_exception_t bad = check_hreg_value(addr, value);

  if (bad != MB_EX_NONE)
  {
    return bad;
  }
  if (addr == MB_HREG_UNIT_ID)
  {
    (void)modbus_map_set_unit_id((uint8_t)value);   /* known valid: above */
    return MB_EX_NONE;
  }
  if (addr == MB_HREG_COMMAND)
  {
    return run_command((mb_rtu_t *)ctx, value);
  }
  return MB_EX_ILLEGAL_DATA_ADDRESS;         /* unreachable: check_span
                                                already gated the address */
}

static mb_exception_t write_bit(void *ctx, uint16_t addr, bool value)
{
  (void)ctx;

  if (addr != MB_COIL_AFE_ON)
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }

  /* Through the reference count, like every other way of asking for this
     rail. Writing the pin here worked until the observer took the rail for a
     sample: its release re-applies whatever the count says, which put the
     AFE straight back on and made a coil written off read back on. */
  if (value)
  {
    (void)Board_PowerAcquire(BOARD_RAIL_AFE, BOARD_USER_HOST);
  }
  else
  {
    (void)Board_PowerRelease(BOARD_RAIL_AFE, BOARD_USER_HOST);
  }
  return MB_EX_NONE;
}

/* ---- identity ---------------------------------------------------------- */

static const char *server_id(void *ctx, uint8_t *run)
{
  (void)ctx;

  /* 0xFF is the specified value for "ON", meaning the server is running. */
  *run = 0xFFU;
  return "coaxial_63100 STM32H753 rev1";
}

/* ---- the model -------------------------------------------------------- */

static mb_data_model_t s_model;

const mb_data_model_t *modbus_map_model(
    void *rtu_ctx,
    mb_exception_t (*user_function)(void *ctx, uint8_t fc,
                                    const uint8_t *req, size_t req_len,
                                    uint8_t *rsp, size_t rsp_cap, size_t *rsp_len))
{
  s_model.user_function      = user_function;
  s_model.validate_range     = validate_range;
  s_model.read_reg           = read_reg;
  s_model.write_reg          = write_reg;
  s_model.validate_reg_value = validate_reg_value;
  s_model.read_bit           = read_bit;
  s_model.write_bit          = write_bit;
  s_model.server_id          = server_id;
  s_model.ctx                = rtu_ctx;

  return &s_model;
}
