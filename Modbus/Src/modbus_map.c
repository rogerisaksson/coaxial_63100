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

/* ---- address space extent --------------------------------------------- */

static uint16_t input_reg_end(void)
{
  return (uint16_t)(MB_IREG_COUNTERS_BASE + MB_IREG_COUNTERS_WORDS);
}

static bool input_reg_mapped(uint16_t addr)
{
  if (addr < (uint16_t)Board_AdcCount())
  {
    return true;
  }
  if ((addr == MB_IREG_DCBUS_MV) || (addr == MB_IREG_NTC_CENTI_C))
  {
    return true;
  }
  if ((addr >= MB_IREG_SYSCLK_HI) && (addr < (uint16_t)(MB_IREG_HCLK_HI + 2U)))
  {
    return true;
  }
  if ((addr >= MB_IREG_COUNTERS_BASE) && (addr < input_reg_end()))
  {
    return true;
  }
  return false;
}

/* A read of several registers must either succeed wholly or fail wholly, so
   every address in the span is checked before any value is produced. The input
   register space has holes in it by design - the map is grouped for legibility
   rather than packed - and a span crossing a hole is ILLEGAL DATA ADDRESS. */
static mb_exception_t validate_range(void *ctx, mb_table_t table, uint16_t addr,
                                     uint16_t qty, bool for_write)
{
  (void)ctx;

  switch (table)
  {
    case MB_TABLE_INPUT_REG:
      if (for_write)
      {
        return MB_EX_ILLEGAL_FUNCTION;
      }
      for (uint16_t i = 0U; i < qty; i++)
      {
        if (!input_reg_mapped((uint16_t)(addr + i)))
        {
          return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
      }
      return MB_EX_NONE;

    case MB_TABLE_HOLDING_REG:
      if ((uint32_t)addr + (uint32_t)qty > (uint32_t)MB_HREG_COUNT)
      {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
      }
      return MB_EX_NONE;

    case MB_TABLE_COIL:
      if ((uint32_t)addr + (uint32_t)qty > (uint32_t)MB_COIL_COUNT)
      {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
      }
      return MB_EX_NONE;

    case MB_TABLE_DISCRETE_INPUT:
      if (for_write)
      {
        return MB_EX_ILLEGAL_FUNCTION;
      }
      if ((uint32_t)addr + (uint32_t)qty > (uint32_t)MB_DIN_COUNT)
      {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
      }
      return MB_EX_NONE;

    default:
      return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
}

/* ---- reads ------------------------------------------------------------- */

/* The counters live in the transport, not here, so the model carries the
   mb_rtu_t as its context. Exposed high word first, matching the big-endian
   convention every other multi-register value on the wire follows. */
static const uint32_t *counter_words(const mb_rtu_t *rtu, uint8_t *n)
{
  static uint32_t snapshot[6];

  snapshot[0] = rtu->counters.bus_message;
  snapshot[1] = rtu->counters.bus_comm_error;
  snapshot[2] = rtu->counters.server_message;
  snapshot[3] = rtu->counters.server_exception;
  snapshot[4] = rtu->counters.server_no_response;
  snapshot[5] = rtu->counters.char_overrun;

  *n = 6U;
  return snapshot;
}

static mb_exception_t read_reg(void *ctx, mb_table_t table, uint16_t addr, uint16_t *out)
{
  mb_rtu_t *rtu = (mb_rtu_t *)ctx;

  if (table == MB_TABLE_HOLDING_REG)
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

  if (table != MB_TABLE_INPUT_REG)
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }

  if (addr < (uint16_t)Board_AdcCount())
  {
    int32_t raw = 0;
    int32_t uv = 0;
    int32_t scaled = 0;
    if (!Board_AdcRead((uint8_t)addr, &raw, &uv, &scaled))
    {
      return MB_EX_SERVER_DEVICE_FAILURE;
    }
    /* Truncating to 16 bits is lossless for both cases: single-ended codes are
       0..65535 and differential codes are -32768..32767, and the master knows
       from the map which reading to interpret as signed. */
    *out = (uint16_t)((uint32_t)raw & 0xFFFFU);
    return MB_EX_NONE;
  }

  if (addr == MB_IREG_DCBUS_MV)
  {
    int32_t dc_raw = 0;
    int32_t mv = 0;
    if (!Board_DcBus(&dc_raw, &mv))
    {
      return MB_EX_SERVER_DEVICE_FAILURE;
    }
    if (mv < 0)
    {
      mv = 0;
    }
    if (mv > 65535)
    {
      mv = 65535;
    }
    *out = (uint16_t)mv;
    return MB_EX_NONE;
  }

  if (addr == MB_IREG_NTC_CENTI_C)
  {
    int32_t ntc_raw = 0;
    int32_t cc = 0;
    if (!Board_Ntc(&ntc_raw, &cc))
    {
      return MB_EX_SERVER_DEVICE_FAILURE;
    }
    if (cc < -32768)
    {
      cc = -32768;
    }
    if (cc > 32767)
    {
      cc = 32767;
    }
    *out = (uint16_t)((uint32_t)cc & 0xFFFFU);
    return MB_EX_NONE;
  }

  if ((addr == MB_IREG_SYSCLK_HI) || (addr == (uint16_t)(MB_IREG_SYSCLK_HI + 1U)))
  {
    const uint32_t hz = Board_SysClkHz();
    *out = (addr == MB_IREG_SYSCLK_HI) ? (uint16_t)(hz >> 16) : (uint16_t)(hz & 0xFFFFU);
    return MB_EX_NONE;
  }

  if ((addr == MB_IREG_HCLK_HI) || (addr == (uint16_t)(MB_IREG_HCLK_HI + 1U)))
  {
    const uint32_t hz = Board_HclkHz();
    *out = (addr == MB_IREG_HCLK_HI) ? (uint16_t)(hz >> 16) : (uint16_t)(hz & 0xFFFFU);
    return MB_EX_NONE;
  }

  if ((addr >= MB_IREG_COUNTERS_BASE) && (addr < input_reg_end()))
  {
    if (rtu == NULL)
    {
      return MB_EX_SERVER_DEVICE_FAILURE;
    }

    uint8_t n = 0U;
    const uint32_t *c = counter_words(rtu, &n);
    const uint16_t off = (uint16_t)(addr - MB_IREG_COUNTERS_BASE);
    const uint16_t idx = (uint16_t)(off / 2U);

    if (idx >= (uint16_t)n)
    {
      return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    *out = ((off & 1U) == 0U) ? (uint16_t)(c[idx] >> 16) : (uint16_t)(c[idx] & 0xFFFFU);
    return MB_EX_NONE;
  }

  return MB_EX_ILLEGAL_DATA_ADDRESS;
}

static mb_exception_t read_bit(void *ctx, mb_table_t table, uint16_t addr, bool *out)
{
  (void)ctx;

  if (table == MB_TABLE_COIL)
  {
    if (addr != MB_COIL_AFE_ON)
    {
      return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    *out = Board_AfeOn();
    return MB_EX_NONE;
  }

  if (table == MB_TABLE_DISCRETE_INPUT)
  {
    if (addr != MB_DIN_PE15)
    {
      return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
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

static mb_exception_t write_reg(void *ctx, uint16_t addr, uint16_t value)
{
  mb_rtu_t *rtu = (mb_rtu_t *)ctx;
  const mb_exception_t bad = check_hreg_value(addr, value);

  if (bad != MB_EX_NONE)
  {
    return bad;
  }

  switch (addr)
  {
    case MB_HREG_UNIT_ID:
      (void)modbus_map_set_unit_id((uint8_t)value);   /* known valid: above */
      return MB_EX_NONE;

    case MB_HREG_COMMAND:
      switch (value)
      {
        case 0U:
          return MB_EX_NONE;

        case MB_CMD_CONSOLE_MODE:
          Board_RequestConsoleMode();
          return MB_EX_NONE;

        case MB_CMD_CLEAR_COUNTERS:
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

        default:
          return MB_EX_ILLEGAL_DATA_VALUE;   /* unreachable: check_hreg_value
                                                 above already refused this */
      }

    default:
      return MB_EX_ILLEGAL_DATA_ADDRESS;     /* unreachable: check_span
                                                 already gated the address */
  }
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
