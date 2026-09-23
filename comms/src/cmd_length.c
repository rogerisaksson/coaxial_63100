/** cmd_length.c - Request-length oracle: where a PDU ends by its own bytes. */
#include "cmd_length.h"

#include "cmd.h"
#include "modbus_slave.h"

/** Standard Modbus request lengths, from the specification: these shapes are
    the protocol's own and cannot drift with this repository. */
#define MB_REQ_ADDR_VALUE_LEN 5U   /* fc, u16 address, u16 quantity or value */
#define MB_REQ_MULTI_HEAD     6U   /* the same and a byte count, then that many bytes */

static uint16_t standard_length(const uint8_t *pdu, uint16_t have)
{
  switch (pdu[0])
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
    case MB_FC_READ_HOLDING_REGS:
    case MB_FC_READ_INPUT_REGS:
    case MB_FC_WRITE_SINGLE_COIL:
    case MB_FC_WRITE_SINGLE_REG:
      return MB_REQ_ADDR_VALUE_LEN;
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGS:
      return (have >= MB_REQ_MULTI_HEAD)
               ? (uint16_t)(MB_REQ_MULTI_HEAD + pdu[MB_REQ_MULTI_HEAD - 1U]) : 0U;
    default:
      return 0U;
  }
}

#define DEVICE_HEAD 3U   /* fc, device, op: every 0x6E request starts so */

/** The audited fixed shapes behind 0x6E. */
static uint16_t device_length(const uint8_t *pdu, uint16_t have)
{
  const uint8_t device = pdu[1];
  const uint8_t op     = pdu[2];

  switch (device)
  {
    case DEVICE_CAL:
      if (op == CAL_OP_SET_PARAM)        /* u8 id, u32 */
      {
        return DEVICE_HEAD + 5U;
      }
      return (op == CAL_OP_GET) ? DEVICE_HEAD : 0U;

    case DEVICE_GATE_DRIVERS:
      /* Op 2 has two shapes since MINOR 8 - u16 x3, or that plus a u32
         period count. */
      if (op == GATEDRIVERS_OP_DUTY)
      {
        return (have >= 10U) ? 13U : 0U;
      }
      return (op == GATEDRIVERS_OP_STATE) ? DEVICE_HEAD : 0U;

    case DEVICE_DAQ:
      /* Op 4's `want` is optional: three bytes might be whole, so three
         proves nothing; the fourth settles it. */
      if (op == DAQ_OP_READ)
      {
        return (have > DEVICE_HEAD) ? (DEVICE_HEAD + 1U) : 0U;
      }
      if ((op == DAQ_OP_STATE) || (op == DAQ_OP_START) || (op == DAQ_OP_STOP)
          || (op == DAQ_OP_LAYOUT) || (op == DAQ_OP_LIVE))
      {
        return DEVICE_HEAD;
      }
      return 0U;

    case DEVICE_TIME:
      return ((op == TIME_OP_LATCH) || (op == TIME_OP_READ)) ? DEVICE_HEAD : 0U;

    case DEVICE_THERMAL:
      return ((op == THERMAL_OP_STATE) || (op == THERMAL_OP_BUDGET)) ? DEVICE_HEAD : 0U;

    case DEVICE_POWER:
      return (op == 0U) ? DEVICE_HEAD : 0U;   /* the rails have one op */

    case DEVICE_DRIVE:
      switch (op)
      {
        case DRIVE_OP_STATE:        return DEVICE_HEAD;
        case DRIVE_OP_MODE:         return DEVICE_HEAD + 1U;   /* u8 mode */
        case DRIVE_OP_SETPOINT:     return DEVICE_HEAD + 5U;   /* u8 id, i32 */
        case DRIVE_OP_SETPOINTS:    return DEVICE_HEAD;
        case DRIVE_OP_THETA:        return DEVICE_HEAD + 4U;   /* i32 */
        case DRIVE_OP_CYCLES_RESET: return DEVICE_HEAD;
        case DRIVE_OP_MODEL:        return DEVICE_HEAD;
        case DRIVE_OP_MODEL_RESET:  return DEVICE_HEAD;
        default:                    return 0U;
      }

    default:
      return 0U;
  }
}

uint16_t cmd_request_length(const uint8_t *pdu, uint16_t have)
{
  if ((pdu == NULL) || (have == 0U))
  {
    return 0U;
  }

  if (pdu[0] == CMD_DEVICE)
  {
    /* 0x6E's ops carry no req_len column anywhere - the table above is the
       one hand-maintained answer, and the suite's prefix sweep is what keeps
       it honest. */
    return (have >= DEVICE_HEAD) ? device_length(pdu, have) : 0U;
  }

  /* Every other custom command states its own request length in the dispatch
     table - the same row the handler is found by, so this cannot drift from
     what dispatch enforces. */
  const cmd_desc_t *d = cmd_find(pdu[0]);

  if (d != NULL)
  {
    return (d->req_len == CMD_LEN_VARIABLE) ? 0U
                                            : (uint16_t)(1U + d->req_len);
  }
  return standard_length(pdu, have);
}
