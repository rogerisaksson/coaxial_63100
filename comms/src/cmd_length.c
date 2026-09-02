/**
  ******************************************************************************
  * @file    cmd_length.c
  * @brief   The request-length oracle: which PDUs end where their bytes say.
  *
  * `mb_rtu`'s early path delivers a frame the moment its last byte arrives
  * when this function can prove the length - the spec's t3.5 silence is a
  * delimiter, and a frame whose shape is fixed carries its own. What this
  * buys, from the measured anatomy (FINDINGS, *Where the write-class
  * transaction's 15 ms goes*): 1.75 ms of board-side silence per proven
  * request, and the host may drop its own pre-TX gap against a board that
  * dispatches this way (MINOR 9).
  *
  * THE INVARIANT THIS TABLE LIVES UNDER: an answer other than 0 must equal
  * the full length of every real request it can match - a shorter answer
  * would execute a truncated frame on a 1-in-65536 CRC. So:
  *
  *   - only shapes with a FIXED tail are answered, and only once enough
  *     bytes have arrived to rule the shorter form out;
  *   - an op that later grows an optional tail (as gate drivers op 2 did)
  *     MUST have its row changed in the same commit, and the suite's
  *     prefix sweep (test_modbus_core) fails the row that fires early;
  *   - everything unproven answers 0 and pays the silence, exactly as
  *     every frame did before this file existed.
  *
  * Portable C11, no HAL: the host suite builds it with gcc and drives it
  * through ctypes beside the Modbus core it serves.
  ******************************************************************************
  */
#include "cmd_length.h"

#include "cmd.h"

/** Standard Modbus request lengths, from the specification: these shapes
  * are the protocol's own and cannot drift with this repository. */
static uint16_t standard_length(const uint8_t *pdu, uint16_t have)
{
  switch (pdu[0])
  {
    case 0x01U:                          /* read coils                    */
    case 0x02U:                          /* read discrete inputs          */
    case 0x03U:                          /* read holding registers        */
    case 0x04U:                          /* read input registers          */
    case 0x05U:                          /* write single coil             */
    case 0x06U:                          /* write single register         */
      return 5U;
    case 0x0FU:                          /* write multiple coils          */
    case 0x10U:                          /* write multiple registers      */
      /* fc, addr u16, qty u16, byte count, then that many bytes. */
      return (have >= 6U) ? (uint16_t)(6U + pdu[5]) : 0U;
    default:
      return 0U;
  }
}

/** The audited fixed shapes behind 0x6E. Each row is the op's whole
  * request - `fc, device, op` is 3 - and a row exists only where the
  * handler takes nothing optional past it. */
static uint16_t device_length(const uint8_t *pdu, uint16_t have)
{
  const uint8_t device = pdu[1];
  const uint8_t op     = pdu[2];

  switch (device)
  {
    case 3U:                             /* the calibration record        */
      if (op == 1U)                      /* set param: u8 id, u32         */
      {
        return 8U;
      }
      return (op == 0U) ? 3U : 0U;

    case 4U:                             /* the gate drivers              */
      /* Op 2 has two shapes since MINOR 8 - u16 x3, or that plus a u32
         period count. Nine bytes might be a whole short form, so nine
         proves nothing; a tenth byte rules the short form out and the
         long form's length is settled. */
      if (op == 2U)
      {
        return (have >= 10U) ? 13U : 0U;
      }
      return (op == 0U) ? 3U : 0U;

    case 6U:                             /* the acquisition task          */
      /* Op 4's `want` is optional: three bytes might be whole, so three
         proves nothing; the fourth settles it. The host always sends
         want, so its reads are all proven. */
      if (op == 4U)
      {
        return (have >= 4U) ? 4U : 0U;
      }
      if ((op == 0U) || (op == 2U) || (op == 3U) || (op == 5U)
          || (op == 6U))
      {
        return 3U;
      }
      return 0U;

    case 7U:                             /* the cycle counter             */
      return ((op == 0U) || (op == 1U)) ? 3U : 0U;

    case 8U:                             /* the thermal observer          */
      return ((op == 0U) || (op == 4U)) ? 3U : 0U;

    case 9U:                             /* the rails                     */
      return (op == 0U) ? 3U : 0U;

    case 10U:                            /* the drive                     */
      switch (op)
      {
        case 0U:  return 3U;             /* state                         */
        case 1U:  return 4U;             /* mode: u8                      */
        case 2U:  return 8U;             /* setpoint: u8 id, i32          */
        case 3U:  return 3U;             /* setpoints                     */
        case 4U:  return 7U;             /* theta: i32                    */
        case 9U:  return 3U;             /* cycles reset                  */
        case 12U: return 3U;             /* model                         */
        case 13U: return 3U;             /* model reset                   */
        default:  return 0U;
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
    /* 0x6E's ops carry no req_len column anywhere - the table above is
       the one hand-maintained answer, and the suite's prefix sweep is
       what keeps it honest. */
    return (have >= 3U) ? device_length(pdu, have) : 0U;
  }

  /* Every other custom command states its own request length in the
     dispatch table - the same row the handler is found by, so this
     cannot drift from what dispatch enforces. CMD_LEN_VARIABLE (the
     ADC table's optional start, the channel map's paged kinds, echo)
     stays unproven and pays the silence. */
  const cmd_desc_t *d = cmd_find(pdu[0]);

  if (d != NULL)
  {
    return (d->req_len == CMD_LEN_VARIABLE) ? 0U
                                            : (uint16_t)(1U + d->req_len);
  }
  return standard_length(pdu, have);
}
