/**
  ******************************************************************************
  * @file    cmd_cal.c
  * @brief   The calibration record's operations behind command 0x6E, device 3.
  *
  * Integers only, like every other payload here: microhms, ppm, microvolts,
  * centikelvin. A scale factor that arrived as a float would be the first
  * floating-point number on this wire and the last one anybody could decode
  * from a hex dump.
  *
  * Reads and edits are cheap and volatile; only op 5 writes flash. That split
  * is deliberate - a rig sets nine parameters and saves once, rather than
  * erasing a sector nine times.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

/** How many parameters op 0 carries: the fifteen it had at MINOR 1. The
    record grew to forty-five with the drive and the whole reply came to
    310 bytes against MB_MAX_PDU's 253 - measured 2026-08-31, every read
    answering SERVER DEVICE FAILURE. Op 0 keeps its shape for a host that
    never heard of the rest; op 8 pages all of them. */
#define CAL_LEGACY_PARAMS 15U

/** Parameters one op 8 reply carries: 60 x 4 = 240 bytes, plus three. */
#define CAL_PAGE 60U

/**
  * @brief op 0 - the whole record, plus whether flash holds one.
  *
  * `stored` is what separates "this board was calibrated" from "this board is
  * running the schematic's numbers", and the two are otherwise identical on
  * the wire.
  */
static cmd_status_t h_cal_get(rd_t *in, wr_t *out)
{
  const board_cal_t *cal = Board_Cal();

  (void)in;

  wr_u8(out, (uint8_t)(Board_CalStored() ? 1U : 0U));
  wr_u16(out, cal->version);
  wr_u8(out, (uint8_t)CAL_LEGACY_PARAMS);

  for (uint8_t id = 0U; id < CAL_LEGACY_PARAMS; id++)
  {
    uint32_t value = 0U;

    (void)Board_CalGetParam(id, &value);
    wr_u32(out, value);
  }

  wr_u8(out, (uint8_t)BOARD_CAL_CHANNELS);

  for (uint8_t i = 0U; i < BOARD_CAL_CHANNELS; i++)
  {
    int32_t offset = 0;
    int32_t gain = 0;

    (void)Board_CalChannel(i, &offset, &gain);
    wr_i32(out, offset);
    wr_i32(out, gain);
  }

  /* The thermal envelope, appended so an older host stops reading above it
     and still parses everything else. Without it "the ceilings are stored"
     is an assertion nobody on the wire can check. */
  wr_u8(out, (uint8_t)BOARD_THERMAL_NODES);

  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_i32(out, cal->soa_limit_centi[i]);
  }
  wr_u32(out, cal->soa_throttle_ppm);

  return CMD_OK;
}

/** @brief op 8 - every scalar parameter, paged from `first`. */
static cmd_status_t h_cal_params(rd_t *in, wr_t *out)
{
  const uint8_t first = (rd_left(in) > 0U) ? rd_u8(in) : 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (first > (uint8_t)BOARD_CAL_PARAM_COUNT)
  {
    return CMD_ERR_VALUE;
  }

  uint8_t count = (uint8_t)(BOARD_CAL_PARAM_COUNT - first);

  count = (count > CAL_PAGE) ? CAL_PAGE : count;
  wr_u8(out, (uint8_t)BOARD_CAL_PARAM_COUNT);
  wr_u8(out, first);
  wr_u8(out, count);
  for (uint8_t id = first; id < (uint8_t)(first + count); id++)
  {
    uint32_t value = 0U;

    (void)Board_CalGetParam(id, &value);
    wr_u32(out, value);
  }
  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}

/** @brief op 1 - one scalar parameter. */
static cmd_status_t h_cal_set_param(rd_t *in, wr_t *out)
{
  const uint8_t  id = rd_u8(in);
  const uint32_t value = rd_u32(in);

  (void)out;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (!Board_CalSetParam(id, value))
  {
    return CMD_ERR_VALUE;
  }

  return CMD_OK;
}

/** @brief op 2 - one channel's offset and gain trim, both at once. */
static cmd_status_t h_cal_set_channel(rd_t *in, wr_t *out)
{
  const uint8_t index = rd_u8(in);
  const int32_t offset = rd_i32(in);
  const int32_t gain = rd_i32(in);

  (void)out;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (!Board_CalSetChannel(index, offset, gain))
  {
    return CMD_ERR_VALUE;
  }

  return CMD_OK;
}

/**
  * @brief op 3 - measure the channel now and keep the reading as its offset.
  *
  * Answers what it measured, because an operator who zeroed the wrong channel
  * needs to see a number that says so.
  */
static cmd_status_t h_cal_zero(rd_t *in, wr_t *out)
{
  const uint8_t index = rd_u8(in);
  int32_t measured = 0;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (index >= BOARD_CAL_CHANNELS)
  {
    return CMD_ERR_VALUE;
  }
  if (!Board_CalZero(index, &measured))
  {
    return CMD_ERR_DEVICE;      /* the conversion did not complete */
  }

  wr_i32(out, measured);
  return CMD_OK;
}

/**
  * @brief op 4 - trim the gain so the channel reports `reference`.
  *
  * The reference is in the channel's own unit: milliamperes for a phase,
  * millivolts for the DC link. Any other channel is refused - see
  * Board_CalSpan on why a logarithmic conversion has no scale factor.
  */
static cmd_status_t h_cal_span(rd_t *in, wr_t *out)
{
  const uint8_t index = rd_u8(in);
  const int32_t reference = rd_i32(in);
  int32_t measured = 0;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (index >= BOARD_CAL_CHANNELS)
  {
    return CMD_ERR_VALUE;
  }
  if (!Board_CalSpan(index, reference, &measured))
  {
    return CMD_ERR_DEVICE;
  }

  wr_i32(out, measured);
  return CMD_OK;
}

/**
  * @brief op 5 - commit to flash.
  *
  * Erases and reprograms the last sector of bank 2, then reads it back. The
  * reply is the read-back, not the programmer's opinion.
  */
static cmd_status_t h_cal_save(rd_t *in, wr_t *out)
{
  (void)in;

  if (!Board_CalSave())
  {
    return CMD_ERR_DEVICE;
  }

  wr_u8(out, 1U);
  return CMD_OK;
}

/** @brief op 6 - re-read flash, discarding uncommitted edits. */
static cmd_status_t h_cal_load(rd_t *in, wr_t *out)
{
  (void)in;

  if (!Board_CalLoad())
  {
    return CMD_ERR_DEVICE;      /* nothing valid stored; record untouched */
  }

  wr_u8(out, 1U);
  return CMD_OK;
}

/**
  * @brief op 7 - back to the schematic's numbers.
  *
  * RAM only. A rig that meant it follows with op 5; one that did not can
  * still get its stored record back with op 6.
  */
static cmd_status_t h_cal_defaults(rd_t *in, wr_t *out)
{
  (void)in;

  Board_CalDefaults();
  wr_u8(out, 1U);
  return CMD_OK;
}

cmd_status_t cmd_cal_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case CAL_OP_GET:         return h_cal_get(in, out);
    case CAL_OP_SET_PARAM:   return h_cal_set_param(in, out);
    case CAL_OP_SET_CHANNEL: return h_cal_set_channel(in, out);
    case CAL_OP_ZERO:        return h_cal_zero(in, out);
    case CAL_OP_SPAN:        return h_cal_span(in, out);
    case CAL_OP_SAVE:        return h_cal_save(in, out);
    case CAL_OP_LOAD:        return h_cal_load(in, out);
    case CAL_OP_DEFAULTS:    return h_cal_defaults(in, out);
    case CAL_OP_PARAMS:      return h_cal_params(in, out);
    default:                 return CMD_ERR_VALUE;
  }
}
