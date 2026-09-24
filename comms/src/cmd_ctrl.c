/** cmd_ctrl.c - The board's loop behind 0x6E, device 12. */
#include "board.h"
#include "board_units.h"
#include "cmd.h"
#include "ctrl.h"
#include "wire.h"

#include <math.h>

/** Rows one op 3 carries: 40 x 6 bytes. */
#define CTRL_ROWS_PER_OP 40U

static int32_t milli_of(float x)
{
  return (int32_t)lrintf(x * MILLI_PER_UNIT);
}

/** A decimal: i32 mantissa, i8 exponent. */
static float rd_dec(rd_t *in)
{
  float v = (float)rd_i32(in);
  int8_t e = rd_i8(in);

  while (e > 0)
  {
    v *= 10.0f;
    e--;
  }
  while (e < 0)
  {
    v /= 10.0f;
    e++;
  }
  return v;
}

/** op 0 - the loop as it stands. */
static cmd_status_t h_ctrl_state(wr_t *out)
{
  board_ctrl_state_t st;

  Board_CtrlState(&st);
  wr_u8(out, (uint8_t)((st.running ? 0x01U : 0U) | (st.playing ? 0x02U : 0U)));
  wr_u8(out, st.measured);
  wr_u8(out, st.command);
  wr_u16(out, st.hz);
  wr_u16(out, st.rows);
  wr_u16(out, st.free);
  wr_u32(out, (uint32_t)lrintf(st.queued_s * MILLI_PER_UNIT));
  wr_u32(out, st.played);
  wr_u32(out, st.idle);
  wr_u32(out, st.blind);
  wr_i32(out, milli_of(st.setpoint));
  wr_i32(out, milli_of(st.ref));
  wr_i32(out, milli_of(st.value));
  wr_i32(out, milli_of(st.estimate));
  wr_i32(out, milli_of(st.out));
  return CMD_OK;
}

/** op 1 - a slot as a kind and its parameters. */
static cmd_status_t h_ctrl_slot(rd_t *in, wr_t *out)
{
  const uint8_t slot = rd_u8(in);
  const uint8_t kind = rd_u8(in);
  const uint8_t n = rd_u8(in);
  float params[CTRL_PARAMS];

  if (n > CTRL_PARAMS)
  {
    wr_took(out, "at most seven parameters a part");
    return CMD_OK;
  }
  for (uint8_t i = 0U; i < n; i++)
  {
    params[i] = rd_dec(in);
  }
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  wr_took(out, Board_CtrlSlot(slot, kind, params, n));
  return CMD_OK;
}

/** op 2 - what it measures, where its command goes, how often. */
static cmd_status_t h_ctrl_wire(rd_t *in, wr_t *out)
{
  const uint8_t measured = rd_u8(in);
  const uint8_t command = rd_u8(in);
  const uint16_t hz = rd_u16(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  wr_took(out, Board_CtrlWire(measured, command, hz));
  return CMD_OK;
}

/** op 3 - rows, each held for its milliseconds; all or none. */
static cmd_status_t h_ctrl_rows(rd_t *in, wr_t *out)
{
  const uint8_t n = rd_u8(in);
  uint16_t ms[CTRL_ROWS_PER_OP];
  float setpoint[CTRL_ROWS_PER_OP];

  if (n > CTRL_ROWS_PER_OP)
  {
    wr_took(out, "at most forty rows an op");
    return CMD_OK;
  }
  for (uint8_t i = 0U; i < n; i++)
  {
    ms[i] = rd_u16(in);
    setpoint[i] = (float)rd_i32(in) / MILLI_PER_UNIT;
  }
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  wr_took(out, Board_CtrlRows(ms, setpoint, n));
  return CMD_OK;
}

/** op 4 - the queue dropped, the setpoint held. */
static cmd_status_t h_ctrl_clear(wr_t *out)
{
  Board_CtrlClear();
  wr_took(out, NULL);
  return CMD_OK;
}

/** op 5 - on or off. */
static cmd_status_t h_ctrl_run(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  wr_took(out, Board_CtrlRun(on != 0U));
  return CMD_OK;
}

cmd_status_t cmd_ctrl_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case CTRL_OP_STATE: return h_ctrl_state(out);
    case CTRL_OP_SLOT:  return h_ctrl_slot(in, out);
    case CTRL_OP_WIRE:  return h_ctrl_wire(in, out);
    case CTRL_OP_ROWS:  return h_ctrl_rows(in, out);
    case CTRL_OP_CLEAR: return h_ctrl_clear(out);
    case CTRL_OP_RUN:   return h_ctrl_run(in, out);

    default:
      return CMD_ERR_VALUE;
  }
}
