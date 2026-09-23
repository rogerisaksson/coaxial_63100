/**
  ******************************************************************************
  * @file    cmd_angle.c
  * @brief   The A1335's operations behind command 0x6E, device 1.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

/** op 0 - one register, as sixteen data bits and four CRC bits. */
static cmd_status_t h_angle_read(rd_t *in, wr_t *out)
{
  const uint8_t reg = rd_u8(in);
  uint16_t value = 0U;
  uint8_t  crc = 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (reg > 0x3FU)
  {
    return CMD_ERR_VALUE;          /* six address bits, Figure 31 */
  }
  if (!Board_AngleReady() && !Board_AngleInit())
  {
    return CMD_ERR_DEVICE;
  }
  if (!Board_AngleRead(reg, &value, &crc))
  {
    return CMD_ERR_DEVICE;
  }

  wr_u8(out, reg);
  wr_u16(out, value);
  wr_u8(out, crc);
  return CMD_OK;
}

/** @brief op 1 - eight data bits into one register. */
static cmd_status_t h_angle_write(rd_t *in, wr_t *out)
{
  const uint8_t reg = rd_u8(in);
  const uint8_t value = rd_u8(in);

  (void)out;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (reg > 0x3FU)
  {
    return CMD_ERR_VALUE;
  }
  if (!Board_AngleReady() && !Board_AngleInit())
  {
    return CMD_ERR_DEVICE;
  }
  if (!Board_AngleWrite(reg, value))
  {
    return CMD_ERR_DEVICE;
  }

  return CMD_OK;
}

/** op 2 - the poll loop's shared record. */
static cmd_status_t h_angle_latest(rd_t *in, wr_t *out)
{
  board_angle_state_t st;

  (void)in;

  Board_AngleState(&st);

  wr_u8(out, st.loop);
  wr_u8(out, st.error);
  wr_u32(out, st.updates);
  wr_u32(out, st.errors);
  wr_u8(out, st.have ? 1U : 0U);
  wr_u8(out, st.reg);
  wr_u16(out, st.value);
  wr_u8(out, st.crc);

  return CMD_OK;
}

/** ops 3 and 4 - stop the poll loop so the part can be configured, */
static cmd_status_t h_angle_hold(rd_t *in, wr_t *out)
{
  board_angle_state_t st;

  (void)in;

  Board_AngleHold();
  Board_AngleState(&st);
  wr_u8(out, st.loop);
  return CMD_OK;
}

static cmd_status_t h_angle_resume(rd_t *in, wr_t *out)
{
  board_angle_state_t st;

  (void)in;

  Board_AngleResume();
  Board_AngleState(&st);
  wr_u8(out, st.loop);
  return CMD_OK;
}

/** op 5 - which register the loop reads, set or asked. */
static cmd_status_t h_angle_pollreg(rd_t *in, wr_t *out)
{
  if ((rd_left(in) > 0U) && !Board_AnglePollReg(rd_u8(in)))
  {
    return CMD_ERR_VALUE;
  }

  wr_u8(out, Board_AnglePollRegGet());
  return CMD_OK;
}

/** @brief op 6 - SPI4's kernel clock and the bitrate derived from it. */
static cmd_status_t h_angle_clock(rd_t *in, wr_t *out)
{
  uint32_t kernel = 0U;
  uint32_t bitrate = 0U;

  (void)in;

  Board_AngleClock(&kernel, &bitrate);
  wr_u32(out, kernel);
  wr_u32(out, bitrate);
  return CMD_OK;
}

/* Whether the host holds the part's loop - what a register read or write
   needs. */
static bool angle_held(void)
{
  board_angle_state_t st;

  Board_AngleState(&st);
  return st.loop == BOARD_ANGLE_LOOP_HELD;
}


cmd_status_t cmd_angle_op(uint8_t op, rd_t *in, wr_t *out)
{
  /* Ops that drive SPI4 are refused while the poll loop runs, the same way
     the IMU's are: hold, configure, resume. */
  if (((op == ANGLE_OP_READ) || (op == ANGLE_OP_WRITE)) && !angle_held())
  {
    return CMD_ERR_DEVICE;
  }

  switch (op)
  {
    case ANGLE_OP_READ:    return h_angle_read(in, out);
    case ANGLE_OP_WRITE:   return h_angle_write(in, out);
    case ANGLE_OP_LATEST:  return h_angle_latest(in, out);
    case ANGLE_OP_HOLD:    return h_angle_hold(in, out);
    case ANGLE_OP_RESUME:  return h_angle_resume(in, out);
    case ANGLE_OP_POLLREG: return h_angle_pollreg(in, out);
    case ANGLE_OP_CLOCK:   return h_angle_clock(in, out);
    default:               return CMD_ERR_VALUE;
  }
}
