/**
  ******************************************************************************
  * @file    cmd_time.c
  * @brief   The cycle counter, latched, behind command 0x6E, device 7.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

static uint32_t s_latched;
static uint32_t s_seq;


/** op 0 - take the counter now. */
static cmd_status_t h_time_latch(wr_t *out)
{
  s_latched = Board_Cycles();
  s_seq++;
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 1 - what was latched, and what the counter says now. */
static cmd_status_t h_time_read(wr_t *out)
{
  wr_u32(out, s_seq);
  wr_u32(out, s_latched);
  wr_u32(out, Board_Cycles());
  wr_u32(out, Board_SysClkHz());
  return CMD_OK;
}


cmd_status_t cmd_time_op(uint8_t op, rd_t *in, wr_t *out)
{
  (void)in;

  switch (op)
  {
    case TIME_OP_LATCH: return h_time_latch(out);
    case TIME_OP_READ:  return h_time_read(out);
    default:            return CMD_ERR_VALUE;
  }
}
