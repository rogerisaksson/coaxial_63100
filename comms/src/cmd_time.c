/**
  ******************************************************************************
  * @file    cmd_time.c
  * @brief   The cycle counter, latched, behind command 0x6E, device 7.
  *
  * Every timestamp here is raw CYCCNT (invariant 2), which leaves a host
  * holding ticks with no idea what o'clock they are. This is how it finds out.
  *
  * Op 0 latches the counter and its reply is worthless on purpose: it can be
  * BROADCAST, and a broadcast has no reply, so the board acts at an instant
  * the host can bracket with no turnaround in the middle. The unicast read
  * afterwards can be as late as it likes - the value stopped moving when it
  * was taken.
  *
  * No wall clock, and none to be given: no RTC and no LSE, so a time held here
  * would drift against nothing, and a plausible wrong time is worse than
  * ticks. The host owns the clock; this owns the ticks.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

static uint32_t s_latched;
static uint32_t s_seq;


/** op 0 - take the counter now. Broadcast this; the reply is the point of
  * op 1, not of this one. */
static cmd_status_t h_time_latch(wr_t *out)
{
  s_latched = Board_Cycles();
  s_seq++;
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 1 - what was latched, and what the counter says now.
  *
  * Both, because they answer different questions: the latch is the instant
  * the host bracketed, and `now` lets it see how long its own read took
  * without a second exchange.
  */
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
