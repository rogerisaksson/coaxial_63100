/** cmd_wbc.c - The loop and the step without an active set ticked on her stand: 0x6E, device 13. */
#include "board.h"
#include "cmd.h"
#include "comms_limits.h"
#include "wire.h"
#include "wbc.h"
#include "wbc_model.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

/** Ticks one request runs at most: the first cold, the rest warm on the same stand. */
#define WBC_TICKS_MAX 16U

/* The loop's state, 471 KB, lives in the acquisition ring while it ticks (`Board_DaqLend`),
   the step's 25 KB there too; the body, 31 KB, and the out in DTCM. */
_Static_assert(sizeof(wbc_stack_t) <= DAQ_BYTES, "the loop's state outgrew the ring");
_Static_assert(sizeof(wbc_wrench_t) <= DAQ_BYTES, "the step's state outgrew the ring");
static wbc_body_t s_body;
static wbc_out_t  s_out;
static int        s_ready;

static int32_t milli_of(double x)
{
  return (int32_t)lrint(x * 1000.0);
}

/** Her stand, the model's own: the pelvis's frame, and the ask that keeps it - both soles,
    the turns and the posture as they are. */
static void stand(wbc_frame_t *base, wbc_ask_t *ask)
{
  if (!s_ready)
  {
    memset(&s_body, 0, sizeof(s_body));
    s_ready = 1;
  }
  memcpy(base->r, wbc_stand_base, sizeof(base->r));
  memcpy(base->p, wbc_stand_base + 9, sizeof(base->p));
  memset(ask, 0, sizeof(*ask));
  ask->stance[0] = ask->stance[1] = 1;
  wbc_body_pose(&s_body, base, wbc_stand_q);
  wbc_quat_of(s_body.t[0].r, ask->turn[0]);
  wbc_quat_of(s_body.t[wbc_trunk].r, ask->turn[1]);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    ask->posture[j] = wbc_stand_q[wbc_driven[j] - 1];
    ask->derate[j] = 1.0;
  }
}

/** op 0 - `ticks` ticks of the loop on her stand (the model's own) from a cleared memory: the
    first's and the last's cycles, the bounds held and the set's changes at the last, the first
    three drives' torques. */
static cmd_status_t h_wbc_tick(rd_t *in, wr_t *out)
{
  const uint8_t ticks = rd_u8(in);
  wbc_stack_t  *s;
  wbc_frame_t   base;
  wbc_ask_t     ask;
  double        u[WBC_N];
  uint32_t      first = 0U, last = 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((ticks == 0U) || (ticks > WBC_TICKS_MAX))
  {
    return CMD_ERR_VALUE;
  }
  s = (wbc_stack_t *)Board_DaqLend(sizeof(*s));
  if (s == NULL)
  {
    wr_took(out, "the acquisition runs, and the loop's state lives in its ring - stop it first");
    return CMD_OK;
  }
  memset(u, 0, sizeof(u));
  stand(&base, &ask);
  wbc_stack_init(s);
  for (uint8_t k = 0U; k < ticks; k++)
  {
    const uint32_t t0 = Board_Cycles();

    wbc_stack_step(&s_body, s, &base, wbc_stand_q, u, &ask, &s_out);
    last = Board_Cycles() - t0;
    if (k == 0U)
    {
      first = last;
    }
  }
  wr_u32(out, first);
  wr_u32(out, last);
  wr_u32(out, Board_SysClkHz());
  wr_u8(out, (uint8_t)s_out.held);
  wr_u8(out, (uint8_t)s_out.passes);
  wr_u8(out, (uint8_t)s_out.stuck);
  for (int j = 0; j < 3; j++)
  {
    wr_i32(out, milli_of(s_out.tau[j]));
  }
  return CMD_OK;
}

/** op 1 - `ticks` ticks of the step without an active set (wbc_wrench.c) on her stand: the
    first's and the last's cycles, the first three drives' torques, each sole's load. */
static cmd_status_t h_wbc_step(rd_t *in, wr_t *out)
{
  const uint8_t ticks = rd_u8(in);
  wbc_wrench_t *w;
  wbc_frame_t   base;
  wbc_ask_t     ask;
  double        u[WBC_N];
  uint32_t      first = 0U, last = 0U;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((ticks == 0U) || (ticks > WBC_TICKS_MAX))
  {
    return CMD_ERR_VALUE;
  }
  w = (wbc_wrench_t *)Board_DaqLend(sizeof(*w));
  if (w == NULL)
  {
    wr_took(out, "the acquisition runs, and the step's state lives in its ring - stop it first");
    return CMD_OK;
  }
  memset(u, 0, sizeof(u));
  stand(&base, &ask);
  wbc_wrench_init(w);
  for (uint8_t k = 0U; k < ticks; k++)
  {
    const uint32_t t0 = Board_Cycles();

    wbc_wrench_step(&s_body, w, &base, wbc_stand_q, u, &ask, &s_out);
    last = Board_Cycles() - t0;
    if (k == 0U)
    {
      first = last;
    }
  }
  wr_u32(out, first);
  wr_u32(out, last);
  wr_u32(out, Board_SysClkHz());
  for (int j = 0; j < 3; j++)
  {
    wr_i32(out, milli_of(s_out.tau[j]));
  }
  wr_i32(out, milli_of(s_out.bears[0]));
  wr_i32(out, milli_of(s_out.bears[1]));
  return CMD_OK;
}

cmd_status_t cmd_wbc_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case WBC_OP_TICK: return h_wbc_tick(in, out);
    case WBC_OP_STEP: return h_wbc_step(in, out);
    default:          return CMD_ERR_VALUE;
  }
}
