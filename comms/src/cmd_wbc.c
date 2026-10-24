/** cmd_wbc.c - The loop and the step ticked on her stand, a sole swung too: 0x6E, device 13. */
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
/** Op 2's swung knee, asked folded to this, rad. */
#define WBC_SWUNG_KNEE 0.6

/* The loop's state, 471 KB, lives in the acquisition ring while it ticks (`Board_DaqLend`),
   the step's 43 KB there too; the body, 31 KB, and the out in DTCM. */
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
  ask->mu[0] = ask->mu[1] = WBC_MU;
  wbc_body_pose(&s_body, base, wbc_stand_q);
  wbc_quat_of(s_body.t[0].r, ask->turn[0]);
  wbc_quat_of(s_body.t[wbc_trunk].r, ask->turn[1]);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    ask->posture[j] = wbc_stand_q[wbc_driven[j] - 1];
    ask->derate[j] = 1.0;
    ask->lo[j] = -HUGE_VAL;
    ask->hi[j] = HUGE_VAL;
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

/** `ticks` ticks of the step without an active set (wbc_wrench.c) for `ask` at her stand's
    pose: the first's and the last's cycles, the first three drives' torques, each sole's
    load. */
static void stepped(wr_t *out, uint8_t ticks, const wbc_frame_t *base, const wbc_ask_t *ask)
{
  wbc_wrench_t *w = (wbc_wrench_t *)Board_DaqLend(sizeof(*w));
  double        u[WBC_N];
  uint32_t      first = 0U, last = 0U;

  if (w == NULL)
  {
    wr_took(out, "the acquisition runs, and the step's state lives in its ring - stop it first");
    return;
  }
  memset(u, 0, sizeof(u));
  wbc_wrench_init(w);
  for (uint8_t k = 0U; k < ticks; k++)
  {
    const uint32_t t0 = Board_Cycles();

    wbc_wrench_step(&s_body, w, base, wbc_stand_q, u, ask, &s_out);
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
}

/** op 1 - the step on her stand. */
static cmd_status_t h_wbc_step(rd_t *in, wr_t *out)
{
  const uint8_t ticks = rd_u8(in);
  wbc_frame_t   base;
  wbc_ask_t     ask;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((ticks == 0U) || (ticks > WBC_TICKS_MAX))
  {
    return CMD_ERR_VALUE;
  }
  stand(&base, &ask);
  stepped(out, ticks, &base, &ask);
  return CMD_OK;
}

/** op 2 - the step with her right sole swung where it is, its knee asked folded, and the
    left's moment past its room: its four rounds; striding `strides` hundredths, what the
    floor cannot give her inertia's (wbc_cone.c) - whole at 100, the rounds beside it
    between. */
static cmd_status_t h_wbc_swung(rd_t *in, wr_t *out)
{
  const uint8_t ticks = rd_u8(in);
  const uint8_t strides = rd_u8(in);
  wbc_frame_t   base;
  wbc_ask_t     ask;

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((ticks == 0U) || (ticks > WBC_TICKS_MAX) || (strides > 100U))
  {
    return CMD_ERR_VALUE;
  }
  stand(&base, &ask);
  ask.stance[1] = 0;
  ask.swing[1] = 1;
  wbc_body_point(&s_body, wbc_sole[1], wbc_sole_at, ask.swing_at[1]);
  wbc_quat_of(s_body.t[wbc_sole[1]].r, ask.swing_quat[1]);
  ask.fold[1] = 1;
  ask.fold_knee[1] = WBC_SWUNG_KNEE;
  ask.strides = (double)strides / 100.0;
  stepped(out, ticks, &base, &ask);
  return CMD_OK;
}

cmd_status_t cmd_wbc_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case WBC_OP_TICK: return h_wbc_tick(in, out);
    case WBC_OP_STEP: return h_wbc_step(in, out);
    case WBC_OP_SWUNG: return h_wbc_swung(in, out);
    default:          return CMD_ERR_VALUE;
  }
}
