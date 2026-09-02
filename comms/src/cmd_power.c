/**
  ******************************************************************************
  * @file    cmd_power.c
  * @brief   The rail reference counts behind 0x6E, device 9.
  *
  * The mask says WHICH subsystem holds a rail, which is the difference
  * between a diagnosis and a guess - the thermal observer once took AFE_ON, a
  * starved poll never released it, and nothing on the wire said why.
  *
  * `on` is the PIN, read back. Reporting both is for the case where the pin
  * and the count disagree.
  *
  * Ops:
  *   0  state        - u8 rails, then per rail: on, users, count, blocked, leased
  *   1  release all  - drop every hold; blunt, for recovering a leak
  ******************************************************************************
  */
#include "board.h"
#include "board_power.h"
#include "cmd.h"
#include "wire.h"

#define OP_STATE       0U
#define OP_RELEASE_ALL 1U


static cmd_status_t op_state(wr_t *out)
{
  wr_u8(out, (uint8_t)BOARD_RAIL_COUNT);

  for (uint8_t rail = 0U; rail < (uint8_t)BOARD_RAIL_COUNT; rail++)
  {
    board_rail_state_t st;

    if (!Board_PowerState((board_rail_t)rail, &st))
    {
      return CMD_ERR_DEVICE;
    }
    wr_u8(out, st.on ? 1U : 0U);
    wr_u8(out, st.users);
    wr_u8(out, st.count);
    wr_u8(out, st.blocked ? 1U : 0U);
    wr_u8(out, st.leased);
  }
  return CMD_OK;
}


cmd_status_t cmd_power_op(uint8_t op, rd_t *in, wr_t *out)
{
  (void)in;

  switch (op)
  {
    case OP_STATE:
      return op_state(out);

    case OP_RELEASE_ALL:
      /* No guard on the gate stage here. Releasing every hold switches the
         AFE rail OFF, which gives the drivers their supply rather than
         taking it away - the direction that is safe while armed. */
      Board_PowerReleaseAll();
      cmd_took(out, NULL);
      return CMD_OK;

    default:
      return CMD_ERR_VALUE;
  }
}
