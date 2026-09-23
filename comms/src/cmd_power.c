/**
  ******************************************************************************
  * @file    cmd_power.c
  * @brief   The rail reference counts behind 0x6E, device 9.
  ******************************************************************************
  */
#include "board.h"
#include "board_power.h"
#include "cmd.h"
#include "wire.h"


static cmd_status_t h_power_state(wr_t *out)
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


/* No guard on the gate stage here. */
static cmd_status_t h_power_release_all(wr_t *out)
{
  Board_PowerReleaseAll();
  wr_took(out, NULL);
  return CMD_OK;
}


cmd_status_t cmd_power_op(uint8_t op, rd_t *in, wr_t *out)
{
  (void)in;

  switch (op)
  {
    case POWER_OP_STATE:       return h_power_state(out);
    case POWER_OP_RELEASE_ALL: return h_power_release_all(out);

    default:
      return CMD_ERR_VALUE;
  }
}
