/**
  ******************************************************************************
  * @file    cmd_thermal.c
  * @brief   The thermal observer behind 0x6E, device 8.
  *
  * MEASURED AND ESTIMATED SIT IN SEPARATE FIELDS, which is the whole point of
  * the reply's shape. The NTC is a measurement and comes with a flag saying
  * whether it exists; the node temperatures are estimates from power and
  * time. A reply that put them in one list would be the mistake invariant 9
  * is about - a value that looks like a measurement without being one.
  *
  * `seconds` is how long the observer has run. Without it an estimate cannot
  * be judged: the network's time constant is 6.8 minutes, so anything under a
  * few minutes has not settled. Judge it on the host - the board does not
  * judge (invariant 10).
  *
  * Ops:
  *   0  state      - measured NTC, estimated nodes, ambient, seconds
  *   1  set node   - u8 node, i32 to_board_milli, i32 capacity_milli
  *   2  set board  - i32 to_ambient_milli, i32 capacity_milli
 *   3  set sample - u32 every_ms, u32 settle_ms
 *   4  budget     - the SOA spend, one byte a node
 *   5  set limit  - u8 node, i32 limit_milli_c, i32 throttle_ppm
  ******************************************************************************
  */
#include "board.h"
#include "cmd.h"
#include "wire.h"

#define OP_STATE     0U
#define OP_SET_NODE  1U
#define OP_SET_BOARD 2U
#define OP_SET_SAMPLE 3U
#define OP_BUDGET     4U
#define OP_SET_LIMIT  5U


static cmd_status_t op_state(wr_t *out)
{
  board_thermal_t th;

  if (!Board_ThermalState(&th))
  {
    return CMD_ERR_DEVICE;
  }

  /* Measured first, with its flag. AFE_ON low means no reference and so no
     measurement at all - not a zero, an absence. */
  wr_u8(out, th.ntc_measured ? 1U : 0U);
  wr_i32(out, th.ntc_centidegc);

  /* Then the estimates, in node order. Append-only like everything here. */
  wr_u8(out, (uint8_t)BOARD_THERMAL_NODES);
  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_i32(out, th.node_centidegc[i]);
  }
  wr_i32(out, th.ambient_centidegc);
  wr_i32(out, th.expected_ntc_centidegc);
  wr_u32(out, th.seconds);
  wr_u8(out, th.settled ? 1U : 0U);

  /* What the sampling is set to. Appended, so an older host stops reading
     here and still parses everything above it. */
  uint32_t every_ms = 0U, settle_ms = 0U;

  Board_ThermalSampling(&every_ms, &settle_ms);
  wr_u32(out, every_ms);
  wr_u32(out, settle_ms);

  /* The other two thermometers, each with its own flag. Separate fields
     because they answer separately: all three share AFE_ON, but a die that
     did not respond over SPI is not the same as a rail that was down. */
  wr_u8(out, th.afe_measured ? 1U : 0U);
  wr_i32(out, th.afe_centidegc);
  wr_u8(out, th.mcu_measured ? 1U : 0U);
  wr_i32(out, th.mcu_centidegc);
  wr_u32(out, th.seen_ms_ago);
  return CMD_OK;
}


static cmd_status_t op_set_node(rd_t *in, wr_t *out)
{
  const uint8_t node = rd_u8(in);
  const int32_t to_board = rd_i32(in);
  const int32_t capacity = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    cmd_took(out, "there are six nodes, 0..5 - see 0x6E device 8 op 0");
    return CMD_OK;
  }
  if ((to_board <= 0) || (capacity <= 0))
  {
    cmd_took(out, "a spreading resistance and a heat capacity are both "
                  "positive; milli-units, so 15200 is 15.2 K/W");
    return CMD_OK;
  }
  if (!Board_ThermalSetNode(node, (float)to_board / 1000.0f,
                            (float)capacity / 1000.0f))
  {
    cmd_took(out, "the observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


static cmd_status_t op_set_board(rd_t *in, wr_t *out)
{
  const int32_t to_ambient = rd_i32(in);
  const int32_t capacity = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((to_ambient <= 0) || (capacity <= 0))
  {
    cmd_took(out, "both are positive; milli-units, so 8330 is 8.33 K/W and "
                  "49000 is 49 J/K");
    return CMD_OK;
  }
  if (!Board_ThermalSetBoard((float)to_ambient / 1000.0f,
                             (float)capacity / 1000.0f))
  {
    cmd_took(out, "the observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


static cmd_status_t op_set_sample(rd_t *in, wr_t *out)
{
  const uint32_t every_ms = rd_u32(in);
  const uint32_t settle_ms = rd_u32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((every_ms != 0U) && (settle_ms >= every_ms))
  {
    cmd_took(out, "the settle has to fit inside the period, or the rail is "
                  "never given back; 300 ms in 5000 is the default");
    return CMD_OK;
  }
  if (!Board_ThermalSetSample(every_ms, settle_ms))
  {
    cmd_took(out, "the observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 4 - what is left of the thermal budget.
  *
  * One byte a node, 0 at ambient and 255 at the limit; degrees stay on op 0.
  * `millis_to_limit` is what a burst plans on - 35 W into the phase node
  * crosses the throttle point with under a second to go.
  */
static cmd_status_t op_budget(wr_t *out)
{
  board_budget_t b;

  if (!Board_ThermalBudget(&b))
  {
    return CMD_ERR_DEVICE;
  }

  wr_u8(out, (uint8_t)BOARD_THERMAL_NODES);
  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_u8(out, b.used[i]);
  }
  wr_u8(out, b.worst);
  wr_u8(out, b.worst_node);
  wr_i32(out, b.millis_to_limit);
  wr_u8(out, b.throttling ? 1U : 0U);
  wr_u8(out, b.tripped ? 1U : 0U);
  wr_u32(out, b.trips);
  return CMD_OK;
}


static cmd_status_t op_set_limit(rd_t *in, wr_t *out)
{
  const uint8_t node = rd_u8(in);
  const int32_t limit_milli = rd_i32(in);
  const int32_t throttle_ppm = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    cmd_took(out, "there are six nodes, 0..5 - see 0x6E device 8 op 0");
    return CMD_OK;
  }
  if (!Board_ThermalSetLimit(node, (float)limit_milli / 1000.0f,
                             (float)throttle_ppm / 1000000.0f))
  {
    cmd_took(out, "the observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


cmd_status_t cmd_thermal_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case OP_STATE:     return op_state(out);
    case OP_SET_NODE:  return op_set_node(in, out);
    case OP_SET_BOARD: return op_set_board(in, out);
    case OP_SET_SAMPLE: return op_set_sample(in, out);
    case OP_BUDGET:     return op_budget(out);
    case OP_SET_LIMIT:  return op_set_limit(in, out);
    default:           return CMD_ERR_VALUE;
  }
}
