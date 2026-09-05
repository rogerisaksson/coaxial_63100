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
  * `seconds` is how long the thermal observer has run. Without it an
  * estimate cannot be judged: the laminate's time constant is minutes, so
  * anything under a few has not settled. Judge it on the host - the board
  * does not judge (invariant 10).
  *
  * Ops:
  *   0  state       - measured NTC, estimated nodes, ambient, seconds;
  *                    MINOR 13 appends the three FET junction rises and
  *                    the speed
  *   1  set node    - u8 node, i32 k_per_w_milli, i32 capacity_milli: the
  *                    node's first path out and its J/K
  *   2  set board   - i32 to_ambient_milli, i32 capacity_milli: the bulk,
  *                    shared out by area
  *   3  set sample  - u32 every_ms, u32 settle_ms
  *   4  budget      - the SOA spend, one byte a node; MINOR 12 appends the
  *                    winding's estimate, spend and own factor
  *   5  set limit   - u8 node, i32 limit_milli_c, i32 throttle_ppm
  *   6  set winding - i32 limit_milli_c, i32 k_per_w_milli, i32 j_per_k_milli
  *   7  nodes       - u8 first: the network's node table from there, ten a
  *                    page: capacity, air path, area share, R_th, forced
  *   8  edges       - the whole edge table: a, b, K/W each
  *   9  set edge    - u8 edge, i32 k_per_w_milli; negative opens it
  ******************************************************************************
  */
#include "board.h"
#include "cmd.h"
#include "wire.h"

#define OP_STATE      0U
#define OP_SET_NODE   1U
#define OP_SET_BOARD  2U
#define OP_SET_SAMPLE 3U
#define OP_BUDGET     4U
#define OP_SET_LIMIT  5U
#define OP_SET_WINDING 6U
#define OP_NODES      7U
#define OP_EDGES      8U
#define OP_SET_EDGE   9U

/** Nodes a page of op 7 carries: five i32 each, so ten fit a frame. */
#define NODES_A_PAGE 10U


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

  /* Then the estimates, in node order. Append-only like everything here;
     the count is what a host follows. */
  wr_u8(out, (uint8_t)BOARD_THERMAL_NODES);
  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_i32(out, th.node_centidegc[i]);
  }
  wr_i32(out, th.ambient_centidegc);
  wr_i32(out, th.expected_ntc_centidegc);
  wr_u32(out, th.seconds);
  wr_u8(out, th.settled ? 1U : 0U);

  uint32_t every_ms = 0U, settle_ms = 0U;

  Board_ThermalSampling(&every_ms, &settle_ms);
  wr_u32(out, every_ms);
  wr_u32(out, settle_ms);

  /* The other two thermometers, each with its own flag. */
  wr_u8(out, th.afe_measured ? 1U : 0U);
  wr_i32(out, th.afe_centidegc);
  wr_u8(out, th.mcu_measured ? 1U : 0U);
  wr_i32(out, th.mcu_centidegc);
  wr_u32(out, th.seen_ms_ago);
  wr_u32(out, th.steps);

  /* MINOR 13, appended (invariant 3): each leg's FET junction over its
     node in centi-kelvin - what the datasheet's 175 C is against - and
     the rotor speed the air paths were evaluated at. */
  for (uint8_t leg = 0U; leg < 3U; leg++)
  {
    wr_i32(out, th.junction_over_centi[leg]);
  }
  wr_i32(out, th.speed_rpm);
  return CMD_OK;
}


static cmd_status_t op_set_node(rd_t *in, wr_t *out)
{
  const uint8_t node = rd_u8(in);
  const int32_t k_per_w = rd_i32(in);
  const int32_t capacity = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    cmd_took(out, "there are twenty nodes, 0..19 - op 0 lists them");
    return CMD_OK;
  }
  if ((k_per_w <= 0) || (capacity <= 0))
  {
    cmd_took(out, "a K/W and a heat capacity are both positive; "
                  "milli-units, so 12000 is 12 K/W");
    return CMD_OK;
  }
  if (!Board_ThermalSetNode(node, (float)k_per_w / 1000.0f,
                            (float)capacity / 1000.0f))
  {
    cmd_took(out, "the thermal observer is not running - it starts with the board");
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
    cmd_took(out, "the thermal observer is not running - it starts with the board");
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
    cmd_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 4 - what is left of the thermal budget. One byte a node, 0 at
  * ambient and 255 at the limit; degrees stay on op 0. */
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
  /* MINOR 11, appended (invariant 3). The clamp's factor in micro, the
     joules each node can still absorb in milli, and the effective duty
     per phase in micro. */
  wr_i32(out, (int32_t)(b.derate * 1000000.0f));
  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_i32(out, (int32_t)(b.soak_j[i] * 1000.0f));
  }
  for (uint8_t i = 0U; i < (uint8_t)BOARD_PWM_PHASES; i++)
  {
    wr_i32(out, (int32_t)(b.duty[i] * 1000000.0f));
  }
  /* MINOR 12, appended: the winding - its estimate in centi-degrees, its
     spend as a byte like a node's, and its OWN clamp factor in micro. */
  wr_i32(out, (int32_t)(b.winding_c * 100.0f));
  wr_u8(out, b.winding_used);
  wr_i32(out, (int32_t)(b.winding_derate * 1000000.0f));
  return CMD_OK;
}


/** op 6 - the winding's envelope: ceiling, K/W and J/K, milli-units. A
  * zero ceiling disables the winding; the constants must be positive. */
static cmd_status_t op_set_winding(rd_t *in, wr_t *out)
{
  const int32_t limit_milli = rd_i32(in);
  const int32_t k_per_w_milli = rd_i32(in);
  const int32_t j_per_k_milli = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((limit_milli < 0) || (k_per_w_milli <= 0) || (j_per_k_milli <= 0))
  {
    cmd_took(out, "milli-units: a ceiling of 120000 is 120 C and zero "
                  "disables it; 2200 is 2.2 K/W and 180000 is 180 J/K, "
                  "both positive");
    return CMD_OK;
  }
  if (!Board_ThermalSetWinding((float)limit_milli / 1000.0f,
                               (float)k_per_w_milli / 1000.0f,
                               (float)j_per_k_milli / 1000.0f))
  {
    cmd_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
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
    cmd_took(out, "there are twenty nodes, 0..19 - op 0 lists them");
    return CMD_OK;
  }
  if (!Board_ThermalSetLimit(node, (float)limit_milli / 1000.0f,
                             (float)throttle_ppm / 1000000.0f))
  {
    cmd_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


/** op 7 - the node table from `first`, NODES_A_PAGE at most: capacity in
  * milli J/K, the air path in milli K/W (0: none), the area share in ppm,
  * R_th in milli K/W, the forced-convection gain in milli. */
static cmd_status_t op_nodes(rd_t *in, wr_t *out)
{
  const uint8_t first = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (first >= (uint8_t)BOARD_THERMAL_NODES)
  {
    return CMD_ERR_VALUE;
  }
  uint8_t count = (uint8_t)(BOARD_THERMAL_NODES - first);

  if (count > NODES_A_PAGE)
  {
    count = NODES_A_PAGE;
  }
  wr_u8(out, (uint8_t)BOARD_THERMAL_NODES);
  wr_u8(out, first);
  wr_u8(out, count);
  for (uint8_t i = first; i < (uint8_t)(first + count); i++)
  {
    float capacity = 0.0f, to_ambient = 0.0f, share = 0.0f, rth = 0.0f;
    float forced = 0.0f;

    if (!Board_ThermalNodeCfg(i, &capacity, &to_ambient, &share, &rth,
                              &forced))
    {
      return CMD_ERR_DEVICE;
    }
    wr_i32(out, (int32_t)(capacity * 1000.0f));
    wr_i32(out, (int32_t)(to_ambient * 1000.0f));
    wr_i32(out, (int32_t)(share * 1000000.0f));
    wr_i32(out, (int32_t)(rth * 1000.0f));
    wr_i32(out, (int32_t)(forced * 1000.0f));
  }
  return CMD_OK;
}


/** op 8 - every edge: the two nodes it joins and the K/W across it in
  * milli, zero for an open one. */
static cmd_status_t op_edges(wr_t *out)
{
  wr_u8(out, (uint8_t)BOARD_THERMAL_EDGES);
  for (uint8_t e = 0U; e < (uint8_t)BOARD_THERMAL_EDGES; e++)
  {
    uint8_t a = 0U, b = 0U;
    float r = 0.0f;

    if (!Board_ThermalEdge(e, &a, &b, &r))
    {
      return CMD_ERR_DEVICE;
    }
    wr_u8(out, a);
    wr_u8(out, b);
    wr_i32(out, (int32_t)(r * 1000.0f));
  }
  return CMD_OK;
}


/** op 9 - one edge's K/W, milli; negative opens it. */
static cmd_status_t op_set_edge(rd_t *in, wr_t *out)
{
  const uint8_t edge = rd_u8(in);
  const int32_t k_per_w_milli = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (edge >= (uint8_t)BOARD_THERMAL_EDGES)
  {
    cmd_took(out, "there are thirty edges, 0..29 - op 8 lists them");
    return CMD_OK;
  }
  if (k_per_w_milli == 0)
  {
    cmd_took(out, "zero is no path at all - send a negative K/W to open an "
                  "edge, or a positive one in milli-units to set it");
    return CMD_OK;
  }
  if (!Board_ThermalSetEdge(edge, (float)k_per_w_milli / 1000.0f))
  {
    cmd_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  cmd_took(out, NULL);
  return CMD_OK;
}


cmd_status_t cmd_thermal_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case OP_STATE:       return op_state(out);
    case OP_SET_NODE:    return op_set_node(in, out);
    case OP_SET_BOARD:   return op_set_board(in, out);
    case OP_SET_SAMPLE:  return op_set_sample(in, out);
    case OP_BUDGET:      return op_budget(out);
    case OP_SET_LIMIT:   return op_set_limit(in, out);
    case OP_SET_WINDING: return op_set_winding(in, out);
    case OP_NODES:       return op_nodes(in, out);
    case OP_EDGES:       return op_edges(out);
    case OP_SET_EDGE:    return op_set_edge(in, out);
    default:             return CMD_ERR_VALUE;
  }
}
