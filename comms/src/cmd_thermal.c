/** cmd_thermal.c - The thermal observer behind 0x6E, device 8. */
#include "board.h"
#include "cmd.h"
#include "wire.h"
#include "board_units.h"

/** Nodes a page of op 7 carries: five i32 each, so ten fit a frame. */
#define NODES_A_PAGE 10U

static cmd_status_t h_thermal_state(wr_t *out)
{
  board_thermal_t th;

  if (!Board_ThermalState(&th))
  {
    return CMD_ERR_DEVICE;
  }

  /* Measured first, with its flag. */
  wr_u8(out, th.ntc_measured ? 1U : 0U);
  wr_i32(out, th.ntc_centidegc);

  /* Then the estimates, in node order. */
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

  /* MINOR 13, appended (invariant 3): each leg's FET junction over its node
     in centi-kelvin - what the datasheet's 175 C is against - and the rotor
     speed the air paths were evaluated at. */
  for (uint8_t leg = 0U; leg < 3U; leg++)
  {
    wr_i32(out, th.junction_over_centi[leg]);
  }
  wr_i32(out, th.speed_rpm);
  return CMD_OK;
}

static cmd_status_t h_thermal_set_node(rd_t *in, wr_t *out)
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
    wr_took(out, "there are twenty nodes, 0..19 - op 0 lists them");
    return CMD_OK;
  }
  if ((k_per_w <= 0) || (capacity <= 0))
  {
    wr_took(out, "a K/W and a heat capacity are both positive; "
                  "milli-units, so 12000 is 12 K/W");
    return CMD_OK;
  }
  if (!Board_ThermalSetNode(node, (float)k_per_w / MILLI_PER_UNIT,
                            (float)capacity / MILLI_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

static cmd_status_t h_thermal_set_board(rd_t *in, wr_t *out)
{
  const int32_t to_ambient = rd_i32(in);
  const int32_t capacity = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((to_ambient <= 0) || (capacity <= 0))
  {
    wr_took(out, "both are positive; milli-units, so 8330 is 8.33 K/W and "
                  "49000 is 49 J/K");
    return CMD_OK;
  }
  if (!Board_ThermalSetBoard((float)to_ambient / MILLI_PER_UNIT,
                             (float)capacity / MILLI_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

static cmd_status_t h_thermal_set_sample(rd_t *in, wr_t *out)
{
  const uint32_t every_ms = rd_u32(in);
  const uint32_t settle_ms = rd_u32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((every_ms != 0U) && (settle_ms >= every_ms))
  {
    wr_took(out, "the settle has to fit inside the period, or the rail is "
                  "never given back; 300 ms in 5000 is the default");
    return CMD_OK;
  }
  if (!Board_ThermalSetSample(every_ms, settle_ms))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

/** op 4 - what is left of the thermal budget. */
static cmd_status_t h_thermal_budget(wr_t *out)
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
  /* MINOR 11, appended (invariant 3). */
  wr_i32(out, (int32_t)(b.derate * PPM_PER_UNIT));
  for (uint8_t i = 0U; i < (uint8_t)BOARD_THERMAL_NODES; i++)
  {
    wr_i32(out, (int32_t)(b.soak_j[i] * MILLI_PER_UNIT));
  }
  for (uint8_t i = 0U; i < (uint8_t)BOARD_PWM_PHASES; i++)
  {
    wr_i32(out, (int32_t)(b.duty[i] * PPM_PER_UNIT));
  }
  /* MINOR 12, appended: the winding - its estimate in centi-degrees, its
     spend as a byte like a node's, and its OWN clamp factor in micro. */
  wr_i32(out, (int32_t)(b.winding_c * CENTI_PER_UNIT));
  wr_u8(out, b.winding_used);
  wr_i32(out, (int32_t)(b.winding_derate * PPM_PER_UNIT));
  return CMD_OK;
}

/** op 6 - the winding's envelope: ceiling, K/W and J/K, milli-units. */
static cmd_status_t h_thermal_set_winding(rd_t *in, wr_t *out)
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
    wr_took(out, "milli-units: a ceiling of 120000 is 120 C and zero "
                  "disables it; 2200 is 2.2 K/W and 180000 is 180 J/K, "
                  "both positive");
    return CMD_OK;
  }
  if (!Board_ThermalSetWinding((float)limit_milli / MILLI_PER_UNIT,
                               (float)k_per_w_milli / MILLI_PER_UNIT,
                               (float)j_per_k_milli / MILLI_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

static cmd_status_t h_thermal_set_limit(rd_t *in, wr_t *out)
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
    wr_took(out, "there are twenty nodes, 0..19 - op 0 lists them");
    return CMD_OK;
  }
  if (!Board_ThermalSetLimit(node, (float)limit_milli / MILLI_PER_UNIT,
                             (float)throttle_ppm / PPM_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

/** op 7 - the node table from `first`, NODES_A_PAGE at most: capacity in
    milli J/K, the air path in milli K/W (0: none), the area share in ppm,
    R_th in milli K/W, the forced-convection gain in milli. */
static cmd_status_t h_thermal_nodes(rd_t *in, wr_t *out)
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
    wr_i32(out, (int32_t)(capacity * MILLI_PER_UNIT));
    wr_i32(out, (int32_t)(to_ambient * MILLI_PER_UNIT));
    wr_i32(out, (int32_t)(share * PPM_PER_UNIT));
    wr_i32(out, (int32_t)(rth * MILLI_PER_UNIT));
    wr_i32(out, (int32_t)(forced * MILLI_PER_UNIT));
  }
  return CMD_OK;
}

/** op 8 - every edge: the two nodes it joins and the K/W across it in milli,
    zero for an open one. */
static cmd_status_t h_thermal_edges(wr_t *out)
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
    wr_i32(out, (int32_t)(r * MILLI_PER_UNIT));
  }
  return CMD_OK;
}

/** op 9 - one edge's K/W, milli; negative opens it. */
static cmd_status_t h_thermal_set_edge(rd_t *in, wr_t *out)
{
  const uint8_t edge = rd_u8(in);
  const int32_t k_per_w_milli = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (edge >= (uint8_t)BOARD_THERMAL_EDGES)
  {
    wr_took(out, "there are thirty edges, 0..29 - op 8 lists them");
    return CMD_OK;
  }
  if (k_per_w_milli == 0)
  {
    wr_took(out, "zero is no path at all - send a negative K/W to open an "
                  "edge, or a positive one in milli-units to set it");
    return CMD_OK;
  }
  if (!Board_ThermalSetEdge(edge, (float)k_per_w_milli / MILLI_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

/** op 10 - the identification beside the observer. */
static cmd_status_t h_thermal_ident(wr_t *out)
{
  board_thermal_ident_t id;

  if (!Board_ThermalIdent(&id))
  {
    return CMD_ERR_DEVICE;
  }
  wr_u8(out, id.state);
  wr_u8(out, id.online_mask);
  wr_u8(out, (uint8_t)BOARD_THERMAL_IDENT_SCALES);
  for (uint8_t k = 0U; k < (uint8_t)BOARD_THERMAL_IDENT_SCALES; k++)
  {
    wr_i32(out, (int32_t)(id.scale[k] * MILLI_PER_UNIT));
    wr_i32(out, (int32_t)(id.sigma[k] * MILLI_PER_UNIT));
  }
  wr_i32(out, (int32_t)(id.innovation_k * MILLI_PER_UNIT));
  wr_i32(out, (int32_t)(id.margin * PPM_PER_UNIT));
  wr_u32(out, id.updates);
  wr_u32(out, 0UL);                /* saves: none, the board keeps nothing */
  wr_u32(out, 0xFFFFFFFFUL);       /* since a save: never */
  /* MINOR 15, appended (invariant 3): the room as identified, centi-C, and
     its sigma in centi-kelvin. */
  wr_i32(out, (int32_t)(id.ambient_c * CENTI_PER_UNIT));
  wr_i32(out, (int32_t)(id.ambient_sigma_k * CENTI_PER_UNIT));
  /* MINOR 16, appended: the floor the margin rises from, micro. */
  wr_i32(out, (int32_t)(id.margin_floor * PPM_PER_UNIT));
  /* MINOR 17, appended: the trip cap as it stands, micro; one with none. */
  wr_i32(out, (int32_t)(id.trip_cap * PPM_PER_UNIT));
  return CMD_OK;
}

/** op 11 - forget what was identified: the margin back at the floor. */
static cmd_status_t h_thermal_ident_reset(wr_t *out)
{
  if (!Board_ThermalIdentReset())
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

/** op 12 - the margin floor, ppm of every ceiling's span, into the record;
    cal op 2 is what persists it. */
static cmd_status_t h_thermal_set_margin(rd_t *in, wr_t *out)
{
  const int32_t floor_ppm = rd_i32(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if ((floor_ppm <= 0) || (floor_ppm > 1000000L))
  {
    wr_took(out, "the floor is a fraction of the span, 1 .. 1 000 000 ppm - "
                  "800 000 is the bench's; zero would trip the stage at boot");
    return CMD_OK;
  }
  if (!Board_ThermalSetMarginFloor((float)floor_ppm / PPM_PER_UNIT))
  {
    wr_took(out, "the thermal observer is not running - it starts with the board");
    return CMD_OK;
  }
  wr_took(out, NULL);
  return CMD_OK;
}

cmd_status_t cmd_thermal_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case THERMAL_OP_STATE:       return h_thermal_state(out);
    case THERMAL_OP_SET_NODE:    return h_thermal_set_node(in, out);
    case THERMAL_OP_SET_BOARD:   return h_thermal_set_board(in, out);
    case THERMAL_OP_SET_SAMPLE:  return h_thermal_set_sample(in, out);
    case THERMAL_OP_BUDGET:      return h_thermal_budget(out);
    case THERMAL_OP_SET_LIMIT:   return h_thermal_set_limit(in, out);
    case THERMAL_OP_SET_WINDING: return h_thermal_set_winding(in, out);
    case THERMAL_OP_NODES:       return h_thermal_nodes(in, out);
    case THERMAL_OP_EDGES:       return h_thermal_edges(out);
    case THERMAL_OP_SET_EDGE:    return h_thermal_set_edge(in, out);
    case THERMAL_OP_IDENT:       return h_thermal_ident(out);
    case THERMAL_OP_IDENT_RESET: return h_thermal_ident_reset(out);
    case THERMAL_OP_SET_MARGIN:  return h_thermal_set_margin(in, out);
    default:             return CMD_ERR_VALUE;
  }
}
