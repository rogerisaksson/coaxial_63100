/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   The observer: integrate the network, then correct it with
  *          whichever thermometers answered.
  *
  * Explicit Euler. The fastest node here has a time constant of tens of
  * seconds and the caller steps this a few times a second, so the stability
  * limit is orders of magnitude away - an implicit solver would buy accuracy
  * nobody can measure. `thermal_step` clamps dt anyway, because a main loop
  * that stalled is exactly when a big dt would arrive.
  ******************************************************************************
  */
#include "thermal.h"

#include <math.h>
#include <string.h>

/** Longest step the integration is allowed to take, seconds.
  *
  * Not stability - it is that a gap this long means the loop was blocked, and
  * integrating across it pretends to know what happened in between. */
#define THERMAL_DT_MAX 2.0f

/** How hard the sensors pull the model per second, 1/s.
  *
  * Low enough that sensor noise does not shake the estimate, high enough that
  * a wrong initial guess is gone in a minute or two. The NTC quantises at
  * about 30 mK and TSEN at 125 mK, so there is nothing to gain from chasing
  * them faster. */
#define THERMAL_ANCHOR_HZ 0.05f

void thermal_defaults(thermal_cfg_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }
  memset(cfg, 0, sizeof(*cfg));

  /* CALIBRATED AGAINST A THERMAL CAMERA 2026-08-28, four states against a
     dead patch of soldermask (emissivity ~0.95, room 20 C).

       state                dead    mcu   regulators  bridge   afe
       1 passive (AFE off)  30.0  +15.0        +8.0    +1.0   +1.0
       2 AFE on, idle       31.1  +14.2        +8.1       -   +5.9
       3 AFE on, full DAQ   31.4  +13.6        +7.6       -   +5.9
       4 AFE off, 3 legs    40.0  +17.3       +20.0   +10.1    0.0

     The differences are what was measured; the absolute level rests on the
     supply's 50 mA, and that supply's shunt is not trustworthy.

       2-1  the AFE chain          +1.1 K -> 0.13 W
       3-2  full DAQ and link      +0.3 K -> 0.04 W
       4-1  switching alone       +10.0 K -> 1.20 W

     The switching 1.20 W fell roughly half on the supply corner and half on
     the bridge - gate charge comes out of the +15V7 buck, so the loss lands
     in the regulators and not only in the drivers. The bridge resistance then
     matches exactly (+10.1 predicted against +10.1 measured); the supply
     corner sits 3 K under, so there is one more term there that scales with
     switching. */
  cfg->board_to_ambient = 8.33f;

  cfg->node[THERMAL_DRIVERS].to_board    = 15.2f;   /* the bridge, from 4-1  */
  cfg->node[THERMAL_PHASES].to_board     = 15.2f;   /* same zone as above */
  cfg->node[THERMAL_MCU].to_board        = 22.5f;
  cfg->node[THERMAL_REGULATORS].to_board = 15.0f;
  cfg->node[THERMAL_AFE].to_board        = 41.5f;
  cfg->node[THERMAL_BOARD].to_board      = 0.0f;    /* it is the board */

  /* Heat capacity. The board dominates: tau 6.8 min against 8.33 K/W is
     about 49 J/K. The parts' own are not measured - they respond in seconds,
     below what this rig can resolve, and only affect the settling. */
  cfg->node[THERMAL_DRIVERS].capacity    = 0.35f;
  cfg->node[THERMAL_PHASES].capacity     = 1.20f;
  cfg->node[THERMAL_MCU].capacity        = 0.90f;
  cfg->node[THERMAL_REGULATORS].capacity = 0.80f;
  cfg->node[THERMAL_AFE].capacity        = 0.30f;
  cfg->node[THERMAL_BOARD].capacity      = 49.0f;

  /* The NTC sits beside a gate driver. Against the camera it read 6.0 K
     over dead board in the passive state (36.0 against 30.0) WHERE NO DRIVER
     WAS WARMING ANYTHING, so that part is mounting. While switching it read
     15.6 K over (55.6 against 40.0), so 9.6 K of it follows driver power. */
  /* Junction over package, for the two parts that report their own die.
     MCU: the camera read the package at 45.0 C in the passive state and the
     internal sensor read 72.0 C - 27 K, and ASSUMED rather than measured,
     because the two readings are from different sessions at different board
     temperatures. It is the term's order of magnitude, not its value.
     A1335: its die read 37.47 C against the camera's 37.0 for the same
     zone, so under a kelvin - it dissipates almost nothing. */
  cfg->node[THERMAL_MCU].die_over_node = 27.0f;
  cfg->node[THERMAL_AFE].die_over_node = 0.5f;

  cfg->ntc_sees_drivers = 1.055f;
  cfg->ntc_offset       = 6.00f;
}

/** Net watts into one node right now: what it makes less what it sheds.
  *
  * The same two terms `thermal_step` integrates, so the budget's dead
  * reckoning and the model cannot drift apart.
  */
static float net_watt(const thermal_t *th, const thermal_power_t *p,
                      thermal_node_t node)
{
  const float made = p->watt[node];

  if (node == THERMAL_BOARD)
  {
    float into = 0.0f;

    for (int i = 0; i < THERMAL_NODES; i++)
    {
      if ((i != THERMAL_BOARD) && (th->cfg.node[i].to_board > 0.0f))
      {
        into += (th->t[i] - th->t[THERMAL_BOARD]) / th->cfg.node[i].to_board;
      }
    }
    const float lost = (th->cfg.board_to_ambient > 0.0f)
                       ? ((th->t[THERMAL_BOARD] - th->ambient)
                          / th->cfg.board_to_ambient) : 0.0f;
    return made + into - lost;
  }

  if (th->cfg.node[node].to_board <= 0.0f)
  {
    return made;
  }
  return made - ((th->t[node] - th->t[THERMAL_BOARD])
                 / th->cfg.node[node].to_board);
}


void thermal_budget(const thermal_t *th, const thermal_power_t *p,
                    const thermal_soa_t *soa, thermal_budget_t *out)
{
  if ((th == NULL) || (p == NULL) || (soa == NULL) || (out == NULL))
  {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->millis_to_limit = -1;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    const float limit = soa->limit_c[i];
    const float span = limit - th->ambient;

    if (!(span > 0.0f))
    {
      continue;              /* no limit set, or one below ambient */
    }

    float part = (th->t[i] - th->ambient) / span;

    if (part < 0.0f)
    {
      part = 0.0f;
    }
    if (part > 1.0f)
    {
      part = 1.0f;
    }
    out->used[i] = (uint8_t)(part * 255.0f);

    if (out->used[i] >= out->worst)
    {
      out->worst = out->used[i];
      out->worst_node = (uint8_t)i;
    }
  }

  out->throttling = ((float)out->worst / 255.0f) >= soa->throttle_at;
  out->tripped = (out->worst >= 255U);

  /* Time left, for the node that has least of it. Capacity over net power:
     if it is not gaining, it is not heading for the limit and -1 says so
     rather than a large number that reads like a promise. */
  const thermal_node_t node = (thermal_node_t)out->worst_node;
  const float gain = net_watt(th, p, node);
  const float capacity = th->cfg.node[node].capacity;
  const float togo = soa->limit_c[node] - th->t[node];

  if ((gain > 0.0f) && (capacity > 0.0f) && (togo > 0.0f))
  {
    const float millis = 1000.0f * togo * capacity / gain;

    out->millis_to_limit = (millis > 2.0e9f) ? 2000000000 : (int32_t)millis;
  }
}


void thermal_losses(thermal_loss_t *loss)
{
  if (loss == NULL)
  {
    return;
  }
  memset(loss, 0, sizeof(*loss));

  /* The IAUCN10S7N021 VDMOS model in electronic_simulations: Ron 1.8 mOhm. */
  loss->rds_on = 1.8e-3f;

  /* RU1||RU2, two Vishay WSHM28187L000FEA of 7 mOhm - docs/HARDWARE.md.
     THIS ONE DOMINATES UNDER LOAD: 100 A through 3.5 mOhm is 35 W against
     the whole dry budget's 1.2 W. No switching parameter matters beside it. */
  loss->r_shunt = 3.5e-3f;

  /* The LM5069's pass FET. Not measured - the camera gave hot swap +6 K
     while switching, but no load went through it then. A plausible value for
     a hot-swap FET until somebody draws current through it. */
  loss->r_hotswap = 5.0e-3f;

  /* Measured 2026-08-28: three legs, 50 %, 24.6 V link -> 1.20 W from
     difference 4-1 on the dead surface. Half fell on the supply corner (gate
     charge comes out of the +15V7 buck) and half on the bridge. */
  loss->switching_watt = 1.20f;
  loss->switch_volts   = 24.6f;
  loss->driver_share   = 0.50f;

  /* Static. Consistent with the supply's 50 mA: 0.666+0.484+0.05 = 1.20 W. */
  loss->mcu_watt = 0.666f;
  loss->ldo_watt = 0.534f;
  loss->afe_watt = 0.13f;      /* from 2-1: the whole AFE chain and sensors */
}

void thermal_power_estimate(thermal_power_t *out, const thermal_load_t *load,
                            const thermal_loss_t *loss)
{
  if ((out == NULL) || (load == NULL) || (loss == NULL))
  {
    return;
  }
  memset(out, 0, sizeof(*out));

  /* Conduction: the current goes through the FET AND the shunt, both in the
     phase. Duty picks which FET conducts, but both halves carry the same
     squared current over a period, so the sum does not depend on duty. */
  float conduction = 0.0f;
  float link_from_phases = 0.0f;
  int legs_driven = 0;

  for (int i = 0; i < 3; i++)
  {
    const float a = load->phase_amps[i];
    conduction += a * a * (loss->rds_on + loss->r_shunt);
    link_from_phases += load->duty[i] * a;
    if (load->switching && (load->duty[i] > 0.0f))
    {
      legs_driven++;
    }
  }
  out->watt[THERMAL_PHASES] = conduction;

  /* Switching: scaled by link voltage and how many legs are driven. The
     C_oss charge goes as Q(V)*V, so nearer linear than square in this range -
     see python_examples/loss_calculation.py. */
  if (load->switching && (legs_driven > 0) && (loss->switch_volts > 0.0f))
  {
    const float scale = (load->link_volts / loss->switch_volts)
                        * ((float)legs_driven / 3.0f);
    const float sw = loss->switching_watt * scale;
    out->watt[THERMAL_DRIVERS]    += sw * loss->driver_share;
    out->watt[THERMAL_REGULATORS] += sw * (1.0f - loss->driver_share);
  }

  /* Hot swap: it is in the link, so it sees link current. With none
     measured it is estimated from the phases - what the link has to supply
     when nothing is stored. This board senses link VOLTS, not amps. */
  const float link_a = (load->link_amps >= 0.0f) ? load->link_amps
                                                 : link_from_phases;
  out->watt[THERMAL_REGULATORS] += link_a * link_a * loss->r_hotswap;

  /* Static. The AFE only draws while AFE_ON is high - and then the drivers
     have no supply, which the switching term above already handles through
     `switching`. */
  out->watt[THERMAL_MCU]        += loss->mcu_watt;
  out->watt[THERMAL_REGULATORS] += loss->ldo_watt;
  out->watt[THERMAL_AFE]        += load->afe_on ? loss->afe_watt : 0.0f;
}

bool thermal_set_node(thermal_t *th, thermal_node_t node,
                      float to_board, float capacity)
{
  if ((th == NULL) || (node >= THERMAL_NODES)
      || !(to_board > 0.0f) || !(capacity > 0.0f))
  {
    return false;
  }
  th->cfg.node[node].to_board = to_board;
  th->cfg.node[node].capacity = capacity;
  return true;
}

bool thermal_set_board(thermal_t *th, float to_ambient, float capacity)
{
  if ((th == NULL) || !(to_ambient > 0.0f) || !(capacity > 0.0f))
  {
    return false;
  }
  th->cfg.board_to_ambient = to_ambient;
  th->cfg.node[THERMAL_BOARD].capacity = capacity;
  return true;
}

bool thermal_set_ntc(thermal_t *th, float offset, float sees_drivers)
{
  /* No upper bound of 1.0. There was one, and it was an unfounded
     assumption: the NTC can rise MORE than the node's surface when it sits
     closer to the heat than the point the node stands for, and on this board
     it does - solved against both camera states it is 1.055. A cap at 1.0
     made the model miss the switching state by 5.6 K with no way to say so. */
  if ((th == NULL) || (sees_drivers < 0.0f) || (sees_drivers > 4.0f))
  {
    return false;
  }
  th->cfg.ntc_offset = offset;
  th->cfg.ntc_sees_drivers = sees_drivers;
  return true;
}

void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius)
{
  if (th == NULL)
  {
    return;
  }
  memset(th, 0, sizeof(*th));
  if (cfg != NULL)
  {
    th->cfg = *cfg;
  }
  else
  {
    thermal_defaults(&th->cfg);
  }
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    th->t[i] = celsius;
  }
  th->ambient = celsius;
}

float thermal_expected_ntc(const thermal_t *th)
{
  if (th == NULL)
  {
    return NAN;
  }
  const float board = th->t[THERMAL_BOARD];
  const float rise  = th->t[THERMAL_DRIVERS] - board;
  return board + th->cfg.ntc_sees_drivers * rise + th->cfg.ntc_offset;
}

float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float driver_rise_k)
{
  if (cfg == NULL)
  {
    return ntc_c;
  }
  return ntc_c - cfg->ntc_offset - cfg->ntc_sees_drivers * driver_rise_k;
}

/** Pull one node to its own die reading, and return the board that implies.
  *
  * The node sits at board + P*theta, both of which the model carries, so
  * subtracting them turns a die reading into a board reading - one that owes
  * nothing to the NTC's position beside a gate driver.
  */
static float anchor_die(thermal_t *th, thermal_node_t node, float seen,
                        const thermal_power_t *p, float k)
{
  /* The die reads the junction; the node is the package. Take the
     junction-to-case rise off first or it is booked as a hotter board. */
  const float at = seen - th->cfg.node[node].die_over_node;

  th->t[node] += k * (at - th->t[node]);
  return at - p->watt[node] * th->cfg.node[node].to_board;
}


void thermal_step(thermal_t *th, const thermal_power_t *p,
                  const thermal_sense_t *seen, float dt_s)
{
  if ((th == NULL) || (p == NULL) || (seen == NULL) || !(dt_s > 0.0f))
  {
    return;
  }
  if (dt_s > THERMAL_DT_MAX)
  {
    dt_s = THERMAL_DT_MAX;
  }

  float board = th->t[THERMAL_BOARD];
  float into_board = 0.0f;

  /* Every source node: what it makes, less what it sheds into the board. */
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    if (i == THERMAL_BOARD)
    {
      continue;
    }
    const thermal_node_cfg_t *n = &th->cfg.node[i];
    if (!(n->to_board > 0.0f) || !(n->capacity > 0.0f))
    {
      continue;
    }
    const float shed = (th->t[i] - board) / n->to_board;
    th->t[i] += (p->watt[i] - shed) * dt_s / n->capacity;
    into_board += shed;
  }

  /* The board: what the sources gave it, less what it loses to air. */
  const thermal_node_cfg_t *b = &th->cfg.node[THERMAL_BOARD];
  if ((b->capacity > 0.0f) && (th->cfg.board_to_ambient > 0.0f))
  {
    const float lost = (board - th->ambient) / th->cfg.board_to_ambient;
    th->t[THERMAL_BOARD] += (into_board + p->watt[THERMAL_BOARD] - lost)
                            * dt_s / b->capacity;
  }

  /* Anchor. Every die that answered corrects its own node and hands back the
     board it implies; the board is pulled toward their mean. The NTC then
     only has to explain what is left, which is the drivers' own rise - the
     one thing its position makes it good at. */
  const float k = THERMAL_ANCHOR_HZ * dt_s;
  float implied = 0.0f;
  int dies = 0;

  if (!isnan(seen->afe_c))
  {
    implied += anchor_die(th, THERMAL_AFE, seen->afe_c, p, k);
    dies++;
  }
  if (!isnan(seen->mcu_c))
  {
    implied += anchor_die(th, THERMAL_MCU, seen->mcu_c, p, k);
    dies++;
  }

  if (dies > 0)
  {
    th->t[THERMAL_BOARD] += k * (implied / (float)dies
                                 - th->t[THERMAL_BOARD]);

    /* Settled is about the BOARD, not the drivers. A die anchors it without
       anyone having to guess how much of the NTC's reading is hot spot, and
       that is exactly the guess the flag exists to warn about. The drivers
       node is a separate question, and the NTC below is what answers it. */
    th->settled = true;

    if (!isnan(seen->ntc_c))
    {
      /* What the NTC sees beyond the board is the drivers' share of their
         rise. Invert that to correct the drivers node. */
      const float over = seen->ntc_c - th->cfg.ntc_offset
                         - th->t[THERMAL_BOARD];
      if (th->cfg.ntc_sees_drivers > 0.01f)
      {
        const float at = th->t[THERMAL_BOARD]
                         + over / th->cfg.ntc_sees_drivers;
        th->t[THERMAL_DRIVERS] += k * (at - th->t[THERMAL_DRIVERS]);
      }
    }

    /* Ambient is what the board's own losses imply, once it is anchored.
       The board carries no ambient sensor, so this is the only way to it. */
    const float lost = into_board + p->watt[THERMAL_BOARD];
    th->ambient += k * ((th->t[THERMAL_BOARD]
                         - lost * th->cfg.board_to_ambient) - th->ambient);
  }
  else if (!isnan(seen->ntc_c))
  {
    /* Degraded: no die answered, so the drivers' rise cannot be separated
       from the board's. Anchor the board on the NTC with the modelled hot
       spot removed and say the estimate is not settled. */
    const float rise = th->t[THERMAL_DRIVERS] - th->t[THERMAL_BOARD];
    const float bulk = thermal_board_from_ntc(&th->cfg, seen->ntc_c, rise);
    th->t[THERMAL_BOARD] += k * (bulk - th->t[THERMAL_BOARD]);
    th->settled = false;
  }

  th->steps++;
}
