/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   The thermal observer: integrate the network, then correct it with
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

  /* Three times the lumped value each, so the three in parallel are the
     15.2 K/W the camera measured. The split moved where the heat is drawn,
     not how much of it there is. */
  for (int leg = 0; leg < 3; leg++)
  {
    cfg->node[THERMAL_DRIVER(leg)].to_board = 45.6f;
    cfg->node[THERMAL_PHASE(leg)].to_board  = 45.6f;
  }
  cfg->node[THERMAL_MCU].to_board        = 22.5f;
  cfg->node[THERMAL_REGULATORS].to_board = 15.0f;
  cfg->node[THERMAL_AFE].to_board        = 41.5f;
  cfg->node[THERMAL_BOARD].to_board      = 0.0f;    /* it is the board */

  /* Heat capacity. The board dominates: tau 6.8 min against 8.33 K/W is
     about 49 J/K. The parts' own are not measured - they respond in seconds,
     below what this rig can resolve, and only affect the settling. */
  for (int leg = 0; leg < 3; leg++)
  {
    /* A third each, so the three together store what the lumped node did -
       and a single leg now warms three times as fast, which is the whole
       point of asking which one is switching. */
    cfg->node[THERMAL_DRIVER(leg)].capacity = 0.35f / 3.0f;
    cfg->node[THERMAL_PHASE(leg)].capacity  = 1.20f / 3.0f;
  }
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


/** How long this node can stay at this power before its ceiling, seconds.
  *
  * The soak divided by what is going into it: `capacity * (limit - t)`
  * over the net watts. ONE DEFINITION, because the throttle and the
  * reported `millis_to_limit` are the same question asked by two callers,
  * and a throttle acting on one number while the host plans on another
  * would be two envelopes.
  *
  * Negative when it is not heading there at all - a node that is cooling
  * has no time to a limit, and a large number would read like a promise.
  */
static float hold_seconds(const thermal_t *th, const thermal_power_t *p,
                          const thermal_soa_t *soa, thermal_node_t node)
{
  const float gain = net_watt(th, p, node);
  const float capacity = th->cfg.node[node].capacity;
  const float togo = soa->limit_c[node] - th->t[node];

  if (!(gain > 0.0f) || !(capacity > 0.0f))
  {
    return -1.0f;                      /* not heading anywhere warmer */
  }
  if (togo <= 0.0f)
  {
    return 0.0f;                       /* already there */
  }
  return togo * capacity / gain;
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

    /* What is left in it, in joules. Never negative: a node past its
       ceiling has no budget rather than a debt, and the trip is what
       says so. */
    const float left = limit - th->t[i];

    out->soak_j[i] = (left > 0.0f) ? (th->cfg.node[i].capacity * left) : 0.0f;

    if (out->used[i] >= out->worst)
    {
      out->worst = out->used[i];
      out->worst_node = (uint8_t)i;
    }
  }

  out->throttling = ((float)out->worst / 255.0f) >= soa->throttle_at;
  out->tripped = (out->worst >= 255U);

  /* The clamp's factor: one below the throttle point, falling to zero at
     the ceiling. A stage that is derating is still driving, which is the
     whole difference between this and the trip.

     ON THE WORSE OF WHERE A NODE IS AND HOW LONG IT HAS. The temperature
     fraction above is where it is. The second fraction is TIME: how far
     into `lookahead_s` of remaining hold the node has come, which is the
     soak divided by the power spending it.

     WHY TIME AND NOT A PROJECTED TEMPERATURE. This projected each node
     forward `lookahead_s` at its present rate and derated on where that
     landed. It is the same idea and it fails on exactly the case this
     board is for: a hard burst into the SOA. Measured 2026-09-03 in
     `test_thermal_core.py` - 100 A in one leg puts 18.4 W into a driver
     node of 0.12 J/K, which is 0.67 s from ambient to a 125 C ceiling,
     and a 2000 ms horizon projects straight past it. The clamp went to
     0.00 from a cold board: the envelope forbade the transient instead
     of shaping it, and a drive that cannot burst is not a drive.

     Time does not have that failure. A node at ambient has its whole
     soak in front of it however much power is on it, so the burst runs;
     what closes the clamp is the hold falling into the window, and the
     window is a reaction time rather than a temperature. It is also
     scale-free across the nodes: a part with twice the power has half
     the hold and derates twice as early in degrees, which is right,
     because it has half the time to act.

     And the knob behaves. `lookahead_s` used to be fatal past the burst
     budget; now raising it only makes the throttle earlier and gentler,
     which is what a bench would expect from a number called lookahead.
     Zero still disables it and leaves the present-only envelope. */
  float spent = (float)out->worst / 255.0f;

  if (soa->lookahead_s > 0.0f)
  {
    for (uint8_t i = 0U; i < THERMAL_NODES; i++)
    {
      const float span = soa->limit_c[i] - th->ambient;

      if (!(span > 0.0f))
      {
        continue;
      }

      const float hold = hold_seconds(th, p, soa, (thermal_node_t)i);

      if (hold < 0.0f)
      {
        continue;             /* not heading anywhere warmer */
      }

      const float pressed = 1.0f - (hold / soa->lookahead_s);

      if (pressed > spent)
      {
        spent = (pressed > 1.0f) ? 1.0f : pressed;
      }
    }
  }

  const float band = 1.0f - soa->throttle_at;

  if ((spent <= soa->throttle_at) || !(band > 0.0f))
  {
    out->derate = 1.0f;
  }
  else
  {
    const float over = (spent - soa->throttle_at) / band;

    out->derate = (over >= 1.0f) ? 0.0f : (1.0f - over);
  }

  /* Time left, for the node that has least of it - the same `hold_seconds`
     the throttle acts on, so the number a host plans a burst with is the
     number the board backed off on. -1 while it is not heading there,
     rather than a large number that reads like a promise. */
  const float left = hold_seconds(th, p, soa, (thermal_node_t)out->worst_node);

  if (left > 0.0f)
  {
    const float millis = 1000.0f * left;

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

  /* The IAUCN10S7N021 VDMOS model in electronic_simulations: Ron 1.8 mOhm
     at 25 C. The tempco is the datasheet's (rev 1.2, fig 8, the VGS=10 V
     ID=88 A curve, datasheets/mosfet/): 1.8 mOhm at 25 C, ~2.55 at 100,
     ~4.1 at 175. A first-order chord 25->150 C gives 7.8e-3 per K - the
     curve is superlinear, so the chord under-reads above 150 where the
     trip should already have acted, and over-reads a little below 0 C. */
  loss->rds_on    = 1.8e-3f;
  loss->rds_alpha = 7.8e-3f;

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
                            const thermal_loss_t *loss,
                            const float *phase_c)
{
  if ((out == NULL) || (load == NULL) || (loss == NULL))
  {
    return;
  }
  memset(out, 0, sizeof(*out));

  float link_from_phases = 0.0f;

  /* Switching, per driven leg. Scaled by link voltage - the C_oss charge
     goes as Q(V)*V, nearer linear than square in this range, see
     python_examples/loss_calculation.py. An unmeasured link is the
     calibration's own voltage, not a scale of zero and not one off a rail
     reading mid-scale - board_thermal.c, invariant 9. */
  const float link = (load->link_volts > 0.0f) ? load->link_volts
                                               : loss->switch_volts;
  const float per_leg = (loss->switch_volts > 0.0f)
                        ? (loss->switching_watt / 3.0f)
                          * (link / loss->switch_volts)
                        : 0.0f;

  /* EACH LEG'S LOSS GOES TO THAT LEG. This used to scale one lumped node by
     how many legs were driven, so switching U alone raised all three by a
     third each. The camera says otherwise: 2026-08-29, U at 50 % with V and
     W idle heated U's half-bridge and nothing else. */
  for (int leg = 0; leg < 3; leg++)
  {
    /* Conduction: the current goes through the FET AND the shunt, both in
       this phase - and both halves of a leg carry the same squared current
       over a period, so the sum does not depend on duty. Dry it measures
       zero, and that is correct: nothing leaves the bridge, so the shunts
       decide, not the duty.

       The FET's resistance follows the node it heats: first order off the
       datasheet chord, floored so a garbage estimate cannot make the loss
       vanish. The shunt stays flat - WSHM28187 is metal strip, its tempco
       two orders below the FET's. */
    const float a = load->phase_amps[leg];
    float rds = loss->rds_on;
    if ((phase_c != NULL) && !isnan(phase_c[leg]))
    {
      float factor = 1.0f + loss->rds_alpha * (phase_c[leg] - 25.0f);
      if (factor < 0.5f)
      {
        factor = 0.5f;
      }
      rds *= factor;
    }
    /* SPLIT WHERE THE HEAT IS MADE. Both of these were booked on the
       phase node, so the model said the shunt cooked while the FET beside
       it in the same current path stayed cold - measured on the stand-in,
       fifteen cells of seventeen on the phase thermometer against three
       on the driver's. They are two parts and they heat separately: the
       FET's watts are the FET's, and it is the one whose resistance
       climbs with its own temperature. The nodes keep their names. */
    /* THE MEAN SQUARE WHERE THERE IS ONE. `a * a` is one instant squared,
       which is the loss only if that instant happened to be the rms - see
       `phase_sq` in the header for why a synchronous sampler makes that
       worse than a coin toss. The signed sample still carries the link
       estimate below, because a mean square has no sign. */
    const float sq = (load->phase_sq[leg] > 0.0f) ? load->phase_sq[leg]
                                                  : (a * a);

    out->watt[THERMAL_DRIVER(leg)] += sq * rds;
    out->watt[THERMAL_PHASE(leg)] = sq * loss->r_shunt;
    link_from_phases += load->duty[leg] * a;

    if (load->switching && (load->duty[leg] > 0.0f))
    {
      out->watt[THERMAL_DRIVER(leg)] += per_leg * loss->driver_share;
      /* The buck is one part feeding all three, so its share stays lumped. */
      out->watt[THERMAL_REGULATORS]  += per_leg * (1.0f - loss->driver_share);
    }
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
  const float rise  = th->t[THERMAL_NTC_NEIGHBOUR] - board;
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

/** Pull one node to its die, and return the board that implies: the node is
  * board + P*theta, so subtracting reaches the board without the NTC. */
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

  /* Each die corrects its node and implies a board; the board takes their
     mean. The NTC then explains only the drivers' rise. */
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

    /* Settled is about the BOARD. A die anchors it without guessing how
       much of the NTC is hot spot, which is the guess the flag warns of. */
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
        th->t[THERMAL_NTC_NEIGHBOUR] +=
            k * (at - th->t[THERMAL_NTC_NEIGHBOUR]);
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
    const float rise = th->t[THERMAL_NTC_NEIGHBOUR] - th->t[THERMAL_BOARD];
    const float bulk = thermal_board_from_ntc(&th->cfg, seen->ntc_c, rise);
    th->t[THERMAL_BOARD] += k * (bulk - th->t[THERMAL_BOARD]);
    th->settled = false;
  }

  th->steps++;
}
