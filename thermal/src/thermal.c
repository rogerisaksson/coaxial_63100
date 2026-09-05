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
  cfg->board_cal_rise_k = 10.0f;   /* the passive state: 1.2 W, +10 K */
  cfg->board_rad_share  = 0.35f;   /* docs/papers: 30-40 % at passive */

  /* Three times the lumped value each, so the three in parallel are the
     15.2 K/W the camera measured. The split moved where the heat is drawn,
     not how much of it there is. */
  for (int leg = 0; leg < 3; leg++)
  {
    /* 28 K/W A LEG, and it is a hip shot between two readings that
       disagree - said so here rather than dressed as a measurement.

          the camera's bridge zone, 15.2 K/W lumped, x3 for three
          parallel legs                                        45.6 K/W
          the datasheet's 25.9 K/W junction-to-air on a 2s2p
          coupon, less Rth JC 0.69 and the board's own 8.33     16.9 K/W
          the geometric mean of the two                         27.7 K/W

       NEITHER DOMINATES. The camera measured THIS board but read a mixed
       copper and soldermask surface through an emissivity nobody corrected -
       the same suspicion the NTC campaign raised. The datasheet is
       characterised on a defined board, but not on this one, and with a single
       device dissipating where ours carries six FETs and three drivers. The
       log-midpoint is what "between two estimates" means when both are ratios.

       WHAT IT COSTS AND WHAT IT KEEPS. Three legs equally loaded, 20 C room,
       against the record's 125 C nodes and 105 C board:

          R_leg    continuous rating   binding node
          45.6     15 to 18 A rms      the shunt
          28       20 to 22 A rms      the shunt
          16.9     about 25 A rms      the shunt

       THE SHUNT BINDS IN EVERY CASE, not the FET - it is 3.5 mOhm against the
       FET's 1.8 and has no case path to hide behind. And the FET's own ceiling
       keeps its margin: at 100 A each carries about 9 W, so Rth JC 0.69 K/W
       puts the junction 6.2 K over its node, and a 125 C node is 131 C at the
       junction against the sheet's 175 C limit. Cutting the spreading by 1.6
       does not spend that 44 K.

       WHAT WOULD REPLACE IT: a camera run under real load, with the board
       reference taken on a surface whose emissivity was corrected. */
    cfg->node[THERMAL_DRIVER(leg)].to_board = 28.0f;
    cfg->node[THERMAL_PHASE(leg)].to_board  = 28.0f;
  }
  cfg->node[THERMAL_MCU].to_board        = 22.5f;
  cfg->node[THERMAL_REGULATORS].to_board = 15.0f;
  cfg->node[THERMAL_AFE].to_board        = 41.5f;
  cfg->node[THERMAL_BOARD].to_board      = 0.0f;    /* it is the board */

  /* Heat capacity. The board dominates: tau 6.8 min against 8.33 K/W is
     about 49 J/K, and that one is MEASURED - fitted to a transient, so it
     is already an effective capacity.

     THE PARTS' OWN ARE NOT MEASURED. This comment used to end "and only
     affect the settling", which was true while the model was a
     steady-state fit and is false now: the envelope divides by exactly
     these numbers. `soak_j` is capacity x (limit - t), `hold_seconds` is
     that over the net watts, and the throttle's reaction window is a
     multiple of it. Every burst figure on record rests on a number
     nobody took.

     Silva 2022 (Appl. Sci. 12, 12555) puts a bound on how wrong: a
     lumped element's effective transient capacity is gamma C, gamma =
     1/3 less a negative term per contact with a better conductor,
     because heat crosses a distributed body one way. If 0.35 was a guess
     at the physical capacity then the transient one is up to three times
     smaller and every burst is three times shorter; if it was already a
     guess at the effective one it stands. Nothing on record says which.

     A power step and the NTC's slope would settle it - with the coupling
     at one the thermistor reads the leg lump, so dT/dt after a step is
     P / capacity outright. It is the only one of the soft numbers a
     transient can reach rather than an equilibrium. */
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

  /* HOW FAR THE THERMISTOR'S ELEMENT SITS TOWARD THE LEG, 0 to 1. See
     the header: it is a weighted average of the leg node and the board,
     so it cannot leave the interval between them at any value of this.

0.30, AND IT IS GEOMETRY NOW. `electronics/Coaxial 63100
     Pick-Place.csv` places NTC1 at (99.62, 79.83) mm and every power part
     beside it, so the fraction stops being a midpoint of a hand-waved range:

        U1V, the V gate driver     8.2 mm    f = 0.50
        Q2V, a V half-bridge FET  15.1 mm    f = 0.33
        Q1V, the other            17.7 mm    f = 0.28
        the next-nearest driver   28.0 mm

     Two-dimensional radial spreading in a plate gives `f = ln(R/r)/ln(R/a)`
     with R half the board's short side, 46 mm off the placements, and `a`
     the source's own radius - 1.5 mm for these packages. The old 0.5 was
     right for the DRIVER IC alone; it is wrong for the node, because the
     model lumps the driver's switching loss and both FETs' conduction onto
     one lump and the thermistor is 8 mm from one of them and 15 to 18 mm
     from the other two. At 100 A the FETs make 18.4 W of that node's 18.6,
     so the fraction is theirs: power-weighted, 0.304.

     AND IT CONFIRMS `THERMAL_NTC_NEIGHBOUR`. U1V is the nearest power part
     by a factor of 3.4 over the next driver, so anchoring the V leg is
     right - that was an assumption until the placements arrived. */
  cfg->ntc_sees_drivers = 0.30f;
  /* AN ELEMENT BETWEEN TWO NODES LAGS BETWEEN THEIR CONSTANTS, and the
     geometric mean is what "between" means for time constants - the
     log-midpoint, not the arithmetic one, because a lag is a ratio and
     not a difference. The leg node is 5.3 s and the board 408 s, so this
     is 47 s.

     IT WAS THE LEG'S OWN, 5.32 s, which made the modelled thermistor
     exactly as quick as the thing it watches. That is the one speed it
     cannot have: the SOA acts on silicon in a fifth of a second to two
     thirds, and a sensor soldered into laminate has to be far slower
     than that or it is not a sensor in laminate, it is a second copy of
     the FET. At 47 s a 100 A burst moves the reading about a kelvin and
     a half in its first second while the leg node moves a hundred and
     forty.

     What lags is the LAMINATE around the part, and the model has no node
     for that local patch - only the leg and the bulk board. This is the
     pair it sits between; a power step and the NTC's own slope would
     replace it with a measurement. */
  cfg->ntc_tau_s = sqrtf((0.35f / 3.0f * 28.0f)
                         * (cfg->node[THERMAL_BOARD].capacity
                            * cfg->board_to_ambient));
  /* RECORDED, NOT APPLIED. The passive state had the thermistor 6.0 K
     over the camera's board with no driver warming anything, and adding
     that to a node made the node hotter than its own source. It is an
     instrument disagreement - kept so a bench can see how big it is, and
     no longer part of any temperature. */
  cfg->ntc_offset       = 6.00f;
}

/** Net watts into one node right now: what it makes less what it sheds.
  *
  * The same two terms `thermal_step` integrates, so the budget's dead
  * reckoning and the model cannot drift apart.
  */
float thermal_board_to_ambient_at(const thermal_cfg_t *cfg, float rise_k)
{
  if (cfg == NULL)
  {
    return 0.0f;
  }

  const float cal = cfg->board_cal_rise_k;

  if (!(cal > 0.0f) || !(rise_k > cal))
  {
    return cfg->board_to_ambient;
  }

  /* CONVECTION, as the fourth root of the rise. `Nu = C Ra^n` with Ra
     linear in the rise, so `h` goes as `dT^n` and everything else in it -
     the fluid properties, the characteristic length, the area - is
     already inside the calibration value.

     A QUARTER, AND IT IS THE REGIME RATHER THAN A CHOICE. A horizontal
     plate is laminar while Ra < 1e7 and a vertical one while Ra < 1e9,
     and both give n = 1/4 there; only past those does it become a third.
     This board runs Ra = 1.3e4 to 6.4e4 on A/P and 8e5 to 4e6 on its own
     side, over rises of 10 to 85 K - three to four decades short of
     leaving laminar, whichever way it is mounted. The orientation does
     not have to be settled to pick the exponent. */
  const float conv = powf(rise_k / cal, 0.25f);

  /* RADIATION, exactly. `h_rad = eps sigma (T^2 + T0^2)(T + T0)`, and
     the emissivity and area are again inside the calibration value, so
     only the ratio of the bracket is needed. Kelvin, because a fourth
     power is not a difference. */
  const float t0 = 293.15f;               /* the 20 C room the fit used */
  const float now = t0 + rise_k;
  const float was = t0 + cal;
  const float rad = ((now * now + t0 * t0) * (now + t0))
                    / ((was * was + t0 * t0) * (was + t0));

  float share = cfg->board_rad_share;

  if (share < 0.0f)
  {
    share = 0.0f;
  }
  if (share > 1.0f)
  {
    share = 1.0f;
  }
  /* The two carry in parallel, so their CONDUCTANCES add. */
  const float better = (1.0f - share) * conv + share * rad;

  return (better > 0.0f) ? (cfg->board_to_ambient / better)
                         : cfg->board_to_ambient;
}


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
    const float rise = th->t[THERMAL_BOARD] - th->ambient;
    const float away = thermal_board_to_ambient_at(&th->cfg, rise);
    const float lost = (away > 0.0f) ? (rise / away) : 0.0f;
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


/** The clamp's factor for a spend: one below the throttle point,
  * falling to zero at the ceiling, linear between. ONE DEFINITION for
  * the board's nodes and the winding, so the two envelopes back off on
  * the same ramp and a bench tuning the throttle point tunes both. */
static float derate_of(float spent, const thermal_soa_t *soa)
{
  const float band = 1.0f - soa->throttle_at;

  if ((spent <= soa->throttle_at) || !(band > 0.0f))
  {
    return 1.0f;
  }
  const float over = (spent - soa->throttle_at) / band;

  return (over >= 1.0f) ? 0.0f : (1.0f - over);
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

    if (out->used[i] >= 255U)
    {
      out->tripped = true;   /* any node at its ceiling, driven or not */
    }

    /* What is left in it, in joules. Never negative: a node past its
       ceiling has no budget rather than a debt, and the trip is what
       says so. */
    const float left = limit - th->t[i];

    out->soak_j[i] = (left > 0.0f) ? (th->cfg.node[i].capacity * left) : 0.0f;

    if (soa->undriven[i])
    {
      continue;              /* nothing the clamp does moves this one */
    }
    if (out->used[i] >= out->worst)
    {
      out->worst = out->used[i];
      out->worst_node = (uint8_t)i;
    }
  }

  out->throttling = ((float)out->worst / 255.0f) >= soa->throttle_at;

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

      if (!(span > 0.0f) || soa->undriven[i])
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

  out->derate = derate_of(spent, soa);

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


void thermal_winding_init(thermal_winding_t *w,
                          const thermal_winding_cfg_t *cfg, float celsius)
{
  if ((w == NULL) || (cfg == NULL))
  {
    return;
  }
  w->cfg = *cfg;
  w->c = celsius;
}


float thermal_winding_watt(const thermal_winding_cfg_t *cfg,
                           const thermal_load_t *load)
{
  if ((cfg == NULL) || (load == NULL))
  {
    return 0.0f;
  }
  float sq = 0.0f;

  for (int leg = 0; leg < 3; leg++)
  {
    const float a = load->phase_amps[leg];

    sq += (load->phase_sq[leg] > 0.0f) ? load->phase_sq[leg] : (a * a);
  }
  return sq * cfg->r_phase;
}


void thermal_winding_step(thermal_winding_t *w, float watt, float ambient,
                          float dt_s)
{
  if ((w == NULL) || !(dt_s > 0.0f) || !(w->cfg.capacity > 0.0f))
  {
    return;
  }
  const float tau = w->cfg.k_per_w * w->cfg.capacity;

  if ((tau > 0.0f) && (dt_s > tau))
  {
    dt_s = tau;
  }
  const float shed = (w->cfg.k_per_w > 0.0f)
                     ? (w->c - ambient) / w->cfg.k_per_w : 0.0f;

  w->c += (watt - shed) * dt_s / w->cfg.capacity;
}


void thermal_winding_budget(const thermal_winding_t *w, float watt,
                            float ambient, const thermal_soa_t *soa,
                            thermal_winding_budget_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->derate = 1.0f;
  if ((w == NULL) || (soa == NULL))
  {
    return;
  }
  const float span = w->cfg.limit_c - ambient;

  if (!(span > 0.0f))
  {
    return;                    /* no ceiling: the winding says nothing */
  }
  float used = (w->c - ambient) / span;

  if (used < 0.0f)
  {
    used = 0.0f;
  }
  if (used > 1.0f)
  {
    used = 1.0f;
  }
  out->used = used;
  out->tripped = used >= 1.0f;

  /* THE SAME TWO FRACTIONS AS A BOARD NODE: where it is, and how far into
     the reaction window its hold has come - the soak over the net power
     spending it. The bigger wins, and the ramp is `derate_of`'s. */
  float spent = used;

  if ((soa->lookahead_s > 0.0f) && (w->cfg.capacity > 0.0f))
  {
    const float shed = (w->cfg.k_per_w > 0.0f)
                       ? (w->c - ambient) / w->cfg.k_per_w : 0.0f;
    const float gain = watt - shed;
    const float togo = w->cfg.limit_c - w->c;

    if (gain > 0.0f)
    {
      const float hold = (togo > 0.0f) ? (togo * w->cfg.capacity / gain)
                                       : 0.0f;
      const float pressed = 1.0f - (hold / soa->lookahead_s);

      if (pressed > spent)
      {
        spent = (pressed > 1.0f) ? 1.0f : pressed;
      }
    }
  }
  out->throttling = spent >= soa->throttle_at;
  out->derate = derate_of(spent, soa);
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
  /* The lagged reading starts where everything else does, plus whatever
     the channel offset is: a first reading, not a ramp from nowhere. */
  th->ntc = celsius;
}

/** Where the thermistor's element is HEADING: the weighted average of
  * the two nodes it is tied to.
  *
  * BETWEEN THEM, ALWAYS. `f` is clamped to [0, 1] here rather than
  * trusted, because a record is a thing a bench writes and an element
  * outside its own interval is the defect this replaced.
  *
  * NO ADDITIVE OFFSET. The 6.0 K the passive state showed is a
  * disagreement between a thermistor and a camera, not a temperature,
  * and adding it to a node made the node hotter than its source. It is
  * reported as a residual now.
  */
static float ntc_target(const thermal_t *th)
{
  const float board = th->t[THERMAL_BOARD];
  const float leg = th->t[THERMAL_NTC_NEIGHBOUR];
  float f = th->cfg.ntc_sees_drivers;

  if (f < 0.0f)
  {
    f = 0.0f;
  }
  if (f > 1.0f)
  {
    f = 1.0f;
  }
  return board + f * (leg - board);
}


float thermal_expected_ntc(const thermal_t *th)
{
  if (th == NULL)
  {
    return NAN;
  }
  /* THE LAGGED STATE, not the target. With no lag configured, or before
     the first step has run, they are the same number. */
  return (th->cfg.ntc_tau_s > 0.0f) ? th->ntc : ntc_target(th);
}

float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float driver_rise_k)
{
  if (cfg == NULL)
  {
    return ntc_c;
  }
  /* The element inverted: `ntc = board + f (leg - board)`, so the board
     is the reading less the share of the rise the thermistor sees. NO
     OFFSET SUBTRACTED - the 6.0 K the campaign found is a disagreement
     between a thermistor and a CAMERA, and it is the camera that reads a
     mixed copper and soldermask surface through an emissivity nobody
     corrected. The board's own sensor is the thermistor. */
  return ntc_c - cfg->ntc_sees_drivers * driver_rise_k;
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
    const float rise = board - th->ambient;
    const float lost = rise / thermal_board_to_ambient_at(&th->cfg, rise);
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
      const float over = seen->ntc_c - th->t[THERMAL_BOARD];
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
    /* AT THE PRESENT RISE. The path off the board is a function of it, so
       inverting exactly would need an iteration; the estimator is already
       a slow filter on `k`, and evaluating the resistance where the board
       is now is the linearisation that filter can carry. */
    const float away = thermal_board_to_ambient_at(
        &th->cfg, th->t[THERMAL_BOARD] - th->ambient);

    th->ambient += k * ((th->t[THERMAL_BOARD] - lost * away) - th->ambient);
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

  /* THE THERMISTOR FOLLOWS, it does not jump. First order toward the
     algebra at `ntc_tau_s`, clamped like every other step here so a dt
     bigger than the constant lands ON the target rather than past it. */
  if (th->cfg.ntc_tau_s > 0.0f)
  {
    float share = dt_s / th->cfg.ntc_tau_s;

    if (share > 1.0f)
    {
      share = 1.0f;
    }
    th->ntc += (ntc_target(th) - th->ntc) * share;
  }
  else
  {
    th->ntc = ntc_target(th);
  }

  /* AND NEVER PAST EITHER OF THEM. The leg sheds only through the copper
     this element sits on - `shed` above is its one path - so the leg
     cannot fall below the copper it drains into, and a passive link in a
     chain fed from one end cannot read above that end, whatever its own
     lag (the series network of docs/papers, 2.3). The lagged state could:
     25 A for two minutes then off, and it read 6 K over a leg that had
     cooled past it, 29 K at 60 A - measured in test_thermal_core, and
     seen on the bench as an NTC warmer than the switches that heat it.
     The lag is the patch's; the bound is the chain's. */
  {
    const float bulk = th->t[THERMAL_BOARD];
    const float leg_now = th->t[THERMAL_NTC_NEIGHBOUR];
    const float low = (bulk < leg_now) ? bulk : leg_now;
    const float high = (bulk < leg_now) ? leg_now : bulk;

    if (th->ntc < low)
    {
      th->ntc = low;
    }
    if (th->ntc > high)
    {
      th->ntc = high;
    }
  }

  th->steps++;
}
