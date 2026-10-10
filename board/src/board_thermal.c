/** board_thermal.c - The thermal observer and its envelope on this hardware. */
#include "board_limits.h"
#include "board.h"
#include "board_units.h"
#include "board_drive.h"
#include "board_hw.h"
#include "board_power.h"
#include "drive.h"
#include "thermal.h"
#include "thermal_ident.h"
#include "thermal_run.h"

#include <math.h>
#include <string.h>

/* The reply's arrays are sized by literals in board.h; the loops that fill
   them run to the enum in thermal.h. */
_Static_assert(BOARD_THERMAL_NODES == (int)THERMAL_NODES, "board.h's node count is thermal.h's");
_Static_assert(BOARD_THERMAL_EDGES == THERMAL_EDGES, "board.h's edge count is thermal.h's");
_Static_assert(BOARD_THERMAL_IDENT_SCALES == THERMAL_IDENT_RECORD,
               "board.h's scale count is thermal_ident.h's");

/** The thermal glue's state: the observer, its losses and power, the envelope and its
    budget, the three thermometers' sampling, the identification, the ceilings' margin. */
static struct
{
  /** The observer, its identification and its envelope: the core's, one code. */
  thermal_run_t run;

  /** The last DC link voltage that was a measurement, volts. */
  float link_volts;
  bool ready;
  uint32_t last_ms;
  /* The clamp's derate held off until this tick: thermal op 14. */
  bool wep;
  uint32_t wep_until;
  /* The host's airspeed across the board, m/s, held until this tick: op 16. */
  float airspeed;
  uint32_t airspeed_until;
  bool holding;                   /**< the thermal observer holds the AFE rail */
  uint32_t held_ms;               /**< when it took it */
  bool afe_was;                   /**< the rail at the last poll */
  uint32_t afe_up_ms;             /**< when the rail was last seen coming up */
  uint32_t sampled_ms;            /**< when the last sample finished */
  thermal_sense_t last_seen;
  uint32_t seen_ms;               /**< when s.last_seen was taken */
  bool seen;                      /**< whether anything has answered */
  uint32_t every_ms;
  uint32_t settle_ms;
  /** The observer's clock, ms since init: the wall's times `haste`. The
      sampling, the trip cap and `seen_ms` run on it. */
  uint32_t millis;
  uint32_t haste;
  uint32_t steps;                 /**< model integrations, for a rate */
  float speed_rpm;                /**< the rotor at the last step */
} s = {
  .link_volts = -1.0f, .last_seen = { NAN, NAN, NAN }, .every_ms = THERMAL_SAMPLE_EVERY_MS,
  .settle_ms = THERMAL_SAMPLE_SETTLE_MS, .haste = 1U
};

/** The floor the margin rises from: the record's, ppm of the span. */
static float margin_floor(void)
{
  const uint32_t ppm = Board_Cal()->soa_margin_floor_ppm;

  return (float)((ppm != 0U) ? ppm : BOARD_SOA_MARGIN_FLOOR_PPM) / PPM_PER_UNIT;
}

/** The envelope out of the calibration record into the run, untrimmed, and the ceilings
    trimmed by the margin now. */
static void soa_from_cal(void)
{
  const board_cal_t *cal = Board_Cal();
  thermal_soa_t *rec = &s.run.record;

  memset(rec, 0, sizeof(*rec));
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    rec->limit_c[i] = (float)cal->soa_limit_centi[i] / CENTI_PER_UNIT;
    /* Which of them the clamp can cool - `board.h` has why the
       housekeeping nodes are judged but not throttled on. */
    rec->undriven[i] = ((cal->soa_undriven_mask >> i) & 1UL) != 0UL;
  }
  /* The winding's ceiling: its own field (op 6, id 48); zero disables it. */
  rec->limit_c[THERMAL_WINDING] = (float)cal->winding_limit_centi / CENTI_PER_UNIT;
  rec->throttle_at = (float)cal->soa_throttle_ppm / PPM_PER_UNIT;
  rec->lookahead_s = (float)cal->soa_lookahead_ms / MILLI_PER_UNIT;
  s.run.margin_floor = margin_floor();
  thermal_run_envelope(&s.run, s.millis);
}

/** The bulk: the laminate's air path, its radiated share, and what the
    thermistor sees of it. */
static void lay_bulk(thermal_cfg_t *cfg, const board_cal_t *cal)
{
  if (cal->thermal_to_ambient_milli != 0U)
  {
    cfg->board_to_ambient = (float)cal->thermal_to_ambient_milli / MILLI_PER_UNIT;
  }
  if (cal->thermal_rad_share_ppm != 0U)
  {
    cfg->board_rad_share = (float)cal->thermal_rad_share_ppm / PPM_PER_UNIT;
  }
  if (cal->thermal_ntc_sees_ppm != 0U)
  {
    cfg->ntc_sees = (float)cal->thermal_ntc_sees_ppm / PPM_PER_UNIT;
  }
  if (cal->thermal_ntc_tau_ms != 0U)
  {
    cfg->ntc_tau_s = (float)cal->thermal_ntc_tau_ms / MILLI_PER_UNIT;
  }
  cfg->rad_board_stator = (float)cal->thermal_rad_board_stator_micro / MICRO_PER_UNIT;
}

/** The bulk laminate shared out by area, as `thermal_set_board` does, before
    any patch's own entry overrides its share. */
static void share_bulk(thermal_cfg_t *cfg, const board_cal_t *cal)
{
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    thermal_node_cfg_t *n = &cfg->node[i];

    const bool shared = n->area_share > 0.0f;

    if (shared && (cal->thermal_to_ambient_milli != 0U))
    {
      n->to_ambient = cfg->board_to_ambient / n->area_share;
    }
    if (shared && (cal->thermal_capacity_milli != 0U))
    {
      n->capacity = ((float)cal->thermal_capacity_milli / MILLI_PER_UNIT)
                    * n->area_share;
    }
  }
}

/** Every non-zero node entry over its own field. */
static void lay_nodes(thermal_cfg_t *cfg, const board_cal_t *cal)
{
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    const board_cal_node_t *rec = &cal->thermal_node[i];
    thermal_node_cfg_t *n = &cfg->node[i];

    if (rec->capacity_milli != 0U)
    {
      n->capacity = (float)rec->capacity_milli / MILLI_PER_UNIT;
    }
    if (rec->to_ambient_milli != 0U)
    {
      n->to_ambient = (float)rec->to_ambient_milli / MILLI_PER_UNIT;
    }
    if (rec->forced_milli != 0U)
    {
      n->forced = (float)rec->forced_milli / MILLI_PER_UNIT;
    }
    if (rec->rth_milli != 0U)
    {
      n->rth_die = (float)rec->rth_milli / MILLI_PER_UNIT;
    }
  }
}

/** Every edge the record names: OPEN cuts it, a value sets it, zero leaves
    the default. */
static void lay_edges(thermal_cfg_t *cfg, const board_cal_t *cal)
{
  for (uint8_t e = 0U; e < (uint8_t)THERMAL_EDGES; e++)
  {
    const uint32_t milli = cal->thermal_edge_milli[e];

    if (milli == BOARD_CAL_EDGE_OPEN)
    {
      cfg->r_edge[e] = 0.0f;
    }
    else if (milli != 0U)
    {
      cfg->r_edge[e] = (float)milli / MILLI_PER_UNIT;
    }
  }
}

/** The winding, from its own three fields (CAL_VERSION 12): a quarter of the
    K/W into the iron, the rest the iron's air path. */
static void lay_winding(thermal_cfg_t *cfg, const board_cal_t *cal)
{
  thermal_run_winding(cfg, (float)cal->winding_k_per_w_milli / MILLI_PER_UNIT,
                      (float)cal->winding_j_per_k_milli / MILLI_PER_UNIT);
}

/** The network: the core's defaults with every non-zero record entry laid
    over its own field, in the order the overrides stack, then the application. */
static void network_from_cal(thermal_cfg_t *cfg)
{
  const board_cal_t *cal = Board_Cal();

  thermal_defaults(cfg);
  lay_bulk(cfg, cal);
  share_bulk(cfg, cal);
  lay_nodes(cfg, cal);
  lay_edges(cfg, cal);
  lay_winding(cfg, cal);
  thermal_application(cfg, (thermal_app_t)cal->thermal_app);
}

/** The losses: the core's table with the record's phase resistance, the one
    loss constant the record carries. */
static void losses_from_cal(thermal_loss_t *loss)
{
  thermal_losses(loss);
  loss->r_phase = (float)Board_Cal()->motor_r_uohm / MICRO_PER_UNIT;
  loss->k_iron = (float)Board_Cal()->thermal_k_iron_milli / MILLI_PER_UNIT;
}

/** The network the observer runs: the record's base with the identified
    scales on it. */
static void network_refresh(void)
{
  network_from_cal(&s.run.base);
  thermal_run_refresh(&s.run);
}

void Board_ThermalInit(void)
{
  thermal_cfg_t base;
  thermal_loss_t loss;

  /* Start on the NTC where its reference is up, otherwise somewhere plausible: down, it
     reads mid-scale (invariant 9). */
  int32_t raw = 0, centi = 0;
  const bool have = Board_AfeOn() && Board_Ntc(&raw, &centi);
  const float start_c = have ? ((float)centi / CENTI_PER_UNIT) : 25.0f;

  network_from_cal(&base);
  losses_from_cal(&loss);
  s.millis = 0U;
  thermal_run_init(&s.run, &base, &loss, start_c);
  /* The envelope comes from the calibration record, not from this file. */
  soa_from_cal();
  s.last_ms = HAL_GetTick();
  s.sampled_ms = 0U;
  s.held_ms = s.last_ms;
  s.holding = false;
  s.afe_was = Board_AfeOn();
  s.afe_up_ms = s.last_ms;
  s.millis = 0U;
  s.steps = 0U;
  s.speed_rpm = 0.0f;
  s.ready = true;
}

/** Read every thermometer. */
static void sense_read(thermal_sense_t *out)
{
  int32_t raw = 0, centi = 0;

  /* The three need the AFE's reference: without it the thermistor reads mid-scale, 25.00 C,
     and the MCU's die 545 C (FINDINGS 2026-09-26). */
  if (!Board_AfeOn())
  {
    return;
  }
  out->ntc_c = Board_Ntc(&raw, &centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;
  out->mcu_c = Board_McuDie(&raw, &centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;

  /* The A1335 sits in the AFE corner, so its die anchors that node, not the
     board. */
  out->afe_c = Board_AngleDie(&centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;
}

/** A sample when one is due on the observer's clock, `clock`; the reference
    settles on the wall's, `now`. */
static void sense_sample(uint32_t now, uint32_t clock, thermal_sense_t *out)
{
  out->ntc_c = NAN;
  out->afe_c = NAN;
  out->mcu_c = NAN;

  /* Zero is off: unsigned, a zero period would borrow the rail every poll. */
  const bool due = (s.every_ms != 0U) && ((clock - s.sampled_ms) >= s.every_ms);

  if (!s.holding && !due)
  {
    return;
  }
  /* The rail up by another: read once it has settled, borrow nothing. */
  if (!s.holding && Board_AfeOn())
  {
    if ((now - s.afe_up_ms) >= s.settle_ms)
    {
      sense_read(out);
      s.sampled_ms = clock;
    }
    return;
  }
  if (!s.holding && !Board_PowerAcquire(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    /* Armed. Back off a whole interval rather than retrying at 10 Hz. */
    s.sampled_ms = clock;
    return;
  }
  if (!s.holding)
  {
    s.holding = true;
    s.held_ms = now;
    return;                         /* the reference has not come up yet */
  }

  if ((now - s.held_ms) < s.settle_ms)
  {
    return;
  }

  sense_read(out);

  (void)Board_PowerRelease(BOARD_RAIL_AFE, BOARD_USER_THERMAL);
  s.holding = false;
  s.sampled_ms = clock;
}

/** The rotor's mechanical speed, rpm, off the drive's observer: its
    electrical rad/s over the record's pole pairs. */
static float speed_now(void)
{
  const drive_t *d = Board_Drive();
  const uint32_t pairs = Board_Cal()->motor_pole_pairs;

  if ((d == NULL) || (pairs == 0U) || (d->mode == DRIVE_OFF))
  {
    return 0.0f;
  }
  const float mech = fabsf(d->obs.omega) / (float)pairs;   /* rad/s */

  return mech * 60.0f / (2.0f * 3.14159265f);
}

/** What heats the board this step: the duties, the synced triple's currents, the link
    while the AFE reads it, the record's dead time, the drive's speed. */
static void load_now(thermal_load_t *load)
{
  const uint32_t period = Board_PwmPeriod();

  load->afe_on = Board_AfeOn();
  /* The drivers' supply is FAULTOUT's (PE15): U9's +15V7 on the schematic's board; on the
     unmodified bench board it follows AFE_ON inversely. */
  load->switching = Board_PwmIsEnabled() && Board_Pe15();
  for (uint8_t i = 0U; i < 3U; i++)
  {
    load->duty[i] = (period > 0U)
                    ? ((float)Board_PwmGetDuty(i) / (float)period) : 0.0f;
  }

  if (Board_SyncArmed())
  {
    board_sync_sample_t sample;

    Board_SyncLatest(&sample);
    for (uint8_t i = 0U; i < 3U; i++)
    {
      /* Through Board_PhaseAmps, so the shunt and gain stay in the
         calibration record (invariant 7). */
      load->phase_amps[i] = Board_PhaseAmps(i, sample.phase[i]);
    }

    /* The mean square, which is what the conduction is made of. */
    (void)Board_SyncMeanSquare(load->phase_sq);
  }

  int32_t dc_raw = 0, millivolt = 0;
  if (load->afe_on && Board_DcBus(&dc_raw, &millivolt))
  {
    s.link_volts = (float)millivolt / MILLI_PER_UNIT;
  }
  /* Zero says "never measured", and the model falls back to the voltage its
     switching figure was calibrated at rather than inventing a scale. */
  load->link_volts = (s.link_volts > 0.0f) ? s.link_volts : 0.0f;
  load->t_dead_s = (float)Board_Cal()->deadtime_ns * 1.0e-9f;
  load->speed_rpm = speed_now();
}

/** A slice of the run on this poll's load and sample, the clamp and the stage acted on - the
    one place this file acts rather than reports. The STO chain's pump fed between the run's
    parts: a slice at -O0 is 140 000 instructions. */
static void run_slice(const thermal_load_t *load, const thermal_sense_t *seen, uint32_t slice)
{
  thermal_run_in_t in;
  thermal_run_out_t out;

  in.load = *load;
  /* The host's airspeed while it holds; the core folds it into the rotor's wash. */
  in.load.airspeed_m_s = ((int32_t)(s.airspeed_until - HAL_GetTick()) > 0) ? s.airspeed : 0.0f;
  in.seen = *seen;
  in.slice_ms = slice;
  in.clock_ms = s.millis;
  s.wep = s.wep && ((int32_t)(s.wep_until - HAL_GetTick()) > 0);
  in.wep = s.wep;
  in.switching = Board_PwmIsEnabled();
  thermal_run_slice(&s.run, &in, &out, Board_StoKeepalive);
  Board_DriveDerate(out.derate);
  if (out.drop)
  {
    Board_PwmDisable();
  }
}

void Board_ThermalPoll(void)
{
  if (!s.ready)
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  const uint32_t since = now - s.last_ms;      /* unsigned: the wrap is free */

  /* A step of the observer's clock, not the wall's: the envelope acts as often on a hasted
     world as on a bench, or the clamp lags a leg's constant (FINDINGS 2026-09-26). */
  if ((since * s.haste) < THERMAL_STEP_MS)
  {
    return;
  }
  s.last_ms = now;
  /* The observer's clock: the wall's, or a world's hasted (op 13). */
  const uint32_t span = since * s.haste;

  s.millis += span;

  thermal_load_t load;

  memset(&load, 0, sizeof(load));
  load.link_amps = -1.0f;               /* < 0: estimate it from the phases */

  load_now(&load);
  s.speed_rpm = load.speed_rpm;

  thermal_sense_t seen;
  const bool afe = Board_AfeOn();

  if (afe && !s.afe_was)
  {
    s.afe_up_ms = now;
  }
  s.afe_was = afe;
  sense_sample(now, s.millis, &seen);

  /* Keep whatever answered. */
  if (!isnan(seen.ntc_c) || !isnan(seen.afe_c) || !isnan(seen.mcu_c))
  {
    s.last_seen = seen;
    s.seen_ms = s.millis;
    s.seen = true;
  }

  /* In slices, the envelope on every one and the STO chain's pump fed between them. */
  const uint32_t most = THERMAL_CATCHUP_MS * s.haste;
  uint32_t left = (span > most) ? most : span;

  while (left > 0U)
  {
    const uint32_t slice = (left > THERMAL_STEP_MS) ? THERMAL_STEP_MS : left;

    left -= slice;
    run_slice(&load, &seen, slice);
    /* A sample is folded in once, on the first slice. */
    seen.ntc_c = seen.afe_c = seen.mcu_c = NAN;
    s.steps++;
    Board_StoKeepalive();
  }

  /* After every poll: the ceilings re-trimmed once the margin has moved a step. */
  thermal_run_follow(&s.run, s.millis);
}

bool Board_ThermalState(board_thermal_t *out)
{
  if ((out == NULL) || !s.ready)
  {
    return false;
  }

  out->ntc_measured = !isnan(s.last_seen.ntc_c);
  out->ntc_centidegc = out->ntc_measured
                       ? (int32_t)(s.last_seen.ntc_c * CENTI_PER_UNIT) : 0;
  out->afe_measured = !isnan(s.last_seen.afe_c);
  out->afe_centidegc = out->afe_measured
                       ? (int32_t)(s.last_seen.afe_c * CENTI_PER_UNIT) : 0;
  out->mcu_measured = !isnan(s.last_seen.mcu_c);
  out->mcu_centidegc = out->mcu_measured
                       ? (int32_t)(s.last_seen.mcu_c * CENTI_PER_UNIT) : 0;
  /* A flag, not `s.seen_ms != 0`: the clock reads 0 at init and at its wrap. */
  out->seen_ms_ago = s.seen ? (s.millis - s.seen_ms) : 0U;

  const thermal_t *th = &s.run.th;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->node_centidegc[i] = (int32_t)(th->t[i] * CENTI_PER_UNIT);
  }
  out->ambient_centidegc = (int32_t)(th->ambient * CENTI_PER_UNIT);
  out->expected_ntc_centidegc = (int32_t)(thermal_expected_ntc(th) * CENTI_PER_UNIT);
  out->seconds = s.millis / MS_PER_S;
  out->steps = s.steps;
  out->settled = th->settled;
  for (int leg = 0; leg < 3; leg++)
  {
    const float over = thermal_junction(th, &s.run.power, THERMAL_DRIVER(leg))
                       - th->t[THERMAL_DRIVER(leg)];

    out->junction_over_centi[leg] = (int32_t)(over * CENTI_PER_UNIT);
  }
  out->speed_rpm = (int32_t)s.speed_rpm;
  return true;
}

bool Board_ThermalBudget(board_budget_t *out)
{
  if ((out == NULL) || !s.ready)
  {
    return false;
  }

  const thermal_budget_t *budget = &s.run.budget;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->used[i] = budget->used[i];
  }
  out->worst = budget->worst;
  out->worst_node = budget->worst_node;
  out->millis_to_limit = budget->millis_to_limit;
  out->throttling = budget->throttling;
  out->tripped = budget->tripped;
  /* What is applied, the recovery slew in it: the raw factor flickers where the clamp
     does not. */
  out->derate = Board_DriveDerating();
  for (uint8_t i = 0U; i < BOARD_THERMAL_NODES; i++)
  {
    out->soak_j[i] = budget->soak_j[i];
  }
  /* The effective duty, off the compares: what the clamp and the derate left. */
  {
    const uint32_t period = Board_PwmPeriod();

    for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
    {
      out->duty[i] = (period > 0U)
        ? ((float)Board_PwmGetDuty(i) / (float)period) : 0.0f;
    }
  }
  out->trips = s.run.trips;
  /* MINOR 12's winding fields, from the winding node. */
  out->winding_c = s.run.th.t[THERMAL_WINDING];
  out->winding_used = budget->used[THERMAL_WINDING];
  out->winding_derate = s.run.winding_derate;
  return true;
}

bool Board_ThermalSetWinding(float limit_c, float k_per_w, float j_per_k)
{
  if (!s.ready)
  {
    return false;
  }
  if (!Board_CalSetWinding((int32_t)(limit_c * CENTI_PER_UNIT),
                           (uint32_t)(k_per_w * MILLI_PER_UNIT),
                           (uint32_t)(j_per_k * MILLI_PER_UNIT)))
  {
    return false;
  }
  /* The estimate carries on: a ceiling or a constant moves the judge, not the winding. */
  network_refresh();
  soa_from_cal();
  return true;
}

bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at)
{
  if (!s.ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  /* Through the record, so a save persists it and one place holds the
     envelope. */
  const board_cal_t *cal = Board_Cal();
  const int32_t centi = (int32_t)(limit_c * CENTI_PER_UNIT);
  const bool kept = (node == (uint8_t)THERMAL_WINDING)
                    ? Board_CalSetWinding(centi, cal->winding_k_per_w_milli,
                                          cal->winding_j_per_k_milli)
                    : Board_CalSetLimit(node, centi);

  if (!kept)
  {
    return false;
  }
  if ((throttle_at > 0.0f) && (throttle_at < 1.0f))
  {
    (void)Board_CalSetThrottle((uint32_t)(throttle_at * PPM_PER_UNIT));
  }
  soa_from_cal();
  return true;
}

bool Board_ThermalSetNode(uint8_t node, float k_per_w, float capacity)
{
  if (!s.ready
      || !thermal_set_node(&s.run.th, (thermal_node_t)node, k_per_w, capacity))
  {
    return false;
  }
  /* And into the record: what runs is what a save keeps. */
  const int edge = thermal_sink_edge((thermal_node_t)node);

  if (edge >= 0)
  {
    (void)Board_CalSetThermalEdge((uint8_t)edge,
                                  (uint32_t)(k_per_w * MILLI_PER_UNIT));
    (void)Board_CalSetThermalNode(node, (uint32_t)(capacity * MILLI_PER_UNIT),
                                  Board_Cal()->thermal_node[node]
                                      .to_ambient_milli);
  }
  else
  {
    (void)Board_CalSetThermalNode(node, (uint32_t)(capacity * MILLI_PER_UNIT),
                                  (uint32_t)(k_per_w * MILLI_PER_UNIT));
  }
  network_refresh();                   /* the record is the base; the scales stay */
  return true;
}

bool Board_ThermalSetEdge(uint8_t edge, float k_per_w)
{
  if (!s.ready || (edge >= (uint8_t)THERMAL_EDGES))
  {
    return false;
  }
  const float r = (k_per_w < 0.0f) ? 0.0f : k_per_w;

  if (!thermal_set_edge(&s.run.th, (int)edge, r))
  {
    return false;
  }
  const bool ok = Board_CalSetThermalEdge(edge, (k_per_w < 0.0f)
                                         ? BOARD_CAL_EDGE_OPEN
                                         : (uint32_t)(k_per_w * MILLI_PER_UNIT));

  network_refresh();
  return ok;
}

bool Board_ThermalEdge(uint8_t edge, uint8_t *a, uint8_t *b, float *k_per_w)
{
  if (!s.ready || (edge >= (uint8_t)THERMAL_EDGES) || (a == NULL)
      || (b == NULL) || (k_per_w == NULL))
  {
    return false;
  }
  *a = thermal_edge((int)edge).a;
  *b = thermal_edge((int)edge).b;
  *k_per_w = s.run.th.cfg.r_edge[edge];
  return true;
}

bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient,
                          float *area_share, float *rth_die, float *forced)
{
  if (!s.ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  const thermal_node_cfg_t *n = &s.run.th.cfg.node[node];

  *capacity = n->capacity;
  *to_ambient = n->to_ambient;
  *area_share = n->area_share;
  *rth_die = n->rth_die;
  *forced = n->forced;
  return true;
}

bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms)
{
  if (!s.ready)
  {
    return false;
  }
  s.every_ms = every_ms;
  s.settle_ms = settle_ms;
  return true;
}

void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms)
{
  *every_ms = s.every_ms;
  *settle_ms = s.settle_ms;
}

bool Board_ThermalWep(uint32_t ms)
{
  if (!s.ready || (ms > THERMAL_WEP_MAX_MS))
  {
    return false;
  }
  s.wep = (ms > 0U);
  s.wep_until = HAL_GetTick() + ms;
  if (s.wep)
  {
    Board_DriveDerate(1.0f);
  }
  return true;
}

bool Board_ThermalAirspeed(uint32_t mm_s)
{
  if (!s.ready || (mm_s > THERMAL_AIRSPEED_MAX_MM_S))
  {
    return false;
  }
  s.airspeed = (float)mm_s / MILLI_PER_UNIT;
  s.airspeed_until = HAL_GetTick() + THERMAL_AIRSPEED_HOLD_MS;
  return true;
}

bool Board_ThermalSetClock(uint32_t haste)
{
  if (!s.ready || (haste == 0U) || (haste > THERMAL_HASTE_MAX))
  {
    return false;
  }
  s.haste = haste;
  return true;
}

bool Board_ThermalSetBoard(float to_ambient, float capacity)
{
  if (!s.ready || !thermal_set_board(&s.run.th, to_ambient, capacity))
  {
    return false;
  }
  const bool ok = Board_CalSetThermalBulk((uint32_t)(to_ambient * MILLI_PER_UNIT),
                                         (uint32_t)(capacity * MILLI_PER_UNIT));

  network_refresh();
  return ok;
}

bool Board_ThermalIdent(board_thermal_ident_t *out)
{
  if ((out == NULL) || !s.ready)
  {
    return false;
  }
  const thermal_ident_t *ident = &s.run.ident;

  out->state = (uint8_t)ident->state;
  out->online_mask = 0U;
  for (int k = 0; k < THERMAL_IDENT_RECORD; k++)
  {
    if (thermal_ident_online((thermal_ident_param_t)k))
    {
      out->online_mask |= (uint8_t)(1U << k);
    }
    out->scale[k] = ident->scale[k];
    out->sigma[k] = thermal_ident_sigma(ident, (thermal_ident_param_t)k);
  }
  out->innovation_k = ident->innovation_k;
  out->margin = s.run.margin;
  out->updates = ident->updates;
  out->ambient_c = thermal_ident_ambient(ident);
  out->ambient_sigma_k = thermal_ident_sigma(ident, THERMAL_IDENT_AMBIENT);
  out->margin_floor = margin_floor();
  out->trip_cap = thermal_run_trip_cap(&s.run, s.millis);
  out->application = (uint8_t)Board_Cal()->thermal_app;
  return true;
}

bool Board_ThermalIdentReset(void)
{
  if (!s.ready)
  {
    return false;
  }
  network_from_cal(&s.run.base);
  thermal_run_identify(&s.run);
  soa_from_cal();
  return true;
}

bool Board_ThermalSetApplication(uint8_t app)
{
  /* What was identified was the old network's. */
  return s.ready && (app < (uint8_t)THERMAL_APPS) && Board_CalSetApplication(app)
         && Board_ThermalIdentReset();
}

bool Board_ThermalSetMarginFloor(float floor)
{
  if (!s.ready || !(floor > 0.0f) || (floor > 1.0f))
  {
    return false;
  }
  /* Through the record, so a save persists it and one place holds the
     envelope - the floor is a limit beside the ceilings. */
  if (!Board_CalSetMarginFloor((uint32_t)(floor * PPM_PER_UNIT + 0.5f)))
  {
    return false;
  }
  soa_from_cal();
  return true;
}
