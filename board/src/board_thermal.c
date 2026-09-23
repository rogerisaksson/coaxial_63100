/** board_thermal.c - Runs the lumped-network thermal observer on this
    hardware. */
#include "board_limits.h"
#include "board.h"
#include "board_units.h"
#include "board_drive.h"
#include "board_hw.h"
#include "board_power.h"
#include "drive.h"
#include "thermal.h"
#include "thermal_ident.h"

#include <math.h>
#include <string.h>

/* The reply's arrays are sized by literals in board.h; the loops that fill
   them run to the enum in thermal.h. */
_Static_assert(BOARD_THERMAL_NODES == (int)THERMAL_NODES,
               "board.h's node count and thermal.h's enum disagree - the "
               "reply array would be written past its end");
_Static_assert(BOARD_THERMAL_EDGES == THERMAL_EDGES,
               "board.h's edge count and thermal.h's table disagree");
_Static_assert(BOARD_THERMAL_IDENT_SCALES == THERMAL_IDENT_RECORD,
               "board.h's scale count and thermal_ident.h's record disagree");

/** Of the winding's K/W, the share that is the edge into the iron; the rest
    is the iron's own air path. */
#define WINDING_INTO_IRON            0.25f

/** The thermal glue's state: the observer with its losses and power, the
    envelope and its budget, the sampling of the three thermometers, the
    identification beside the observer, and the margin the ceilings are
    trimmed by. */
static struct
{
  thermal_t th;
  thermal_loss_t loss;
  thermal_power_t power;

  /** The last DC link voltage that was a MEASUREMENT, volts. */
  float link_volts;
  thermal_soa_t soa;
  thermal_budget_t budget;
  /** The winding's own factor, beside the whole's: which envelope holds the
      stage back. */
  float winding_derate;
  uint32_t trips;
  bool ready;
  uint32_t last_ms;
  bool holding;                   /**< the thermal observer holds the AFE rail  */
  uint32_t held_ms;               /**< when it took it                  */
  uint32_t sampled_ms;            /**< when the last sample finished    */
  thermal_sense_t last_seen;
  uint32_t seen_ms;               /**< when s.last_seen was taken       */
  bool seen;                      /**< whether anything answered yet    */
  uint32_t every_ms;
  uint32_t settle_ms;
  uint32_t millis;
  uint32_t steps;                 /**< model integrations, for a rate  */
  float speed_rpm;                /**< the rotor at the last step      */

  /* THE IDENTIFICATION BESIDE THE OBSERVER. */
  thermal_ident_t ident;
  thermal_cfg_t base;
  /** The margin the ceilings are trimmed by now - what the board acts on and
      what op 10 reports: the identification's, or the trip cap while one is
      in force, whichever is less. */
  float margin;
  /** The trip cap and when it was set - THERMAL_TRIP_MARGIN at a trip,
      recovering at THERMAL_TRIP_RECOVER_PER_S; one and nothing to recover
      when no trip is in force. */
  float trip_cap;
  uint32_t trip_ms;
} s = {
  .link_volts = -1.0f, .winding_derate = 1.0f,
  .last_seen = { NAN, NAN, NAN }, .every_ms = THERMAL_SAMPLE_EVERY_MS,
  .settle_ms = THERMAL_SAMPLE_SETTLE_MS, .margin = 1.0f,
  .trip_cap = 1.0f
};

/* THERMAL_IDENT_NOISE_K, THERMAL_MARGIN_REF_C and THERMAL_MARGIN_STEP - the
   identification's noise floor, the margin's reference and how far it must
   move to re-trim - are in board_limits.h with the rest of the fixed
   numbers. */

/** The floor the margin rises from: the record's, ppm of the span. */
static float margin_floor(void)
{
  const uint32_t ppm = Board_Cal()->soa_margin_floor_ppm;

  return (float)((ppm != 0U) ? ppm : BOARD_SOA_MARGIN_FLOOR_PPM) / PPM_PER_UNIT;
}

/** The trip cap as it stands now: what it was set to plus what the minutes
    since have given back, never above one. */
static float trip_cap_now(void)
{
  if (s.trip_cap >= 1.0f)
  {
    return 1.0f;
  }
  const float back = (float)(HAL_GetTick() - s.trip_ms) / MILLI_PER_UNIT
                     * THERMAL_TRIP_RECOVER_PER_S;
  const float cap = s.trip_cap + back;

  return (cap < 1.0f) ? cap : 1.0f;
}

/** The margin now: the identification's for its doubt, or the trip cap,
    whichever keeps more in hand. */
static float margin_now(void)
{
  const float earned = thermal_ident_margin(&s.ident, margin_floor());
  const float cap = trip_cap_now();

  return (cap < earned) ? cap : earned;
}

/** Copy the envelope out of the calibration record into the thermal observer. */
static void soa_from_cal(void)
{
  const board_cal_t *cal = Board_Cal();

  memset(&s.soa, 0, sizeof(s.soa));
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    s.soa.limit_c[i] = (float)cal->soa_limit_centi[i] / CENTI_PER_UNIT;
    /* Which of them the clamp can actually cool - `board.h` has why the
       housekeeping nodes are judged but not throttled on. */
    s.soa.undriven[i] = ((cal->soa_undriven_mask >> i) & 1UL) != 0UL;
  }
  /* The winding's ceiling is the record's own field, kept since 12 so op 6
     and id 48 keep their meaning; zero disables it as before. */
  s.soa.limit_c[THERMAL_WINDING] = (float)cal->winding_limit_centi / CENTI_PER_UNIT;
  s.soa.throttle_at = (float)cal->soa_throttle_ppm / PPM_PER_UNIT;
  s.soa.lookahead_s = (float)cal->soa_lookahead_ms / MILLI_PER_UNIT;

  /* THE POLICY. */
  const float margin = margin_now();

  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    /* THE TRIP KEEPS THE RECORD'S CEILING; the trim below is the throttle's. */
    s.soa.trip_c[i] = s.soa.limit_c[i];
    if (s.soa.limit_c[i] > THERMAL_MARGIN_REF_C)
    {
      s.soa.limit_c[i] = THERMAL_MARGIN_REF_C
                         + margin * (s.soa.limit_c[i] - THERMAL_MARGIN_REF_C);
    }
  }
  s.margin = margin;
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
  const float k = (float)cal->winding_k_per_w_milli / MILLI_PER_UNIT;
  const int into_iron = thermal_sink_edge(THERMAL_WINDING);

  cfg->node[THERMAL_WINDING].capacity =
      (float)cal->winding_j_per_k_milli / MILLI_PER_UNIT;
  if ((k > 0.0f) && (into_iron >= 0))
  {
    cfg->r_edge[into_iron] = WINDING_INTO_IRON * k;
    cfg->node[THERMAL_STATOR].to_ambient = (1.0f - WINDING_INTO_IRON) * k;
  }
}

/** The network: the core's defaults with every non-zero record entry laid
    over its own field, in the order the overrides stack. */
static void network_from_cal(thermal_cfg_t *cfg)
{
  const board_cal_t *cal = Board_Cal();

  thermal_defaults(cfg);
  lay_bulk(cfg, cal);
  share_bulk(cfg, cal);
  lay_nodes(cfg, cal);
  lay_edges(cfg, cal);
  lay_winding(cfg, cal);
}

/** The losses: the core's table with the record's phase resistance, the one
    loss constant the record carries. */
static void losses_from_cal(void)
{
  thermal_losses(&s.loss);
  s.loss.r_phase = (float)Board_Cal()->motor_r_uohm / MICRO_PER_UNIT;
  s.loss.k_iron = (float)Board_Cal()->thermal_k_iron_milli / MILLI_PER_UNIT;
}

/** The network the observer runs: the record's base with the identified
    scales on it. */
static void network_refresh(void)
{
  network_from_cal(&s.base);
  thermal_ident_apply(&s.ident, &s.base, &s.th.cfg);
}

void Board_ThermalInit(void)
{
  thermal_cfg_t cfg;

  /* Start on the NTC if there is one, otherwise somewhere plausible. */
  int32_t raw = 0, centi = 0;
  const bool have = Board_Ntc(&raw, &centi);
  const float start_c = have ? ((float)centi / CENTI_PER_UNIT) : 25.0f;

  network_from_cal(&s.base);
  losses_from_cal();
  /* Fresh every boot: scales at one, the room at the thermistor, doubted
     whole. */
  thermal_ident_init(&s.ident, start_c, THERMAL_IDENT_NOISE_K);
  thermal_ident_apply(&s.ident, &s.base, &cfg);
  /* The envelope comes from the calibration record, not from this file. */
  soa_from_cal();

  thermal_init(&s.th, &cfg, start_c);
  memset(&s.power, 0, sizeof(s.power));
  s.winding_derate = 1.0f;
  s.last_ms = HAL_GetTick();
  s.sampled_ms = s.last_ms;
  s.held_ms = s.last_ms;
  s.holding = false;
  s.millis = 0U;
  s.steps = 0U;
  s.speed_rpm = 0.0f;
  s.ready = true;
}

/** How fast the derate may RECOVER, per second. */
#define THERMAL_DERATE_RECOVER_PER_S 0.05f

static float derate_applied(float want, uint32_t since_ms)
{
  static float held = 1.0f;

  if (want <= held)
  {
    held = want;              /* down is immediate */
  }
  else
  {
    held += THERMAL_DERATE_RECOVER_PER_S * ((float)since_ms / MILLI_PER_UNIT);
    if (held > want)
    {
      held = want;
    }
  }
  if (held < 0.0f)
  {
    held = 0.0f;
  }
  if (held > 1.0f)
  {
    held = 1.0f;
  }
  return held;
}

/** Read every thermometer. */
static void sense_read(thermal_sense_t *out)
{
  int32_t raw = 0, centi = 0;

  out->ntc_c = Board_Ntc(&raw, &centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;
  out->mcu_c = Board_McuDie(&raw, &centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;

  /* The A1335 sits in the AFE corner, so its die anchors THAT node - not the
     board. */
  out->afe_c = Board_AngleDie(&centi) ? ((float)centi / CENTI_PER_UNIT) : NAN;
}

static void sense_sample(uint32_t now, thermal_sense_t *out)
{
  out->ntc_c = NAN;
  out->afe_c = NAN;
  out->mcu_c = NAN;

  /* Zero is OFF, and it has to be said out loud: the period test is
     unsigned, so a zero period made the observer borrow the rail on EVERY
     poll instead of never, and pinned PE15 low. */
  const bool due = (s.every_ms != 0U) && ((now - s.sampled_ms) >= s.every_ms);

  if (!s.holding && !due)
  {
    return;
  }
  /* Somebody else already has the rail up - read it and borrow nothing. */
  if (!s.holding && Board_AfeOn())
  {
    sense_read(out);
    s.sampled_ms = now;
    return;
  }
  if (!s.holding && !Board_PowerAcquire(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
  {
    /* Armed. Back off a whole interval rather than retrying at 10 Hz. */
    s.sampled_ms = now;
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
  s.sampled_ms = now;
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

/** What heats the board this step: the duties, the phase currents while the
    synced triple is armed, the link voltage when the AFE lets it be read,
    the dead time the record holds, the speed the drive estimates. */
static void load_now(thermal_load_t *load)
{
  const uint32_t period = Board_PwmPeriod();

  load->afe_on = Board_AfeOn();
  load->switching = Board_PwmIsEnabled();
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

    /* AND THE MEAN SQUARE, which is what the conduction is actually made of. */
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

/** After every poll: the ceilings follow the margin, re-trimmed when it has
    moved a step - every sample while the evidence comes in, never on a slice
    that changed nothing. */
static void margin_follow(void)
{
  const float now = margin_now();

  if (fabsf(now - s.margin) >= THERMAL_MARGIN_STEP)
  {
    soa_from_cal();
  }
}

/** One slice of the observer: the losses on its own last estimate, the step,
    the identification beside it, the room, the budget. */
static void step_slice(const thermal_load_t *load, const thermal_sense_t *seen,
                       uint32_t slice)
{
  const float dt = (float)slice / MILLI_PER_UNIT;
  /* The FET tempco feeds on the observer's own last estimate: the driver
     node a leg heats is the junction its on-resistance follows. */
  const float phase_c[3] = { s.th.t[THERMAL_DRIVER(0)],
                             s.th.t[THERMAL_DRIVER(1)],
                             s.th.t[THERMAL_DRIVER(2)] };

  thermal_power_estimate(&s.power, load, &s.loss, phase_c);
  thermal_step(&s.th, &s.power, seen, load, dt);
  /* THE IDENTIFICATION, beside it: the shadow and its sensitivities step
     with the same power and the same slice; when a sample moves the scales
     the observer's network takes them at once. */
  if (thermal_ident_step(&s.ident, &s.th, &s.base, &s.power, load, seen, dt))
  {
    thermal_ident_apply(&s.ident, &s.base, &s.th.cfg);
  }
  /* THE ROOM IS THE IDENTIFICATION'S: the board has no ambient sensor, and
     the observer's rise is against what the identification says the room is. */
  s.th.ambient = thermal_ident_ambient(&s.ident);
  thermal_budget(&s.th, &s.power, &s.soa, &s.budget);
  /* The winding's own factor, so a host can say which envelope holds the
     stage back; the whole's already includes it. */
  s.winding_derate = thermal_node_derate(&s.th, &s.power, &s.soa,
                                         THERMAL_WINDING);
}

/** THE ONE PLACE THIS FILE ACTS RATHER THAN REPORTS, and it acts twice. */
static void hold_envelope(uint32_t slice, uint32_t now)
{
  Board_DriveDerate(derate_applied(s.budget.derate, slice));

  if (!s.budget.tripped || !Board_PwmIsEnabled())
  {
    return;
  }
  Board_PwmDisable();
  s.trips++;
  /* AND THE ENVELOPE SHRINKS: the trip cap, from now, recovering a percent a
     minute (board_limits.h). */
  s.trip_cap = THERMAL_TRIP_MARGIN;
  s.trip_ms = now;
  soa_from_cal();
}

void Board_ThermalPoll(void)
{
  if (!s.ready)
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  const uint32_t since = now - s.last_ms;      /* unsigned: the wrap is free */

  if (since < THERMAL_STEP_MS)
  {
    return;
  }
  s.last_ms = now;

  thermal_load_t load;

  memset(&load, 0, sizeof(load));
  load.link_amps = -1.0f;               /* < 0: estimate it from the phases */

  load_now(&load);
  s.speed_rpm = load.speed_rpm;

  thermal_sense_t seen;

  sense_sample(now, &seen);

  /* Keep whatever answered. */
  if (!isnan(seen.ntc_c) || !isnan(seen.afe_c) || !isnan(seen.mcu_c))
  {
    s.last_seen = seen;
    s.seen_ms = now;
    s.seen = true;
  }

  /* IN SLICES, AND THE ENVELOPE ON EVERY ONE. */
  uint32_t left = (since > THERMAL_CATCHUP_MS) ? THERMAL_CATCHUP_MS : since;

  while (left > 0U)
  {
    const uint32_t slice = (left > THERMAL_STEP_MS) ? THERMAL_STEP_MS : left;

    left -= slice;
    step_slice(&load, &seen, slice);
    hold_envelope(slice, now);
    s.steps++;
  }

  /* Milliseconds, divided only on the way out. */
  s.millis += since;

  margin_follow();
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
  /* A flag, not `s.seen_ms != 0`: HAL_GetTick() is 0 at boot and again every
     49.7 days, and a sample taken on that tick would read "just now" for as
     long as the board stayed up. */
  out->seen_ms_ago = s.seen ? (HAL_GetTick() - s.seen_ms) : 0U;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->node_centidegc[i] = (int32_t)(s.th.t[i] * CENTI_PER_UNIT);
  }
  out->ambient_centidegc = (int32_t)(s.th.ambient * CENTI_PER_UNIT);
  out->expected_ntc_centidegc = (int32_t)(thermal_expected_ntc(&s.th) * CENTI_PER_UNIT);
  out->seconds = s.millis / MS_PER_S;
  out->steps = s.steps;
  out->settled = s.th.settled;
  for (int leg = 0; leg < 3; leg++)
  {
    const float over = thermal_junction(&s.th, &s.power, THERMAL_DRIVER(leg))
                       - s.th.t[THERMAL_DRIVER(leg)];

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

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->used[i] = s.budget.used[i];
  }
  out->worst = s.budget.worst;
  out->worst_node = s.budget.worst_node;
  out->millis_to_limit = s.budget.millis_to_limit;
  out->throttling = s.budget.throttling;
  out->tripped = s.budget.tripped;
  /* What is APPLIED, not what the arithmetic asked for: the recovery slew is
     part of the answer and a host that saw the raw factor would see it
     flicker while the clamp did not. */
  out->derate = Board_DriveDerating();
  for (uint8_t i = 0U; i < BOARD_THERMAL_NODES; i++)
  {
    out->soak_j[i] = s.budget.soak_j[i];
  }
  /* The EFFECTIVE duty, off the compares themselves rather than off whatever
     was last asked for: what the clamp and the derate left. */
  {
    const uint32_t period = Board_PwmPeriod();

    for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
    {
      out->duty[i] = (period > 0U)
        ? ((float)Board_PwmGetDuty(i) / (float)period) : 0.0f;
    }
  }
  out->trips = s.trips;
  /* MINOR 12's winding fields, from the node it is now: the same numbers a
     host read when it was a separate element. */
  out->winding_c = s.th.t[THERMAL_WINDING];
  out->winding_used = s.budget.used[THERMAL_WINDING];
  out->winding_derate = s.winding_derate;
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
  /* The estimate carries on from where it is: a new ceiling or a new
     constant changes what the winding is judged by, not what it is at. */
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
      || !thermal_set_node(&s.th, (thermal_node_t)node, k_per_w, capacity))
  {
    return false;
  }
  /* And into the record's RAM copy, the same number: what the observer runs
     and what a save would keep cannot differ. */
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

  if (!thermal_set_edge(&s.th, (int)edge, r))
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
  *k_per_w = s.th.cfg.r_edge[edge];
  return true;
}

bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient,
                          float *area_share, float *rth_die, float *forced)
{
  if (!s.ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  const thermal_node_cfg_t *n = &s.th.cfg.node[node];

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

bool Board_ThermalSetBoard(float to_ambient, float capacity)
{
  if (!s.ready || !thermal_set_board(&s.th, to_ambient, capacity))
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
  out->state = (uint8_t)s.ident.state;
  out->online_mask = 0U;
  for (int k = 0; k < THERMAL_IDENT_RECORD; k++)
  {
    if (thermal_ident_online((thermal_ident_param_t)k))
    {
      out->online_mask |= (uint8_t)(1U << k);
    }
    out->scale[k] = s.ident.scale[k];
    out->sigma[k] = thermal_ident_sigma(&s.ident, (thermal_ident_param_t)k);
  }
  out->innovation_k = s.ident.innovation_k;
  out->margin = s.margin;
  out->updates = s.ident.updates;
  out->ambient_c = thermal_ident_ambient(&s.ident);
  out->ambient_sigma_k = thermal_ident_sigma(&s.ident, THERMAL_IDENT_AMBIENT);
  out->margin_floor = margin_floor();
  out->trip_cap = trip_cap_now();
  return true;
}

bool Board_ThermalIdentReset(void)
{
  if (!s.ready)
  {
    return false;
  }
  thermal_ident_init(&s.ident, s.th.ambient, THERMAL_IDENT_NOISE_K);
  thermal_ident_apply(&s.ident, &s.base, &s.th.cfg);
  soa_from_cal();
  return true;
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
