/**
  ******************************************************************************
  * @file    board_thermal.c
  * @brief   Runs the lumped-network thermal observer on this hardware.
  *
  * `thermal/` is the network and knows no hardware. This reads the sensors,
  * gathers what the board is doing - the currents, the link, the dead time
  * out of the record, the speed out of the drive - and steps the model from
  * the main loop.
  *
  * The NTC is a MEASUREMENT; every node is an estimate. `0x6E` device 8 keeps
  * them in separate fields, and invariant 9 is why.
  *
  * All three thermometers sit behind AFE_ON, which the gate drivers share
  * through an inverted gate. The thermal observer borrows the rail, reads,
  * and gives it back; `Board_PowerAcquire` refuses while the stage is armed,
  * so a run at duty is never interrupted and the model carries on open.
  *
  * THE NETWORK IS THE RECORD'S, over the core's derived defaults: every
  * non-zero entry in `board_cal_t`'s thermal tables overlays the default
  * for that one field at init and whenever a setter writes one, so what the
  * observer runs and what a save would keep cannot differ.
  ******************************************************************************
  */
#include "board_limits.h"
#include "board.h"
#include "board_drive.h"
#include "board_hw.h"
#include "board_power.h"
#include "drive.h"
#include "thermal.h"

#include <math.h>
#include <string.h>

/* The reply's arrays are sized by literals in board.h; the loops that fill
   them run to the enum in thermal.h. Nothing tied them, so adding a node to
   the enum wrote past the end of a caller's stack local. */
_Static_assert(BOARD_THERMAL_NODES == (int)THERMAL_NODES,
               "board.h's node count and thermal.h's enum disagree - the "
               "reply array would be written past its end");
_Static_assert(BOARD_THERMAL_EDGES == THERMAL_EDGES,
               "board.h's edge count and thermal.h's table disagree");

static thermal_t      s_th;
static thermal_loss_t s_loss;
static thermal_power_t s_power;

/** The last DC link voltage that was a MEASUREMENT, volts. Negative: none yet.
  *
  * INVARIANT 9, and it cost a factor of 1.6. AFE_ON powers the ADC reference,
  * and switching needs AFE_ON low - so every estimate taken while the stage
  * runs reads the link at exact mid-scale. Through the 49.9k/2.2k divider
  * that is 39.1 V, against a supply on 24.9, and the switching term scales
  * by link volts: measured 2026-08-29, one leg at 50 % came out 14.9 K over
  * board where the calibration says 9.1.
  *
  * The bus does not move when the rail toggles, so the last real reading is
  * the honest one to carry. The thermal observer's own periodic sample is
  * what refreshes it - that runs with the AFE up, which is the point of it.
  */
static float s_link_volts = -1.0f;
static thermal_soa_t  s_soa;
static thermal_budget_t s_budget;
/** The winding's own factor, beside the whole's: which envelope holds
  * the stage back. */
static float          s_winding_derate = 1.0f;
static uint32_t       s_trips;
static bool           s_ready;
static uint32_t       s_last_ms;
static bool           s_holding;      /**< the thermal observer holds the AFE rail  */
static uint32_t       s_held_ms;      /**< when it took it                  */
static uint32_t       s_sampled_ms;   /**< when the last sample finished    */
static thermal_sense_t s_last_seen = { NAN, NAN, NAN };
static uint32_t       s_seen_ms;      /**< when s_last_seen was taken       */
static bool           s_seen;         /**< whether anything answered yet    */
static uint32_t       s_every_ms = THERMAL_SAMPLE_EVERY_MS;
static uint32_t       s_settle_ms = THERMAL_SAMPLE_SETTLE_MS;
static uint32_t       s_millis;
static uint32_t       s_steps;        /**< model integrations, for a rate  */
static float          s_speed_rpm;    /**< the rotor at the last step      */


/** Copy the envelope out of the calibration record into the thermal observer. */
static void soa_from_cal(void)
{
  const board_cal_t *cal = Board_Cal();

  memset(&s_soa, 0, sizeof(s_soa));
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    s_soa.limit_c[i] = (float)cal->soa_limit_centi[i] / 100.0f;
    /* Which of them the clamp can actually cool - `board.h` has why the
       housekeeping nodes are judged but not throttled on. */
    s_soa.undriven[i] = ((cal->soa_undriven_mask >> i) & 1UL) != 0UL;
  }
  /* The winding's ceiling is the record's own field, kept since 12 so
     op 6 and id 48 keep their meaning; zero disables it as before. */
  s_soa.limit_c[THERMAL_WINDING] = (float)cal->winding_limit_centi / 100.0f;
  s_soa.throttle_at = (float)cal->soa_throttle_ppm / 1000000.0f;
  s_soa.lookahead_s = (float)cal->soa_lookahead_ms / 1000.0f;
}


/** The network: the core's defaults with every non-zero record entry laid
  * over its own field. The winding's three record fields feed its node
  * and its edge into the iron the way they did the separate element:
  * a quarter of the K/W into the iron, the rest the iron's air path. */
static void network_from_cal(thermal_cfg_t *cfg)
{
  const board_cal_t *cal = Board_Cal();

  thermal_defaults(cfg);

  if (cal->thermal_to_ambient_milli != 0U)
  {
    cfg->board_to_ambient = (float)cal->thermal_to_ambient_milli / 1000.0f;
  }
  if (cal->thermal_rad_share_ppm != 0U)
  {
    cfg->board_rad_share = (float)cal->thermal_rad_share_ppm / 1.0e6f;
  }
  if (cal->thermal_ntc_sees_ppm != 0U)
  {
    cfg->ntc_sees = (float)cal->thermal_ntc_sees_ppm / 1.0e6f;
  }
  if (cal->thermal_ntc_tau_ms != 0U)
  {
    cfg->ntc_tau_s = (float)cal->thermal_ntc_tau_ms / 1000.0f;
  }
  cfg->rad_board_stator = (float)cal->thermal_rad_board_stator_micro / 1.0e6f;

  /* The bulk laminate: shared out by area, as `thermal_set_board` does,
     before any patch's own entry overrides its share. */
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    thermal_node_cfg_t *n = &cfg->node[i];

    if (n->area_share > 0.0f)
    {
      if (cal->thermal_to_ambient_milli != 0U)
      {
        n->to_ambient = cfg->board_to_ambient / n->area_share;
      }
      if (cal->thermal_capacity_milli != 0U)
      {
        n->capacity = ((float)cal->thermal_capacity_milli / 1000.0f)
                      * n->area_share;
      }
    }
  }
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    const board_cal_node_t *rec = &cal->thermal_node[i];
    thermal_node_cfg_t *n = &cfg->node[i];

    if (rec->capacity_milli != 0U)
    {
      n->capacity = (float)rec->capacity_milli / 1000.0f;
    }
    if (rec->to_ambient_milli != 0U)
    {
      n->to_ambient = (float)rec->to_ambient_milli / 1000.0f;
    }
    if (rec->forced_milli != 0U)
    {
      n->forced = (float)rec->forced_milli / 1000.0f;
    }
    if (rec->rth_milli != 0U)
    {
      n->rth_die = (float)rec->rth_milli / 1000.0f;
    }
  }
  for (uint8_t e = 0U; e < (uint8_t)THERMAL_EDGES; e++)
  {
    const uint32_t milli = cal->thermal_edge_milli[e];

    if (milli == BOARD_CAL_EDGE_OPEN)
    {
      cfg->r_edge[e] = 0.0f;
    }
    else if (milli != 0U)
    {
      cfg->r_edge[e] = (float)milli / 1000.0f;
    }
  }

  /* The winding, from its own three fields (CAL_VERSION 12). */
  {
    const float k = (float)cal->winding_k_per_w_milli / 1000.0f;
    const int into_iron = thermal_sink_edge(THERMAL_WINDING);

    cfg->node[THERMAL_WINDING].capacity =
        (float)cal->winding_j_per_k_milli / 1000.0f;
    if ((k > 0.0f) && (into_iron >= 0))
    {
      cfg->r_edge[into_iron] = 0.25f * k;
      cfg->node[THERMAL_STATOR].to_ambient = 0.75f * k;
    }
  }
}


/** The losses: the core's table with the record's phase resistance, the
  * one loss constant the record carries. */
static void losses_from_cal(void)
{
  thermal_losses(&s_loss);
  s_loss.r_phase = (float)Board_Cal()->motor_r_uohm / 1.0e6f;
  s_loss.k_iron = (float)Board_Cal()->thermal_k_iron_milli / 1000.0f;
}


void Board_ThermalInit(void)
{
  thermal_cfg_t cfg;

  network_from_cal(&cfg);
  losses_from_cal();
  /* The envelope comes from the calibration record, not from this file. A
     ceiling the firmware invented would be the judgement invariant 10
     forbids; one it was given is a parameter like any other. */
  soa_from_cal();

  /* Start on the NTC if there is one, otherwise somewhere plausible. A wrong
     starting point is gone within a few minutes through the anchoring. The
     motor starts where the board does: a motor that has not turned is at
     the room. */
  int32_t raw = 0, centi = 0;
  const bool have = Board_Ntc(&raw, &centi);

  thermal_init(&s_th, &cfg, have ? ((float)centi / 100.0f) : 25.0f);
  memset(&s_power, 0, sizeof(s_power));
  s_winding_derate = 1.0f;
  s_last_ms = HAL_GetTick();
  s_sampled_ms = s_last_ms;
  s_held_ms = s_last_ms;
  s_holding = false;
  s_millis = 0U;
  s_steps = 0U;
  s_speed_rpm = 0.0f;
  s_ready = true;
}


/** How fast the derate may RECOVER, per second. Falling is immediate.
  *
  * ASYMMETRIC ON PURPOSE. `thermal_budget` reports the factor the
  * present ramp deserves, and that factor is part of a loop: cut the
  * clamp and the ramp goes away, so the next poll sees no ramp and asks
  * for full current again. Measured on the stand-in, that oscillated
  * between 1.00 and 0.00 every hundred milliseconds while the node sat
  * at nine tenths of its ceiling - a stage chattering at its own poll
  * rate, which is worse for the silicon than the derate was for.
  *
  * Cutting instantly and recovering over twenty seconds breaks the loop:
  * the node has time to actually cool before the current comes back.
  * FOUR SECONDS WAS NOT ENOUGH - the phase node's own constant is about
  * eighteen, so a derate that recovered in four re-heated it before it
  * had cooled and the stage tripped anyway. The recovery has to be slow
  * against the thing it is protecting, not against the poll.
  */
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
    held += THERMAL_DERATE_RECOVER_PER_S * ((float)since_ms / 1000.0f);
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


/** Read every thermometer. One borrow serves all three - they are never
  * available apart, so reading them separately would triple the time the
  * drivers spend unpowered and buy nothing. */
static void sense_read(thermal_sense_t *out)
{
  int32_t raw = 0, centi = 0;

  out->ntc_c = Board_Ntc(&raw, &centi) ? ((float)centi / 100.0f) : NAN;
  out->mcu_c = Board_McuDie(&raw, &centi) ? ((float)centi / 100.0f) : NAN;

  /* The A1335 sits in the AFE corner, so its die anchors THAT node - not
     the board. Getting that backwards is what made this sensor look
     useless. Two SPI frames, and only inside the borrow: the part loses its
     supply with AFE_ON low like everything else here. */
  out->afe_c = Board_AngleDie(&centi) ? ((float)centi / 100.0f) : NAN;
}


static void sense_sample(uint32_t now, thermal_sense_t *out)
{
  out->ntc_c = NAN;
  out->afe_c = NAN;
  out->mcu_c = NAN;

  /* Somebody else already has the rail up - read it and borrow nothing.
     STILL ON THE INTERVAL: free of the rail is not free of the bus, and
     the A1335's register rotation is shared with the angle poll. */
  if (!s_holding && Board_AfeOn())
  {
    if ((s_every_ms == 0U) || ((now - s_sampled_ms) < s_every_ms))
    {
      return;
    }
    sense_read(out);
    s_sampled_ms = now;
    return;
  }

  if (!s_holding)
  {
    /* Zero is OFF, and it has to be said out loud: the period test is
       unsigned, so a zero period made the observer borrow the rail on
       EVERY poll instead of never, and pinned PE15 low. */
    if ((s_every_ms == 0U) || ((now - s_sampled_ms) < s_every_ms))
    {
      return;
    }
    if (!Board_PowerAcquire(BOARD_RAIL_AFE, BOARD_USER_THERMAL))
    {
      /* Armed. Back off a whole interval rather than retrying at 10 Hz. */
      s_sampled_ms = now;
      return;
    }
    s_holding = true;
    s_held_ms = now;
    return;                         /* the reference has not come up yet */
  }

  if ((now - s_held_ms) < s_settle_ms)
  {
    return;
  }

  sense_read(out);

  (void)Board_PowerRelease(BOARD_RAIL_AFE, BOARD_USER_THERMAL);
  s_holding = false;
  s_sampled_ms = now;
}


/** The rotor's mechanical speed, rpm, off the drive's observer: its
  * electrical rad/s over the record's pole pairs. Zero when the drive is
  * not running its law - a speed nobody estimated is not a speed. */
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


/** What heats the board this step: the duties, the phase currents while
  * the synced triple is armed, the link voltage when the AFE lets it be
  * read, the dead time the record holds, the speed the drive estimates.
  * Unarmed, the current is unknown and zero is the only honest answer. */
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
         calibration record (invariant 7). No invariant 9 guard needed:
         the channels are differential, so an unpowered reference's
         mid-scale is zero amperes. */
      load->phase_amps[i] = Board_PhaseAmps(i, sample.phase[i]);
    }

    /* AND THE MEAN SQUARE, which is what the conduction is actually made
       of. The sample above stays for the link estimate, which needs a
       sign; `phase_sq` zero means none arrived. */
    (void)Board_SyncMeanSquare(load->phase_sq);
  }

  int32_t dc_raw = 0, millivolt = 0;
  if (load->afe_on && Board_DcBus(&dc_raw, &millivolt))
  {
    s_link_volts = (float)millivolt / 1000.0f;
  }
  /* Zero says "never measured", and the model falls back to the voltage its
     switching figure was calibrated at rather than inventing a scale. */
  load->link_volts = (s_link_volts > 0.0f) ? s_link_volts : 0.0f;
  load->t_dead_s = (float)Board_Cal()->deadtime_ns * 1.0e-9f;
  load->speed_rpm = speed_now();
}


void Board_ThermalPoll(void)
{
  if (!s_ready)
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  const uint32_t since = now - s_last_ms;      /* unsigned: the wrap is free */

  if (since < THERMAL_STEP_MS)
  {
    return;
  }
  s_last_ms = now;

  thermal_load_t load;

  memset(&load, 0, sizeof(load));
  load.link_amps = -1.0f;               /* < 0: estimate it from the phases */

  load_now(&load);
  s_speed_rpm = load.speed_rpm;

  thermal_sense_t seen;

  sense_sample(now, &seen);

  /* Keep whatever answered. */
  if (!isnan(seen.ntc_c) || !isnan(seen.afe_c) || !isnan(seen.mcu_c))
  {
    s_last_seen = seen;
    s_seen_ms = now;
    s_seen = true;
  }

  /* IN SLICES, AND THE ENVELOPE ON EVERY ONE. The gap is normally exactly
     THERMAL_STEP_MS and this loop runs once; what it exists for is the gap
     that is not, because a starved main loop used to cost the throttle its
     whole ramp - the ramp is the last `lookahead_s * (1 - throttle_at)` of
     a node's hold, 200 ms at the record's numbers, and one step longer
     than that lands on the far side of it. Measured 2026-09-03 in
     `test_thermal_core.py`. The core sub-steps its own integration inside
     each slice; the slice is the ENVELOPE's cadence. */
  uint32_t left = (since > THERMAL_CATCHUP_MS) ? THERMAL_CATCHUP_MS : since;

  while (left > 0U)
  {
    const uint32_t slice = (left > THERMAL_STEP_MS) ? THERMAL_STEP_MS : left;

    left -= slice;

    /* The FET tempco feeds on the observer's own last estimate: the
       driver node a leg heats is the junction its on-resistance follows. */
    const float phase_c[3] = { s_th.t[THERMAL_DRIVER(0)],
                               s_th.t[THERMAL_DRIVER(1)],
                               s_th.t[THERMAL_DRIVER(2)] };

    thermal_power_estimate(&s_power, &load, &s_loss, phase_c);
    thermal_step(&s_th, &s_power, &seen, &load, (float)slice / 1000.0f);
    thermal_budget(&s_th, &s_power, &s_soa, &s_budget);
    /* The winding's own factor, so a host can say which envelope holds
       the stage back; the whole's already includes it. */
    s_winding_derate = thermal_node_derate(&s_th, &s_power, &s_soa,
                                           THERMAL_WINDING);

    /* THE ONE PLACE THIS FILE ACTS RATHER THAN REPORTS, and it acts twice.

       FIRST IT DERATES. Past the throttle point the drive's current clamp
       is scaled toward zero, so the stage keeps driving on less - which is
       what a thermal envelope is for. THEN, only if that was not enough,
       it drops MOE - every gate to its idle level in hardware, the same
       path the break uses. Protection, not a verdict on a reading: the
       estimate is reported either way and the limits came from the
       calibration record rather than from here. One clamp, every node the
       clamp reaches - the winding among them since it is a node. */
    Board_DriveDerate(derate_applied(s_budget.derate, slice));

    if (s_budget.tripped && Board_PwmIsEnabled())
    {
      Board_PwmDisable();
      s_trips++;
    }
    s_steps++;
  }

  /* Milliseconds, divided only on the way out. The whole gap, not the
     slices: this is wall time, and a stall happened whether or not the
     observer chose to integrate all of it. */
  s_millis += since;
}


bool Board_ThermalState(board_thermal_t *out)
{
  if ((out == NULL) || !s_ready)
  {
    return false;
  }

  out->ntc_measured = !isnan(s_last_seen.ntc_c);
  out->ntc_centidegc = out->ntc_measured
                       ? (int32_t)(s_last_seen.ntc_c * 100.0f) : 0;
  out->afe_measured = !isnan(s_last_seen.afe_c);
  out->afe_centidegc = out->afe_measured
                       ? (int32_t)(s_last_seen.afe_c * 100.0f) : 0;
  out->mcu_measured = !isnan(s_last_seen.mcu_c);
  out->mcu_centidegc = out->mcu_measured
                       ? (int32_t)(s_last_seen.mcu_c * 100.0f) : 0;
  /* A flag, not `s_seen_ms != 0`: HAL_GetTick() is 0 at boot and again every
     49.7 days, and a sample taken on that tick would read "just now" for as
     long as the board stayed up. */
  out->seen_ms_ago = s_seen ? (HAL_GetTick() - s_seen_ms) : 0U;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->node_centidegc[i] = (int32_t)(s_th.t[i] * 100.0f);
  }
  out->ambient_centidegc = (int32_t)(s_th.ambient * 100.0f);
  out->expected_ntc_centidegc = (int32_t)(thermal_expected_ntc(&s_th) * 100.0f);
  out->seconds = s_millis / 1000U;
  out->steps = s_steps;
  out->settled = s_th.settled;
  for (int leg = 0; leg < 3; leg++)
  {
    const float over = thermal_junction(&s_th, &s_power, THERMAL_DRIVER(leg))
                       - s_th.t[THERMAL_DRIVER(leg)];

    out->junction_over_centi[leg] = (int32_t)(over * 100.0f);
  }
  out->speed_rpm = (int32_t)s_speed_rpm;
  return true;
}


bool Board_ThermalBudget(board_budget_t *out)
{
  if ((out == NULL) || !s_ready)
  {
    return false;
  }

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->used[i] = s_budget.used[i];
  }
  out->worst = s_budget.worst;
  out->worst_node = s_budget.worst_node;
  out->millis_to_limit = s_budget.millis_to_limit;
  out->throttling = s_budget.throttling;
  out->tripped = s_budget.tripped;
  /* What is APPLIED, not what the arithmetic asked for: the recovery
     slew is part of the answer and a host that saw the raw factor would
     see it flicker while the clamp did not. */
  out->derate = Board_DriveDerating();
  for (uint8_t i = 0U; i < BOARD_THERMAL_NODES; i++)
  {
    out->soak_j[i] = s_budget.soak_j[i];
  }
  /* The EFFECTIVE duty, off the compares themselves rather than off
     whatever was last asked for: what the clamp and the derate left. */
  {
    const uint32_t period = Board_PwmPeriod();

    for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
    {
      out->duty[i] = (period > 0U)
        ? ((float)Board_PwmGetDuty(i) / (float)period) : 0.0f;
    }
  }
  out->trips = s_trips;
  /* MINOR 12's winding fields, from the node it is now: the same numbers
     a host read when it was a separate element. */
  out->winding_c = s_th.t[THERMAL_WINDING];
  out->winding_used = s_budget.used[THERMAL_WINDING];
  out->winding_derate = s_winding_derate;
  return true;
}


bool Board_ThermalSetWinding(float limit_c, float k_per_w, float j_per_k)
{
  if (!s_ready)
  {
    return false;
  }
  if (!Board_CalSetWinding((int32_t)(limit_c * 100.0f),
                           (uint32_t)(k_per_w * 1000.0f),
                           (uint32_t)(j_per_k * 1000.0f)))
  {
    return false;
  }
  /* The estimate carries on from where it is: a new ceiling or a new
     constant changes what the winding is judged by, not what it is at. */
  thermal_cfg_t cfg;

  network_from_cal(&cfg);
  s_th.cfg = cfg;
  soa_from_cal();
  return true;
}


bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at)
{
  if (!s_ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  /* Through the record, so a save persists it and one place holds the
     envelope. The winding's ceiling is its own field. */
  if (node == (uint8_t)THERMAL_WINDING)
  {
    const board_cal_t *cal = Board_Cal();

    if (!Board_CalSetWinding((int32_t)(limit_c * 100.0f),
                             cal->winding_k_per_w_milli,
                             cal->winding_j_per_k_milli))
    {
      return false;
    }
  }
  else if (!Board_CalSetLimit(node, (int32_t)(limit_c * 100.0f)))
  {
    return false;
  }
  if ((throttle_at > 0.0f) && (throttle_at < 1.0f))
  {
    (void)Board_CalSetThrottle((uint32_t)(throttle_at * 1000000.0f));
  }
  soa_from_cal();
  return true;
}


bool Board_ThermalSetNode(uint8_t node, float k_per_w, float capacity)
{
  if (!s_ready
      || !thermal_set_node(&s_th, (thermal_node_t)node, k_per_w, capacity))
  {
    return false;
  }
  /* And into the record's RAM copy, the same number: what the observer
     runs and what a save would keep cannot differ. A sink that is an edge
     goes to the edge table; one that is the air to the node's. */
  const int edge = thermal_sink_edge((thermal_node_t)node);

  if (edge >= 0)
  {
    (void)Board_CalSetThermalEdge((uint8_t)edge,
                                  (uint32_t)(k_per_w * 1000.0f));
    (void)Board_CalSetThermalNode(node, (uint32_t)(capacity * 1000.0f),
                                  Board_Cal()->thermal_node[node]
                                      .to_ambient_milli);
  }
  else
  {
    (void)Board_CalSetThermalNode(node, (uint32_t)(capacity * 1000.0f),
                                  (uint32_t)(k_per_w * 1000.0f));
  }
  return true;
}


bool Board_ThermalSetEdge(uint8_t edge, float k_per_w)
{
  if (!s_ready || (edge >= (uint8_t)THERMAL_EDGES))
  {
    return false;
  }
  const float r = (k_per_w < 0.0f) ? 0.0f : k_per_w;

  if (!thermal_set_edge(&s_th, (int)edge, r))
  {
    return false;
  }
  return Board_CalSetThermalEdge(edge, (k_per_w < 0.0f)
                                 ? BOARD_CAL_EDGE_OPEN
                                 : (uint32_t)(k_per_w * 1000.0f));
}


bool Board_ThermalEdge(uint8_t edge, uint8_t *a, uint8_t *b, float *k_per_w)
{
  if (!s_ready || (edge >= (uint8_t)THERMAL_EDGES) || (a == NULL)
      || (b == NULL) || (k_per_w == NULL))
  {
    return false;
  }
  *a = THERMAL_EDGE_ENDS[edge].a;
  *b = THERMAL_EDGE_ENDS[edge].b;
  *k_per_w = s_th.cfg.r_edge[edge];
  return true;
}


bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient,
                          float *area_share, float *rth_die, float *forced)
{
  if (!s_ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  const thermal_node_cfg_t *n = &s_th.cfg.node[node];

  *capacity = n->capacity;
  *to_ambient = n->to_ambient;
  *area_share = n->area_share;
  *rth_die = n->rth_die;
  *forced = n->forced;
  return true;
}


bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms)
{
  if (!s_ready)
  {
    return false;
  }
  s_every_ms = every_ms;
  s_settle_ms = settle_ms;
  return true;
}


void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms)
{
  *every_ms = s_every_ms;
  *settle_ms = s_settle_ms;
}


bool Board_ThermalSetBoard(float to_ambient, float capacity)
{
  if (!s_ready || !thermal_set_board(&s_th, to_ambient, capacity))
  {
    return false;
  }
  return Board_CalSetThermalBulk((uint32_t)(to_ambient * 1000.0f),
                                 (uint32_t)(capacity * 1000.0f));
}
