/**
  ******************************************************************************
  * @file    board_thermal.c
  * @brief   Runs the lumped thermal observer on this hardware.
  *
  * `thermal/` is the network and knows no hardware. This reads the sensors,
  * gathers what the board is doing, and steps the model from the main loop.
  *
  * The NTC is a MEASUREMENT; every node is an estimate. `0x6E` device 8 keeps
  * them in separate fields, and invariant 9 is why.
  *
  * All three thermometers sit behind AFE_ON, which the gate drivers share
  * through an inverted gate. The thermal observer borrows the rail, reads, and gives
  * it back; `Board_PowerAcquire` refuses while the stage is armed, so a run
  * at duty is never interrupted and the model carries on open.
  ******************************************************************************
  */
#include "board_limits.h"
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "thermal.h"

#include <math.h>
#include <string.h>

/* The reply's array is sized by a literal in board.h; the loop that fills it
   runs to the enum in thermal.h. Nothing tied them, so adding a node to the
   enum wrote past the end of a caller's stack local. */
_Static_assert(BOARD_THERMAL_NODES == (int)THERMAL_NODES,
               "board.h's node count and thermal.h's enum disagree - the "
               "reply array would be written past its end");

static thermal_t      s_th;
static thermal_loss_t s_loss;

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
  * the honest one to carry. The thermal observer's own periodic sample is what
  * refreshes it - that runs with the AFE up, which is the point of it.
  */
static float s_link_volts = -1.0f;
static thermal_soa_t  s_soa;
static thermal_budget_t s_budget;
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
  s_soa.throttle_at = (float)cal->soa_throttle_ppm / 1000000.0f;
  s_soa.lookahead_s = (float)cal->soa_lookahead_ms / 1000.0f;
}


void Board_ThermalInit(void)
{
  thermal_cfg_t cfg;

  thermal_defaults(&cfg);
  thermal_losses(&s_loss);
  /* The envelope comes from the calibration record, not from this file. A
     ceiling the firmware invented would be the judgement invariant 10
     forbids; one it was given is a parameter like any other. */
  soa_from_cal();

  /* Start on the NTC if there is one, otherwise somewhere plausible. A wrong
     starting point is gone within a few minutes through the anchoring. */
  int32_t raw = 0, centi = 0;
  const bool have = Board_Ntc(&raw, &centi);

  thermal_init(&s_th, &cfg, have ? ((float)centi / 100.0f) : 25.0f);
  s_last_ms = HAL_GetTick();
  s_sampled_ms = s_last_ms;
  s_held_ms = s_last_ms;
  s_holding = false;
  s_millis = 0U;
  s_steps = 0U;
  s_ready = true;
}

/** Read every thermometer. One borrow serves all three - they are never
  * available apart, so reading them separately would triple the time the
  * drivers spend unpowered and buy nothing. */
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
     STILL ON THE INTERVAL. This path used to read on EVERY poll, so with the
     AFE held up by anything else the thermal observer did two ADC conversions - one
     of them 810.5 cycles - and two SPI4 transactions ten times a second, for
     an anchor whose gain is 0.05 Hz. Free of the rail is not free of the
     bus, and the A1335's register rotation is shared with the angle poll. */
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
    /* Zero is OFF, and it has to be said out loud. The period test is
       unsigned, so `(now - then) < 0` is never true - a zero period made the
       thermal observer borrow the rail on EVERY poll instead of never, which is the
       opposite of what the setter documents. Measured: it held AFE_ON
       continuously, which pinned PE15 low and made the conformance suite's
       independent witness read the same both ways. */
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


/** What heats the board this step: the duties, the phase currents while
  * the synced triple is armed, the link voltage when the AFE lets it be
  * read. Unarmed, the current is unknown and zero is the only honest
  * answer - a guessed current becomes a guessed conduction loss that
  * looks measured. */
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
         calibration record (invariant 7) - this was a hard zero for want
         of that path. Dry it reads ~0 A and that is right: nothing leaves
         the bridge. No invariant 9 guard needed: the channels are
         differential, so an unpowered reference's mid-scale is zero
         amperes - the same answer unsupplied amplifiers give. */
      load->phase_amps[i] = Board_PhaseAmps(i, sample.phase[i]);
    }

    /* AND THE MEAN SQUARE, which is what the conduction is actually made
       of. The sample above stays for the link estimate, which needs a
       sign; `phase_sq` zero means none arrived and the estimator falls
       back to squaring the sample. */
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

  /* Zeroed and then the one field that is not zero. Positionally the
     initializer had to be rewritten every time the struct grew a member,
     and the compiler was right to complain about it - `phase_sq` arrived
     and it was two warnings, in a build whose bar is none. */
  thermal_load_t load;

  memset(&load, 0, sizeof(load));
  load.link_amps = -1.0f;               /* < 0: estimate it from the phases */

  load_now(&load);

  thermal_power_t p;
  thermal_sense_t seen;

  sense_sample(now, &seen);

  /* Keep whatever answered. Board_ThermalState used to take its own ADC
     reading on every query, which cost a conversion per host poll and could
     land in the middle of the sampler's borrow. */
  if (!isnan(seen.ntc_c) || !isnan(seen.afe_c) || !isnan(seen.mcu_c))
  {
    s_last_seen = seen;
    s_seen_ms = now;
    s_seen = true;
  }

  /* IN SLICES, AND THE ENVELOPE ON EVERY ONE. The gap is normally exactly
     THERMAL_STEP_MS and this loop runs once; what it exists for is the gap
     that is not, because a starved main loop used to cost the throttle its
     whole ramp. The ramp is the last `lookahead_s * (1 - throttle_at)` of a
     node's hold - 300 ms at the record's numbers - and one step longer than
     that lands on the far side of it.

     Measured 2026-09-03 in `test_thermal_core.py`, a 100 A burst over the
     same two seconds of model time: at 100 ms steps the throttle first
     acts at 0.40 s with the node at 81 C and the clamp still coming off
     1.00; at 1 s steps the first look is at 178 C with the clamp already
     at 0.00, and at 2 s - which is all `thermal.c` will integrate at once -
     at 335 C. Same envelope, same power, different step: the throttle band
     is only there if something looks inside it.

     The stand-in had the same defect in its own shape and it is fixed the
     same way (`coaxial/simulated/power.py`), so the two still rehearse the
     same play. */
  uint32_t left = (since > THERMAL_CATCHUP_MS) ? THERMAL_CATCHUP_MS : since;

  while (left > 0U)
  {
    const uint32_t slice = (left > THERMAL_STEP_MS) ? THERMAL_STEP_MS : left;

    left -= slice;

    /* The FET tempco feeds on the observer's own last estimate: the phase
       node a leg heats is the junction its on-resistance follows. Taken
       inside the loop because the estimate is what the previous slice just
       moved; at start the nodes sit at ambient and the correction is a few
       percent. */
    const float phase_c[3] = { s_th.t[THERMAL_PHASE(0)],
                               s_th.t[THERMAL_PHASE(1)],
                               s_th.t[THERMAL_PHASE(2)] };

    thermal_power_estimate(&p, &load, &s_loss, phase_c);
    thermal_step(&s_th, &p, &seen, (float)slice / 1000.0f);
    thermal_budget(&s_th, &p, &s_soa, &s_budget);

    /* THE ONE PLACE THIS FILE ACTS RATHER THAN REPORTS, and it acts twice.

       FIRST IT DERATES. Past the throttle point the drive's current clamp
       is scaled toward zero, so the stage keeps driving on less - which is
       what a thermal envelope is for. Without it the envelope was a cliff:
       full current until the ceiling and then MOE off, with a throttle
       band that was computed, published and used by nothing.

       THEN, only if that was not enough, it drops MOE - every gate to its
       idle level in hardware, staying there until something arms it again,
       the same path the break uses. Protection, not a verdict on a
       reading: the estimate is reported either way and the limits came
       from the calibration record rather than from here. */
    Board_DriveDerate(derate_applied(s_budget.derate, slice));

    if (s_budget.tripped && Board_PwmIsEnabled())
    {
      Board_PwmDisable();
      s_trips++;
    }
    s_steps++;
  }

  /* Milliseconds, divided only on the way out. `since / 1000U` per step was
     integer division of about 100, so the counter never left zero. The whole
     gap, not the slices: this is wall time, and a stall happened whether or
     not the observer chose to integrate all of it. */
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
     long as the board stayed up. Same shape as the lease sentinel. */
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
  return true;
}


bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at)
{
  if (!s_ready || (node >= (uint8_t)THERMAL_NODES))
  {
    return false;
  }
  /* Through the record, so a save persists it and one place holds the
     envelope. Then re-read, so what the thermal observer uses and what would be
     written to flash cannot differ. */
  if (!Board_CalSetLimit(node, (int32_t)(limit_c * 100.0f)))
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


bool Board_ThermalSetNode(uint8_t node, float to_board, float capacity)
{
  return s_ready && thermal_set_node(&s_th, (thermal_node_t)node,
                                     to_board, capacity);
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
  return s_ready && thermal_set_board(&s_th, to_ambient, capacity);
}
