/**
  ******************************************************************************
  * @file    board_thermal.c
  * @brief   Runs the lumped observer on this hardware.
  *
  * `Thermal/` is the network and knows no hardware. This file is the bridge:
  * it reads the NTC, gathers what the board is doing now, and steps the model
  * from the main loop.
  *
  * MEASURED AND ESTIMATED
  * The NTC is a measurement. Everything else is an estimate from power and
  * time, and the two must never be mixed in the reply - `0x6E` device 8 keeps
  * them in separate fields for exactly that reason. Invariant 10 holds: the
  * board reports, it does not judge, and an estimate that looks like a
  * measurement is the confusion invariant 9 exists for.
  *
  * The NTC needs AFE_ON, which the gate drivers share through an inverted
  * gate. Rather than sit blind, the observer BORROWS the rail: it acquires it
  * through `board_power.h`, waits for the reference, reads, and gives it
  * back. `Board_PowerAcquire` refuses while the stage is armed, so a run at
  * duty is never interrupted and the model carries on open there.
  *
  * Borrowing costs the state it measures: 300 ms of settle per sample takes
  * the drivers' supply away for that long. Every 60 s that is 0.5 % of the
  * time in the wrong state, which against tau 6.8 min does not move the
  * equilibrium - measured on the bench 2026-08-28 as 0.42 s a sample.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "thermal.h"

#include <math.h>
#include <string.h>

/** How often the model is stepped from the main loop. The fastest node has a
  * time constant of tens of seconds, so 10 Hz is ample and costs nothing. */
#define THERMAL_STEP_MS 100U

/* The reply's array is sized by a literal in board.h; the loop that fills it
   runs to the enum in thermal.h. Nothing tied them, so adding a node to the
   enum wrote past the end of a caller's stack local. */
_Static_assert(BOARD_THERMAL_NODES == (int)THERMAL_NODES,
               "board.h's node count and thermal.h's enum disagree - the "
               "reply array would be written past its end");

/** How often the rail is borrowed for an NTC sample, by default.
  *
  * Five seconds against a 6.8-minute time constant is 80 samples a tau, so
  * the anchoring sees the board move rather than jumping to it. The cost is
  * the settle below: 300 ms in 5000 is 6 % of the time with the drivers
  * unpowered, which is why it is settable and why the gate stage refuses it
  * outright rather than trading it off. */
#define THERMAL_SAMPLE_EVERY_MS 5000U

/** How long the reference is given before the sample is believed.
  *
  * MEASURED 2026-08-28, paired A/B, 12 pairs alternating which went first so
  * the board's own drift cancels: 500 ms minus 100 ms is +0.005 K with a
  * standard error of 0.008 K - 0.6 sigma, and below the NTC's own 30 mK
  * quantisation. The reference is up well before 100 ms.
  *
  * An earlier reading of the same question was wrong and is worth keeping:
  * four samples ALL taken at 300 ms agreed to 50 mK, which was read as "the
  * settle is done". It is not evidence - four samples taken equally early
  * would agree equally well while all being equally wrong. Only the paired
  * difference answers it.
  *
  * 500 ms rather than 100 because the margin is free: sampling is refused
  * outright while the gate stage is armed, so the extra 400 ms is only ever
  * spent on an idle board. */
#define THERMAL_SAMPLE_SETTLE_MS 500U

static thermal_t      s_th;
static thermal_loss_t s_loss;
static thermal_soa_t  s_soa;
static thermal_budget_t s_budget;
static uint32_t       s_trips;
static bool           s_ready;
static uint32_t       s_last_ms;
static bool           s_holding;      /**< the observer holds the AFE rail  */
static uint32_t       s_held_ms;      /**< when it took it                  */
static uint32_t       s_sampled_ms;   /**< when the last sample finished    */
static thermal_sense_t s_last_seen = { NAN, NAN, NAN };
static uint32_t       s_seen_ms;      /**< when s_last_seen was taken       */
static uint32_t       s_every_ms = THERMAL_SAMPLE_EVERY_MS;
static uint32_t       s_settle_ms = THERMAL_SAMPLE_SETTLE_MS;
static uint32_t       s_millis;

/** Copy the envelope out of the calibration record into the observer. */
static void soa_from_cal(void)
{
  const board_cal_t *cal = Board_Cal();

  memset(&s_soa, 0, sizeof(s_soa));
  for (uint8_t i = 0U; i < (uint8_t)THERMAL_NODES; i++)
  {
    s_soa.limit_c[i] = (float)cal->soa_limit_centi[i] / 100.0f;
  }
  s_soa.throttle_at = (float)cal->soa_throttle_ppm / 1000000.0f;
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
  s_ready = true;
}

/** Borrow the AFE rail, read EVERY thermometer, give it back.
  *
  * All three sit behind the same switch, so one borrow serves all of them -
  * three observations for one 500 ms perturbation instead of three. Reading
  * them separately would triple the time the gate drivers spend unpowered
  * and buy nothing, because they are never available apart.
  *
  * Spread over several polls rather than blocking: the settle is 500 ms and
  * the main loop also carries the STO keepalive, which is the one thing that
  * must not stop.
  */
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
     AFE held up by anything else the observer did two ADC conversions - one
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
       observer borrow the rail on EVERY poll instead of never, which is the
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

  thermal_load_t load = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                          0.0f, -1.0f, false, false };

  load.afe_on = Board_AfeOn();
  load.switching = Board_PwmIsEnabled();

  const uint32_t period = Board_PwmPeriod();

  for (uint8_t i = 0U; i < 3U; i++)
  {
    load.duty[i] = (period > 0U)
                   ? ((float)Board_PwmGetDuty(i) / (float)period) : 0.0f;
  }

  /* The phases come from the synced triple while it is armed. When it is
     not, the current is unknown, and zero is then the only honest answer: a
     guessed current becomes a guessed conduction loss that looks measured. */
  if (Board_SyncArmed())
  {
    board_sync_sample_t sample;

    Board_SyncLatest(&sample);
    for (uint8_t i = 0U; i < 3U; i++)
    {
      /* Raw codes to amperes belong in the calibration record - invariant 7
         says the conversion lives where it is defined, not here. Until that
         path exists the current is left at zero rather than scaled by a
         literal. */
      (void)sample.phase[i];
      load.phase_amps[i] = 0.0f;
    }
  }

  int32_t dc_raw = 0, millivolt = 0;
  if (Board_DcBus(&dc_raw, &millivolt))
  {
    load.link_volts = (float)millivolt / 1000.0f;
  }

  thermal_power_t p;
  thermal_power_estimate(&p, &load, &s_loss);

  thermal_sense_t seen;

  sense_sample(now, &seen);

  /* Keep whatever answered. Board_ThermalState used to take its own ADC
     reading on every query, which cost a conversion per host poll and could
     land in the middle of the sampler's borrow. */
  if (!isnan(seen.ntc_c) || !isnan(seen.afe_c) || !isnan(seen.mcu_c))
  {
    s_last_seen = seen;
    s_seen_ms = now;
  }

  thermal_step(&s_th, &p, &seen, (float)since / 1000.0f);

  thermal_budget(&s_th, &p, &s_soa, &s_budget);

  /* THE ONE PLACE THIS FILE ACTS RATHER THAN REPORTS. A trip drops MOE, so
     every gate goes to its idle level in hardware and stays there until
     something arms it again - the same path the break uses. It is protection,
     not a verdict on a reading: the estimate is still reported either way,
     and the limits came from the calibration record rather than from here. */
  if (s_budget.tripped && Board_PwmIsEnabled())
  {
    Board_PwmDisable();
    s_trips++;
  }

  /* Milliseconds, divided only on the way out. `since / 1000U` per step was
     integer division of about 100, so the counter never left zero. */
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
  out->seen_ms_ago = (s_seen_ms != 0U) ? (HAL_GetTick() - s_seen_ms) : 0U;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    out->node_centidegc[i] = (int32_t)(s_th.t[i] * 100.0f);
  }
  out->ambient_centidegc = (int32_t)(s_th.ambient * 100.0f);
  out->expected_ntc_centidegc = (int32_t)(thermal_expected_ntc(&s_th) * 100.0f);
  out->seconds = s_millis / 1000U;
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
     envelope. Then re-read, so what the observer uses and what would be
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
