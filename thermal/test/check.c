/* Run the thermal observer against the campaign's four measured states.
 *
 * Build and run with the host gcc, no hardware:
 *   gcc -std=c11 -Wall -Wextra -Wconversion -I../inc ../src/thermal.c check.c
 *       -lm -o check && ./check
 *
 * This is not the arithmetic in thermal.c - it tests that the model with
 * those parameters actually lands on the temperatures it was calibrated from.
 * A network can have the right resistances and still not converge.
 */
#include "thermal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define AMBIENT   20.0f
#define SETTLE_S  (60.0f * 60.0f)      /* one hour, ~9 tau */
/* THE RATE THE BOARD RUNS AT, derived so the two cannot disagree. A leg
   holds a third of what the lumped node did, so 35 W moves it 8.75 K per
   step; at the 0.5 s this used to be, the whole ceiling fell inside one
   step and the budget was asked to warn about something already over -
   the test measured its own resolution, not the budget. */
#include "../../board/inc/board_limits.h"
#define STEP_S    ((float)THERMAL_STEP_MS / 1000.0f)

struct sample
{
  const char *tag;
  float watt[THERMAL_NODES];
  float board_measured;                /* dead surface, from the camera */
  float ntc_measured;                  /* -1 = not read in that state    */
};

/* The powers are what the differences gave; see thermal.c for the working.
   Passive: 0.666 mcu + 0.484 LDO drop + 0.05 other = 1.20 W (supply 50 mA). */
static const struct sample CASES[] =
{
  { "1 passive", { 0.0f, 0.0f, 0.666f, 0.534f, 0.0f,  0.0f }, 30.0f, 36.0f },
  { "2 afe on",  { 0.0f, 0.0f, 0.666f, 0.534f, 0.13f, 0.0f }, 31.1f, -1.0f },
  { "3 traffic", { 0.0f, 0.0f, 0.706f, 0.534f, 0.13f, 0.0f }, 31.4f, -1.0f },
  { "4 switch",  { 0.60f, 0.0f, 0.666f, 1.134f, 0.0f, 0.0f }, 40.0f, 55.6f },
};

/* Does a die sensor buy anything? Start 30 K wrong with ONLY the MCU die and
 * no NTC. The board should converge anyway: the node's rise is its power
 * times its spreading resistance, both of which the model has. The NTC could
 * not do this - it sits in the drivers' hot spot.
 */
static int die_anchor(void)
{
  /* A DIE reading, not the package. The camera saw the package at 45.0 C in
     the passive state and the internal sensor read 72.0 - the 27 K between
     them is junction-to-case, and feeding the package number here was what
     made this check pass while the live board estimated itself 6.4 K above
     an NTC that cannot be below it. */
  const float truth = 72.0f;
  const float board = 30.0f;
  thermal_cfg_t cfg;
  thermal_t th;
  thermal_power_t p = { { 0.0f } };
  int bad = 0;

  thermal_defaults(&cfg);
  thermal_init(&th, &cfg, 60.0f);      /* deliberately wrong, 30 K out */
  th.ambient = AMBIENT;
  p.watt[THERMAL_MCU] = 0.666f;
  p.watt[THERMAL_REGULATORS] = 0.534f;

  for (float t = 0.0f; t < SETTLE_S; t += STEP_S)
  {
    const thermal_sense_t only_mcu = { NAN, NAN, truth };

    thermal_step(&th, &p, &only_mcu, STEP_S);
  }

  const float got = th.t[THERMAL_BOARD];
  const float want = truth - cfg.node[THERMAL_MCU].die_over_node
                     - 0.666f * cfg.node[THERMAL_MCU].to_board;

  printf("\nthe MCU die alone, started 30 K wrong and no NTC\n");
  printf("  board  %6.2f   implied by the die %6.2f   %+.2f K\n",
         got, want, got - want);
  printf("  settled %s   (a die anchors the board without the NTC)\n",
         th.settled ? "true" : "FALSE");

  if (fabsf(got - want) > 0.5f)
  {
    printf("  ^ the die did not pull the board to what it implies\n");
    bad++;
  }
  if (fabsf(got - board) > 6.0f)
  {
    printf("  ^ %.1f K from the camera's %.1f - the MCU K/W is the suspect\n",
           got - board, board);
  }
  return bad;
}


/* A deep burst: does the budget warn while there is still time to act?
 *
 * The question a burst asks is not "how hot is it" but "how long may I stay
 * here". So this drives a hard load from a cold board and checks that the
 * warning arrives with seconds still on the clock, not after the limit.
 */
static int burst_budget(void)
{
  thermal_cfg_t cfg;
  thermal_soa_t soa;
  thermal_t th;
  thermal_power_t p = { { 0.0f } };
  thermal_budget_t b;
  int bad = 0;

  thermal_defaults(&cfg);
  /* The envelope is the board's, out of its calibration record - there is
     no compiled-in copy to ask for. A test states what it is testing. */
  memset(&soa, 0, sizeof(soa));
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    soa.limit_c[i] = 125.0f;
  }
  soa.limit_c[THERMAL_BOARD] = 105.0f;
  soa.throttle_at = 0.85f;
  thermal_init(&th, &cfg, AMBIENT);
  th.ambient = AMBIENT;

  /* 100 A through one leg: the shunt alone is 35 W, which is the number that
     dwarfs everything the dry calibration ever saw. */
  p.watt[THERMAL_PHASE_U] = 35.0f;

  float warned_at = -1.0f, tripped_at = -1.0f, warned_left = 0.0f;

  for (float t = 0.0f; t < 600.0f; t += STEP_S)
  {
    const thermal_sense_t blind = { NAN, NAN, NAN };

    thermal_step(&th, &p, &blind, STEP_S);
    th.ambient = AMBIENT;
    thermal_budget(&th, &p, &soa, &b);

    if (b.throttling && (warned_at < 0.0f))
    {
      warned_at = t;
      warned_left = (float)b.millis_to_limit;
    }
    if (b.tripped && (tripped_at < 0.0f))
    {
      tripped_at = t;
    }
  }

  printf("\n35 W into the phases from a cold board\n");
  printf("  throttling at %6.1f s, %.0f ms still on the clock\n",
         warned_at, warned_left);
  printf("  tripped at    %6.1f s   phases %.1f C, board %.1f C\n",
         tripped_at, th.t[THERMAL_PHASE_U], th.t[THERMAL_BOARD]);

  if (warned_at < 0.0f)
  {
    printf("  ^ it never throttled - the budget is not watching\n");
    bad++;
  }
  else if ((tripped_at >= 0.0f) && (warned_at >= tripped_at))
  {
    printf("  ^ the warning came with the trip, which is no warning\n");
    bad++;
  }
  if ((warned_at >= 0.0f) && (warned_left <= 0.0f))
  {
    printf("  ^ warned with no time reported - a burst cannot plan on that\n");
    bad++;
  }
  return bad;
}

static int rds_tempco(void)
{
  /* The FET's on-resistance follows the node it heats. 100 A through one
     leg: at 25 C the FET makes 18 W beside the shunt's 35; at a 100 C
     phase node the datasheet chord says 1.585x that, and a NULL or NaN
     estimate keeps the flat figure - the pre-tempco behaviour, bit for
     bit. */
  thermal_loss_t loss;
  thermal_load_t load;
  thermal_power_t flat, hot, cold;
  int bad = 0;

  thermal_losses(&loss);
  memset(&load, 0, sizeof(load));
  load.phase_amps[0] = 100.0f;
  load.link_amps = 0.0f;

  const float at_100[3] = { 100.0f, 25.0f, 25.0f };
  const float frozen[3] = { -300.0f, 25.0f, 25.0f };

  thermal_power_estimate(&flat, &load, &loss, NULL);
  thermal_power_estimate(&hot, &load, &loss, at_100);
  thermal_power_estimate(&cold, &load, &loss, frozen);

  const float fet_flat = flat.watt[THERMAL_PHASE_U]
                         - 100.0f * 100.0f * loss.r_shunt;
  const float fet_hot = hot.watt[THERMAL_PHASE_U]
                        - 100.0f * 100.0f * loss.r_shunt;
  const float want = 1.0f + loss.rds_alpha * 75.0f;

  printf("\n100 A conduction, the FET share by phase-node temperature\n");
  printf("  25 C %6.2f W   100 C %6.2f W   ratio %.3f against %.3f asked\n",
         (double)fet_flat, (double)fet_hot,
         (double)(fet_hot / fet_flat), (double)want);

  if (fabsf(fet_hot / fet_flat - want) > 0.01f)
  {
    printf("  ^ the tempco is not the datasheet chord\n");
    bad++;
  }
  if (fabsf(fet_flat - 100.0f * 100.0f * loss.rds_on) > 0.01f)
  {
    printf("  ^ a NULL estimate no longer means the 25 C figure\n");
    bad++;
  }
  if (cold.watt[THERMAL_PHASE_U]
      < 100.0f * 100.0f * (0.5f * loss.rds_on + loss.r_shunt) - 0.01f)
  {
    printf("  ^ the cold floor did not hold - a garbage estimate can "
           "erase the loss\n");
    bad++;
  }
  return bad;
}


int main(void)
{
  int bad = 0;

  /* THE NTC COLUMNS NOW CARRY A RESIDUAL, and it is not a regression.
     The thermistor is an element between the leg node and the board
     since 2026-09-04, so it can no longer be hotter than what heats it,
     and the campaign's one switching state implies a fraction of 1.05 -
     which no passive body can have. The disagreement had been living in
     the coupling; it shows here instead. FINDINGS has which of the three
     inputs is the suspect. */
  printf("%-11s %9s %10s %8s   %9s %9s\n",
         "state", "board mod", "board meas", "err", "ntc mod", "ntc meas");

  for (size_t i = 0; i < sizeof(CASES) / sizeof(CASES[0]); i++)
  {
    const struct sample *s = &CASES[i];
    thermal_cfg_t cfg;
    thermal_t th;
    thermal_power_t p = { { 0.0f } };

    thermal_defaults(&cfg);
    thermal_init(&th, &cfg, AMBIENT);
    th.ambient = AMBIENT;

    for (int n = 0; n < THERMAL_NODES; n++)
    {
      p.watt[n] = s->watt[n];
    }

    /* No anchoring: this tests the open network, not the sensor correction.
       NAN on both makes thermal_step integrate only. */
    for (float t = 0.0f; t < SETTLE_S; t += STEP_S)
    {
      const thermal_sense_t blind = { NAN, NAN, NAN };

      thermal_step(&th, &p, &blind, STEP_S);
      th.ambient = AMBIENT;            /* the room does not drift */
    }

    const float board = th.t[THERMAL_BOARD];
    const float err = board - s->board_measured;
    const float ntc = thermal_expected_ntc(&th);

    printf("%-11s %9.2f %10.2f %+8.2f   %9.2f %9s\n",
           s->tag, board, s->board_measured, err, ntc,
           s->ntc_measured < 0.0f ? "-" : "");
    if (s->ntc_measured >= 0.0f)
    {
      printf("%-11s %31s   %9.2f %9.2f  (err %+.2f)\n",
             "", "", ntc, s->ntc_measured, ntc - s->ntc_measured);
    }

    if (fabsf(err) > 2.0f)
    {
      printf("   ^ board off by more than 2 K\n");
      bad++;
    }
  }

  printf("\n%s\n", bad ? "THE MODEL DOES NOT REPRODUCE ITS OWN CALIBRATION"
                       : "all four within 2 K");
  bad += die_anchor();
  bad += burst_budget();
  bad += rds_tempco();
  return bad ? 1 : 0;
}
