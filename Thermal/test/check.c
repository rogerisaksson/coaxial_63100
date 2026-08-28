/* Run the observer against the campaign's four measured states.
 *
 * Build and run with the host gcc, no hardware:
 *   gcc -std=c11 -Wall -Wextra -Wconversion -I../Inc ../Src/thermal.c check.c
 *       -lm -o check && ./check
 *
 * This is not the arithmetic in thermal.c - it tests that the model with
 * those parameters actually lands on the temperatures it was calibrated from.
 * A network can have the right resistances and still not converge.
 */
#include "thermal.h"

#include <math.h>
#include <stdio.h>

#define AMBIENT   20.0f
#define SETTLE_S  (60.0f * 60.0f)      /* en timme, ~9 tau */
#define STEP_S    0.5f

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

int main(void)
{
  int bad = 0;

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
      thermal_step(&th, &p, NAN, NAN, STEP_S);
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
  return bad ? 1 : 0;
}
