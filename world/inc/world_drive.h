/** world_drive.h - A stand-in board's drive: the board's law and loop on the motor it models. */
#ifndef WORLD_DRIVE_H
#define WORLD_DRIVE_H

#include "ctrl_loop.h"
#include "drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The stand-in (coaxial.simulated.drive) runs the law the board runs (drive.c) on the motor the
   board models (drive_model.c), its loop (ctrl_loop.c) ticked in the same sample: one array of
   slots, a run of periods a call, the host's Python its edge - the record, the setpoints, the
   stage, the reads. */

/** The stand-ins a process holds (WORLD_STANDS). */
#define WORLD_DRIVES 64

/** The read's row, in order (`DRIVE_ROW`, tools.cores.stand): the law's periods, the periods
    it tripped in, whether its injection has two whole cycles, whether the bridge switched last
    period, that period's three duties, the phases' amps at its sample, the shaft rad. */
#define WORLD_DRIVE_ROW 11

/** The loop's row (`LOOP_ROW`): ctrl_loop_state_t's fields in their order. */
#define WORLD_DRIVE_LOOP_ROW 16

typedef struct
{
  drive_t law;              /**< the board's: modes, loop, observers; its model the motor */
  ctrl_loop_t loop;         /**< device 12: a feedback ticked before the law's step */
  double shaft;             /**< rad mechanical, unwrapped */
  float theta_was;          /**< the rotor's electrical angle the period before */
  float duty[DRIVE_PHASES]; /**< the last period's compares, 0..1 */
  bool driven;              /**< and whether the bridge switched on them */
  uint32_t trips;
} world_drive_t;

/** Slot `i` fresh at a period of `ts` s: the law at its defaults, the rotor at rest. */
void world_drive_init(int i, float ts);

/** Slot `i`'s law, for the harness's named access (drive/test/harness.c); NULL where none. */
drive_t *world_drive_law(int i);

/** Slot `i`'s loop, for ctrl_loop's own calls; NULL where none. */
ctrl_loop_t *world_drive_loop(int i);

/** The rotor back at theta0, at rest, its shaft where it stood (drive_model_init). */
void world_drive_rest(int i);

/** `periods` of the law on its motor - sampled, stepped, advanced on the step before's duties
    (drive_step_virtual). On the converters' source the stage is the board's: MOE as `moe`
    tells the law, and with it down or its drivers not `supplied` the rotor coasts on an open
    bridge. How many of them tripped. */
uint32_t world_drive_run(int i, uint32_t periods, bool moe, bool supplied);

/** Slot `i`'s row into `out` (WORLD_DRIVE_ROW); how many. */
int world_drive_read(int i, double *out);

/** Slot `i`'s loop as it stands into `out` (WORLD_DRIVE_LOOP_ROW); how many. */
int world_drive_loop_read(int i, double *out);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_DRIVE_H */
