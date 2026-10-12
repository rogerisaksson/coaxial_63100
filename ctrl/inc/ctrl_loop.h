/** ctrl_loop.h - A board's loop on its drive: ctrl's runner ticked in the drive's sample. */
#ifndef CTRL_LOOP_H
#define CTRL_LOOP_H

#include "ctrl.h"
#include "drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/** What the loop measures: the shaft's angle, deg; the loop's estimate, rad/s; iq, A. */
#define CTRL_LOOP_ANGLE    0U
#define CTRL_LOOP_OMEGA    1U
#define CTRL_LOOP_IQ       2U
#define CTRL_LOOP_MEASURES 3U

/** Where its command goes: the drive's theta, rad; iq_ref, A. */
#define CTRL_LOOP_THETA    0U
#define CTRL_LOOP_IQ_REF   1U
#define CTRL_LOOP_COMMANDS 2U

/** Slots, in the order a pass runs them. */
#define CTRL_LOOP_SLOTS    4U

/** One loop: every executive's - the part's (board_ctrl.c), a stand-in's (world_drive.c). */
typedef struct
{
  ctrl_runner_t r;
  bool     running;
  bool     wired;
  uint8_t  measured;
  uint8_t  command;
  uint16_t divider;     /**< PWM periods a tick */
  uint16_t count;
  float    dt;
  uint32_t blind;       /**< ticks with no reading: the command stood */
} ctrl_loop_t;

/** The loop as it stands, its channels in their units. */
typedef struct
{
  bool     running;
  bool     playing;
  uint8_t  measured;
  uint8_t  command;
  uint16_t hz;          /**< the tick rate kept: the PWM rate over a whole divider */
  uint16_t rows;        /**< queued, the one playing not counted */
  uint16_t free;
  float    queued_s;    /**< the queued rows and what is left of the one playing */
  uint32_t played;      /**< rows finished */
  uint32_t idle;        /**< ticks with no row playing */
  uint32_t blind;
  float    setpoint;
  float    ref;
  float    value;
  float    estimate;
  float    out;
} ctrl_loop_state_t;

/** Once a PWM period, before the drive steps: whether a tick is due now. */
bool ctrl_loop_due(ctrl_loop_t *l);

/** The tick that is due: the measure off `d`, or the shaft's `angle_deg` (NAN: no reading,
    the command stands); the runner stepped; its command into `d`'s setpoints - a current
    within `d`'s clamp, a theta as the host's is taken, the held frame moved by its change. */
void ctrl_loop_step(ctrl_loop_t *l, drive_t *d, float angle_deg);

void ctrl_loop_state(const ctrl_loop_t *l, ctrl_loop_state_t *out);

/** A slot, 0 prefilter .. 3 regulator, as a kind and its parameters; stopped only. */
const char *ctrl_loop_slot(ctrl_loop_t *l, uint8_t slot, uint8_t kind, const float *params,
                           uint8_t n);

/** What the loop measures, where its command goes, how often it ticks on a drive of period
    `ts`; stopped only. */
const char *ctrl_loop_wire(ctrl_loop_t *l, float ts, uint8_t measured, uint8_t command,
                           uint16_t hz);

/** `n` rows onto the ring, all or none. */
const char *ctrl_loop_rows(ctrl_loop_t *l, const uint16_t *ms, const float *setpoint, uint8_t n);

/** On: the feedback reset, ticking. Off: stopped; the drive keeps the last command. Call
    where the tick cannot run. */
const char *ctrl_loop_run(ctrl_loop_t *l, bool on);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_LOOP_H */
