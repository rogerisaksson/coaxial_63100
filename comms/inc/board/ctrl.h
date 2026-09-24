/** board/ctrl.h - what the comms stack needs from board_ctrl.c; included by board.h. */
#ifndef COMMS_BOARD_CTRL_H
#define COMMS_BOARD_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What the loop measures: the A1335's angle, deg; the observer's speed, rad/s; iq, A. */
#define BOARD_CTRL_ANGLE    0U
#define BOARD_CTRL_OMEGA    1U
#define BOARD_CTRL_IQ       2U
#define BOARD_CTRL_MEASURES 3U

/** Where its command goes: the drive's theta, rad; iq_ref, A. */
#define BOARD_CTRL_THETA    0U
#define BOARD_CTRL_IQ_REF   1U
#define BOARD_CTRL_COMMANDS 2U

/** Slots, in the order a pass runs them. */
#define BOARD_CTRL_SLOTS    4U

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
  uint32_t blind;       /**< ticks with no reading: the command stood */
  float    setpoint;
  float    ref;
  float    value;
  float    estimate;
  float    out;
} board_ctrl_state_t;

void Board_CtrlState(board_ctrl_state_t *out);

/** A slot, 0 prefilter .. 3 regulator, as a kind and its parameters; stopped only. */
const char *Board_CtrlSlot(uint8_t slot, uint8_t kind, const float *params, uint8_t n);

/** What the loop measures, where its command goes, how often it ticks; stopped only. */
const char *Board_CtrlWire(uint8_t measured, uint8_t command, uint16_t hz);

/** `n` rows onto the ring, all or none. */
const char *Board_CtrlRows(const uint16_t *ms, const float *setpoint, uint8_t n);

/** Every queued row dropped; the setpoint held. */
void Board_CtrlClear(void);

/** On: the feedback reset, ticking. Off: stopped; the drive keeps the last command. */
const char *Board_CtrlRun(bool on);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_CTRL_H */
