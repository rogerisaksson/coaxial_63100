/** board/ctrl.h - what the comms stack needs from board_ctrl.c; included by board.h. */
#ifndef COMMS_BOARD_CTRL_H
#define COMMS_BOARD_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "ctrl_loop.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The loop as it stands: the core's. */
typedef ctrl_loop_state_t board_ctrl_state_t;

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
