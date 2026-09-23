/** board/power.h - what the comms stack needs from board_power.c; included by board.h. */
#ifndef COMMS_BOARD_POWER_H
#define COMMS_BOARD_POWER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** What the board can see of the Safe Torque Off chain. */
typedef struct
{
  bool    afe_on;             /**< false makes both readings meaningless */
  bool    pilot_ok;           /**< the Cinj channel answered */
  int32_t pilot_raw;          /**< recovered pilot, raw code */
  int32_t pilot_microvolts;
  bool    level_ok;           /**< the Clevel channel answered */
  int32_t level_raw;          /**< integrator level - the margin left */
  int32_t level_microvolts;
  bool    stopped;            /**< TIM1 break latched: nFAULT on PE15 */
  uint32_t keepalive;         /**< edges pumped since boot - the loop rate */
  uint32_t worst_gap;         /**< longest gap between edges, CYCCNT ticks */
} board_sto_state_t;

void Board_StoState(board_sto_state_t *out);

/** One edge into the STO charge pump. Call from the main loop, unguarded. */
void Board_StoKeepalive(void);

/** Forget the worst gap seen so far, so a run can be measured on its own. */
void Board_StoKeepaliveReset(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_POWER_H */
