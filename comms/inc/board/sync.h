/** board/sync.h - what the comms stack needs from board_sync.c; included by board.h. */
#ifndef COMMS_BOARD_SYNC_H
#define COMMS_BOARD_SYNC_H

#include <stdbool.h>
#include <stdint.h>
#include "board/pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One simultaneous triple, latched by the injected end-of-sequence. */
typedef struct
{
  int16_t  phase[BOARD_PWM_PHASES];  /**< U, V, W, raw codes               */
  uint16_t at;                       /**< TIM1->CNT when it was latched    */
  uint32_t dcbus;                    /**< DC link, raw single-ended: rank 2 on ADC3 of the same sequence */
  uint32_t ntc;                      /**< the thermistor, rank 2 on ADC1: the thermal observer's thermometer
      while the drive holds the converters */
} board_sync_sample_t;

/** What the synced path is doing, for the command layer to report. */
typedef struct
{
  bool     ready;                    /**< timer and injected groups exist  */
  bool     armed;                    /**< triggering and latching          */
  uint32_t updates;                  /**< triples latched since arming     */
  uint32_t overruns;                 /**< sequences that arrived too soon  */
  uint16_t trigger;                  /**< CCR4 - the sample point, ticks   */
  board_sync_sample_t latest;
} board_sync_state_t;

/** Is there a timer to trigger from and an injected group to trigger? */
bool Board_SyncReady(void);

/** Start latching. False unless ready. Refuses the meter while armed. */
const char *Board_SyncArm(void);
void Board_SyncDisarm(void);
bool Board_SyncArmed(void);

/** The last triple, copied whole so no reader mixes two conversions. */
void Board_SyncLatest(board_sync_sample_t *out);

/** Mean of the squared phase current since the last call, A^2 a leg. */
bool Board_SyncMeanSquare(float *out);

/** Where in the PWM period the triple is taken, as CCR4 in timer ticks. */
bool Board_SyncSetTrigger(uint16_t ticks);
uint16_t Board_SyncTrigger(void);
void Board_SyncState(board_sync_state_t *out);

/** From the injected end-of-sequence callback, and from the overrun one. */
void Board_SyncOnInjected(const void *hadc);
void Board_SyncOverrun(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_SYNC_H */
