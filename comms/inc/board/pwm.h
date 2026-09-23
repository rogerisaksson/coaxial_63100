/** board/pwm.h - what the comms stack needs from board_pwm.c; included by board.h. */
#ifndef COMMS_BOARD_PWM_H
#define COMMS_BOARD_PWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Half bridges on this board, and compare registers on TIM1. */
#define BOARD_PWM_PHASES 3U

/** What the gate drivers are doing, for the command layer to report verbatim. */
typedef struct
{
  bool     ready;                    /**< TIM1 clocked and given a period  */
  bool     enabled;                  /**< master output enable is set      */
  bool     fault;                    /**< break latched - see PE15/BKIN    */
  uint32_t period;                   /**< ARR + 1, in timer ticks          */
  uint8_t  deadtime;                 /**< BDTR DTG, raw - not nanoseconds  */
  uint16_t duty[BOARD_PWM_PHASES];   /**< compare ticks, as last accepted  */
  bool     bypassed;                 /**< BDTR.BKE cleared - break ignored */
  uint8_t  pins;                     /**< PE8..PE13 as one IDR read: bit 0 UL, 1 UH, 2 VL, 3 VH, 4 WL, 5 WH */
  uint16_t at;                       /**< TIM1->CNT beside that read, so a host knows where in the period */
} board_pwm_state_t;

bool Board_PwmInit(void);

/** Dead time at runtime, in nanoseconds. */
const char *Board_PwmSetDeadTime(uint32_t ns);
uint32_t Board_PwmDeadTimeNs(void);

/** DTG counts the smallest dead time can be, at this timer clock. */
uint8_t Board_PwmDeadTimeFloor(void);

/** Trim for a bridge whose two transitions are not symmetric. */
const char *Board_PwmSetDeadTimeSkew(int8_t counts);
int8_t Board_PwmDeadTimeSkew(void);

/** Has TIM1 been configured at all? False until MX_TIM1_Init exists. */
bool Board_PwmReady(void);

/** ARR + 1, or 0 when the timer is not configured. */
uint32_t Board_PwmPeriod(void);

/** Arm the outputs, at zero duty. False if not ready or a break is latched. */
bool Board_PwmEnable(void);

/** Drop every gate. The one call that works whatever else is true. */
void Board_PwmDisable(void);

bool Board_PwmIsEnabled(void);

/** Is the break latched? It is nFAULT arriving through TIM1_BKIN. */
bool Board_PwmFault(void);

/** Disconnect TIM1's break input, for bench work with the gate drivers
    unpowered. */
bool Board_PwmSetBreakBypass(bool on);
bool Board_PwmBreakBypassed(void);

/** The silent host's stage cleanup: MOE down, the break bypass back in
    force. */
void Board_PwmSessionDrop(void);

/** Clear the break latch. Does NOT re-arm - the caller must ask again. */
bool Board_PwmClearFault(void);

/** Which legs have their two gate pins joined, bit 0 = U, 1 = V, 2 = W. */
uint8_t Board_PwmGateShorts(void);

/** All three, or none: never a cycle built from two calls. */
const char *Board_PwmSetAll(const uint16_t *ticks);

/** The same triple held for exactly `periods` PWM periods, then zeroed by
    TIM1's update interrupt - 10 ms asked for is 500 periods, not the link's
    93-108 ms. */
const char *Board_PwmSetAllCounted(const uint16_t *ticks, uint32_t periods);

/** Periods left of a counted hold, 0 when free-running or expired. */
uint32_t Board_PwmPeriodsLeft(void);

/** Duty in ticks Q16.16, dithered so the MEAN is what was asked for. */
const char *Board_PwmSetAllFine(const uint32_t *ticks_q16);

/** Two compare triples, A one PWM period and B the next, swapped by the
    update interrupt at every overflow so each lands - preloaded - at the
    underflow and owns a whole period. */
const char *Board_PwmSetAlternate(const uint16_t *a, const uint16_t *b);
void Board_PwmDutyRequested(uint32_t *ticks_q16);
void Board_PwmDitherStep(void);

/** The drive's hold on the compares. */
void Board_PwmDriveOwn(bool on);
void Board_PwmSetNext(const uint16_t *ticks);

uint16_t Board_PwmGetDuty(uint8_t phase);

void Board_PwmState(board_pwm_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_PWM_H */
