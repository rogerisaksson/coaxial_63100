/**
  ******************************************************************************
  * @file    board_power.h
  * @brief   Who is using a rail, and switching it off when nobody is.
  *
  * A count, not a boolean: two subsystems both wanted AFE_ON and the first to
  * finish switched it off under the second. Users are NAMED so a leak says
  * which one.
  *
  * AFE_ON IS INVERTED and shared with the gate drivers: high powers the front
  * end, the reference, the NTC, the A1335 and the BNO08X, and takes the
  * drivers' supply away. Acquire is refused while the stage is armed.
  ******************************************************************************
  */
#ifndef BOARD_POWER_H
#define BOARD_POWER_H

#include "board_limits.h"   /* BOARD_POWER_LEASE_MS */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The rails this board can switch. One so far; the shape is what matters. */
typedef enum
{
  BOARD_RAIL_AFE = 0,
  BOARD_RAIL_COUNT
} board_rail_t;

/** Who is holding a rail. One bit each, so a leak names itself. */
typedef enum
{
  BOARD_USER_HOST    = 0, /**< the host asked, over 0x6D or 0x6E          */
  BOARD_USER_THERMAL = 1, /**< the observer, for a sample between steps   */
  BOARD_USER_IMU     = 2, /**< the BNO08X poll                            */
  BOARD_USER_ANGLE   = 3, /**< the A1335 poll                             */
  BOARD_USER_DAQ     = 4, /**< a running acquisition                      */
  BOARD_USER_COUNT
} board_user_t;

/** What a rail is doing and who put it there. */
typedef struct
{
  bool    on;        /**< the pin, read back rather than remembered */
  uint8_t users;     /**< bitmask of board_user_t                   */
  uint8_t count;     /**< how many bits are set                     */
  bool    blocked;   /**< an acquire would be refused right now     */
  uint8_t leased;    /**< bitmask of the holds that expire          */
} board_rail_state_t;

/**
  * @brief  Take a hold on a rail, switching it on if nobody had it.
  * @param  rail  which rail
  * @param  user  who is asking
  * @return true if the hold is now held.
  *
  * Idempotent per user; a second acquire renews the lease. Refused while the
  * gate stage is armed and the rail is BOARD_RAIL_AFE.
  */
bool Board_PowerAcquire(board_rail_t rail, board_user_t user);

/**
  * @brief  Expire any lease that has run out. Call from the main loop.
  *
  * Beside `Board_StoKeepalive`, outside every branch: measured 2026-08-28,
  * `link_busy()` starved the poll that held the release and AFE_ON stayed
  * high until reset. Every hold but the host's expires on its own.
  */
void Board_PowerPoll(void);

/**
  * @brief  Let go. The rail switches off when the last user does.
  * @return true if the rail is now off.
  */
bool Board_PowerRelease(board_rail_t rail, board_user_t user);

/** Whether @p user currently holds @p rail. */
bool Board_PowerHolds(board_rail_t rail, board_user_t user);

/** Fill @p out with what the rail is doing. False if @p rail is not one. */
bool Board_PowerState(board_rail_t rail, board_rail_state_t *out);

/**
  * @brief  Drop every hold and switch the rails off.
  *
  * Blunt: anything mid-measurement loses its supply. It exists so a leaked
  * hold needs no power cycle.
  */
void Board_PowerReleaseAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_POWER_H */
