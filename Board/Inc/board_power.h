/**
  ******************************************************************************
  * @file    board_power.h
  * @brief   Who is using a rail, and switching it off when nobody is.
  *
  * A rail is switched by whoever needs it and switched back when the last
  * user lets go - a reference count, not a boolean. The boolean is what
  * caused the problem this exists to fix: two subsystems both wanting AFE_ON,
  * the first one finishing and switching it off under the second.
  *
  * USERS ARE NAMED, and that is the point of the enum rather than a bare
  * count. A count that fails to reach zero says only "something is holding
  * it"; a bitmask says which, and `0x6E` device 9 reports it. A leaked hold
  * is otherwise invisible until an unrelated measurement comes back wrong.
  *
  * THE AFE RAIL IS INVERTED AND SHARED WITH THE GATE DRIVERS. AFE_ON high
  * powers the analog front end, the reference, the NTC, the A1335 and the
  * BNO08X - and takes the supply *away* from the gate drivers. So a hold on
  * this rail is not free while the stage is armed, and `Board_PowerAcquire`
  * refuses it there rather than dropping six gate drivers mid-PWM.
  *
  * A BORROWED HOLD IS A LEASE, NOT A HOLD. Measured 2026-08-28: the observer
  * took the rail, the host then talked hard enough that `link_busy()` kept
  * `Board_ThermalPoll` from running, and AFE_ON stayed high indefinitely -
  * the release lived in the same starved poll as the acquire. So every hold
  * but the host's expires on its own, and `Board_PowerPoll` runs beside the
  * STO keepalive where nothing gates it.
  *
  * The host's hold does NOT expire. It was asked for by name over the wire
  * and only the wire takes it back; a rail that switched itself off under a
  * host that asked for it is the same bug from the other end.
  ******************************************************************************
  */
#ifndef BOARD_POWER_H
#define BOARD_POWER_H

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

/** How long a borrowed hold lasts without renewal, milliseconds.
  *
  * Long enough for the observer's 300 ms settle several times over, short
  * enough that a starved poll costs one lease and not a session. */
#define BOARD_POWER_LEASE_MS 3000U

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
  * Idempotent per user, and a second acquire RENEWS the lease rather than
  * counting twice - so a poll that acquires every pass keeps its hold alive
  * without tracking its own calls. Refused, and false, while the gate stage
  * is armed and the rail is BOARD_RAIL_AFE - see the file banner.
  */
bool Board_PowerAcquire(board_rail_t rail, board_user_t user);

/**
  * @brief  Expire any lease that has run out. Call from the main loop.
  *
  * Belongs beside `Board_StoKeepalive`, OUTSIDE every branch: it is the
  * thing that recovers a hold whose owner stopped running, so gating it on
  * the same condition that starved the owner would defeat it.
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
  * @brief  Drop every hold on every rail and switch them off.
  *
  * For the host to recover a leaked hold without a power cycle. It is a
  * blunt instrument: anything mid-measurement loses its supply.
  */
void Board_PowerReleaseAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_POWER_H */
