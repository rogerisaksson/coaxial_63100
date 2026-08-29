/**
  ******************************************************************************
  * @file    board_power.c
  * @brief   Reference-counted rails. The header carries the reasoning.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"
#include "board_power.h"
#include "link.h"

/* One bit per user in a uint8_t. A sixth user is free; a ninth is a silent
   truncation in bit_of and everything that reads the mask. */
_Static_assert(BOARD_USER_COUNT <= 8, "the users mask is one uint8_t");

/** One bitmask per rail. A bit is a user, so a leak names itself. */
static uint8_t s_users[BOARD_RAIL_COUNT];

/** When each hold runs out, HAL ticks. Zero means it never does - only the
  * host gets that, because only the host can be asked to give it back. */
static uint32_t s_expires[BOARD_RAIL_COUNT][BOARD_USER_COUNT];

static uint8_t bit_of(board_user_t user)
{
  return (uint8_t)(1U << (uint8_t)user);
}

static uint8_t count_bits(uint8_t mask)
{
  uint8_t n = 0U;

  while (mask != 0U)
  {
    mask = (uint8_t)(mask & (uint8_t)(mask - 1U));
    n++;
  }
  return n;
}

/** Whether taking @p rail now would drop something that must not be dropped.
  *
  * AFE_ON high removes the gate drivers' supply. With the stage armed that
  * leaves six driver inputs switching into unpowered drivers, so the answer
  * is no while MOE is set - the measurement waits, the power stage does not.
  */
static bool blocked(board_rail_t rail)
{
  return (rail == BOARD_RAIL_AFE) && Board_PwmIsEnabled();
}

static void apply(board_rail_t rail)
{
  if (rail == BOARD_RAIL_AFE)
  {
    Board_SetAfeOn(s_users[rail] != 0U);
  }
}

bool Board_PowerAcquire(board_rail_t rail, board_user_t user)
{
  if ((rail >= BOARD_RAIL_COUNT) || (user >= BOARD_USER_COUNT))
  {
    return false;
  }

  const uint8_t bit = bit_of(user);
  const bool had = (s_users[rail] & bit) != 0U;

  /* Only a NEW hold can be refused. Renewing one already granted must not
     fail because the stage armed meanwhile - that would strand the owner
     holding a rail it can no longer keep alive. */
  if (!had && blocked(rail))
  {
    return false;
  }

  s_users[rail] |= bit;

  /* Zero is the sentinel for "never expires", so a lease that lands exactly
     on it would never be collected - a 3 s window once every 49.7 days where
     a leaked hold becomes permanent. Step past it. */
  uint32_t at = HAL_GetTick() + BOARD_POWER_LEASE_MS;

  if (at == 0U)
  {
    at = 1U;
  }
  s_expires[rail][user] = (user == BOARD_USER_HOST) ? 0U : at;
  apply(rail);
  return true;
}

bool Board_PowerRelease(board_rail_t rail, board_user_t user)
{
  if ((rail >= BOARD_RAIL_COUNT) || (user >= BOARD_USER_COUNT))
  {
    return false;
  }

  s_users[rail] &= (uint8_t)~bit_of(user);
  s_expires[rail][user] = 0U;
  apply(rail);
  return s_users[rail] == 0U;
}

bool Board_PowerHolds(board_rail_t rail, board_user_t user)
{
  if ((rail >= BOARD_RAIL_COUNT) || (user >= BOARD_USER_COUNT))
  {
    return false;
  }
  return (s_users[rail] & bit_of(user)) != 0U;
}

bool Board_PowerState(board_rail_t rail, board_rail_state_t *out)
{
  if ((rail >= BOARD_RAIL_COUNT) || (out == NULL))
  {
    return false;
  }

  /* The pin, read back. What the count says should be true of it is exactly
     the thing worth catching when it is not. */
  out->on = (rail == BOARD_RAIL_AFE) ? Board_AfeOn() : false;
  out->users = s_users[rail];
  out->count = count_bits(s_users[rail]);
  out->blocked = blocked(rail);

  out->leased = 0U;
  for (uint8_t user = 0U; user < (uint8_t)BOARD_USER_COUNT; user++)
  {
    if (s_expires[rail][user] != 0U)
    {
      out->leased |= bit_of((board_user_t)user);
    }
  }
  return true;
}

void Board_PowerPoll(void)
{
  const uint32_t now = HAL_GetTick();

  /* THE HOST'S HOLDS DIE WITH THE HOST. Its reference is deliberately
     unleased so a session can keep a rail as long as it likes - and that is
     exactly why a killed script left AFE_ON high until somebody noticed.
     A live host talks; silence is the board's evidence that this one does
     not, and the rail goes down on its own. */
  static uint32_t seen_count;
  static uint32_t seen_at;
  const uint32_t heard = link_rx_count();

  if (heard != seen_count)
  {
    seen_count = heard;
    seen_at = now;
  }

  if ((uint32_t)(now - seen_at) > BOARD_POWER_HOST_QUIET_MS)
  {
    for (uint8_t rail = 0U; rail < (uint8_t)BOARD_RAIL_COUNT; rail++)
    {
      if ((s_users[rail] & bit_of(BOARD_USER_HOST)) != 0U)
      {
        s_users[rail] &= (uint8_t)~bit_of(BOARD_USER_HOST);
        s_expires[rail][BOARD_USER_HOST] = 0U;
        apply((board_rail_t)rail);
      }
    }
  }

  for (uint8_t rail = 0U; rail < (uint8_t)BOARD_RAIL_COUNT; rail++)
  {
    for (uint8_t user = 0U; user < (uint8_t)BOARD_USER_COUNT; user++)
    {
      const uint32_t at = s_expires[rail][user];

      /* Signed difference, so the tick wrap costs nothing - the same
         arithmetic the RTU timer uses, and for the same reason. */
      if ((at != 0U) && ((int32_t)(now - at) >= 0))
      {
        s_users[rail] &= (uint8_t)~bit_of((board_user_t)user);
        s_expires[rail][user] = 0U;
        apply((board_rail_t)rail);
      }
    }
  }
}


void Board_PowerReleaseAll(void)
{
  for (uint8_t rail = 0U; rail < (uint8_t)BOARD_RAIL_COUNT; rail++)
  {
    s_users[rail] = 0U;
    for (uint8_t user = 0U; user < (uint8_t)BOARD_USER_COUNT; user++)
    {
      s_expires[rail][user] = 0U;
    }
    apply((board_rail_t)rail);
  }
}
