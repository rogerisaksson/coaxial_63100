/**
  ******************************************************************************
  * @file    cmd_bridge.c
  * @brief   The bridge's operations behind command 0x6E, device 4.
  *
  * TIM1, the synced phase triple and the Safe Torque Off chain answer as one
  * device because a caller tuning the sample point needs all three in the
  * same breath: where the trigger sits, what came back, and whether the
  * chain still holds. Three round trips would sample three different
  * moments.
  *
  * Nothing here judges. Op 0 reports registers and raw codes; the ops that
  * write refuse only what the hardware cannot do - a duty past ARR, a
  * trigger past ARR, an arm with no timer - and never because a number
  * looked wrong. Invariant 10.
  ******************************************************************************
  */
#include "cmd.h"
#include "board.h"
#include "wire.h"

/**
  * @brief op 0 - the bridge, the triple and the STO chain, one sample.
  *
  * Flags first so a reader that only wants "is it running" stops after one
  * byte. `at` is TIM1->CNT when the triple was latched, which is what makes
  * the sample point measurable rather than assumed: move `trigger` and `at`
  * moves with it.
  */
static cmd_status_t h_bridge_state(wr_t *out)
{
  board_pwm_state_t pwm;
  board_sync_state_t sync;
  board_sto_state_t sto;

  Board_PwmState(&pwm);
  Board_SyncState(&sync);
  Board_StoState(&sto);

  wr_u8(out, (uint8_t)((pwm.ready    ? 0x01U : 0U)
                     | (pwm.enabled  ? 0x02U : 0U)
                     | (pwm.fault    ? 0x04U : 0U)
                     | (sync.ready   ? 0x08U : 0U)
                     | (sync.armed   ? 0x10U : 0U)
                     | (sto.afe_on   ? 0x20U : 0U)
                     | (sto.pilot_ok ? 0x40U : 0U)
                     | (sto.level_ok ? 0x80U : 0U)));
  wr_u16(out, (uint16_t)pwm.period);
  wr_u8(out, pwm.deadtime);
  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    wr_u16(out, pwm.duty[i]);
  }
  wr_u16(out, sync.trigger);
  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    wr_i16(out, sync.latest.phase[i]);
  }
  wr_u16(out, sync.latest.at);
  wr_u32(out, sync.updates);
  wr_u32(out, sync.overruns);
  wr_u32(out, sto.keepalive);
  wr_u32(out, sto.worst_gap);
  wr_i32(out, sto.pilot_raw);
  wr_i32(out, sto.pilot_microvolts);
  wr_i32(out, sto.level_raw);
  wr_i32(out, sto.level_microvolts);
  /* Appended, not squeezed into the first byte: that one is full, and
     moving any offset would break every decoder for one bit. */
  wr_u8(out, pwm.bypassed ? 0x01U : 0x00U);

  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 1 - master output enable. Enabling always arms at zero duty. */
static cmd_status_t h_bridge_pwm(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  bool ok;
  if (on != 0U)
  {
    ok = Board_PwmEnable();
  }
  else
  {
    Board_PwmDisable();
    ok = true;
  }

  wr_u8(out, ok ? 1U : 0U);
  return CMD_OK;
}


/** op 2 - all three compares, or none. A half update is a step nobody asked
    for, so the board takes the triple together or refuses it. */
static cmd_status_t h_bridge_duty(rd_t *in, wr_t *out)
{
  uint16_t ticks[BOARD_PWM_PHASES];

  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    ticks[i] = rd_u16(in);
  }
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  wr_u8(out, Board_PwmSetAll(ticks) ? 1U : 0U);
  return CMD_OK;
}


/** op 3 - start or stop latching the injected triple. Arming takes the
    converters away from the meter; disarming gives them back. */
static cmd_status_t h_bridge_sync(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  bool ok;
  if (on != 0U)
  {
    ok = Board_SyncArm();
  }
  else
  {
    Board_SyncDisarm();
    ok = true;
  }

  wr_u8(out, ok ? 1U : 0U);
  return CMD_OK;
}


/** op 4 - move the sample point. Replies with CCR4 as it reads back, which
    is the only answer worth having: asking for a tick past ARR changes
    nothing and the reply says so. */
static cmd_status_t h_bridge_trigger(rd_t *in, wr_t *out)
{
  const uint16_t ticks = rd_u16(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  (void)Board_SyncSetTrigger(ticks);
  wr_u16(out, Board_SyncTrigger());
  return CMD_OK;
}


/** op 7 - forget the worst keepalive gap, so a run is measured on its own. */
static cmd_status_t h_bridge_gapreset(wr_t *out)
{
  Board_StoKeepaliveReset();
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 6 - disconnect the break input, for bench work. Loud on purpose: it
    shows in the state reply, and a reset puts it back. */
static cmd_status_t h_bridge_bypass(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  wr_u8(out, Board_PwmSetBreakBypass(on != 0U) ? 1U : 0U);
  return CMD_OK;
}


/** op 5 - clear the break latch. Does not re-arm; the caller asks again. */
static cmd_status_t h_bridge_clear(wr_t *out)
{
  wr_u8(out, Board_PwmClearFault() ? 1U : 0U);
  return CMD_OK;
}


cmd_status_t cmd_bridge_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case BRIDGE_OP_STATE:   return h_bridge_state(out);
    case BRIDGE_OP_PWM:     return h_bridge_pwm(in, out);
    case BRIDGE_OP_DUTY:    return h_bridge_duty(in, out);
    case BRIDGE_OP_SYNC:    return h_bridge_sync(in, out);
    case BRIDGE_OP_TRIGGER: return h_bridge_trigger(in, out);
    case BRIDGE_OP_CLEAR:   return h_bridge_clear(out);
    case BRIDGE_OP_BYPASS:  return h_bridge_bypass(in, out);
    case BRIDGE_OP_GAPRST:  return h_bridge_gapreset(out);
    default:                return CMD_ERR_VALUE;
  }
}
