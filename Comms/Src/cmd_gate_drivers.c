/**
  ******************************************************************************
  * @file    cmd_gate_drivers.c
  * @brief   The gate drivers' operations behind command 0x6E, device 4.
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
  * @brief op 0 - the gate drivers, the triple and the STO chain, one sample.
  *
  * Flags first so a reader that only wants "is it running" stops after one
  * byte. `at` is TIM1->CNT when the triple was latched, which is what makes
  * the sample point measurable rather than assumed: move `trigger` and `at`
  * moves with it.
  */
static cmd_status_t h_gate_drivers_state(wr_t *out)
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
  /* What was asked for, beside what the register holds this period. With
     the dither running the two differ by a tick most of the time, and a
     caller comparing them would otherwise think it had been rounded.
     Appended, like flags2 and for the same reason: moving an offset breaks
     every decoder for a field most of them do not read. */
  uint32_t wanted[BOARD_PWM_PHASES];
  Board_PwmDutyRequested(wanted);
  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    wr_u32(out, wanted[i]);
  }

  /* The six gate signals as one instant, and the counter beside them. A
     host asking six times would get six instants and could see a leg with
     both FETs on, which is the one state the dead time exists to prevent.
     At the end, because putting it before `requested` shifted every offset
     after it - which is the same mistake this file already carries a
     warning about, made again. */
  wr_u8(out, pwm.pins);
  wr_u16(out, pwm.at);

  /* The dead time in nanoseconds beside the raw DTG above, its skew, and
     the smallest DTG this timer clock allows. Appended. */
  wr_u32(out, Board_PwmDeadTimeNs());
  wr_u8(out, (uint8_t)Board_PwmDeadTimeSkew());
  wr_u8(out, Board_PwmDeadTimeFloor());

  /* Which legs have their two gate pins on one node - bit 0 U, 1 V, 2 W.
     A joined pair cannot be driven complementary, so the leg never
     switches and its driver sees a level, not PWM. Measured rather than
     assumed, and appended like everything above it. Reads 0 while armed,
     because the probe needs the pins. */
  wr_u8(out, Board_PwmGateShorts());

  return wr_ok(out) ? CMD_OK : CMD_ERR_DEVICE;
}


/** op 1 - master output enable. Enabling always arms at zero duty. */
static cmd_status_t h_gate_drivers_pwm(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = NULL;

  if (on != 0U)
  {
    refusal = Board_PwmEnable()
                ? NULL
                : "the gate drivers would not enable - a latched break outranks "
                  "the request, and clearing it does not help while nFAULT "
                  "is low; bypass the break for bench work";
  }
  else
  {
    Board_PwmDisable();
  }

  cmd_took(out, refusal);
  return CMD_OK;
}


/** op 2 - all three compares, or none. A half update is a step nobody asked
    for, so the board takes the triple together or refuses it. */
static cmd_status_t h_gate_drivers_duty(rd_t *in, wr_t *out)
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

  cmd_took(out, Board_PwmSetAll(ticks));
  return CMD_OK;
}


/** op 10 - two triples, A one period and B the next, swapped by TIM1's
    update interrupt for as long as they stand. What a host cannot do at
    one write per 15 ms: a phase pair driven back and forth every 20 us. */
static cmd_status_t h_gate_drivers_alternate(rd_t *in, wr_t *out)
{
  uint16_t a[BOARD_PWM_PHASES];
  uint16_t b[BOARD_PWM_PHASES];

  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    a[i] = rd_u16(in);
  }
  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    b[i] = rd_u16(in);
  }
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  cmd_took(out, Board_PwmSetAlternate(a, b));
  return CMD_OK;
}


/** op 8 - all three compares in ticks Q16.16, dithered.
  *
  * One tick of ARR 2375 is 0.0421 % of duty, so an asked-for 34.54 % is
  * 820.32 ticks and neither 820 nor 821 is it. This keeps the fraction and
  * a first-order sigma-delta in TIM1's update interrupt pays it back, so
  * the MEAN duty is what was asked for rather than the nearest tick.
  *
  * All three or none, for the same reason op 2 is.
  */
static cmd_status_t h_gate_drivers_dutyq(rd_t *in, wr_t *out)
{
  uint32_t ticks[BOARD_PWM_PHASES];

  for (uint8_t i = 0U; i < BOARD_PWM_PHASES; i++)
  {
    ticks[i] = rd_u32(in);
  }
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  cmd_took(out, Board_PwmSetAllFine(ticks));
  return CMD_OK;
}


/** op 3 - start or stop latching the injected triple. Arming takes the
    converters away from the meter; disarming gives them back. */
static cmd_status_t h_gate_drivers_sync(rd_t *in, wr_t *out)
{
  const uint8_t on = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  const char *refusal = NULL;

  if (on != 0U)
  {
    refusal = Board_SyncArm();
  }
  else
  {
    Board_SyncDisarm();
  }

  cmd_took(out, refusal);
  return CMD_OK;
}


/** op 4 - move the sample point. Replies with CCR4 as it reads back, which
    is the only answer worth having: asking for a tick past ARR changes
    nothing and the reply says so. */
static cmd_status_t h_gate_drivers_trigger(rd_t *in, wr_t *out)
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
static cmd_status_t h_gate_drivers_gapreset(wr_t *out)
{
  Board_StoKeepaliveReset();
  wr_u8(out, 1U);
  return CMD_OK;
}


/** op 6 - disconnect the break input, for bench work. Loud on purpose: it
    shows in the state reply, and a reset puts it back. */
static cmd_status_t h_gate_drivers_bypass(rd_t *in, wr_t *out)
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
static cmd_status_t h_gate_drivers_clear(wr_t *out)
{
  wr_u8(out, Board_PwmClearFault() ? 1U : 0U);
  return CMD_OK;
}


/** op 9 - the dead time, in nanoseconds, and its skew in DTG counts.
  *
  * Both in one op because they constrain each other: a skew is only legal
  * against a dead time big enough to carry it, so setting them apart means
  * an order that works and an order that does not. The board floors the
  * dead time at 20 ns and refuses a skew that would take either half under
  * it, in its own words.
  */
static cmd_status_t h_gate_drivers_deadtime(rd_t *in, wr_t *out)
{
  const uint32_t ns = rd_u32(in);
  const int8_t skew = (int8_t)rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  /* Skew to zero first: the new dead time is checked against the skew that
     will be in force, not the one being replaced. */
  (void)Board_PwmSetDeadTimeSkew(0);

  const char *refusal = Board_PwmSetDeadTime(ns);

  if (refusal == NULL)
  {
    refusal = Board_PwmSetDeadTimeSkew(skew);
  }

  cmd_took(out, refusal);
  wr_u32(out, Board_PwmDeadTimeNs());
  wr_u8(out, (uint8_t)Board_PwmDeadTimeSkew());
  wr_u8(out, Board_PwmDeadTimeFloor());
  return CMD_OK;
}


cmd_status_t cmd_gate_drivers_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case GATEDRIVERS_OP_STATE:   return h_gate_drivers_state(out);
    case GATEDRIVERS_OP_PWM:     return h_gate_drivers_pwm(in, out);
    case GATEDRIVERS_OP_DUTY:    return h_gate_drivers_duty(in, out);
    case GATEDRIVERS_OP_SYNC:    return h_gate_drivers_sync(in, out);
    case GATEDRIVERS_OP_TRIGGER: return h_gate_drivers_trigger(in, out);
    case GATEDRIVERS_OP_CLEAR:   return h_gate_drivers_clear(out);
    case GATEDRIVERS_OP_BYPASS:  return h_gate_drivers_bypass(in, out);
    case GATEDRIVERS_OP_GAPRST:  return h_gate_drivers_gapreset(out);
    case GATEDRIVERS_OP_DUTYQ:   return h_gate_drivers_dutyq(in, out);
    case GATEDRIVERS_OP_DEADTIME: return h_gate_drivers_deadtime(in, out);
    case GATEDRIVERS_OP_ALTERNATE: return h_gate_drivers_alternate(in, out);
    default:                return CMD_ERR_VALUE;
  }
}
