/** cmd_aero.c - The frame's core (aero/) laid and stepped a pass: 0x6E, device 14. */
#include "board.h"
#include "cmd.h"
#include "wire.h"
#include "aero.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/** The wire's units: a millionth of a field's own, a thousandth of a weight's and a rate's, a
    billionth of an inertia's; the most a field's millionths say. */
#define MICRO 1e-6
#define MILLI 1e-3
#define NANO  1e-9
#define MOST  2147.0

/** A reading's fields before its pass's seconds: the frame's axes, its rates, its speed, its
    height, each rotor's thrust as given, at its least and at its most. */
#define AERO_FIELDS 28U

_Static_assert(4U * AERO_FIELDS + 4U + sizeof(aero_payload_t) == AERO_STEP_BYTES,
               "a pass's request moved");
_Static_assert(offsetof(aero_in_t, dt) == 4U * AERO_FIELDS, "the reading's fields moved");

static aero_t  s_core;
static uint8_t s_laid;

static float scaled(rd_t *in, double unit)
{
  const int32_t whole = rd_i32(in);

  return (float)((double)whole * unit);
}

static void put_micro(wr_t *out, float x)
{
  wr_i32(out, (int32_t)lrint(1e6 * fmin(MOST, fmax(-MOST, (double)x))));
}

/** op 0 - the frame laid, the core's memory as before a first pass: its mass, gravity, its
    inertia about x, y and z, each rotor's place and the way it turns, its drag's torque a
    thrust, the share's weights, the observers' bandwidths, the height's rate, the air's lean,
    the frame's drag, the rotors' loss, the wind's rate. */
static cmd_status_t h_aero_lay(rd_t *in, wr_t *out)
{
  aero_params_t p;

  p.mass = scaled(in, MICRO);
  p.gravity = scaled(in, MICRO);
  for (int j = 0; j < 3; j++)
  {
    p.inertia[j] = scaled(in, NANO);
  }
  for (int i = 0; i < 4; i++)
  {
    p.at_x[i] = scaled(in, MICRO);
  }
  for (int i = 0; i < 4; i++)
  {
    p.at_z[i] = scaled(in, MICRO);
  }
  for (int i = 0; i < 4; i++)
  {
    const int8_t way = rd_i8(in);

    p.spin[i] = (float)way;
  }
  p.kappa = scaled(in, MICRO);
  for (int k = 0; k < 4; k++)
  {
    p.weigh[k] = scaled(in, MILLI);
  }
  p.wo_turn = scaled(in, MILLI);
  p.wo_pull = scaled(in, MILLI);
  p.rise = scaled(in, MILLI);
  p.lean = scaled(in, MICRO);
  p.drag = scaled(in, MICRO);
  p.inflow = scaled(in, MICRO);
  p.wo_wind = scaled(in, MILLI);
  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  aero_init(&s_core, &p);
  s_laid = 1U;
  wr_took(out, NULL);
  return CMD_OK;
}

/** A pass's answer: the cycles it took and SYSCLK; each rotor's thrust, which stand at an end,
    the wrench asked and the one given, the axis asked and its miss, what the observers hold,
    the heading and the wind held. */
static void answered(wr_t *out, uint32_t took, const aero_out_t *got)
{
  wr_took(out, NULL);
  wr_u32(out, took);
  wr_u32(out, Board_SysClkHz());
  for (int i = 0; i < 4; i++)
  {
    put_micro(out, got->thrust[i]);
  }
  wr_u32(out, got->bound);
  for (int k = 0; k < 4; k++)
  {
    put_micro(out, got->asked[k]);
  }
  for (int k = 0; k < 4; k++)
  {
    put_micro(out, got->gave[k]);
  }
  for (int j = 0; j < 3; j++)
  {
    put_micro(out, got->axis[j]);
  }
  for (int j = 0; j < 3; j++)
  {
    put_micro(out, got->miss[j]);
  }
  for (int j = 0; j < 3; j++)
  {
    put_micro(out, got->turned[j]);
  }
  for (int j = 0; j < 3; j++)
  {
    put_micro(out, got->pushed[j]);
  }
  for (int j = 0; j < 2; j++)
  {
    put_micro(out, got->head[j]);
  }
  for (int j = 0; j < 3; j++)
  {
    put_micro(out, got->wind[j]);
  }
}

/** op 1 - a pass: the reading - the frame's axes, its rates, its speed, its height, each
    rotor's thrust as given and at its least and its most, the pass's microseconds - then the
    payload as its author's bytes (aero_payload_t); its answer (`answered`). Refused before a
    frame is laid. */
static cmd_status_t h_aero_step(rd_t *in, wr_t *out)
{
  aero_in_t      read;
  aero_payload_t flown;
  aero_out_t     got;
  float          fields[AERO_FIELDS];

  for (int k = 0; k < 28; k++)
  {
    fields[k] = scaled(in, MICRO);
  }
  memcpy(&read, fields, sizeof(fields));

  const uint32_t micros = rd_u32(in);
  const uint8_t *raw = rd_bytes(in, (uint16_t)sizeof(flown));

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }
  if (s_laid == 0U)
  {
    wr_took(out, "no frame laid: op 0 first");
    return CMD_OK;
  }
  read.dt = (float)((double)micros * MICRO);
  memcpy(&flown, raw, sizeof(flown));

  const uint32_t t0 = Board_Cycles();

  aero_step(&s_core, &flown, &read, &got);
  answered(out, Board_Cycles() - t0, &got);
  return CMD_OK;
}

cmd_status_t cmd_aero_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case AERO_OP_LAY:  return h_aero_lay(in, out);
    case AERO_OP_STEP: return h_aero_step(in, out);
    default:           return CMD_ERR_VALUE;
  }
}
