/**
  ******************************************************************************
  * @file    cmd_device.c
  * @brief   Command 0x6E: every peripheral device, chosen by a device byte.
  *
  * One function code for all of them because there are no codes left - the
  * specification's user-defined ranges are 65..72 and 100..110, and this
  * board had spent all but 110. A second code answered ILLEGAL FUNCTION from
  * the protocol layer before dispatch saw it.
  *
  *     0x6E <device> <op> [payload]
  *
  * Adding a device is a row in the table below and an op dispatcher beside
  * cmd_imu.c and cmd_angle.c. What is *fitted* is a different question and
  * has a different answer: command 0x6D kind 4, the parts list.
  ******************************************************************************
  */
#include "cmd.h"
#include "wire.h"

static cmd_status_t h_device(rd_t *in, wr_t *out)
{
  const uint8_t device = rd_u8(in);
  const uint8_t op = rd_u8(in);

  if (!rd_ok(in))
  {
    return CMD_ERR_LENGTH;
  }

  switch (device)
  {
    case DEVICE_IMU:   return cmd_imu_op(op, in, out);
    case DEVICE_ANGLE: return cmd_angle_op(op, in, out);
    default:           return CMD_ERR_VALUE;
  }
}

static const cmd_desc_t DEVICE_TABLE[] =
{
  { CMD_DEVICE, "device", CMD_LEN_VARIABLE, h_device },
};

const cmd_desc_t *cmd_device_table(uint8_t *count)
{
  *count = (uint8_t)(sizeof(DEVICE_TABLE) / sizeof(DEVICE_TABLE[0]));
  return DEVICE_TABLE;
}
