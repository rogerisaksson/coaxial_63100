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
#include "dev_serial.h"
#include "wire.h"


/** What the link can carry in records per second at this record size.
  *
  * Both terms move: the stride, and whichever port is answering. The share is
  * measured, not derived, and leaves out the request, the turnaround and the
  * host's latency - none of which this board can compute.
  */
uint32_t cmd_link_records_per_second(uint16_t record_bytes)
{
  const uint32_t baud = dev_uart_baud();

  if ((record_bytes == 0U) || (baud == 0U))
  {
    return 0U;
  }
  return ((baud / 10U) * CMD_LINK_SHARE_PCT / 100U) / record_bytes;
}


void cmd_took(wr_t *out, const char *refusal)
{
  if (refusal == NULL)
  {
    wr_u8(out, 1U);
    return;
  }
  wr_u8(out, 0U);
  wr_str(out, refusal);
}


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
    case DEVICE_LINK:  return cmd_link_op(op, in, out);
    case DEVICE_CAL:   return cmd_cal_op(op, in, out);
    case DEVICE_GATE_DRIVERS: return cmd_gate_drivers_op(op, in, out);
    case DEVICE_LOG:    return cmd_log_op(op, in, out);
    case DEVICE_DAQ:    return cmd_daq_op(op, in, out);
    case DEVICE_TIME:   return cmd_time_op(op, in, out);
    case DEVICE_THERMAL: return cmd_thermal_op(op, in, out);
    case DEVICE_POWER:  return cmd_power_op(op, in, out);
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
