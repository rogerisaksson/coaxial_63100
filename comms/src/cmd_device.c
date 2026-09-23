/** cmd_device.c - Command 0x6E: dispatch on the device byte. */
#include "boot.h"
#include "cmd.h"
#include "dev_serial.h"
#include "wire.h"

/** What the link can carry in records per second at this record size. */
uint32_t cmd_link_records_per_second(uint16_t record_bytes)
{
  const uint32_t baud = dev_uart_baud();

  if ((record_bytes == 0U) || (baud == 0U))
  {
    return 0U;
  }
  return ((baud / 10U) * CMD_LINK_SHARE_PCT / 100U) / record_bytes;
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
    case DEVICE_DRIVE:  return cmd_drive_op(op, in, out);
    case DEVICE_BOOT:   return cmd_boot_op(op, in, out);
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
