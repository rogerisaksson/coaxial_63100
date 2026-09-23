/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Request/response command layer. Protocol-agnostic, table-driven.
  ******************************************************************************
  */
#ifndef CMD_H
#define CMD_H

#include "wire.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_VERSION    0x41U
#define CMD_ADC_TABLE  0x42U
#define CMD_ADC_SCAN   0x43U
#define CMD_ADC_NOISE  0x44U
#define CMD_CLOCK      0x45U
#define CMD_AFE        0x46U
#define CMD_LINK_STATS 0x47U
#define CMD_CONSOLE    0x48U

#define CMD_TEST_GATE  0x64U
#define CMD_ECHO       0x65U
#define CMD_PIN_MODE   0x66U
#define CMD_PIN_READ   0x67U
#define CMD_PIN_WRITE  0x68U
#define CMD_PORT_READ  0x69U
#define CMD_PORT_WRITE 0x6AU
#define CMD_ANALOG_BURST 0x6BU
#define CMD_SELF_TEST    0x6CU
#define CMD_CHANNELS     0x6DU
/* The last user-defined function code there is. */
#define CMD_DEVICE       0x6EU

/* Which peripheral 0x6E's payload is addressed to. */
#define DEVICE_IMU       0U
#define DEVICE_ANGLE     1U
#define DEVICE_LINK      2U
#define DEVICE_CAL       3U
#define DEVICE_GATE_DRIVERS    4U
#define DEVICE_LOG       5U
#define DEVICE_DAQ       6U
#define DEVICE_TIME      7U
#define DEVICE_THERMAL   8U
#define DEVICE_POWER     9U
#define DEVICE_DRIVE    10U

/* The thermal observer, device 8. */
#define THERMAL_OP_STATE        0U
#define THERMAL_OP_SET_NODE     1U
#define THERMAL_OP_SET_BOARD    2U
#define THERMAL_OP_SET_SAMPLE   3U
#define THERMAL_OP_BUDGET       4U
#define THERMAL_OP_SET_LIMIT    5U
#define THERMAL_OP_SET_WINDING  6U
#define THERMAL_OP_NODES        7U
#define THERMAL_OP_EDGES        8U
#define THERMAL_OP_SET_EDGE     9U
#define THERMAL_OP_IDENT        10U
#define THERMAL_OP_IDENT_RESET  11U
#define THERMAL_OP_SET_MARGIN   12U

/** Device 9's ops: the rails and who holds them. */
#define POWER_OP_STATE        0U  /**< -> u8 rails, per rail on, users, count, blocked, leased */
#define POWER_OP_RELEASE_ALL  1U  /**< every hold dropped -> u8 took; blunt, for a leak */

/** Device 10's ops: the control law. */
#define DRIVE_OP_STATE        0U  /**< -> mode, fault, flags, frames, dq, costs */
#define DRIVE_OP_MODE         1U  /**< u8 mode -> u8 took                        */
#define DRIVE_OP_SETPOINT     2U  /**< u8 id, i32 value -> u8 took               */
#define DRIVE_OP_SETPOINTS    3U  /**< -> u8 count, i32 x count                  */
#define DRIVE_OP_THETA        4U  /**< i32 urad -> u8 took; both frames          */
#define DRIVE_OP_WINDOW       5U  /**< -> the window since the last take, reset  */
#define DRIVE_OP_MOMENTS_ARM  6U  /**< u32 periods -> u8 took                    */
#define DRIVE_OP_MOMENTS      7U  /**< -> done, n, want, trigger, 4 x channel    */
#define DRIVE_OP_RELOAD       8U  /**< parameters out of the record -> u8 took   */
#define DRIVE_OP_CYCLES_RESET 9U  /**< forget the worst step cost -> u8          */
#define DRIVE_OP_SOURCE      10U  /**< u8 0 converters, 1 the model -> u8 took   */
#define DRIVE_OP_MODEL_PARAM 11U  /**< u8 id, i32 value -> u8 took               */
#define DRIVE_OP_MODEL       12U  /**< -> u8 source, i32 theta, omega, id, iq, vdc */
#define DRIVE_OP_MODEL_RESET 13U  /**< the rotor back to theta0, at rest -> u8   */
#define DRIVE_OP_OBSERVERS   14U  /**< -> the back-EMF chain beside the loop     */

/** Device 4's ops: the gate drivers, the synced triple and the STO chain. */
#define GATEDRIVERS_OP_STATE    0U   /**< -> flags, registers, triple, STO      */
#define GATEDRIVERS_OP_PWM      1U   /**< u8 on  -> u8 took                     */
#define GATEDRIVERS_OP_DUTY     2U   /**< u16 x3 [, u32 periods] -> u8 took     */
#define GATEDRIVERS_OP_SYNC     3U   /**< u8 on  -> u8 took                     */
#define GATEDRIVERS_OP_TRIGGER  4U   /**< u16 CCR4 -> u16 as it reads back      */
#define GATEDRIVERS_OP_CLEAR    5U   /**< -> u8 took; does NOT re-arm           */
#define GATEDRIVERS_OP_BYPASS   6U   /**< u8 on -> u8 took; drops BDTR.BKE      */
#define GATEDRIVERS_OP_GAP_RESET 7U   /**< -> u8; forget the worst keepalive gap */
#define GATEDRIVERS_OP_DUTY_FINE 8U   /**< u32 x3 ticks Q16.16 -> u8 took        */
#define GATEDRIVERS_OP_DEADTIME 9U   /**< u32 ns, i8 skew -> u8 took            */
#define GATEDRIVERS_OP_ALTERNATE 10U /**< u16 x3 ticks A, u16 x3 ticks B -> u8 took: A one period, B the next */

/** Device 5's ops: the measurement ring. */
#define LOG_OP_STATE    0U   /**< -> u8 sources, u16 count, u16 depth, u32 dropped */
#define LOG_OP_ARM      1U   /**< u8 source mask -> u8 took; empties the ring */
#define LOG_OP_TAKE     2U   /**< [u8 want] -> u8 got, then got x 14-byte records */

/** Device 6's ops: one acquisition task, DAQmx's shape cut to this board. */
#define DAQ_OP_STATE     0U  /**< -> flags, stride, fields, counts, config   */
#define DAQ_OP_CONFIGURE 1U  /**< channels, clock, sample_time, dec, acc, n  */
#define DAQ_OP_START     2U
#define DAQ_OP_STOP      3U
#define DAQ_OP_READ      4U  /**< [u8 want] -> u8 got, then got x stride     */
#define DAQ_OP_LAYOUT    5U  /**< -> what each field is, named by the board  */
#define DAQ_OP_LIVE      6U  /**< -> u8 fresh, then the accumulator, reset   */
/* MINOR 4: the anti-alias chain, and a tone to prove the path carried it. */
#define DAQ_OP_FILTER    7U  /**< u8 count, u16 decimate, i32 x 5 x count   */
#define DAQ_OP_TONE      8U  /** < u32 hz, u32 rate, i32 amp, i32 offset, u8 kind: 0 sine, 1 ramp */
#define DAQ_OP_RUNG      9U  /** < u8 rung, u16 boxcar, u8 count, u16 decimate, i32 x 5 x count */

/** Device 7's ops: the cycle counter, latched. */
#define TIME_OP_LATCH    0U  /**< take CYCCNT now                            */
#define TIME_OP_READ     1U  /**< -> u32 seq, latched, now, sysclk_hz        */

/* Operations under CMD_IMU. */
#define IMU_OP_ID      0U
#define IMU_OP_READ    1U
#define IMU_OP_FEATURE 2U
#define IMU_OP_PROBE   3U
#define IMU_OP_RESET   4U
#define IMU_OP_WRITE   5U
#define IMU_OP_PINS    6U
#define IMU_OP_WAKE    7U
#define IMU_OP_LATEST  8U
#define IMU_OP_HOLD    9U
#define IMU_OP_RESUME  10U

/* The A1335's operations, device 1. */
#define ANGLE_OP_READ    0U
#define ANGLE_OP_WRITE   1U
#define ANGLE_OP_LATEST  2U
#define ANGLE_OP_HOLD    3U
#define ANGLE_OP_RESUME  4U
#define ANGLE_OP_POLLREG 5U
#define ANGLE_OP_CLOCK   6U

/* The serial ports, device 2. */
#define LINK_OP_ECHO     0U
#define LINK_OP_STATS    1U

/* The calibration record, device 3. Only CAL_OP_SAVE touches flash. */
#define CAL_OP_GET         0U
#define CAL_OP_SET_PARAM   1U
#define CAL_OP_SET_CHANNEL 2U
#define CAL_OP_ZERO        3U
#define CAL_OP_SPAN        4U
#define CAL_OP_SAVE        5U
#define CAL_OP_LOAD        6U
#define CAL_OP_DEFAULTS    7U
/* MINOR 2: u8 first -> u8 total, u8 first, u8 count, u32 x count. */
#define CAL_OP_PARAMS      8U

/* 2.0, 2026-08-29: the thermal nodes went per leg, which REPURPOSED wire
   indices - device 8 node order and the cal record's ceilings both. */
#define CMD_PROTO_MAJOR 2U
#define CMD_PROTO_MINOR 18U        /* 1: gate drivers op 10, alternate 2: device 10, the drive; the DC link
   appended to gate drivers op 0 3: a daq record ends with u16 count, and
   accumulate 0 closes it on the clock. */

/** Request payload length of a command that takes a variable-length payload. */
#define CMD_LEN_VARIABLE 0xFFU

typedef enum
{
  CMD_OK = 0,
  CMD_ERR_UNKNOWN,   /**< no such command code            */
  CMD_ERR_LENGTH,    /**< request payload length is wrong */
  CMD_ERR_VALUE,     /**< a field is out of range         */
  CMD_ERR_DEVICE     /**< the board could not comply      */
} cmd_status_t;

/** One command implementation. */
typedef cmd_status_t (*cmd_handler_t)(rd_t *in, wr_t *out);

typedef struct
{
  uint8_t       code;
  const char   *name;
  uint8_t       req_len;   /**< exact length, or CMD_LEN_VARIABLE */
  cmd_handler_t fn;
} cmd_desc_t;

/** The command table for this board, defined in cmd_board.c. */
const cmd_desc_t *cmd_board_table(uint8_t *count);

/** Test fixture commands, defined in cmd_test.c. */
const cmd_desc_t *cmd_test_table(uint8_t *count);

/** Command 0x6E, the device dispatch. See cmd_device.c. */
const cmd_desc_t *cmd_device_table(uint8_t *count);

/** One device's operations. */
cmd_status_t cmd_imu_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_angle_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_link_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_cal_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_gate_drivers_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_log_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_daq_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_thermal_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_power_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_drive_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_boot_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_time_op(uint8_t op, rd_t *in, wr_t *out);


/** One subsystem: a command table, named, with what it is for. */
typedef struct
{
  const char *name;
  const char *what;
  uint8_t     commands;
} cmd_group_t;

/** How many subsystems this firmware has. */
/** Share of the raw line rate a stream of records may claim. */
#define CMD_LINK_SHARE_PCT 75U

/** Records per second the link can carry at this record size. */
uint32_t cmd_link_records_per_second(uint16_t record_bytes);

uint8_t cmd_group_count(void);

/** Subsystem `index`, or NULL past the end. */
const cmd_group_t *cmd_group(uint8_t index);

/** Flat iteration over every table, for listing and for the dispatcher. */
uint16_t          cmd_count(void);
const cmd_desc_t *cmd_at(uint16_t index);

/** Find a command by code, or NULL. */
const cmd_desc_t *cmd_find(uint8_t code);

/** Run one command.
    @param  rsp_len  Response payload length on success, 0 otherwise.
    @return CMD_OK, or the reason it could not run. */
cmd_status_t cmd_dispatch(uint8_t code, const uint8_t *req, uint16_t req_len,
                          uint8_t *rsp, uint16_t rsp_cap, uint16_t *rsp_len);

#ifdef __cplusplus
}
#endif

#endif /* CMD_H */
