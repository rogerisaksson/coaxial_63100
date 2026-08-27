/**
  ******************************************************************************
  * @file    cmd.h
  * @brief   Request/response command layer. Protocol-agnostic, table-driven.
  *
  * A command is a code, an expected request length, and one function that reads
  * a request payload and writes a response payload. It knows nothing about
  * framing, addressing, CRCs or UARTs - the protocol below it carries the
  * bytes, and for Modbus these codes ride in the user-definable function code
  * range that the specification reserves for exactly this (65..72, 100..110).
  *
  * Broadcast: a command carried by a broadcast frame is executed and produces
  * no response. Handlers do not need to care; the protocol layer drops the
  * response. Commands that only read are therefore pointless as broadcasts,
  * and commands that act are the useful ones.
  *
  * WIRE FORMAT
  * ===========
  * All integers big-endian. No floating point on the wire: physical quantities
  * are scaled integers in the units named below. Strings are one length byte
  * followed by that many ASCII characters, never terminated.
  *
  * 0x41 VERSION       req: -
  *                    rsp: u8 proto_major, u8 proto_minor,
  *                         u8 fw_major, u8 fw_minor, u8 fw_patch,
  *                         str device, str mcu, str build,
  *                         u16 command_count, str description, str type
  *
  *                    THE FROZEN COMMAND. The protocol major is first so any
  *                    host can read two bytes, decide whether it understands
  *                    this device, and stop. Fields may only be APPENDED after
  *                    that - an old host decodes the prefix it knows and
  *                    ignores the rest. Reorder or resize anything here and
  *                    you have created a new major whether you meant to or not.
  *
  *                    A host selects its codec on the protocol major ALONE.
  *                    The firmware version is for the test record; binding a
  *                    host to it means every rebuild breaks the host.
  *
  * 0x42 ADC_TABLE     req: -
  *                    rsp: u8 count, then per channel:
  *                         u8 adc_index (1..3), u8 channel, str pin,
  *                         u8 differential, str signal,
  *                         i32 raw, i32 microvolts_at_pin,
  *                         u8 unit  (0 none, 1 millivolt, 2 centidegC),
  *                         i32 scaled  (meaningful only when unit != 0)
  *                    The blank columns of the ASCII table become unit = 0 and
  *                    scaled = 0: a channel whose physical quantity is not
  *                    defined reports that fact rather than a made-up number.
  *
  * 0x43 ADC_SCAN      req: -
  *                    rsp: i32 phase_u_raw, i32 phase_v_raw, i32 phase_w_raw,
  *                         i32 dcbus_raw, i32 dcbus_millivolt,
  *                         i32 ntc_raw, i32 ntc_centidegc,
  *                         u8 afe_on, u8 pe15
  *
  * 0x44 ADC_NOISE     req: u8 adc_index (1..3), u16 samples (1..1000)
  *                    rsp: u16 samples, i32 mean_microvolt, i32 min_raw,
  *                         i32 max_raw, u32 span_raw, u32 stddev_microvolt
  *
  * 0x45 CLOCK         req: -
  *                    rsp: u32 sysclk_hz, u32 hclk_hz, u32 cyccnt,
  *                         u32 ticks_per_us, u8 sysclk_source
  *                         (0 HSI, 1 CSI, 2 HSE, 3 PLL1)
  *
  * 0x46 AFE           req: u8 action (0 read, 1 off, 2 on, 3 toggle)
  *                    rsp: u8 afe_on, u8 pe15
  *
  * 0x47 LINK_STATS    req: -
  *                    rsp: u8 unit_id, u32 t15_ticks, u32 t35_ticks,
  *                         u32 bus_message, u32 bus_comm_error,
  *                         u32 server_message, u32 server_exception,
  *                         u32 server_no_response, u32 char_overrun
  *
  * 0x48 CONSOLE       req: -
  *                    rsp: -   (the ASCII console resumes once this is answered)
  *
  * TEST FIXTURE COMMANDS, in the specification's second user-definable range
  * (100..110). Raw pin access for a production rig. Everything that
  * reconfigures or drives a pin needs the gate open; reads never do.
  *
  * 0x64 TEST_GATE     req: u32 key (0x54455354, ASCII "TEST"), u8 open
  *                    rsp: u8 open
  *                    A wrong key is ILLEGAL DATA VALUE and leaves the gate as
  *                    it was, so the mode cannot be entered by accident.
  *
  * 0x65 ECHO          req: 0..250 arbitrary bytes
  *                    rsp: the same bytes, unchanged
  *                    A link test: proves framing, CRC and both codecs round
  *                    trip without depending on any board state.
  *
  * 0x66 PIN_MODE      req: u8 port ('A'..'K'), u8 pin (0..15),
  *                         u8 mode (0 input, 1 out PP, 2 out OD, 3 analog),
  *                         u8 pull (0 none, 1 up, 2 down)
  *                    rsp: -
  * 0x67 PIN_READ      req: u8 port, u8 pin        rsp: u8 level
  * 0x68 PIN_WRITE     req: u8 port, u8 pin, u8 level
  *                    rsp: u8 level read back from the pin
  * 0x69 PORT_READ     req: u8 port                rsp: u16 IDR
  * 0x6A PORT_WRITE    req: u8 port, u16 mask, u16 value
  *                    rsp: u16 IDR read back
  *                    Written through BSRR so it is atomic. Reserved pins are
  *                    masked out of the write rather than rejecting it.
  *
  * 0x6B ANALOG_BURST  req: u16 channel_mask (bit i = channel i of the table),
  *                         u16 samples (1..10000),
  *                         u32 interval_us (0 = as fast as conversions allow)
  *                    rsp: u16 samples_taken, u32 elapsed_us, u8 count,
  *                         then per channel, ascending index:
  *                           u8 index, i32 mean_milliraw, i32 min_raw,
  *                           i32 max_raw, u32 sd_milliraw
  *
  *                    RAW CODES ONLY, on purpose. The host owns the scaling -
  *                    its divider ratios, its thermistor constants, its
  *                    reference voltage - so a fixture with different parts
  *                    needs no firmware change. Means and deviations are in
  *                    milli-codes (raw x 1000) to carry fractions without a
  *                    float on the wire.
  *
  *                    elapsed_us is measured, not assumed, so a host can see
  *                    the sample rate it actually got. A burst longer than 5 s
  *                    is refused rather than left to outlive the master.
  *
  * 0x6D CHANNELS      req: u8 kind (0 analog, 1 digital IO, 2 reserved)
  *                    rsp, kind 0: u8 count, then per analog channel:
  *                         u8 index, u8 adc_index, u8 channel, str pin,
  *                         u8 direction, u8 differential, str signal, u8 unit
  *                    rsp, kind 1 and 2: u8 count, then per pin:
  *                         str pin, u8 direction, str signal
  *
  *                    Kinds 1 and 2 are kept apart on purpose. Kind 1 is the
  *                    digital I/O: what a fixture may read or set without
  *                    breaking anything. Kind 2 is the bus and the debug
  *                    port - USART3, JTAG - which are not channels and are
  *                    never to be driven; they are reported only so "why was
  *                    PB10 refused" has an answer.
  *
  *                    Sections, not one reply, because one does not fit:
  *                    measured, all of it together came to 273 bytes against
  *                    MB_MAX_PDU's 253 and the writer's overflow flag turned
  *                    the first live call into an 0x04.
  *
  *                    direction is 0 in, 1 out, 2 both, from the MCU's side.
  *                    Analog channels are all inputs and say so rather than
  *                    leaving the host to assume it.
  *
  *                    This is the map. Nothing above the firmware should
  *                    carry a copy of it - see docs/PROTOCOL.md.
  *
  * 0x6C SELF_TEST     req: -
  *                    rsp: u8 count, then per check:
  *                           str name, u8 status, i32 value
  *                         status: 0 pass, 1 fail, 2 info
  *
  *                    PASS/FAIL only where the board can PROVE the answer from
  *                    its own registers or its own flash - a locked PLL, a
  *                    calibration that ran, a checksum. Anything a calibrated
  *                    instrument would have to judge is reported as INFO with
  *                    its value, and the decision belongs to the test executive
  *                    on the line.
  *
  *                    This board is a dumb slave: it measures and reports. A
  *                    limit compiled into firmware is a limit nobody on the
  *                    line can see, change, or record against a calibration
  *                    certificate.
  *
  * Reserved pins - USART3 on PB10/PB11, and the debug port on PA13..PA15, PB3
  * and PB4 - are refused in every mode. Touching them would sever the link the
  * command arrived on, or the ability to reflash.
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
/* The last user-defined function code there is. MODBUS reserves 65..72 and
   100..110 for them (modbus_slave.c, fc_is_user_defined); this repository has
   spent 0x41..0x48 and 0x64..0x6D, so everything the IMU needs goes behind
   one code with an operation byte in front. 0x6F answered ILLEGAL FUNCTION
   from the protocol layer before dispatch ever saw it - measured. */
#define CMD_DEVICE       0x6EU

/* Which peripheral 0x6E's payload is addressed to. See cmd_device.c on why
   there is a device byte rather than a function code each. */
#define DEVICE_IMU       0U
#define DEVICE_ANGLE     1U
#define DEVICE_LINK      2U
#define DEVICE_CAL       3U
#define DEVICE_BRIDGE    4U
#define DEVICE_LOG       5U
#define DEVICE_DAQ       6U

/** Device 4's ops: the bridge, the synced triple and the STO chain. */
#define BRIDGE_OP_STATE    0U   /**< -> flags, registers, triple, STO      */
#define BRIDGE_OP_PWM      1U   /**< u8 on  -> u8 took                     */
#define BRIDGE_OP_DUTY     2U   /**< u16 x3 -> u8 took, all three or none  */
#define BRIDGE_OP_SYNC     3U   /**< u8 on  -> u8 took                     */
#define BRIDGE_OP_TRIGGER  4U   /**< u16 CCR4 -> u16 as it reads back      */
#define BRIDGE_OP_CLEAR    5U   /**< -> u8 took; does NOT re-arm           */
#define BRIDGE_OP_BYPASS   6U   /**< u8 on -> u8 took; drops BDTR.BKE      */
#define BRIDGE_OP_GAPRST   7U   /**< -> u8; forget the worst keepalive gap */

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

#define CMD_PROTO_MAJOR 1U
#define CMD_PROTO_MINOR 14U

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

/**
  * @brief One command implementation.
  *
  * Reads from @p in and writes to @p out. Both are total: overflow and underrun
  * set a sticky flag rather than failing at the point of use, so a handler is a
  * flat run of statements and the dispatcher checks once.
  */
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

/** The BNO08X commands. See cmd_imu.c. */
const cmd_desc_t *cmd_device_table(uint8_t *count);

/** One device's operations. Called by cmd_device.c after it has taken the
  * device byte off the request; `in` is positioned at the op's payload. */
cmd_status_t cmd_imu_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_angle_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_link_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_cal_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_bridge_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_log_op(uint8_t op, rd_t *in, wr_t *out);
cmd_status_t cmd_daq_op(uint8_t op, rd_t *in, wr_t *out);

/**
  * @brief One subsystem: a command table, named, with what it is for.
  *
  * The board says what it is made of for the same reason it says what its
  * channels are - a host that answers that from a table of its own is a
  * second answer to a question only the firmware can settle. Adding a
  * command table adds a subsystem, because they are the same thing.
  */
typedef struct
{
  const char *name;
  const char *what;
  uint8_t     commands;
} cmd_group_t;

/** How many subsystems this firmware has. */
uint8_t cmd_group_count(void);

/** Subsystem `index`, or NULL past the end. */
const cmd_group_t *cmd_group(uint8_t index);

/** Flat iteration over every table, for listing and for the dispatcher. */
uint16_t          cmd_count(void);
const cmd_desc_t *cmd_at(uint16_t index);

/** Find a command by code, or NULL. */
const cmd_desc_t *cmd_find(uint8_t code);

/**
  * @brief  Run one command.
  * @param  rsp_len  Response payload length on success, 0 otherwise.
  * @return CMD_OK, or the reason it could not run.
  */
cmd_status_t cmd_dispatch(uint8_t code, const uint8_t *req, uint16_t req_len,
                          uint8_t *rsp, uint16_t rsp_cap, uint16_t *rsp_len);

#ifdef __cplusplus
}
#endif

#endif /* CMD_H */
