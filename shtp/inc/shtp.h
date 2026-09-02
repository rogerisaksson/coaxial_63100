/**
  ******************************************************************************
  * @file    shtp.h
  * @brief   CEVA SHTP framing and SH-2 report decoding. No hardware.
  *
  * The transport half of talking to a BNO08X. This translation unit knows
  * nothing about SPI, STM32, CMSIS or timers: it turns a byte buffer into a
  * header and a cargo, and a cargo into reports. Clocking the bytes in and out
  * is the board's job (see board.h, Board_ImuTransfer).
  *
  * That split is the same one modbus/ keeps, and for the same reason: it makes
  * this file compilable and testable on a host - see test_shtp_core.py.
  *
  * Raw counts only. The BNO08X reports fixed-point integers whose Q point is a
  * property of each report, and converting them to m/s^2 or radians is the
  * host's job - invariant 10. Nothing here scales anything.
  *
  * Datasheet references are to BNO080_085-Datasheet v1.17, in datasheets/.
  ******************************************************************************
  */
#ifndef SHTP_H
#define SHTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Every cargo is prefixed with four bytes: length LSB, length MSB, channel,
    sequence number. Figure 1-26. */
#define SHTP_HEADER_LEN 4U

/** Bit 15 of the length field marks a continuation of a previous transfer;
    bits 14:0 are the total byte count INCLUDING these four. Figure 1-26. */
#define SHTP_CONTINUATION 0x8000U
#define SHTP_LENGTH_MASK  0x7FFFU

/** 0xFFFF is reserved: "a failed peripheral can too easily produce 0xFFFF".
    A header reading it is a dead bus, not a 32 kB cargo. Figure 1-26. */
#define SHTP_LENGTH_RESERVED 0xFFFFU

/** The six channels the BNO08X supports, section 1.3.1. */
#define SHTP_CH_COMMAND    0U   /**< SHTP's own command channel        */
#define SHTP_CH_EXECUTABLE 1U   /**< reset / on / sleep, Figure 1-27   */
#define SHTP_CH_CONTROL    2U   /**< SH-2 control, Figure 1-30         */
#define SHTP_CH_INPUT      3U   /**< input sensor reports, non-wake    */
#define SHTP_CH_WAKE       4U   /**< wake-configured sensor reports    */
#define SHTP_CH_GYRO_RV    5U   /**< gyro rotation vector              */

/** Executable channel writes and reads, Figure 1-27. */
#define SHTP_EXEC_RESET 1U
#define SHTP_EXEC_ON    2U
#define SHTP_EXEC_SLEEP 3U

/** SH-2 report ids on the control channel, Figure 1-30. */
#define SH2_GET_FEATURE_REQUEST  0xFEU
#define SH2_SET_FEATURE_COMMAND  0xFDU
#define SH2_GET_FEATURE_RESPONSE 0xFCU
#define SH2_PRODUCT_ID_REQUEST   0xF9U
#define SH2_PRODUCT_ID_RESPONSE  0xF8U
#define SH2_COMMAND_REQUEST      0xF2U
#define SH2_COMMAND_RESPONSE     0xF1U

/** Input report ids this firmware names. The BNO08X defines more; these are
    the ones a bring-up asks for. Section 2. */
#define SH2_REPORT_ACCELEROMETER   0x01U
#define SH2_REPORT_GYROSCOPE       0x02U
#define SH2_REPORT_MAGNETIC_FIELD  0x03U
#define SH2_REPORT_LINEAR_ACCEL    0x04U
#define SH2_REPORT_ROTATION_VECTOR 0x05U
#define SH2_REPORT_GRAVITY         0x06U
#define SH2_REPORT_GAME_ROTATION   0x08U
#define SH2_REPORT_TIMEBASE        0xFBU

/** The largest cargo this firmware will assemble. The protocol allows 32766;
    a bring-up reading a handful of sensor reports needs nothing like it, and
    the buffer is static. */
#define SHTP_MAX_CARGO 256U

typedef struct
{
  uint16_t length;        /**< bytes 14:0, header included               */
  bool     continuation;  /**< bit 15 set                                */
  uint8_t  channel;
  uint8_t  seq;
} shtp_header_t;

/**
  * @brief  Decode the four-byte header.
  * @return False if the length field is the reserved 0xFFFF, or shorter than
  *         the header it is counting. Both mean the bus, not a cargo.
  */
bool shtp_parse_header(const uint8_t *raw, shtp_header_t *out);

/**
  * @brief  Write a header plus payload into `buf`.
  * @return Total bytes written, or 0 if it would not fit.
  */
size_t shtp_build(uint8_t *buf, size_t cap, uint8_t channel, uint8_t seq,
                  const uint8_t *payload, size_t len);

/** Product ID response, Figure 1-29. Raw fields; nothing is interpreted. */
typedef struct
{
  uint8_t  reset_cause;
  uint8_t  sw_major;
  uint8_t  sw_minor;
  uint32_t sw_part;
  uint32_t sw_build;
  uint16_t sw_patch;
} shtp_product_id_t;

/** @return False unless the cargo is a 0xF8 response of the full 16 bytes. */
bool shtp_parse_product_id(const uint8_t *cargo, size_t len,
                           shtp_product_id_t *out);

/**
  * @brief One input report, as it arrived.
  *
  * `x`..`w` are the report's own fixed-point counts. The Q point belongs to
  * the report id and is applied by the host, not here - invariant 10. `w` is
  * meaningful only for the quaternion reports; `count` says how many of the
  * four fields the report actually carried.
  */
typedef struct
{
  uint8_t report_id;
  uint8_t seq;
  uint8_t status;      /**< bits 1:0 are the accuracy, section 1.3.5.2 */
  uint8_t delay;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t w;
  uint8_t count;       /**< 3 for a vector, 4 for a quaternion         */
} shtp_report_t;

/**
  * @brief  Walk one input cargo and decode the reports in it.
  * @param  cargo  The cargo WITHOUT the SHTP header.
  * @return How many reports were written to `out`.
  *
  * A cargo on channel 3 opens with a timebase report and then carries one or
  * more sensor reports back to back (Figure 5-2). An unknown report id ends
  * the walk rather than guessing its length: the reports are not
  * self-delimiting, so a wrong length would silently mis-frame the rest.
  */
size_t shtp_parse_reports(const uint8_t *cargo, size_t len,
                          shtp_report_t *out, size_t max);

/** Length of one input report, or 0 if this firmware does not know the id. */
/** Input report ids this firmware picks out of a channel 3 cargo. */
#define SH2_ROTATION_VECTOR      0x05U
#define SH2_GAME_ROTATION_VECTOR 0x08U

size_t shtp_report_len(uint8_t report_id);

/**
  * @brief  Build a Set Feature command, Figure 1-33.
  * @param  interval_us  Report interval. 0 disables the sensor.
  * @return Bytes written into `buf`, or 0 if 17 do not fit.
  */
size_t shtp_set_feature(uint8_t *buf, size_t cap, uint8_t report_id,
                        uint32_t interval_us);

#ifdef __cplusplus
}
#endif

#endif /* SHTP_H */
