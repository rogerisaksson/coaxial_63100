/** shtp.h - CEVA SHTP framing and SH-2 report decoding. */
#ifndef SHTP_H
#define SHTP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Every cargo is prefixed with four bytes: length LSB, length MSB, channel,
    sequence number. */
#define SHTP_HEADER_LEN 4U

/** Bit 15 of the length field marks a continuation of a previous transfer;
    bits 14:0 are the total byte count INCLUDING these four. */
#define SHTP_CONTINUATION 0x8000U
#define SHTP_LENGTH_MASK  0x7FFFU

/** 0xFFFF is reserved: "a failed peripheral can too easily produce 0xFFFF". */
#define SHTP_LENGTH_RESERVED 0xFFFFU

/** The six channels the BNO08X supports, section 1.3.1. */
#define SHTP_CH_COMMAND    0U   /**< SHTP's own command channel */
#define SHTP_CH_EXECUTABLE 1U   /**< reset / on / sleep, Figure 1-27 */
#define SHTP_CH_CONTROL    2U   /**< SH-2 control, Figure 1-30 */
#define SHTP_CH_INPUT      3U   /**< input sensor reports, non-wake */
#define SHTP_CH_WAKE       4U   /**< wake-configured sensor reports */
#define SHTP_CH_GYRO_RV    5U   /**< gyro rotation vector */

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

/** Input report ids this firmware names. */
#define SH2_REPORT_ACCELEROMETER   0x01U
#define SH2_REPORT_GYROSCOPE       0x02U
#define SH2_REPORT_MAGNETIC_FIELD  0x03U
#define SH2_REPORT_LINEAR_ACCEL    0x04U
#define SH2_REPORT_ROTATION_VECTOR 0x05U
#define SH2_REPORT_GRAVITY         0x06U
#define SH2_REPORT_GAME_ROTATION   0x08U
#define SH2_REPORT_TIMEBASE        0xFBU

/** The largest cargo this firmware will assemble. */
#define SHTP_MAX_CARGO 256U

typedef struct
{
  uint16_t length;        /**< bytes 14:0, header included */
  bool     continuation;  /**< bit 15 set */
  uint8_t  channel;
  uint8_t  seq;
} shtp_header_t;

/** Decode the four-byte header.
    @return False if the length field is the reserved 0xFFFF, or shorter than */
bool shtp_parse_header(const uint8_t *raw, shtp_header_t *out);

/** Write a header plus payload into `buf`.
    @return Total bytes written, or 0 if it would not fit. */
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

/** One input report, as it arrived. */
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
  uint8_t count;       /**< 3 for a vector, 4 for a quaternion */
} shtp_report_t;

/** Walk one input cargo and decode the reports in it.
    @param  cargo  The cargo WITHOUT the SHTP header.
    @return How many reports were written to `out`. */
size_t shtp_parse_reports(const uint8_t *cargo, size_t len,
                          shtp_report_t *out, size_t max);

/** Input report ids this firmware picks out of a channel 3 cargo. */
#define SH2_ROTATION_VECTOR      0x05U
#define SH2_GAME_ROTATION_VECTOR 0x08U

/** Length of one input report, or 0 if this firmware does not know the id. */
size_t shtp_report_len(uint8_t report_id);

/** Build a Set Feature command, Figure 1-33.
    @param  interval_us  Report interval. 0 disables the sensor.
    @return Bytes written into `buf`, or 0 if 17 do not fit. */
size_t shtp_set_feature(uint8_t *buf, size_t cap, uint8_t report_id,
                        uint32_t interval_us);

#ifdef __cplusplus
}
#endif

#endif /* SHTP_H */
