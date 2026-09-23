/** board/imu.h - what the comms stack needs from board_imu.c; included by board.h. */
#ifndef COMMS_BOARD_IMU_H
#define COMMS_BOARD_IMU_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The IMU poll loop's shared record: what it saw, and what went wrong. */
typedef struct
{
  uint8_t  loop;        /**< BOARD_IMU_LOOP_* */
  uint8_t  error;       /**< BOARD_IMU_ERR_*, the last one seen */
  uint8_t  last_fault;  /**< the last one that was not NONE, kept */
  uint8_t  last_fault_id; /**< for FRAME, the report id that stopped it */
  uint32_t updates;     /**< rotation vectors written, monotonic */
  uint32_t cargoes;     /**< cargoes taken off SPI2 */
  uint32_t errors;      /**< reads that failed */
  bool     have;        /**< whether the quaternion below means anything */
  uint8_t  report_id;
  uint8_t  status;      /**< accuracy in bits 1:0 */
  int16_t  i;
  int16_t  j;
  int16_t  k;
  int16_t  real;        /**< all four Q14 counts - the scale is the host's */

  /* THE THREE VECTORS, each on its own report and its own Q point - the
     scale stays the host's, as the quaternion's does. */
  bool     have_accel;
  bool     have_gyro;
  bool     have_mag;
  int16_t  accel[3];    /**< SH2 0x01, Q8, m/s^2 */
  int16_t  gyro[3];     /**< SH2 0x02, Q9, rad/s */
  int16_t  mag[3];      /**< SH2 0x03, Q4, uT */
  uint8_t  accel_status;
  uint8_t  gyro_status;
  uint8_t  mag_status;
} board_imu_state_t;

#define BOARD_IMU_LOOP_OFF   0U  /**< AFE_ON is low; nothing to poll */
#define BOARD_IMU_LOOP_INIT  1U  /**< powered, not yet brought up */
#define BOARD_IMU_LOOP_RUN   2U  /**< polling */
#define BOARD_IMU_LOOP_HELD  3U  /**< stopped, so the host may configure */

#define BOARD_IMU_ERR_NONE   0U
#define BOARD_IMU_ERR_POWER  1U  /**< AFE_ON went away under it */
#define BOARD_IMU_ERR_INIT   2U  /**< the part did not come up */
#define BOARD_IMU_ERR_READ   3U  /**< a cargo read failed */
#define BOARD_IMU_ERR_FRAME  4U  /**< a report id with no length */
#define BOARD_IMU_ERR_NOWAKE 5U  /**< wrote without an H_INTN acknowledge */

/** Advance the IMU poll loop. */
void Board_ImuPoll(void);

/** Read the shared record. The only way a host sees the stream. */
void Board_ImuState(board_imu_state_t *out);

/** Stop the loop so the part can be configured, or start it again. */
void Board_ImuHold(void);
void Board_ImuResume(void);

/** Bring SPI2 to what the BNO08X needs and take PB12 as a GPIO chip
    @return False if the peripheral would not re-initialise. */
bool Board_ImuInit(void);

/** SPI2 and the IMU's control pins, without resetting the part. */
bool Board_ImuBusInit(void);

/** Pulse NRSTN with BOOTN held high, then wait out the part's own */
void Board_ImuReset(void);

/** Whether Board_ImuInit() succeeded. Every call below fails until it has. */
bool Board_ImuReady(void);

/** Read one SHTP cargo, if the part has one waiting.
    @param  channel  The SHTP channel it arrived on.
    @param  cargo    The cargo WITHOUT its four-byte header.
    @param  len      Cargo bytes, 0 when the part had nothing to say.
    @return False on a transfer error or a header that contradicts itself. */
bool Board_ImuRead(uint8_t *channel, uint8_t *cargo, uint16_t cap,
                   uint16_t *len);

/** Frame a payload onto an SHTP channel and clock it out.
    @return False on a bad channel, a payload that will not fit, or a */
bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len);

/** Wait up to @p ms for the part to say it has something. */
bool Board_ImuWaitReady(uint32_t ms);

/** Ask the part to report `report_id` every `interval_us`, and REMEMBER it. */
bool Board_ImuSetFeature(uint8_t report_id, uint32_t interval_us);

/** What was last asked for. Interval zero means nothing has been. */
void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us,
                           bool *pending);

/** Collect and discard whatever the part has queued.
    @return How many cargoes were drained. */
uint8_t Board_ImuDrain(uint8_t limit);

/** SPI2's four pins on port B, chip select first: PB12 H_CSN, PB13 SCK, PB14
    MISO, PB15 MOSI - the rows board_io.c lists, by number for the pin check. */
#define BOARD_IMU_SPI_PIN_FIRST 12U
#define BOARD_IMU_SPI_PIN_COUNT 4U

/** Drive and release GPIOB pin `pin`, reporting what the pin then read. */
uint8_t Board_ImuPinCheck(uint8_t pin);

/** Assert PS0/WAKE on a drained part and time H_INTN's answer. */
uint16_t Board_ImuWakeTest(uint16_t ms);

/** Clock `len` bytes out, keep what comes back; false if the transfer failed. */
bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select);

/** The SPI2 kernel clock and the bit rate Board_ImuInit settled on, so the
    bench can see the number rather than infer it from a silent part. */
void Board_ImuClock(uint32_t *kernel_hz, uint32_t *bitrate_hz);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_IMU_H */
