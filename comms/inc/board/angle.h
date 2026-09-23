/** board/angle.h - what the comms stack needs from board_angle.c; included by board.h. */
#ifndef COMMS_BOARD_ANGLE_H
#define COMMS_BOARD_ANGLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The A1335's poll loop record, the same shape as the IMU's. */
typedef struct
{
  uint8_t  loop;        /**< BOARD_ANGLE_LOOP_*                          */
  uint8_t  error;       /**< BOARD_ANGLE_ERR_*, the last one seen        */
  uint32_t updates;     /**< readings written, monotonic                 */
  uint32_t errors;      /**< reads that failed                           */
  bool     have;        /**< whether `value` means anything              */
  uint8_t  reg;         /**< which register it came from                 */
  uint16_t value;       /**< the sixteen data bits, unscaled             */
  uint8_t  crc;         /**< the four CRC bits, unchecked - see the .c   */
} board_angle_state_t;

#define BOARD_ANGLE_LOOP_OFF  0U  /**< no supply, or not yet brought up  */
#define BOARD_ANGLE_LOOP_RUN  1U  /**< polling                           */
#define BOARD_ANGLE_LOOP_HELD 2U  /**< stopped, so the host may configure */

#define BOARD_ANGLE_ERR_NONE   0U
#define BOARD_ANGLE_ERR_POWER  1U  /**< AFE_ON went away under it        */
#define BOARD_ANGLE_ERR_INIT   2U  /**< SPI4 would not configure         */
#define BOARD_ANGLE_ERR_READ   3U  /**< the transfer failed              */
#define BOARD_ANGLE_ERR_SILENT 4U  /**< all ones: absent or unpowered    */

bool Board_AngleInit(void);
bool Board_AngleReady(void);
void Board_AngleClock(uint32_t *kernel_hz, uint32_t *bitrate_hz);

/** One 20-bit packet: the register's sixteen data bits and its four CRC
    bits, neither interpreted here. */
bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc);

/** The A1335's own die, centi-degrees C. */
bool Board_AngleDie(int32_t *centidegc);
bool Board_AngleWrite(uint8_t reg, uint8_t value);

/** Advance the angle sensor's poll loop. */
void Board_AnglePoll(void);
void Board_AngleState(board_angle_state_t *out);
void Board_AngleHold(void);
void Board_AngleResume(void);

/** Which register the loop reads. */
bool Board_AnglePollReg(uint8_t reg);
uint8_t Board_AnglePollRegGet(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_ANGLE_H */
