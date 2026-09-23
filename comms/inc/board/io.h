/** board/io.h - what the comms stack needs from board_io.c; included by board.h. */
#ifndef COMMS_BOARD_IO_H
#define COMMS_BOARD_IO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *Board_Name(void);

/** Which way a pin's signal runs, seen from the MCU. */
#define BOARD_DIR_IN    0U
#define BOARD_DIR_OUT   1U
#define BOARD_DIR_INOUT 2U

/** One digital channel: a pin this board actually uses for something. */
typedef struct
{
  const char *pin;      /**< "PB2"                                  */
  uint8_t     dir;      /**< BOARD_DIR_*                            */
  const char *signal;   /**< what the pin carries on this board     */
  bool        usable;   /**< false where raw pin access is refused  */
} board_dchan_t;

uint8_t Board_DigitalCount(void);

/** The drivable pins - what `0x6D` kind 1 reports - as one word, bit i being
    slot i. */
uint32_t Board_DigitalMask(void);
uint8_t  Board_DigitalIoCount(void);
bool     Board_DigitalIoChan(uint8_t slot, board_dchan_t *info);

/** How many pins a DAQ record carries, and which. */
uint8_t  Board_DigitalSampledCount(void);
bool     Board_DigitalSampledChan(uint8_t slot, board_dchan_t *info);
bool    Board_DigitalChan(uint8_t index, board_dchan_t *info);

/** Whether a fixture may drive this pin at all. */
bool Board_PinUsable(char port, uint8_t pin);

/** What is fitted on the board, one entry per part. */
typedef struct
{
  const char *name;    /**< the part, as it is marked              */
  const char *what;    /**< what it does, one line                 */
  const char *where;   /**< the bus or pins it sits on             */
  const char *power;   /**< what must be on for it, or "" for none */
  uint8_t     state;   /**< BOARD_PART_* below                     */
} board_part_t;

#define BOARD_PART_UNKNOWN   0U  /**< nothing here can prove it either way */
#define BOARD_PART_READY     1U  /**< it answered                         */
#define BOARD_PART_UNPOWERED 2U  /**< what powers it is off               */
#define BOARD_PART_SILENT    3U  /**< powered, and did not answer         */

uint8_t Board_PartCount(void);
bool Board_Part(uint8_t index, board_part_t *info);

bool Board_AfeOn(void);
void Board_SetAfeOn(bool on);
bool Board_Pe15(void);

/** UART5's 120 ohm termination, PE14: closed by the last node on the
    segment, open on the rest (docs/BOOT.md). */
void Board_SetTermination(bool closed);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_IO_H */
