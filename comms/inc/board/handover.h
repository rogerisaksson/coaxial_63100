/** board/handover.h - what the comms stack needs from board_boot.c; included by board.h. */
#ifndef COMMS_BOARD_HANDOVER_H
#define COMMS_BOARD_HANDOVER_H

#include "boot.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** This board's type as the bootloader names it. */
#define BOARD_BOOT_TYPE  BOOT_TYPE_COAXIAL_63100

/** Who this node is: what a bootloader left in the handover slot, or the
    defaults where none did (docs/BOOT.md). */
typedef struct
{
  uint8_t type;       /**< BOARD_BOOT_TYPE                              */
  uint8_t unit;       /**< the unit id answered to                      */
  uint8_t position;   /**< down the limb; 0 where nobody assigned one   */
  uint8_t flags;      /**< assign's flags; 0 where nobody assigned them */
  bool    assigned;   /**< a bootloader left these, or they are defaults */
} board_identity_t;

/** First thing in main(): VTOR to this image, the sample path copied to
    ITCM. Here, not in the startup, so CubeMX can regenerate that. */
void Board_Early(void);

/** Apply what the bootloader left: the unit id and the termination. */
void Board_BootInit(void);
board_identity_t Board_Identity(void);

/** The MCU's unique id, twelve bytes little-endian off UID_BASE. */
void Board_Uid(uint8_t *out);

/** Back to the bootloader: the reset waits for the reply to leave the wire,
    and the bootloader finds STAY in the slot. */
void Board_BootStay(void);
void Board_BootPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_HANDOVER_H */
