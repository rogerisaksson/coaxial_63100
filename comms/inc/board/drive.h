/** board/drive.h - what the comms stack needs from board_drive.c; included by board.h. */
#ifndef COMMS_BOARD_DRIVE_H
#define COMMS_BOARD_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Scale the drive's current clamp, 1.0 down to 0.0. */
void Board_DriveDerate(float factor);

/** What that factor is now. */
float Board_DriveDerating(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_DRIVE_H */
