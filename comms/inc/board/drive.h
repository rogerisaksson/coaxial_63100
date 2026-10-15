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

/** The clamp as the record has it, A, the factor aside. */
float Board_DriveClamp(void);

/** Continuous War Emergency Power escalation parameter gamma_WEP in [0, 1]. */
void Board_DriveSetWep(float gamma_wep);
float Board_DriveWep(void);

/** Anchor the drive's real-time junction observer to the PCB NTC temperature (deg C). */
void Board_DriveSetPcbTemp(float t_ntc);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_DRIVE_H */
