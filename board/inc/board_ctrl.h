/** board_ctrl.h - The loop's tick, for the drive's sample. */
#ifndef BOARD_CTRL_H
#define BOARD_CTRL_H

#include "drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Once a PWM period, before the drive steps: a tick every divider periods, its command
    written into `d`'s setpoints. */
void Board_CtrlTick(drive_t *d);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CTRL_H */
