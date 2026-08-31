/**
  ******************************************************************************
  * @file    board_drive.h
  * @brief   The control law on this board: what cmd_drive.c may ask of it.
  *
  * Apart from board.h because it carries Drive/'s own types, and board.h
  * carries stdint and stdbool and nothing else. Same shape as
  * board_power.h.
  ******************************************************************************
  */
#ifndef BOARD_DRIVE_H
#define BOARD_DRIVE_H

#include "drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Once, after the calibration record has loaded and TIM1 is running. */
void Board_DriveInit(void);

/** The drive as it stands. Read only; the interrupt writes it. */
const drive_t *Board_Drive(void);

/** The PWM period the drive integrates over, seconds. */
float Board_DriveTs(void);

/** Enter a mode. Arms the sync if it is not, reloads the parameters from
  * the record, and refuses in the drive's own words. */
const char *Board_DriveSetMode(uint8_t mode);

/** One setpoint by id, in its integer unit - see cmd_drive.c. */
const char *Board_DriveSetpoint(uint8_t id, int32_t value);
void Board_DriveSetpointsGet(int32_t *out);

/** Put both frames at an angle. */
void Board_DriveSetTheta(int32_t microradians);

/** The window since the last take, then a new one. */
void Board_DriveWindowTake(drive_window_t *out);

void Board_DriveMomentsArm(uint32_t periods);
void Board_DriveMoments(drive_moments_t *out);

/** What one step cost in raw CYCCNT, last and worst. */
void Board_DriveCycles(uint32_t *last, uint32_t *max);
void Board_DriveCyclesReset(void);

/** The worst end of a step in TIM1 ticks past the trigger - conversion,
  * interrupt entry and step together, against the period's 2 x ARR. */
uint16_t Board_DriveExitTicks(void);

/** Whether the drive is committing the compares, so the host's own duty
  * writes are refused while it does. */
bool Board_DriveOwnsCompares(void);

/** Take the parameters out of the calibration record. */
void Board_DriveParamsFromCal(void);

/** Where the samples come from: 0 the converters, 1 the model. Refused
  * while a mode runs. The model needs no reference and no stage; its
  * duties reach the gates only if MOE happens to be set. */
const char *Board_DriveSetSource(uint8_t source);

/** One model parameter by id, in its integer unit - see cmd_drive.c. */
const char *Board_DriveModelParam(uint8_t id, int32_t value);

/** The rotor back to theta0, at rest, the pipeline empty. */
void Board_DriveModelReset(void);

/** From ADC3's injected end-of-sequence: the triple in raw centred codes,
  * the DC link raw single-ended. */
void Board_DriveOnSample(const int16_t *phase, uint32_t dcbus_raw);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_DRIVE_H */
