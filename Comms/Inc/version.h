/**
  ******************************************************************************
  * @file    version.h
  * @brief   Firmware and protocol versions. One place, nothing computed.
  *
  * FW_VERSION_*  what this build is; bump on any firmware change.
  * CMD_PROTO_*   what the wire looks like (cmd.h). MINOR when a field is
  *               APPENDED, MAJOR when an existing one moves, resizes or
  *               changes meaning - whether that was intended or not.
  *
  * A host reads both from 0x41 and selects its codec on the protocol MAJOR
  * alone; the firmware version is for the test record. Tying a host to it
  * means every rebuild breaks the host.
  ******************************************************************************
  */
#ifndef VERSION_H
#define VERSION_H

#define FW_VERSION_MAJOR 1U
#define FW_VERSION_MINOR 5U
#define FW_VERSION_PATCH 0U

#define FW_VERSION_STRING "1.5.0"

/* Costs reproducibility, deliberately: a rig that cannot tell which build a
   board carries cannot investigate a failure after the fact. */
#define FW_BUILD_STRING (__DATE__ " " __TIME__)

#define FW_DEVICE_NAME "coaxial_63100"
#define FW_MCU_NAME    "STM32H753VIT6"

/* One word, so a host can group a bus by it: the node that is not an
 * inverter is the one worth seeing in a list. */
#define FW_DEVICE_TYPE "bldc_inverter"

/* A name is not a description: "coaxial_63100" picks a codec and says nothing
 * about what is on the other end. With several units answering, that is a list
 * of devices instead of a list of numbers. The rating is in it because the
 * rating is the name; no measured value ever is - invariant 10. */
#define FW_DEVICE_DESCRIPTION   "Three-phase BLDC inverter, 63 V / 100 A, PCB mounted coaxially behind an "   "outrunner's stator. Instrumentation only: no timer, no PWM, no gate drive."

#endif /* VERSION_H */
