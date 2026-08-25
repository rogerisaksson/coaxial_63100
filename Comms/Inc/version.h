/**
  ******************************************************************************
  * @file    version.h
  * @brief   Firmware and protocol versions. One place, nothing computed.
  *
  * TWO VERSIONS, ON PURPOSE
  * =======================
  * FW_VERSION_*    what this build is. Bump it whenever the firmware changes.
  * CMD_PROTO_*     what the wire looks like (in cmd.h). Bump the MINOR when
  *                 fields are APPENDED, the MAJOR when anything existing moves,
  *                 shrinks, or changes meaning.
  *
  * A host reads both from command 0x41 and selects its codec on the protocol
  * MAJOR alone. The firmware version is for the test record, not for deciding
  * how to talk - tying a host to firmware numbers means every rebuild breaks it.
  *
  * THE APPEND-ONLY RULE
  * ===================
  * Command 0x41's payload may only ever grow at the end. That is what lets an
  * old host talk to new firmware: it decodes the prefix it knows and ignores
  * the rest. Reorder or resize a field and you have made a new MAJOR, whether
  * you meant to or not.
  ******************************************************************************
  */
#ifndef VERSION_H
#define VERSION_H

#define FW_VERSION_MAJOR 1U
#define FW_VERSION_MINOR 4U
#define FW_VERSION_PATCH 1U

#define FW_VERSION_STRING "1.4.1"

/* __DATE__ and __TIME__ make the binary non-reproducible, which is a real cost.
   It is paid deliberately: a production rig that cannot tell which build a
   board is carrying cannot investigate a failure after the fact. */
#define FW_BUILD_STRING (__DATE__ " " __TIME__)

#define FW_DEVICE_NAME "coaxial_63100"
#define FW_MCU_NAME    "STM32H753VIT6"

/* What KIND of device this is, from the device. One word, so a host can
 * group a bus by it: several joints of a machine are several inverters,
 * and the node that is not one is the one worth seeing in a list. */
#define FW_DEVICE_TYPE "bldc_inverter"

/* What this device IS, in one line, from the device itself.
 *
 * A name is not a description: "coaxial_63100" tells a host which codec to
 * use and nothing about what is on the other end of the bus. With several
 * units answering, that is the difference between a list of numbers and a
 * list of devices. Appended to the frozen record, which is what the
 * append-only rule is for.
 *
 * The rating is in it because the rating is the name, and no measured value
 * ever belongs here - invariant 10. */
#define FW_DEVICE_DESCRIPTION   "Three-phase BLDC inverter, 63 V / 100 A, PCB mounted coaxially behind an "   "outrunner's stator. Instrumentation only: no timer, no PWM, no gate drive."

#endif /* VERSION_H */
