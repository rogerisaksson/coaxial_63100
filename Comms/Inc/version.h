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
#define FW_VERSION_PATCH 0U

#define FW_VERSION_STRING "1.4.0"

/* __DATE__ and __TIME__ make the binary non-reproducible, which is a real cost.
   It is paid deliberately: a production rig that cannot tell which build a
   board is carrying cannot investigate a failure after the fact. */
#define FW_BUILD_STRING (__DATE__ " " __TIME__)

#define FW_DEVICE_NAME "coaxial_63100"
#define FW_MCU_NAME    "STM32H753VIT6"

#endif /* VERSION_H */
