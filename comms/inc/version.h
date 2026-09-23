/** version.h - Firmware and protocol versions. */
#ifndef VERSION_H
#define VERSION_H

#define FW_VERSION_MAJOR 1U
#define FW_VERSION_MINOR 6U
#define FW_VERSION_PATCH 0U

#define FW_VERSION_STRING "1.6.0"

/* Costs reproducibility, deliberately: a rig that cannot tell which build a
   board carries cannot investigate a failure after the fact. */
#define FW_BUILD_STRING (__DATE__ " " __TIME__)

#define FW_DEVICE_NAME "coaxial_63100"
#define FW_MCU_NAME    "STM32H753VIT6"

/* One word, so a host can group a bus by it: the node that is not an
   inverter is the one worth seeing in a list. */
#define FW_DEVICE_TYPE "bldc_inverter"

/* A name is not a description: "coaxial_63100" picks a codec and says
   nothing about what is on the other end. */
#define FW_DEVICE_DESCRIPTION   "Three-phase BLDC inverter, 63 V / 100 A, PCB mounted coaxially behind an "   "outrunner's stator. Gate stage armed on request; drive written, unproven on a motor."

#endif /* VERSION_H */
