/**
  ******************************************************************************
  * @file    modbus_crc.h
  * @brief   CRC-16/MODBUS for RTU framing.
  *
  * Reflected polynomial 0xA001, init 0xFFFF, no final XOR, no result
  * reflection. On the wire it goes LOW BYTE FIRST - the opposite of every
  * other 16-bit field in a PDU. That reversal is the most common RTU bug, so
  * the byte order lives in modbus_crc_append(), not at each call site.
  *
  * Checked against the catalogue value: CRC of the nine bytes "123456789"
  * is 0x4B37.
  ******************************************************************************
  */
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Compute the CRC-16/MODBUS of a byte range. */
uint16_t modbus_crc16(const uint8_t *data, size_t len);

/**
  * @brief  Append the CRC of buf[0..len-1] at buf[len], low byte first.
  * @return The new total length, len + 2. The caller must guarantee capacity.
  */
size_t modbus_crc_append(uint8_t *buf, size_t len);

/**
  * @brief  Check a complete frame whose last two bytes are its CRC.
  * @return Non-zero if the CRC matches. Frames shorter than 3 bytes fail.
  */
int modbus_crc_check(const uint8_t *frame, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_CRC_H */
