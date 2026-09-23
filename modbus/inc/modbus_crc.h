/** modbus_crc.h - CRC-16/MODBUS for RTU framing. */
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Compute the CRC-16/MODBUS of a byte range. */
uint16_t modbus_crc16(const uint8_t *data, size_t len);

/** Append the CRC of buf[0..len-1] at buf[len], low byte first.
    @return The new total length, len + 2. The caller must guarantee capacity. */
size_t modbus_crc_append(uint8_t *buf, size_t len);

/** Check a complete frame whose last two bytes are its CRC.
    @return Non-zero if the CRC matches. Frames shorter than 3 bytes fail. */
int modbus_crc_check(const uint8_t *frame, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_CRC_H */
