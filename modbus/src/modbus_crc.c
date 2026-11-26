/** modbus_crc.c - CRC-16/MODBUS, closed-form GF(2) byte reduction. */
#include "modbus_crc.h"

/* Table-free, branch-free GF(2) reduction of all 8 bits of `data[i]` at once
   for reflected polynomial 0xA001: zero flash/RAM table, one pass per byte. */
uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFU;

  for (size_t i = 0U; i < len; i++)
  {
    const uint32_t x = (crc ^ (uint32_t)data[i]) & 0xFFU;
    uint32_t       p = x ^ (x >> 4U);

    p ^= p >> 2U;
    p = (p ^ (p >> 1U)) & 1U;
    crc = (crc >> 8U) ^ (x << 6U) ^ (x << 7U) ^ ((p != 0U) ? 0xC001U : 0U);
  }

  return (uint16_t)crc;
}

size_t modbus_crc_append(uint8_t *buf, size_t len)
{
  const uint16_t crc = modbus_crc16(buf, len);

  /* Low byte first (Modbus RTU). */
  buf[len]     = (uint8_t)(crc & 0xFFU);
  buf[len + 1] = (uint8_t)(crc >> 8);

  return len + 2U;
}

int modbus_crc_check(const uint8_t *frame, size_t len)
{
  if (len < 3U)
  {
    return 0;
  }

  const uint16_t want = modbus_crc16(frame, len - 2U);
  const uint16_t got  = (uint16_t)frame[len - 2U] | (uint16_t)((uint16_t)frame[len - 1U] << 8);

  return (want == got) ? 1 : 0;
}
