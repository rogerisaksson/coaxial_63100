/**
  ******************************************************************************
  * @file    modbus_crc.c
  * @brief   CRC-16/MODBUS, bit-serial.
  ******************************************************************************
  */
#include "modbus_crc.h"

/* Bit-serial rather than a 512-byte table. */
uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;

  for (size_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i];

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
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
