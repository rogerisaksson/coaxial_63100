/** native_a1335.c - The Allegro A1335 on SPI4 natively: Coaxial63100_A1335.cs in C. */

/* 20-bit packets as board_angle.c sends them, four 5-bit words, MSB first. A read is answered
   in the next packet - the register's 16 bits, then a 4-bit CRC (x^4 + x + 1, seed 0xF). ANG
   is the shaft's mechanical angle in twelve bits (native.c's), TSEN its die's temperature in
   eighths of a kelvin, FIELD the stand-in's magnet; the rest reads zero. Unpowered - AFE_ON
   low - it clocks out all ones, as an absent part does. */
#include "native.h"

#include <math.h>

#define A1335_WORDS         4U
#define A1335_WORD_BITS     5U
#define A1335_WORD_MASK     0x1FU
#define A1335_ADDRESS_SHIFT 12U
#define A1335_DATA_SHIFT    4U
#define A1335_RW_SHIFT      18U
#define A1335_ALL_ONES      0xFFFFFU
#define A1335_COUNTS        4096
#define A1335_ANG           0x20U
#define A1335_TSEN          0x28U
#define A1335_FIELD         0x2AU
#define A1335_GAUSS         380.0

static struct
{
  uint32_t words;
  uint32_t command;
  uint32_t reply;
} a;

void a1335_open(void)
{
  a.words = 0U;
  a.command = 0U;
  a.reply = A1335_ALL_ONES;
}

static uint16_t a1335_register(uint32_t reg)
{
  switch (reg)
  {
    case A1335_ANG:
    {
      const double turns = native_shaft_degrees() / 360.0;

      return (uint16_t)((int)floor((turns - floor(turns)) * A1335_COUNTS) & (A1335_COUNTS - 1));
    }
    case A1335_TSEN:
    {
      const double eighths = floor((native_angle_celsius() + 273.15) * 8.0 + 0.5);

      return (uint16_t)((eighths < 0.0) ? 0.0 : (eighths > 4095.0) ? 4095.0 : eighths);
    }
    case A1335_FIELD:
      return (uint16_t)A1335_GAUSS;
    default:
      return 0U;
  }
}

/* CRC-4, x^4 + x + 1, seed 0xF, over the 16 data bits MSB first. */
static uint32_t a1335_crc4(uint16_t value)
{
  uint32_t crc = 0xFU;

  for (int bit = 15; bit >= 0; bit--)
  {
    const uint32_t top = ((crc >> 3) & 1U) ^ (((uint32_t)value >> (uint32_t)bit) & 1U);

    crc = ((crc << 1) & 0xFU) ^ ((top != 0U) ? 0x3U : 0U);
  }
  return crc;
}

static void a1335_packet(uint32_t packet)
{
  const uint32_t reg = (packet >> A1335_ADDRESS_SHIFT) & 0x3FU;
  const bool write = ((packet >> A1335_RW_SHIFT) & 1U) != 0U;

  if (write || !native_powered())
  {
    a.reply = A1335_ALL_ONES;
    return;
  }
  const uint16_t value = a1335_register(reg);

  a.reply = ((uint32_t)value << A1335_DATA_SHIFT) | a1335_crc4(value);
}

/** Chip select, PE4: low selects; rising ends the packet. */
void a1335_select(bool level)
{
  if (!level)
  {
    a.words = 0U;
    a.command = 0U;
    return;
  }
  if (a.words == A1335_WORDS)
  {
    a1335_packet(a.command);
  }
  a.words = 0U;
}

uint8_t a1335_transmit(uint8_t word)
{
  const uint32_t at = (a.words < A1335_WORDS) ? a.words : (A1335_WORDS - 1U);
  const uint32_t shift = A1335_WORD_BITS * (A1335_WORDS - 1U - at);
  const uint32_t reply = native_powered() ? a.reply : A1335_ALL_ONES;

  a.command = (a.command << A1335_WORD_BITS) | ((uint32_t)word & A1335_WORD_MASK);
  a.words++;
  return (uint8_t)((reply >> shift) & A1335_WORD_MASK);
}
