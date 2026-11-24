/** say.c - the ASCII console's words written without stdio or heap: text, decimals, hex. */
#include "say.h"

#include "board.h"

/* The console wrote through printf: stdio, and newlib's heap under it (stdout's buffer at its first
   call), no firmware's. Each word goes out through the board's console as it is made, the bytes
   the same. */

void say_text(const char *text)
{
  uint16_t n = 0U;
  while (text[n] != '\0')
  {
    n++;
  }
  Board_ConsoleWrite((const uint8_t *)text, n);
}

void say_u32(uint32_t value)
{
  uint8_t  digits[10];
  uint32_t rest = value;
  uint8_t  n = 0U;
  do
  {
    n++;
    digits[10U - n] = (uint8_t)((uint32_t)'0' + (rest % 10U));
    rest /= 10U;
  } while (rest != 0U);
  Board_ConsoleWrite(&digits[10U - n], n);
}

void say_hex(uint32_t value, uint8_t width)
{
  static const uint8_t HEX[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                   '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  uint8_t  digits[8];
  uint32_t rest = value;
  uint8_t  n = 0U;
  do
  {
    n++;
    digits[8U - n] = HEX[rest & 0xFU];
    rest >>= 4U;
  } while (((rest != 0U) || (n < width)) && (n < 8U));
  Board_ConsoleWrite(&digits[8U - n], n);
}
