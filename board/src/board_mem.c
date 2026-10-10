/** board_mem.c - memcpy and memset a word a turn, from ITCM. */

/* newlib-nano's moved a byte a turn, a taken branch each, behind a veneer from D2 SRAM
   (tools/target/hot.py, 2026-10-10). */
#include "board/mem.h"

#include <stdint.h>

/* A word that may alias whatever it copies. */
typedef uint32_t __attribute__((may_alias)) word_t;

/* Not turned back into calls to themselves: a byte loop is GCC's memcpy. */
#define MEM_OWN __attribute__((optimize("no-tree-loop-distribute-patterns")))

MEM_OWN void *memcpy(void *restrict dst, const void *restrict src, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;

  if ((((uintptr_t)d ^ (uintptr_t)s) & 3U) == 0U)
  {
    for (; (n > 0U) && (((uintptr_t)d & 3U) != 0U); n--)
    {
      *d++ = *s++;
    }
    word_t *dw = (word_t *)(void *)d;
    const word_t *sw = (const word_t *)(const void *)s;

    for (; n >= 16U; n -= 16U)
    {
      dw[0] = sw[0];
      dw[1] = sw[1];
      dw[2] = sw[2];
      dw[3] = sw[3];
      dw += 4;
      sw += 4;
    }
    for (; n >= 4U; n -= 4U)
    {
      *dw++ = *sw++;
    }
    d = (uint8_t *)dw;
    s = (const uint8_t *)sw;
  }
  for (; n > 0U; n--)
  {
    *d++ = *s++;
  }
  return dst;
}

MEM_OWN void *memset(void *dst, int c, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  const uint8_t byte = (uint8_t)c;
  const uint32_t word = 0x01010101U * byte;

  for (; (n > 0U) && (((uintptr_t)d & 3U) != 0U); n--)
  {
    *d++ = byte;
  }
  word_t *dw = (word_t *)(void *)d;

  for (; n >= 16U; n -= 16U)
  {
    dw[0] = word;
    dw[1] = word;
    dw[2] = word;
    dw[3] = word;
    dw += 4;
  }
  for (; n >= 4U; n -= 4U)
  {
    *dw++ = word;
  }
  d = (uint8_t *)dw;
  for (; n > 0U; n--)
  {
    *d++ = byte;
  }
  return dst;
}
