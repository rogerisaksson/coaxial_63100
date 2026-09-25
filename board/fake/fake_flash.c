/** fake_flash.c - The record sector on the host: RAM, erased at first sight, kept while loaded. */
#include "board/flash.h"

#include <stdalign.h>
#include <stddef.h>
#include <string.h>

#define FAKE_SECTOR_BYTES 4096U

static struct
{
  bool ready;
  alignas(8) uint8_t cells[FAKE_SECTOR_BYTES];
} s;

static uint8_t *fake_sector(void)
{
  if (!s.ready)
  {
    memset(s.cells, 0xFF, sizeof s.cells);
    s.ready = true;
  }
  return s.cells;
}

const void *Board_FlashRecord(void)
{
  return fake_sector();
}

bool Board_FlashRecordWrite(const void *image, uint32_t bytes)
{
  uint8_t *cells = fake_sector();

  if ((image == NULL) || ((bytes % BOARD_FLASH_WORD_BYTES) != 0U) || (bytes > sizeof s.cells))
  {
    return false;
  }
  memset(cells, 0xFF, sizeof s.cells);
  memcpy(cells, image, bytes);
  return true;
}
