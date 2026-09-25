/** board_flash.c - The calibration record's sector: bank 2, sector 7, 0x081E0000..0x081FFFFF. */
#include "board.h"
#include "board_hw.h"

#include <stddef.h>

const void *Board_FlashRecord(void)
{
  return (const void *)BOOT_RECORD_BASE;
}

bool Board_FlashRecordWrite(const void *image, uint32_t bytes)
{
  FLASH_EraseInitTypeDef erase = {0};
  const uint8_t *from = (const uint8_t *)image;
  uint32_t sector_error = 0U;
  bool     ok = true;

  if ((image == NULL) || ((bytes % BOARD_FLASH_WORD_BYTES) != 0U) ||
      (HAL_FLASH_Unlock() != HAL_OK))
  {
    return false;
  }

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.Banks        = FLASH_BANK_2;
  erase.Sector       = FLASH_SECTOR_7;
  erase.NbSectors    = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    ok = false;
  }

  for (uint32_t at = 0U; ok && (at < bytes); at += BOARD_FLASH_WORD_BYTES)
  {
    /* On H7 the third argument is the ADDRESS of the 32-byte source, not the data. */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, BOOT_RECORD_BASE + at,
                          (uint32_t)(uintptr_t)&from[at]) != HAL_OK)
    {
      ok = false;
    }
  }

  (void)HAL_FLASH_Lock();
  return ok;
}
