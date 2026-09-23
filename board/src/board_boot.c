/**
  ******************************************************************************
  * @file    board_boot.c
  * @brief   What the application knows about the bootloader it came through:
  *          the header the bootloader validates, the handover slot in DTCM
  *          both images share, and the way back.
  ******************************************************************************
  */
#include "board.h"
#include "boot.h"
#include "main.h"
#include "modbus_map.h"
#include "version.h"

/** How long the reply to `stay` has to leave the wire before the reset: a
    frame at 115 200 is two milliseconds; this covers any link. */
#define BOOT_STAY_DELAY_MS   50U

/** The MCU's unique id: three words at UID_BASE. */
#define UID_WORDS            3U

typedef struct
{
  uint32_t magic;
  uint32_t bytes;
  uint32_t version;
  uint32_t type;
} app_header_t;

extern uint32_t _app_size;   /* the linker: the image's bytes in flash */

__attribute__((section(".app_header"), used))
const app_header_t app_header =
{
  BOOT_HEADER_MAGIC,
  (uint32_t)&_app_size,
  (FW_VERSION_MAJOR << 16) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH,
  BOARD_BOOT_TYPE,
};

/** The slot both images share, at the top of DTCM: NOLOAD, so it is exactly
    what the last image left. */
__attribute__((section(".boot_hand")))
boot_hand_t boot_hand;

static struct
{
  bool     stay;
  uint32_t stay_at;
} s;

void Board_BootInit(void)
{
  if (boot_hand.magic != BOOT_HAND_MAGIC)
  {
    return;                          /* no bootloader assigned anything */
  }
  (void)modbus_map_set_unit_id(boot_hand.unit);
  Board_SetTermination((boot_hand.flags & BOOT_FLAG_TERMINATE) != 0U);
}

board_identity_t Board_Identity(void)
{
  board_identity_t id;

  id.assigned = (boot_hand.magic == BOOT_HAND_MAGIC);
  id.type = BOARD_BOOT_TYPE;
  id.unit = modbus_map_unit_id();
  id.position = id.assigned ? boot_hand.position : 0U;
  id.flags = id.assigned ? boot_hand.flags : 0U;
  return id;
}

void Board_Uid(uint8_t *out)
{
  const uint32_t *word = (const uint32_t *)UID_BASE;

  for (uint32_t i = 0U; i < UID_WORDS; i++)
  {
    out[4U * i]      = (uint8_t)(word[i] & 0xFFU);
    out[4U * i + 1U] = (uint8_t)((word[i] >> 8) & 0xFFU);
    out[4U * i + 2U] = (uint8_t)((word[i] >> 16) & 0xFFU);
    out[4U * i + 3U] = (uint8_t)((word[i] >> 24) & 0xFFU);
  }
}

void Board_BootStay(void)
{
  s.stay = true;
  s.stay_at = HAL_GetTick();
}

void Board_BootPoll(void)
{
  if (!s.stay || ((HAL_GetTick() - s.stay_at) < BOOT_STAY_DELAY_MS))
  {
    return;
  }
  boot_hand.stay = BOOT_STAY_MAGIC;
  NVIC_SystemReset();
}
