/**
  ******************************************************************************
  * @file    board_cal.c
  * @brief   The scaling parameters and per-channel corrections, on the board.
  *
  * Here and not in a host because a calibration belongs to ONE physical
  * board: a host carrying it answers for the wrong board the moment it is
  * pointed at a second.
  *
  * It judges nothing. A scale factor says what a code is worth, never
  * whether it is acceptable (invariant 10).
  *
  * Integers throughout, in the unit that makes them integers: microhms, ppm,
  * microvolts, centikelvin. The wire bans floating point.
  *
  * Last sector of bank 2 - the image is 92 KB in bank 1, so an erase here
  * does not stall the core fetching its own instructions.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include "modbus_crc.h"

#include <stddef.h>
#include <string.h>

/* Bank 2, sector 7: 0x081E0000..0x081FFFFF. Nothing else is linked here -
   STM32H753xx_FLASH.ld places .text and .rodata from 0x08000000 and the image
   is under 128 KB. */
#define CAL_FLASH_ADDR   0x081E0000UL
#define CAL_FLASH_SECTOR FLASH_SECTOR_7
#define CAL_FLASH_BANK   FLASH_BANK_2

/* 'CX63' - a sector that has never been written reads 0xFF everywhere, and a
   magic is how that is told from a record whose fields happen to be large. */
#define CAL_MAGIC   0x43583633UL
/* 2 since the +5 and gate-supply senses were added: the record carries one
   trim per channel, so its length moved. A stored version 1 is rejected by
   the check below and the defaults are used, which is the right answer -
   trims measured against seven channels do not index nine. */
/* 4: the thermal envelope joined the record. A stored 3 is refused
   rather than read with the new fields as whatever flash held. */
/* 5: the half-bridge dead time joined the record. A stored 4 is refused
   rather than read with the new field as whatever flash held. */
#define CAL_VERSION 5U

/* H7 programs a 256-bit flash word at a time, so the image written is padded
   to a multiple of 32 bytes. sizeof(board_cal_t) is 104 today. */
#define CAL_WORD_BYTES 32U
#define CAL_IMAGE_BYTES (((sizeof(board_cal_t) + CAL_WORD_BYTES - 1U) / \
                          CAL_WORD_BYTES) * CAL_WORD_BYTES)

static board_cal_t s_cal;

/* Compiled-in defaults: what the schematic says, traced 2026-08-26 from
   electronics/Coaxial 63100 Schematics.pdf. Every one of them is arithmetic
   from a resistor value, and not one has been measured - which is exactly
   why they are defaults a rig can overwrite rather than constants it cannot.
   The derivations are in docs/HARDWARE.md. */
static const board_cal_t CAL_DEFAULTS =
{
  .magic            = CAL_MAGIC,
  .version          = CAL_VERSION,
  .channels         = BOARD_CAL_CHANNELS,

  /* The thermal envelope, centi-degrees C per node in thermal_node_t order:
     drivers, phases, mcu, regulators, afe, board.

       phases  12500  IAUCN10S7N021 Tj max - docs/HARDWARE.md
       mcu     12500  STM32H753 Tj max
       board   10500  ESTIMATE - laminate, well under any part on it
       others  12500  ESTIMATE - those datasheets are not in this tree

     Derate at 85 %: the board's constant is 6.8 minutes but a deep burst
     moves a node in seconds, so a throttle that waits for the ceiling
     arrives after it. */
  .soa_limit_centi  = { 12500, 12500, 12500, 12500, 12500, 10500 },
  .soa_throttle_ppm = 850000UL,
  .vref_uv          = 3300000UL,      /* U2 REF2033, 3.3 V +/-0.05 %       */
  .shunt_uohm       = 3500UL,         /* RU1 || RU2, 7 mohm each           */
  .amp_gain_ppm     = 4545455UL,      /* THS4551, Rf 1.5k / Rg 330         */
  .bus_r_top_ohm    = 49900UL,        /* R12                               */
  .bus_r_bottom_ohm = 2200UL,         /* R11                               */
  .r5_r_top_ohm     = 10000UL,        /* R113 element 2, +5 to PA4         */
  .r5_r_bottom_ohm  = 10000UL,        /* R113 element 1, PA4 to GND        */
  .vg_r_top_ohm     = 57000UL,        /* R119 47k + R113 element 3 10k     */
  .vg_r_bottom_ohm  = 10000UL,        /* R113 element 4, PA5 to GND        */

  /* 30 ns, asked for 2026-08-29. The arithmetic it replaces: 59.4 ns of
     worst-corner gate overlap plus the 2EDL8034's 6 ns TDMOFF is about
     65 ns needed, and 80 ns was fitted against that. Kept here because
     it is the number a bench can change without a rebuild, and the
     firmware still refuses anything under its own 20 ns floor. */
  .deadtime_ns      = 30UL,
  .ntc_r25_ohm      = 10000UL,        /* NCU18XH103D60RB                   */
  .ntc_beta_mk      = 3380000UL,      /* B25/50 = 3380 K, in milli-kelvin  */
  .ntc_rfixed_ohm   = 10000UL,        /* R100, ERA-3AEB103V 0.1 %          */
  .ntc_t25_ck       = 29815UL,        /* 298.15 K                          */
  .chan             = { { 0, 0 } },   /* no offset, no gain trim           */
};

/* CRC-16 over everything ahead of the crc field itself. Reused from the
   Modbus core rather than adding a second checksum: it is already built, it
   is already host-tested, and it is hardware-free. */
static uint16_t cal_crc(const board_cal_t *cal)
{
  return modbus_crc16((const uint8_t *)cal,
                      offsetof(board_cal_t, crc));
}

static bool cal_valid(const board_cal_t *cal)
{
  return (cal->magic == CAL_MAGIC) &&
         (cal->version == CAL_VERSION) &&
         (cal->channels == BOARD_CAL_CHANNELS) &&
         (cal->crc == cal_crc(cal));
}

void Board_CalInit(void)
{
  const board_cal_t *stored = (const board_cal_t *)CAL_FLASH_ADDR;

  if (cal_valid(stored))
  {
    s_cal = *stored;
    return;
  }

  /* Never written, or written by an older layout, or corrupted. Defaults are
     the honest answer in all three cases; the difference is only visible in
     Board_CalStored(), which a host can ask. */
  s_cal = CAL_DEFAULTS;
  s_cal.crc = cal_crc(&s_cal);
}

const board_cal_t *Board_Cal(void)
{
  return &s_cal;
}

bool Board_CalStored(void)
{
  return cal_valid((const board_cal_t *)CAL_FLASH_ADDR);
}

void Board_CalDefaults(void)
{
  s_cal = CAL_DEFAULTS;
  s_cal.crc = cal_crc(&s_cal);
}

bool Board_CalLoad(void)
{
  const board_cal_t *stored = (const board_cal_t *)CAL_FLASH_ADDR;

  if (!cal_valid(stored))
  {
    return false;
  }

  s_cal = *stored;
  return true;
}

bool Board_CalSave(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint8_t  image[CAL_IMAGE_BYTES];
  uint32_t sector_error = 0U;
  bool     ok = true;

  s_cal.crc = cal_crc(&s_cal);

  /* Pad with 0xFF, which is what an erased cell reads, so the tail of the
     last flash word is indistinguishable from never having been written. */
  memset(image, 0xFF, sizeof(image));
  memcpy(image, &s_cal, sizeof(s_cal));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.Banks        = CAL_FLASH_BANK;
  erase.Sector       = CAL_FLASH_SECTOR;
  erase.NbSectors    = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    ok = false;
  }

  for (uint32_t at = 0U; ok && (at < CAL_IMAGE_BYTES); at += CAL_WORD_BYTES)
  {
    /* On H7 the third argument is the ADDRESS of the 32-byte source, not the
       data. Passing the data is the mistake this comment exists to prevent. */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, CAL_FLASH_ADDR + at,
                          (uint32_t)(uintptr_t)&image[at]) != HAL_OK)
    {
      ok = false;
    }
  }

  (void)HAL_FLASH_Lock();

  /* Read it back rather than trust the programmer's return: a save that
     reports success and left the sector unreadable is the failure this whole
     record exists to survive. */
  return ok && cal_valid((const board_cal_t *)CAL_FLASH_ADDR);
}

/* Which scalar an id names. One table for the setter and the getter, so the
   two cannot drift into disagreeing about what id 6 is. */
static uint32_t *cal_field(uint8_t id)
{
  switch (id)
  {
    case BOARD_CAL_VREF_UV:      return &s_cal.vref_uv;
    case BOARD_CAL_SHUNT_UOHM:   return &s_cal.shunt_uohm;
    case BOARD_CAL_AMP_GAIN_PPM: return &s_cal.amp_gain_ppm;
    case BOARD_CAL_BUS_R_TOP:    return &s_cal.bus_r_top_ohm;
    case BOARD_CAL_BUS_R_BOTTOM: return &s_cal.bus_r_bottom_ohm;
    case BOARD_CAL_NTC_R25:      return &s_cal.ntc_r25_ohm;
    case BOARD_CAL_NTC_BETA_MK:  return &s_cal.ntc_beta_mk;
    case BOARD_CAL_NTC_RFIXED:   return &s_cal.ntc_rfixed_ohm;
    case BOARD_CAL_NTC_T25_CK:   return &s_cal.ntc_t25_ck;
    case BOARD_CAL_R5_R_TOP:     return &s_cal.r5_r_top_ohm;
    case BOARD_CAL_R5_R_BOTTOM:  return &s_cal.r5_r_bottom_ohm;
    case BOARD_CAL_VG_R_TOP:     return &s_cal.vg_r_top_ohm;
    case BOARD_CAL_VG_R_BOTTOM:  return &s_cal.vg_r_bottom_ohm;
    case BOARD_CAL_DEADTIME_NS:  return &s_cal.deadtime_ns;
    default:                     return NULL;
  }
}

bool Board_CalSetParam(uint8_t id, uint32_t value)
{
  uint32_t *field = cal_field(id);

  if (field == NULL)
  {
    return false;
  }

  /* Every one of the nine is a divisor or a multiplicand somewhere, so zero
     is refused for all of them alike. Checked BEFORE the assignment: the
     version that assigned first and rolled back after did so by reloading
     flash, which does nothing at all on a board whose record has never been
     saved - measured by test_conformance.py, and it left vref at zero. */
  if (value == 0U)
  {
    return false;
  }

  *field = value;
  return true;
}

bool Board_CalGetParam(uint8_t id, uint32_t *value)
{
  const uint32_t *field = cal_field(id);

  if ((field == NULL) || (value == NULL))
  {
    return false;
  }

  *value = *field;
  return true;
}

bool Board_CalSetLimit(uint8_t node, int32_t limit_centi)
{
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    return false;
  }
  s_cal.soa_limit_centi[node] = limit_centi;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}


bool Board_CalSetThrottle(uint32_t ppm)
{
  if ((ppm == 0U) || (ppm >= 1000000U))
  {
    return false;
  }
  s_cal.soa_throttle_ppm = ppm;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}


bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm)
{
  if (index >= BOARD_CAL_CHANNELS)
  {
    return false;
  }

  /* A gain trim of -1e6 ppm is a scale factor of zero, and everything below
     it changes the sign of the reading. Both are corrections nobody makes on
     purpose, and both are indistinguishable from a broken channel later. */
  if (gain_ppm <= -1000000)
  {
    return false;
  }

  s_cal.chan[index].offset_raw = offset_raw;
  s_cal.chan[index].gain_ppm = gain_ppm;
  return true;
}

bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm)
{
  if ((index >= BOARD_CAL_CHANNELS) || (offset_raw == NULL) ||
      (gain_ppm == NULL))
  {
    return false;
  }

  *offset_raw = s_cal.chan[index].offset_raw;
  *gain_ppm = s_cal.chan[index].gain_ppm;
  return true;
}

int32_t Board_CalApply(uint8_t index, int32_t raw)
{
  if (index >= BOARD_CAL_CHANNELS)
  {
    return raw;
  }

  const board_cal_chan_t *c = &s_cal.chan[index];
  const int64_t corrected = (int64_t)raw - (int64_t)c->offset_raw;

  if (c->gain_ppm == 0)
  {
    return (int32_t)corrected;
  }

  /* 64-bit because a full-scale code times a million overflows 32 bits at
     4295 counts, which every channel here exceeds. */
  return (int32_t)(corrected +
                   (corrected * (int64_t)c->gain_ppm) / 1000000);
}
