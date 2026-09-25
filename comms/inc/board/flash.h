/** board/flash.h - the calibration record's flash sector, for board_cal.c; included by board.h. */
#ifndef COMMS_BOARD_FLASH_H
#define COMMS_BOARD_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Bytes programmed at a time (one 256-bit H7 flash word); an image is a multiple. */
#define BOARD_FLASH_WORD_BYTES 32U

/** The record sector as mapped; an erased cell reads 0xFF. */
const void *Board_FlashRecord(void);

/** Erase the sector and program `bytes` of `image`, a multiple of BOARD_FLASH_WORD_BYTES. */
bool Board_FlashRecordWrite(const void *image, uint32_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_FLASH_H */
