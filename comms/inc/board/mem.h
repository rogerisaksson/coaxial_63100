/** board/mem.h - the board's memcpy and memset, board_mem.c's: the C library's names. */
#ifndef COMMS_BOARD_MEM_H
#define COMMS_BOARD_MEM_H

#include <stddef.h>

void *memcpy(void *restrict dst, const void *restrict src, size_t n);
void *memset(void *dst, int c, size_t n);

#endif /* COMMS_BOARD_MEM_H */
