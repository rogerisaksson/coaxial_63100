/** board/clock.h - what the comms stack needs from board_clock.c; included by board.h. */
#ifndef COMMS_BOARD_CLOCK_H
#define COMMS_BOARD_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t at;                /**< Board_Cycles() at capture, raw ticks    */
  uint8_t  source;
  uint8_t  seq;               /**< per source, so a dropped run is visible */
  int16_t  v[4];
} board_sample_t;

uint32_t Board_SysClkHz(void);
uint32_t Board_HclkHz(void);

/** Enable the cycle counter the comms stack uses as its timebase. */
void Board_TimebaseInit(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_CLOCK_H */
