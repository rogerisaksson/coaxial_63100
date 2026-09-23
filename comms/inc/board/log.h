/** board/log.h - what the comms stack needs from board_log.c; included by board.h. */
#ifndef COMMS_BOARD_LOG_H
#define COMMS_BOARD_LOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One measurement, whatever took it. */
#define BOARD_LOG_SOURCE_PHASES 0U   /**< v = U, V, W, TIM1->CNT at latch  */
#define BOARD_LOG_SOURCE_ANGLE  1U   /**< v = value, crc, register         */
#define BOARD_LOG_SOURCE_IMU    2U   /**< v = quaternion i, j, k, real     */
#define BOARD_LOG_SOURCE_DRIVE  3U   /** < v = id, iq in 10 mA, theta_hat as a turn in 65536, innovation in 0.1
    mrad */
#define BOARD_LOG_SOURCES       4U

/** 1024 x 16 B = 16 KB of DTCM, which is 20 ms of history at the injected
    group's 50 kHz - long enough to hold a burst while the host drains it
    fifteen records per round trip. */
#define BOARD_LOG_DEPTH 1024U

/** Arm the ring for a bitmask of sources, and empty it. */
void Board_LogEnable(uint8_t sources, uint32_t min_gap_cycles);
uint8_t Board_LogSources(void);

/** Called by the producers. Silently ignored for a source not armed. */
void Board_LogPush(uint8_t source, const int16_t *v, uint8_t n);

uint16_t Board_LogCount(void);
uint32_t Board_LogDropped(void);

/** Pushes refused by the rate limit, as opposed to lost to a full ring. */
uint32_t Board_LogThinned(void);

/** Copy out up to `max` oldest-first, and free their slots. */
uint16_t Board_LogTake(board_sample_t *out, uint16_t max);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_LOG_H */
