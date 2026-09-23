/** board/selftest.h - what the comms stack needs from board_selftest.c; included by board.h. */
#ifndef COMMS_BOARD_SELFTEST_H
#define COMMS_BOARD_SELFTEST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One result from the board's self test. */
#define BOARD_CHECK_PASS 0U
#define BOARD_CHECK_FAIL 1U
#define BOARD_CHECK_INFO 2U

#define BOARD_SELFTEST_MAX 16U

typedef struct
{
  const char *name;
  uint8_t     status;
  int32_t     value;   /**< meaning is per check; 0 where there is none */
} board_check_t;

/** Run every self check and fill @p out.
    @return Number of checks written, never more than @p capacity. */
uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity);

/** Leave the binary link and resume the ASCII console, once the reply is out. */
void Board_RequestConsoleMode(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_SELFTEST_H */
