/** comms_limits.h - The WIRE's fixed numbers - what a reply carries, and how
    long the command layer waits for a part to answer. */
#ifndef COMMS_LIMITS_H
#define COMMS_LIMITS_H

#include "board_limits.h"

/* ---- THE SERIAL LINK --------------------------------------------------- */

/* One rate for all three. */

#define DEV_UART_BAUD 115200U

/* One Modbus RTU frame is 256 bytes at most. */

#define DEV_RING 256U

#define LINK_BITS_PER_CHAR 11U

/* ---- THE IMU, THROUGH A MODBUS REPLY ----------------------------------- */

/* What one Modbus reply can carry. */

#define IMU_CARGO 200U

/* Per read attempt, and eight of them: 40 ms worst case for a part that
   never answers. */

#define IMU_ANSWER_WAIT_MS 5U

/* The relation that matters, and the reason both files exist in one place: a
   cargo larger than the reply is truncated, and the board layer drops the
   rest rather than leaving it to desynchronise the next read. */
_Static_assert(IMU_CARGO <= IMU_BUF,
               "a Modbus reply cannot carry more than the driver read");

#endif /* COMMS_LIMITS_H */
