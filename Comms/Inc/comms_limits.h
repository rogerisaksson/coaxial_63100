/**
  ******************************************************************************
  * @file    comms_limits.h
  * @brief   The WIRE's fixed numbers - what a reply carries, and how long the
  *          command layer waits for a part to answer.
  *
  * Split from the drivers' `board_limits.h` so the includes run one way:
  * this file may reach down into Board/, and nothing in Board/ reaches up.
  *
  * Where a number here has to hold against one down there, it is asserted
  * rather than remembered - see IMU_CARGO.
  ******************************************************************************
  */
#ifndef COMMS_LIMITS_H
#define COMMS_LIMITS_H

#include "board_limits.h"


/* ---- THE SERIAL LINK --------------------------------------------------- */

/* One rate for all three. CubeMX carries 9216000 on the two RS485 ports,

   which is not a Modbus rate on any bus - the runtime value is this one. */

#define DEV_UART_BAUD 115200U



/* One Modbus RTU frame is 256 bytes at most. Sized to hold a whole one so a

   frame that begins while the main loop is inside a long board call is not

   half lost - which is the failure this ring exists to prevent, not a

   throughput problem. */

#define DEV_RING 256U



#define LINK_BITS_PER_CHAR 11U

/* ---- THE IMU, THROUGH A MODBUS REPLY ----------------------------------- */

/* What one Modbus reply can carry. The part's advertisement is longer than

   this - 276 bytes, measured - so a cargo that big arrives truncated and the

   board layer drops the rest rather than leaving it to desynchronise the

   next read. A bring-up asks for reports, which fit. */

#define IMU_CARGO 200U

/* Per read attempt, and eight of them: 40 ms worst case for a part that

   never answers. The wait pumps the STO keepalive, so a slow part does

   not stop the charge pump. */

#define IMU_ANSWER_WAIT_MS 5U

/* The relation that matters, and the reason both files exist in one place:
   a cargo larger than the reply is truncated, and the board layer drops the
   rest rather than leaving it to desynchronise the next read. Asserted, not
   remembered - it was invisible while the two lived in two layers. */
_Static_assert(IMU_CARGO <= IMU_BUF,
               "a Modbus reply cannot carry more than the driver read");

#endif /* COMMS_LIMITS_H */
