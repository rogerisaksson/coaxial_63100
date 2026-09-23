/**
  ******************************************************************************
  * @file    board.h
  * @brief   Everything the comms stack needs from this board, one header per
  *          board file, in dependency order.
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

#include "board/io.h"
#include "board/clock.h"
#include "board/adc.h"
#include "board/cal.h"
#include "board/pwm.h"
#include "board/sync.h"
#include "board/log.h"
#include "board/daq.h"
#include "board/imu.h"
#include "board/angle.h"
#include "board/thermal.h"
#include "board/power.h"
#include "board/drive.h"
#include "board/selftest.h"
#include "board/handover.h"

#endif /* BOARD_H */
