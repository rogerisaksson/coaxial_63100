/**
  ******************************************************************************
  * @file    board_hw.h
  * @brief   The CubeMX-owned peripheral handles, for the board layer only.
  *
  * main.c is where CubeMX defines these, and CubeMX will keep doing so. Rather
  * than let every board file write its own extern, they are declared once here.
  * Nothing above the board layer includes this: the comms stack sees board.h.
  ******************************************************************************
  */
#ifndef BOARD_HW_H
#define BOARD_HW_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi2;

#endif /* BOARD_HW_H */
