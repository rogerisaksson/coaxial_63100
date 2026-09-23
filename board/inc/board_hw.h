/**
  ******************************************************************************
  * @file    board_hw.h
  * @brief   The CubeMX-owned peripheral handles, for the board layer only.
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
extern SPI_HandleTypeDef hspi4;

#endif /* BOARD_HW_H */
