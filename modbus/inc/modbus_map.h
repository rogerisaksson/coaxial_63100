/**
  ******************************************************************************
  * @file    modbus_map.h
  * @brief   This board as a Modbus data model.
  ******************************************************************************
  */
#ifndef MODBUS_MAP_H
#define MODBUS_MAP_H

#include "modbus_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Input register addresses. */
#define MB_IREG_ADC_BASE       0x0000U
#define MB_IREG_ADC_COUNT_MAX  0x0010U
#define MB_IREG_DCBUS_MV       0x0010U
#define MB_IREG_NTC_CENTI_C    0x0011U
#define MB_IREG_SYSCLK_HI      0x0020U
#define MB_IREG_HCLK_HI        0x0022U
#define MB_IREG_COUNTERS_BASE  0x0030U
#define MB_IREG_COUNTERS_WORDS 12U

/* Holding register addresses. */
#define MB_HREG_UNIT_ID        0x0000U
#define MB_HREG_COMMAND        0x0001U
#define MB_HREG_COUNT          2U

/* Command register values. */
#define MB_CMD_CONSOLE_MODE    0x0001U
#define MB_CMD_CLEAR_COUNTERS  0x0002U

/* Bit space. */
#define MB_COIL_AFE_ON         0x0000U
#define MB_COIL_COUNT          1U
#define MB_DIN_PE15            0x0000U
#define MB_DIN_COUNT           1U

/** The data model for this board. */
const mb_data_model_t *modbus_map_model(
    void *rtu_ctx,
    mb_exception_t (*user_function)(void *ctx, uint8_t fc,
                                    const uint8_t *req, size_t req_len,
                                    uint8_t *rsp, size_t rsp_cap, size_t *rsp_len));

/** Current unit address, as possibly changed through holding register 0. */
uint8_t modbus_map_unit_id(void);

/** Set the unit address; values outside 1..247 are rejected. */
bool modbus_map_set_unit_id(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_MAP_H */
