/**
  ******************************************************************************
  * @file    modbus_map.h
  * @brief   This board as a Modbus data model.
  *
  * REGISTER MAP
  * ============
  * All addresses are zero-based PDU addresses, i.e. what goes on the wire.
  * A master using one-based "4x/3x" notation must subtract one.
  *
  * INPUT REGISTERS - FC 0x04, read only
  * -----------------------------------
  *   0x0000..0x0006  raw ADC code, one per configured channel, in table order:
  *                     0x0000  ADC3 IN1  PC3_C/PC2_C  diff  Phase U
  *                     0x0001  ADC1 IN3  PA6/PA7      diff  Phase V
  *                     0x0002  ADC2 IN4  PC4/PC5      diff  Phase W
  *                     0x0003  ADC2 IN5  PB1          SE
  *                     0x0004  ADC1 IN9  PB0          SE    NTC
  *                     0x0005  ADC3 IN10 PC0          SE    DC bus
  *                     0x0006  ADC3 IN11 PC1          SE
  *                   Differential codes are signed, two's complement.
  *                   Single-ended codes are unsigned.
  *   0x0010          DC bus millivolts, unsigned
  *   0x0011          NTC temperature in hundredths of a degree C, signed
  *   0x0020,0x0021   SYSCLK in Hz, high word then low word
  *   0x0022,0x0023   HCLK in Hz, high word then low word
  *   0x0030..0x003B  six 32-bit RTU diagnostic counters, high word first:
  *                     0x0030 bus message        0x0034 server message
  *                     0x0032 bus comm error     0x0036 server exception
  *                                               0x0038 server no response
  *                                               0x003A character overrun
  *
  * HOLDING REGISTERS - FC 0x03 / 0x06 / 0x10, read write
  * ----------------------------------------------------
  *   0x0000          unit address, 1..247. Takes effect on the next frame, so
  *                   the response to the write still uses the old address.
  *   0x0001          command register. Reads back 0. Accepted values:
  *                     0x0001  leave Modbus mode, resume the ASCII console
  *                     0x0002  zero the diagnostic counters
  *                   Any other non-zero value is ILLEGAL DATA VALUE.
  *
  * COILS - FC 0x01 / 0x05 / 0x0F, read write
  * -----------------------------------------
  *   0x0000          AFE_ON (PB2). Powers the analog front end AND the voltage
  *                   reference, so with this coil off every ADC channel reads
  *                   exact mid-scale rather than a real measurement.
  *
  * DISCRETE INPUTS - FC 0x02, read only
  * -----------------------------------
  *   0x0000          PE15
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

/**
  * @brief  The data model for this board.
  *
  * Its ctx is the mb_rtu_t whose counters are exposed at 0x0030 and cleared by
  * the command register, so the caller must pass one.
  */
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
