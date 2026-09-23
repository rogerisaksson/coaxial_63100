/** modbus_slave.h - Portable Modbus server (slave) PDU engine. */
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Function codes this engine implements. */
#define MB_FC_READ_COILS            0x01U
#define MB_FC_READ_DISCRETE_INPUTS  0x02U
#define MB_FC_READ_HOLDING_REGS     0x03U
#define MB_FC_READ_INPUT_REGS       0x04U
#define MB_FC_WRITE_SINGLE_COIL     0x05U
#define MB_FC_WRITE_SINGLE_REG      0x06U
#define MB_FC_WRITE_MULTIPLE_COILS  0x0FU
#define MB_FC_WRITE_MULTIPLE_REGS   0x10U
#define MB_FC_REPORT_SERVER_ID      0x11U

/** Exception codes. */
typedef enum
{
  MB_EX_NONE                  = 0x00,
  MB_EX_ILLEGAL_FUNCTION      = 0x01,
  MB_EX_ILLEGAL_DATA_ADDRESS  = 0x02,
  MB_EX_ILLEGAL_DATA_VALUE    = 0x03,
  MB_EX_SERVER_DEVICE_FAILURE = 0x04,
  MB_NO_REPLY                 = 0xFF
} mb_exception_t;

/** Which of the four Modbus data tables an access refers to. */
typedef enum
{
  MB_TABLE_COIL,            /**< read/write bit */
  MB_TABLE_DISCRETE_INPUT,  /**< read-only bit */
  MB_TABLE_HOLDING_REG,     /**< read/write 16-bit */
  MB_TABLE_INPUT_REG        /**< read-only 16-bit */
} mb_table_t;

/** Application data model. */
typedef struct
{
  mb_exception_t (*validate_range)(void *ctx, mb_table_t table, uint16_t addr,
                                   uint16_t qty, bool for_write);
  mb_exception_t (*read_reg)(void *ctx, mb_table_t table, uint16_t addr, uint16_t *out);
  mb_exception_t (*write_reg)(void *ctx, uint16_t addr, uint16_t value);
  mb_exception_t (*read_bit)(void *ctx, mb_table_t table, uint16_t addr, bool *out);
  mb_exception_t (*write_bit)(void *ctx, uint16_t addr, bool value);

  /** Optional: would write_reg accept this value, without applying it? */
  mb_exception_t (*validate_reg_value)(void *ctx, uint16_t addr, uint16_t value);

  /** Report Server ID (FC 0x11) payload. */
  const char *(*server_id)(void *ctx, uint8_t *run);

  /** Handle a function code from the specification's user-definable
      @param req      Request payload, i.e. the PDU after the function code.
      @param rsp      Where to put the response payload, again after the code.
      @param rsp_len  Response payload length on success. */
  mb_exception_t (*user_function)(void *ctx, uint8_t fc,
                                  const uint8_t *req, size_t req_len,
                                  uint8_t *rsp, size_t rsp_cap, size_t *rsp_len);

  void *ctx;
} mb_data_model_t;

/** Server instance. No global state, so several may coexist. */
typedef struct
{
  const mb_data_model_t *model;
} mb_slave_t;

/** Largest request or response PDU: 253 bytes (256-byte ADU less unit id and CRC). */
#define MB_MAX_PDU 253U

void mb_slave_init(mb_slave_t *slave, const mb_data_model_t *model);

/** Execute one request PDU.
    @param  req      Request PDU: function code followed by its data.
    @param  req_len  Length of req, at least 1.
    @param  rsp      Response buffer, at least MB_MAX_PDU bytes.
    @param  rsp_cap  Capacity of rsp.
    @return Response PDU length, or 0 if no response is to be sent. */
size_t mb_slave_execute(mb_slave_t *slave, const uint8_t *req, size_t req_len,
                        uint8_t *rsp, size_t rsp_cap);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_SLAVE_H */
