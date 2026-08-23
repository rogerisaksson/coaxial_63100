/**
  ******************************************************************************
  * @file    modbus_slave.h
  * @brief   Portable Modbus server (slave) PDU engine.
  *
  * MODBUS Application Protocol V1.1b3. This translation unit knows nothing
  * about UARTs, STM32, CMSIS or timers: it takes a request PDU in a buffer and
  * writes a response PDU into another buffer. Framing, addressing and CRC are
  * the transport's job (see modbus_rtu.h). That split is what makes this file
  * compilable and testable on a host.
  *
  * The application supplies a data model as a vtable of small callbacks. Reads
  * and writes are per item; all quantity limits, bit packing and PDU layout
  * live here so they are implemented and reviewed exactly once.
  *
  * Multi-item writes are validated across their whole range BEFORE any item is
  * applied. A write that would fail half way must not leave the device half
  * written, so the engine never starts applying until it knows the entire
  * range is acceptable.
  ******************************************************************************
  */
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

/** Exception codes. MB_EX_NONE is not a wire value; it means "no error". */
typedef enum
{
  MB_EX_NONE                  = 0x00,
  MB_EX_ILLEGAL_FUNCTION      = 0x01,
  MB_EX_ILLEGAL_DATA_ADDRESS  = 0x02,
  MB_EX_ILLEGAL_DATA_VALUE    = 0x03,
  MB_EX_SERVER_DEVICE_FAILURE = 0x04
} mb_exception_t;

/** Which of the four Modbus data tables an access refers to. */
typedef enum
{
  MB_TABLE_COIL,            /**< read/write bit    */
  MB_TABLE_DISCRETE_INPUT,  /**< read-only bit     */
  MB_TABLE_HOLDING_REG,     /**< read/write 16-bit */
  MB_TABLE_INPUT_REG        /**< read-only 16-bit  */
} mb_table_t;

/**
  * @brief Application data model.
  *
  * validate_range() is called once per request with the full address span and
  * must return MB_EX_ILLEGAL_DATA_ADDRESS for any part of it that is not
  * mapped, or MB_EX_NONE if every address in [addr, addr+qty) is accessible in
  * the requested direction. The engine has already rejected illegal
  * quantities and 16-bit address wrap before this is called.
  *
  * read_item()/write_item() are then called per address and may assume the
  * address is valid. They may still fail with MB_EX_SERVER_DEVICE_FAILURE.
  *
  * Unused callbacks may be NULL; the engine then answers MB_EX_ILLEGAL_FUNCTION
  * for the function codes that need them.
  */
typedef struct
{
  mb_exception_t (*validate_range)(void *ctx, mb_table_t table, uint16_t addr,
                                   uint16_t qty, bool for_write);
  mb_exception_t (*read_reg)(void *ctx, mb_table_t table, uint16_t addr, uint16_t *out);
  mb_exception_t (*write_reg)(void *ctx, uint16_t addr, uint16_t value);
  mb_exception_t (*read_bit)(void *ctx, mb_table_t table, uint16_t addr, bool *out);
  mb_exception_t (*write_bit)(void *ctx, uint16_t addr, bool value);

  /**
    * @brief Optional: would write_reg accept this value, without applying it?
    *
    * validate_range only checks addressability, so a multi-register write
    * (FC 0x10) spanning several registers can apply the first few through
    * write_reg() and only then discover the last one's VALUE is illegal -
    * leaving the device half written despite the client seeing one exception
    * for the whole request. If this is set, the engine calls it for every
    * item in a multi-register write before applying any of them, so a bad
    * value anywhere in the span refuses the whole write instead of applying
    * a prefix of it. May be NULL: a model with no per-value rule beyond
    * addressability, or one whose writes are side-effect-free enough that a
    * partial apply cannot matter, has nothing to gain from it.
    */
  mb_exception_t (*validate_reg_value)(void *ctx, uint16_t addr, uint16_t value);

  /** Report Server ID (FC 0x11) payload. Return the id string; set *run to
      0xFF for "running" or 0x00 for "stopped". May be NULL. */
  const char *(*server_id)(void *ctx, uint8_t *run);

  /**
    * @brief Handle a function code from the specification's user-definable
    *        ranges, 65..72 and 100..110.
    *
    * @param req      Request payload, i.e. the PDU after the function code.
    * @param rsp      Where to put the response payload, again after the code.
    * @param rsp_len  Response payload length on success.
    *
    * This is the seam the application's own binary commands hang off. NULL
    * makes every user-defined code answer ILLEGAL FUNCTION.
    */
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

/**
  * @brief  Execute one request PDU.
  * @param  req      Request PDU: function code followed by its data.
  * @param  req_len  Length of req, at least 1.
  * @param  rsp      Response buffer, at least MB_MAX_PDU bytes.
  * @param  rsp_cap  Capacity of rsp.
  * @return Response PDU length, or 0 if no response is to be sent.
  *
  * A malformed request - one whose length does not match its function code -
  * produces 0, not an exception: a length mismatch means the frame cannot be
  * trusted to have been parsed correctly at all.
  */
size_t mb_slave_execute(mb_slave_t *slave, const uint8_t *req, size_t req_len,
                        uint8_t *rsp, size_t rsp_cap);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_SLAVE_H */
