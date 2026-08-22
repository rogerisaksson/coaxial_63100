/**
  ******************************************************************************
  * @file    modbus_rtu.h
  * @brief   Modbus RTU transport: framing, addressing, CRC. Portable.
  *
  * MODBUS over Serial Line Specification and Implementation Guide V1.02.
  *
  * RTU has no length field and no start or end delimiter. A frame is delimited
  * ONLY by silence on the line: at least t3.5 of idle before and after it, and
  * never more than t1.5 of idle between two characters inside it. That is the
  * defining property of the transmission mode, so this layer is a timing state
  * machine first and a parser second.
  *
  * Time is injected as a free-running counter of arbitrary TICKS, together with
  * how many ticks make a microsecond. Ticks rather than microseconds on purpose:
  * the natural time source on this target is a 32-bit CPU cycle counter, and
  * dividing it down to microseconds first would move the wrap point off a power
  * of two, at which point unsigned subtraction across the wrap silently stops
  * giving the right answer. Fed raw, the counter may wrap freely as long as it
  * is sampled more often than its period. This file therefore has no dependency
  * on any clock, timer or hardware header, and is host-testable.
  *
  * Silence rules, per V1.02 section 2.5.1.1: above 19200 baud the recommended
  * fixed values are t1.5 = 750 us and t3.5 = 1.750 ms; at or below 19200 baud
  * both are derived from the character time. Both branches are implemented.
  ******************************************************************************
  */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "modbus_slave.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum RTU ADU: unit id + 253 PDU bytes + 2 CRC. */
#define MB_RTU_ADU_MAX 256U

/** Smallest possible ADU: unit id + function code + CRC. */
#define MB_RTU_ADU_MIN 4U

/** Broadcast address. Acted upon, never answered. */
#define MB_RTU_BROADCAST 0U

/** Diagnostic counters, named as in MODBUS over Serial Line V1.02. */
typedef struct
{
  uint32_t bus_message;        /**< every frame seen on the bus            */
  uint32_t bus_comm_error;     /**< CRC failures and framing/overrun drops */
  uint32_t server_message;     /**< frames addressed to this unit          */
  uint32_t server_exception;   /**< exception responses sent               */
  uint32_t server_no_response; /**< frames handled with no reply           */
  uint32_t char_overrun;       /**< frames lost to a UART overrun          */
} mb_rtu_counters_t;

typedef struct
{
  mb_slave_t *slave;
  uint8_t     unit_id;

  uint32_t t15_ticks;
  uint32_t t35_ticks;

  uint8_t  rx[MB_RTU_ADU_MAX];
  uint16_t rx_len;
  bool     receiving;
  bool     frame_bad;      /**< inter-character gap, overflow or UART error */
  bool     saw_overrun;
  uint32_t last_event_ticks;

  uint8_t  tx[MB_RTU_ADU_MAX];
  uint16_t tx_len;

  mb_rtu_counters_t counters;
} mb_rtu_t;

/**
  * @brief  Initialise the transport.
  * @param  unit_id       This server address, 1..247. Never 0.
  * @param  baud          Line rate, used to derive the silence intervals.
  * @param  bits_per_char Bit times per character on the wire. The
  *                       specification assumes 11 (start + 8 data + parity +
  *                       stop); a link running 8N1 physically sends 10, but
  *                       11 is the conservative choice for timing because it
  *                       lengthens the computed silences.
  * @param  ticks_per_us  Ticks of the injected counter per microsecond.
  */
void mb_rtu_init(mb_rtu_t *rtu, mb_slave_t *slave, uint8_t unit_id,
                 uint32_t baud, uint8_t bits_per_char, uint32_t ticks_per_us);

/** Feed one received byte. */
void mb_rtu_on_byte(mb_rtu_t *rtu, uint8_t byte, uint32_t now_ticks);

/**
  * @brief  Report a UART receive error: overrun, framing, parity or noise.
  *
  * The current frame is poisoned and will be discarded once the line goes
  * quiet. Reporting the error is not optional: on this hardware a latched
  * overrun flag stops reception permanently, so the port layer must clear the
  * flag and tell the transport that the frame is worthless.
  */
void mb_rtu_on_error(mb_rtu_t *rtu, uint32_t now_ticks);

/**
  * @brief  Advance the state machine; call as often as possible.
  * @param  out  Receives a pointer to the response frame, if any.
  * @return Response frame length, or 0 if there is nothing to send.
  */
size_t mb_rtu_service(mb_rtu_t *rtu, uint32_t now_ticks, const uint8_t **out);

/** True while a frame is being received or is awaiting its closing silence. */
bool mb_rtu_busy(const mb_rtu_t *rtu);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
