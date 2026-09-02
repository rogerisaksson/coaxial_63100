/**
  ******************************************************************************
  * @file    modbus_rtu.h
  * @brief   Modbus RTU transport: framing, addressing, CRC. Portable.
  *
  * MODBUS over Serial Line Specification and Implementation Guide V1.02.
  *
  * A frame is delimited only by silence: t3.5 of idle before and after, never
  * more than t1.5 between two characters inside it. So this layer is a timing
  * state machine first and a parser second.
  *
  * Time is injected as a free-running counter of arbitrary TICKS plus ticks per
  * microsecond. Ticks, never microseconds: the source here is a 32-bit cycle
  * counter, and dividing it down moves the wrap off a power of two, at which
  * point unsigned subtraction across the wrap silently stops being right. Fed
  * raw it may wrap freely if sampled more often than its period - so no clock,
  * timer or hardware header is needed, and the file is host-testable.
  *
  * Silence, per V1.02 section 2.5.1.1: above 19200 baud the fixed t1.5 = 750 us
  * and t3.5 = 1.750 ms; at or below, both derive from the character time. Both
  * branches are implemented.
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

/** The request-length oracle, for ending a frame without the silence.

    Given the PDU bytes received so far, returns the full PDU length when
    that is already knowable - a fixed-shape function code, or a variable
    one whose bytes so far settle it - and 0 when it is not. With an
    oracle set, a frame whose length matches and whose CRC checks is
    delivered the moment its last byte arrives instead of after t3.5 of
    silence: 1.75 ms of every such transaction, both directions of the
    spec's fixed gap. A wrong answer is not slow, it is CORRUPTION - a
    truncated frame executed on a lucky CRC - so the oracle's table is
    validated against the real encoders by the test suite, and anything
    it cannot prove waits out the silence exactly as before. */
typedef uint16_t (*mb_rtu_length_fn)(const uint8_t *pdu, uint16_t have);

typedef struct
{
  mb_slave_t *slave;
  uint8_t     unit_id;
  mb_rtu_length_fn length_hint;   /**< NULL: every frame waits t3.5 */

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
  * @param  bits_per_char Bit times per character. The specification assumes 11
  *                       (start + 8 data + parity + stop); 8N1 sends 10, but 11
  *                       lengthens the computed silences, so it is the safe one.
  * @param  ticks_per_us  Ticks of the injected counter per microsecond.
  */
void mb_rtu_init(mb_rtu_t *rtu, mb_slave_t *slave, uint8_t unit_id,
                 uint32_t baud, uint8_t bits_per_char, uint32_t ticks_per_us);

/** Feed one received byte. */
void mb_rtu_on_byte(mb_rtu_t *rtu, uint8_t byte, uint32_t now_ticks);

/** Install the request-length oracle (NULL removes it). Separate from
    init so the timing behaviour is opt-in per port and the init
    signature every test drives stays put. */
void mb_rtu_set_length_hint(mb_rtu_t *rtu, mb_rtu_length_fn hint);

/**
  * @brief  Report a UART receive error: overrun, framing, parity or noise.
  *
  * The frame is poisoned and discarded once the line goes quiet. Not optional:
  * a latched overrun stops reception permanently on this silicon, so the port
  * layer must clear the flag and say the frame is worthless.
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
