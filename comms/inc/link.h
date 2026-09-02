/**
  ******************************************************************************
  * @file    link.h
  * @brief   The comms stack, assembled: device + protocol + commands.
  *
  * One instance, because the board has one line. Swapping the protocol means
  * changing which one link_init() builds; swapping the wire means passing a
  * different dev_serial_t. Neither touches the command table.
  *
  * The ASCII console shares USART3, so the link is opened explicitly and the
  * main loop must not print while it is open: a blocking transmit inside a
  * frame corrupts RTU framing and stalls reception long enough to latch an
  * overrun.
  ******************************************************************************
  */
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t  unit_id;
  uint32_t t15_ticks;
  uint32_t t35_ticks;
  uint32_t bus_message;
  uint32_t bus_comm_error;
  uint32_t server_message;
  uint32_t server_exception;
  uint32_t server_no_response;
  uint32_t char_overrun;
} link_stats_t;

/** The board's three Modbus ports. 0 is the debug probe's VCP, which shares
  * the wire with the ASCII console; 1 and 2 are RS485 and carry Modbus only,
  * open from boot - there is no console on a bus with other devices on it. */
#define LINK_CONSOLE 0U
#define LINK_RS485_1 1U
#define LINK_RS485_2 2U
#define LINK_COUNT   3U

/** Build the stack. Call once, after the UARTs and the cycle counter are up. */
void link_init(void);

/** Name of the protocol currently bound, e.g. "modbus-rtu". */
const char *link_proto_name(void);

bool link_active(void);

/** True while the RTU receiver has part of a frame in hand.
  *
  * What anything long-running in the main loop must check first: a frame is
  * delimited by silence, so a millisecond spent elsewhere reads as the gap
  * that ends it. */
bool link_busy(void);

/** Take the line. Anything already in the receiver is discarded, and framing
    state starts clean; the diagnostic counters survive, same as link_close(). */
void link_open(void);

/** Give the line back to the console at once. */
void link_close(void);

/** Give it back after the frame in flight has been answered. */
void link_request_close(void);

/** Service the stack. Call every main-loop pass while link_active(). */
void link_poll(void);

/** Bytes received on any port since boot.
  *
  * The board's only evidence that a host is still there. A session polls; a
  * script that was killed does not, and its holds would otherwise outlive
  * it. A count rather than a time because this layer has no clock of its
  * own - the caller holds the time. */
uint32_t link_rx_count(void);

uint8_t  link_unit_id(void);
void     link_stats(link_stats_t *out);

/** One port's counters. link_stats() is this for LINK_CONSOLE. */
void link_stats_of(uint8_t index, link_stats_t *out);

const char *link_name(uint8_t index);
bool        link_port_open(uint8_t index);

/** The port whose request a command handler is answering.
  *
  * What anything that transmits must check: putting bytes on this port mid
  * transaction lands them in front of the reply. */
uint8_t link_current(void);
bool        link_is_rs485(uint8_t index);
uint32_t    link_baud(uint8_t index);

/** Print the link state and command list to the ASCII console. */
void Link_ReportStatus(void);
uint32_t link_ticks_per_us(void);

#ifdef __cplusplus
}
#endif

#endif /* LINK_H */
