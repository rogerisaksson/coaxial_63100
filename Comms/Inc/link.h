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

/** Build the stack. Call once, after the UART and the cycle counter are up. */
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

uint8_t  link_unit_id(void);
void     link_stats(link_stats_t *out);

/** Print the link state and command list to the ASCII console. */
void Link_ReportStatus(void);
uint32_t link_ticks_per_us(void);

#ifdef __cplusplus
}
#endif

#endif /* LINK_H */
