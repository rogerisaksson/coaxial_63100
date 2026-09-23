/**
  ******************************************************************************
  * @file    link.h
  * @brief   The comms stack, assembled: device + protocol + commands.
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

/** The board's three Modbus ports. */
#define LINK_CONSOLE 0U
#define LINK_RS485_1 1U
#define LINK_RS485_2 2U
#define LINK_COUNT   3U

/** Build the stack. Call once, after the UARTs and the cycle counter are up. */
void link_init(void);

/** Name of the protocol currently bound, e.g. "modbus-rtu". */
const char *link_proto_name(void);

bool link_active(void);

/** True while the RTU receiver has part of a frame in hand. */
bool link_busy(void);

/** Take the line. */
void link_open(void);

/** Give the line back to the console at once. */
void link_close(void);

/** Give it back after the frame in flight has been answered. */
void link_request_close(void);

/** Service the stack. Call every main-loop pass while link_active(). */
void link_poll(void);

/** Bytes received on any port since boot. */
uint32_t link_rx_count(void);

uint8_t  link_unit_id(void);
void     link_stats(link_stats_t *out);

/** One port's counters. link_stats() is this for LINK_CONSOLE. */
void link_stats_of(uint8_t index, link_stats_t *out);

const char *link_name(uint8_t index);
bool        link_port_open(uint8_t index);

/** The port whose request a command handler is answering. */
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
