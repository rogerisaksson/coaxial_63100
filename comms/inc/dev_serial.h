/**
  ******************************************************************************
  * @file    dev_serial.h
  * @brief   Byte-oriented serial device, bottom layer of the comms stack.
  *
  *     cmd        request/response commands, protocol-agnostic
  *      |
  *     proto      framing and addressing        (Modbus RTU is the first one)
  *      |
  *     dev        this: bytes, errors, a clock  (USART3 is the first one)
  *
  * A protocol needs four things from a device: pull a waiting byte, learn the
  * receiver faulted, push a frame out, read a tick counter for silence timing.
  * Nothing above this header mentions a UART, so the protocol layer stays
  * host-testable against a fake.
  *
  * ticks() must wrap at 2^32 - raw CPU cycles, not divided-down microseconds.
  * Dividing moves the wrap off a power of two and unsigned elapsed-time
  * arithmetic then breaks silently across it.
  ******************************************************************************
  */
#ifndef DEV_SERIAL_H
#define DEV_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /**
    * @brief  Take one received byte and the tick it arrived at.
    * @return False when the receiver is empty.
    *
    * The character's tick, not the caller's: timestamping when the main loop
    * got round to it made RTU measure the loop's silence, not the wire's - a
    * 276-byte IMU cargo is seventeen characters at 115200.
    */
  bool (*get)(void *ctx, uint8_t *byte, uint32_t *tick);

  /**
    * @brief  Test and clear any sticky receive error.
    * @return True if the receiver had faulted, in which case the caller must
    *         treat the frame in progress as lost.
    *
    * Clearing is not optional: a latched overrun stops reception permanently
    * until ICR clears it, which is how this board has lost its link before.
    */
  bool (*fault)(void *ctx);

  /** Send a whole frame. Blocking; the line is half duplex by convention. */
  void (*put)(void *ctx, const uint8_t *data, uint16_t len);

  /** Free-running counter, wrapping at 2^32. */
  uint32_t (*ticks)(void *ctx);

  /** Ticks per microsecond, for deriving protocol silence intervals. */
  uint32_t (*ticks_per_us)(void *ctx);

  /** Discard anything buffered and clear all error flags. */
  void (*purge)(void *ctx);

  void *ctx;
} dev_serial_t;

/** The USART3 instance on this board: PB10/PB11, 115200 8N1, polled. */
/** The board's three serial ports, in link.h's order: 0 USART3 on the debug
  * probe's VCP, 1 USART2 and 2 UART5 on RS485. NULL past the end. */
#define DEV_UART_COUNT 3U

const dev_serial_t *dev_uart(uint8_t index);

/** What the schematic calls the port, for the console and for 0x47. */
const char *dev_uart_name(uint8_t index);

/** True for the two whose receiver hears their own transmission - RE is tied
  * to GND on both transceivers, so their put() purges afterwards. */
bool dev_uart_rs485(uint8_t index);

/** Transmit four patterns on `index` and report which came back.
  *
  * A bitmask, one bit per pattern, and `seen` counts how many bytes returned
  * at all. On an RS485 port every bit should set: RE is tied to GND, so the
  * transceiver hears itself and a byte that does not come back means the
  * driver, the receiver or the wiring between them. On USART3 nothing comes
  * back and 0 is the right answer.
  *
  * It puts four bytes on the bus. On a segment with other devices that is
  * four bytes of noise, which is why nothing calls it on a timer. */
uint8_t dev_uart_echo(uint8_t index, uint8_t *seen);

/** Bytes the receive ring had no room for, since boot. Not zero means the
  * main loop stopped draining long enough to lose bus traffic. */
uint32_t dev_uart_dropped(uint8_t index);

/** The rate all three run. Not what the .ioc carries on the RS485 pair. */
uint32_t dev_uart_baud(void);

/** One port's actual rate: USART3 is always 115200 (the recovery path),
  * the RS485 pair follows the calibration record's `link_baud`. */
uint32_t dev_uart_port_baud(uint8_t index);

/** Re-init USART2 and UART5 at `baud` - the record's value, applied from
  * main() after the record loads and before link_init(). */
bool dev_uart_set_rs485_baud(uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* DEV_SERIAL_H */
