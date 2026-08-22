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
  * A protocol needs exactly four things from a device: pull a byte if one is
  * waiting, learn that the receiver faulted, push a frame out, and read a
  * monotonic tick counter for silence timing. Nothing above this header
  * mentions a UART, so the protocol layer stays host-testable against a fake.
  *
  * ticks() returns a free-running counter that MUST wrap at 2^32 - raw CPU
  * cycles, not a divided-down microsecond count. Dividing first moves the wrap
  * off a power of two and unsigned elapsed-time arithmetic then breaks
  * silently across it.
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
  /** Take one received byte. False when the receiver is empty. */
  bool (*get)(void *ctx, uint8_t *byte);

  /**
    * @brief  Test and clear any sticky receive error.
    * @return True if the receiver had faulted, in which case the caller must
    *         treat the frame in progress as lost.
    *
    * Clearing is not optional. On STM32 a latched overrun stops reception
    * permanently until the flag is cleared through ICR, which is how this
    * board has lost its serial link before.
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
const dev_serial_t *dev_usart3(void);

/** Line rate the device is configured for, needed to derive t1.5 and t3.5. */
uint32_t dev_usart3_baud(void);

#ifdef __cplusplus
}
#endif

#endif /* DEV_SERIAL_H */
