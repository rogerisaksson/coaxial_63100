/** fake_uart.c - The host's serial ports, for the fake board. */

/* Bytes in from the host at 115200's pace, a frame out to it, a clock of one tick a
   microsecond that the exchange steps. */
#include "board/cal.h"
#include "board/thermal.h"
#include "board_hw.h"
#include "dev_serial.h"
#include "link.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* A character at 115200 8N1: 10 bits, 86.8 us. */
#define FAKE_CHAR_US 87U
#define FAKE_BYTES   4096U

/* How long an exchange waits for its answer: past t3.5 and any handler. */
#define FAKE_WAIT_US 50000U

static struct
{
  uint8_t  in[FAKE_BYTES];
  uint32_t at[FAKE_BYTES];
  uint16_t head, tail;
  uint8_t  out[FAKE_BYTES];
  uint16_t out_len;
} s_port[DEV_UART_COUNT];

static uint32_t s_clock;

static bool get(void *ctx, uint8_t *byte, uint32_t *tick)
{
  uint8_t i = (uint8_t)(uintptr_t)ctx;

  if (s_port[i].head == s_port[i].tail)
  {
    return false;
  }
  *byte = s_port[i].in[s_port[i].tail];
  *tick = s_port[i].at[s_port[i].tail];
  s_port[i].tail = (uint16_t)((s_port[i].tail + 1U) % FAKE_BYTES);
  return true;
}

static bool fault(void *ctx)
{
  (void)ctx;
  return false;
}

static void put(void *ctx, const uint8_t *data, uint16_t len)
{
  uint8_t i = (uint8_t)(uintptr_t)ctx;
  uint16_t room = (uint16_t)(FAKE_BYTES - s_port[i].out_len);
  uint16_t n = len < room ? len : room;

  memcpy(&s_port[i].out[s_port[i].out_len], data, n);
  s_port[i].out_len = (uint16_t)(s_port[i].out_len + n);
}

static uint32_t ticks(void *ctx)
{
  (void)ctx;
  return s_clock;
}

uint32_t HAL_GetTick(void)
{
  return s_clock / 1000U;
}

/* One pass of main()'s loop, as far as the fake builds it. */
static void fake_loop(void)
{
  if (!link_busy())
  {
    Board_ThermalPoll();
  }
  link_poll();
}

static uint32_t ticks_per_us(void *ctx)
{
  (void)ctx;
  return 1U;
}

static void purge(void *ctx)
{
  uint8_t i = (uint8_t)(uintptr_t)ctx;

  s_port[i].tail = s_port[i].head;
}

const dev_serial_t *dev_uart(uint8_t index)
{
  static dev_serial_t devs[DEV_UART_COUNT];

  if (index >= DEV_UART_COUNT)
  {
    return NULL;
  }
  devs[index] = (dev_serial_t){get, fault, put, ticks, ticks_per_us, purge,
                               (void *)(uintptr_t)index};
  return &devs[index];
}

const char *dev_uart_name(uint8_t index)
{
  static const char *const names[DEV_UART_COUNT] = {"USART3", "USART2", "UART5"};

  return index < DEV_UART_COUNT ? names[index] : "?";
}

/* USART2 and UART5 are the RS485 pair: their receivers hear their own transmission, so
   an echo matches all four patterns. */
bool dev_uart_rs485(uint8_t index)
{
  return (index != LINK_CONSOLE) && (index < DEV_UART_COUNT);
}

uint8_t dev_uart_echo(uint8_t index, uint8_t *seen)
{
  bool heard = dev_uart_rs485(index);

  if (seen != NULL)
  {
    *seen = heard ? 4U : 0U;
  }
  return heard ? 0x0FU : 0U;
}

uint32_t dev_uart_dropped(uint8_t index)
{
  (void)index;
  return 0U;
}

uint32_t dev_uart_baud(void)
{
  return 115200U;
}

uint32_t dev_uart_port_baud(uint8_t index)
{
  (void)index;
  return 115200U;
}

/* The host's side, through ctypes. */

/** The record and the stack up, the console port in binary mode, as a host's 'm' leaves it. */
void fake_open(void)
{
  Board_CalInit();
  memset(s_port, 0, sizeof s_port);
  s_clock = 0U;
  link_init();
  Board_ThermalInit();
  link_open();
}

/** `req` in on the console port at the line's pace, the stack pumped until its answer
    is out or FAKE_WAIT_US have passed; the answer into `out`, its length returned. */
uint16_t fake_exchange(const uint8_t *req, uint16_t len, uint8_t *out, uint16_t cap)
{
  s_port[LINK_CONSOLE].out_len = 0U;
  for (uint16_t k = 0U; k < len; k++)
  {
    s_clock += FAKE_CHAR_US;
    s_port[LINK_CONSOLE].in[s_port[LINK_CONSOLE].head] = req[k];
    s_port[LINK_CONSOLE].at[s_port[LINK_CONSOLE].head] = s_clock;
    s_port[LINK_CONSOLE].head = (uint16_t)((s_port[LINK_CONSOLE].head + 1U) % FAKE_BYTES);
    fake_loop();
  }
  for (uint32_t waited = 0U; waited < FAKE_WAIT_US && s_port[LINK_CONSOLE].out_len == 0U;
       waited += 100U)
  {
    s_clock += 100U;
    fake_loop();
  }
  uint16_t n = s_port[LINK_CONSOLE].out_len < cap ? s_port[LINK_CONSOLE].out_len : cap;
  memcpy(out, s_port[LINK_CONSOLE].out, n);
  return n;
}
