/**
  ******************************************************************************
  * @file    dev_uart.c
  * @brief   The board's three serial ports as dev_serial_t, and the only file
  *          that touches a USART or its interrupt.
  *
  * Register-level rather than HAL_UART_Receive because the protocol needs two
  * things the HAL call will not give: a single-byte take with no state machine
  * in the way, and explicit control over the sticky error flags.
  *
  * USART3 is the debug probe's VCP and carries either the console or Modbus.
  * USART2 and UART5 are RS485 and carry Modbus only - there is no console on
  * a bus with other devices on it.
  *
  * **The RS485 ports receive on interrupt, and each byte carries the tick it
  * arrived at.** A multidrop bus is not quiet between our own transactions:
  * every frame on the segment lands here, including the ones addressed to
  * another node, and RTU delimits frames by silence. Polling from the main
  * loop timestamped a byte when the loop got round to it - a 276-byte IMU
  * cargo is 1.5 ms, which at 115200 is seventeen characters - so the silence
  * the protocol measured was the main loop's and not the bus's. Worse, with
  * the FIFO disabled the seventeenth byte overruns, and on this silicon a
  * latched ORE ends reception until it is cleared through ICR.
  *
  * **The RS485 ports hear themselves.** RE is tied to GND on both THVD1450s
  * (schematic RS485.SchDoc: U5 and U6 pin 2 sit on the GND net), so the
  * receiver stays enabled while the hardware DE drives the line, and every
  * byte transmitted comes straight back in. Their put() purges afterwards -
  * without it the reply lands in the receiver as a request.
  ******************************************************************************
  */
#include "board.h"
#include "dev_serial.h"

#include "main.h"

/* Defined in main.c by the MX_*_Init functions. */
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

/* One rate for all three. CubeMX carries 9216000 on the two RS485 ports,
   which is not a Modbus rate on any bus - the runtime value is this one. */
#define DEV_UART_BAUD 115200U

#define DEV_ERR_FLAGS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define DEV_ERR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

/* One Modbus RTU frame is 256 bytes at most. Sized to hold a whole one so a
   frame that begins while the main loop is inside a long board call is not
   half lost - which is the failure this ring exists to prevent, not a
   throughput problem. */
#define DEV_RING 256U

typedef struct
{
  uint8_t  byte;
  uint32_t tick;      /**< DWT->CYCCNT when the character arrived */
} dev_stamped_t;

typedef struct
{
  UART_HandleTypeDef *uart;
  bool                echoes;      /**< RS485 with RE tied low                */
  bool                interrupt;   /**< receives through the ISR below        */
  IRQn_Type           irq;
  const char         *name;

  volatile dev_stamped_t ring[DEV_RING];
  volatile uint16_t      head;     /**< written by the ISR                    */
  volatile uint16_t      tail;     /**< written by the main loop              */
  volatile bool          faulted;
  volatile uint32_t      dropped;  /**< bytes the ring had no room for        */
} dev_port_t;

static dev_port_t s_ports[DEV_UART_COUNT] =
{
  { .echoes = false, .interrupt = false, .irq = USART3_IRQn,
    .name = "USART3" },
  { .echoes = true,  .interrupt = true,  .irq = USART2_IRQn,
    .name = "USART2" },
  { .echoes = true,  .interrupt = true,  .irq = UART5_IRQn,
    .name = "UART5"  },
};

static dev_serial_t s_devs[DEV_UART_COUNT];
static bool         s_built;

static UART_HandleTypeDef *uart_of(void *ctx)
{
  return ((dev_port_t *)ctx)->uart;
}

/* The whole interrupt. One byte, one timestamp, no HAL: HAL_UART_IRQHandler
   runs a transfer state machine this layer does not use and would fight. */
static void on_irq(dev_port_t *p)
{
  USART_TypeDef *u = p->uart->Instance;
  const uint32_t isr = u->ISR;

  if ((isr & DEV_ERR_FLAGS) != 0U)
  {
    u->ICR = DEV_ERR_CLEAR;
    p->faulted = true;
  }

  if ((isr & USART_ISR_RXNE_RXFNE) != 0U)
  {
    const uint32_t tick = DWT->CYCCNT;
    const uint8_t  byte = (uint8_t)(u->RDR & 0xFFU);
    const uint16_t next = (uint16_t)((p->head + 1U) % DEV_RING);

    if (next == p->tail)
    {
      p->dropped++;          /* the main loop is not draining; say so */
    }
    else
    {
      p->ring[p->head].byte = byte;
      p->ring[p->head].tick = tick;
      p->head = next;
    }
  }
}

void USART2_IRQHandler(void)
{
  on_irq(&s_ports[1]);
}

void UART5_IRQHandler(void)
{
  on_irq(&s_ports[2]);
}

static bool u_get(void *ctx, uint8_t *byte, uint32_t *tick)
{
  dev_port_t *p = (dev_port_t *)ctx;

  if (p->interrupt)
  {
    if (p->tail == p->head)
    {
      return false;
    }

    *byte = p->ring[p->tail].byte;
    *tick = p->ring[p->tail].tick;
    p->tail = (uint16_t)((p->tail + 1U) % DEV_RING);
    return true;
  }

  USART_TypeDef *u = p->uart->Instance;

  if ((u->ISR & USART_ISR_RXNE_RXFNE) == 0U)
  {
    return false;
  }

  /* Polled: the best timestamp available is now, which is what this port had
     before any of them had an interrupt. It is the console's wire and the
     master on it is a person or a script, not a bus. */
  *tick = DWT->CYCCNT;
  *byte = (uint8_t)(u->RDR & 0xFFU);
  return true;
}

static bool u_fault(void *ctx)
{
  dev_port_t *p = (dev_port_t *)ctx;

  if (p->interrupt)
  {
    if (!p->faulted)
    {
      return false;
    }
    p->faulted = false;      /* the ISR already cleared the hardware flags */
    return true;
  }

  USART_TypeDef *u = p->uart->Instance;

  if ((u->ISR & DEV_ERR_FLAGS) == 0U)
  {
    return false;
  }

  /* Clear the flags first, then drop the byte that came with them. Reading RDR
     alone does NOT clear ORE - it needs ORECF - and a latched ORE ends
     reception for good. */
  u->ICR = DEV_ERR_CLEAR;

  if ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    (void)u->RDR;
  }

  return true;
}

static void u_purge(void *ctx)
{
  dev_port_t    *p = (dev_port_t *)ctx;
  USART_TypeDef *u = p->uart->Instance;

  u->ICR = DEV_ERR_CLEAR;

  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    (void)u->RDR;
  }

  p->tail    = p->head;
  p->faulted = false;
}

static void u_put(void *ctx, const uint8_t *data, uint16_t len)
{
  USART_TypeDef *u = uart_of(ctx)->Instance;

  /* Register level like the receive side, and for a second reason on top of
     that one: HAL_UART_Transmit blocks for the whole frame. Measured, a
     53-byte reply at 115200 stalled the main loop 4.6 ms - ten times what
     the STO latch holds, and by far the worst gap on the board. The wait is
     a spin either way; this one feeds the charge pump while it waits. */
  for (uint16_t i = 0U; i < len; i++)
  {
    while ((u->ISR & USART_ISR_TXE_TXFNF) == 0U)
    {
      Board_StoKeepalive();
    }
    u->TDR = data[i];
  }

  while ((u->ISR & USART_ISR_TC) == 0U)
  {
    Board_StoKeepalive();
  }

  /* Everything just sent came back in on the RS485 ports, through the
     interrupt and into the ring. Dropped here and not upstream: the protocol
     layer never saw these bytes, so there is no framing state to unwind. */
  if (((dev_port_t *)ctx)->echoes)
  {
    u_purge(ctx);
  }
}

static uint32_t u_ticks(void *ctx)
{
  (void)ctx;
  /* Raw cycle counter, enabled at boot by ClockStability_Init(). Wraps at
     exactly 2^32, which is what the protocol layer's timing requires. */
  return DWT->CYCCNT;
}

static uint32_t u_ticks_per_us(void *ctx)
{
  (void)ctx;
  const uint32_t per = SystemCoreClock / 1000000U;
  return (per == 0U) ? 1U : per;
}

const dev_serial_t *dev_uart(uint8_t index)
{
  if (index >= DEV_UART_COUNT)
  {
    return NULL;
  }

  /* Built on first use rather than as a static initialiser: huart2, huart3
     and huart5 are not constant expressions. */
  if (!s_built)
  {
    s_ports[0].uart = &huart3;
    s_ports[1].uart = &huart2;
    s_ports[2].uart = &huart5;

    for (uint8_t i = 0U; i < DEV_UART_COUNT; i++)
    {
      s_devs[i].get          = u_get;
      s_devs[i].fault        = u_fault;
      s_devs[i].put          = u_put;
      s_devs[i].ticks        = u_ticks;
      s_devs[i].ticks_per_us = u_ticks_per_us;
      s_devs[i].purge        = u_purge;
      s_devs[i].ctx          = &s_ports[i];

      if (s_ports[i].interrupt)
      {
        /* Priority 5: below anything that must not be delayed and above the
           systick the delays here use, so a byte is never late for a frame
           boundary. RXNE and the error sources both, because an overrun that
           nobody is told about is an overrun nobody clears. */
        HAL_NVIC_SetPriority(s_ports[i].irq, 5U, 0U);
        HAL_NVIC_EnableIRQ(s_ports[i].irq);
        SET_BIT(s_ports[i].uart->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
        SET_BIT(s_ports[i].uart->Instance->CR3, USART_CR3_EIE);
      }
    }

    s_built = true;
  }

  return &s_devs[index];
}

/* Four patterns, one byte at a time. Not one four-byte burst: the FIFO is
   disabled on all three ports, so a blocking multi-byte transmit overruns its
   own receiver. 0x00 and 0xFF catch a line stuck at either rail, 0x5A and
   0xA5 one bit-shifted or inverted. */
static const uint8_t DEV_ECHO_PATTERN[4] = { 0x00U, 0xFFU, 0x5AU, 0xA5U };

uint8_t dev_uart_echo(uint8_t index, uint8_t *seen)
{
  uint8_t matched = 0U;
  uint8_t back = 0U;

  if (index >= DEV_UART_COUNT)
  {
    return 0U;
  }

  const dev_serial_t *dev = dev_uart(index);
  dev_port_t         *p = (dev_port_t *)dev->ctx;

  for (uint8_t i = 0U; i < 4U; i++)
  {
    uint8_t byte = DEV_ECHO_PATTERN[i];
    uint8_t got = 0U;
    uint32_t tick = 0U;

    u_purge(dev->ctx);
    (void)HAL_UART_Transmit(p->uart, &byte, 1U, 10U);

    /* One character at 115200 is 95 us. Two milliseconds is twenty times
       that and still short enough that four of them cannot outlive a
       master's patience. */
    const uint32_t until = HAL_GetTick() + 2U;

    while (HAL_GetTick() <= until)
    {
      if (u_get(dev->ctx, &got, &tick))
      {
        back++;
        if (got == byte)
        {
          matched |= (uint8_t)(1U << i);
        }
        break;
      }
    }
  }

  u_purge(dev->ctx);

  if (seen != NULL)
  {
    *seen = back;
  }

  return matched;
}

uint32_t dev_uart_dropped(uint8_t index)
{
  return (index < DEV_UART_COUNT) ? s_ports[index].dropped : 0U;
}

const char *dev_uart_name(uint8_t index)
{
  return (index < DEV_UART_COUNT) ? s_ports[index].name : "?";
}

bool dev_uart_rs485(uint8_t index)
{
  return (index < DEV_UART_COUNT) && s_ports[index].echoes;
}

uint32_t dev_uart_baud(void)
{
  return DEV_UART_BAUD;
}
