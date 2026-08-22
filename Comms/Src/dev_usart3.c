/**
  ******************************************************************************
  * @file    dev_usart3.c
  * @brief   USART3 as a dev_serial_t. The only file that touches the USART.
  *
  * Register-level rather than HAL_UART_Receive because the protocol needs two
  * things the HAL call will not give: a non-blocking single-byte take with no
  * state machine in the way, and explicit control over the sticky error flags.
  ******************************************************************************
  */
#include "dev_serial.h"

#include "main.h"

/* Defined in main.c by MX_USART3_UART_Init(). */
extern UART_HandleTypeDef huart3;

#define DEV_USART3_BAUD 115200U

#define DEV_ERR_FLAGS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define DEV_ERR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF)

static bool u3_get(void *ctx, uint8_t *byte)
{
  (void)ctx;
  USART_TypeDef *u = huart3.Instance;

  if ((u->ISR & USART_ISR_RXNE_RXFNE) == 0U)
  {
    return false;
  }

  *byte = (uint8_t)(u->RDR & 0xFFU);
  return true;
}

static bool u3_fault(void *ctx)
{
  (void)ctx;
  USART_TypeDef *u = huart3.Instance;

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

static void u3_put(void *ctx, const uint8_t *data, uint16_t len)
{
  (void)ctx;
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 100U);
}

static uint32_t u3_ticks(void *ctx)
{
  (void)ctx;
  /* Raw cycle counter, enabled at boot by ClockStability_Init(). Wraps at
     exactly 2^32, which is what the protocol layer's timing requires. */
  return DWT->CYCCNT;
}

static uint32_t u3_ticks_per_us(void *ctx)
{
  (void)ctx;
  const uint32_t per = SystemCoreClock / 1000000U;
  return (per == 0U) ? 1U : per;
}

static void u3_purge(void *ctx)
{
  USART_TypeDef *u = huart3.Instance;

  u->ICR = DEV_ERR_CLEAR;

  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    (void)u->RDR;
  }

  (void)ctx;
}

static const dev_serial_t s_usart3 =
{
  .get          = u3_get,
  .fault        = u3_fault,
  .put          = u3_put,
  .ticks        = u3_ticks,
  .ticks_per_us = u3_ticks_per_us,
  .purge        = u3_purge,
  .ctx          = NULL,
};

const dev_serial_t *dev_usart3(void)
{
  return &s_usart3;
}

uint32_t dev_usart3_baud(void)
{
  return DEV_USART3_BAUD;
}
