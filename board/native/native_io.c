/** native_io.c - The chip's pins, SPI2 and SPI4 with DMA1, natively: the parts on them. */

/* A pin reads what drives it: H_INTN (PD8) the BNO085, BKIN (PE15) the drivers' nFAULT high,
   one configured an input its pull, any other its ODR - CubeMX's MX_GPIO_Init sets the
   outputs, and does not run here. A write reaches the part on the pin: the A1335's
   chip select PE4, the BNO085's PB12, WAKE PD9 and NRSTN PD10. SPI2 exchanges bytes with the
   BNO085 in HAL_SPI_TransmitReceive, main() waiting their time on the line; SPI4's DMA packet
   runs once CSTART is set with both streams enabled, and the receiver's stream interrupts. A
   stream's address register holds the low 32 bits of a buffer in this image: the image's own
   high bits restore it. */
#include "native.h"
#include "stm32h7xx.h"

#include <string.h>

#define PORT_B   1U
#define PORT_D   3U
#define PORT_E   4U
#define PORTS    11U
#define SPI2_KERNEL_HZ 190000000U
#define SPI4_KERNEL_HZ 118750000U
#define SPI_PRESCALER_SHIFT 28U
#define SPI_SR_EOT (0x1UL << 3U)
#define DMA_LISR_TCIF0 (0x1UL << 5U)
#define DMA_LISR_TCIF1 (0x1UL << 11U)
#define BITS_PER_BYTE 8U

GPIO_TypeDef native_gpio[PORTS];
SPI_TypeDef  native_spi2;
SPI_TypeDef  native_spi4;
DMA_TypeDef  native_dma1;
DMA_Stream_TypeDef native_dma1_stream[2];
DMAMUX_Channel_TypeDef native_dmamux1[2];

/* CubeMX's handles for the two buses. */
SPI_HandleTypeDef hspi2 = { SPI2, { 0U } };
SPI_HandleTypeDef hspi4 = { SPI4, { 0U } };

/* The A1335's DMA end (board/src/board_angle.c). */
void DMA1_Stream0_IRQHandler(void);

static struct
{
  uint32_t input[PORTS];
  uint32_t pullup[PORTS];
} io;

static uint8_t io_port(const GPIO_TypeDef *port)
{
  return (uint8_t)(port - native_gpio);
}

static uint32_t io_levels(uint8_t k)
{
  uint32_t v = (native_gpio[k].ODR & ~io.input[k]) | (io.pullup[k] & io.input[k]);

  if (k == PORT_D)
  {
    v = (v & ~(uint32_t)GPIO_PIN_8) | (bno085_interrupt() ? GPIO_PIN_8 : 0U);
  }
  if (k == PORT_E)
  {
    v |= GPIO_PIN_15;
  }
  return v;
}

void native_io_open(void)
{
  memset(native_gpio, 0, sizeof native_gpio);
  memset(&native_spi2, 0, sizeof native_spi2);
  memset(&native_spi4, 0, sizeof native_spi4);
  memset(&native_dma1, 0, sizeof native_dma1);
  memset(native_dma1_stream, 0, sizeof native_dma1_stream);
  memset(native_dmamux1, 0, sizeof native_dmamux1);
  memset(&io, 0, sizeof io);
  for (uint8_t k = 0U; k < PORTS; k++)
  {
    native_gpio[k].IDR = io_levels(k);
  }
}

void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *init)
{
  const uint8_t k = io_port(port);

  if ((k >= PORTS) || (init == NULL))
  {
    return;
  }
  const uint32_t pins = init->Pin & 0xFFFFU;

  io.input[k] = (init->Mode == GPIO_MODE_INPUT) ? (io.input[k] | pins) : (io.input[k] & ~pins);
  io.pullup[k] = (init->Pull == GPIO_PULLUP) ? (io.pullup[k] | pins) : (io.pullup[k] & ~pins);
  port->IDR = io_levels(k);
}

void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins)
{
  const uint8_t k = io_port(port);

  if (k < PORTS)
  {
    io.input[k] |= pins;          /* reset state: analog, read as no pull */
    io.pullup[k] &= ~pins;
    port->IDR = io_levels(k);
  }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  const uint8_t k = io_port(port);
  const bool level = (state == GPIO_PIN_SET);

  if (k >= PORTS)
  {
    return;
  }
  port->ODR = level ? (port->ODR | pin) : (port->ODR & ~(uint32_t)pin);
  if ((k == PORT_E) && (pin == GPIO_PIN_4))
  {
    a1335_select(level);
  }
  if (k == PORT_B && pin == GPIO_PIN_12)
  {
    bno085_pin(0U, level);
  }
  if (k == PORT_D && pin == GPIO_PIN_9)
  {
    bno085_pin(1U, level);
  }
  if (k == PORT_D && pin == GPIO_PIN_10)
  {
    bno085_pin(2U, level);
  }
  port->IDR = io_levels(k);
}

GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *port, uint16_t pin)
{
  const uint8_t k = io_port(port);

  return ((k < PORTS) && ((io_levels(k) & pin) != 0U)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t clock)
{
  return (clock == RCC_PERIPHCLK_SPI2) ? SPI2_KERNEL_HZ
         : (clock == RCC_PERIPHCLK_SPI4) ? SPI4_KERNEL_HZ : 0U;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
  hspi->Instance->CFG1 = hspi->Init.BaudRatePrescaler | hspi->Init.DataSize
                         | hspi->Init.FifoThreshold;
  hspi->Instance->CFG2 = hspi->Init.CLKPolarity | hspi->Init.CLKPhase | hspi->Init.NSS
                         | hspi->Init.FirstBit;
  hspi->Instance->CR1 = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi)
{
  hspi->Instance->CR1 = 0U;
  return HAL_OK;
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  (void)hspi;
}

/* SPI2's bytes with the BNO085, main() waiting their time on the line. */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *tx,
                                          uint8_t *rx, uint16_t size, uint32_t timeout_ms)
{
  const bool imu = (hspi->Instance == SPI2);
  const uint32_t kernel = imu ? SPI2_KERNEL_HZ : SPI4_KERNEL_HZ;
  const uint32_t bitrate = kernel >> ((hspi->Init.BaudRatePrescaler >> SPI_PRESCALER_SHIFT) + 1U);

  (void)timeout_ms;
  for (uint16_t i = 0U; i < size; i++)
  {
    rx[i] = imu ? bno085_transmit(tx[i]) : a1335_transmit(tx[i]);
  }
  if (bitrate != 0U)
  {
    native_wait(((uint64_t)size * BITS_PER_BYTE * SystemCoreClock) / bitrate);
  }
  return HAL_OK;
}

/* A buffer's address from the 32 bits a stream holds: the high half this image's own. */
static uint8_t *io_address(uint32_t low)
{
  const uintptr_t high = (uintptr_t)native_gpio & ~(uintptr_t)UINT32_MAX;

  return (uint8_t *)(high | (uintptr_t)low);
}

/* SPI4's packet by DMA1, once started: stream 1 out, stream 0 in, the receiver's end. */
void native_io_poll(void)
{
  DMA_Stream_TypeDef *const rx = DMA1_Stream0;
  DMA_Stream_TypeDef *const tx = DMA1_Stream1;

  if (((SPI4->CR1 & SPI_CR1_CSTART) == 0U) || ((rx->CR & DMA_SxCR_EN) == 0U)
      || ((tx->CR & DMA_SxCR_EN) == 0U))
  {
    return;
  }
  uint8_t *in = io_address(rx->M0AR);
  const uint8_t *out = io_address(tx->M0AR);
  const uint32_t n = (tx->NDTR < rx->NDTR) ? tx->NDTR : rx->NDTR;

  for (uint32_t i = 0U; i < n; i++)
  {
    in[i] = a1335_transmit(out[i]);
  }
  rx->NDTR = 0U;
  tx->NDTR = 0U;
  rx->CR &= ~DMA_SxCR_EN;
  tx->CR &= ~DMA_SxCR_EN;
  SPI4->CR1 &= ~SPI_CR1_CSTART;
  SPI4->SR |= SPI_SR_EOT;
  DMA1->LISR |= DMA_LISR_TCIF0 | DMA_LISR_TCIF1;
  if ((rx->CR & DMA_SxCR_TCIE) != 0U)
  {
    native_irq(DMA1_Stream0_IRQn, DMA1_Stream0_IRQHandler);
  }
}
