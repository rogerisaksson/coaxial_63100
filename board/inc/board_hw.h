/** board_hw.h - CubeMX handles and shared SPI helpers, for the board layer only. */
#ifndef BOARD_HW_H
#define BOARD_HW_H

#include "main.h"
#include "board.h"
#include "board_units.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;

/** A port letter's registers, A-K; NULL for any other letter. */
static inline GPIO_TypeDef *board_port(char port)
{
  switch (port)
  {
    case 'A': return GPIOA;
    case 'B': return GPIOB;
    case 'C': return GPIOC;
    case 'D': return GPIOD;
    case 'E': return GPIOE;
    case 'F': return GPIOF;
    case 'G': return GPIOG;
    case 'H': return GPIOH;
    case 'I': return GPIOI;
    case 'J': return GPIOJ;
    case 'K': return GPIOK;
    default:  return NULL;
  }
}

/** The fastest SPI prescaler whose bitrate is <= limit_hz, and that bitrate.
    An unclocked kernel (0) gets the slowest. */
static inline uint32_t board_spi_prescaler(uint32_t kernel_hz, uint32_t limit_hz,
                                           uint32_t *bitrate_hz)
{
  static const uint32_t DIVIDERS[] =
  {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };
  const uint32_t n = sizeof(DIVIDERS) / sizeof(DIVIDERS[0]);

  for (uint32_t i = 0U; (kernel_hz != 0U) && (i < n); i++)
  {
    if ((kernel_hz >> (i + 1U)) <= limit_hz)
    {
      *bitrate_hz = kernel_hz >> (i + 1U);
      return DIVIDERS[i];
    }
  }
  *bitrate_hz = kernel_hz >> n;
  return DIVIDERS[n - 1U];
}

/** Busy-wait `us` microseconds, feeding the STO pump meanwhile. */
static inline void board_spin_us(uint32_t us)
{
  const uint32_t start = Board_Cycles();

  while ((uint32_t)(Board_Cycles() - start) < (us * (SystemCoreClock / US_PER_S)))
  {
    Board_StoKeepalive();
  }
}

#endif /* BOARD_HW_H */
