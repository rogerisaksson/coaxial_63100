/** stm32h7xx.h - The device and its HAL on this host, as native.c runs them. */

/* What board/src's files built natively take: TIM1, GPIOA-K, RCC, ADC1-3, SPI2 and SPI4, DMA1
   and DMAMUX1 as structs, the bits CMSIS's stm32h753xx.h and HAL's headers give them, the HAL
   calls board/native answers. */
#ifndef STM32H7XX_H
#define STM32H7XX_H

#include <stdint.h>

/* SYSCLK in Hz, as CMSIS's system_stm32h7xx.h names it (board/fake/fake_board.c's). */
extern uint32_t SystemCoreClock;

/* The registers the files touch. */
typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR5;
  volatile uint32_t BDTR;
} TIM_TypeDef;

typedef struct
{
  volatile uint32_t MODER;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
} GPIO_TypeDef;

typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CFG1;
  volatile uint32_t CFG2;
  volatile uint32_t SR;
  volatile uint32_t IFCR;
  volatile uint32_t TXDR;
  volatile uint32_t RXDR;
} SPI_TypeDef;

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t NDTR;
  volatile uint32_t PAR;
  volatile uint32_t M0AR;
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;
  volatile uint32_t LIFCR;
} DMA_TypeDef;

typedef struct
{
  volatile uint32_t CCR;
} DMAMUX_Channel_TypeDef;

typedef struct
{
  volatile uint32_t APB2ENR;
} RCC_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t PCSEL;
  volatile uint32_t DR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
} ADC_TypeDef;

extern TIM_TypeDef  native_tim1;
extern GPIO_TypeDef native_gpio[11];
extern RCC_TypeDef  native_rcc;
extern ADC_TypeDef  native_adc3;
extern SPI_TypeDef  native_spi2;
extern SPI_TypeDef  native_spi4;
extern DMA_TypeDef  native_dma1;
extern DMA_Stream_TypeDef native_dma1_stream[2];
extern DMAMUX_Channel_TypeDef native_dmamux1[2];

#define TIM1  (&native_tim1)
#define GPIOA (&native_gpio[0])
#define GPIOB (&native_gpio[1])
#define GPIOC (&native_gpio[2])
#define GPIOD (&native_gpio[3])
#define GPIOE (&native_gpio[4])
#define GPIOF (&native_gpio[5])
#define GPIOG (&native_gpio[6])
#define GPIOH (&native_gpio[7])
#define GPIOI (&native_gpio[8])
#define GPIOJ (&native_gpio[9])
#define GPIOK (&native_gpio[10])
#define RCC   (&native_rcc)
#define ADC3  (&native_adc3)
#define SPI2  (&native_spi2)
#define SPI4  (&native_spi4)
#define DMA1  (&native_dma1)
#define DMA1_Stream0 (&native_dma1_stream[0])
#define DMA1_Stream1 (&native_dma1_stream[1])
#define DMAMUX1_Channel0 (&native_dmamux1[0])
#define DMAMUX1_Channel1 (&native_dmamux1[1])

typedef enum
{
  DMA1_Stream0_IRQn = 11,
  TIM1_UP_IRQn = 25
} IRQn_Type;

#define TIM_CR1_CEN          (0x1UL << 0U)
#define TIM_CR1_DIR          (0x1UL << 4U)
#define TIM_CR1_CMS          (0x3UL << 5U)
#define TIM_CR1_ARPE         (0x1UL << 7U)
#define TIM_CR2_MMS2         (0xFUL << 20U)
#define TIM_TRGO2_OC5REF     (0x8UL << 20U)
#define TIM_DIER_UIE         (0x1UL << 0U)
#define TIM_SR_UIF           (0x1UL << 0U)
#define TIM_SR_BIF           (0x1UL << 7U)
#define TIM_BDTR_DTG         (0xFFUL << 0U)
#define TIM_BDTR_BKE         (0x1UL << 12U)
#define TIM_BDTR_BKP         (0x1UL << 13U)
#define TIM_BDTR_MOE         (0x1UL << 15U)
#define TIM_CCER_CC1E        (0x1UL << 0U)
#define TIM_CCER_CC1NE       (0x1UL << 2U)
#define TIM_CCER_CC2E        (0x1UL << 4U)
#define TIM_CCER_CC2NE       (0x1UL << 6U)
#define TIM_CCER_CC3E        (0x1UL << 8U)
#define TIM_CCER_CC3NE       (0x1UL << 10U)
#define RCC_APB2ENR_TIM1EN   (0x1UL << 0U)
#define GPIO_BSRR_BR0_Pos    (16U)

#define SPI_CR1_SPE          (0x1UL << 0U)
#define SPI_CR1_CSTART       (0x1UL << 9U)
#define SPI_CFG1_RXDMAEN     (0x1UL << 14U)
#define SPI_CFG1_TXDMAEN     (0x1UL << 15U)
#define SPI_IFCR_EOTC        (0x1UL << 3U)
#define SPI_IFCR_TXTFC       (0x1UL << 4U)
#define DMA_SxCR_EN          (0x1UL << 0U)
#define DMA_SxCR_TCIE        (0x1UL << 4U)
#define DMA_SxCR_DIR_0       (0x1UL << 6U)
#define DMA_SxCR_MINC        (0x1UL << 10U)
#define DMA_LIFCR_CFEIF0     (0x1UL << 0U)
#define DMA_LIFCR_CDMEIF0    (0x1UL << 2U)
#define DMA_LIFCR_CTEIF0     (0x1UL << 3U)
#define DMA_LIFCR_CHTIF0     (0x1UL << 4U)
#define DMA_LIFCR_CTCIF0     (0x1UL << 5U)
#define DMA_LIFCR_CFEIF1     (0x1UL << 6U)
#define DMA_LIFCR_CDMEIF1    (0x1UL << 8U)
#define DMA_LIFCR_CTEIF1     (0x1UL << 9U)
#define DMA_LIFCR_CHTIF1     (0x1UL << 10U)
#define DMA_LIFCR_CTCIF1     (0x1UL << 11U)
#define DMA_REQUEST_SPI4_RX  83U
#define DMA_REQUEST_SPI4_TX  84U

#define ADC_FLAG_OVR         (0x1UL << 4U)
#define ADC_FLAG_JEOC        (0x1UL << 5U)
#define ADC_FLAG_JEOS        (0x1UL << 6U)
#define ADC_FLAG_AWD1        (0x1UL << 7U)
#define ADC_FLAG_AWD2        (0x1UL << 8U)
#define ADC_FLAG_AWD3        (0x1UL << 9U)
#define ADC_FLAG_JQOVF       (0x1UL << 10U)

#define SET_BIT(REG, BIT)    ((REG) |= (BIT))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
  ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

/* PRIMASK (board_irq.h's); WFI asleep to the next interrupt (native.c). */
extern uint32_t native_primask;
void native_wfi(void);
static inline void __disable_irq(void) { native_primask = 1U; }
static inline void __enable_irq(void) { native_primask = 0U; }
static inline void __WFI(void) { native_wfi(); }
static inline void __DSB(void) { }

/* ---- HAL ------------------------------------------------------------------------------ */

typedef enum
{
  HAL_OK = 0,
  HAL_ERROR = 1
} HAL_StatusTypeDef;

#define DISABLE 0U
#define ENABLE  1U

typedef struct
{
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

#define GPIO_PIN_0                 (1U << 0U)
#define GPIO_PIN_1                 (1U << 1U)
#define GPIO_PIN_2                 (1U << 2U)
#define GPIO_PIN_3                 (1U << 3U)
#define GPIO_PIN_4                 (1U << 4U)
#define GPIO_PIN_5                 (1U << 5U)
#define GPIO_PIN_6                 (1U << 6U)
#define GPIO_PIN_7                 (1U << 7U)
#define GPIO_PIN_8                 (1U << 8U)
#define GPIO_PIN_9                 (1U << 9U)
#define GPIO_PIN_10                (1U << 10U)
#define GPIO_PIN_11                (1U << 11U)
#define GPIO_PIN_12                (1U << 12U)
#define GPIO_PIN_13                (1U << 13U)
#define GPIO_PIN_14                (1U << 14U)
#define GPIO_PIN_15                (1U << 15U)
#define GPIO_MODE_INPUT            0x00U
#define GPIO_MODE_OUTPUT_PP        0x01U
#define GPIO_MODE_AF_PP            0x02U
#define GPIO_MODE_AF_OD            0x12U
#define GPIO_NOPULL                0x00U
#define GPIO_PULLUP                0x01U
#define GPIO_PULLDOWN              0x02U
#define GPIO_SPEED_FREQ_LOW        0x00U
#define GPIO_SPEED_FREQ_VERY_HIGH  0x03U
#define GPIO_AF1_TIM1              0x01U

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
} GPIO_PinState;

void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *init);
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *port, uint16_t pin);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);

/* The kernel clocks HARDWARE.md gives: SPI2 190 MHz (PLL1Q), SPI4 118.75 MHz (APB2). */
#define RCC_PERIPHCLK_SPI2          1U
#define RCC_PERIPHCLK_SPI4          2U
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t clock);
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE()  ((void)0)

#define SPI_DATASIZE_5BIT           0x04UL
#define SPI_DATASIZE_8BIT           0x07UL
#define SPI_POLARITY_HIGH           (0x1UL << 25U)
#define SPI_PHASE_2EDGE             (0x1UL << 24U)
#define SPI_NSS_SOFT                (0x1UL << 26U)
#define SPI_NSS_PULSE_DISABLE       0x0UL
#define SPI_FIRSTBIT_MSB            0x0UL
#define SPI_FIFO_THRESHOLD_01DATA   0x0UL
#define SPI_BAUDRATEPRESCALER_2     0x00000000UL
#define SPI_BAUDRATEPRESCALER_4     0x10000000UL
#define SPI_BAUDRATEPRESCALER_8     0x20000000UL
#define SPI_BAUDRATEPRESCALER_16    0x30000000UL
#define SPI_BAUDRATEPRESCALER_32    0x40000000UL
#define SPI_BAUDRATEPRESCALER_64    0x50000000UL
#define SPI_BAUDRATEPRESCALER_128   0x60000000UL
#define SPI_BAUDRATEPRESCALER_256   0x70000000UL

typedef struct
{
  uint32_t BaudRatePrescaler;
  uint32_t CLKPhase;
  uint32_t CLKPolarity;
  uint32_t DataSize;
  uint32_t FifoThreshold;
  uint32_t FirstBit;
  uint32_t NSS;
  uint32_t NSSPMode;
} SPI_InitTypeDef;

typedef struct
{
  SPI_TypeDef *Instance;
  SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct
{
  uint32_t unused;
} UART_HandleTypeDef;

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *tx,
                                          uint8_t *rx, uint16_t size, uint32_t timeout_ms);

/* Channels by their number; the die's sensor is ADC3's 18, marked apart from ADC1's. */
#define ADC_CHANNEL_1           1U
#define ADC_CHANNEL_3           3U
#define ADC_CHANNEL_4           4U
#define ADC_CHANNEL_5           5U
#define ADC_CHANNEL_9           9U
#define ADC_CHANNEL_10          10U
#define ADC_CHANNEL_11          11U
#define ADC_CHANNEL_18          18U
#define ADC_CHANNEL_19          19U
#define ADC_CHANNEL_TEMPSENSOR  (0x100U | 18U)
#define __LL_ADC_CHANNEL_TO_DECIMAL_NB(ch) ((ch) & 0x1FU)

#define ADC_SINGLE_ENDED        0x7FU
#define ADC_DIFFERENTIAL_ENDED  0x0100007FU
#define ADC_SCAN_ENABLE         1U
#define ADC_REGULAR_RANK_1      1U
#define ADC_INJECTED_RANK_1     1U
#define ADC_INJECTED_RANK_2     2U
#define ADC_OFFSET_NONE         0U
#define ADC_EXTERNALTRIGINJEC_T1_TRGO2          1U
#define ADC_EXTERNALTRIGINJECCONV_EDGE_RISING   1U
#define HAL_ADC_STATE_INJ_EOC   0x2000U

#define ADC_SAMPLETIME_1CYCLE_5     0U
#define ADC_SAMPLETIME_2CYCLES_5    1U
#define ADC_SAMPLETIME_8CYCLES_5    2U
#define ADC_SAMPLETIME_16CYCLES_5   3U
#define ADC_SAMPLETIME_32CYCLES_5   4U
#define ADC_SAMPLETIME_64CYCLES_5   5U
#define ADC_SAMPLETIME_387CYCLES_5  6U
#define ADC_SAMPLETIME_810CYCLES_5  7U

typedef struct
{
  uint32_t ScanConvMode;
} ADC_InitTypeDef;

typedef struct
{
  ADC_TypeDef *Instance;
  ADC_InitTypeDef Init;
  volatile uint32_t State;
} ADC_HandleTypeDef;

typedef struct
{
  uint32_t Channel;
  uint32_t Rank;
  uint32_t SamplingTime;
  uint32_t SingleDiff;
  uint32_t OffsetNumber;
  uint32_t Offset;
  uint32_t OffsetSignedSaturation;
} ADC_ChannelConfTypeDef;

typedef struct
{
  uint32_t InjectedChannel;
  uint32_t InjectedRank;
  uint32_t InjectedSamplingTime;
  uint32_t InjectedSingleDiff;
  uint32_t InjectedOffsetNumber;
  uint32_t InjectedNbrOfConversion;
  uint32_t InjectedDiscontinuousConvMode;
  uint32_t AutoInjectedConv;
  uint32_t QueueInjectedContext;
  uint32_t ExternalTrigInjecConv;
  uint32_t ExternalTrigInjecConvEdge;
  uint32_t InjecOversamplingMode;
} ADC_InjectionConfTypeDef;

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, const ADC_ChannelConfTypeDef *c);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout_ms);
uint32_t HAL_ADC_GetValue(const ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc,
                                                  const ADC_InjectionConfTypeDef *c);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc);

/* The die sensor's factory points, native.c's; stm32h7xx_ll_adc.h's arithmetic. */
extern uint16_t native_ts_cal[2];
#define LL_ADC_RESOLUTION_16B      0U
#define TEMPSENSOR_CAL1_TEMP       30L
#define TEMPSENSOR_CAL2_TEMP       110L
#define TEMPSENSOR_CAL_VREFANALOG  3300UL
#define __LL_ADC_CALC_TEMPERATURE(vref_mv, data, resolution)                        \
  (((((int32_t)(((data) * (vref_mv)) / TEMPSENSOR_CAL_VREFANALOG)                   \
      - (int32_t)native_ts_cal[0])                                                  \
     * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP))                      \
    / (int32_t)((int32_t)native_ts_cal[1] - (int32_t)native_ts_cal[0]))             \
   + TEMPSENSOR_CAL1_TEMP)

#endif /* STM32H7XX_H */
