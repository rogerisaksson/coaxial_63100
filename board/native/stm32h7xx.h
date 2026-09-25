/** stm32h7xx.h - The device and its HAL on this host, as native.c runs them. */

/* What board_pwm.c, board_sync.c and board_adc.c take: TIM1, GPIOE, RCC and ADC1-3 as
   structs, the bits CMSIS's stm32h753xx.h and HAL's headers give them, the HAL calls
   native.c answers. */
#ifndef STM32H7XX_H
#define STM32H7XX_H

#include <stdint.h>

/* The registers the three files touch. */
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
  volatile uint32_t BSRR;
} GPIO_TypeDef;

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
extern GPIO_TypeDef native_gpioe;
extern RCC_TypeDef  native_rcc;
extern ADC_TypeDef  native_adc3;

#define TIM1  (&native_tim1)
#define GPIOE (&native_gpioe)
#define RCC   (&native_rcc)
#define ADC3  (&native_adc3)

typedef enum
{
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

/* PRIMASK (board_irq.h's). */
extern uint32_t native_primask;
static inline void __disable_irq(void) { native_primask = 1U; }
static inline void __enable_irq(void) { native_primask = 0U; }

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

#define GPIO_PIN_8                 (1U << 8U)
#define GPIO_PIN_9                 (1U << 9U)
#define GPIO_PIN_10                (1U << 10U)
#define GPIO_PIN_11                (1U << 11U)
#define GPIO_PIN_12                (1U << 12U)
#define GPIO_PIN_13                (1U << 13U)
#define GPIO_PIN_15                (1U << 15U)
#define GPIO_MODE_AF_PP            0x02U
#define GPIO_MODE_AF_OD            0x12U
#define GPIO_NOPULL                0x00U
#define GPIO_PULLUP                0x01U
#define GPIO_SPEED_FREQ_LOW        0x00U
#define GPIO_SPEED_FREQ_VERY_HIGH  0x03U
#define GPIO_AF1_TIM1              0x01U

void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *init);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);

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
