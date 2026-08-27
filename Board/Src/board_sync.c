/**
  ******************************************************************************
  * @file    board_sync.c
  * @brief   Phase current sampled where the bridge is quiet: TIM1 triggers,
  *          three ADCs convert at once, this latches the result.
  *
  * The board's channel map already makes the hard part easy: Phase U is on
  * ADC3, V on ADC1 and W on ADC2 - one phase per converter - so three
  * converters started by one timer event sample the same instant by
  * construction. No dual or triple mode is needed to get simultaneity.
  *
  * This is a SECOND acquisition path, not a change to the meter. The
  * instrumentation reads in board_adc.c use the regular group, one channel
  * at a time, reconfigured per read. A current loop cannot: it needs three
  * channels converted together at a point the timer chooses. So the phases
  * get the INJECTED group, which has its own sequence, its own results and
  * its own trigger, and which preempts rather than disturbs the regular one.
  *
  * Nothing here is configured yet. TIM1 is not in the .ioc's peripheral list
  * and no ADC has an injected group, so Board_SyncReady() is false and every
  * entry point refuses. What this file settles is the shape and the
  * interlocks; what it waits for is the timer.
  ******************************************************************************
  */
#include "board.h"
#include "stm32h7xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/** Phase order in the latched triple: U, V, W - the order board_adc lists. */
#define SYNC_U 0U
#define SYNC_V 1U
#define SYNC_W 2U

/** ADC channels the phases sit on, from the table in board_adc.c. That table
    is the only place that says which phase is where; this mirrors the three
    it needs and nothing else. */
#define SYNC_U_CHANNEL ADC_CHANNEL_1     /* ADC3 IN1,  PC3_C/PC2_C */
#define SYNC_V_CHANNEL ADC_CHANNEL_3     /* ADC1 IN3,  PA6/PA7     */
#define SYNC_W_CHANNEL ADC_CHANNEL_4     /* ADC2 IN4,  PC4/PC5     */

static bool SYNC_ConfigPhase(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  /* Through HAL, not by writing PCSEL directly. Two reasons, one of which
     cost a build: ADC_CHANNEL_n is an encoded register value and not a
     channel index, so `1 << ADC_CHANNEL_1` shifts past the width of the
     type - the compiler caught that. The other is that a differential
     channel needs its negative input selected too, and HAL knows which pin
     that is. Measured on target and recorded in board_adc.c: a differential
     read leaves ADC3 PCSEL at 0xC03, four bits for two channels. */
  ADC_InjectionConfTypeDef in = {0};

  in.InjectedChannel = channel;
  in.InjectedRank = ADC_INJECTED_RANK_1;
  in.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  in.InjectedSingleDiff = ADC_DIFFERENTIAL_ENDED;
  in.InjectedOffsetNumber = ADC_OFFSET_NONE;
  in.InjectedNbrOfConversion = 1U;
  in.InjectedDiscontinuousConvMode = DISABLE;
  in.AutoInjectedConv = DISABLE;
  in.QueueInjectedContext = DISABLE;
  in.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  in.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  in.InjecOversamplingMode = DISABLE;

  return (HAL_ADCEx_InjectedConfigChannel(hadc, &in) == HAL_OK);
}


static board_sync_sample_t s_latest;
static uint32_t s_updates;
static uint32_t s_overruns;
static bool s_armed;


bool Board_SyncArmed(void)
{
  return s_armed;
}


bool Board_SyncReady(void)
{
  /* Two things have to be true, and neither is yet: a timer to trigger from,
     and an injected sequence to trigger. JSQR reads zero on an ADC whose
     injected group was never set up, which is exactly what "CubeMX has not
     generated it" looks like from here. */
  if (!Board_PwmReady())
  {
    return false;
  }
  return (hadc1.Instance->JSQR != 0U)
      && (hadc2.Instance->JSQR != 0U)
      && (hadc3.Instance->JSQR != 0U);
}


void Board_SyncLatest(board_sync_sample_t *out)
{
  if (out == NULL)
  {
    return;
  }

  /* Copied under a disabled interrupt, not field by field: the loop writes
     all three phases from one conversion and a reader that caught two of
     them from this triple and one from the last would see a current sum that
     never existed. */
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  *out = s_latest;
  if (!masked)
  {
    __enable_irq();
  }
}


void Board_SyncCounts(uint32_t *updates, uint32_t *overruns)
{
  if (updates != NULL)
  {
    *updates = s_updates;
  }
  if (overruns != NULL)
  {
    *overruns = s_overruns;
  }
}


void Board_SyncOnInjected(const void *hadc)
{
  /* Called from the injected end-of-sequence callback. Short on purpose: it
     latches and returns. Anything that talks - a printf, a frame - inside
     here corrupts RTU framing and latches a UART overrun, which on this
     silicon kills reception for good (invariant 5). */
  if (!s_armed)
  {
    return;
  }

  if (hadc == (const void *)&hadc3)
  {
    s_latest.phase[SYNC_U] = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc3,
                                                                ADC_INJECTED_RANK_1);
    s_latest.phase[SYNC_V] = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1,
                                                                ADC_INJECTED_RANK_1);
    s_latest.phase[SYNC_W] = (int16_t)HAL_ADCEx_InjectedGetValue(&hadc2,
                                                                ADC_INJECTED_RANK_1);
    s_latest.at = TIM1->CNT;
    s_updates++;
  }
}


void Board_SyncOverrun(void)
{
  s_overruns++;
}


bool Board_SyncArm(void)
{
  if (!Board_SyncReady())
  {
    return false;
  }
  if (s_armed)
  {
    return true;
  }

  /* PCSEL is the trap this board has already been caught by twice, and the
     injected path meets it from the other side. The meter clears PCSEL and
     selects one channel per read; an injected sequence needs all three
     phases selected at once and left that way. So the two paths cannot both
     own the converters, and this is where that is decided - see
     Board_AdcMeterAllowed().

     Invariant 6 says every read path configures its channel and clears
     PCSEL. This path does it once, here, rather than per conversion,
     because a hardware trigger leaves nowhere to do it per conversion. */
  if (!SYNC_ConfigPhase(&hadc3, SYNC_U_CHANNEL)
      || !SYNC_ConfigPhase(&hadc1, SYNC_V_CHANNEL)
      || !SYNC_ConfigPhase(&hadc2, SYNC_W_CHANNEL))
  {
    return false;
  }

  s_latest.phase[SYNC_U] = 0;
  s_latest.phase[SYNC_V] = 0;
  s_latest.phase[SYNC_W] = 0;
  s_latest.at = 0U;
  s_updates = 0U;
  s_overruns = 0U;

  if (HAL_ADCEx_InjectedStart_IT(&hadc3) != HAL_OK
      || HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK
      || HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
  {
    Board_SyncDisarm();
    return false;
  }

  s_armed = true;
  return true;
}


void Board_SyncDisarm(void)
{
  s_armed = false;

  if (Board_PwmReady())
  {
    (void)HAL_ADCEx_InjectedStop_IT(&hadc3);
    (void)HAL_ADCEx_InjectedStop(&hadc1);
    (void)HAL_ADCEx_InjectedStop(&hadc2);
  }
}


void Board_SyncState(board_sync_state_t *out)
{
  if (out == NULL)
  {
    return;
  }

  out->ready = Board_SyncReady();
  out->armed = s_armed;
  out->updates = s_updates;
  out->overruns = s_overruns;
  Board_SyncLatest(&out->latest);
}


/* HAL's weak callbacks, overridden here rather than in Core/: main.c holds
   CubeMX functions and the two poll calls, and this is neither. ADC3_IRQHandler
   is generated - the NVIC entry survived CubeIDE's rewrite even though the
   TIM1 block did not - so the chain runs the moment an injected group exists
   to complete. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  Board_SyncOnInjected(hadc);
}


void HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc)
{
  /* The trigger arrived before the last sequence finished. Counted rather
     than acted on: what it means depends on the sample point, which is a
     TIM1 setting nobody has chosen yet. */
  (void)hadc;
  Board_SyncOverrun();
}
