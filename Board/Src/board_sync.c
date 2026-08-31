/**
  ******************************************************************************
  * @file    board_sync.c
  * @brief   Phase current sampled where the stage is quiet: TIM1 triggers,
  *          three ADCs convert at once, this latches the result.
  *
  * One phase per converter - U on ADC3, V on ADC1, W on ADC2 - so one timer
  * event samples the same instant by construction. No dual or triple mode.
  *
  * A SECOND path, not a change to the meter: board_adc.c reads the regular
  * group one channel at a time, and a current loop needs three together at a
  * point the timer picks, so the phases get the INJECTED group.
  *
  * What this file owns on top of TIM1 is the sample point: CCR5 and TRGO2,
  * set here because CubeMX stores MasterOutputTrigger2 as "null" and emits
  * TIM_TRGO2_RESET.
  ******************************************************************************
  */
#include "board.h"
#include "board_drive.h"
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

/** The DC link, rank 2 on ADC3 behind Phase U. The drive needs it every
    period and the meter is locked out while this is armed; one more
    conversion on the sequence costs a third of a microsecond. */
#define SYNC_DCBUS_CHANNEL ADC_CHANNEL_10 /* ADC3 IN10, PC0         */

/** The NTC, rank 2 on ADC1 behind Phase V, for the same reason: the
    thermal observer reads it through the meter, and the meter is locked
    out for as long as the drive runs. */
#define SYNC_NTC_CHANNEL ADC_CHANNEL_9    /* ADC1 IN9,  PB0         */

/** How far below the top OC5REF falls. The trigger is its rising edge, so
    the ADC starts that far after the counter turns - inside the zero vector,
    no gate edge within the sampling window. 15 ticks is 63 ns. */
#define SYNC_TRIGGER_LEAD 15U


/** CCR5 as last set. Zero means nobody has chosen, so arming picks the
    default lead. Kept across disarm so a tuning run is not undone by it. */
static uint16_t s_trigger;


static void SYNC_ConfigTrigger(void)
{
  if (s_trigger == 0U)
  {
    s_trigger = (uint16_t)(TIM1->ARR - SYNC_TRIGGER_LEAD);
  }
  /* Channel 5, not 4: CubeMX reported channel 4 in conflict with another
     peripheral and it moved to 5. Both are internal - neither has an output
     pin - and OC5REF drives TRGO2 exactly as OC4REF did. Anything reading
     `trigger` sees the same number it always did. */
  TIM1->CCR5 = s_trigger;
  MODIFY_REG(TIM1->CR2, TIM_CR2_MMS2, TIM_TRGO2_OC5REF);
}


bool Board_SyncSetTrigger(uint16_t ticks)
{
  /* Straight into CCR5, armed or not: moving the sample point while the
     triples are running is the whole point of being able to move it. */
  if (!Board_PwmReady() || ticks > TIM1->ARR)
  {
    return false;
  }
  s_trigger = ticks;
  TIM1->CCR5 = ticks;
  return true;
}


uint16_t Board_SyncTrigger(void)
{
  return Board_PwmReady() ? (uint16_t)TIM1->CCR5 : 0U;
}

static bool SYNC_ConfigPhase(ADC_HandleTypeDef *hadc, uint32_t channel,
                             uint32_t rank, uint32_t nbr, uint32_t single_diff)
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
  in.InjectedRank = rank;
  in.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  in.InjectedSingleDiff = single_diff;
  in.InjectedOffsetNumber = ADC_OFFSET_NONE;
  in.InjectedNbrOfConversion = nbr;
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
  /* A timer to trigger from, and that is all. The injected sequences are
     this file's own - SYNC_ConfigPhase writes them at arm time - so a JSQR
     of zero here is the disarmed state, not a missing prerequisite. Reading
     it as one deadlocked Arm against Ready. */
  return Board_PwmReady();
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
    /* Through Board_AdcDifferential, not a cast: JDR is offset binary and
       casting it to int16_t put every quiet phase at the negative rail -
       measured, U read -31344 where the meter read +1423. */
    /* The data registers themselves. HAL_ADCEx_InjectedGetValue is a
       switch on the rank behind two asserts, compiled at -O0 five times
       a period; JDRx is the number it returns. */
    s_latest.phase[SYNC_U] = (int16_t)Board_AdcDifferential(hadc3.Instance->JDR1);
    s_latest.phase[SYNC_V] = (int16_t)Board_AdcDifferential(hadc1.Instance->JDR1);
    s_latest.phase[SYNC_W] = (int16_t)Board_AdcDifferential(hadc2.Instance->JDR1);
    s_latest.at = TIM1->CNT;
    s_latest.dcbus = hadc3.Instance->JDR2;
    s_latest.ntc = hadc1.Instance->JDR2;
    s_updates++;

    const int16_t logged[4] = { s_latest.phase[SYNC_U], s_latest.phase[SYNC_V],
                                s_latest.phase[SYNC_W], (int16_t)s_latest.at };
    Board_LogPush(BOARD_LOG_SOURCE_PHASES, logged, 4U);
    Board_DaqOnInjected(s_latest.phase);
    Board_DriveOnSample(s_latest.phase, s_latest.dcbus);
  }
}


void Board_SyncOverrun(void)
{
  s_overruns++;
}


const char *Board_SyncArm(void)
{
  if (!Board_SyncReady())
  {
    return "no timer to trigger from - TIM1 is not configured, so the "
           "firmware needs regenerating and reflashing";
  }
  if (s_armed)
  {
    return NULL;                 /* already armed is not a refusal */
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
  /* Two ranks need scan mode. With it off the HAL discards
     InjectedNbrOfConversion and writes JSQR from rank 1 alone - measured
     2026-08-31 over SWD, JSQR 0x2A0: JL 0, no JSQ2, the DC link reading
     zero while PCSEL already carried its channel. CubeMX generates ADC3
     with scan off because the meter converts one channel at a time, and
     it still does: the regular sequence keeps its length of one. Once,
     here, because this is the one path that needs it. */
  if (hadc3.Init.ScanConvMode != ADC_SCAN_ENABLE)
  {
    hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
      return "ADC3 would not re-initialise with scan mode on, which the "
             "two-rank injected sequence needs - reset the board";
    }
  }
  if (hadc1.Init.ScanConvMode != ADC_SCAN_ENABLE)
  {
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      return "ADC1 would not re-initialise with scan mode on, which the "
             "two-rank injected sequence needs - reset the board";
    }
  }

  if (!SYNC_ConfigPhase(&hadc3, SYNC_U_CHANNEL, ADC_INJECTED_RANK_1, 2U,
                        ADC_DIFFERENTIAL_ENDED)
      || !SYNC_ConfigPhase(&hadc3, SYNC_DCBUS_CHANNEL, ADC_INJECTED_RANK_2, 2U,
                           ADC_SINGLE_ENDED)
      || !SYNC_ConfigPhase(&hadc1, SYNC_V_CHANNEL, ADC_INJECTED_RANK_1, 2U,
                           ADC_DIFFERENTIAL_ENDED)
      || !SYNC_ConfigPhase(&hadc1, SYNC_NTC_CHANNEL, ADC_INJECTED_RANK_2, 2U,
                           ADC_SINGLE_ENDED)
      || !SYNC_ConfigPhase(&hadc2, SYNC_W_CHANNEL, ADC_INJECTED_RANK_1, 1U,
                           ADC_DIFFERENTIAL_ENDED))
  {
    return "an injected group would not configure - check AFE_ON is on "
           "and that no meter read is in flight";
  }

  SYNC_ConfigTrigger();

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
    return "an injected group would not start - disarm, check AFE_ON, "
           "and arm again";
  }

  s_armed = true;
  return NULL;
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
  out->trigger = Board_SyncTrigger();
  Board_SyncLatest(&out->latest);
}


/* HAL's weak callbacks, overridden here rather than in Core/: main.c holds
   CubeMX functions and the two poll calls, and this is neither. ADC3_IRQHandler
   is generated, so the chain runs as soon as Board_SyncArm has made an
   injected group for it to complete. */
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
