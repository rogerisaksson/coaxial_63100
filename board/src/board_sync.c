/** board_sync.c - Phase currents sampled mid-period: TIM1 triggers, three ADCs. */
#include "board.h"
#include "board_irq.h"
#include "board_drive.h"
#include "stm32h7xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/** Phase order in the latched triple: U, V, W - the order board_adc lists. */
#define SYNC_U 0U
#define SYNC_V 1U
#define SYNC_W 2U

/** ADC channels the phases sit on, from the table in board_adc.c. */
#define SYNC_U_CHANNEL ADC_CHANNEL_1     /* ADC3 IN1,  PC3_C/PC2_C */
#define SYNC_V_CHANNEL ADC_CHANNEL_3     /* ADC1 IN3,  PA6/PA7 */
#define SYNC_W_CHANNEL ADC_CHANNEL_4     /* ADC2 IN4,  PC4/PC5 */

/** The DC link, rank 2 on ADC3 behind Phase U. */
#define SYNC_DCBUS_CHANNEL ADC_CHANNEL_10 /* ADC3 IN10, PC0 */

/** The NTC, rank 2 on ADC1 behind Phase V, for the same reason: the thermal
    observer reads it through the meter, and the meter is locked out for as
    long as the drive runs. */
#define SYNC_NTC_CHANNEL ADC_CHANNEL_9    /* ADC1 IN9,  PB0 */

/** How far below the top OC5REF falls. */
#define SYNC_TRIGGER_LEAD 15U

/** The injected group's state: whether it is armed and ready, the trigger,
    the latest triple and the counts of updates and overruns. */
static struct
{
  /** CCR5 as last set. */
  uint16_t trigger;

  /* THE MEAN SQUARE, ACCUMULATED WHERE THE SAMPLES ARE. */
  int64_t sq[3];
  int64_t sum[3];
  uint32_t squares;

  board_sync_sample_t latest;
  uint32_t updates;
  uint32_t overruns;
  bool armed;
} s;

static void SYNC_ConfigTrigger(void)
{
  if (s.trigger == 0U)
  {
    s.trigger = (uint16_t)(TIM1->ARR - SYNC_TRIGGER_LEAD);
  }
  /* Channel 5, not 4: CubeMX reported channel 4 in conflict with another
     peripheral and it moved to 5. */
  TIM1->CCR5 = s.trigger;
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
  s.trigger = ticks;
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
  /* Through HAL, not by writing PCSEL directly. */
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

bool Board_SyncArmed(void)
{
  return s.armed;
}

bool Board_SyncReady(void)
{
  /* A timer to trigger from, and that is all. */
  return Board_PwmReady();
}

bool Board_SyncMeanSquare(float *out)
{
  int64_t sq[3], sum[3];
  uint32_t n;

  if (out == NULL)
  {
    return false;
  }

  /* Taken and reset under one disabled interrupt, as `Board_SyncLatest`
     copies the triple: a reader that caught the sum from one period and the
     count from the next would divide by the wrong number. */
  const uint32_t masked = Board_IrqHold();
  for (uint8_t leg = 0U; leg < 3U; leg++)
  {
    sq[leg] = s.sq[leg];
    sum[leg] = s.sum[leg];
    s.sq[leg] = 0;
    s.sum[leg] = 0;
  }
  n = s.squares;
  s.squares = 0U;
  Board_IrqRelease(masked);

  if (n == 0U)
  {
    return false;
  }

  /* THE AFFINE CONVERSION UNDONE ONCE, not per sample. */
  for (uint8_t leg = 0U; leg < 3U; leg++)
  {
    const float k = Board_PhaseAmps(leg, 0);
    const float g = Board_PhaseAmps(leg, 1) - k;
    const float mean_sq = (float)((double)sq[leg] / (double)n);
    const float mean_c = (float)((double)sum[leg] / (double)n);

    out[leg] = (g * g * mean_sq) + (2.0f * g * k * mean_c) + (k * k);
    if (out[leg] < 0.0f)
    {
      out[leg] = 0.0f;
    }
  }
  return true;
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
  const uint32_t masked = Board_IrqHold();
  *out = s.latest;
  Board_IrqRelease(masked);
}

void Board_SyncOnInjected(const void *hadc)
{
  /* Called from the injected end-of-sequence callback. */
  if (!s.armed)
  {
    return;
  }

  if (hadc == (const void *)&hadc3)
  {
    /* Through Board_AdcDifferential, not a cast: JDR is offset binary and
       casting it to int16_t put every quiet phase at the negative rail -
       measured, U read -31344 where the meter read +1423. */
    /* The data registers themselves. */
    s.latest.phase[SYNC_U] = (int16_t)Board_AdcDifferential(hadc3.Instance->JDR1);
    s.latest.phase[SYNC_V] = (int16_t)Board_AdcDifferential(hadc1.Instance->JDR1);
    s.latest.phase[SYNC_W] = (int16_t)Board_AdcDifferential(hadc2.Instance->JDR1);
    for (uint8_t leg = 0U; leg < 3U; leg++)
    {
      const int32_t c = s.latest.phase[leg];

      s.sq[leg] += (int64_t)c * (int64_t)c;
      s.sum[leg] += c;
    }
    s.squares++;
    s.latest.at = TIM1->CNT;
    s.latest.dcbus = hadc3.Instance->JDR2;
    s.latest.ntc = hadc1.Instance->JDR2;
    s.updates++;

    const int16_t logged[4] = { s.latest.phase[SYNC_U], s.latest.phase[SYNC_V],
                                s.latest.phase[SYNC_W], (int16_t)s.latest.at };
    Board_LogPush(BOARD_LOG_SOURCE_PHASES, logged, 4U);
    Board_DaqOnInjected(&s.latest);
    Board_DriveOnSample(s.latest.phase, s.latest.dcbus);
  }
}

void Board_SyncOverrun(void)
{
  s.overruns++;
}

/* Scan mode on, once, on an ADC CubeMX generated without it - the two-rank
   injected sequence needs it. */
static const char *scan_mode_on(ADC_HandleTypeDef *adc, const char *refusal)
{
  if (adc->Init.ScanConvMode == ADC_SCAN_ENABLE)
  {
    return NULL;
  }
  adc->Init.ScanConvMode = ADC_SCAN_ENABLE;
  return (HAL_ADC_Init(adc) == HAL_OK) ? NULL : refusal;
}

const char *Board_SyncArm(void)
{
  if (!Board_SyncReady())
  {
    return "no timer to trigger from - TIM1 is not configured, so the "
           "firmware needs regenerating and reflashing";
  }
  if (s.armed)
  {
    return NULL;                 /* already armed is not a refusal */
  }

  /* PCSEL is the trap this board has already been caught by twice, and the
     injected path meets it from the other side. */
  /* Two ranks need scan mode. */
  const char *refused = scan_mode_on(
      &hadc3, "ADC3 would not re-initialise with scan mode on, which the "
              "two-rank injected sequence needs - reset the board");

  if (refused == NULL)
  {
    refused = scan_mode_on(
        &hadc1, "ADC1 would not re-initialise with scan mode on, which the "
                "two-rank injected sequence needs - reset the board");
  }
  if (refused != NULL)
  {
    return refused;
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

  s.latest.phase[SYNC_U] = 0;
  s.latest.phase[SYNC_V] = 0;
  s.latest.phase[SYNC_W] = 0;
  s.latest.at = 0U;
  s.updates = 0U;
  s.overruns = 0U;

  if (HAL_ADCEx_InjectedStart_IT(&hadc3) != HAL_OK
      || HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK
      || HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
  {
    Board_SyncDisarm();
    return "an injected group would not start - disarm, check AFE_ON, "
           "and arm again";
  }

  s.armed = true;
  return NULL;
}

void Board_SyncDisarm(void)
{
  s.armed = false;

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
  out->armed = s.armed;
  out->updates = s.updates;
  out->overruns = s.overruns;
  out->trigger = Board_SyncTrigger();
  Board_SyncLatest(&out->latest);
}

/* HAL's weak callbacks, overridden here rather than in core/: main.c holds
   CubeMX functions and the two poll calls, and this is neither. */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  Board_SyncOnInjected(hadc);
}

void HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef *hadc)
{
  /* The trigger arrived before the last sequence finished. */
  (void)hadc;
  Board_SyncOverrun();
}
