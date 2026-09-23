/** board_adc.c - The ADC channel table and its reads. */
#include "board.h"
#include "board_units.h"
#include "board_hw.h"

#include <math.h>

/* The converter's 16-bit result: the full scale of codes, the half of it
   that is a differential reading's span, and the mid code that reads 0 V in
   offset binary. */
#define ADC_CODES      65536.0f
#define ADC_HALF_CODES 32768.0f
#define ADC_MID_CODE   32768

/* ADC+/- reference. */
static float cal_vref(void)
{
  return (float)Board_Cal()->vref_uv / MICRO_PER_UNIT;
}

/* One formula for what a code is worth, so the corrected read below cannot
   drift from the raw one above it. */
/** The eight sampling times the H7 offers, shortest first. */
static const uint32_t SAMPLE_TIMES[] =
{
  ADC_SAMPLETIME_1CYCLE_5,   ADC_SAMPLETIME_2CYCLES_5,
  ADC_SAMPLETIME_8CYCLES_5,  ADC_SAMPLETIME_16CYCLES_5,
  ADC_SAMPLETIME_32CYCLES_5, ADC_SAMPLETIME_64CYCLES_5,
  ADC_SAMPLETIME_387CYCLES_5, ADC_SAMPLETIME_810CYCLES_5,
};
#define SAMPLE_TIME_COUNT (sizeof(SAMPLE_TIMES) / sizeof(SAMPLE_TIMES[0]))

/* 1.5 cycles is what every read used before this was settable, and it stays
   the default: FINDINGS records it as ruled out for the quiet channels
   because the 15 nF node cap supplies the S&H charge. */
static uint32_t s_sample_time = ADC_SAMPLETIME_1CYCLE_5;
static uint8_t  s_sample_index;

bool Board_AdcSetSampleTime(uint8_t index)
{
  if (index >= SAMPLE_TIME_COUNT)
  {
    return false;
  }
  s_sample_time = SAMPLE_TIMES[index];
  s_sample_index = index;
  return true;
}

uint8_t Board_AdcSampleTime(void)
{
  return s_sample_index;
}

int32_t Board_AdcDifferential(uint32_t raw)
{
  /* Offset binary, 32768 = 0 V (proven on ADC3 CH1 against 0.5 V). */
  return (int32_t)raw - ADC_MID_CODE;
}

static float code_to_volts(int32_t code, uint32_t singleDiff)
{
  return (singleDiff == ADC_SINGLE_ENDED)
         ? ((float)code / ADC_CODES) * cal_vref()
         : ((float)code / ADC_HALF_CODES) * cal_vref();
}

/* One blocking read: rank 1 reconfigured, Start/PollForConversion/GetValue/Stop. */
static bool ADC_ReadOneChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t singleDiff,
                                int32_t *outRaw, float *outVolts, uint32_t sampleTime)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  *outRaw = 0;
  *outVolts = 0.0f;

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  /* Per channel, falling back to the shared setting. */
  sConfig.SamplingTime = (sampleTime != 0U) ? sampleTime : s_sample_time;
  sConfig.SingleDiff = singleDiff;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;

  /* HAL only ORs into PCSEL and never clears it, so every channel ever
     configured on this ADC stays preselected and connected to the sampling
     network (measured on target: ADC3 PCSEL = 0xC03, i.e. */
  /* Not while the current loop owns the converters. */
  if (Board_SyncArmed())
  {
    return false;
  }

  hadc->Instance->PCSEL = 0U;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK || HAL_ADC_Start(hadc) != HAL_OK)
  {
    return false;
  }

  /* A timed-out conversion used to leave *outRaw at 0 and say nothing. */
  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
  {
    HAL_ADC_Stop(hadc);
    return false;
  }

  const uint32_t raw = HAL_ADC_GetValue(hadc);

  *outRaw = (singleDiff == ADC_SINGLE_ENDED)
            ? (int32_t)raw                  /* 0..65535, 0 = 0 V */
            : Board_AdcDifferential(raw);
  *outVolts = code_to_volts(*outRaw, singleDiff);

  HAL_ADC_Stop(hadc);
  return true;
}

/* NTC on PB0 (ADC1 IN9): 3.3V -> NTC (high side) -> PB0 -> 10k fixed (low
   side) -> GND. */
static float NTC_VoltsToCelsius(float v_node)
{
  const board_cal_t *cal = Board_Cal();

  if (v_node <= 0.0f || v_node >= cal_vref())
  {
    return NAN; /* divider math breaks down at the rails */
  }

  const float t25 = (float)cal->ntc_t25_ck / 100.0f;
  const float beta = (float)cal->ntc_beta_mk / MILLI_PER_UNIT;
  float r_ntc = (float)cal->ntc_rfixed_ohm * (cal_vref() / v_node - 1.0f);
  float inv_T = (1.0f / t25) + (1.0f / beta) *
                logf(r_ntc / (float)cal->ntc_r25_ohm);
  return (1.0f / inv_T) - KELVIN_AT_ZERO_C;
}

/* PC0/IN10 is fed through an external 49.9k/2.2k resistor divider (R12/R11,
   top/bottom to GND), so the pin voltage is only 2.2/(49.9+2.2) of the real
   DC bus voltage. */
static float DC_BUS_VoltsFromDivider(float v_node)
{
  const board_cal_t *cal = Board_Cal();

  return v_node * (float)(cal->bus_r_top_ohm + cal->bus_r_bottom_ohm) /
         (float)cal->bus_r_bottom_ohm;
}

/* A phase code is the drop across RU1||RU2 - two 7 mohm WSHM2818, so 3.5
   mohm - seen through the THS4551's Rf 1.5k over Rg 330. */
static float PHASE_AmpsFromShunt(float v_pin)
{
  const board_cal_t *cal = Board_Cal();
  const float volts_per_amp = ((float)cal->shunt_uohm / MICRO_PER_UNIT) *
                              ((float)cal->amp_gain_ppm / PPM_PER_UNIT);

  return v_pin / volts_per_amp;
}

/* Pass 1: every single-ended channel (these ride on the same ADC silicon as
   a phase, but aren't phase current/voltage themselves - labelled by pin/
   purpose rather than U/V/W). */
/* One row per configured ADC channel, read and printed in a single table. */
typedef enum
{
  ADC_UNIT_NONE = 0,   /* leave the scaled/unit columns empty */
  ADC_UNIT_DCBUS,      /* volts at the DC bus, via the external divider */
  ADC_UNIT_NTC,        /* degrees C, via the R25/B thermistor conversion */
  ADC_UNIT_PHASE,      /* amperes, via the shunt and the amplifier gain */
  /* Two more voltages, and two more dividers. */
  ADC_UNIT_RAIL5,      /* the +5 rail, through R113's 10k/10k */
  ADC_UNIT_VGATE,      /* the gate driver supply, 47k+10k over 10k */
  ADC_UNIT_DIE         /* degrees C, from the die's factory calibration */
} AdcUnit;

typedef struct
{
  ADC_HandleTypeDef *hadc;

  const char        *adcName;
  uint32_t           channel;
  const char        *chName;
  const char        *pin;
  uint32_t           singleDiff;
  const char        *signal;   /* "" where the pin has no assigned signal */
  AdcUnit            unit;
  uint32_t           sampleTime; /* 0 = use the shared setting */
} AdcChannelDesc;

/* Which ADC carries which phase is fixed by the pinout, not by preference: U
   on ADC3, V on ADC1, W on ADC2. */
static const AdcChannelDesc s_adcTable[] =
{
  { &hadc3, "ADC3", ADC_CHANNEL_1,  "IN1",  "PC3_C/PC2_C", ADC_DIFFERENTIAL_ENDED, "Phase U", ADC_UNIT_PHASE  , 0U },
  { &hadc1, "ADC1", ADC_CHANNEL_3,  "IN3",  "PA6/PA7",     ADC_DIFFERENTIAL_ENDED, "Phase V", ADC_UNIT_PHASE  , 0U },
  { &hadc2, "ADC2", ADC_CHANNEL_4,  "IN4",  "PC4/PC5",     ADC_DIFFERENTIAL_ENDED, "Phase W", ADC_UNIT_PHASE  , 0U },
  { &hadc2, "ADC2", ADC_CHANNEL_5,  "IN5",  "PB1",         ADC_SINGLE_ENDED,       "Clevel",  ADC_UNIT_NONE  , 0U },
  { &hadc1, "ADC1", ADC_CHANNEL_9,  "IN9",  "PB0",         ADC_SINGLE_ENDED,       "NTC",     ADC_UNIT_NTC   , 0U },
  { &hadc3, "ADC3", ADC_CHANNEL_10, "IN10", "PC0",         ADC_SINGLE_ENDED,       "DC bus",  ADC_UNIT_DCBUS , 0U },
  { &hadc3, "ADC3", ADC_CHANNEL_11, "IN11", "PC1",         ADC_SINGLE_ENDED,       "Cinj",    ADC_UNIT_NONE  , 0U },
  /* The two supply senses, both single-ended off ADC1. */
  { &hadc1, "ADC1", ADC_CHANNEL_18, "IN18", "PA4",         ADC_SINGLE_ENDED,       "+5V",   ADC_UNIT_RAIL5   , 0U },
  { &hadc1, "ADC1", ADC_CHANNEL_19, "IN19", "PA5",         ADC_SINGLE_ENDED,       "Vgate", ADC_UNIT_VGATE   , 0U },

  /* The die's own thermometer. */
  { &hadc3, "ADC3", ADC_CHANNEL_TEMPSENSOR, "VSENSE", "internal", ADC_SINGLE_ENDED, "MCU die", ADC_UNIT_DIE, ADC_SAMPLETIME_810CYCLES_5 },
};

uint8_t Board_AdcCount(void)
{
  return (uint8_t)(sizeof(s_adcTable) / sizeof(s_adcTable[0]));
}

/* The calibration record is indexed by these same numbers, and the record's
   length is checked against the table at compile time: a row added without a
   matching row there is a channel that would silently stop being corrected. */
#define CH_PHASE_U 0U
#define CH_PHASE_V 1U
#define CH_PHASE_W 2U
#define CH_NTC     4U
#define CH_DCBUS   5U
#define CH_MCU_DIE 9U   /* last row: the internal sensor */

/** Amperes from a centred phase code, channel trim included. */
float Board_PhaseAmps(uint8_t leg, int32_t centred)
{
  static const uint8_t index[3] = { CH_PHASE_U, CH_PHASE_V, CH_PHASE_W };

  if (leg >= 3U)
  {
    return 0.0f;
  }
  return PHASE_AmpsFromShunt(code_to_volts(Board_CalApply(index[leg], centred),
                                           ADC_DIFFERENTIAL_ENDED));
}

bool Board_AdcIsPhase(uint8_t index)
{
  return (index == CH_PHASE_U) || (index == CH_PHASE_V) ||
         (index == CH_PHASE_W);
}

int32_t Board_AdcPhaseSlot(uint8_t index, const int16_t *phase)
{
  /* The injected triple is U, V, W in that order - board_sync.c's SYNC_U,
     SYNC_V, SYNC_W - and so is the channel table's first three rows. */
  if (phase == NULL)
  {
    return 0;
  }
  if (index == CH_PHASE_U) { return phase[0]; }
  if (index == CH_PHASE_V) { return phase[1]; }
  return phase[2];
}

bool Board_AdcInjected(uint8_t index)
{
  /* WHAT THE SEQUENCE ACTUALLY CONVERTS, which is more than the triple: rank
     2 carries the DC link on ADC3 and the NTC on ADC1, and both are latched
     at the same instant as the phases. */
  return Board_AdcIsPhase(index) || (index == CH_DCBUS) ||
         (index == CH_NTC);
}

int32_t Board_AdcInjectedSlot(uint8_t index,
                              const board_sync_sample_t *sample)
{
  if (sample == NULL)
  {
    return 0;
  }
  if (index == CH_DCBUS) { return (int32_t)sample->dcbus; }
  if (index == CH_NTC)   { return (int32_t)sample->ntc; }
  return Board_AdcPhaseSlot(index, sample->phase);
}

_Static_assert(BOARD_CAL_CHANNELS ==
               (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "the calibration record and the ADC table disagree on how "
               "many channels there are");

/* The CH_* constants are POSITIONS in the table above, and read_index takes
   them without a bounds check - it is called from paths that pass a
   constant, so the check would only ever fire on a table that had already
   been edited wrong. */
_Static_assert(CH_MCU_DIE < (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "CH_MCU_DIE is past the end of the ADC table");

/* The acquisition task's arrays are sized by their own constant, and it was
   left at nine when the die sensor made the table ten. */
_Static_assert(BOARD_DAQ_MAX_CHANNELS ==
               (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "the acquisition task cannot reach every ADC channel");
_Static_assert(CH_DCBUS < (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "CH_DCBUS is past the end of the ADC table");
_Static_assert(CH_NTC < (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "CH_NTC is past the end of the ADC table");
_Static_assert(CH_PHASE_W < (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "CH_PHASE_W is past the end of the ADC table");

/* One uncorrected read of a table row: the code as the converter gave it. */
static bool read_row(const AdcChannelDesc *d, int32_t *raw, float *volts)
{
  return ADC_ReadOneChannel(d->hadc, d->channel, d->singleDiff, raw, volts,
                            d->sampleTime);
}

/* One read by table index, corrected. */
static bool read_index(uint8_t index, int32_t *raw, float *volts)
{
  const AdcChannelDesc *d = &s_adcTable[index];

  /* The meter is locked out while the injected group owns PCSEL, but two
     single-ended channels ride that group as rank 2 - the DC link on ADC3,
     the NTC on ADC1 - so the thermal observer keeps its thermometer and the
     link keeps reading under the drive. */
  if (Board_SyncArmed() && ((index == CH_NTC) || (index == CH_DCBUS)))
  {
    board_sync_sample_t latched;

    Board_SyncLatest(&latched);
    *raw = Board_CalApply(index, (int32_t)((index == CH_NTC)
                                           ? latched.ntc : latched.dcbus));
    *volts = code_to_volts(*raw, ADC_SINGLE_ENDED);
    return true;
  }

  if (!read_row(d, raw, volts))
  {
    return false;
  }

  *raw = Board_CalApply(index, *raw);
  *volts = code_to_volts(*raw, d->singleDiff);
  return true;
}

static uint8_t board_adc_index(const ADC_HandleTypeDef *h)
{
  if (h == &hadc1) { return 1U; }
  if (h == &hadc2) { return 2U; }
  if (h == &hadc3) { return 3U; }
  return 0U;
}

static uint8_t board_unit(AdcUnit u)
{
  if (u == ADC_UNIT_DCBUS) { return BOARD_UNIT_MILLIVOLT; }
  if (u == ADC_UNIT_NTC)   { return BOARD_UNIT_CENTIDEGC; }
  if (u == ADC_UNIT_PHASE) { return BOARD_UNIT_MILLIAMP; }
  if (u == ADC_UNIT_RAIL5) { return BOARD_UNIT_MILLIVOLT; }
  if (u == ADC_UNIT_VGATE) { return BOARD_UNIT_MILLIVOLT; }
  if (u == ADC_UNIT_DIE)   { return BOARD_UNIT_CENTIDEGC; }
  return BOARD_UNIT_NONE;
}

bool Board_AdcChan(uint8_t index, board_chan_t *info)
{
  if ((index >= Board_AdcCount()) || (info == NULL))
  {
    return false;
  }

  const AdcChannelDesc *d = &s_adcTable[index];

  info->adc_index    = board_adc_index(d->hadc);
  info->channel      = (uint8_t)__LL_ADC_CHANNEL_TO_DECIMAL_NB(d->channel);
  info->pin          = d->pin;
  info->differential = (d->singleDiff == ADC_DIFFERENTIAL_ENDED);
  info->signal       = d->signal;
  info->unit         = board_unit(d->unit);

  return true;
}

bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled)
{
  if ((index >= Board_AdcCount()) || (raw == NULL) || (microvolts == NULL) ||
      (scaled == NULL))
  {
    return false;
  }

  const AdcChannelDesc *d = &s_adcTable[index];
  float v;

  if (!read_index(index, raw, &v))
  {
    return false;
  }

  *microvolts = (int32_t)(v * MICRO_PER_UNIT);
  *scaled     = 0;

  if (d->unit == ADC_UNIT_DCBUS)
  {
    *scaled = (int32_t)(DC_BUS_VoltsFromDivider(v) * MILLI_PER_UNIT);
  }

  if (d->unit == ADC_UNIT_NTC)
  {
    const float c = NTC_VoltsToCelsius(v);
    *scaled = isnan(c) ? 0 : (int32_t)(c * CENTI_PER_UNIT);
  }

  if (d->unit == ADC_UNIT_PHASE)
  {
    *scaled = (int32_t)(PHASE_AmpsFromShunt(v) * MILLI_PER_UNIT);
  }

  if (d->unit == ADC_UNIT_DIE)
  {
    /* The die's own factory calibration, read from system memory. */
    *scaled = (int32_t)(__LL_ADC_CALC_TEMPERATURE(
                            Board_Cal()->vref_uv / MICRO_PER_MILLI,
                            (uint32_t)*raw, LL_ADC_RESOLUTION_16B) * 100);
  }

  /* The two rails behind a divider: the record's resistors, and no reading
     through a divider whose bottom leg is unknown. */
  const bool divided = (d->unit == ADC_UNIT_RAIL5) || (d->unit == ADC_UNIT_VGATE);
  const board_cal_t *cal = Board_Cal();
  const uint32_t top = (d->unit == ADC_UNIT_RAIL5)
                     ? cal->r5_r_top_ohm : cal->vg_r_top_ohm;
  const uint32_t bottom = (d->unit == ADC_UNIT_RAIL5)
                        ? cal->r5_r_bottom_ohm : cal->vg_r_bottom_ohm;

  if (divided && (bottom > 0UL))
  {
    *scaled = (int32_t)(v * (float)(top + bottom) / (float)bottom
                        * MILLI_PER_UNIT);
  }

  return true;
}

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w)
{
  float fu, fv, fw;

  if ((u == NULL) || (v == NULL) || (w == NULL))
  {
    return false;
  }

  /* Short-circuited: once one phase has failed the scan is refused whole,
     and the remaining conversions would only cost time to discard. */
  if (!read_index(CH_PHASE_U, u, &fu) ||
      !read_index(CH_PHASE_V, v, &fv) ||
      !read_index(CH_PHASE_W, w, &fw))
  {
    return false;
  }

  return true;
}

void Board_PhaseScale(uint8_t leg, int32_t *offset_raw, float *amps_per_code)
{
  /* Board_PhaseAmps, linearised: the record is affine in the code and the
     shunt arithmetic is linear in the volts, so one factor carries the lot. */
  static const uint8_t index[3] = { CH_PHASE_U, CH_PHASE_V, CH_PHASE_W };
  int32_t offset = 0;
  int32_t ppm = 0;

  if (leg >= 3U)
  {
    *offset_raw = 0;
    *amps_per_code = 0.0f;
    return;
  }
  (void)Board_CalChannel(index[leg], &offset, &ppm);
  *offset_raw = offset;
  *amps_per_code = (1.0f + (float)ppm / PPM_PER_UNIT)
                   * PHASE_AmpsFromShunt(code_to_volts(1, ADC_DIFFERENTIAL_ENDED));
}

void Board_DcBusScale(int32_t *offset_raw, float *volts_per_code)
{
  int32_t offset = 0;
  int32_t ppm = 0;

  (void)Board_CalChannel(CH_DCBUS, &offset, &ppm);
  *offset_raw = offset;
  *volts_per_code = (1.0f + (float)ppm / PPM_PER_UNIT)
                    * DC_BUS_VoltsFromDivider(code_to_volts(1, ADC_SINGLE_ENDED));
}

bool Board_DcBus(int32_t *raw, int32_t *millivolts)
{
  float v;

  if ((raw == NULL) || (millivolts == NULL))
  {
    return false;
  }

  if (!read_index(CH_DCBUS, raw, &v))
  {
    return false;
  }

  *millivolts = (int32_t)(DC_BUS_VoltsFromDivider(v) * MILLI_PER_UNIT);

  return true;
}

/** The MCU's own die, centi-degrees C. */
bool Board_McuDie(int32_t *raw, int32_t *centidegc)
{
  int32_t microvolts = 0;

  if ((raw == NULL) || (centidegc == NULL))
  {
    return false;
  }
  return Board_AdcRead(CH_MCU_DIE, raw, &microvolts, centidegc);
}

bool Board_Ntc(int32_t *raw, int32_t *centidegc)
{
  float v;

  if ((raw == NULL) || (centidegc == NULL))
  {
    return false;
  }

  if (!read_index(CH_NTC, raw, &v))
  {
    return false;
  }

  const float c = NTC_VoltsToCelsius(v);

  /* NAN at the divider rails, where the resistance is not recoverable. */
  if (isnan(c))
  {
    return false;
  }

  *centidegc = (int32_t)(c * CENTI_PER_UNIT);
  return true;
}

/* Zero and span live here rather than in board_cal.c because both have to
   take a reading, and the ADC is this file's. */

/* The row a zero or a span acts on, with the record's offset and gain for
   it; NULL past the table, with nowhere to report, or with no record. */
static const AdcChannelDesc *cal_row(uint8_t index, const int32_t *measured,
                                     int32_t *offset, int32_t *gain)
{
  if ((index >= Board_AdcCount()) || (measured == NULL) ||
      !Board_CalChannel(index, offset, gain))
  {
    return NULL;
  }
  return &s_adcTable[index];
}

bool Board_CalZero(uint8_t index, int32_t *measured)
{
  int32_t offset = 0;
  int32_t gain = 0;
  int32_t raw = 0;
  float   v;
  const AdcChannelDesc *d = cal_row(index, measured, &offset, &gain);

  /* The uncorrected read on purpose: the offset is what the ADC said with
     nothing applied, and measuring it through the old offset would fold the
     previous zero into the new one. */
  if ((d == NULL) || !read_row(d, &raw, &v))
  {
    return false;
  }

  *measured = raw;
  return Board_CalSetChannel(index, raw, gain);
}

bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured)
{
  int32_t offset = 0;
  int32_t gain = 0;
  int32_t raw = 0;
  float   v;
  const AdcChannelDesc *d = cal_row(index, measured, &offset, &gain);

  /* A gain trim is a scale factor, so it only means something where the
     reported quantity is linear in the code. */
  if ((d == NULL) || ((d->unit != ADC_UNIT_PHASE) && (d->unit != ADC_UNIT_DCBUS))
      || !read_row(d, &raw, &v))
  {
    return false;
  }

  const int32_t after = raw - offset;

  *measured = after;

  const float volts = code_to_volts(after, d->singleDiff);
  const float now = (d->unit == ADC_UNIT_PHASE)
                    ? (PHASE_AmpsFromShunt(volts) * MILLI_PER_UNIT)
                    : (DC_BUS_VoltsFromDivider(volts) * MILLI_PER_UNIT);

  /* No finite factor turns nothing into something. */
  if ((now > -1.0f) && (now < 1.0f))
  {
    return false;
  }

  const float ppm = (((float)reference / now) - 1.0f) * PPM_PER_UNIT;

  /* Board_CalSetChannel refuses <= -1e6 for the sign flip; this catches the
     other end, where a reference off by orders of magnitude would store a
     factor nobody could later recognise as a mistake. */
  if ((ppm <= -PPM_PER_UNIT) || (ppm >= 1000.0f * PPM_PER_UNIT))
  {
    return false;
  }

  return Board_CalSetChannel(index, offset, (int32_t)ppm);
}

/** Welford's running mean and spread with the extremes, in one pass: no
    sample buffer however long the run. */
typedef struct
{
  double   mean;
  double   m2;
  int32_t  lo;
  int32_t  hi;
  uint32_t n;
} welford_t;

#define WELFORD_EMPTY { 0.0, 0.0, INT32_MAX, INT32_MIN, 0U }

static void welford_add(welford_t *w, int32_t raw)
{
  const double d = (double)raw - w->mean;

  w->n++;
  w->mean += d / (double)w->n;
  w->m2 += d * ((double)raw - w->mean);
  w->lo = (raw < w->lo) ? raw : w->lo;
  w->hi = (raw > w->hi) ? raw : w->hi;
}

/** The sample standard deviation, in the samples' own unit. */
static double welford_sd(const welford_t *w)
{
  return (w->n > 1U) ? sqrt(w->m2 / (double)(w->n - 1U)) : 0.0;
}

bool Board_AdcNoise(uint8_t adc_index, uint16_t samples,
                    int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw,
                    uint32_t *span_raw, uint32_t *stddev_uv)
{
  uint8_t index;

  /* Which ADC carries which phase is the pinout's answer, and the table
     already holds it - these are its rows, not a second mapping. */
  if (adc_index == 1U) { index = CH_PHASE_V; }
  else if (adc_index == 2U) { index = CH_PHASE_W; }
  else if (adc_index == 3U) { index = CH_PHASE_U; }
  else { return false; }

  if ((samples < 1U) || (samples > BOARD_ADC_BURST_MAX))
  {
    return false;
  }

  welford_t w = WELFORD_EMPTY;

  for (uint16_t i = 0U; i < samples; i++)
  {
    int32_t raw;
    float   v;

    /* Statistics over a set with a failed conversion in it are not
       statistics. */
    if (!read_index(index, &raw, &v))
    {
      return false;
    }
    welford_add(&w, raw);
  }

  /* One LSB of a differential reading is VREF/32768. */
  const double lsb_uv = ((double)cal_vref() / (double)ADC_HALF_CODES)
                         * (double)MICRO_PER_UNIT;

  *mean_uv   = (int32_t)(w.mean * lsb_uv);
  *min_raw   = w.lo;
  *max_raw   = w.hi;
  *span_raw  = (uint32_t)(w.hi - w.lo);
  *stddev_uv = (uint32_t)(welford_sd(&w) * lsb_uv);

  return true;
}

/* Wall-clock pacing from the cycle counter. */
static void wait_until(uint32_t start_cycles, uint32_t target_cycles)
{
  while ((uint32_t)(Board_Cycles() - start_cycles) < target_cycles)
  {
    /* busy wait: a burst is a deliberate blocking measurement */
  }
}

bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us,
                    board_burst_t *out, uint8_t *count, uint32_t *elapsed_us)
{
  const uint8_t  total   = Board_AdcCount();
  const uint32_t per_us  = SystemCoreClock / US_PER_S;

  if ((samples < 1U) || (samples > BOARD_BURST_MAX_SAMPLES) || (mask == 0U))
  {
    return false;
  }

  /* Refuse a burst that would outlive the master's timeout rather than
     starting one and leaving the link silent for a minute. */
  if (((uint64_t)samples * (uint64_t)interval_us) > (uint64_t)BOARD_BURST_MAX_US)
  {
    return false;
  }

  uint8_t n = 0U;

  for (uint8_t i = 0U; i < total; i++)
  {
    if ((mask & (uint16_t)(1U << i)) == 0U)
    {
      continue;
    }

    out[n].index = i;
    n++;
  }

  if (n == 0U)
  {
    return false;
  }

  welford_t acc[BOARD_BURST_MAX_CHAN];

  for (uint8_t c = 0U; c < n; c++)
  {
    acc[c] = (welford_t)WELFORD_EMPTY;
  }
  const uint32_t t0 = Board_Cycles();
  const uint32_t step = interval_us * per_us;

  for (uint16_t s = 0U; s < samples; s++)
  {
    for (uint8_t c = 0U; c < n; c++)
    {
      int32_t raw = 0;
      float   v;

      if (!read_index(out[c].index, &raw, &v))
      {
        return false;
      }
      welford_add(&acc[c], raw);
    }

    if (step != 0U)
    {
      wait_until(t0, (uint32_t)(step * (uint32_t)(s + 1U)));
    }
  }

  const uint32_t elapsed_cycles = (uint32_t)(Board_Cycles() - t0);

  for (uint8_t c = 0U; c < n; c++)
  {
    out[c].min_raw       = acc[c].lo;
    out[c].max_raw       = acc[c].hi;
    out[c].mean_milliraw = (int32_t)(acc[c].mean * 1000.0);
    out[c].sd_milliraw   = (uint32_t)(welford_sd(&acc[c]) * 1000.0);
  }

  *count      = n;
  *elapsed_us = elapsed_cycles / per_us;

  return true;
}
