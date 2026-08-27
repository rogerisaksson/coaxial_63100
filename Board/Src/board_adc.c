/**
  ******************************************************************************
  * @file    board_adc.c
  * @brief   ADC access for this board: the channel table and the readings.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include <math.h>


/* ADC+/- reference. VREFBUF is deliberately disabled and VREF+ left
   high-impedance, so the reference comes from the AFE - which is why every
   channel reads exact mid-scale with AFE_ON low. The AFE's own source is U2,
   a REF2033 (schematic Regulators.SchDoc), so 3.3 V is a specified part and
   not a rail nobody measured. It lives in the calibration record all the
   same, because a rig with a calibrated meter can do better than a datasheet
   tolerance and this board should keep what it learns. */
static float cal_vref(void)
{
  return (float)Board_Cal()->vref_uv / 1000000.0f;
}


/* One formula for what a code is worth, so the corrected read below cannot
   drift from the raw one above it. */
static float code_to_volts(int32_t code, uint32_t singleDiff)
{
  return (singleDiff == ADC_SINGLE_ENDED)
         ? ((float)code / 65536.0f) * cal_vref()
         : ((float)code / 32768.0f) * cal_vref();
}


/* Blocking single-shot differential read, converted to volts.
   Empirically verified against a known ~0.5 V input on ADC3 CH1 (PC2/PC3):
   the raw result is OFFSET BINARY, not two's complement - code 32768 (mid
   of the 16-bit range) means 0 V differential, not code 0. The original
   two's-complement assumption made a genuine near-zero signal (code near
   32768) read as close to full-scale negative, which is exactly the
   "-3.3000 V on all three channels" saturation we saw before anything was
   even connected. */
/* Two independent single-shot reads instead of one two-rank scan sequence -
   reconfigures the channel between reads and reuses the same proven
   Start/PollForConversion/GetValue/Stop pattern already working for
   ADC1/ADC2, rather than trusting an unverified assumption about how HAL
   polls multiple ranks within one scan. Confirmed by hardware measurement
   (~1 V applied directly on PC0) that the earlier scan-based version read
   0 V on CH10 even though the pin genuinely had signal on it - i.e. the
   two-rank scan approach was the bug, not the wiring. */
/* General single-channel read: reconfigures the given ADC's rank-1 channel
   and does one Start/PollForConversion/GetValue/Stop cycle.
   Deliberately not hardware scan mode (multiple ranks in one Start), for the
   reason recorded above: it silently returned 0 on a live signal and was
   never explained. Sequential single-shot reads are slower and proven. */
static bool ADC_ReadOneChannel(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t singleDiff,
                                int32_t *outRaw, float *outVolts)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  *outRaw = 0;
  *outVolts = 0.0f;

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = singleDiff;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;

  /* HAL only ORs into PCSEL and never clears it, so every channel ever
     configured on this ADC stays preselected and connected to the sampling
     network (measured on target: ADC3 PCSEL = 0xC03, i.e. ch 0/1/10/11 all
     live at once). Clear it and let HAL_ADC_ConfigChannel select just this
     channel - for differential mode it also sets the negative input's bit
     via ADC_CHANNEL_DIFF_NEG_INPUT, so that mapping is not repeated here.
     Safe because HAL_ADC_Stop disables the ADC and PCSEL is writable with
     ADEN = 0. */
  /* Not while the current loop owns the converters. The injected sequence
     leaves all three phase channels selected in PCSEL and is triggered by
     the timer; this path clears PCSEL and selects one. Whichever ran second
     would read the other's channels - the same PCSEL trap that has caught
     this board twice, arriving from a third direction. */
  if (Board_SyncArmed())
  {
    return false;
  }

  hadc->Instance->PCSEL = 0U;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK || HAL_ADC_Start(hadc) != HAL_OK)
  {
    return false;
  }

  /* A timed-out conversion used to leave *outRaw at 0 and say nothing. On a
     differential channel code 0 is itself a valid reading - 0 V - so a failed
     conversion was indistinguishable from a measurement, which is the one
     thing this board must never produce. */
  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
  {
    HAL_ADC_Stop(hadc);
    return false;
  }

  const uint32_t raw = HAL_ADC_GetValue(hadc);

  *outRaw = (singleDiff == ADC_SINGLE_ENDED)
            ? (int32_t)raw                  /* 0..65535, 0 = 0 V           */
            : (int32_t)raw - 32768;         /* offset binary, 32768 = 0 V  */
  *outVolts = code_to_volts(*outRaw, singleDiff);

  HAL_ADC_Stop(hadc);
  return true;
}

/* NTC on PB0 (ADC1 IN9): 3.3V -> NTC (high side) -> PB0 -> 10k fixed (low
   side) -> GND. Thermistor is Murata NCU18XH103D60RB: R25=10k +/-0.5%,
   B25/50=3380K +/-0.7% - confirmed against Murata's published spec via web
   search on 2026-08-19, not assumed.

   The four numbers now come from the calibration record rather than from
   #defines here, so a board with a different thermistor is a record edit
   and not a rebuild. The defaults in board_cal.c are these same values. */
static float NTC_VoltsToCelsius(float v_node)
{
  const board_cal_t *cal = Board_Cal();

  if (v_node <= 0.0f || v_node >= cal_vref())
  {
    return NAN; /* divider math breaks down at the rails */
  }

  const float t25 = (float)cal->ntc_t25_ck / 100.0f;
  const float beta = (float)cal->ntc_beta_mk / 1000.0f;
  float r_ntc = (float)cal->ntc_rfixed_ohm * (cal_vref() / v_node - 1.0f);
  float inv_T = (1.0f / t25) + (1.0f / beta) *
                logf(r_ntc / (float)cal->ntc_r25_ohm);
  return (1.0f / inv_T) - 273.15f;
}

/* PC0/IN10 is fed through an external 49.9k/2.2k resistor divider (R12/R11,
   top/bottom to GND), so the pin voltage is only 2.2/(49.9+2.2) of the real
   DC bus voltage. Scale back up to get the actual source voltage. */
static float DC_BUS_VoltsFromDivider(float v_node)
{
  const board_cal_t *cal = Board_Cal();

  return v_node * (float)(cal->bus_r_top_ohm + cal->bus_r_bottom_ohm) /
         (float)cal->bus_r_bottom_ohm;
}

/* A phase code is the drop across RU1||RU2 - two 7 mohm WSHM2818, so 3.5
   mohm - seen through the THS4551's Rf 1.5k over Rg 330. Traced off the
   schematic and never measured, which is why both live in the record where a
   rig can span them against a clamp meter rather than as constants only a
   rebuild can change. */
static float PHASE_AmpsFromShunt(float v_pin)
{
  const board_cal_t *cal = Board_Cal();
  const float volts_per_amp = ((float)cal->shunt_uohm / 1000000.0f) *
                              ((float)cal->amp_gain_ppm / 1000000.0f);

  return v_pin / volts_per_amp;
}

/* Pass 1: every single-ended channel (these ride on the same ADC silicon as
   a phase, but aren't phase current/voltage themselves - labelled by pin/
   purpose rather than U/V/W). */
/* One row per configured ADC channel, read and printed in a single table.

   The "scaled" and "unit" columns are left blank where no physical quantity
   is defined for that input, rather than filled with the pin voltage dressed
   up as something it is not. PB1/IN5 and PC1/IN11 have no assigned signal at
   all - only a pin - so they stay blank.

   The three phase inputs used to be blank for the same reason. They are not
   any more: the shunt and the amplifier chain were traced off the schematic
   on 2026-08-26 and both now sit in the calibration record. */
typedef enum
{
  ADC_UNIT_NONE = 0,   /* leave the scaled/unit columns empty */
  ADC_UNIT_DCBUS,      /* volts at the DC bus, via the external divider */
  ADC_UNIT_NTC,        /* degrees C, via the R25/B thermistor conversion */
  ADC_UNIT_PHASE       /* amperes, via the shunt and the amplifier gain */
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
} AdcChannelDesc;

/* Which ADC carries which phase is fixed by the pinout, not by preference:
   U on ADC3, V on ADC1, W on ADC2. This table is the only place that says so
   - the aliases and per-phase channel macros that used to repeat it here are
   gone, because every reader now goes through read_index below. */
static const AdcChannelDesc s_adcTable[] =
{
  { &hadc3, "ADC3", ADC_CHANNEL_1,  "IN1",  "PC3_C/PC2_C", ADC_DIFFERENTIAL_ENDED, "Phase U", ADC_UNIT_PHASE  },
  { &hadc1, "ADC1", ADC_CHANNEL_3,  "IN3",  "PA6/PA7",     ADC_DIFFERENTIAL_ENDED, "Phase V", ADC_UNIT_PHASE  },
  { &hadc2, "ADC2", ADC_CHANNEL_4,  "IN4",  "PC4/PC5",     ADC_DIFFERENTIAL_ENDED, "Phase W", ADC_UNIT_PHASE  },
  { &hadc2, "ADC2", ADC_CHANNEL_5,  "IN5",  "PB1",         ADC_SINGLE_ENDED,       "Clevel",  ADC_UNIT_NONE  },
  { &hadc1, "ADC1", ADC_CHANNEL_9,  "IN9",  "PB0",         ADC_SINGLE_ENDED,       "NTC",     ADC_UNIT_NTC   },
  { &hadc3, "ADC3", ADC_CHANNEL_10, "IN10", "PC0",         ADC_SINGLE_ENDED,       "DC bus",  ADC_UNIT_DCBUS },
  { &hadc3, "ADC3", ADC_CHANNEL_11, "IN11", "PC1",         ADC_SINGLE_ENDED,       "Cinj",    ADC_UNIT_NONE  },
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

_Static_assert(BOARD_CAL_CHANNELS ==
               (sizeof(s_adcTable) / sizeof(s_adcTable[0])),
               "the calibration record and the ADC table disagree on how "
               "many channels there are");

/* One read by table index, corrected. Every reading this file hands out goes
   through here: a calibration applied at some call sites and not others is
   harder to reason about than none at all. The raw read above stays
   uncorrected, because zeroing has to measure what the ADC actually said. */
static bool read_index(uint8_t index, int32_t *raw, float *volts)
{
  const AdcChannelDesc *d = &s_adcTable[index];

  if (!ADC_ReadOneChannel(d->hadc, d->channel, d->singleDiff, raw, volts))
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

  *microvolts = (int32_t)(v * 1000000.0f);
  *scaled     = 0;

  if (d->unit == ADC_UNIT_DCBUS)
  {
    *scaled = (int32_t)(DC_BUS_VoltsFromDivider(v) * 1000.0f);
  }

  if (d->unit == ADC_UNIT_NTC)
  {
    const float c = NTC_VoltsToCelsius(v);
    *scaled = isnan(c) ? 0 : (int32_t)(c * 100.0f);
  }

  if (d->unit == ADC_UNIT_PHASE)
  {
    *scaled = (int32_t)(PHASE_AmpsFromShunt(v) * 1000.0f);
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

  *millivolts = (int32_t)(DC_BUS_VoltsFromDivider(v) * 1000.0f);

  return true;
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

  /* NAN at the divider rails, where the resistance is not recoverable.
     Reporting that as a temperature would be a lie; fail instead. */
  if (isnan(c))
  {
    return false;
  }

  *centidegc = (int32_t)(c * 100.0f);
  return true;
}

/* Zero and span live here rather than in board_cal.c because both have to
   take a reading, and the ADC is this file's. Everything that only edits the
   record is over there. */

bool Board_CalZero(uint8_t index, int32_t *measured)
{
  int32_t offset = 0;
  int32_t gain = 0;
  int32_t raw = 0;
  float   v;

  if ((index >= Board_AdcCount()) || (measured == NULL) ||
      !Board_CalChannel(index, &offset, &gain))
  {
    return false;
  }

  const AdcChannelDesc *d = &s_adcTable[index];

  /* The uncorrected read on purpose: the offset is what the ADC said with
     nothing applied, and measuring it through the old offset would fold the
     previous zero into the new one. */
  if (!ADC_ReadOneChannel(d->hadc, d->channel, d->singleDiff, &raw, &v))
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

  if ((index >= Board_AdcCount()) || (measured == NULL) ||
      !Board_CalChannel(index, &offset, &gain))
  {
    return false;
  }

  const AdcChannelDesc *d = &s_adcTable[index];

  /* A gain trim is a scale factor, so it only means something where the
     reported quantity is linear in the code. The thermistor is logarithmic -
     "make it read 40 C" is not a factor - and a channel with no unit has
     nothing to be told. Both are refused rather than approximated. */
  if ((d->unit != ADC_UNIT_PHASE) && (d->unit != ADC_UNIT_DCBUS))
  {
    return false;
  }

  if (!ADC_ReadOneChannel(d->hadc, d->channel, d->singleDiff, &raw, &v))
  {
    return false;
  }

  const int32_t after = raw - offset;

  *measured = after;

  const float volts = code_to_volts(after, d->singleDiff);
  const float now = (d->unit == ADC_UNIT_PHASE)
                    ? (PHASE_AmpsFromShunt(volts) * 1000.0f)
                    : (DC_BUS_VoltsFromDivider(volts) * 1000.0f);

  /* No finite factor turns nothing into something. One milli-unit is the
     resolution the reference is given in, so below it there is no ratio to
     take - zero first, then span against a reference that actually moves the
     channel. */
  if ((now > -1.0f) && (now < 1.0f))
  {
    return false;
  }

  const float ppm = (((float)reference / now) - 1.0f) * 1000000.0f;

  /* Board_CalSetChannel refuses <= -1e6 for the sign flip; this catches the
     other end, where a reference off by orders of magnitude would store a
     factor nobody could later recognise as a mistake. */
  if ((ppm <= -1000000.0f) || (ppm >= 1000000000.0f))
  {
    return false;
  }

  return Board_CalSetChannel(index, offset, (int32_t)ppm);
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

  if ((samples < 1U) || (samples > 1000U))
  {
    return false;
  }

  /* Welford in one pass: no sample buffer, so this needs nothing declared
     later in the file, and there is no second loop over stored samples. */
  double  mean = 0.0;
  double  m2 = 0.0;
  int32_t lo = INT32_MAX;
  int32_t hi = INT32_MIN;

  for (uint16_t i = 0U; i < samples; i++)
  {
    int32_t raw;
    float   v;

    /* Statistics over a set with a failed conversion in it are not
       statistics. Abort rather than fold a zero into the mean. */
    if (!read_index(index, &raw, &v))
    {
      return false;
    }

    const double d = (double)raw - mean;
    mean += d / (double)(i + 1U);
    m2 += d * ((double)raw - mean);

    if (raw < lo) { lo = raw; }
    if (raw > hi) { hi = raw; }
  }

  const double var = (samples > 1U) ? (m2 / (double)(samples - 1U)) : 0.0;

  /* One LSB of a differential reading is VREF/32768. Reported in microvolts so
     no float ever goes on the wire. */
  const double lsb_uv = ((double)cal_vref() / 32768.0) * 1000000.0;

  *mean_uv   = (int32_t)(mean * lsb_uv);
  *min_raw   = lo;
  *max_raw   = hi;
  *span_raw  = (uint32_t)(hi - lo);
  *stddev_uv = (uint32_t)(sqrt(var) * lsb_uv);

  return true;
}

/* Wall-clock pacing from the cycle counter. Unsigned subtraction, so a wrap
   mid-wait is harmless - the same reason the comms stack uses CYCCNT raw. */
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
  const uint32_t per_us  = SystemCoreClock / 1000000U;

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

    out[n].index         = i;
    out[n].min_raw       = INT32_MAX;
    out[n].max_raw       = INT32_MIN;
    out[n].mean_milliraw = 0;
    out[n].sd_milliraw   = 0U;
    n++;
  }

  if (n == 0U)
  {
    return false;
  }

  /* Welford per channel, so no sample buffer is needed however long the burst. */
  double mean[BOARD_BURST_MAX_CHAN] = { 0.0 };
  double m2[BOARD_BURST_MAX_CHAN] = { 0.0 };

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

      const double delta = (double)raw - mean[c];
      mean[c] += delta / (double)(s + 1U);
      m2[c] += delta * ((double)raw - mean[c]);

      if (raw < out[c].min_raw) { out[c].min_raw = raw; }
      if (raw > out[c].max_raw) { out[c].max_raw = raw; }
    }

    if (step != 0U)
    {
      wait_until(t0, (uint32_t)(step * (uint32_t)(s + 1U)));
    }
  }

  const uint32_t elapsed_cycles = (uint32_t)(Board_Cycles() - t0);

  for (uint8_t c = 0U; c < n; c++)
  {
    const double var = (samples > 1U) ? (m2[c] / (double)(samples - 1U)) : 0.0;

    out[c].mean_milliraw = (int32_t)(mean[c] * 1000.0);
    out[c].sd_milliraw   = (uint32_t)(sqrt(var) * 1000.0);
  }

  *count      = n;
  *elapsed_us = elapsed_cycles / per_us;

  return true;
}
