/** fake_board.c - The board API on the host, every call answered neutrally. */

/* No part fitted, nothing read, every out-parameter zeroed, every setting taken (a NULL
   refusal): so the offline suites drive comms/ through its own wire
   (tools/cores/fakeboard.py). Generated once from the prototypes; what a check needs is
   answered by hand here - the AFE rail, the PWM, the channel table, the raw codes and
   the clocks. The record is board/src/board_cal.c over fake_flash.c, the thermal observer
   board/src/board_thermal.c, the acquisition board/src/board_daq.c. */
#include "board/adc.h"
#include "board/angle.h"
#include "board/cal.h"
#include "board/clock.h"
#include "board/ctrl.h"
#include "board/daq.h"
#include "board/handover.h"
#include "board/imu.h"
#include "board/io.h"
#include "board/log.h"
#include "board/power.h"
#include "board/pwm.h"
#include "board/selftest.h"
#include "board/sync.h"
#include "board/thermal.h"
#include "board_drive.h"
#include "board_hw.h"
#include "board_power.h"
#include "testrig.h"

#include <stddef.h>
#include <string.h>

/* Answered by hand, and remembered: the AFE rail - who holds it - and the PWM's enable,
   so a host that switches either reads back what it asked for. 50 kHz centre-aligned at
   475 MHz is ARR 4750. */
#define FAKE_PWM_PERIOD 4750U

static struct
{
  uint8_t  users;
  bool     pwm_enabled;
  uint32_t noise;
  bool     derated;
  float    derate;
} s;

/* What a temperature reads: the room, 25.00 C, the label an unpowered AFE gives the NTC. */
#define FAKE_ROOM_CENTI 2500

/* A raw code as the rail leaves it: the AFE off, the reference unpowered and every code
   exact mid-scale; on, 1..8 codes of noise above it. */
#define FAKE_MID_CODE 32768

/* A burst's pass at full speed, as the real scan of the table takes it. */
#define FAKE_PASS_US 20U

static int32_t fake_code(void)
{
  if (s.users == 0U)
  {
    return FAKE_MID_CODE;
  }
  s.noise = (s.noise * 1664525U) + 1013904223U;
  return FAKE_MID_CODE + 1 + (int32_t)(s.noise >> 29);
}

/* The board's facts, as board/src/board_adc.c's table and the clock tree have them: the
   channels in the table's order, SYSCLK 475 MHz with HCLK half of it. */
#define FAKE_SYSCLK_HZ 475000000U

uint32_t SystemCoreClock = FAKE_SYSCLK_HZ;

static const board_chan_t s_chan[] =
{
  { 3U, 1U,  "PC3_C/PC2_C", true,  "Phase U", BOARD_UNIT_MILLIAMP  },
  { 1U, 3U,  "PA6/PA7",     true,  "Phase V", BOARD_UNIT_MILLIAMP  },
  { 2U, 4U,  "PC4/PC5",     true,  "Phase W", BOARD_UNIT_MILLIAMP  },
  { 2U, 5U,  "PB1",         false, "Clevel",  BOARD_UNIT_NONE      },
  { 1U, 9U,  "PB0",         false, "NTC",     BOARD_UNIT_CENTIDEGC },
  { 3U, 10U, "PC0",         false, "DC bus",  BOARD_UNIT_MILLIVOLT },
  { 3U, 11U, "PC1",         false, "Cinj",    BOARD_UNIT_NONE      },
  { 1U, 18U, "PA4",         false, "+5V",     BOARD_UNIT_MILLIVOLT },
  { 1U, 19U, "PA5",         false, "Vgate",   BOARD_UNIT_MILLIVOLT },
  { 3U, 18U, "internal",    false, "MCU die", BOARD_UNIT_CENTIDEGC },
};

bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us, board_burst_t *out, uint8_t *count, uint32_t *elapsed_us)
{
  uint8_t n = 0U;

  if ((out == NULL) || (count == NULL) || (elapsed_us == NULL) || (samples == 0U))
  {
    return false;
  }
  for (uint8_t index = 0U; index < Board_AdcCount(); index++)
  {
    if ((mask & (1U << index)) != 0U)
    {
      const int32_t code = fake_code();

      out[n].index         = index;
      out[n].mean_milliraw = code * 1000;
      out[n].min_raw       = code;
      out[n].max_raw       = code;
      out[n].sd_milliraw   = 0U;
      n++;
    }
  }
  *count = n;
  *elapsed_us = (uint32_t)samples * ((interval_us != 0U) ? interval_us : FAKE_PASS_US);
  return n != 0U;
}

bool Board_AdcChan(uint8_t index, board_chan_t *info)
{
  if ((index >= Board_AdcCount()) || (info == NULL))
  {
    return false;
  }
  *info = s_chan[index];
  return true;
}

uint32_t Board_AdcClockHz(void)
{
  return (uint32_t)0;
}

uint8_t Board_AdcCount(void)
{
  return (uint8_t)(sizeof s_chan / sizeof s_chan[0]);
}

bool Board_AdcNoise(uint8_t adc_index, uint16_t samples, int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw, uint32_t *span_raw, uint32_t *stddev_uv)
{
  (void)adc_index;
  (void)samples;
  if (mean_uv != NULL)
  {
    memset(mean_uv, 0, sizeof *mean_uv);
  }
  if (min_raw != NULL)
  {
    memset(min_raw, 0, sizeof *min_raw);
  }
  if (max_raw != NULL)
  {
    memset(max_raw, 0, sizeof *max_raw);
  }
  if (span_raw != NULL)
  {
    memset(span_raw, 0, sizeof *span_raw);
  }
  if (stddev_uv != NULL)
  {
    memset(stddev_uv, 0, sizeof *stddev_uv);
  }
  return true;
}

bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled)
{
  if ((index >= Board_AdcCount()) || (raw == NULL) || (microvolts == NULL) || (scaled == NULL))
  {
    return false;
  }
  *raw = fake_code();
  *microvolts = *scaled = 0;
  return true;
}

uint8_t Board_AdcSampleTime(void)
{
  return (uint8_t)0;
}

bool Board_AfeOn(void)
{
  return s.users != 0U;
}

void Board_AngleClock(uint32_t *kernel_hz, uint32_t *bitrate_hz)
{
  if (kernel_hz != NULL)
  {
    memset(kernel_hz, 0, sizeof *kernel_hz);
  }
  if (bitrate_hz != NULL)
  {
    memset(bitrate_hz, 0, sizeof *bitrate_hz);
  }
}

void Board_AngleHold(void)
{
}

bool Board_AngleInit(void)
{
  return false;
}

bool Board_AnglePollReg(uint8_t reg)
{
  (void)reg;
  return false;
}

uint8_t Board_AnglePollRegGet(void)
{
  return (uint8_t)0;
}

bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc)
{
  (void)reg;
  if (value != NULL)
  {
    memset(value, 0, sizeof *value);
  }
  if (crc != NULL)
  {
    memset(crc, 0, sizeof *crc);
  }
  return true;
}

bool Board_AngleReady(void)
{
  return false;
}

void Board_AngleResume(void)
{
}

void Board_AngleState(board_angle_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

bool Board_AngleWrite(uint8_t reg, uint8_t value)
{
  (void)reg;
  (void)value;
  return false;
}

void Board_BootStay(void)
{
}

void Board_CtrlClear(void)
{
}

const char * Board_CtrlRows(const uint16_t *ms, const float *setpoint, uint8_t n)
{
  (void)ms;
  (void)setpoint;
  (void)n;
  return NULL;
}

const char * Board_CtrlRun(bool on)
{
  (void)on;
  return NULL;
}

const char * Board_CtrlSlot(uint8_t slot, uint8_t kind, const float *params, uint8_t n)
{
  (void)slot;
  (void)kind;
  (void)params;
  (void)n;
  return NULL;
}

void Board_CtrlState(board_ctrl_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

const char * Board_CtrlWire(uint8_t measured, uint8_t command, uint16_t hz)
{
  (void)measured;
  (void)command;
  (void)hz;
  return NULL;
}

bool Board_DcBus(int32_t *raw, int32_t *millivolts)
{
  if (raw != NULL)
  {
    memset(raw, 0, sizeof *raw);
  }
  if (millivolts != NULL)
  {
    memset(millivolts, 0, sizeof *millivolts);
  }
  return true;
}

bool Board_DigitalChan(uint8_t index, board_dchan_t *info)
{
  (void)index;
  if (info != NULL)
  {
    memset(info, 0, sizeof *info);
  }
  return true;
}

uint8_t Board_DigitalCount(void)
{
  return (uint8_t)0;
}

bool Board_DigitalSampledChan(uint8_t slot, board_dchan_t *info)
{
  (void)slot;
  if (info != NULL)
  {
    memset(info, 0, sizeof *info);
  }
  return true;
}

uint8_t Board_DigitalSampledCount(void)
{
  return (uint8_t)0;
}

const drive_t * Board_Drive(void)
{
  static drive_t none;
  return &none;
}

void Board_DriveCycles(uint32_t *last, uint32_t *max)
{
  if (last != NULL)
  {
    memset(last, 0, sizeof *last);
  }
  if (max != NULL)
  {
    memset(max, 0, sizeof *max);
  }
}

void Board_DriveCyclesReset(void)
{
}

uint16_t Board_DriveExitTicks(void)
{
  return (uint16_t)0;
}

const char * Board_DriveModelParam(uint8_t id, int32_t value)
{
  (void)id;
  (void)value;
  return NULL;
}

void Board_DriveModelReset(void)
{
}

void Board_DriveMoments(drive_moments_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

void Board_DriveMomentsArm(uint32_t periods)
{
  (void)periods;
}

bool Board_DriveOwnsCompares(void)
{
  return false;
}

void Board_DriveParamsFromCal(void)
{
}

const char * Board_DriveSetMode(uint8_t mode)
{
  (void)mode;
  return NULL;
}

const char * Board_DriveSetSource(uint8_t source)
{
  (void)source;
  return NULL;
}

void Board_DriveSetTheta(int32_t microradians)
{
  (void)microradians;
}

const char * Board_DriveSetpoint(uint8_t id, int32_t value)
{
  (void)id;
  (void)value;
  return NULL;
}

void Board_DriveSetpointsGet(int32_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

float Board_DriveTs(void)
{
  return 0.0f;
}

void Board_DriveWindowTake(drive_window_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

uint32_t Board_HclkHz(void)
{
  return FAKE_SYSCLK_HZ / 2U;
}

board_identity_t Board_Identity(void)
{
  static const board_identity_t none;
  return none;
}

void Board_ImuClock(uint32_t *kernel_hz, uint32_t *bitrate_hz)
{
  if (kernel_hz != NULL)
  {
    memset(kernel_hz, 0, sizeof *kernel_hz);
  }
  if (bitrate_hz != NULL)
  {
    memset(bitrate_hz, 0, sizeof *bitrate_hz);
  }
}

uint8_t Board_ImuDrain(uint8_t limit)
{
  (void)limit;
  return (uint8_t)0;
}

void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us, bool *pending)
{
  if (report_id != NULL)
  {
    memset(report_id, 0, sizeof *report_id);
  }
  if (interval_us != NULL)
  {
    memset(interval_us, 0, sizeof *interval_us);
  }
  if (pending != NULL)
  {
    memset(pending, 0, sizeof *pending);
  }
}

void Board_ImuHold(void)
{
}

bool Board_ImuInit(void)
{
  return false;
}

uint8_t Board_ImuPinCheck(uint8_t pin)
{
  (void)pin;
  return (uint8_t)0;
}

bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select)
{
  (void)len;
  (void)select;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return true;
}

bool Board_ImuRead(uint8_t *channel, uint8_t *cargo, uint16_t cap, uint16_t *len)
{
  (void)cap;
  if (channel != NULL)
  {
    memset(channel, 0, sizeof *channel);
  }
  if (cargo != NULL)
  {
    memset(cargo, 0, sizeof *cargo);
  }
  if (len != NULL)
  {
    memset(len, 0, sizeof *len);
  }
  return true;
}

bool Board_ImuReady(void)
{
  return false;
}

void Board_ImuReset(void)
{
}

void Board_ImuResume(void)
{
}

bool Board_ImuSetFeature(uint8_t report_id, uint32_t interval_us)
{
  (void)report_id;
  (void)interval_us;
  return false;
}

void Board_ImuState(board_imu_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

bool Board_ImuWaitReady(uint32_t ms)
{
  (void)ms;
  return false;
}

uint16_t Board_ImuWakeTest(uint16_t ms)
{
  (void)ms;
  return (uint16_t)0;
}

bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len)
{
  (void)channel;
  (void)payload;
  (void)len;
  return false;
}

uint16_t Board_LogCount(void)
{
  return (uint16_t)0;
}

uint32_t Board_LogDropped(void)
{
  return (uint32_t)0;
}

void Board_LogEnable(uint8_t sources, uint32_t min_gap_cycles)
{
  (void)sources;
  (void)min_gap_cycles;
}

uint8_t Board_LogSources(void)
{
  return (uint8_t)0;
}

uint16_t Board_LogTake(board_sample_t *out, uint16_t max)
{
  (void)max;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return (uint16_t)0;
}

uint32_t Board_LogThinned(void)
{
  return (uint32_t)0;
}

bool Board_Ntc(int32_t *raw, int32_t *centidegc)
{
  if ((raw == NULL) || (centidegc == NULL))
  {
    return false;
  }
  *raw = fake_code();
  *centidegc = FAKE_ROOM_CENTI;
  return true;
}

bool Board_Part(uint8_t index, board_part_t *info)
{
  (void)index;
  if (info != NULL)
  {
    memset(info, 0, sizeof *info);
  }
  return true;
}

uint8_t Board_PartCount(void)
{
  return (uint8_t)0;
}

bool Board_Pe15(void)
{
  return false;
}

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w)
{
  if (u != NULL)
  {
    memset(u, 0, sizeof *u);
  }
  if (v != NULL)
  {
    memset(v, 0, sizeof *v);
  }
  if (w != NULL)
  {
    memset(w, 0, sizeof *w);
  }
  return true;
}

bool Board_PowerAcquire(board_rail_t rail, board_user_t user)
{
  if ((rail >= BOARD_RAIL_COUNT) || (user >= BOARD_USER_COUNT))
  {
    return false;
  }
  s.users |= (uint8_t)(1U << user);
  return true;
}

bool Board_PowerRelease(board_rail_t rail, board_user_t user)
{
  if ((rail >= BOARD_RAIL_COUNT) || (user >= BOARD_USER_COUNT))
  {
    return false;
  }
  s.users &= (uint8_t)~(1U << user);
  return true;
}

void Board_PowerReleaseAll(void)
{
  s.users = 0U;
}

bool Board_PowerState(board_rail_t rail, board_rail_state_t *out)
{
  if ((rail >= BOARD_RAIL_COUNT) || (out == NULL))
  {
    return false;
  }
  memset(out, 0, sizeof *out);
  out->on    = s.users != 0U;
  out->users = s.users;
  for (uint8_t bits = s.users; bits != 0U; bits &= (uint8_t)(bits - 1U))
  {
    out->count++;
  }
  return true;
}

bool Board_PwmClearFault(void)
{
  return false;
}

uint8_t Board_PwmDeadTimeFloor(void)
{
  return (uint8_t)0;
}

uint32_t Board_PwmDeadTimeNs(void)
{
  return (uint32_t)0;
}

int8_t Board_PwmDeadTimeSkew(void)
{
  return (int8_t)0;
}

void Board_PwmDisable(void)
{
  s.pwm_enabled = false;
}

void Board_PwmDutyRequested(uint32_t *ticks_q16)
{
  if (ticks_q16 != NULL)
  {
    memset(ticks_q16, 0, sizeof *ticks_q16);
  }
}

bool Board_PwmEnable(void)
{
  s.pwm_enabled = true;
  return true;
}

uint8_t Board_PwmGateShorts(void)
{
  return (uint8_t)0;
}

bool Board_PwmIsEnabled(void)
{
  return s.pwm_enabled;
}

uint32_t Board_PwmPeriodsLeft(void)
{
  return (uint32_t)0;
}

const char * Board_PwmSetAllCounted(const uint16_t *ticks, uint32_t periods)
{
  (void)ticks;
  (void)periods;
  return NULL;
}

const char * Board_PwmSetAllFine(const uint32_t *ticks_q16)
{
  (void)ticks_q16;
  return NULL;
}

const char * Board_PwmSetAlternate(const uint16_t *a, const uint16_t *b)
{
  (void)a;
  (void)b;
  return NULL;
}

bool Board_PwmSetBreakBypass(bool on)
{
  (void)on;
  return false;
}

const char * Board_PwmSetDeadTime(uint32_t ns)
{
  (void)ns;
  return NULL;
}

const char * Board_PwmSetDeadTimeSkew(int8_t counts)
{
  (void)counts;
  return NULL;
}

void Board_PwmState(board_pwm_state_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memset(out, 0, sizeof *out);
  out->ready   = true;
  out->enabled = s.pwm_enabled;
  out->period  = FAKE_PWM_PERIOD;
}

void Board_RequestConsoleMode(void)
{
}

uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity)
{
  (void)capacity;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return (uint8_t)0;
}

void Board_StoKeepaliveReset(void)
{
}

void Board_StoIdle(void)
{
}

bool Board_SyncIrq(void)
{
  return false;
}

void Board_StoState(board_sto_state_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memset(out, 0, sizeof *out);
  out->afe_on = s.users != 0U;
}

const char * Board_SyncArm(void)
{
  return NULL;
}

bool Board_SyncArmed(void)
{
  return false;
}

void Board_SyncDisarm(void)
{
}

bool Board_SyncSetTrigger(uint16_t ticks)
{
  (void)ticks;
  return false;
}

void Board_SyncState(board_sync_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

uint16_t Board_SyncTrigger(void)
{
  return (uint16_t)0;
}

uint32_t Board_SysClkHz(void)
{
  return FAKE_SYSCLK_HZ;
}

uint8_t Board_SysClkSource(void)
{
  return (uint8_t)0;
}

void Board_Uid(uint8_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

bool testrig_gate(uint32_t key, bool open)
{
  (void)key;
  (void)open;
  return false;
}

bool testrig_open(void)
{
  return false;
}

bool testrig_pin_allowed(char port, uint8_t pin)
{
  (void)port;
  (void)pin;
  return false;
}

bool testrig_pin_mode(char port, uint8_t pin, uint8_t mode, uint8_t pull)
{
  (void)port;
  (void)pin;
  (void)mode;
  (void)pull;
  return false;
}

bool testrig_pin_read(char port, uint8_t pin, bool *level)
{
  (void)port;
  (void)pin;
  if (level != NULL)
  {
    memset(level, 0, sizeof *level);
  }
  return true;
}

bool testrig_pin_write(char port, uint8_t pin, bool level)
{
  (void)port;
  (void)pin;
  (void)level;
  return false;
}

bool testrig_port_read(char port, uint16_t *value)
{
  (void)port;
  if (value != NULL)
  {
    memset(value, 0, sizeof *value);
  }
  return true;
}

bool testrig_port_write(char port, uint16_t mask, uint16_t value)
{
  (void)port;
  (void)mask;
  (void)value;
  return false;
}

/* What board_thermal.c reads besides: the MCU die at room, no angle sensor fitted, no
   current, the PWM idle at its period, no synchronous sample, the drive's derating as set. */
bool Board_McuDie(int32_t *raw, int32_t *centidegc)
{
  if ((raw == NULL) || (centidegc == NULL))
  {
    return false;
  }
  *raw = fake_code();
  *centidegc = FAKE_ROOM_CENTI;
  return true;
}

bool Board_AngleDie(int32_t *centidegc)
{
  (void)centidegc;
  return false;
}

float Board_PhaseAmps(uint8_t leg, int32_t centred)
{
  (void)leg;
  (void)centred;
  return 0.0f;
}

uint32_t Board_PwmPeriod(void)
{
  return FAKE_PWM_PERIOD;
}

uint16_t Board_PwmGetDuty(uint8_t phase)
{
  (void)phase;
  return 0U;
}

void Board_SyncLatest(board_sync_sample_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

bool Board_SyncMeanSquare(float *out)
{
  (void)out;
  return false;
}

void Board_DriveDerate(float factor)
{
  s.derate = factor;
  s.derated = true;
}

float Board_DriveDerating(void)
{
  return s.derated ? s.derate : 1.0f;
}

/* What board_daq.c reads besides: no timer, so no injected sequence; the H7's eight
   sampling times; no drivable pin. */
bool Board_AdcInjected(uint8_t index)
{
  (void)index;
  return false;
}

int32_t Board_AdcInjectedSlot(uint8_t index, const board_sync_sample_t *sample)
{
  (void)index;
  (void)sample;
  return 0;
}

bool Board_AdcSetSampleTime(uint8_t index)
{
  return index < 8U;
}

uint32_t Board_DigitalMask(void)
{
  return 0U;
}

/* Zero and span are board_adc.c's: they read the ADC. Zero takes the code as the offset;
   nothing the fake reads is a current or a voltage, so no span factor exists. */
bool Board_CalZero(uint8_t index, int32_t *measured)
{
  int32_t offset = 0;
  int32_t gain = 0;

  if ((measured == NULL) || !Board_CalChannel(index, &offset, &gain))
  {
    return false;
  }
  *measured = fake_code();
  return Board_CalSetChannel(index, *measured, gain);
}

bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured)
{
  (void)index;
  (void)reference;
  if (measured != NULL)
  {
    *measured = 0;
  }
  return false;
}
