/** fake_board.c - The board API on the host, every call answered neutrally. */

/* No part fitted, nothing read, every out-parameter zeroed, every setting taken (a NULL
   refusal): so the offline suites drive comms/ through its own wire
   (tools/cores/fakeboard.py). Generated once from the prototypes; what a check needs is
   answered by hand here. */
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
#include "board_power.h"
#include "testrig.h"

#include <stddef.h>
#include <string.h>

bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us, board_burst_t *out, uint8_t *count, uint32_t *elapsed_us)
{
  (void)mask;
  (void)samples;
  (void)interval_us;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  if (count != NULL)
  {
    memset(count, 0, sizeof *count);
  }
  if (elapsed_us != NULL)
  {
    memset(elapsed_us, 0, sizeof *elapsed_us);
  }
  return true;
}

bool Board_AdcChan(uint8_t index, board_chan_t *info)
{
  (void)index;
  if (info != NULL)
  {
    memset(info, 0, sizeof *info);
  }
  return true;
}

uint32_t Board_AdcClockHz(void)
{
  return (uint32_t)0;
}

uint8_t Board_AdcCount(void)
{
  return (uint8_t)0;
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
  (void)index;
  if (raw != NULL)
  {
    memset(raw, 0, sizeof *raw);
  }
  if (microvolts != NULL)
  {
    memset(microvolts, 0, sizeof *microvolts);
  }
  if (scaled != NULL)
  {
    memset(scaled, 0, sizeof *scaled);
  }
  return true;
}

uint8_t Board_AdcSampleTime(void)
{
  return (uint8_t)0;
}

bool Board_AfeOn(void)
{
  return false;
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

const board_cal_t * Board_Cal(void)
{
  static board_cal_t none;
  return &none;
}

bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm)
{
  (void)index;
  if (offset_raw != NULL)
  {
    memset(offset_raw, 0, sizeof *offset_raw);
  }
  if (gain_ppm != NULL)
  {
    memset(gain_ppm, 0, sizeof *gain_ppm);
  }
  return true;
}

void Board_CalDefaults(void)
{
}

bool Board_CalGetParam(uint8_t id, uint32_t *value)
{
  (void)id;
  if (value != NULL)
  {
    memset(value, 0, sizeof *value);
  }
  return true;
}

bool Board_CalLoad(void)
{
  return false;
}

bool Board_CalSave(void)
{
  return false;
}

bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm)
{
  (void)index;
  (void)offset_raw;
  (void)gain_ppm;
  return false;
}

bool Board_CalSetParam(uint8_t id, uint32_t value)
{
  (void)id;
  (void)value;
  return false;
}

bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured)
{
  (void)index;
  (void)reference;
  if (measured != NULL)
  {
    memset(measured, 0, sizeof *measured);
  }
  return true;
}

bool Board_CalStored(void)
{
  return false;
}

bool Board_CalZero(uint8_t index, int32_t *measured)
{
  (void)index;
  if (measured != NULL)
  {
    memset(measured, 0, sizeof *measured);
  }
  return true;
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

uint32_t Board_Cycles(void)
{
  return (uint32_t)0;
}

uint32_t Board_DaqAvailable(void)
{
  return (uint32_t)0;
}

const char * Board_DaqConfigure(const board_daq_config_t *cfg)
{
  (void)cfg;
  return NULL;
}

bool Board_DaqField(uint8_t field, uint8_t *channel)
{
  (void)field;
  if (channel != NULL)
  {
    memset(channel, 0, sizeof *channel);
  }
  return true;
}

bool Board_DaqRateIsAuto(void)
{
  return false;
}

const char * Board_DaqSetFilter(const void *sections, uint8_t count, uint16_t decimate)
{
  (void)sections;
  (void)count;
  (void)decimate;
  return NULL;
}

void Board_DaqSetInterval(uint32_t interval_us)
{
  (void)interval_us;
}

const char * Board_DaqSetRung(uint8_t rung, uint16_t boxcar, const void *sections, uint8_t count, uint16_t decimate)
{
  (void)rung;
  (void)boxcar;
  (void)sections;
  (void)count;
  (void)decimate;
  return NULL;
}

const char * Board_DaqSetTone(uint32_t hz, uint32_t rate_hz, int32_t amplitude, int32_t offset, uint8_t kind)
{
  (void)hz;
  (void)rate_hz;
  (void)amplitude;
  (void)offset;
  (void)kind;
  return NULL;
}

const char * Board_DaqStart(void)
{
  return NULL;
}

void Board_DaqState(board_daq_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

void Board_DaqStop(void)
{
}

uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records)
{
  (void)max_records;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return (uint16_t)0;
}

void Board_DaqTakeLive(board_daq_live_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
}

uint32_t Board_DaqTriggersPerRecord(void)
{
  return (uint32_t)0;
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
  return (uint32_t)0;
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
  if (raw != NULL)
  {
    memset(raw, 0, sizeof *raw);
  }
  if (centidegc != NULL)
  {
    memset(centidegc, 0, sizeof *centidegc);
  }
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
  (void)rail;
  (void)user;
  return false;
}

bool Board_PowerRelease(board_rail_t rail, board_user_t user)
{
  (void)rail;
  (void)user;
  return false;
}

void Board_PowerReleaseAll(void)
{
}

bool Board_PowerState(board_rail_t rail, board_rail_state_t *out)
{
  (void)rail;
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
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
  return false;
}

uint8_t Board_PwmGateShorts(void)
{
  return (uint8_t)0;
}

bool Board_PwmIsEnabled(void)
{
  return false;
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
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
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

void Board_StoState(board_sto_state_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
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
  return (uint32_t)0;
}

uint8_t Board_SysClkSource(void)
{
  return (uint8_t)0;
}

bool Board_ThermalBudget(board_budget_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return true;
}

bool Board_ThermalEdge(uint8_t edge, uint8_t *a, uint8_t *b, float *k_per_w)
{
  (void)edge;
  if (a != NULL)
  {
    memset(a, 0, sizeof *a);
  }
  if (b != NULL)
  {
    memset(b, 0, sizeof *b);
  }
  if (k_per_w != NULL)
  {
    memset(k_per_w, 0, sizeof *k_per_w);
  }
  return true;
}

bool Board_ThermalIdent(board_thermal_ident_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return true;
}

bool Board_ThermalIdentReset(void)
{
  return false;
}

bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient, float *area_share, float *rth_die, float *forced)
{
  (void)node;
  if (capacity != NULL)
  {
    memset(capacity, 0, sizeof *capacity);
  }
  if (to_ambient != NULL)
  {
    memset(to_ambient, 0, sizeof *to_ambient);
  }
  if (area_share != NULL)
  {
    memset(area_share, 0, sizeof *area_share);
  }
  if (rth_die != NULL)
  {
    memset(rth_die, 0, sizeof *rth_die);
  }
  if (forced != NULL)
  {
    memset(forced, 0, sizeof *forced);
  }
  return true;
}

void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms)
{
  if (every_ms != NULL)
  {
    memset(every_ms, 0, sizeof *every_ms);
  }
  if (settle_ms != NULL)
  {
    memset(settle_ms, 0, sizeof *settle_ms);
  }
}

bool Board_ThermalSetBoard(float to_ambient, float capacity)
{
  (void)to_ambient;
  (void)capacity;
  return false;
}

bool Board_ThermalSetEdge(uint8_t edge, float k_per_w)
{
  (void)edge;
  (void)k_per_w;
  return false;
}

bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at)
{
  (void)node;
  (void)limit_c;
  (void)throttle_at;
  return false;
}

bool Board_ThermalSetMarginFloor(float floor)
{
  (void)floor;
  return false;
}

bool Board_ThermalSetNode(uint8_t node, float to_board, float capacity)
{
  (void)node;
  (void)to_board;
  (void)capacity;
  return false;
}

bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms)
{
  (void)every_ms;
  (void)settle_ms;
  return false;
}

bool Board_ThermalSetWinding(float limit_c, float k_per_w, float j_per_k)
{
  (void)limit_c;
  (void)k_per_w;
  (void)j_per_k;
  return false;
}

bool Board_ThermalState(board_thermal_t *out)
{
  if (out != NULL)
  {
    memset(out, 0, sizeof *out);
  }
  return true;
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
