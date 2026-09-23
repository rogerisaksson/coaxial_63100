/**
  ******************************************************************************
  * @file    board.h
  * @brief   Everything the comms stack needs from this board, and nothing more.
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

#include "boot.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Physical quantity a channel can be converted to, 0 meaning none is defined. */
#define BOARD_UNIT_NONE      0U
#define BOARD_UNIT_MILLIVOLT 1U
#define BOARD_UNIT_CENTIDEGC 2U
#define BOARD_UNIT_MILLIAMP  3U

typedef struct
{
  uint8_t     adc_index;      /**< 1, 2 or 3                          */
  uint8_t     channel;        /**< ADC channel number, decimal        */
  const char *pin;            /**< e.g. "PC3_C/PC2_C"                 */
  bool        differential;
  const char *signal;         /**< "" where the pin has no assignment */
  uint8_t     unit;           /**< BOARD_UNIT_*                       */
} board_chan_t;

const char *Board_Name(void);

/* ---- discrete I/O ------------------------------------------------------- */

/** Which way a pin's signal runs, seen from the MCU. */
#define BOARD_DIR_IN    0U
#define BOARD_DIR_OUT   1U
#define BOARD_DIR_INOUT 2U

/** One digital channel: a pin this board actually uses for something. */
typedef struct
{
  const char *pin;      /**< "PB2"                                  */
  uint8_t     dir;      /**< BOARD_DIR_*                            */
  const char *signal;   /**< what the pin carries on this board     */
  bool        usable;   /**< false where raw pin access is refused  */
} board_dchan_t;

uint8_t Board_DigitalCount(void);

/** The drivable pins - what `0x6D` kind 1 reports - as one word, bit i being
    slot i. */
uint32_t Board_DigitalMask(void);
uint8_t  Board_DigitalIoCount(void);
bool     Board_DigitalIoChan(uint8_t slot, board_dchan_t *info);

/** How many pins a DAQ record carries, and which. */
uint8_t  Board_DigitalSampledCount(void);
bool     Board_DigitalSampledChan(uint8_t slot, board_dchan_t *info);
bool    Board_DigitalChan(uint8_t index, board_dchan_t *info);

/** Whether a fixture may drive this pin at all. */
bool Board_PinUsable(char port, uint8_t pin);

/** What is fitted on the board, one entry per part. */
typedef struct
{
  const char *name;    /**< the part, as it is marked              */
  const char *what;    /**< what it does, one line                 */
  const char *where;   /**< the bus or pins it sits on             */
  const char *power;   /**< what must be on for it, or "" for none */
  uint8_t     state;   /**< BOARD_PART_* below                     */
} board_part_t;

#define BOARD_PART_UNKNOWN   0U  /**< nothing here can prove it either way */
#define BOARD_PART_READY     1U  /**< it answered                         */
#define BOARD_PART_UNPOWERED 2U  /**< what powers it is off               */
#define BOARD_PART_SILENT    3U  /**< powered, and did not answer         */

/** The IMU poll loop's shared record: what it saw, and what went wrong. */
typedef struct
{
  uint8_t  loop;        /**< BOARD_IMU_LOOP_*                            */
  uint8_t  error;       /**< BOARD_IMU_ERR_*, the last one seen          */
  uint8_t  last_fault;  /**< the last one that was not NONE, kept        */
  uint8_t  last_fault_id; /**< for FRAME, the report id that stopped it */
  uint32_t updates;     /**< rotation vectors written, monotonic         */
  uint32_t cargoes;     /**< cargoes taken off SPI2                      */
  uint32_t errors;      /**< reads that failed                           */
  bool     have;        /**< whether the quaternion below means anything */
  uint8_t  report_id;
  uint8_t  status;      /**< accuracy in bits 1:0                        */
  int16_t  i;
  int16_t  j;
  int16_t  k;
  int16_t  real;        /**< all four Q14 counts - the scale is the host's */

  /* THE THREE VECTORS, each on its own report and its own Q point - the
     scale stays the host's, as the quaternion's does. */
  bool     have_accel;
  bool     have_gyro;
  bool     have_mag;
  int16_t  accel[3];    /**< SH2 0x01, Q8, m/s^2                          */
  int16_t  gyro[3];     /**< SH2 0x02, Q9, rad/s                          */
  int16_t  mag[3];      /**< SH2 0x03, Q4, uT                             */
  uint8_t  accel_status;
  uint8_t  gyro_status;
  uint8_t  mag_status;
} board_imu_state_t;

#define BOARD_IMU_LOOP_OFF   0U  /**< AFE_ON is low; nothing to poll     */
#define BOARD_IMU_LOOP_INIT  1U  /**< powered, not yet brought up        */
#define BOARD_IMU_LOOP_RUN   2U  /**< polling                            */
#define BOARD_IMU_LOOP_HELD  3U  /**< stopped, so the host may configure */

#define BOARD_IMU_ERR_NONE   0U
#define BOARD_IMU_ERR_POWER  1U  /**< AFE_ON went away under it          */
#define BOARD_IMU_ERR_INIT   2U  /**< the part did not come up           */
#define BOARD_IMU_ERR_READ   3U  /**< a cargo read failed                */
#define BOARD_IMU_ERR_FRAME  4U  /**< a report id with no length         */
#define BOARD_IMU_ERR_NOWAKE 5U  /**< wrote without an H_INTN acknowledge */

/** Advance the IMU poll loop. */
void Board_ImuPoll(void);

/** Read the shared record. The only way a host sees the stream. */
void Board_ImuState(board_imu_state_t *out);

/** Stop the loop so the part can be configured, or start it again. */
void Board_ImuHold(void);
void Board_ImuResume(void);

/** The A1335's poll loop record, the same shape as the IMU's. */
typedef struct
{
  uint8_t  loop;        /**< BOARD_ANGLE_LOOP_*                          */
  uint8_t  error;       /**< BOARD_ANGLE_ERR_*, the last one seen        */
  uint32_t updates;     /**< readings written, monotonic                 */
  uint32_t errors;      /**< reads that failed                           */
  bool     have;        /**< whether `value` means anything              */
  uint8_t  reg;         /**< which register it came from                 */
  uint16_t value;       /**< the sixteen data bits, unscaled             */
  uint8_t  crc;         /**< the four CRC bits, unchecked - see the .c   */
} board_angle_state_t;

#define BOARD_ANGLE_LOOP_OFF  0U  /**< no supply, or not yet brought up  */
#define BOARD_ANGLE_LOOP_RUN  1U  /**< polling                           */
#define BOARD_ANGLE_LOOP_HELD 2U  /**< stopped, so the host may configure */

#define BOARD_ANGLE_ERR_NONE   0U
#define BOARD_ANGLE_ERR_POWER  1U  /**< AFE_ON went away under it        */
#define BOARD_ANGLE_ERR_INIT   2U  /**< SPI4 would not configure         */
#define BOARD_ANGLE_ERR_READ   3U  /**< the transfer failed              */
#define BOARD_ANGLE_ERR_SILENT 4U  /**< all ones: absent or unpowered    */

bool Board_AngleInit(void);
bool Board_AngleReady(void);
void Board_AngleClock(uint32_t *kernel_hz, uint32_t *bitrate_hz);

/** One 20-bit packet: the register's sixteen data bits and its four CRC
    bits, neither interpreted here. */
bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc);

/** The A1335's own die, centi-degrees C. */
bool Board_AngleDie(int32_t *centidegc);
bool Board_AngleWrite(uint8_t reg, uint8_t value);

/** Advance the angle sensor's poll loop. */
void Board_AnglePoll(void);
void Board_AngleState(board_angle_state_t *out);
void Board_AngleHold(void);
void Board_AngleResume(void);

/** Which register the loop reads. */
bool Board_AnglePollReg(uint8_t reg);
uint8_t Board_AnglePollRegGet(void);

/** Half bridges on this board, and compare registers on TIM1. */
#define BOARD_PWM_PHASES 3U

/** What the gate drivers are doing, for the command layer to report verbatim. */
typedef struct
{
  bool     ready;                    /**< TIM1 clocked and given a period  */
  bool     enabled;                  /**< master output enable is set      */
  bool     fault;                    /**< break latched - see PE15/BKIN    */
  uint32_t period;                   /**< ARR + 1, in timer ticks          */
  uint8_t  deadtime;                 /**< BDTR DTG, raw - not nanoseconds  */
  uint16_t duty[BOARD_PWM_PHASES];   /**< compare ticks, as last accepted  */
  bool     bypassed;                 /**< BDTR.BKE cleared - break ignored */
  uint8_t  pins;                     /** < PE8..PE13 as one IDR read: bit 0 UL, 1 UH, 2 VL, 3 VH, 4 WL, 5 WH */
  uint16_t at;                       /** < TIM1->CNT beside that read, so a host knows where in the period */
} board_pwm_state_t;

bool Board_PwmInit(void);

/** Dead time at runtime, in nanoseconds. */
const char *Board_PwmSetDeadTime(uint32_t ns);
uint32_t Board_PwmDeadTimeNs(void);

/** DTG counts the smallest dead time can be, at this timer clock. */
uint8_t Board_PwmDeadTimeFloor(void);

/** Trim for a bridge whose two transitions are not symmetric. */
const char *Board_PwmSetDeadTimeSkew(int8_t counts);
int8_t Board_PwmDeadTimeSkew(void);

/** What the board can see of the Safe Torque Off chain. */
typedef struct
{
  bool    afe_on;             /**< false makes both readings meaningless  */
  bool    pilot_ok;           /**< the Cinj channel answered              */
  int32_t pilot_raw;          /**< recovered pilot, raw code              */
  int32_t pilot_microvolts;
  bool    level_ok;           /**< the Clevel channel answered            */
  int32_t level_raw;          /**< integrator level - the margin left     */
  int32_t level_microvolts;
  bool    stopped;            /**< TIM1 break latched: nFAULT on PE15     */
  uint32_t keepalive;         /**< edges pumped since boot - the loop rate */
  uint32_t worst_gap;         /**< longest gap between edges, CYCCNT ticks */
} board_sto_state_t;

void Board_StoState(board_sto_state_t *out);

/** One edge into the STO charge pump. Call from the main loop, unguarded. */
void Board_StoKeepalive(void);

/** Forget the worst gap seen so far, so a run can be measured on its own. */
void Board_StoKeepaliveReset(void);


/** One simultaneous triple, latched by the injected end-of-sequence. */
typedef struct
{
  int16_t  phase[BOARD_PWM_PHASES];  /**< U, V, W, raw codes               */
  uint16_t at;                       /**< TIM1->CNT when it was latched    */
  uint32_t dcbus;                    /** < DC link, raw single-ended: rank 2 on ADC3 of the same sequence */
  uint32_t ntc;                      /** < the thermistor, rank 2 on ADC1: the thermal observer's thermometer
      while the drive holds the converters */
} board_sync_sample_t;

/** What the synced path is doing, for the command layer to report. */
typedef struct
{
  bool     ready;                    /**< timer and injected groups exist  */
  bool     armed;                    /**< triggering and latching          */
  uint32_t updates;                  /**< triples latched since arming     */
  uint32_t overruns;                 /**< sequences that arrived too soon  */
  uint16_t trigger;                  /**< CCR4 - the sample point, ticks   */
  board_sync_sample_t latest;
} board_sync_state_t;

/* ---- the acquisition task ----------------------------------------------- */

/** As many channels as the ADC table has rows. */
#define BOARD_DAQ_MAX_CHANNELS 10U

#define BOARD_DAQ_CLOCK_SOFTWARE 0U  /**< the main loop, as fast as it gets round */
#define BOARD_DAQ_CLOCK_TIM1     1U  /**< the injected group, one per PWM period  */

/** The sensor fields a record may append - SNAPSHOTS, never sums: a summed
    quaternion means nothing. */
#define BOARD_DAQ_SENSOR_ORIENTATION (1U << 0)  /**< i, j, k, real - Q14  */
#define BOARD_DAQ_SENSOR_ACCEL       (1U << 1)  /**< x, y, z, status - Q8 */
#define BOARD_DAQ_SENSOR_GYRO        (1U << 2)  /**< x, y, z, status - Q9 */
#define BOARD_DAQ_SENSOR_MAG         (1U << 3)  /**< x, y, z, status - Q4 */
#define BOARD_DAQ_SENSOR_SHAFT       (1U << 4)  /**< value, crc, reg, have */
#define BOARD_DAQ_MAX_SENSORS 5U

/** What a task is. Every field is the caller's; nothing is inferred. */
typedef struct
{
  uint16_t channels;     /** < bitmask over the ADC table's rows. */
  uint8_t  clock;        /**< BOARD_DAQ_CLOCK_*                          */
  uint8_t  sample_time;  /**< 0..7, the converter's own sampling window  */
  uint16_t decimate;     /**< keep one trigger in N; 1 keeps every one   */
  uint16_t accumulate;   /** < sum N samples per record; 1 sums nothing, 0 closes the record on
      interval_us instead */
  uint32_t records;      /**< stop after this many, or 0 to run on       */
  uint8_t  digital;      /**< append the digital pins to every record    */
  uint32_t interval_us;  /**< software clock: minimum gap between samples*/
  uint8_t  adapt;        /**< climb the ladder when the ring fills      */
  uint16_t sensors;      /** < BOARD_DAQ_SENSOR_* mask; software clock only - the poll records are
      the main loop's, and a TIM1-clocked record closes in ADC3's interrupt,
      which would read them torn */
} board_daq_config_t;

typedef struct
{
  bool     running;
  bool     done;         /**< a finite task reached its record count     */
  bool     lost_power;   /** < stopped because AFE_ON went off, and the buffers were emptied with it
      - invariant 9 */
  uint16_t stride;       /** < bytes per record: 4 (stamp) + 4 per channel + 1 per sampled pin + 8
      per sensor field + 2 (count). */
  uint8_t  fields;
  uint32_t available;    /**< whole records waiting to be taken          */
  uint32_t produced;
  uint32_t dropped;      /**< records the buffer had no room for         */
  /* THE BUFFER LEVEL, and it takes both numbers to be one: `available` alone
     is a count nobody can read as full or empty without knowing what the
     ring holds at THIS stride, which changes with the channel count. */
  uint32_t capacity;     /**< whole records the ring holds at `stride`   */
  uint32_t worst;        /**< the fullest it has been since the start    */
  uint8_t  rung;         /**< which rung of the ladder is running        */
  uint8_t  rungs;        /**< how many the host sent                     */
  uint32_t rung_changes; /**< how often it has climbed or fallen         */
  /* SWEEPS, not records: what the acquisition loop is actually managing
     underneath the decimation. */
  uint32_t triggers;
  board_daq_config_t config;
} board_daq_state_t;

/** The always-available accumulator: every trigger adds to it and a read
    takes it away. */
typedef struct
{
  int32_t  sum;
  uint32_t additions;                  /**< how many went into this one     */
  int32_t  lowest;                     /**< what the channel did in the     */
  int32_t  highest;                    /**< window, not inferred from a mean*/
} board_daq_slot_t;

typedef struct
{
  bool     fresh;                      /**< anything arrived since the last */
  uint32_t first;                      /**< Board_Cycles(), raw ticks       */
  uint32_t last;
  uint32_t digital;                    /**< pins at `last`                  */
  board_daq_slot_t slot[BOARD_DAQ_MAX_CHANNELS];
} board_daq_live_t;

/** Copy the accumulator out and reset it. */
void Board_DaqTakeLive(board_daq_live_t *out);

/** NULL when it took, and the reason in the board's own words when it did
    not. */
const char *Board_DaqConfigure(const board_daq_config_t *cfg);
/** Override the software clock's interval after configuring. */
void Board_DaqSetInterval(uint32_t interval_us);

const char *Board_DaqStart(void);
void Board_DaqStop(void);
void Board_DaqState(board_daq_state_t *out);

/** Load the anti-alias chain the host designed, or clear it.
    @return NULL, or the board's own words for what is wrong. */
const char *Board_DaqSetFilter(const void *sections, uint8_t count,
                               uint16_t decimate);

/** One rung of the ladder: a whole design and its boxcar. */
const char *Board_DaqSetRung(uint8_t rung, uint16_t boxcar,
                             const void *sections, uint8_t count,
                             uint16_t decimate);

/** A known tone in place of the converter, for proving the path. */
/** What the generator makes. */
#define BOARD_DAQ_TONE_SINE 0U
#define BOARD_DAQ_TONE_RAMP 1U

const char *Board_DaqSetTone(uint32_t hz, uint32_t rate_hz,
                             int32_t amplitude, int32_t offset,
                             uint8_t kind);

/** Advance the tone generator. */
void Board_DaqTonePoll(void);

/** Whether a tone is standing in for the converter. */
bool Board_DaqToneOn(void);

/** Triggers one record costs: the decimation, the accumulate AND the
    filter's own decimation. */
uint32_t Board_DaqTriggersPerRecord(void);

/** Whether the task asked for no rate and had one chosen for it. */
bool Board_DaqRateIsAuto(void);

/** Which channel field `n` of a record carries. */
bool Board_DaqField(uint8_t field, uint8_t *channel);

/** Advanced by the main loop for a software-clocked task. */
void Board_DaqPoll(void);

/** Fed from the injected end-of-sequence for a TIM1-clocked one. */
void Board_DaqOnInjected(const board_sync_sample_t *sample);

uint32_t Board_DaqAvailable(void);
uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records);


/** One measurement, whatever took it. */
#define BOARD_LOG_SOURCE_PHASES 0U   /**< v = U, V, W, TIM1->CNT at latch  */
#define BOARD_LOG_SOURCE_ANGLE  1U   /**< v = value, crc, register         */
#define BOARD_LOG_SOURCE_IMU    2U   /**< v = quaternion i, j, k, real     */
#define BOARD_LOG_SOURCE_DRIVE  3U   /** < v = id, iq in 10 mA, theta_hat as a turn in 65536, innovation in 0.1
    mrad */
#define BOARD_LOG_SOURCES       4U

/** 1024 x 16 B = 16 KB of DTCM, which is 20 ms of history at the injected
    group's 50 kHz - long enough to hold a burst while the host drains it
    fifteen records per round trip. */
#define BOARD_LOG_DEPTH 1024U

typedef struct
{
  uint32_t at;                /**< Board_Cycles() at capture, raw ticks    */
  uint8_t  source;
  uint8_t  seq;               /**< per source, so a dropped run is visible */
  int16_t  v[4];
} board_sample_t;

/** Arm the ring for a bitmask of sources, and empty it. */
void Board_LogEnable(uint8_t sources, uint32_t min_gap_cycles);
uint8_t Board_LogSources(void);

/** Called by the producers. Silently ignored for a source not armed. */
void Board_LogPush(uint8_t source, const int16_t *v, uint8_t n);

uint16_t Board_LogCount(void);
uint32_t Board_LogDropped(void);

/** Pushes refused by the rate limit, as opposed to lost to a full ring. */
uint32_t Board_LogThinned(void);

/** Copy out up to `max` oldest-first, and free their slots. */
uint16_t Board_LogTake(board_sample_t *out, uint16_t max);


/** A differential code as the converter gives it: offset binary, 32768 is 0
    V. */
int32_t Board_AdcDifferential(uint32_t raw);

/** Is there a timer to trigger from and an injected group to trigger? */
bool Board_SyncReady(void);

/** Start latching. False unless ready. Refuses the meter while armed. */
const char *Board_SyncArm(void);
void Board_SyncDisarm(void);
bool Board_SyncArmed(void);

/** The last triple, copied whole so no reader mixes two conversions. */
void Board_SyncLatest(board_sync_sample_t *out);

/** Mean of the squared phase current since the last call, A^2 a leg. */
bool Board_SyncMeanSquare(float *out);

/** Where in the PWM period the triple is taken, as CCR4 in timer ticks. */
bool Board_SyncSetTrigger(uint16_t ticks);
uint16_t Board_SyncTrigger(void);
void Board_SyncState(board_sync_state_t *out);

/** From the injected end-of-sequence callback, and from the overrun one. */
void Board_SyncOnInjected(const void *hadc);
void Board_SyncOverrun(void);


/** Has TIM1 been configured at all? False until MX_TIM1_Init exists. */
bool Board_PwmReady(void);

/** ARR + 1, or 0 when the timer is not configured. */
uint32_t Board_PwmPeriod(void);

/** Arm the outputs, at zero duty. False if not ready or a break is latched. */
bool Board_PwmEnable(void);

/** Drop every gate. The one call that works whatever else is true. */
void Board_PwmDisable(void);

bool Board_PwmIsEnabled(void);

/** Is the break latched? It is nFAULT arriving through TIM1_BKIN. */
bool Board_PwmFault(void);

/** Disconnect TIM1's break input, for bench work with the gate drivers
    unpowered. */
bool Board_PwmSetBreakBypass(bool on);
bool Board_PwmBreakBypassed(void);

/** The silent host's stage cleanup: MOE down, the break bypass back in
    force. */
void Board_PwmSessionDrop(void);

/** Clear the break latch. Does NOT re-arm - the caller must ask again. */
bool Board_PwmClearFault(void);

/** Which legs have their two gate pins joined, bit 0 = U, 1 = V, 2 = W. */
uint8_t Board_PwmGateShorts(void);

/** All three, or none: never a cycle built from two calls. */
const char *Board_PwmSetAll(const uint16_t *ticks);

/** The same triple held for exactly `periods` PWM periods, then zeroed by
    TIM1's update interrupt - 10 ms asked for is 500 periods, not the link's
    93-108 ms. */
const char *Board_PwmSetAllCounted(const uint16_t *ticks, uint32_t periods);

/** Periods left of a counted hold, 0 when free-running or expired. */
uint32_t Board_PwmPeriodsLeft(void);

/** Duty in ticks Q16.16, dithered so the MEAN is what was asked for. */
const char *Board_PwmSetAllFine(const uint32_t *ticks_q16);

/** Two compare triples, A one PWM period and B the next, swapped by the
    update interrupt at every overflow so each lands - preloaded - at the
    underflow and owns a whole period. */
const char *Board_PwmSetAlternate(const uint16_t *a, const uint16_t *b);
void Board_PwmDutyRequested(uint32_t *ticks_q16);
void Board_PwmDitherStep(void);

/** The drive's hold on the compares. */
void Board_PwmDriveOwn(bool on);
void Board_PwmSetNext(const uint16_t *ticks);

uint16_t Board_PwmGetDuty(uint8_t phase);

void Board_PwmState(board_pwm_state_t *out);


uint8_t Board_PartCount(void);
bool Board_Part(uint8_t index, board_part_t *info);

/** ADC sampling time, as an index 0..7 into the H7's eight, shortest first. */
bool    Board_AdcSetSampleTime(uint8_t index);
uint8_t Board_AdcSampleTime(void);

/** Is this channel one the injected group converts? Only those three can be
    clocked from TIM1; everything else has to come through the meter. */
bool    Board_AdcIsPhase(uint8_t index);

/** Whether the injected sequence converts this channel at all - the three
    phases and, on rank 2, the DC link and the NTC. */
bool    Board_AdcInjected(uint8_t index);

/** One channel's value out of a latched injected sample. */
int32_t Board_AdcInjectedSlot(uint8_t index,
                              const board_sync_sample_t *sample);
int32_t Board_AdcPhaseSlot(uint8_t index, const int16_t *phase);

uint8_t Board_AdcCount(void);
bool    Board_AdcChan(uint8_t index, board_chan_t *info);

/** Read one channel.
    @param  microvolts  Voltage at the ADC pin. Not the sensed quantity for the
    @param  scaled      Physical quantity in the channel's unit, 0 when the */
bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled);

/* False from any of these four means no reading was taken - a bad index, or
   a conversion that did not complete. */

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w);
bool Board_DcBus(int32_t *raw, int32_t *millivolts);
bool Board_Ntc(int32_t *raw, int32_t *centidegc);

/** The MCU die, centi-degrees C. */
bool Board_McuDie(int32_t *raw, int32_t *centidegc);

/** Amperes from a centred phase code - what Board_AdcDifferential returns. */
float Board_PhaseAmps(uint8_t leg, int32_t centred);

/** The affine form of the two conversions the drive needs at 50 kHz:
    quantity = (code - offset) * per_code, with the record's trim folded into
    the factor. */
void Board_PhaseScale(uint8_t leg, int32_t *offset_raw,
                      float *amps_per_code);
void Board_DcBusScale(int32_t *offset_raw, float *volts_per_code);

/* ---- calibration -------------------------------------------------------- */

/** Channels the record carries a correction for. */
/** Nodes in the thermal observer. */
/** TWENTY SINCE 2026-09-05, from ten: the laminate as seven patches that
    follow the copper, the hot swap as a node of its own, and the motor
    behind the board as three - the winding, the stator's iron and the
    rotor's bell. */
#define BOARD_THERMAL_NODES 20

/** The edges of the network, `thermal.c`'s table: each a K/W the record can
    overlay and the wire can name. */
#define BOARD_THERMAL_EDGES 30

/** The identification's scales on the wire - `thermal_ident.h`'s
    THERMAL_IDENT_RECORD, held to it by a static assert in board_thermal.c. */
#define BOARD_THERMAL_IDENT_SCALES 4

/** The margin floor's default, parts per million of every ceiling's span:
    the bench's 80 % - "keep to 80 % of the SOA when switching starts, with
    the thermal situation unknown". */
#define BOARD_SOA_MARGIN_FLOOR_PPM 800000UL

/** The indices, for a record or a host that has to name one. */
#define BOARD_THERMAL_DRIVER_U     0
#define BOARD_THERMAL_DRIVER_V     1
#define BOARD_THERMAL_DRIVER_W     2
#define BOARD_THERMAL_PHASE_U      3
#define BOARD_THERMAL_PHASE_V      4
#define BOARD_THERMAL_PHASE_W      5
#define BOARD_THERMAL_MCU          6
#define BOARD_THERMAL_REGULATORS   7
#define BOARD_THERMAL_AFE          8
#define BOARD_THERMAL_BOARD        9    /**< the laminate's centre patch */
#define BOARD_THERMAL_HOTSWAP      10
#define BOARD_THERMAL_PATCH_U      11
#define BOARD_THERMAL_PATCH_V      12
#define BOARD_THERMAL_PATCH_W      13
#define BOARD_THERMAL_PATCH_LEFT   14
#define BOARD_THERMAL_PATCH_BOTTOM 15
#define BOARD_THERMAL_PATCH_RIGHT  16
#define BOARD_THERMAL_WINDING      17
#define BOARD_THERMAL_STATOR       18
#define BOARD_THERMAL_ROTOR        19

/** One node's network entry in the record, milli-units; ZERO MEANS THE
    CORE'S DEFAULT for that field, so a record that never carried the network
    gets the derived one, and a default that improves reaches a board whose
    record has nothing to say about it. */
typedef struct
{
  uint32_t capacity_milli;     /**< J/K                                  */
  uint32_t to_ambient_milli;   /**< K/W to the air; a patch's own share  */
  uint32_t forced_milli;       /**< per sqrt(krpm)                       */
  uint32_t rth_milli;          /**< junction over node per watt          */
} board_cal_node_t;

/** An edge the record OPENS rather than defaults: the mount on a bench. */
#define BOARD_CAL_EDGE_OPEN 0xFFFFFFFFUL

#define BOARD_CAL_CHANNELS 10U

/** Which scalar Board_CalSetParam/GetParam addresses. */
#define BOARD_CAL_VREF_UV      0U  /**< ADC reference, microvolts           */
#define BOARD_CAL_SHUNT_UOHM   1U  /**< phase shunt, microhms               */
#define BOARD_CAL_AMP_GAIN_PPM 2U  /**< phase amplifier gain, ppm of 1 V/V  */
#define BOARD_CAL_BUS_R_TOP    3U  /**< DC link divider top, ohms           */
#define BOARD_CAL_BUS_R_BOTTOM 4U  /**< DC link divider bottom, ohms        */
#define BOARD_CAL_NTC_R25      5U  /**< thermistor at 25 C, ohms            */
#define BOARD_CAL_NTC_BETA_MK  6U  /**< B constant, milli-kelvin            */
#define BOARD_CAL_NTC_RFIXED   7U  /**< divider partner, ohms               */
#define BOARD_CAL_NTC_T25_CK   8U  /**< reference temperature, centikelvin  */
/* The two supply senses. */
#define BOARD_CAL_R5_R_TOP     9U  /**< +5 sense divider top, ohms          */
#define BOARD_CAL_R5_R_BOTTOM 10U  /**< +5 sense divider bottom, ohms       */
#define BOARD_CAL_VG_R_TOP    11U  /**< gate supply divider top, ohms       */
#define BOARD_CAL_VG_R_BOTTOM 12U  /**< gate supply divider bottom, ohms    */
#define BOARD_CAL_DEADTIME_NS 13U  /**< half-bridge dead time, nanoseconds  */
#define BOARD_CAL_DEADTIME_SKEW 14U /**< lead-lag trim, DTG counts         */
/* One past the last id above. */
/* CAL_VERSION 8: what the drive is told. */
#define BOARD_CAL_MOTOR_R_UOHM        15U  /**< phase resistance, microhms     */
#define BOARD_CAL_MOTOR_LD_NH         16U  /**< d inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LQ_NH         17U  /**< q inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LAMBDA_UVS    18U  /**< PM flux linkage, uV.s          */
#define BOARD_CAL_MOTOR_POLE_PAIRS    19U
#define BOARD_CAL_DRV_KP_MV_PER_A     20U  /**< current loop kp, mV/A          */
#define BOARD_CAL_DRV_KI_V_PER_AS     21U  /**< current loop ki, V/(A.s)       */
#define BOARD_CAL_DRV_L1_MILLI        22U  /**< rotor observer angle gain, 1e-3      */
#define BOARD_CAL_DRV_L2_MILLI        23U  /**< rotor observer speed gain, 1e-3/s    */
#define BOARD_CAL_DRV_INJ_MV          24U  /**< injection amplitude, mV; 0 off */
#define BOARD_CAL_DRV_INJ_PERIODS     25U  /**< PWM periods per half cycle     */
#define BOARD_CAL_DRV_INJ_PHASE_MRAD  26U  /**< injection axis off d, signed   */
#define BOARD_CAL_DRV_EPS_GAIN_UA_PER_RAD 27U /**< demodulated uA/rad, signed */
#define BOARD_CAL_DRV_I_MAX_MA        28U  /**< reference clamp, mA            */
#define BOARD_CAL_DRV_I_TRIP_MA       29U  /**< the stage drops past this, mA  */
#define BOARD_CAL_DRV_V_FRAC_PPM      30U  /**< of Vdc/sqrt3 the vector may use*/
#define BOARD_CAL_DRV_SIGN            31U  /**< 1, or -1 as 0xFFFFFFFF         */
#define BOARD_CAL_DRV_W_LO_MRAD_S     32U  /**< back-EMF blend starts, mrad/s  */
#define BOARD_CAL_DRV_W_HI_MRAD_S     33U  /**< injection off above, mrad/s    */
#define BOARD_CAL_DRV_DT_STEP_MA      34U  /**< dead-time table spacing, mA    */
#define BOARD_CAL_DRV_DT_MV           35U  /**< 35..42: the table, mV          */
#define BOARD_CAL_DRV_SIGMA_I_UA      43U  /**< measured current noise, uA rms */
#define BOARD_CAL_DRV_TRIGGER_TICKS   44U  /**< the sample point chosen; 0 none*/
/* CAL_VERSION 9. */
#define BOARD_CAL_LINK_RATE           45U  /** < the RS485 pair's rate (the wire and the host say `link_baud`) */
/* CAL_VERSION 12: the winding's envelope. */
#define BOARD_CAL_WINDING_K_MILLI     46U  /**< K/W to the air, milli         */
#define BOARD_CAL_WINDING_J_MILLI     47U  /**< J/K, milli                     */
#define BOARD_CAL_WINDING_LIMIT_CENTI 48U  /**< ceiling, centi-degrees; 0 off  */
#define BOARD_CAL_PARAM_COUNT 46U

/** One channel's correction, applied to the raw code before any scaling. */
typedef struct
{
  int32_t offset_raw;   /**< subtracted first; what a zero measures       */
  int32_t gain_ppm;     /**< then scaled by 1 + gain_ppm/1e6              */
} board_cal_chan_t;

/** The whole record, as it sits in flash. */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t channels;
  uint32_t vref_uv;
  uint32_t shunt_uohm;
  uint32_t amp_gain_ppm;
  uint32_t bus_r_top_ohm;
  uint32_t bus_r_bottom_ohm;
  uint32_t r5_r_top_ohm;
  uint32_t r5_r_bottom_ohm;
  uint32_t vg_r_top_ohm;
  uint32_t vg_r_bottom_ohm;

  /* The half-bridge dead time. */
  uint32_t deadtime_ns;

  /* Lead against lag, in DTG counts. */
  uint32_t deadtime_skew;
  uint32_t ntc_r25_ohm;
  uint32_t ntc_beta_mk;
  uint32_t ntc_rfixed_ohm;
  uint32_t ntc_t25_ck;
  board_cal_chan_t chan[BOARD_CAL_CHANNELS];

  /* The thermal envelope. */
  int32_t  soa_limit_centi[BOARD_THERMAL_NODES];
  uint32_t soa_throttle_ppm;   /**< where derating starts, parts per million */
  /* CAL_VERSION 10: how far ahead the throttle looks, milliseconds. */
  uint32_t soa_lookahead_ms;
  /* CAL_VERSION 11: which nodes the current clamp cannot cool, one bit per
     BOARD_THERMAL_NODES index, bit 0 the first. */
  uint32_t soa_undriven_mask;

  /* CAL_VERSION 8: the drive. */
  uint32_t motor_r_uohm;
  uint32_t motor_ld_nh;
  uint32_t motor_lq_nh;
  uint32_t motor_lambda_uvs;
  uint32_t motor_pole_pairs;
  uint32_t drv_kp_mv_per_a;
  uint32_t drv_ki_v_per_as;
  uint32_t drv_l1_milli;
  uint32_t drv_l2_milli;
  uint32_t drv_inj_mv;
  uint32_t drv_inj_periods;
  uint32_t drv_inj_phase_mrad;
  uint32_t drv_eps_gain_ua_per_rad;
  uint32_t drv_i_max_ma;
  uint32_t drv_i_trip_ma;
  uint32_t drv_v_frac_ppm;
  uint32_t drv_sign;
  uint32_t drv_w_lo_mrad_s;
  uint32_t drv_w_hi_mrad_s;
  uint32_t drv_dt_step_ma;
  uint32_t drv_dt_mv[8];
  uint32_t drv_sigma_i_ua;
  uint32_t drv_trigger_ticks;

  /* CAL_VERSION 9: the RS485 pair's baud, applied to USART2 and UART5 at
     init. */
  uint32_t link_baud;

  /* CAL_VERSION 12: the winding's envelope. */
  uint32_t winding_k_per_w_milli;
  uint32_t winding_j_per_k_milli;
  int32_t  winding_limit_centi;

  /* CAL_VERSION 13: THE NETWORK, so the board carries the model it runs and
     an identification running on the board has somewhere to put what it
     learns. */
  board_cal_node_t thermal_node[BOARD_THERMAL_NODES];
  uint32_t thermal_edge_milli[BOARD_THERMAL_EDGES];
  uint32_t thermal_to_ambient_milli;     /**< the whole face, K/W          */
  uint32_t thermal_capacity_milli;       /**< the whole laminate, J/K      */
  uint32_t thermal_rad_share_ppm;
  uint32_t thermal_ntc_sees_ppm;
  uint32_t thermal_ntc_tau_ms;
  uint32_t thermal_rad_board_stator_micro; /**< W/K at 300 K; 0 = bench    */
  uint32_t thermal_k_iron_milli;         /**< W per (krpm)^2              */

  /* CAL_VERSION 15: THE MARGIN FLOOR, parts per million of every ceiling's
     span over 25 C - what the envelope keeps while the identification has no
     evidence for its model, rising to the whole span as the evidence comes
     in (`thermal_ident_margin`). */
  uint32_t soa_margin_floor_ppm;

  uint16_t crc;
} board_cal_t;

/** The margin floor into the record's RAM copy, ppm of the span;
    `Board_CalSave` is what commits it. */
bool Board_CalSetMarginFloor(uint32_t ppm);

/** Overlay one node's, one edge's or the bulk's network entry in the
    record's RAM copy; `Board_CalSave` is what commits it. */
bool Board_CalSetThermalNode(uint8_t node, uint32_t capacity_milli,
                             uint32_t to_ambient_milli);
bool Board_CalSetThermalEdge(uint8_t edge, uint32_t k_per_w_milli);
bool Board_CalSetThermalBulk(uint32_t to_ambient_milli,
                             uint32_t capacity_milli);

/** Load the stored record, or fall back to the compiled-in defaults. */
void Board_CalInit(void);

/** The record in force now, stored or default. */
const board_cal_t *Board_Cal(void);

/** Whether flash holds a valid record, as against these being the defaults. */
bool Board_CalStored(void);

/** Replace the working record with the compiled-in defaults. */
void Board_CalDefaults(void);

/** Re-read flash, discarding uncommitted edits. */
bool Board_CalLoad(void);

/** Commit the working record to flash and read it back to prove it landed. */
bool Board_CalSave(void);

/* False from either of these means the id or the index does not exist, or
   the value would make a conversion divide by zero. */
bool Board_CalSetParam(uint8_t id, uint32_t value);
bool Board_CalGetParam(uint8_t id, uint32_t *value);

bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm);

/** One node's ceiling, centi-degrees C. */
bool Board_CalSetLimit(uint8_t node, int32_t limit_centi);

/** Where derating starts, parts per million of the budget. */
bool Board_CalSetThrottle(uint32_t ppm);

/** The winding's envelope: ceiling in centi-degrees (zero disables), K/W and
    J/K in milli. */
bool Board_CalSetWinding(int32_t limit_centi, uint32_t k_per_w_milli,
                         uint32_t j_per_k_milli);
bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm);

/** Correct one raw code: offset first, then gain.
    @return The code unchanged for an index the record does not cover, because */
int32_t Board_CalApply(uint8_t index, int32_t raw);

/** Measure a channel now and store the reading as its offset.
    @param  measured  The code that was stored, before correction. */
bool Board_CalZero(uint8_t index, int32_t *measured);

/** Measure a channel now and trim its gain so the reading equals */
bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured);

/* ---- IMU ---------------------------------------------------------------- */

/** Bring SPI2 to what the BNO08X needs and take PB12 as a GPIO chip
    @return False if the peripheral would not re-initialise. */
bool Board_ImuInit(void);

/** SPI2 and the IMU's control pins, without resetting the part. */
bool Board_ImuBusInit(void);

/** Pulse NRSTN with BOOTN held high, then wait out the part's own */
void Board_ImuReset(void);

/** Whether Board_ImuInit() succeeded. Every call below fails until it has. */
bool Board_ImuReady(void);

/** Read one SHTP cargo, if the part has one waiting.
    @param  channel  The SHTP channel it arrived on.
    @param  cargo    The cargo WITHOUT its four-byte header.
    @param  len      Cargo bytes, 0 when the part had nothing to say.
    @return False on a transfer error or a header that contradicts itself. */
bool Board_ImuRead(uint8_t *channel, uint8_t *cargo, uint16_t cap,
                   uint16_t *len);

/** Frame a payload onto an SHTP channel and clock it out.
    @return False on a bad channel, a payload that will not fit, or a */
bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len);

/** Wait up to @p ms for the part to say it has something. */
bool Board_ImuWaitReady(uint32_t ms);

/** Ask the part to report `report_id` every `interval_us`, and REMEMBER it. */
bool Board_ImuSetFeature(uint8_t report_id, uint32_t interval_us);

/** What was last asked for. Interval zero means nothing has been. */
void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us,
                           bool *pending);

/** Collect and discard whatever the part has queued.
    @return How many cargoes were drained. */
uint8_t Board_ImuDrain(uint8_t limit);

/** SPI2's four pins on port B, chip select first: PB12 H_CSN, PB13 SCK, PB14
    MISO, PB15 MOSI - the rows board_io.c lists, by number for the pin check. */
#define BOARD_IMU_SPI_PIN_FIRST 12U
#define BOARD_IMU_SPI_PIN_COUNT 4U

/** Drive and release GPIOB pin `pin`, reporting what the pin then read. */
uint8_t Board_ImuPinCheck(uint8_t pin);

/** Assert PS0/WAKE on a drained part and time H_INTN's answer. */
uint16_t Board_ImuWakeTest(uint16_t ms);

/** Clock four bytes out and hand back exactly what came in.
    @return False only if the transfer itself failed. */
/** Clock `len` bytes and keep what comes back, with no framing. */
bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select);

/** The SPI2 kernel clock and the bit rate Board_ImuInit settled on, so the
    bench can see the number rather than infer it from a silent part. */
void Board_ImuClock(uint32_t *kernel_hz, uint32_t *bitrate_hz);

bool Board_AfeOn(void);
void Board_SetAfeOn(bool on);
bool Board_Pe15(void);

/** UART5's 120 ohm termination, PE14: closed by the last node on the
    segment, open on the rest (docs/BOOT.md). */
void Board_SetTermination(bool closed);

uint32_t Board_SysClkHz(void);
uint32_t Board_HclkHz(void);

/** What the converters are actually clocked at, after the prescaler. */
uint32_t Board_AdcClockHz(void);
uint8_t  Board_SysClkSource(void);   /**< 0 HSI, 1 CSI, 2 HSE, 3 PLL1, 4 other */
uint32_t Board_Cycles(void);

/** True when SYSCLK comes from the HSE crystal, directly or through PLL1. */
bool Board_SysClkOnCrystal(void);

/** Enable the cycle counter the comms stack uses as its timebase. */
void Board_TimebaseInit(void);

/** Per-channel result of a burst. */
typedef struct
{
  uint8_t  index;
  int32_t  mean_milliraw;
  int32_t  min_raw;
  int32_t  max_raw;
  uint32_t sd_milliraw;
} board_burst_t;

/** Longest burst the firmware will accept, so a request cannot outlive the
    master's patience or wedge the link. */
#define BOARD_BURST_MAX_US 5000000UL

/** Most passes one burst may make. */
#define BOARD_BURST_MAX_SAMPLES 10000U

/** Channels one burst can cover, and the size every caller's `out` array
    must have. */
#define BOARD_BURST_MAX_CHAN 16U

/** The most samples one burst takes: the reply is a summary, so this bounds
    the time the link waits, not a buffer. */
#define BOARD_ADC_BURST_MAX 1000U

/** Sample a set of channels repeatedly and return per-channel statistics.
    @param  mask         Bit i selects channel i of the channel table.
    @param  samples      1..10000 passes over the selected set.
    @param  interval_us  Requested spacing between passes; 0 means as fast as
    @param  out          At least Board_AdcCount() entries.
    @param  count        Channels actually measured, in ascending index order.
    @param  elapsed_us   Wall time the burst took, so the host can see the rate
    @return False if the mask is empty, the count is out of range, the burst */
bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us,
                    board_burst_t *out, uint8_t *count, uint32_t *elapsed_us);

/** Sample one ADC back to back and return basic noise statistics.
    @param  adc_index  1..3; the differential phase channel on that ADC.
    @param  samples    1..1000.
    @return False if either argument is out of range, or a conversion failed. */
bool Board_AdcNoise(uint8_t adc_index, uint16_t samples,
                    int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw,
                    uint32_t *span_raw, uint32_t *stddev_uv);

/* ---- self test ---------------------------------------------------------- */

/** One result from the board's self test. */
#define BOARD_CHECK_PASS 0U
#define BOARD_CHECK_FAIL 1U
#define BOARD_CHECK_INFO 2U

#define BOARD_SELFTEST_MAX 16U

typedef struct
{
  const char *name;
  uint8_t     status;
  int32_t     value;   /**< meaning is per check; 0 where there is none */
} board_check_t;

/** Run every self check and fill @p out.
    @return Number of checks written, never more than @p capacity. */
uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity);

/** Leave the binary link and resume the ASCII console, once the reply is out. */
void Board_RequestConsoleMode(void);


/** What the thermal observer knows: one measurement, the rest estimates. */
typedef struct
{
  bool    ntc_measured;                        /**< the thermistor answered              */
  int32_t ntc_centidegc;                       /**< MEASURED, valid only above           */
  bool    afe_measured;                        /**< the A1335's die answered             */
  int32_t afe_centidegc;                       /**< MEASURED, valid only above           */
  bool    mcu_measured;                        /**< the MCU's die answered               */
  int32_t mcu_centidegc;                       /**< MEASURED, valid only above           */
  uint32_t seen_ms_ago;                        /**< age of the whole sample              */
  uint32_t steps;                              /**< model integrations since boot        */
  int32_t node_centidegc[BOARD_THERMAL_NODES]; /**< ESTIMATED                            */
  int32_t ambient_centidegc;                   /**< ESTIMATED - there is no sensor       */
  int32_t expected_ntc_centidegc;              /**< the model's own NTC, for the error   */
  uint32_t seconds;                            /**< how long it has run                  */
  bool    settled;                             /**< the anchoring has converged          */
  /** MINOR 13: each leg's FET junction over its node, centi-K - half the
      node's watts through R_th,JC - and the rotor speed the air paths were
      evaluated at. */
  int32_t junction_over_centi[3];
  int32_t speed_rpm;
} board_thermal_t;

/** The online identification beside the observer (`thermal_ident.h`): what
    it believes the network's scales are, how sure, and what the envelope
    keeps in hand for that. */
typedef struct
{
  uint8_t  state;                   /**< thermal_ident_state_t - a word    */
  uint8_t  online_mask;             /**< bit k: scale k is moved by samples */
  float    scale[BOARD_THERMAL_IDENT_SCALES];
  float    sigma[BOARD_THERMAL_IDENT_SCALES];
  float    innovation_k;            /**< filtered prediction error, kelvin  */
  float    margin;                  /**< the envelope's factor now, floor..1*/
  uint32_t updates;                 /**< samples that moved the scales      */
  /** MINOR 15: the room as identified beside the scales, degrees C, and how
      sure - the board has no ambient sensor; this is what the observer's
      `ambient` is set from. */
  float    ambient_c;
  float    ambient_sigma_k;
  /** MINOR 16: the floor the margin rises from, the record's. */
  float    margin_floor;
  /** MINOR 17: the trip cap as it stands - THERMAL_TRIP_MARGIN at a trip,
      recovering at THERMAL_TRIP_RECOVER_PER_S - or one with no trip in hand. */
  float    trip_cap;
} board_thermal_ident_t;

bool Board_ThermalIdent(board_thermal_ident_t *out);

/** Forget what was identified: scales to one, the room where the observer
    has it, UNCERTAIN, the margin at the floor. */
bool Board_ThermalIdentReset(void);

/** The margin floor, a fraction of every ceiling's span, through the record
    - `Board_CalSave` persists it. */
bool Board_ThermalSetMarginFloor(float floor);

/** One edge of the network: which two nodes, and the K/W across it now. */
bool Board_ThermalEdge(uint8_t edge, uint8_t *a, uint8_t *b, float *k_per_w);

/** Change one edge's K/W, in the observer and in the record's RAM copy;
    negative opens it. */
bool Board_ThermalSetEdge(uint8_t edge, float k_per_w);

/** One node's network entry as the observer runs it. */
bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient,
                          float *area_share, float *rth_die, float *forced);

/** The thermal budget: how much is spent and how long is left. */
typedef struct
{
  uint8_t  used[BOARD_THERMAL_NODES];
  uint8_t  worst;
  uint8_t  worst_node;
  int32_t  millis_to_limit;  /**< -1 when it is not heading for a limit */
  bool     throttling;
  bool     tripped;
  uint32_t trips;            /**< how many times it has stopped the stage */
  /** What the current clamp is being multiplied by right now, 1 to 0. */
  float    derate;
  /** Joules each node can still absorb before its ceiling. */
  float    soak_j[BOARD_THERMAL_NODES];
  /** What the compares actually hold, as a fraction of the period. */
  float    duty[BOARD_PWM_PHASES];
  /** MINOR 12: THE WINDING, the one node that is not on the board. */
  float    winding_c;
  uint8_t  winding_used;
  float    winding_derate;
} board_budget_t;

bool Board_ThermalBudget(board_budget_t *out);

/** Set one node's ceiling, degrees C. Zero disables that node's limit. */
bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at);

/** The winding's ceiling, degrees C (zero disables), and its K/W and J/K:
    written to the record and taken up by the observer at once. */
bool Board_ThermalSetWinding(float limit_c, float k_per_w, float j_per_k);

/** Scale the drive's current clamp, 1.0 down to 0.0. */
void Board_DriveDerate(float factor);

/** What that factor is now. */
float Board_DriveDerating(void);

void Board_ThermalInit(void);
void Board_ThermalPoll(void);
bool Board_ThermalState(board_thermal_t *out);
bool Board_ThermalSetNode(uint8_t node, float to_board, float capacity);
bool Board_ThermalSetBoard(float to_ambient, float capacity);

/** How often the thermal observer borrows the AFE rail for an NTC sample.
    @param  every_ms   period between samples; 0 stops sampling entirely
    @param  settle_ms  how long the reference is given before the read */
bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms);

/** What the sampling is set to now. */
void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms);

/* ---- the bootloader's side (board_boot.c) ------------------------------- */

/** This board's type as the bootloader names it. */
#define BOARD_BOOT_TYPE  BOOT_TYPE_COAXIAL_63100

/** Who this node is: what a bootloader left in the handover slot, or the
    defaults where none did (docs/BOOT.md). */
typedef struct
{
  uint8_t type;       /**< BOARD_BOOT_TYPE                              */
  uint8_t unit;       /**< the unit id answered to                      */
  uint8_t position;   /**< down the limb; 0 where nobody assigned one   */
  uint8_t flags;      /**< assign's flags; 0 where nobody assigned them */
  bool    assigned;   /**< a bootloader left these, or they are defaults */
} board_identity_t;

/** Apply what the bootloader left: the unit id and the termination. */
void Board_BootInit(void);
board_identity_t Board_Identity(void);

/** The MCU's unique id, twelve bytes little-endian off UID_BASE. */
void Board_Uid(uint8_t *out);

/** Back to the bootloader: the reset waits for the reply to leave the wire,
    and the bootloader finds STAY in the slot. */
void Board_BootStay(void);
void Board_BootPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
