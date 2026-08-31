/**
  ******************************************************************************
  * @file    board.h
  * @brief   Everything the comms stack needs from this board, and nothing more.
  *
  * The ADC helpers in main.c are static and stay static: they are the reporting
  * code's own business. This is the whole surface the command handlers may use.
  * The dependency runs one way - the comms stack asks, main.c answers.
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

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

/**
  * @brief One digital channel: a pin this board actually uses for something.
  *
  * `usable` is false for the pins raw access is refused on - the link and the
  * debug port. They are listed rather than hidden, because "PB10 is USART3_TX
  * and you may not drive it" is the answer a fixture needs; leaving them out
  * only means someone asks again with a pin write.
  */
typedef struct
{
  const char *pin;      /**< "PB2"                                  */
  uint8_t     dir;      /**< BOARD_DIR_*                            */
  const char *signal;   /**< what the pin carries on this board     */
  bool        usable;   /**< false where raw pin access is refused  */
} board_dchan_t;

uint8_t Board_DigitalCount(void);

/** The drivable pins - what `0x6D` kind 1 reports - as one word, bit i
    being slot i. Sampled by the acquisition task alongside the converters;
    the layout names the bits, so nothing above has to count them. */
uint32_t Board_DigitalMask(void);
uint8_t  Board_DigitalIoCount(void);
bool     Board_DigitalIoChan(uint8_t slot, board_dchan_t *info);
bool    Board_DigitalChan(uint8_t index, board_dchan_t *info);

/**
  * @brief  Whether a fixture may drive this pin at all.
  *
  * The reserved list is the pin table, not a second list beside it: testrig.c
  * used to keep its own, and two lists of what PB10 is are one edit away from
  * disagreeing.
  */
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

/** The IMU poll loop's shared record: what it saw, and what went wrong.
  *
  * Written only by Board_ImuPoll and read only by the command layer. There is
  * one writer and one reader and both run from the same main loop, so no
  * lock: what would need one is a second writer, and adding one is what this
  * comment exists to argue against.
  */
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

/** Advance the IMU poll loop. Cheap when there is nothing waiting: one GPIO
  * read. Call it from the main loop, and not while the RTU receiver is
  * mid-frame - a 276-byte cargo at 1.48 MHz is 1.5 ms, which reads as a t3.5
  * gap and splits the frame in two. */
void Board_ImuPoll(void);

/** Read the shared record. The only way a host sees the stream. */
void Board_ImuState(board_imu_state_t *out);

/** Stop the loop so the part can be configured, or start it again.
  * Configuring under a running loop is two masters on one SPI bus. */
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
  * bits, neither interpreted here. */
bool Board_AngleRead(uint8_t reg, uint16_t *value, uint8_t *crc);

/** The A1335's own die, centi-degrees C. Needs AFE_ON like the part
  * itself does. Measures the die, not the board - which is the point. */
bool Board_AngleDie(int32_t *centidegc);
bool Board_AngleWrite(uint8_t reg, uint8_t value);

/** Advance the angle sensor's poll loop. One packet when it runs, which is
  * 13 us at the bitrate this picks - short enough not to need staging the
  * way the IMU's 276-byte cargo did. */
void Board_AnglePoll(void);
void Board_AngleState(board_angle_state_t *out);
void Board_AngleHold(void);
void Board_AngleResume(void);

/** Which register the loop reads. Settable because the register map came
  * from a reference implementation, not from the datasheet in this tree. */
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
  uint8_t  pins;                     /**< PE8..PE13 as one IDR read: bit 0
                                          UL, 1 UH, 2 VL, 3 VH, 4 WL, 5 WH */
  uint16_t at;                       /**< TIM1->CNT beside that read, so a
                                          host knows where in the period    */
} board_pwm_state_t;

bool Board_PwmInit(void);

/** Dead time at runtime, in nanoseconds. Floored at 20 ns, which is a floor
  * and not a default: the 2EDL8034 has no interlock, so this is the only
  * thing between the two FETs of a leg. Refuses with its reason. */
const char *Board_PwmSetDeadTime(uint32_t ns);
uint32_t Board_PwmDeadTimeNs(void);

/** DTG counts the smallest dead time can be, at this timer clock. */
uint8_t Board_PwmDeadTimeFloor(void);

/** Trim for a bridge whose two transitions are not symmetric. Positive
  * lengthens the dead time on the transition the counter reaches counting
  * up and shortens the other by the same, so the pair still averages what
  * was asked for. Neither half may go under the floor. NOT MEASURED. */
const char *Board_PwmSetDeadTimeSkew(int8_t counts);
int8_t Board_PwmDeadTimeSkew(void);

/** What the board can see of the Safe Torque Off chain. Reports; judges
    nothing - deciding "released" from a Clevel threshold is a test
    executive's job, not this board's. See board_sto.c. */
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
  uint32_t dcbus;                    /**< DC link, raw single-ended: rank 2
                                          on ADC3 of the same sequence      */
  uint32_t ntc;                      /**< the thermistor, rank 2 on ADC1:
                                          the observer's thermometer while
                                          the drive holds the converters   */
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

/** What a task is. Every field is the caller's; nothing is inferred. */
typedef struct
{
  uint16_t channels;     /**< bitmask over the ADC table's rows. 16 bits
                              because the ninth channel did not fit in 8 */
  uint8_t  clock;        /**< BOARD_DAQ_CLOCK_*                          */
  uint8_t  sample_time;  /**< 0..7, the converter's own sampling window  */
  uint16_t decimate;     /**< keep one trigger in N; 1 keeps every one   */
  uint16_t accumulate;   /**< sum N samples per record; 1 sums nothing   */
  uint32_t records;      /**< stop after this many, or 0 to run on       */
  uint8_t  digital;      /**< append the digital pins to every record    */
  uint32_t interval_us;  /**< software clock: minimum gap between samples*/
} board_daq_config_t;

typedef struct
{
  bool     running;
  bool     done;         /**< a finite task reached its record count     */
  bool     lost_power;   /**< stopped because AFE_ON went off, and the
                              buffers were emptied with it - invariant 9  */
  uint16_t stride;       /**< bytes per record: 4 + 4 per enabled channel*/
  uint8_t  fields;
  uint32_t available;    /**< whole records waiting to be taken          */
  uint32_t produced;
  uint32_t dropped;      /**< records the buffer had no room for         */
  board_daq_config_t config;
} board_daq_state_t;

/** The always-available accumulator: every trigger adds to it and a read
    takes it away. Unlike the ring it CANNOT overflow - a slow link makes
    the averaging window longer, not the data older, and there is nothing to
    drop. Message in a bottle or fibre, the same code and the same answer.

    `sum` holds one total per configured field, `count` how many went into
    it, and `first`/`last` the span they came from. Divide if you want the
    mean; the count is right there. */
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

/** Copy the accumulator out and reset it. `fresh` is false when nothing has
    arrived since the previous take, which is what a caller blocks on. */
void Board_DaqTakeLive(board_daq_live_t *out);

/** NULL when it took, and the reason in the board's own words when it did
    not. The board owns the words: it is the only thing that knows which
    check failed, and a host guessing at a list of possible causes is the
    second answer this codebase keeps deleting. */
const char *Board_DaqConfigure(const board_daq_config_t *cfg);
/** Override the software clock's interval after configuring. The command
    layer uses it to fit a free-running task to the link it answers on;
    only that layer knows the baud. */
void Board_DaqSetInterval(uint32_t interval_us);

const char *Board_DaqStart(void);
void Board_DaqStop(void);
void Board_DaqState(board_daq_state_t *out);

/** Which channel field `n` of a record carries. This is what lets a host
    decode the bytes without a copy of the record shape. */
bool Board_DaqField(uint8_t field, uint8_t *channel);

/** Advanced by the main loop for a software-clocked task. */
void Board_DaqPoll(void);

/** Fed from the injected end-of-sequence for a TIM1-clocked one. */
void Board_DaqOnInjected(const int16_t *phase);

uint32_t Board_DaqAvailable(void);
uint16_t Board_DaqTake(uint8_t *out, uint16_t max_records);


/** One measurement, whatever took it. 16 bytes so the ring is a round
    number and fifteen fit in one Modbus reply. `v` is source-defined and
    raw - every conversion stays where it was defined (invariant 7). */
#define BOARD_LOG_SOURCE_PHASES 0U   /**< v = U, V, W, TIM1->CNT at latch  */
#define BOARD_LOG_SOURCE_ANGLE  1U   /**< v = value, crc, register         */
#define BOARD_LOG_SOURCE_IMU    2U   /**< v = quaternion i, j, k, real     */
#define BOARD_LOG_SOURCE_DRIVE  3U   /**< v = id, iq in 10 mA, theta_hat as
                                          a turn in 65536, innovation in
                                          0.1 mrad                          */
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

/** Arm the ring for a bitmask of sources, and empty it. Zero disables.
  *
  * `min_gap_cycles` is the least a source must leave between its own
  * pushes, so a fast producer cannot fill the ring and lock a slow one out.
  * Zero lets every source run free, which is what the angle loop did.
  */
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


/** A differential code as the converter gives it: offset binary, 32768 is
    0 V. Every differential read goes through this, regular or injected. */
int32_t Board_AdcDifferential(uint32_t raw);

/** Is there a timer to trigger from and an injected group to trigger? */
bool Board_SyncReady(void);

/** Start latching. False unless ready. Refuses the meter while armed. */
const char *Board_SyncArm(void);
void Board_SyncDisarm(void);
bool Board_SyncArmed(void);

/** The last triple, copied whole so no reader mixes two conversions. */
void Board_SyncLatest(board_sync_sample_t *out);

/** Where in the PWM period the triple is taken, as CCR4 in timer ticks.
    Takes effect immediately, armed or not. False if out of range. */
bool Board_SyncSetTrigger(uint16_t ticks);
uint16_t Board_SyncTrigger(void);
void Board_SyncState(board_sync_state_t *out);

/** From the injected end-of-sequence callback, and from the overrun one.
    The handle is opaque here on purpose: this header carries stdint and
    stdbool and nothing else, and one HAL type in it drags the whole tree
    into everything that reads a board fact. */
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

/** Disconnect TIM1's break input, for bench work with the gate drivers unpowered.
    Clearing the latch alone cannot work: with PE15 low the break is a level
    and the hardware holds MOE clear. Does not survive a reset. */
bool Board_PwmSetBreakBypass(bool on);
bool Board_PwmBreakBypassed(void);

/** The silent host's stage cleanup: MOE down, the break bypass back in
    force. board_power.c calls it once per quiet transition, so a killed
    script's armed stage does not outlive its rail claims. A live
    session's broker keeps the link speaking; the observers never
    stopped and are untouched. */
void Board_PwmSessionDrop(void);

/** Clear the break latch. Does NOT re-arm - the caller must ask again. */
bool Board_PwmClearFault(void);

/** Which legs have their two gate pins joined, bit 0 = U, 1 = V, 2 = W.
    Borrows the pins as GPIO, so it answers 0 while the stage is armed. */
uint8_t Board_PwmGateShorts(void);

/** All three, or none: never a cycle built from two calls. */
const char *Board_PwmSetAll(const uint16_t *ticks);

/** Duty in ticks Q16.16, dithered so the MEAN is what was asked for.
    One tick of ARR 2375 is 0.0421 % of duty, so 34.54 % lands between two
    of them; a first-order sigma-delta in TIM1's update interrupt spends
    the whole ticks and carries the fraction. Idle tones come with it. */
const char *Board_PwmSetAllFine(const uint32_t *ticks_q16);

/** Two compare triples, A one PWM period and B the next, swapped by the
    update interrupt at every overflow so each lands - preloaded - at the
    underflow and owns a whole period. NULL when taken, else the refusal. */
const char *Board_PwmSetAlternate(const uint16_t *a, const uint16_t *b);
void Board_PwmDutyRequested(uint32_t *ticks_q16);
void Board_PwmDitherStep(void);

/** The drive's hold on the compares. While on: the dither and the
    alternate are off, a host duty write is refused, and the triple
    Board_PwmSetNext leaves is committed by the update interrupt at the
    UNDERFLOW - so it lands, preloaded, at the next overflow and the pulse
    it shapes is symmetric. Written at the overflow it would land mid-pulse,
    and an fs/2 injection would average to nothing at the sample point. */
void Board_PwmDriveOwn(bool on);
void Board_PwmSetNext(const uint16_t *ticks);

uint16_t Board_PwmGetDuty(uint8_t phase);

void Board_PwmState(board_pwm_state_t *out);


uint8_t Board_PartCount(void);
bool Board_Part(uint8_t index, board_part_t *info);

/** ADC sampling time, as an index 0..7 into the H7's eight, shortest first.
    Applies to every channel the meter reads; 0 (1.5 cycles) is the default
    and what every measurement before this used. */
bool    Board_AdcSetSampleTime(uint8_t index);
uint8_t Board_AdcSampleTime(void);

/** Is this channel one the injected group converts? Only those three can
    be clocked from TIM1; everything else has to come through the meter. */
bool    Board_AdcIsPhase(uint8_t index);
int32_t Board_AdcPhaseSlot(uint8_t index, const int16_t *phase);

uint8_t Board_AdcCount(void);
bool    Board_AdcChan(uint8_t index, board_chan_t *info);

/**
  * @brief  Read one channel.
  * @param  microvolts  Voltage at the ADC pin. Not the sensed quantity for the
  *                     phase inputs, which sit behind unknown AFE gain.
  * @param  scaled      Physical quantity in the channel's unit, 0 when the
  *                     channel has no defined unit.
  */
bool Board_AdcRead(uint8_t index, int32_t *raw, int32_t *microvolts, int32_t *scaled);

/* False from any of these four means no reading was taken - a bad index, or a
   conversion that did not complete. It is never a measurement of zero: on a
   differential channel code 0 is 0 V, so a failure reported as data would be
   indistinguishable from a signal. */

bool Board_PhaseRaw(int32_t *u, int32_t *v, int32_t *w);
bool Board_DcBus(int32_t *raw, int32_t *millivolts);
bool Board_Ntc(int32_t *raw, int32_t *centidegc);

/** The MCU die, centi-degrees C. Needs the ADC reference like
  * everything else, so it is blind whenever AFE_ON is low. */
bool Board_McuDie(int32_t *raw, int32_t *centidegc);

/** Amperes from a centred phase code - what Board_AdcDifferential returns.
    The shunt and the amplifier gain come from the calibration record. */
float Board_PhaseAmps(uint8_t leg, int32_t centred);

/** The affine form of the two conversions the drive needs at 50 kHz:
    quantity = (code - offset) * per_code, with the record's trim folded
    into the factor. Cached by board_drive.c at every mode change, so an
    edit to the record reaches the loop then - and the loop never calls
    into the meter's -O0 arithmetic. Measured 2026-08-31: three
    Board_PhaseAmps calls were most of a 6 756-cycle interrupt. */
void Board_PhaseScale(uint8_t leg, int32_t *offset_raw,
                      float *amps_per_code);
void Board_DcBusScale(int32_t *offset_raw, float *volts_per_code);

/* ---- calibration -------------------------------------------------------- */

/** Channels the record carries a correction for. The ADC table's length, and
    checked against it at init - a table that grew past this is a record that
    would silently stop correcting the new channels. */
/** Nodes in the thermal observer. Mirrors thermal_node_t, and the
  * calibration record carries one ceiling per node. Ten, not six: the
  * drivers and the phases are three nodes each, one per leg. The count on
  * the wire lets a host follow the LENGTH - the meaning of the indices
  * changed, which is why this was CMD_PROTO MAJOR 2 (cmd.h). */
#define BOARD_THERMAL_NODES 10

#define BOARD_CAL_CHANNELS 10U

/** Which scalar Board_CalSetParam/GetParam addresses. Integers in the unit
    that makes them integers, because the wire bans floating point. */
#define BOARD_CAL_VREF_UV      0U  /**< ADC reference, microvolts           */
#define BOARD_CAL_SHUNT_UOHM   1U  /**< phase shunt, microhms               */
#define BOARD_CAL_AMP_GAIN_PPM 2U  /**< phase amplifier gain, ppm of 1 V/V  */
#define BOARD_CAL_BUS_R_TOP    3U  /**< DC link divider top, ohms           */
#define BOARD_CAL_BUS_R_BOTTOM 4U  /**< DC link divider bottom, ohms        */
#define BOARD_CAL_NTC_R25      5U  /**< thermistor at 25 C, ohms            */
#define BOARD_CAL_NTC_BETA_MK  6U  /**< B constant, milli-kelvin            */
#define BOARD_CAL_NTC_RFIXED   7U  /**< divider partner, ohms               */
#define BOARD_CAL_NTC_T25_CK   8U  /**< reference temperature, centikelvin  */
/* The two supply senses. Their own dividers, because a divider is the
   channel's and not a unit's - R113 gives the +5 rail 10k/10k and the
   gate supply 47k+10k over 10k. */
#define BOARD_CAL_R5_R_TOP     9U  /**< +5 sense divider top, ohms          */
#define BOARD_CAL_R5_R_BOTTOM 10U  /**< +5 sense divider bottom, ohms       */
#define BOARD_CAL_VG_R_TOP    11U  /**< gate supply divider top, ohms       */
#define BOARD_CAL_VG_R_BOTTOM 12U  /**< gate supply divider bottom, ohms    */
#define BOARD_CAL_DEADTIME_NS 13U  /**< half-bridge dead time, nanoseconds  */
#define BOARD_CAL_DEADTIME_SKEW 14U /**< lead-lag trim, DTG counts         */
/* One past the last id above. It is a COUNT, not a coincidence: op 0
   walks 0..COUNT-1, so an id added without moving this is a field the
   board holds and never reports - measured, deadtime_ns read back as
   absent from a record that had it. */
/* CAL_VERSION 8: what the drive is told. In the record for the reason the
   dead time is - a board runs the same drive after a reset that it ran
   before - and in the units that make them integers. A signed one travels
   as its two's complement in the u32. The commissioning
   (host/coaxial/commission.py) measures and writes them. */
#define BOARD_CAL_MOTOR_R_UOHM        15U  /**< phase resistance, microhms     */
#define BOARD_CAL_MOTOR_LD_NH         16U  /**< d inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LQ_NH         17U  /**< q inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LAMBDA_UVS    18U  /**< PM flux linkage, uV.s          */
#define BOARD_CAL_MOTOR_POLE_PAIRS    19U
#define BOARD_CAL_DRV_KP_MV_PER_A     20U  /**< current loop kp, mV/A          */
#define BOARD_CAL_DRV_KI_V_PER_AS     21U  /**< current loop ki, V/(A.s)       */
#define BOARD_CAL_DRV_L1_MILLI        22U  /**< observer angle gain, 1e-3      */
#define BOARD_CAL_DRV_L2_MILLI        23U  /**< observer speed gain, 1e-3/s    */
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
#define BOARD_CAL_PARAM_COUNT 45U

/** One channel's correction, applied to the raw code before any scaling. */
typedef struct
{
  int32_t offset_raw;   /**< subtracted first; what a zero measures       */
  int32_t gain_ppm;     /**< then scaled by 1 + gain_ppm/1e6              */
} board_cal_chan_t;

/** The whole record, as it sits in flash. Append-only for the same reason
    command 0x41 is: a stored record from an older firmware is read back by a
    newer one, and a moved field is a silently wrong calibration. Growing it
    means bumping CAL_VERSION in board_cal.c, which invalidates what is
    stored rather than misreading it. */
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

  /* The half-bridge dead time. Here and not a #define because it is the
     one number between the two FETs of a leg, and a compile-time constant
     means the board carries whatever the last flash happened to hold -
     measured 2026-08-29, a stale binary reported 79 ns for an hour after
     the source said 30. In the record it is asked for, stored, and read
     back. */
  uint32_t deadtime_ns;

  /* Lead against lag, in DTG counts. The gate drive is not symmetric -
     one edge goes through a different resistor than the other - so the
     two transitions of a leg need not want the same dead time. Positive
     lengthens the one the counter reaches counting up and shortens the
     other by the same, so the pair still averages `deadtime_ns`.

     Zero until something is measured. Nothing here has been on a scope,
     and a trim invented from a datasheet would be a number pretending to
     be a measurement. */
  uint32_t deadtime_skew;
  uint32_t ntc_r25_ohm;
  uint32_t ntc_beta_mk;
  uint32_t ntc_rfixed_ohm;
  uint32_t ntc_t25_ck;
  board_cal_chan_t chan[BOARD_CAL_CHANNELS];

  /* The thermal envelope. In the record and not in the source because a
     ceiling the firmware invented would be exactly the judgement invariant
     10 forbids - this way the board holds a limit it was GIVEN, and one
     board can carry a different envelope from the next without a rebuild.
     Zero disables a node's ceiling, which is what a node with no measurement
     behind it deserves. */
  int32_t  soa_limit_centi[BOARD_THERMAL_NODES];
  uint32_t soa_throttle_ppm;   /**< where derating starts, parts per million */

  /* CAL_VERSION 8: the drive. Ids BOARD_CAL_MOTOR_* and BOARD_CAL_DRV_*,
     in that order; board_drive.c turns them into the floats it runs on. */
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

  uint16_t crc;
} board_cal_t;

/** Load the stored record, or fall back to the compiled-in defaults. Called
    once from main() before anything reads a channel. */
void Board_CalInit(void);

/** The record in force now, stored or default. */
const board_cal_t *Board_Cal(void);

/** Whether flash holds a valid record, as against these being the defaults. */
bool Board_CalStored(void);

/** Replace the working record with the compiled-in defaults. RAM only until
    Board_CalSave(). */
void Board_CalDefaults(void);

/** Re-read flash, discarding uncommitted edits. False if nothing valid is
    stored, in which case the working record is untouched. */
bool Board_CalLoad(void);

/** Commit the working record to flash and read it back to prove it landed. */
bool Board_CalSave(void);

/* False from either of these means the id or the index does not exist, or the
   value would make a conversion divide by zero. */
bool Board_CalSetParam(uint8_t id, uint32_t value);
bool Board_CalGetParam(uint8_t id, uint32_t *value);

bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm);

/** One node's ceiling, centi-degrees C. Zero disables it. Changes the record
    in RAM; `Board_CalSave` is what makes it survive a power cycle. */
bool Board_CalSetLimit(uint8_t node, int32_t limit_centi);

/** Where derating starts, parts per million of the budget. */
bool Board_CalSetThrottle(uint32_t ppm);
bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm);

/**
  * @brief  Correct one raw code: offset first, then gain.
  * @return The code unchanged for an index the record does not cover, because
  *         refusing to report is worse than reporting uncorrected.
  */
int32_t Board_CalApply(uint8_t index, int32_t raw);

/**
  * @brief  Measure a channel now and store the reading as its offset.
  *
  * The zero of "zero and span": whatever is applied to the input at this
  * moment becomes the new origin. The board does not know or check what that
  * is - pointing it at a live input is the operator's mistake to make.
  *
  * @param  measured  The code that was stored, before correction.
  */
bool Board_CalZero(uint8_t index, int32_t *measured);

/**
  * @brief  Measure a channel now and trim its gain so the reading equals
  *         `reference`, in the channel's own raw units after offset.
  *
  * The span of "zero and span": apply a known reference, say what it should
  * read, and the correction follows. Refused when the channel reads zero
  * after offset - there is no finite gain that turns nothing into something.
  */
bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured);

/* ---- IMU ---------------------------------------------------------------- */

/**
  * @brief  Bring SPI2 to what the BNO08X needs and take PB12 as a GPIO chip
  *         select. See board_imu.c for what CubeMX generated and why it does
  *         not match the part.
  * @return False if the peripheral would not re-initialise.
  */
bool Board_ImuInit(void);

/** SPI2 and the IMU's control pins, without resetting the part.
  * What Board_ImuPoll uses, so the reset's 130 ms can be staged across main
  * loop passes rather than spent in one. */
bool Board_ImuBusInit(void);

/**
  * @brief  Pulse NRSTN with BOOTN held high, then wait out the part's own
  *         initialisation. Board_ImuInit() ends with this.
  *
  * CubeMX drives both PD10 (NRSTN) and PD11 (BOOTN) low at boot, which holds
  * the part in reset and strapped for the bootloader. Neither is a state the
  * firmware wants and neither can be fixed in the .ioc's initial level alone,
  * because BOOTN must be high BEFORE NRSTN is released - it is sampled there.
  */
void Board_ImuReset(void);

/** Whether Board_ImuInit() succeeded. Every call below fails until it has. */
bool Board_ImuReady(void);

/**
  * @brief  Read one SHTP cargo, if the part has one waiting.
  * @param  channel  The SHTP channel it arrived on.
  * @param  cargo    The cargo WITHOUT its four-byte header.
  * @param  len      Cargo bytes, 0 when the part had nothing to say.
  * @return False on a transfer error or a header that contradicts itself.
  *         True with *len == 0 is an idle part, which is not an error.
  */
bool Board_ImuRead(uint8_t *channel, uint8_t *cargo, uint16_t cap,
                   uint16_t *len);

/**
  * @brief  Frame a payload onto an SHTP channel and clock it out.
  * @return False on a bad channel, a payload that will not fit, or a
  *         transfer error. The channel's sequence number advances only on
  *         a transfer that went out.
  */
bool Board_ImuWrite(uint8_t channel, const uint8_t *payload, uint16_t len);

/** Wait up to @p ms for the part to say it has something. False on timeout.
  *
  * For a caller that asked a question and must not read before the answer
  * exists. Pumps the STO charge pump while it spins.
  */
bool Board_ImuWaitReady(uint32_t ms);

/** Ask the part to report `report_id` every `interval_us`, and REMEMBER it.
  *
  * The BNO08X forgets on reset and AFE_ON resets it, so the poll re-applies
  * this after every init. Without that, one blink of the rail stopped the
  * reports and the loop still called itself running. */
bool Board_ImuSetFeature(uint8_t report_id, uint32_t interval_us);

/** What was last asked for. Interval zero means nothing has been. */
void Board_ImuFeatureAsked(uint8_t *report_id, uint32_t *interval_us,
                           bool *pending);

/**
  * @brief  Collect and discard whatever the part has queued.
  * @return How many cargoes were drained.
  *
  * A reset leaves three messages waiting - the SHTP advertisement, the
  * executable's reset announcement and SH-2's unsolicited initialisation
  * (5.2.1) - and H_INTN stays asserted until they are taken. Writing on top
  * of them clocks a request into a part that is mid-sentence.
  */
uint8_t Board_ImuDrain(uint8_t limit);

/** Drive and release GPIOB pin `pin`, reporting what the pin then read.
  *
  * Bit 0 drove high and read high, bit 1 drove low and read low, bit 2 read
  * high with the pull-up, bit 3 read low with the pull-down. 0x0F is a pin
  * nothing else is holding. Leaves the pin an input and forces the next IMU
  * command to re-initialise SPI2.
  */
uint8_t Board_ImuPinCheck(uint8_t pin);

/** Assert PS0/WAKE on a drained part and time H_INTN's answer.
  *
  * Milliseconds, or 0xFFFF if the line never asserted inside `ms`, or 0xFFFE
  * if the part was still holding it low and the question could not be put.
  * A write clocks into a part that has not answered this, which is a write
  * nothing acts on.
  */
uint16_t Board_ImuWakeTest(uint16_t ms);

/**
  * @brief  Clock four bytes out and hand back exactly what came in.
  * @return False only if the transfer itself failed.
  *
  * No parsing. 0xFF four times is a part that is absent, unpowered or held in
  * reset, because MISO floats or idles high; four zeros is a part that is
  * there and has nothing to say. Telling those apart is the first question at
  * a bench and the header parser cannot answer it - it refuses both.
  */
/** Clock `len` bytes and keep what comes back, with no framing.
  *
  * `select` is the bring-up question: with it false the transfer runs with
  * chip select left high, which the part must ignore. Data coming back
  * anyway says chip select is not reaching it - the one hardware fault this
  * firmware can prove from the inside.
  */
bool Board_ImuProbe(uint8_t *out, uint8_t len, bool select);

/** The SPI2 kernel clock and the bit rate Board_ImuInit settled on, so the
    bench can see the number rather than infer it from a silent part. */
void Board_ImuClock(uint32_t *kernel_hz, uint32_t *bitrate_hz);

bool Board_AfeOn(void);
void Board_SetAfeOn(bool on);
bool Board_Pe15(void);

uint32_t Board_SysClkHz(void);
uint32_t Board_HclkHz(void);
uint8_t  Board_SysClkSource(void);   /**< 0 HSI, 1 CSI, 2 HSE, 3 PLL1, 4 other */
uint32_t Board_Cycles(void);

/** True when SYSCLK comes from the HSE crystal, directly or through PLL1. */
bool Board_SysClkOnCrystal(void);

/** Enable the cycle counter the comms stack uses as its timebase. */
void Board_TimebaseInit(void);

/** Per-channel result of a burst. Raw codes only: scaling is the host's job,
    so a different divider or thermistor needs no firmware change. Means and
    deviations are in milli-codes (raw x 1000) to keep fractions without
    putting a float on the wire. */
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

/** Most passes one burst may make. Named because the command handler checks
    it too: see h_adc_burst for why the limits live in both places. */
#define BOARD_BURST_MAX_SAMPLES 10000U

/** Channels one burst can cover, and the size every caller's `out` array must
    have. The bound is the selector's, not the table's: `mask` is 16 bits, so
    no request can ever name a seventeenth channel however the table grows. */
#define BOARD_BURST_MAX_CHAN 16U

/**
  * @brief  Sample a set of channels repeatedly and return per-channel statistics.
  * @param  mask         Bit i selects channel i of the channel table.
  * @param  samples      1..10000 passes over the selected set.
  * @param  interval_us  Requested spacing between passes; 0 means as fast as
  *                      the conversions allow.
  * @param  out          At least Board_AdcCount() entries.
  * @param  count        Channels actually measured, in ascending index order.
  * @param  elapsed_us   Wall time the burst took, so the host can see the rate
  *                      it really got rather than the one it asked for.
  * @return False if the mask is empty, the count is out of range, the burst
  *         would exceed BOARD_BURST_MAX_US, or a conversion failed.
  */
bool Board_AdcBurst(uint16_t mask, uint16_t samples, uint32_t interval_us,
                    board_burst_t *out, uint8_t *count, uint32_t *elapsed_us);

/**
  * @brief  Sample one ADC back to back and return basic noise statistics.
  * @param  adc_index  1..3; the differential phase channel on that ADC.
  * @param  samples    1..1000.
  * @return False if either argument is out of range, or a conversion failed.
  */
bool Board_AdcNoise(uint8_t adc_index, uint16_t samples,
                    int32_t *mean_uv, int32_t *min_raw, int32_t *max_raw,
                    uint32_t *span_raw, uint32_t *stddev_uv);

/* ---- self test ---------------------------------------------------------- */

/**
  * @brief One result from the board's self test.
  *
  * status is PASS or FAIL only where the board can genuinely PROVE the answer
  * from its own registers - a clock that is not locked, a calibration that never
  * ran, a checksum that does not match. Anything that would need a calibrated
  * instrument to judge is reported as INFO with its value, and the decision
  * belongs to whatever test executive is driving the line.
  *
  * That split is deliberate. This board is a dumb slave: it measures and
  * reports. It does not know what "good" is, and a limit compiled into firmware
  * is a limit nobody on the line can see or change.
  */
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

/**
  * @brief  Run every self check and fill @p out.
  * @return Number of checks written, never more than @p capacity.
  */
uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity);

/** Leave the binary link and resume the ASCII console, once the reply is out. */
void Board_RequestConsoleMode(void);


/** What the observer knows: one measurement, the rest estimates.
  *
  * `ntc_measured` is what tells them apart and must not be ignored - with
  * AFE_ON low there is no NTC measurement at all, and the nodes then run
  * open on power and time.
  */
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
} board_thermal_t;

/** The thermal budget: how much is spent and how long is left.
  *
  * `used` is one byte per node - 0 at ambient, 255 at the limit - because
  * "how close am I" is the question, and a temperature cannot answer it
  * without the limit beside it.
  */
typedef struct
{
  uint8_t  used[BOARD_THERMAL_NODES];
  uint8_t  worst;
  uint8_t  worst_node;
  int32_t  millis_to_limit;  /**< -1 when it is not heading for a limit */
  bool     throttling;
  bool     tripped;
  uint32_t trips;            /**< how many times it has stopped the stage */
} board_budget_t;

bool Board_ThermalBudget(board_budget_t *out);

/** Set one node's ceiling, degrees C. Zero disables that node's limit. */
bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at);

void Board_ThermalInit(void);
void Board_ThermalPoll(void);
bool Board_ThermalState(board_thermal_t *out);
bool Board_ThermalSetNode(uint8_t node, float to_board, float capacity);
bool Board_ThermalSetBoard(float to_ambient, float capacity);

/**
  * @brief  How often the observer borrows the AFE rail for an NTC sample.
  * @param  every_ms   period between samples; 0 stops sampling entirely
  * @param  settle_ms  how long the reference is given before the read
  *
  * Sampling costs the state it measures - the rail is shared with the gate
  * drivers through an inverted gate - so the trade is the caller's to make.
  */
bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms);

/** What the sampling is set to now. */
void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
