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

uint8_t Board_PartCount(void);
bool Board_Part(uint8_t index, board_part_t *info);

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

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
