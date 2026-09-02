/**
  ******************************************************************************
  * @file    drive.h
  * @brief   The control law: one PWM period in, three duties out.
  *
  * Portable C11, float, no HAL - the bargain modbus/, shtp/ and thermal/
  * make, so `drive/test/harness.c` runs it on the host against a motor model
  * and `test_drive_core.py` drives that through ctypes. Nothing here reads a
  * register; board_drive.c hands it amperes and volts and takes duties back.
  *
  * What it is: a dq current loop with decoupling, dead-time compensation, a
  * min-max SVM, square-wave HF injection with its demodulator, a two-state
  * PLL in Kalman form, a back-EMF error above a crossover speed, an I/f ramp,
  * a saturation-pulse polarity test, and the statistics a host needs to
  * judge all of it. What it is NOT: proven on a motor. Every number in
  * `drive_defaults` is a placeholder the commissioning writes over.
  *
  * The board stays a dumb slave (invariant 10). Every gain and every limit
  * arrives from the host or the calibration record; the one thing this
  * decides on its own is to drop the stage when a current passes the trip
  * it was GIVEN, which is the thermal ceiling's exception again.
  *
  * Frames: electrical radians, amplitude-invariant Clarke, the d axis on
  * the magnet. `theta_cmd` is the frame HOLD and VOLT work in; `theta_hat`
  * is the rotor observer's. Positive current is what the shunts call positive,
  * times `sign` - a parameter, because the shunt direction is traced off a
  * schematic and nothing has run current through a leg to check it.
  ******************************************************************************
  */
#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVE_PHASES 3U

/** Points in the dead-time compensation table, at multiples of `dt_step`. */
#define DRIVE_DT_POINTS 8U

/** Lags of the innovation autocorrelation the window keeps, 1..LAGS. */
#define DRIVE_LAGS 7U

/** Channels the raw-code moments cover: U, V, W, DC bus. */
#define DRIVE_MOMENT_CHANNELS 4U

typedef enum
{
  DRIVE_OFF = 0,     /**< duties at zero, everything else still runs      */
  DRIVE_VOLT,        /**< open loop: vd, vq in the command frame           */
  DRIVE_HOLD,        /**< current control in the command frame; I/f when
                          omega_target is not zero                        */
  DRIVE_SENSORLESS,  /**< current control in the rotor observer's frame          */
  DRIVE_POLARITY,    /**< two voltage pulses along theta_hat, then OFF     */
  DRIVE_MODES
} drive_mode_t;

typedef enum
{
  DRIVE_FAULT_NONE = 0,
  DRIVE_FAULT_OVERCURRENT,   /**< a phase passed `i_trip`; the stage was dropped */
  DRIVE_FAULT_STAGE,         /**< MOE went away under a running mode          */
  DRIVE_FAULT_SUPPLY         /**< AFE_ON went away: readings mean nothing     */
} drive_fault_t;

/** What the host tells the drive, in SI. The record keeps integers. */
typedef struct
{
  float r;             /**< phase resistance, ohm                          */
  float ld;            /**< d inductance, henry                            */
  float lq;            /**< q inductance, henry                            */
  float lambda;        /**< PM flux linkage, V.s                           */
  float pole_pairs;
  float kp;            /**< current loop, V/A                              */
  float ki;            /**< current loop, V/(A.s)                          */
  float l1;            /**< rotor observer angle gain, 1                         */
  float l2;            /**< rotor observer speed gain, 1/s                       */
  float inj_volts;     /**< HF injection amplitude, V                      */
  uint16_t inj_periods;/**< PWM periods per injection half cycle; 1 = fs/2 */
  float inj_phase;     /**< injection axis, rad from the frame's d axis    */
  float eps_gain;      /**< demodulated amps per radian of angle error     */
  float i_max;         /**< reference clamp, A                             */
  float i_trip;        /**< drop the stage past this, A, per phase         */
  float v_frac;        /**< of Vdc/sqrt3 the voltage vector may use        */
  float sign;          /**< +1 or -1: shunt polarity against the drive     */
  float w_lo;          /**< below this the injection error alone, rad/s   */
  float w_hi;          /**< above this the back-EMF error alone, rad/s    */
  float dt_step;       /**< dead-time table spacing, A                     */
  float dt_volts[DRIVE_DT_POINTS]; /**< voltage error at k*dt_step, V     */
} drive_params_t;

/** What a mode is asked to do. Runtime only; never in the record. */
typedef struct
{
  float id_ref;        /**< A, in the mode's frame                         */
  float iq_ref;        /**< A                                              */
  float theta;         /**< rad: the command frame's angle (HOLD, VOLT)    */
  float omega_target;  /**< rad/s the command frame ramps to (I/f)         */
  float accel;         /**< rad/s^2 of that ramp                           */
  float vd;            /**< VOLT mode, V                                   */
  float vq;
  float pol_volts;     /**< POLARITY: pulse amplitude, V                   */
  uint16_t pol_periods;/**< POLARITY: periods per pulse                    */
  uint16_t pol_gap;    /**< POLARITY: periods between them                 */
} drive_setpoints_t;

/** One period's measurement, already in amperes and volts. */
typedef struct
{
  float i[DRIVE_PHASES];   /**< phase currents as the shunts report them */
  float vdc;               /**< DC link, V                                */
} drive_sample_t;

typedef struct
{
  float duty[DRIVE_PHASES];   /**< 0..1 per leg                          */
} drive_out_t;

/** Feedback ring: one injection cycle of dq samples. Caps `inj_periods`
  * at half of it. */
#define DRIVE_FB_RING 16U

/** Running sums over a window, double so 10^5 periods do not lose bits.
  * One count per field: the innovation and the HF amplitude arrive once
  * per injection cycle, the rest every period. */
typedef struct
{
  uint32_t n;
  double sum;
  double sumsq;
} drive_acc_t;

/** Which fields the window accumulates, in wire order. */
typedef enum
{
  DRIVE_ACC_ID = 0, DRIVE_ACC_IQ, DRIVE_ACC_VD, DRIVE_ACC_VQ,
  DRIVE_ACC_EPS, DRIVE_ACC_IH, DRIVE_ACC_VDC, DRIVE_ACC_FIELDS
} drive_acc_field_t;

/** The window's totals and the innovation's lagged products. */
typedef struct
{
  uint32_t n;
  drive_acc_t acc[DRIVE_ACC_FIELDS];
  double  lag[DRIVE_LAGS + 1U];      /**< sum e[k] e[k-j], j = 0..LAGS    */
  float   e_ring[DRIVE_LAGS + 1U];   /**< the last innovations            */
  uint8_t e_head;
  float   i_peak;                    /**< largest |i_dq| seen             */
} drive_window_t;

/** Raw-code moments: what the converter did at one sample point. */
typedef struct
{
  int64_t  sum[DRIVE_MOMENT_CHANNELS];
  uint64_t sumsq[DRIVE_MOMENT_CHANNELS];
  int32_t  lo[DRIVE_MOMENT_CHANNELS];
  int32_t  hi[DRIVE_MOMENT_CHANNELS];
  uint32_t n;
  uint32_t want;                     /**< periods asked for; 0 is idle    */
} drive_moments_t;

/** Where the samples come from: the converters, or the model below. */
typedef enum
{
  DRIVE_SOURCE_ADC = 0,
  DRIVE_SOURCE_MODEL
} drive_source_t;

/** A PMSM and its inverter, for running the law with no motor and no
  * front end - drive_model.c. The same model test_drive_core.py holds in
  * Python; the two are compared there. */
typedef struct
{
  float r, ld, lq, lambda, pole_pairs;
  float sat;           /**< Ld bends by this fraction at i_sat of d current */
  float i_sat;
  float j;             /**< kg m^2                                         */
  float b;             /**< N m s                                          */
  float load;          /**< N m                                            */
  float v_dt;          /**< the inverter's dead-time volts per phase       */
  float i_knee;        /**< where that error saturates, A                  */
  float vdc;           /**< the link it runs from, V                       */
  float noise;         /**< current noise sd on each shunt, A              */
  float theta0;        /**< the rotor's angle at init, rad electrical      */
  uint8_t sub;         /**< Euler sub-steps per period                     */
} drive_model_params_t;

typedef struct
{
  drive_model_params_t p;
  float theta;         /**< electrical, the truth the rotor observer is judged by */
  float omega;         /**< electrical                                      */
  float id, iq;        /**< in the rotor's own frame                        */
  float duty_prev[DRIVE_PHASES]; /**< the pipeline: last step's duties     */
  float c, s;                    /**< cos/sin of theta at the sample      */
  float i_abc[DRIVE_PHASES];     /**< the sample's currents, noise-free   */
  uint32_t rng;
} drive_model_t;

/** The whole controller. Owned by the caller; drive_init fills it. */
typedef struct
{
  float ts;                          /**< PWM period, s                   */
  drive_params_t p;
  drive_setpoints_t sp;
  drive_mode_t mode;
  drive_fault_t fault;
  drive_source_t source;
  drive_model_t model;

  /** A cycle counter the board lends, or NULL: with it the virtual step
    * records what its three blocks cost, so an interrupt that outgrew the
    * period is read block by block rather than guessed at. */
  uint32_t (*cycles)(void);
  uint32_t cyc_sample, cyc_step, cyc_advance;

  /* the rotor observer */
  float theta_hat;
  float omega_hat;
  float eps;                         /**< last innovation, rad            */
  float eps_amps;                    /**< last demodulated error, A       */
  float ih;                          /**< last HF current amplitude, A    */
  float e_bemf;                      /**< last back-EMF angle error, rad  */

  /* the command frame */
  float theta_cmd;
  float omega_cmd;

  /* the current loop */
  float xd, xq;                      /**< integrators, V                  */
  float id, iq;                      /**< this period, in the loop frame  */
  float vd, vq;                      /**< this period's demand, V         */
  float va_out, vb_out;              /**< the fundamental applied, alpha/beta */
  float vdc;
  float   fb_d[DRIVE_FB_RING];       /**< feedback ring, see feedback()   */
  float   fb_q[DRIVE_FB_RING];
  uint8_t fb_head;
  uint8_t fb_fill;

  /* the injection */
  float    inj_sign;                 /**< +1 or -1 this period            */
  uint16_t inj_count;                /**< periods left at this sign       */
  float    sign_hist[4];             /**< the sign each recent step wrote */
  float    iq_prev;                  /**< in the injection frame          */
  float    id_prev;
  bool     have_prev;                /**< iq_prev holds a sample          */
  float    acc_q;                    /**< this cycle's demodulated sums   */
  float    acc_d;
  uint16_t cyc_count;                /**< periods into this cycle         */
  uint8_t  inj_warm;                 /**< cycles seen since it started    */
  float    demod_q;                  /**< last full-cycle demodulated q   */
  float    demod_d;
  bool     inj_valid;                /**< two whole cycles have been seen */

  /* POLARITY */
  uint32_t pol_step;
  float    pol_pos;                  /**< peak |id| on the +pulse, A      */
  float    pol_neg;

  uint32_t periods;                  /**< steps since init                */
  drive_window_t win;
  drive_moments_t mom;
} drive_t;

/** Defaults: placeholders the commissioning replaces, and said so. */
void drive_defaults(drive_params_t *p);

void drive_init(drive_t *d, float ts);

/** Put both frames at `theta`: the polarity flip, or a known start. */
void drive_set_theta(drive_t *d, float theta);

/** Change mode. NULL when taken; otherwise why not, in the board's words.
  * `stage_enabled` and `powered` are the caller's facts about MOE and
  * AFE_ON: a mode that switches needs the first, one that measures the
  * second. */
const char *drive_set_mode(drive_t *d, drive_mode_t mode, bool stage_enabled,
                           bool powered);

/** One PWM period. `stage_enabled` false runs the estimators on what the
  * converter sees and returns zero duties. Returns true when the stage
  * must be DROPPED now - a current past `i_trip` - and the mode is OFF. */
bool drive_step(drive_t *d, const drive_sample_t *in, bool stage_enabled,
                drive_out_t *out);

/** Copy the window out and start a new one. */
void drive_window_take(drive_t *d, drive_window_t *out);

/* ---- the model as the sample source ---------------------------------- */

void drive_model_defaults(drive_model_params_t *p);

/** The rotor at `p.theta0`, at rest, no current, the pipeline empty. */
void drive_model_init(drive_model_t *m);

/** The three phase currents and the link as the shunts would report
  * them now, with the noise asked for. */
void drive_model_sample(drive_model_t *m, drive_sample_t *out);

/** One period at these duties. */
void drive_model_advance(drive_model_t *m, const float *duty, float ts);

/** One period with the model as the source: sample, step, advance with
  * the step BEFORE's duties - the pipeline the stage has. The stage is
  * treated as enabled so the law runs; the caller applies `out` to real
  * gates only if MOE is set. Returns the trip like drive_step. */
bool drive_step_virtual(drive_t *d, drive_out_t *out);

/** Arm the raw-code moments for `periods`; zero forgets them. */
void drive_moments_arm(drive_t *d, uint32_t periods);

/** Feed one period's raw codes. Ignored while idle or done. */
void drive_moments_feed(drive_t *d, const int32_t *codes);

/* ---- the arithmetic, exposed for the harness ------------------------- */

void drive_clarke(const float *iabc, float *alpha, float *beta);
void drive_park(float alpha, float beta, float theta, float *d, float *q);
void drive_inv_park(float d, float q, float theta, float *alpha, float *beta);

/** The same with cos and sin already taken: one pair serves a whole step.
  * Measured 2026-08-31, four trig calls of the step's 6 756 cycles. */
void drive_park_cs(float alpha, float beta, float c, float s, float *d,
                   float *q);
void drive_inv_park_cs(float d, float q, float c, float s, float *alpha,
                       float *beta);

/** Min-max SVM: alpha/beta volts and the link to three duties, 0..1.
  * Returns the fraction the vector was scaled by to fit - 1.0 fitted. */
float drive_svm(float valpha, float vbeta, float vdc, float *duty);

/** Wrap to [0, 2 pi). */
float drive_wrap(float theta);

/** sin and cos together, a polynomial: 3e-6 worst, no library call. */
void drive_sincos(float theta, float *s, float *c);

/** The dead-time table at `amps`, odd in the current. */
float drive_dt_volts(const drive_params_t *p, float amps);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
