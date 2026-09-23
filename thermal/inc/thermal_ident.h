/**
  ******************************************************************************
  * @file    thermal_ident.h
  * @brief   Online identification of the thermal graph's parameters from
  *          the board's own thermometers, while it runs.
  ******************************************************************************
  */
#ifndef THERMAL_IDENT_H
#define THERMAL_IDENT_H

#include "thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The identified quantities, in wire order. */
typedef enum
{
  THERMAL_IDENT_AIR = 0,
  THERMAL_IDENT_CAPACITY,
  THERMAL_IDENT_SPREAD,
  THERMAL_IDENT_NTC,
  THERMAL_IDENT_AMBIENT,
  THERMAL_IDENT_PARAMS
} thermal_ident_param_t;

/** How many of them are scales on the record's network: the first four -
    what the wire carries as scales, and what `thermal_ident_apply`
    multiplies. */
#define THERMAL_IDENT_RECORD 4

/** Where the room may be identified to, degrees C. */
#define THERMAL_IDENT_AMBIENT_MIN_C -40.0f
#define THERMAL_IDENT_AMBIENT_MAX_C  85.0f

/** What the estimate is worth, in wire order. */
typedef enum
{
  THERMAL_IDENT_UNCERTAIN = 0,
  THERMAL_IDENT_CONVERGING,
  THERMAL_IDENT_STABLE
} thermal_ident_state_t;

/** Where a scale may go: a quarter to four times the derived default. */
#define THERMAL_IDENT_SCALE_MIN 0.25f
#define THERMAL_IDENT_SCALE_MAX 4.0f

typedef struct
{
  /** The scales on the derived defaults, 1.0 each until the sensors say
      otherwise - and, at THERMAL_IDENT_AMBIENT, the room in degrees C, which
      is not a scale but is estimated by the same step. */
  float scale[THERMAL_IDENT_PARAMS];
  /** The covariance of the scales, the RLS's own. */
  float p[THERMAL_IDENT_PARAMS][THERMAL_IDENT_PARAMS];
  /** The shadow: the graph run open loop since the last sample. */
  thermal_t shadow;
  /** Sensitivity of every shadow node to every scale, since the last sample;
      and of the shadow's lagged thermistor. */
  float s[THERMAL_IDENT_PARAMS][THERMAL_NODES];
  float s_ntc[THERMAL_IDENT_PARAMS];
  /** Seconds the shadow has run since the last sample. */
  float horizon_s;
  /** Which thermometers answered at the seat, so the next sample's
      innovation is judged only where the shadow was seated on a reading:
      NTC, MCU die, AFE die. */
  bool seated[3];
  /** What each seated thermometer read at the seat, so the next sample can
      say whether the board MOVED since: a board that is not switching burns
      nothing that moves its temperature, and its readings agree with the
      shadow whatever the air scale - the observer's ambient estimate absorbs
      the difference. */
  float seat_reading[3];
  /** Samples still to pass unjudged after a blind gap, while the observer's
      anchors bring the laminate the thermometers do not reach back to them. */
  uint8_t settle_left;
  /** Seconds since a thermometer last answered - what says a gap was a blind
      run. */
  float since_sample_s;
  /** The innovation, filtered: rms of prediction error over the last
      samples, kelvin. */
  float innovation_k;
  /** What the thermometers' own noise makes an unavoidable innovation,
      kelvin: quantisation and a sample's spread. */
  float noise_k;
  thermal_ident_state_t state;
  uint32_t updates;        /**< samples that moved the scales            */
  uint32_t stable_runs;    /**< updates in a row that stayed at the floor */
  /** True once a sample has been taken: before that the shadow has nothing
      to be compared against. */
  bool primed;
} thermal_ident_t;

/** Start: scales at one, the room at `ambient_c` (what the thermistor read
    at boot - a board that has not run is at the room), covariance wide,
    state UNCERTAIN. */
void thermal_ident_init(thermal_ident_t *id, float ambient_c, float noise_k);

/** The room as identified, degrees C: what the observer's `ambient` should
    be set to after every step. */
float thermal_ident_ambient(const thermal_ident_t *id);

/** `out` = `base` with the scales applied. */
void thermal_ident_apply(const thermal_ident_t *id, const thermal_cfg_t *base,
                         thermal_cfg_t *out);

/** One step of the identification beside the observer's.
    @param  th     The observer, after its own step this slice.
    @param  base   The derived defaults the scales multiply.
    @param  p      This slice's dissipation.
    @param  load   This slice's load - the speed.
    @param  seen   The thermometers, NAN where none answered.
    @param  dt_s   The slice.
    @return True when a sample moved the scales, so the caller can apply */
bool thermal_ident_step(thermal_ident_t *id, const thermal_t *th,
                        const thermal_cfg_t *base, const thermal_power_t *p,
                        const thermal_load_t *load,
                        const thermal_sense_t *seen, float dt_s);

/** The uncertainty of one scale: the square root of its variance. */
float thermal_ident_sigma(const thermal_ident_t *id,
                          thermal_ident_param_t which);

/** Whether the samples move this scale. */
bool thermal_ident_online(thermal_ident_param_t which);

/** How far the model is doubted, 0..1: none with the innovation at the
    thermometers' floor and every online quantity known to its STABLE sigma,
    all of it at the innovation that says UNCERTAIN or a sigma at its prior -
    the worse of the two, each normalised. */
float thermal_ident_doubt(const thermal_ident_t *id);

/** The margin the caller's envelope should keep, `floor`..1: the floor while
    the model is doubted whole, one when not at all, the doubt between - the
    number the envelope multiplies its ceilings' spans by. */
float thermal_ident_margin(const thermal_ident_t *id, float floor);

/** The shortest interval a prediction is judged over, seconds: shorter than
    this the sensitivities have not grown out of the noise. */
#define THERMAL_IDENT_MIN_HORIZON_S 20.0f

/** The longest the shadow runs before it is re-seated on the observer
    whether or not a sample came: past this the open-loop shadow of a wrong
    model has drifted into a regime where the sensitivities no longer
    describe it. */
#define THERMAL_IDENT_MAX_HORIZON_S 600.0f

/** A gap longer than this since the previous sample is a blind run, and the
    sample that ends it seats the shadow without being judged: three of the
    board's thirty-second intervals. */
#define THERMAL_IDENT_BLIND_S 90.0f

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_IDENT_H */
