/**
  ******************************************************************************
  * @file    thermal_ident.h
  * @brief   Online identification of the thermal graph's parameters from
  *          the board's own thermometers, while it runs.
  *
  * THE GRAPH HAS SIXTY NUMBERS AND THE BOARD HAS THREE THERMOMETERS. An
  * identification that moved every edge would fit noise; one that moves
  * nothing leaves the derived defaults standing where a box, a fan or a
  * heat sink has made them wrong by a factor. So it carries FOUR SCALES
  * on the groups the sensors could see, each a multiplier on the derived
  * default, one by construction - and moves the two of them a cooldown
  * actually shows (`thermal_ident_online`):
  *
  *   AIR       the whole face's path to the air - the bulk K/W. What a box,
  *             an airy shelf, a fan or a heat sink changes, and what the
  *             cooldown's equilibrium and the idle level give away.
  *   CAPACITY  the laminate's J/K - the cooldown's time constant once the
  *             air path is known.
  *   SPREAD    the sources' edges into their patches, all ten together -
  *             how far a leg's heat reaches the thermistor, which is what
  *             the reading's shape after a run says.
  *   NTC       where the thermistor's element sits toward the V patch.
  *
  * The parts' own capacities - the burst budget's - are NOT among them:
  * seconds against a sensor in laminate that lags minutes, nothing on the
  * board can see them, and a scale nobody can observe would wander. They
  * stay the derived band until a bench measures one.
  *
  * HOW. A SHADOW of the observer runs open loop from the last sample - no
  * anchors, the same power and speed - and beside it the sensitivity of
  * every node to every scale, integrated with the same slices from finite
  * differences of the flows (no analytic Jacobian to get wrong). When a
  * thermometer answers, the shadow's prediction of it against the reading
  * is the innovation, the sensitivities are the regressor, and a Kalman
  * measurement step on the scales moves them - a little process noise a
  * sample so they may drift, an innovation gate so no one sample moves
  * them past three sigma, and a covariance floor while UNCERTAIN so a
  * wrong answer is not held with certainty. The shadow is then seated on
  * the observer's state - MEASUREMENT-CONSISTENT, see `seated` below -
  * and the sensitivities on the seat's own Jacobian: the next innovation
  * is again a prediction error over one interval, which is what an
  * identification should be fed - not a residual an anchor has already
  * pulled toward zero, and not the state's error either. The observer
  * itself keeps anchoring as before; only its parameters move, and only
  * within [SCALE_MIN, SCALE_MAX].
  *
  * WHEN THE DATA ARRIVES. All three thermometers sit behind AFE_ON, and
  * switching needs AFE_ON low, so nothing is identified DURING a run: it
  * is the idle board and the cooldown after every run that feed this,
  * and a cooldown is exactly where the air path, the laminate's mass and
  * the spread of the last run's heat show.
  *
  * CONFIDENCE, said as a state. The covariance's diagonal is how sure the
  * estimate is of each scale, the filtered innovation how well the model
  * predicts against the sensors' own noise. UNCERTAIN is the model not
  * trusted - fresh, or one that just stopped predicting: a board put in a
  * box reads that way within a few intervals, and the covariance is
  * inflated so the scales can move fast. CONVERGING is the estimate
  * tightening. STABLE is every scale known to a tenth and the model
  * predicting at the noise floor. What the states are worth to the
  * envelope is the caller's policy (board_thermal.c): a margin trimmed
  * while the model is not trusted, so the silicon and the laminate are
  * not run to ceilings computed on a network that has just been proved
  * wrong.
  *
  * Portable C11, host-tested: `test_thermal_core` runs it against a
  * ground truth whose situation it can change.
  ******************************************************************************
  */
#ifndef THERMAL_IDENT_H
#define THERMAL_IDENT_H

#include "thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The scales, in wire order. Append only. */
typedef enum
{
  THERMAL_IDENT_AIR = 0,
  THERMAL_IDENT_CAPACITY,
  THERMAL_IDENT_SPREAD,
  THERMAL_IDENT_NTC,
  THERMAL_IDENT_PARAMS
} thermal_ident_param_t;

/** What the estimate is worth, in wire order. */
typedef enum
{
  THERMAL_IDENT_UNCERTAIN = 0,
  THERMAL_IDENT_CONVERGING,
  THERMAL_IDENT_STABLE
} thermal_ident_state_t;

/** Where a scale may go: a quarter to four times the derived default.
  * Outside that the default is not a model of this board and the bench
  * should hear about it rather than have it absorbed. */
#define THERMAL_IDENT_SCALE_MIN 0.25f
#define THERMAL_IDENT_SCALE_MAX 4.0f

typedef struct
{
  /** The scales on the derived defaults, 1.0 each until the sensors say
    * otherwise. */
  float scale[THERMAL_IDENT_PARAMS];
  /** The covariance of the scales, the RLS's own. Its diagonal's square
    * roots are `sigma` below. */
  float p[THERMAL_IDENT_PARAMS][THERMAL_IDENT_PARAMS];
  /** The shadow: the graph run open loop since the last sample. */
  thermal_t shadow;
  /** Sensitivity of every shadow node to every scale, since the last
    * sample; and of the shadow's lagged thermistor. */
  float s[THERMAL_IDENT_PARAMS][THERMAL_NODES];
  float s_ntc[THERMAL_IDENT_PARAMS];
  /** Seconds the shadow has run since the last sample. */
  float horizon_s;
  /** Which thermometers answered at the seat, so the next sample's
    * innovation is judged only where the shadow was seated on a reading:
    * NTC, MCU die, AFE die.
    *
    * THE STATE'S ERROR IS NOT THE PARAMETERS'. The shadow starts from the
    * observer's state, and after ten blind minutes of a run that state
    * is off by whatever the wrong scales made of it; a prediction error
    * read raw at the next sample would charge all of that to the scales'
    * last thirty seconds of sensitivity. Measured on the host: the air
    * scale landed but the other three ran to their clamps and the state
    * never left UNCERTAIN. Subtracting the seat's residual was tried and
    * was not enough - a fraction of tens of kelvin is still kelvin. So
    * the seat is MEASUREMENT-CONSISTENT: the shadow's thermistor is set
    * to what the thermistor read, each die's node to what its reading
    * implies and the patch under it to what the node implies, and the
    * innovation at the next sample is the reading's change over the
    * interval against the model's - a difference a slow error cannot
    * enter. */
  bool seated[3];
  /** What each seated thermometer read at the seat, so the next sample
    * can say whether the board MOVED since: a board that is not
    * switching burns nothing that moves its temperature, and its
    * readings agree with the shadow whatever the air scale - the
    * observer's ambient estimate absorbs the difference. Such a sample
    * says nothing and is neither judged nor learned from, so an idling
    * board stays UNCERTAIN and the envelope keeps its margin until
    * something burns (the bench's rule, 2026-09-05). */
  float seat_reading[3];
  /** Samples still to pass unjudged after a blind gap, while the
    * observer's anchors bring the laminate the thermometers do not
    * reach back to them. */
  uint8_t settle_left;
  /** Seconds since a thermometer last answered - what says a gap was a
    * blind run. Not `horizon_s`: that is reset by a seat. */
  float since_sample_s;
  /** The innovation, filtered: rms of prediction error over the last
    * samples, kelvin. What the state is judged on against `noise_k`. */
  float innovation_k;
  /** What the thermometers' own noise makes an unavoidable innovation,
    * kelvin: quantisation and a sample's spread. */
  float noise_k;
  thermal_ident_state_t state;
  uint32_t updates;        /**< samples that moved the scales            */
  uint32_t stable_runs;    /**< updates in a row that stayed at the floor */
  /** True once a sample has been taken: before that the shadow has
    * nothing to be compared against. */
  bool primed;
} thermal_ident_t;

/** Start: scales at one, covariance wide, state UNCERTAIN. `noise_k` is
  * the thermometers' floor - 0.1 K for this board's NTC and dies. */
void thermal_ident_init(thermal_ident_t *id, float noise_k);

/** Start from scales a record kept: the covariance narrowed to what a
  * saved model deserves, the state CONVERGING - trusted enough to run on,
  * not yet proved against this boot's sensors. */
void thermal_ident_resume(thermal_ident_t *id, const float *scale,
                          float noise_k);

/** `out` = `base` with the scales applied. The caller runs the observer
  * on `out` and keeps `base` as the derived defaults (with the record's
  * overlays), so a scale is always a multiplier on the same thing. */
void thermal_ident_apply(const thermal_ident_t *id, const thermal_cfg_t *base,
                         thermal_cfg_t *out);

/**
  * @brief  One step of the identification beside the observer's.
  * @param  th     The observer, after its own step this slice.
  * @param  base   The derived defaults the scales multiply.
  * @param  p      This slice's dissipation.
  * @param  load   This slice's load - the speed.
  * @param  seen   The thermometers, NAN where none answered.
  * @param  dt_s   The slice.
  * @return True when a sample moved the scales, so the caller can apply
  *         them to the observer and think about saving them.
  *
  * Runs the shadow and its sensitivities forward; when a thermometer
  * answers and the shadow has run at least THERMAL_IDENT_MIN_HORIZON_S,
  * updates the scales on the innovation and re-seats the shadow on the
  * observer.
  */
bool thermal_ident_step(thermal_ident_t *id, const thermal_t *th,
                        const thermal_cfg_t *base, const thermal_power_t *p,
                        const thermal_load_t *load,
                        const thermal_sense_t *seen, float dt_s);

/** The uncertainty of one scale: the square root of its variance. */
float thermal_ident_sigma(const thermal_ident_t *id,
                          thermal_ident_param_t which);

/** Whether the samples move this scale. AIR and CAPACITY are identified
  * online; SPREAD and NTC are held at the record's values - a cooldown
  * puts no power through the legs' edges and nothing on the board reads
  * a FET, so the data cannot see them, and a scale the data cannot see
  * absorbs what the others leave over (measured: to a clamp, every
  * time). Their sigma is the prior's and is not what the state is
  * judged on. */
bool thermal_ident_online(thermal_ident_param_t which);

/** The margin the caller's envelope should keep for this state, 0..1:
  * one when STABLE, less while the model is not trusted. The policy the
  * bench asked for - "so the silicon and the laminate are not stressed
  * for nothing" - as one number the envelope multiplies its spans by. */
float thermal_ident_margin(thermal_ident_state_t state);

/** The shortest interval a prediction is judged over, seconds: shorter
  * than this the sensitivities have not grown out of the noise. */
#define THERMAL_IDENT_MIN_HORIZON_S 20.0f

/** The longest the shadow runs before it is re-seated on the observer
  * whether or not a sample came: past this the open-loop shadow of a
  * wrong model has drifted into a regime where the sensitivities no
  * longer describe it. */
#define THERMAL_IDENT_MAX_HORIZON_S 600.0f

/** A gap longer than this since the previous sample is a blind run, and
  * the sample that ends it seats the shadow without being judged: three
  * of the board's thirty-second intervals. */
#define THERMAL_IDENT_BLIND_S 90.0f

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_IDENT_H */
