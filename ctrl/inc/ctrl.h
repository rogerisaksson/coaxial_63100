/** ctrl.h - host/machine/parts.py and a feedback in C11, step for step, for a board's loops. */
#ifndef CTRL_H
#define CTRL_H

#include <stdbool.h>
#include <stdint.h>

/** A part's kind; `ctrl_kind_names` spells each as its class in machine.parts. */
typedef enum
{
  CTRL_NONE = 0,
  CTRL_GAIN,          /**< k */
  CTRL_SLEW,          /**< rate */
  CTRL_WRAP,          /**< zero */
  CTRL_LOW_PASS,      /**< tau */
  CTRL_SPEED_KALMAN,  /**< kt j b q r */
  CTRL_PI,            /**< kp ki limit */
  CTRL_ANGLE_HOLD,    /**< poles theta0 ki trim most */
  CTRL_DIRECT,        /**< limit */
  CTRL_SPEED_PI,      /**< hz limit kt j b load_k scale */
  CTRL_KINDS
} ctrl_kind_t;

/** The most parameters a kind takes. */
#define CTRL_PARAMS 7U

/** One part: its kind, its parameters in the order PARAMS names them, its state. */
typedef struct
{
  uint8_t kind;
  float   p[CTRL_PARAMS];
  float   x;          /**< an integrator */
  float   y;          /**< a filter's output, a Kalman's speed */
  float   at;         /**< AngleHold: the command's angle, deg */
  float   was;        /**< SpeedPI: the last setpoint; SpeedKalman: its variance */
  bool    primed;     /**< SpeedKalman: seen a measurement */
} ctrl_part_t;

/** One feedback loop: setpoint -> prefilter -> regulator -> command; measured -> measure
    -> estimator -> regulator. A slot of kind CTRL_NONE passes its input through. */
typedef struct
{
  ctrl_part_t prefilter;
  ctrl_part_t measure;
  ctrl_part_t estimator;
  ctrl_part_t regulator;
  float       ref;        /**< the prefiltered setpoint */
  float       value;      /**< the measured value after `measure` */
  float       estimate;   /**< what the regulator is fed back */
  float       command;    /**< the regulator's output; the estimator's next input */
} ctrl_feedback_t;

/** Rows a runner queues. */
#define CTRL_ROWS 64U

/** One row the host streams: hold `setpoint` for `ms`. */
typedef struct
{
  uint16_t ms;
  float    setpoint;
} ctrl_row_t;

/** A feedback playing rows: the host pushes (comms), one tick plays (the ISR). */
typedef struct
{
  ctrl_feedback_t   f;
  ctrl_row_t        row[CTRL_ROWS];
  volatile uint16_t head;     /**< written by the pusher only */
  volatile uint16_t tail;     /**< written by the tick only */
  float             setpoint; /**< the row playing, or the last one, held */
  float             left_s;   /**< the row playing's time left */
  bool              playing;
  uint32_t          played;   /**< rows finished */
  uint32_t          idle;     /**< ticks with no row playing */
} ctrl_runner_t;

/** Class names by kind, "" for CTRL_NONE. */
extern const char *const ctrl_kind_names[CTRL_KINDS];

/** Parameters each kind takes. */
extern const uint8_t ctrl_kind_params[CTRL_KINDS];

/** A part of `kind` with `params` (ctrl_kind_params[kind] of them), reset. False: no kind. */
bool ctrl_part_set(ctrl_part_t *part, uint8_t kind, const float *params);

/** Its state back to where it was set. */
void ctrl_part_reset(ctrl_part_t *part);

/** A filter's step: y from x. Any other kind passes x through. */
float ctrl_filter(ctrl_part_t *part, float dt, float x);

/** An estimator's step: the estimate from the measurement and the last command. */
float ctrl_estimate(ctrl_part_t *part, float dt, float measured, float command);

/** A regulator's step. `accel` NAN: the setpoint's own slope; `held`: the inner loop is
    saturated, the integrator holds. Any other kind passes the setpoint through. */
float ctrl_regulate(ctrl_part_t *part, float dt, float setpoint, float measured, float accel,
                    bool held);

/** Every slot back to where it was set, the channels to zero. */
void ctrl_feedback_reset(ctrl_feedback_t *f);

/** One pass, in slot order; the command. */
float ctrl_feedback_step(ctrl_feedback_t *f, float dt, float setpoint, float measured);

/** Rows that still fit. */
uint16_t ctrl_rows_free(const ctrl_runner_t *r);

/** A row onto the ring; false when it is full. */
bool ctrl_rows_push(ctrl_runner_t *r, uint16_t ms, float setpoint);

/** Seconds queued, the row playing included. */
float ctrl_rows_seconds(const ctrl_runner_t *r);

/** Every queued row dropped; the setpoint held. Call where the tick cannot run. */
void ctrl_rows_clear(ctrl_runner_t *r);

/** One tick: the next row when the last ran out, then the feedback; the command. */
float ctrl_runner_step(ctrl_runner_t *r, float dt, float measured);

#endif /* CTRL_H */
