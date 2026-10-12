/** aero.h - a frame flown from a compiled intent: its payload's latent targets and impedances
    onto the frame's attitude, an extended state observer for what the air and the rotors do
    unasked, the thrusts shared inside each rotor's reach by an active set. A pass has no
    branch: a mode dropped is a NaN, masked. Portable C11. */
#ifndef AERO_H
#define AERO_H

#include <stdint.h>

/** The latent modes, the payload's order: the collective, m/s^2 along the rotors' axis; that
    axis's tilt from up, rad, as an attitude's - the nose up, the axis back; the right wing
    down, the axis right - in the level frame of the heading the core holds, the frame's own
    as it first stood turned on by the yaw rate asked; the rates of the nose to its right,
    up, and of the right wing down, rad/s, about the frame's own axes. */
#define AERO_THRUST     0U
#define AERO_PITCH      1U
#define AERO_ROLL       2U
#define AERO_YAW_RATE   3U
#define AERO_PITCH_RATE 4U
#define AERO_ROLL_RATE  5U
#define AERO_MODES      6U

/** A mode's rows as blended: where it is asked, how much of it is tracked (1, a NaN's 0), its
    stiffness, 1/s^2, its damping, 1/s, its oscillator's amplitude. The attitude's two targets
    are blended as the point they are, on the sphere (aero_turn.c): their row is the payload's. */
#define AERO_TARGET 0U
#define AERO_WEIGHT 1U
#define AERO_STIFF  2U
#define AERO_DAMP   3U
#define AERO_SWING  4U
#define AERO_ROWS   5U

/** The task constraints, a bit and a value each: the height held, m over the floor; the height
    risen from where the payload came at the most, m. */
#define AERO_MAINTAIN 0U
#define AERO_ASCENT   1U
#define AERO_TASKS    2U

#define AERO_ROTORS 4U
#define AERO_AXES   3U
/** A wrench's rows: the collective, N, and the turns about the frame's x, y and z, N m. */
#define AERO_WRENCH 4U

/** One compiled intent, as `machine.aerolisp` packs it: a NaN target drops its mode. */
typedef struct
{
  uint32_t sequence_id;
  uint32_t blend_ms;
  float    target[AERO_MODES];
  float    stiffness[AERO_MODES];
  float    damping[AERO_MODES];
  uint32_t cpg_enabled;
  float    cpg_hz;
  float    cpg_amplitude[AERO_MODES];
  uint32_t constraints;
  float    constraint[AERO_TASKS];
} aero_payload_t;

/** The frame, laid once: kg, m/s^2, kg m^2 about its x, y (its rotors' axis) and z (its nose);
    each rotor's place, m, and the way it turns; its drag's torque a thrust, m; the wrench's
    rows' weights in the share; the observers' bandwidths on its rates and its speed, rad/s;
    the rate a height's constraint is come to at, 1/s; the most the air may lean its axis off
    the ask, a tangent; the frame's drag a kg on its speed through the air squared, 1/m; what
    a rotor's thrust loses a m/s of air along its axis, of that thrust's root, sqrt(N) s/m;
    the rate a push is laid to the wind at, rad/s. */
typedef struct
{
  float mass;
  float gravity;
  float inertia[AERO_AXES];
  float at_x[AERO_ROTORS];
  float at_z[AERO_ROTORS];
  float spin[AERO_ROTORS];
  float kappa;
  float weigh[AERO_WRENCH];
  float wo_turn;
  float wo_pull;
  float rise;
  float lean;
  float drag;
  float inflow;
  float wo_wind;
} aero_params_t;

/** A pass's reading: the frame's axes in the world by rows (world = r frame, y up); its rates
    about its own axes, rad/s; its speed in the world, m/s; its height, m; each rotor's thrust
    as its speed has it, and at the least and the most it reaches by the next pass, N; the
    pass, s. */
typedef struct
{
  float r[9];
  float spin[AERO_AXES];
  float vel[AERO_AXES];
  float height;
  float given[AERO_ROTORS];
  float lo[AERO_ROTORS];
  float hi[AERO_ROTORS];
  float dt;
} aero_in_t;

/** A pass's answer: each rotor's thrust, N; the wrench asked and the one those thrusts give;
    the rotors at their least (bits 0..3) and their most (4..7); where the rotors' axis is
    asked to point in the frame and the tilt that takes it there, about the frame's axes; what
    the observers hold: the rates' disturbance, rad/s^2, and the speed's, m/s^2 in the world;
    the heading held, cos and sin, from z toward x; the wind held, m/s in the world. */
typedef struct
{
  float    thrust[AERO_ROTORS];
  float    asked[AERO_WRENCH];
  float    gave[AERO_WRENCH];
  uint32_t bound;
  float    axis[AERO_AXES];
  float    miss[AERO_AXES];
  float    turned[AERO_AXES];
  float    pushed[AERO_AXES];
  float    head[2];
  float    wind[AERO_AXES];
} aero_out_t;

/** The core's memory, laid once (`aero_init`). */
typedef struct
{
  aero_params_t p;
  uint32_t sequence;                      /**< the payload flown */
  uint32_t swinging;                      /**< all ones while its oscillator is on */
  uint32_t begun;                         /**< all ones from the first reading on */
  uint32_t kick;                          /**< all ones the pass an oscillator began */
  uint32_t came;                          /**< all ones the pass a payload came */
  float    since;                         /**< s into the blend */
  float    share;                         /**< the blend made, 0..1 */
  float    base;                          /**< the height the payload came at, m */
  float    from[AERO_ROWS][AERO_MODES + 1U];  /**< the rows as the payload found them */
  float    to[AERO_ROWS][AERO_MODES + 1U];    /**< as it asks them */
  float    now[AERO_ROWS][AERO_MODES + 1U];   /**< as blended; the last column none */
  float    ask[AERO_MODES + 1U];          /**< each target with its oscillator on it */
  float    wave[2];                       /**< the oscillator: cos, sin */
  uint32_t headed;                        /**< all ones once a heading is held */
  float    head[2];                       /**< the heading held: cos, sin, from z toward x */
  float    began[AERO_AXES];              /**< where the axis was asked as the payload came, */
  float    stood[AERO_AXES];              /**< and last pass, in the world; the payload's */
  float    point[AERO_AXES];              /**< own point as blended, and the rate it turns */
  float    turns[AERO_AXES];              /**< at about the frame's axes */
  float    axis[AERO_AXES];               /**< where the axis is asked to point, in the frame */
  float    miss[AERO_AXES];
  float    wound[AERO_AXES];              /**< a rate mode's miss summed, rad */
  float    held[AERO_AXES];               /**< 1 where the thrusts gave its torque short */
  float    torque[AERO_AXES];             /**< asked about the frame's axes, N m */
  float    lift;                          /**< asked of the four, N */
  float    aim;                           /**< how far the axis points where asked, 0..1 */
  float    spin_hat[AERO_AXES];           /**< the rates' observer: its rates, its disturbance */
  float    turn_hat[AERO_AXES];
  float    vel_hat[AERO_AXES];            /**< the speed's observer: its speed, the push it */
  float    push_hat[AERO_AXES];           /**< holds, m/s^2, and the wind, m/s, in the world; */
  float    wind[AERO_AXES];               /**< the frame's speed through that wind, m/s; the */
  float    flow[AERO_AXES];               /**< push and the frame's drag together, m/s^2, and */
  float    air[AERO_AXES];                /**< what the rotors lose a m/s of air along their */
  float    lost;                          /**< axis, 1/s */
} aero_t;

/** The memory for a frame, at rest, no payload flown. */
void aero_init(aero_t *s, const aero_params_t *p);

/** One pass: the payload - its sequence's change is its coming -, the reading, the answer. */
void aero_step(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out);

/** The thrusts nearest `want` (a wrench, AERO_WRENCH's rows) by the rows' weights `weigh`,
    each in `lo`..`hi`: the best of every set of rotors held at an end. `bound`: which are. */
void aero_share(const aero_t *s, const float weigh[AERO_WRENCH], const double want[AERO_WRENCH],
                const float lo[AERO_ROTORS], const float hi[AERO_ROTORS], double f[AERO_ROTORS],
                uint32_t *bound);

#endif /* AERO_H */
