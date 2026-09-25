/** world.h - What a machine's motors turn, in C11: each one's load, the body they carry. */
#ifndef WORLD_H
#define WORLD_H

#include <stdbool.h>
#include <stdint.h>

#include "drive.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The most motors one world carries: a humanoid's joints and a spare. */
#define WORLD_MOTORS 32U

/** What a motor's shaft turns. */
typedef enum
{
  WORLD_FREE = 0,     /**< nothing: the rotor alone */
  WORLD_JOINT,        /**< a link through a gear, gravity on it */
  WORLD_ROTOR,        /**< a propeller on the shaft: drag and thrust as speed squared */
  WORLD_WHEEL,        /**< a wheel through a gear, its share of the vehicle */
  WORLD_LOAD_KINDS
} world_load_kind_t;

/** One motor's load, at the load's side of its gear. */
typedef struct
{
  uint8_t kind;       /**< world_load_kind_t */
  float gear;         /**< motor turns a load turn; a rotor's is 1 */
  float inertia;      /**< kg m^2 about the load's axis: a link, a propeller, a wheel */
  float mass;         /**< joint: the link's, kg */
  float arm;          /**< joint: its centre of mass from the axis, m */
  float damping;      /**< N m s at the load */
  float k_drag;       /**< rotor: N m / (rad/s)^2 */
  float k_thrust;     /**< rotor: N / (rad/s)^2 */
  float radius;       /**< wheel: m */
  float angle;        /**< joint: rad from hanging straight down, at the start */
} world_load_t;

/** What the loads carry together. */
typedef enum
{
  WORLD_GROUND = 0,   /**< nothing moves but the loads: a fixed base */
  WORLD_LIFT,         /**< the rotors' thrust against its weight, vertical */
  WORLD_VEHICLE,      /**< the wheels' along a slope, a rider pushing */
  WORLD_BODY_KINDS
} world_body_kind_t;

typedef struct
{
  uint8_t kind;       /**< world_body_kind_t */
  float mass;         /**< kg, rider included */
  float gravity;      /**< m/s^2 */
  float slope;        /**< vehicle: rad, uphill positive */
  float crr;          /**< vehicle: rolling resistance */
  float cda;          /**< vehicle: drag area, m^2 */
  float rho;          /**< vehicle: air, kg/m^3 */
  float rider;        /**< vehicle: the rider's push, N */
  float period;       /**< vehicle: a kick's period, s; 0 pushes steadily (pedalling) */
  float duty;         /**< vehicle: the part of a period a kick pushes */
} world_body_t;

typedef struct
{
  world_body_t body;
  world_load_t load[WORLD_MOTORS];
  uint8_t motors;

  double t;                        /**< s, the body's time */
  float  shaft[WORLD_MOTORS];      /**< rad, each motor's mechanical angle, unwrapped */
  float  speed[WORLD_MOTORS];      /**< rad/s, each motor's mechanical speed */
  float  height, climb;            /**< lift: m, m/s */
  float  velocity, distance;       /**< vehicle: m/s, m */
} world_t;

/** Everything at rest where the loads start: shafts at zero, the body on the ground. */
void world_init(world_t *w);

/** Motor `i`'s load at its shaft: the torque it opposes, signed as drive_model's `load`, and
    the inertia it adds there. */
void world_load(const world_t *w, uint8_t i, float *torque, float *inertia);

/** Motor `i` has moved: its shaft's angle (unwrapped) and speed, mechanical. */
void world_motor(world_t *w, uint8_t i, float shaft, float speed);

/** The body brought to time `t`: a lift's height, a vehicle's distance. */
void world_advance(world_t *w, double t);

/** A joint's angle from hanging, rad; a wheel's vehicle speed, m/s; a rotor's thrust, N. */
float world_output(const world_t *w, uint8_t i);

/** One board's motor and what it turns: the firmware's PMSM model (drive_model.c) on motor
    `index` of a world. */
typedef struct
{
  drive_model_t motor;
  world_t      *world;
  uint8_t       index;
  float         j_motor;           /**< the rotor's own inertia, before the load's */
  float         shaft;             /**< rad, mechanical, unwrapped */
  float         theta_was;         /**< the model's electrical angle last period */
} world_plant_t;

void world_plant_init(world_plant_t *p, world_t *w, uint8_t index,
                      const drive_model_params_t *motor);

/** One PWM period: driven at these duties, or with the gates off the inverter open and the
    rotor coasting; the shunts' currents and the link as they read now. */
void world_plant_step(world_plant_t *p, const float *duty, bool driven, float ts,
                      drive_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_H */
