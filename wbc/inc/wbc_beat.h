/** wbc_beat.h - the robot's 1 kHz pass beside its law on the host (wbc_beat.c), each part one call on a struct laid
    once: its felt - its IMU, encoders, drives' torques and link into its edge's array, its observer's and its
    filter's sums - and its drive - its drives' tick, its law's joints told its boards. Portable C11. */
#ifndef WBC_BEAT_H
#define WBC_BEAT_H

#include "wbc_gmo.h"
#include "wbc_pass.h"

/** The robot's pass (machine.process.sprung.passed): pointers into its world's pass, its observer, its drives'
    tick and its arrays, its edge's offsets (machine.senses.edge), laid once (tools.cores.beat). */
typedef struct
{
  wbc_world_t  *world;    /**< the robot's world's pass: its IMU and encoders read, its boards told (wbc_sil_drive) */
  wbc_gmo_t    *gmo;      /**< the robot's observer: its sums over the robot's law's pass (wbc_gmo_felt) */
  wbc_tick_t   *tick;     /**< the robot's drives' pass (wbc_pass_tick) */
  double       *y;        /**< the robot's edge's array */
  const double *motor;    /**< the robot's world's motors' torques (nj) and its link's volts, into y */
  const double *link;
  double       *imu;      /**< the robot's filter's sums (machine.senses.inekf): its spin and specific force (nimu), their span */
  double       *span;
  const int    *idx;      /**< the robot's law's joints told (n): their setpoints, feed, stiffness (NULL its boards') and clamps */
  const double *target;
  const double *feed;
  const double *kp;
  const double *limit;
  int           quat_at;  /**< y's offsets: the robot's turn, spin, specific force, angles, rates, motors' angles, torques, link */
  int           gyro_at;
  int           accel_at;
  int           q_at;
  int           qd_at;
  int           qm_at;
  int           tau_at;
  int           link_at;
  int           imu_at;   /**< the robot's filter's IMU in y: from here, nimu of them */
  int           nimu;
  int           nj;       /**< the robot's joints: y's torques */
  int           n;        /**< the robot's law's joints told */
} wbc_beat_t;

/** The robot's felt `dt` s on (edge.felt, observer.felt, inekf.felt): its IMU and encoders into y
    (wbc_pass_felt), its drives' torques and its link's volts after them, its observer's sums (wbc_gmo_felt),
    its filter's - numpy's `imu += dt * y[IMU]` and `span += dt`. */
void wbc_beat_felt(wbc_beat_t *b, double dt);

/** The robot's drive `dt` s on at `now` (motion.drives and its law's joints told): its drives' tick
    (wbc_pass_tick, `wo` its observers' bandwidth, `limp` how far it rests limp), its law's joints told its
    boards (wbc_sil_drive). */
void wbc_beat_drive(wbc_beat_t *b, double dt, double now, double wo, double limp);

/** A pass between the robot's law's: its felt, then its drive. */
void wbc_beat_passed(wbc_beat_t *b, double dt, double now, double wo, double limp);

#endif
