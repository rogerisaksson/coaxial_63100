/** wbc_beat.h - her 1 kHz pass beside her law on the host (wbc_beat.c), each part one call on a struct laid
    once: her felt - her IMU, encoders, drives' torques and link into her edge's array, her observer's and her
    filter's sums - and her drive - her drives' tick, her law's joints told her boards. Portable C11. */
#ifndef WBC_BEAT_H
#define WBC_BEAT_H

#include "wbc_gmo.h"
#include "wbc_pass.h"

/** Her pass (machine.process.sprung.passed): pointers into her world's pass, her observer, her drives' tick and
    her arrays, her edge's offsets (machine.senses.edge), laid once (tools.cores.beat). */
typedef struct
{
  wbc_world_t  *world;    /**< her world's pass: her IMU and encoders read, her boards told (wbc_sil_drive) */
  wbc_gmo_t    *gmo;      /**< her observer: its sums over her law's pass (wbc_gmo_felt) */
  wbc_tick_t   *tick;     /**< her drives' pass (wbc_pass_tick) */
  double       *y;        /**< her edge's array */
  const double *motor;    /**< her world's motors' torques (nj) and its link's volts, into y */
  const double *link;
  double       *imu;      /**< her filter's sums (machine.senses.inekf): her spin and specific force (nimu), their span */
  double       *span;
  const int    *idx;      /**< her law's joints told (n): their setpoints, feed, stiffness (NULL her boards') and clamps */
  const double *target;
  const double *feed;
  const double *kp;
  const double *limit;
  int           quat_at;  /**< y's offsets: her turn, spin, specific force, angles, rates, motors' angles, torques, link */
  int           gyro_at;
  int           accel_at;
  int           q_at;
  int           qd_at;
  int           qm_at;
  int           tau_at;
  int           link_at;
  int           imu_at;   /**< her filter's IMU in y: from here, nimu of them */
  int           nimu;
  int           nj;       /**< her joints: y's torques */
  int           n;        /**< her law's joints told */
} wbc_beat_t;

/** Her felt `dt` s on (edge.felt, observer.felt, inekf.felt): her IMU and encoders into y (wbc_pass_felt), her
    drives' torques and her link's volts after them, her observer's sums (wbc_gmo_felt), her filter's - numpy's
    `imu += dt * y[IMU]` and `span += dt`. */
void wbc_beat_felt(wbc_beat_t *b, double dt);

/** Her drive `dt` s on at `now` (motion.drives and her law's joints told): her drives' tick (wbc_pass_tick, `wo`
    her observers' bandwidth, `limp` how far she rests limp), her law's joints told her boards (wbc_sil_drive). */
void wbc_beat_drive(wbc_beat_t *b, double dt, double now, double wo, double limp);

/** A pass between her law's: her felt, then her drive. */
void wbc_beat_passed(wbc_beat_t *b, double dt, double now, double wo, double limp);

#endif
