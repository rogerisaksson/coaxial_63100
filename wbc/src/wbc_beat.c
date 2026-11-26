/** wbc_beat.c - the robot's 1 kHz pass beside its law on the host (wbc_beat.h): machine.process.sprung.passed's
    edge.felt, observer.felt, inekf.felt, motion.drives and its law's joints told, in that order, numpy's
    arithmetic operation for operation. Portable C11. */
#include "wbc_beat.h"

void wbc_beat_felt(wbc_beat_t *b, double dt)
{
  const int              nj = b->nj, nimu = b->nimu;
  double *restrict       y = b->y, *restrict imu = b->imu;
  const double *restrict motor = b->motor;

  wbc_pass_felt(b->world, y, b->quat_at, b->gyro_at, b->accel_at, b->q_at, b->qd_at, b->qm_at);
  {
    double *restrict y_tau = y + b->tau_at;

    for (int i = 0; i < nj; i++)
    {
      y_tau[i] = motor[i];
    }
  }
  y[b->link_at] = b->link[0];
  wbc_gmo_felt(b->gmo, &y[b->quat_at], &y[b->accel_at], &y[b->tau_at], &y[b->qm_at], &y[b->q_at], dt);
  {
    const double *restrict y_imu = y + b->imu_at;

    for (int i = 0; i < nimu; i++)
    {
      imu[i] += dt * y_imu[i];
    }
  }
  b->span[0] += dt;
}

void wbc_beat_drive(wbc_beat_t *b, double dt, double now, double wo, double limp)
{
  wbc_pass_tick(b->tick, dt, now, wo, limp);
  wbc_sil_drive(b->world, b->n, b->idx, b->target, b->feed, b->kp, b->limit);
}

void wbc_beat_passed(wbc_beat_t *b, double dt, double now, double wo, double limp)
{
  wbc_beat_felt(b, dt);
  if ((b->ahead != NULL) && (*b->ahead < 0) && (b->ahead_fn != NULL))
  {
    *b->ahead = b->ahead_fn(b->stage, b->model, b->data);
  }
  wbc_beat_drive(b, dt, now, wo, limp);
}
