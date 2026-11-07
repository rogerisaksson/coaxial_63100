/** wbc_beat.c - her 1 kHz pass beside her law on the host (wbc_beat.h): machine.process.sprung.passed's
    edge.felt, observer.felt, inekf.felt, motion.drives and her law's joints told, in that order, numpy's
    arithmetic operation for operation. Portable C11. */
#include "wbc_beat.h"

void wbc_beat_felt(wbc_beat_t *b, double dt)
{
  double *y = b->y;

  wbc_pass_felt(b->world, y, b->quat_at, b->gyro_at, b->accel_at, b->q_at, b->qd_at, b->qm_at);
  for (int i = 0; i < b->nj; i++)
  {
    y[b->tau_at + i] = b->motor[i];
  }
  y[b->link_at] = b->link[0];
  wbc_gmo_felt(b->gmo, &y[b->quat_at], &y[b->accel_at], &y[b->tau_at], dt);
  for (int i = 0; i < b->nimu; i++)
  {
    b->imu[i] += dt * y[b->imu_at + i];
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
  wbc_beat_drive(b, dt, now, wo, limp);
}
