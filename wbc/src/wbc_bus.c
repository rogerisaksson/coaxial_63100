/** wbc_bus.c - the master bus insert's stages (wbc_bus.h): her ask's jerk and her law's torque
    command's slew soft-clipped, the command passed through the tank. No branch: masks and clips.
    Portable C11. */
#include "wbc_bus.h"
#include "wbc_np.h"

#include <math.h>

/* x soft-clipped at lim, its knee 6 dB under it: x itself to half of lim, a cubic from there
   meeting lim flat, C2 at the knee. Its knee at zero, a hop's push lost 5 % at its drive's own jerk */
static double softclip(double x, double lim)
{
  const double knee = 0.5 * lim, w = 1.5 * (lim - knee), a = fabs(x);
  const double v = npclip((a - knee) / w, 0.0, 1.0);

  return copysign(npmin(a, knee) + w * (v - ((v * v) * v) / 3.0), x);
}

/* a biquad, transposed direct form II, its a0 1 */
static double biquad(const double *bb, const double *aa, double *z1, double *z2, double x)
{
  const double y = bb[0] * x + *z1;

  *z1 = (bb[1] * x - aa[0] * y) + *z2;
  *z2 = bb[2] * x - aa[1] * y;
  return y;
}

double wbc_bus_ask(wbc_bus_t *b, int i, double dt, double qa)
{
  const double primed = (double)b->primed;

  b->qa_out[i] = primed * (b->qa_out[i] + softclip(qa - b->qa_out[i], b->jerk[i] * dt)) + (1.0 - primed) * qa;
  return b->qa_out[i];
}

double wbc_bus_slew(wbc_bus_t *b, int i, double dt, double tau)
{
  const double primed = (double)b->primed;

  b->tau_out[i] = primed * (b->tau_out[i] + softclip(tau - b->tau_out[i], b->slew[i] * dt)) + (1.0 - primed) * tau;
  return b->tau_out[i];
}

double wbc_bus_rate(const wbc_bus_k *k, wbc_bus_t *b, int i, double dt, double qd)
{
  const double open = (double)(fabs(qd) >= k->gate_rad_s);
  const double g = npclip(b->gate[i] + dt * (open / k->gate_attack_s - (1.0 - open) / k->gate_release_s), 0.0, 1.0);

  b->gate[i] = g;
  b->qd[i] = biquad(k->d_b, k->d_a, b->dz1 + i, b->dz2 + i, qd * ((g * g) * (3.0 - 2.0 * g)));
  return b->qd[i];
}

double wbc_bus_feed(const wbc_bus_k *k, wbc_bus_t *b, int i, double dt, double qd, double feed, double damping)
{
  const int    n = b->n;
  const double v = biquad(k->lp_b, k->lp_a, b->bv + 2 * n + i, b->bv + 3 * n + i,
                          biquad(k->hp_b, k->hp_a, b->bv + i, b->bv + n + i, qd));
  const double t = biquad(k->lp_b, k->lp_a, b->bt + 2 * n + i, b->bt + 3 * n + i,
                          biquad(k->hp_b, k->hp_a, b->bt + i, b->bt + n + i, feed));
  const double p = t * v, s = (double)(v > 0.0) - (double)(v < 0.0);
  const double rev = (double)(s * b->sign[i] < 0.0);
  double       hot, stale, trip, gain, cmd;

  /* the detector: a half cycle of the band's rate ended, its band power past trip_w counts one */
  b->half_e[i] += p * dt;
  b->half_t[i] += dt;
  hot = (double)(b->half_e[i] > k->trip_w * b->half_t[i]);
  stale = (double)(b->half_t[i] > k->period_s) * (1.0 - hot);
  b->count[i] = (rev * (hot * (b->count[i] + 1.0)) + (1.0 - rev) * b->count[i]) * (1.0 - stale);
  b->half_e[i] *= 1.0 - npmax(rev, stale);
  b->half_t[i] *= 1.0 - npmax(rev, stale);
  b->sign[i] = s + (double)(s == 0.0) * b->sign[i];
  b->power += p;
  /* the passivity controller: on a parasitic oscillation, the detector's - the command's band rolled
     back, the damping injected; released over recover_s - her drives are pure torque, no P gain of
     their own. Tripped by the tank at its floor too, a hop's push drained it in 30 ms, every drive
     rolled back and she rose 669 mm off one sole */
  trip = (double)(b->count[i] >= k->reversals);
  b->alpha[i] = npmax(trip, b->alpha[i] - dt / k->recover_s);
  gain = b->alpha[i] * (1.0 - k->rollback);
  cmd = (feed - gain * t) - ((b->alpha[i] * k->inject) * damping) * b->qd[i];
  /* what the bus changed of the command, for her drive's observer to know what moved its rotor */
  b->given[i] = cmd - feed;
  return cmd;
}

void wbc_bus_pass(const wbc_bus_k *k, wbc_bus_t *b, double dt)
{
  b->energy = npclip(b->energy - dt * b->power, k->tank_min_j, k->tank_max_j);
  b->power = 0.0;
  b->primed = 1;
}
