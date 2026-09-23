/** drive_observer.c - Back-EMF observers: dual flux low, plain flux high. */
#include "drive.h"

#include <math.h>
#include <stddef.h>

/** Wrap to (-pi, pi]. */
static float wrap(float a)
{
  const float two_pi = 6.2831853f;

  while (a > 3.1415927f)
  {
    a -= two_pi;
  }
  while (a <= -3.1415927f)
  {
    a += two_pi;
  }
  return a;
}

void drive_observer_init(drive_obs_t *o, const drive_params_t *p, float ts)
{
  float lam = p->lambda;

  if (o == NULL)
  {
    return;
  }
  lam = (lam > 0.0f) ? lam : 1e-6f;

  o->wc = DRIVE_OBS_WC;
  o->cross = DRIVE_OBS_CROSS;
  o->pll_kp = DRIVE_OBS_PLL_KP;
  o->pll_ki = DRIVE_OBS_PLL_KI;
  o->blend_lo = DRIVE_OBS_BLEND_LO * o->wc;
  o->blend_hi = DRIVE_OBS_BLEND_HI * o->wc;

  /* The dual model starts on the d axis with the flux it should have, so its
     first steps are a correction rather than a search. */
  o->psi_a = lam;
  o->psi_b = 0.0f;
  o->leak_a = 0.0f;
  o->leak_b = 0.0f;
  o->pll_theta = 0.0f;
  o->pll_omega = 0.0f;
  o->flux_theta = 0.0f;
  o->flux_omega = 0.0f;
  o->theta = 0.0f;
  o->omega = 0.0f;
  o->blend = 0.0f;
  o->lambda_hat = lam;
  o->dual_theta = 0.0f;
  o->flux_only = 0.0f;
  o->valid = false;
  o->ts = (ts > 0.0f) ? ts : 20e-6f;
}

void drive_observer_sync(drive_obs_t *o, const drive_params_t *p,
                         float theta, float omega)
{
  if (o == NULL)
  {
    return;
  }
  /* A HAND-OVER, NOT A COLD START. */
  o->pll_theta = theta;
  o->pll_omega = omega;
  o->flux_theta = theta;
  o->flux_omega = omega;
  o->theta = theta;
  o->omega = omega;
  if (p != NULL)
  {
    const float lam = (p->lambda > 0.0f) ? p->lambda : 1e-6f;

    o->psi_a = lam * cosf(theta);
    o->psi_b = lam * sinf(theta);
    o->lambda_hat = lam;
  }
}

/** The dual model: the voltage integrator, pulled toward the current model
    at `cross`, and a PLL on the rotor flux that comes out. */
static void step_dual(drive_obs_t *o, const drive_params_t *p,
                      float va, float vb, float ia, float ib, float ts)
{
  const float model_a = p->ld * ia + p->lambda * cosf(o->pll_theta);
  const float model_b = p->ld * ib + p->lambda * sinf(o->pll_theta);

  o->psi_a += ts * (va - p->r * ia + o->cross * (model_a - o->psi_a));
  o->psi_b += ts * (vb - p->r * ib + o->cross * (model_b - o->psi_b));

  const float rotor_a = o->psi_a - p->ld * ia;
  const float rotor_b = o->psi_b - p->ld * ib;
  const float size = sqrtf(rotor_a * rotor_a + rotor_b * rotor_b);

  if (size <= 0.0f)
  {
    return;
  }
  /* The PLL's error is the rotor flux across the angle it holds, which is
     sin(difference) and wants no atan2. */
  const float eps = (rotor_b * cosf(o->pll_theta)
                     - rotor_a * sinf(o->pll_theta)) / size;

  o->pll_omega += o->pll_ki * eps * ts;
  o->pll_theta = wrap(o->pll_theta + (o->pll_omega + o->pll_kp * eps) * ts);
  o->dual_theta = o->pll_theta;
}

/** The plain model: a leaking integrator, and the leak's cost put back. */
static void step_flux(drive_obs_t *o, const drive_params_t *p,
                      float va, float vb, float ia, float ib, float ts)
{
  o->leak_a += ts * (va - p->r * ia - o->wc * o->leak_a);
  o->leak_b += ts * (vb - p->r * ib - o->wc * o->leak_b);

  /* THE SPEED COMES FROM THE PLL, not from this model's own angle. */
  const float speed = o->pll_omega;
  const float w = fabsf(speed);
  float gain = 1.0f;
  float lead = 0.0f;

  if (w > 0.0f)
  {
    const float ratio = o->wc / w;

    gain = sqrtf(1.0f + ratio * ratio);
    lead = atan2f(o->wc, w) * ((speed >= 0.0f) ? 1.0f : -1.0f);
  }

  const float cl = cosf(lead);
  const float sl = sinf(lead);
  const float psi_a = gain * (o->leak_a * cl - o->leak_b * sl);
  const float psi_b = gain * (o->leak_a * sl + o->leak_b * cl);
  const float rotor_a = psi_a - p->ld * ia;
  const float rotor_b = psi_b - p->ld * ib;
  const float was = o->flux_theta;

  o->flux_theta = atan2f(rotor_b, rotor_a);
  o->flux_only = o->flux_theta;
  /* THE MAGNITUDE IS LAMBDA, and it is the one thing on this board that can
     see the magnets - the NTC is on the PCB and the rotor is across an air
     gap. */
  o->lambda_hat = sqrtf(rotor_a * rotor_a + rotor_b * rotor_b);

  if (ts > 0.0f)
  {
    const float raw = wrap(o->flux_theta - was) / ts;
    const float a = DRIVE_OBS_SPEED_FILTER * ts;

    o->flux_omega += ((a < 1.0f) ? a : 1.0f) * (raw - o->flux_omega);
  }
}

void drive_observer_step(drive_obs_t *o, const drive_params_t *p,
                         float va, float vb, float ia, float ib, float ts)
{
  if ((o == NULL) || (p == NULL) || (ts <= 0.0f))
  {
    return;
  }

  step_dual(o, p, va, vb, ia, ib, ts);
  step_flux(o, p, va, vb, ia, ib, ts);

  /* The blend, on the dual's own speed: it is the one that is right where
     the hand-over starts, so it is the one that says when to leave. */
  const float w = fabsf(o->pll_omega);
  const float span = o->blend_hi - o->blend_lo;
  float g = (span > 0.0f) ? ((w - o->blend_lo) / span) : 1.0f;

  g = (g < 0.0f) ? 0.0f : ((g > 1.0f) ? 1.0f : g);
  o->blend = g;

  const float x = (1.0f - g) * cosf(o->dual_theta) + g * cosf(o->flux_only);
  const float y = (1.0f - g) * sinf(o->dual_theta) + g * sinf(o->flux_only);

  o->theta = atan2f(y, x);
  o->omega = (1.0f - g) * o->pll_omega + g * o->flux_omega;
  /* Below the leak's own corner neither model has a back-EMF to work with,
     whatever either of them is reporting. */
  o->valid = (w > o->wc);
}
