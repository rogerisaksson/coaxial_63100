/**
  ******************************************************************************
  * @file    drive_observer.c
  * @brief   The back-EMF observer chain: dual flux low, plain flux high.
  *
  * What commutates this board is the injection and the Kalman-form PLL in
  * drive.c. This runs beside it on the same samples and answers the same
  * question a second way, so a bench can compare the two without a shaft
  * sensor and without changing what drives the gates.
  *
  * TWO OBSERVERS, BLENDED, because they fail at opposite ends - measured
  * over plants drawn with the Monte Carlo's own tolerances
  * (notebook_examples/foc_montecarlo.ipynb, *The grip, in quantities the
  * board can measure*), angle error in degrees rms at 63 V:
  *
  *      rad/s el     rpm    dual    flux   chain
  *            20      14     0.5    52.5     0.5
  *           500     341     1.1     5.2     1.1
  *          5000    3410     8.2     3.0     3.0
  *         15000   10231    24.0     7.0     7.0
  *
  * DUAL FLUX is the voltage model `integral of (v - R i)` held down by the
  * current model `L i + lambda` rather than by a leak, with a PLL on the
  * result. Nothing to correct for, so it is exact where the others pay a
  * lag - and its PLL runs out of bandwidth at the top.
  *
  * PLAIN FLUX leaks its integrator at `wc` and puts back what the leak
  * costs: `sqrt(1 + (wc/w)^2)` short and `atan(wc/w)` late. Useless below
  * `wc` and the best of them above, where that correction has shrunk to
  * nothing.
  *
  * The blend rides the observer's OWN speed over DRIVE_OBS_BLEND_LO to
  * _HI, both scaled off `wc` - the one constant that sets where each is
  * good - and is applied to the unit vectors, an angle not being a
  * quantity to average. In the overlap it beats both, because a lag
  * correction and an integrator's leak are not the same error.
  *
  * NEITHER SEES A STANDSTILL. Both live on `v - R i` and a rotor that is
  * not turning makes no back-EMF; `valid` says so, and below that speed
  * the injection is the only thing that knows where the rotor is.
  *
  * Portable C11 like the rest of drive/: no HAL, no board, built by the
  * host suite and driven through ctypes.
  ******************************************************************************
  */
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

  /* The dual model starts on the d axis with the flux it should have, so
     its first steps are a correction rather than a search. */
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
  /* A HAND-OVER, NOT A COLD START. The PLL acquires at `pll_ki`, which
     is 8000 rad/s^2 - a quarter of a second to reach 2000 rad/s and two
     to reach 15 000. Measured: started at rest against a rotor already
     at 2000, the chain read 103 degrees for the whole of a 0.6 s run and
     the flux magnitude inflated 61-fold, because the leak correction
     `sqrt(1 + (wc/w)^2)` divides by the speed it does not have yet.
     Nothing is wrong with the observers; they were asked a question no
     integrator can answer from nothing. The drive already holds an
     estimate, so this is where it hands it over. */
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


/** The dual model: the voltage integrator, pulled toward the current
  * model at `cross`, and a PLL on the rotor flux that comes out. */
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
  /* The PLL's error is the rotor flux across the angle it holds, which
     is sin(difference) and wants no atan2. */
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

  /* THE SPEED COMES FROM THE PLL, not from this model's own angle. The
     correction is atan(wc/w), which at w = 0 is a quarter turn - so an
     observer that starts at rest and reads its own derivative begins 90
     degrees out, and an angle that wrong makes a derivative that cannot
     recover it. Measured: the C sat at 103 degrees from 2000 rad/s up
     while the Python it was ported from, seeded with the truth, held 2.8.
     The dual model is exact where this one hands over, so its PLL is the
     speed to correct against. */
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
  /* THE MAGNITUDE IS LAMBDA, and it is the one thing on this board that
     can see the magnets - the NTC is on the PCB and the rotor is across
     an air gap. It is only lambda where `R i` is small against `v`,
     which is at speed; low down an error in R lands here instead. */
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
  /* Below the leak's own corner neither model has a back-EMF to work
     with, whatever either of them is reporting. */
  o->valid = (w > o->wc);
}
