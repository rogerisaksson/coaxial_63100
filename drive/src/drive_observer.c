/** drive_observer.c - Back-EMF observers: dual flux low, plain flux high. */
#include "drive.h"

#include <math.h>
#include <stddef.h>

/** Wrap to [-pi, pi]. */
static float wrap(float a)
{
  a -= 6.2831853f * floorf((a + 3.1415927f) * 0.15915494f);
  return a - 6.2831853f * floorf((a + 3.1415927f) * 0.15915494f);
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
  /* A hand-over, not a cold start. */
  o->pll_theta = theta;
  o->pll_omega = omega;
  o->flux_theta = theta;
  o->flux_omega = omega;
  o->theta = theta;
  o->omega = omega;
  if (p != NULL)
  {
    const float lam = (p->lambda > 0.0f) ? p->lambda : 1e-6f;
    float sn, cs;

    drive_sincos(theta, &sn, &cs);
    o->psi_a = lam * cs;
    o->psi_b = lam * sn;
    o->lambda_hat = lam;
  }
  o->valid = (fabsf(omega) > o->wc);
}

/** The dual model: the voltage integrator, pulled toward the current model
    at `cross`, and a PLL on the rotor flux that comes out. */
static void step_dual(drive_obs_t *o, const drive_params_t *p,
                      float va, float vb, float ia, float ib, float ts)
{
  float sp, cp;

  drive_sincos(o->pll_theta, &sp, &cp);

  const float model_a = p->ld * ia + p->lambda * cp;
  const float model_b = p->ld * ib + p->lambda * sp;

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
  const float eps = (rotor_b * cp - rotor_a * sp) / size;

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

  /* The speed comes from the PLL, or the flux model's own filtered speed
     when already valid, whichever is higher in magnitude, so a fast spool
     after handover transitions to the flux model. Multiplying by
     (1 + j wc / speed) undoes both the leak's attenuation and its phase lag
     without a square root, an arctangent or a sine. */
  const float speed = (o->valid && (fabsf(o->flux_omega) > fabsf(o->pll_omega))) ? o->flux_omega : o->pll_omega;
  const float k = (speed != 0.0f) ? (o->wc / speed) : 0.0f;
  const float psi_a = o->leak_a - k * o->leak_b;
  const float psi_b = o->leak_b + k * o->leak_a;
  const float rotor_a = psi_a - p->ld * ia;
  const float rotor_b = psi_b - p->ld * ib;
  const float was = o->flux_theta;

  o->flux_theta = drive_atan2(rotor_b, rotor_a);
  o->flux_only = o->flux_theta;
  /* The magnitude is lambda, and it is the one thing on this board that can
     see the magnets - the NTC is on the PCB and the rotor is across an air
     gap. */
  o->lambda_hat = sqrtf(rotor_a * rotor_a + rotor_b * rotor_b);

  if (ts > 0.0f)
  {
    const float raw = wrap(o->flux_theta - was) / ts;
    const float a = DRIVE_OBS_SPEED_FILTER * ts;

    o->flux_omega += fminf(a, 1.0f) * (raw - o->flux_omega);
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

  /* The blend, on the dual's own speed, or the higher flux speed when
     already valid: the dual is right where the handover starts, but on a
     fast spool the flux model takes over when the PLL slips behind. */
  const float w = o->valid ? fmaxf(fabsf(o->pll_omega), fabsf(o->flux_omega)) : fabsf(o->pll_omega);
  const float span = o->blend_hi - o->blend_lo;
  float g = (span > 0.0f) ? ((w - o->blend_lo) / span) : 1.0f;

  g = fminf(fmaxf(g, 0.0f), 1.0f);
  o->blend = g;

  float sd, cd, sf, cf;

  drive_sincos(o->dual_theta, &sd, &cd);
  drive_sincos(o->flux_only, &sf, &cf);

  const float x = (1.0f - g) * cd + g * cf;
  const float y = (1.0f - g) * sd + g * sf;

  o->theta = drive_atan2(y, x);
  o->omega = (1.0f - g) * o->pll_omega + g * o->flux_omega;
  /* Below the leak's own corner neither model has a back-EMF to work with,
     whatever either of them is reporting. */
  o->valid = (w > o->wc);
}
