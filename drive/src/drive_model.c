/**
  ******************************************************************************
  * @file    drive_model.c
  * @brief   A PMSM and an inverter in front of it, for the drive to run
  *          against when the converters cannot answer.
  *
  * The second sample source. On the bench board AFE_ON high unpowers the
  * gate drivers, so the real currents and a switching stage are never
  * available together; this is how the rotor observer is watched working -
  * predictably, against a rotor whose angle is known - while the half
  * bridges switch dry or not at all.
  *
  * The same model test_drive_core.py integrates in Python, in the same
  * form: dq in the rotor frame, Ld bent by the d current (the saturation
  * saliency an SPM shows), friction and a load on the shaft, the inverter's
  * dead-time volts odd in each phase current, and the two-period pipeline
  * the firmware has between a duty asked for and the sample that shows it.
  * Sub-stepped Euler with one trig pair per period: the electrical time
  * constant is hundreds of microseconds against a 5 us sub-step, and a
  * period turns the rotor milliradians.
  ******************************************************************************
  */
#include "drive.h"

#include <math.h>
#include <string.h>

#define TWO_PI_F    6.2831853f
#define HALF_SQRT3  0.8660254f
#define INV_SQRT3   0.57735027f


void drive_model_defaults(drive_model_params_t *p)
{
  /* The order of magnitude of a small outrunner, the same set
     drive_defaults carries so a virtual run is consistent out of the box.
     Placeholders like those: a real motor's numbers come from the
     commissioning, and a model worth comparing against carries them. */
  memset(p, 0, sizeof(*p));
  p->r = 0.05f;
  p->ld = 20e-6f;
  p->lq = 25e-6f;
  p->lambda = 0.005f;
  p->pole_pairs = 7.0f;
  p->sat = 0.0f;
  p->i_sat = 5.0f;
  p->j = 2e-5f;
  p->b = 1e-5f;
  p->load = 0.0f;
  p->v_dt = 0.0f;
  p->i_knee = 0.3f;
  p->vdc = 24.0f;
  p->noise = 0.0f;
  p->theta0 = 0.0f;
  p->sub = 4U;
}


void drive_model_init(drive_model_t *m)
{
  m->theta = drive_wrap(m->p.theta0);
  m->omega = 0.0f;
  m->id = 0.0f;
  m->iq = 0.0f;
  m->rng = 0x2545F491UL;
  memset(m->duty_prev, 0, sizeof(m->duty_prev));
  m->c = 1.0f;
  m->s = 0.0f;
  memset(m->i_abc, 0, sizeof(m->i_abc));
}


/** Roughly Gaussian, cheap: three uniforms from an LCG, centred. */
static float model_noise(drive_model_t *m, float sd)
{
  if (sd <= 0.0f)
  {
    return 0.0f;
  }
  float sum = 0.0f;

  for (uint8_t k = 0U; k < 3U; k++)
  {
    m->rng = m->rng * 1664525U + 1013904223U;   /* U: the LCG wraps in
                                       uint32_t on every target; UL made an
                                       LP64 host widen the product to 64
                                       bits before the same wrap */
    sum += (float)(m->rng >> 8) / 16777216.0f - 0.5f;
  }
  return sum * 2.0f * sd;                 /* three uniforms: sd is 0.5 */
}


static float model_ld(const drive_model_t *m)
{
  return m->p.ld * (1.0f - m->p.sat * tanhf(m->id / m->p.i_sat));
}


void drive_model_sample(drive_model_t *m, drive_sample_t *out)
{
  /* The three phase currents as the shunts would report them, at the top
     of the period - amplitude-invariant, with the noise the caller asked
     for on each. The trig pair is kept for the advance that follows:
     a period turns the rotor milliradians. */
  drive_sincos(m->theta, &m->s, &m->c);
  const float ia = m->id * m->c - m->iq * m->s;
  const float ib = m->id * m->s + m->iq * m->c;

  m->i_abc[0] = ia;
  m->i_abc[1] = -0.5f * ia + HALF_SQRT3 * ib;
  m->i_abc[2] = -0.5f * ia - HALF_SQRT3 * ib;
  for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
  {
    out->i[k] = m->i_abc[k] + model_noise(m, m->p.noise);
  }
  out->vdc = m->p.vdc;
}


void drive_model_advance(drive_model_t *m, const float *duty, float ts)
{
  /* One period at these duties: the average-voltage inverter with its
     neutral floating, the dead-time error taken off each phase by the sign
     of its current, then the machine in its own frame. */
  const float mean = (duty[0] + duty[1] + duty[2]) / 3.0f;
  float v[DRIVE_PHASES];

  for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
  {
    v[k] = m->p.vdc * (duty[k] - mean);
  }
  if (m->p.v_dt > 0.0f)
  {
    for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
    {
      v[k] -= m->p.v_dt * tanhf(m->i_abc[k] / m->p.i_knee);
    }
  }

  const float va = (2.0f * v[0] - v[1] - v[2]) / 3.0f;
  const float vb = (v[1] - v[2]) * INV_SQRT3;
  const float c = m->c;
  const float s = m->s;
  const float vd = va * c + vb * s;
  const float vq = vb * c - va * s;
  const uint8_t sub = (m->p.sub == 0U) ? 1U : m->p.sub;
  const float dt = ts / (float)sub;
  const float ld = model_ld(m);          /* once: it bends with the period's current */

  for (uint8_t k = 0U; k < sub; k++)
  {
    const float did = (vd - m->p.r * m->id + m->omega * m->p.lq * m->iq) / ld;
    const float diq = (vq - m->p.r * m->iq - m->omega * ld * m->id
                       - m->omega * m->p.lambda) / m->p.lq;

    m->id += did * dt;
    m->iq += diq * dt;

    const float torque = 1.5f * m->p.pole_pairs
                         * (m->p.lambda * m->iq + (ld - m->p.lq) * m->id * m->iq);
    float wm = m->omega / m->p.pole_pairs;

    wm += (torque - m->p.b * wm - m->p.load) / m->p.j * dt;
    m->omega = wm * m->p.pole_pairs;
    m->theta += m->omega * dt;
  }
  m->theta = drive_wrap(m->theta);
}


bool drive_step_virtual(drive_t *d, drive_out_t *out)
{
  /* Sample, step, advance with the duty from the step BEFORE - the
     firmware's pipeline: what this step asks for shapes the period after
     the next one, so the model sees it a period late, like the stage. */
  drive_sample_t in;
  const uint32_t t0 = d->cycles ? d->cycles() : 0U;

  drive_model_sample(&d->model, &in);

  const uint32_t t1 = d->cycles ? d->cycles() : 0U;
  const bool trip = drive_step(d, &in, true, out);
  const uint32_t t2 = d->cycles ? d->cycles() : 0U;

  drive_model_advance(&d->model, d->model.duty_prev, d->ts);
  memcpy(d->model.duty_prev, out->duty, sizeof(d->model.duty_prev));
  if (d->cycles)
  {
    /* Where the period goes, so an interrupt that outgrew it can be read
       block by block rather than guessed at. */
    d->cyc_sample = t1 - t0;
    d->cyc_step = t2 - t1;
    d->cyc_advance = d->cycles() - t2;
  }
  return trip;
}
