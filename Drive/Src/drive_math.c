/**
  ******************************************************************************
  * @file    drive_math.c
  * @brief   The frame transforms, the modulator and the dead-time table.
  *
  * Amplitude-invariant Clarke, so a dq ampere is a phase ampere. The three
  * phases go into alpha rather than ia alone: the common mode - an offset
  * drift or a pickup that lands on all three shunts alike - cancels there,
  * and with two phases it would ride straight into the loop.
  ******************************************************************************
  */
#include "drive.h"

#include <math.h>

#define TWO_PI      6.2831853f
#define INV_SQRT3   0.57735027f
#define HALF_SQRT3  0.8660254f


void drive_clarke(const float *iabc, float *alpha, float *beta)
{
  *alpha = (2.0f * iabc[0] - iabc[1] - iabc[2]) / 3.0f;
  *beta  = (iabc[1] - iabc[2]) * INV_SQRT3;
}


void drive_park_cs(float alpha, float beta, float c, float s, float *d,
                   float *q)
{
  *d = alpha * c + beta * s;
  *q = beta * c - alpha * s;
}


void drive_inv_park_cs(float d, float q, float c, float s, float *alpha,
                       float *beta)
{
  *alpha = d * c - q * s;
  *beta  = d * s + q * c;
}


void drive_park(float alpha, float beta, float theta, float *d, float *q)
{
  drive_park_cs(alpha, beta, cosf(theta), sinf(theta), d, q);
}


void drive_inv_park(float d, float q, float theta, float *alpha, float *beta)
{
  drive_inv_park_cs(d, q, cosf(theta), sinf(theta), alpha, beta);
}


float drive_svm(float valpha, float vbeta, float vdc, float *duty)
{
  /* Min-max: the zero sequence that centres the three phase voltages in
     the link, which puts the vector's linear range at Vdc/sqrt3 instead of
     Vdc/2. Past that the VECTOR is scaled back, never a phase clipped on
     its own - clipping one phase bends the vector's angle, and an angle
     error in the applied voltage is an angle error in every estimate
     built on it. */
  float v[DRIVE_PHASES];
  float scale = 1.0f;

  if (vdc <= 0.0f)
  {
    for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
    {
      duty[k] = 0.0f;
    }
    return 0.0f;
  }

  v[0] = valpha;
  v[1] = -0.5f * valpha + HALF_SQRT3 * vbeta;
  v[2] = -0.5f * valpha - HALF_SQRT3 * vbeta;

  float lo = v[0];
  float hi = v[0];

  for (uint8_t k = 1U; k < DRIVE_PHASES; k++)
  {
    lo = (v[k] < lo) ? v[k] : lo;
    hi = (v[k] > hi) ? v[k] : hi;
  }

  if ((hi - lo) > vdc)
  {
    scale = vdc / (hi - lo);
    for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
    {
      v[k] *= scale;
    }
    lo *= scale;
    hi *= scale;
  }

  const float zero = -0.5f * (hi + lo);

  for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
  {
    float d = 0.5f + (v[k] + zero) / vdc;

    d = (d < 0.0f) ? 0.0f : d;
    d = (d > 1.0f) ? 1.0f : d;
    duty[k] = d;
  }
  return scale;
}


float drive_wrap(float theta)
{
  theta -= TWO_PI * floorf(theta / TWO_PI);
  return (theta >= TWO_PI) ? 0.0f : theta;
}


float drive_dt_volts(const drive_params_t *p, float amps)
{
  /* Odd in the current, linear between the points, held past the last:
     what the inverter loses to dead time saturates once the current is
     large against the ripple, and the table is measured out to where it
     does. */
  if (p->dt_step <= 0.0f)
  {
    return 0.0f;
  }

  const float mag = (amps < 0.0f) ? -amps : amps;
  const float pos = mag / p->dt_step;
  const float last = (float)(DRIVE_DT_POINTS - 1U);
  float v;

  if (pos >= last)
  {
    v = p->dt_volts[DRIVE_DT_POINTS - 1U];
  }
  else
  {
    const uint32_t k = (uint32_t)pos;
    const float frac = pos - (float)k;

    v = p->dt_volts[k] + frac * (p->dt_volts[k + 1U] - p->dt_volts[k]);
  }
  return (amps < 0.0f) ? -v : v;
}
