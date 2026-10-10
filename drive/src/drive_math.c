/** drive_math.c - Frame transforms, modulator, dead-time table. */
#include "drive.h"

#include <math.h>

#define TWO_PI      6.2831853f
#define PI          3.1415927f
#define HALF_PI     1.5707963f
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

void drive_sincos(float theta, float *s, float *c)
{
  /* A ninth-order odd polynomial on [-pi/2, pi/2] after folding, 3e-6 at the
     worst point - a control law needs milliradians. */
  float x = theta - TWO_PI * floorf((theta + PI) / TWO_PI);   /* (-pi, pi] */
  float xs = x;
  float xc = HALF_PI - x;                       /* cos x = sin(pi/2 - x) */

  if (xs > HALF_PI)
  {
    xs = PI - xs;
  }
  else if (xs < -HALF_PI)
  {
    xs = -PI - xs;
  }
  if (xc > HALF_PI)
  {
    xc = PI - xc;
  }
  else if (xc < -HALF_PI)
  {
    xc = -PI - xc;
  }

  const float s2 = xs * xs;
  const float c2 = xc * xc;

  *s = xs * (1.0f + s2 * (-1.6666667e-1f + s2 * (8.3333333e-3f
             + s2 * (-1.9841270e-4f + s2 * 2.7557319e-6f))));
  *c = xc * (1.0f + c2 * (-1.6666667e-1f + c2 * (8.3333333e-3f
             + c2 * (-1.9841270e-4f + c2 * 2.7557319e-6f))));
}

float drive_tanh(float x)
{
  /* 13/6 odd over even past the clamp where a float's tanh is 1: libm's tanhf took expm1f
     and its errno path from the plant twice a step (2026-10-10). */
  const float c = fminf(fmaxf(x, -7.90531110763549805f), 7.90531110763549805f);
  const float c2 = c * c;
  const float p = c * (4.89352455891786e-03f + c2 * (6.37261928875436e-04f
                  + c2 * (1.48572235717979e-05f + c2 * (5.12229709037114e-08f
                  + c2 * (-8.60467152213735e-11f + c2 * (2.00018790482477e-13f
                  + c2 * -2.76076847742355e-16f))))));
  const float q = 4.89352518554385e-03f + c2 * (2.26843463243900e-03f
                  + c2 * (1.18534705686654e-04f + c2 * 1.19825839466702e-06f));

  return p / q;
}

float drive_atan2(float y, float x)
{
  /* An eleventh-order odd polynomial on [0, 1] after folding into the octant,
     1.7e-6 at the worst point: the library's was called four times a period
     (2026-09-27). */
  const float ay = fabsf(y);
  const float ax = fabsf(x);

  if ((ax == 0.0f) && (ay == 0.0f))
  {
    return 0.0f;
  }

  const float t = (ay > ax) ? (ax / ay) : (ay / ax);
  const float t2 = t * t;
  float r = t * (9.9997726e-1f + t2 * (-3.3262347e-1f + t2 * (1.9354346e-1f
            + t2 * (-1.1643287e-1f + t2 * (5.265332e-2f + t2 * -1.172120e-2f)))));

  if (ay > ax)
  {
    r = HALF_PI - r;
  }
  if (x < 0.0f)
  {
    r = PI - r;
  }
  return (y < 0.0f) ? -r : r;
}

void drive_park(float alpha, float beta, float theta, float *d, float *q)
{
  float s, c;

  drive_sincos(theta, &s, &c);
  drive_park_cs(alpha, beta, c, s, d, q);
}

void drive_inv_park(float d, float q, float theta, float *alpha, float *beta)
{
  float s, c;

  drive_sincos(theta, &s, &c);
  drive_inv_park_cs(d, q, c, s, alpha, beta);
}

float drive_svm(float valpha, float vbeta, float vdc, float *duty)
{
  /* Min-max: the zero sequence that centres the three phase voltages in the
     link, which puts the vector's linear range at Vdc/sqrt3 instead of
     Vdc/2. */
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
  /* The two compares first: a step moves an angle by a fraction of a turn,
     and floorf is a library call. */
  if (theta >= 0.0f && theta < TWO_PI)
  {
    return theta;
  }
  if (theta >= TWO_PI && theta < 2.0f * TWO_PI)
  {
    return theta - TWO_PI;
  }
  if (theta < 0.0f && theta >= -TWO_PI)
  {
    return (theta + TWO_PI >= TWO_PI) ? 0.0f : theta + TWO_PI;
  }
  theta -= TWO_PI * floorf(theta / TWO_PI);
  return (theta >= TWO_PI) ? 0.0f : theta;
}

float drive_dt_volts(const drive_params_t *p, float amps)
{
  /* Odd in the current, linear between the points, held past the last: what
     the inverter loses to dead time saturates once the current is large
     against the ripple, and the table is measured out to where it does. */
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
