/**
  ******************************************************************************
  * @file    harness.c
  * @brief   A flat C API over Drive/, so test_drive_core.py can run the
  *          control law on the host through ctypes, against a motor model
  *          written in Python.
  *
  * Built by the Python suite with the host gcc, never by the firmware build.
  * Test scaffolding; it must not appear in the root CMakeLists.
  *
  * Parameters, setpoints and the state cross as flat float arrays in the
  * orders the PARAM_ORDER / SP_ORDER / STATE_ORDER comments give; the Python
  * side names them by the same lists.
  ******************************************************************************
  */
#include "drive.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

/* PARAM_ORDER: r, ld, lq, lambda, pole_pairs, kp, ki, l1, l2, inj_volts,
   inj_periods, inj_phase, eps_gain, i_max, i_trip, v_frac, sign, w_lo,
   w_hi, dt_step, dt_volts[0..7] */
#define PARAMS (20 + DRIVE_DT_POINTS)

/* SP_ORDER: id_ref, iq_ref, theta, omega_target, accel, vd, vq, pol_volts,
   pol_periods, pol_gap */
#define SETPOINTS 10

/* STATE_ORDER: theta_hat, omega_hat, theta_cmd, omega_cmd, id, iq, vd, vq,
   eps, eps_amps, ih, mode, fault, pol_pos, pol_neg, periods, demod_d,
   demod_q, vdc, e_bemf, xd, xq */
#define STATES 22


API drive_t *drv_new(float ts)
{
  drive_t *d = calloc(1U, sizeof(*d));

  if (d != NULL)
  {
    drive_init(d, ts);
  }
  return d;
}


API void drv_free(drive_t *d)
{
  free(d);
}


API int drv_param_count(void)
{
  return PARAMS;
}


API void drv_params_set(drive_t *d, const float *v, int n)
{
  drive_params_t *p = &d->p;

  if (n != PARAMS)
  {
    return;
  }
  p->r = v[0];  p->ld = v[1];  p->lq = v[2];  p->lambda = v[3];
  p->pole_pairs = v[4];
  p->kp = v[5];  p->ki = v[6];  p->l1 = v[7];  p->l2 = v[8];
  p->inj_volts = v[9];
  p->inj_periods = (uint16_t)v[10];
  p->inj_phase = v[11];  p->eps_gain = v[12];
  p->i_max = v[13];  p->i_trip = v[14];  p->v_frac = v[15];  p->sign = v[16];
  p->w_lo = v[17];  p->w_hi = v[18];  p->dt_step = v[19];
  for (uint32_t k = 0U; k < DRIVE_DT_POINTS; k++)
  {
    p->dt_volts[k] = v[20U + k];
  }
}


API void drv_params_get(const drive_t *d, float *v, int n)
{
  const drive_params_t *p = &d->p;

  if (n != PARAMS)
  {
    return;
  }
  v[0] = p->r;  v[1] = p->ld;  v[2] = p->lq;  v[3] = p->lambda;
  v[4] = p->pole_pairs;
  v[5] = p->kp;  v[6] = p->ki;  v[7] = p->l1;  v[8] = p->l2;
  v[9] = p->inj_volts;  v[10] = (float)p->inj_periods;
  v[11] = p->inj_phase;  v[12] = p->eps_gain;
  v[13] = p->i_max;  v[14] = p->i_trip;  v[15] = p->v_frac;  v[16] = p->sign;
  v[17] = p->w_lo;  v[18] = p->w_hi;  v[19] = p->dt_step;
  for (uint32_t k = 0U; k < DRIVE_DT_POINTS; k++)
  {
    v[20U + k] = p->dt_volts[k];
  }
}


API void drv_setpoints_set(drive_t *d, const float *v, int n)
{
  drive_setpoints_t *s = &d->sp;

  if (n != SETPOINTS)
  {
    return;
  }
  s->id_ref = v[0];  s->iq_ref = v[1];  s->theta = v[2];
  s->omega_target = v[3];  s->accel = v[4];
  s->vd = v[5];  s->vq = v[6];  s->pol_volts = v[7];
  s->pol_periods = (uint16_t)v[8];  s->pol_gap = (uint16_t)v[9];
}


API const char *drv_set_mode(drive_t *d, int mode, int enabled, int powered)
{
  const char *why = drive_set_mode(d, (drive_mode_t)mode, enabled != 0,
                                   powered != 0);

  return (why == NULL) ? "" : why;
}


API void drv_set_theta(drive_t *d, float theta)
{
  drive_set_theta(d, theta);
}


API int drv_step(drive_t *d, const float *i3, float vdc, int enabled,
                 float *duty3)
{
  drive_sample_t in;
  drive_out_t out;

  in.i[0] = i3[0];
  in.i[1] = i3[1];
  in.i[2] = i3[2];
  in.vdc = vdc;

  const bool trip = drive_step(d, &in, enabled != 0, &out);

  duty3[0] = out.duty[0];
  duty3[1] = out.duty[1];
  duty3[2] = out.duty[2];
  return trip ? 1 : 0;
}


API int drv_state_count(void)
{
  return STATES;
}


API void drv_state(const drive_t *d, float *v, int n)
{
  if (n != STATES)
  {
    return;
  }
  v[0] = d->theta_hat;  v[1] = d->omega_hat;
  v[2] = d->theta_cmd;  v[3] = d->omega_cmd;
  v[4] = d->id;  v[5] = d->iq;  v[6] = d->vd;  v[7] = d->vq;
  v[8] = d->eps;  v[9] = d->eps_amps;  v[10] = d->ih;
  v[11] = (float)d->mode;  v[12] = (float)d->fault;
  v[13] = d->pol_pos;  v[14] = d->pol_neg;
  v[15] = (float)d->periods;
  v[16] = d->demod_d;  v[17] = d->demod_q;  v[18] = d->vdc;
  v[19] = d->e_bemf;  v[20] = d->xd;  v[21] = d->xq;
}


/* WINDOW_ORDER: n, then per field (n, sum, sumsq) for id, iq, vd, vq, eps,
   ih, vdc, then lag[0..7], then i_peak. Takes and resets. */
#define WINDOW_DOUBLES (1 + 3 * DRIVE_ACC_FIELDS + (DRIVE_LAGS + 1) + 1)


API int drv_window_count(void)
{
  return WINDOW_DOUBLES;
}


API void drv_window(drive_t *d, double *v, int n)
{
  drive_window_t w;
  int at = 0;

  if (n != WINDOW_DOUBLES)
  {
    return;
  }
  drive_window_take(d, &w);
  v[at++] = (double)w.n;
  for (uint32_t f = 0U; f < (uint32_t)DRIVE_ACC_FIELDS; f++)
  {
    v[at++] = (double)w.acc[f].n;
    v[at++] = w.acc[f].sum;
    v[at++] = w.acc[f].sumsq;
  }
  for (uint32_t j = 0U; j <= DRIVE_LAGS; j++)
  {
    v[at++] = w.lag[j];
  }
  v[at++] = (double)w.i_peak;
}


API void drv_moments_arm(drive_t *d, unsigned periods)
{
  drive_moments_arm(d, periods);
}


API void drv_moments_feed(drive_t *d, const int *codes4)
{
  int32_t c[DRIVE_MOMENT_CHANNELS];

  for (uint32_t k = 0U; k < DRIVE_MOMENT_CHANNELS; k++)
  {
    c[k] = (int32_t)codes4[k];
  }
  drive_moments_feed(d, c);
}


/* MOMENTS_ORDER: n, want, then per channel sum, sumsq, lo, hi. */
#define MOMENT_DOUBLES (2 + 4 * DRIVE_MOMENT_CHANNELS)


API int drv_moments_count(void)
{
  return MOMENT_DOUBLES;
}


API void drv_moments(const drive_t *d, double *v, int n)
{
  int at = 0;

  if (n != MOMENT_DOUBLES)
  {
    return;
  }
  v[at++] = (double)d->mom.n;
  v[at++] = (double)d->mom.want;
  for (uint32_t k = 0U; k < DRIVE_MOMENT_CHANNELS; k++)
  {
    v[at++] = (double)d->mom.sum[k];
    v[at++] = (double)d->mom.sumsq[k];
    v[at++] = (double)d->mom.lo[k];
    v[at++] = (double)d->mom.hi[k];
  }
}


/* ---- the model as the source ------------------------------------------ */

/* MODEL_ORDER: r, ld, lq, lambda, pole_pairs, sat, i_sat, j, b, load, v_dt,
   i_knee, vdc, noise, theta0, sub */
#define MODEL_PARAMS 16


API int drv_model_param_count(void)
{
  return MODEL_PARAMS;
}


API void drv_model_params_set(drive_t *d, const float *v, int n)
{
  drive_model_params_t *p = &d->model.p;

  if (n != MODEL_PARAMS)
  {
    return;
  }
  p->r = v[0];  p->ld = v[1];  p->lq = v[2];  p->lambda = v[3];
  p->pole_pairs = v[4];  p->sat = v[5];  p->i_sat = v[6];
  p->j = v[7];  p->b = v[8];  p->load = v[9];  p->v_dt = v[10];
  p->i_knee = v[11];  p->vdc = v[12];  p->noise = v[13];  p->theta0 = v[14];
  p->sub = (uint8_t)v[15];
}


API void drv_source(drive_t *d, int model)
{
  d->source = model ? DRIVE_SOURCE_MODEL : DRIVE_SOURCE_ADC;
  if (model)
  {
    drive_model_init(&d->model);
  }
}


/* MODEL_STATE_ORDER: theta, omega, id, iq */
API void drv_model_state(const drive_t *d, float *v)
{
  v[0] = d->model.theta;
  v[1] = d->model.omega;
  v[2] = d->model.id;
  v[3] = d->model.iq;
}


API int drv_step_virtual(drive_t *d, float *duty3)
{
  drive_out_t out;
  const bool trip = drive_step_virtual(d, &out);

  duty3[0] = out.duty[0];
  duty3[1] = out.duty[1];
  duty3[2] = out.duty[2];
  return trip ? 1 : 0;
}


/* ---- the arithmetic on its own ---------------------------------------- */

API float drv_svm(float va, float vb, float vdc, float *duty3)
{
  return drive_svm(va, vb, vdc, duty3);
}


API void drv_clarke(const float *iabc, float *ab)
{
  drive_clarke(iabc, &ab[0], &ab[1]);
}


API void drv_park(float alpha, float beta, float theta, float *dq)
{
  drive_park(alpha, beta, theta, &dq[0], &dq[1]);
}


API void drv_inv_park(float dd, float q, float theta, float *ab)
{
  drive_inv_park(dd, q, theta, &ab[0], &ab[1]);
}


API float drv_wrap(float theta)
{
  return drive_wrap(theta);
}


API float drv_dt_volts(const drive_t *d, float amps)
{
  return drive_dt_volts(&d->p, amps);
}
