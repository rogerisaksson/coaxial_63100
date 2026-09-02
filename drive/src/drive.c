/**
  ******************************************************************************
  * @file    drive.c
  * @brief   One PWM period of the control law, and the modes it runs in.
  *
  * Timing, which every delay constant here comes from: the sample lands at
  * the top of the triangle, this runs a few microseconds later, the duties
  * it returns are committed at the next underflow and land at the following
  * overflow, so the pulse they shape is the one centred two periods on.
  * The current that shows that pulse is sampled PIPELINE periods after the
  * step that asked for it. The host model in test_drive_core.py carries the
  * same constant, so a demodulator right here is right there.
  *
  * Square-wave injection: the same voltage for `inj_periods` periods, then
  * its negative. inj_periods 1 is fs/2, the highest there is and the least
  * audible; more periods lower the frequency and raise the current the same
  * volts buy (i_h ~ V.T/L), which is the trade the SNR budget makes.
  * Demodulated as sign x (i[k] - i[k-1]) over one whole cycle, so the
  * fundamental's slope cancels: it does not correlate with a sign that
  * sums to zero.
  ******************************************************************************
  */
#include "drive.h"

#include <math.h>
#include <string.h>

/** Periods from the step that asks for a duty to the sample that shows it. */
#define PIPELINE 2U

#define PI_F      3.1415927f
#define TWO_PI_F  6.2831853f
#define INV_SQRT3 0.57735027f


void drive_defaults(drive_params_t *p)
{
  /* PLACEHOLDERS. A small outrunner's order of magnitude so the loop is
     stable on the host model, and nothing else: the commissioning writes
     every one of these over, and the injection is OFF until it does. */
  memset(p, 0, sizeof(*p));
  p->r = 0.05f;
  p->ld = 20e-6f;
  p->lq = 25e-6f;
  p->lambda = 0.005f;
  p->pole_pairs = 7.0f;
  p->kp = 0.1f;
  p->ki = 250.0f;
  p->l1 = 0.1f;
  p->l2 = 100.0f;
  p->inj_volts = 0.0f;
  p->inj_periods = 1U;
  p->inj_phase = 0.0f;
  p->eps_gain = 0.0f;
  p->i_max = 5.0f;
  p->i_trip = 20.0f;
  p->v_frac = 0.95f;
  p->sign = 1.0f;
  p->w_lo = 60.0f;
  p->w_hi = 120.0f;
  p->dt_step = 1.0f;
}


static void loop_reset(drive_t *d)
{
  d->xd = 0.0f;
  d->xq = 0.0f;
  d->vd = 0.0f;
  d->vq = 0.0f;
  d->va_out = 0.0f;
  d->vb_out = 0.0f;
  d->inj_count = 0U;
  d->cyc_count = 0U;
  d->acc_q = 0.0f;
  d->acc_d = 0.0f;
  d->inj_warm = 0U;
  d->inj_valid = false;
  d->have_prev = false;
  d->ih = 0.0f;
  d->fb_fill = 0U;
  memset(d->sign_hist, 0, sizeof(d->sign_hist));
}


void drive_init(drive_t *d, float ts)
{
  memset(d, 0, sizeof(*d));
  d->ts = ts;
  drive_defaults(&d->p);
  drive_model_defaults(&d->model.p);
  drive_model_init(&d->model);
  d->source = DRIVE_SOURCE_ADC;
  d->inj_sign = 1.0f;
  loop_reset(d);
}


void drive_set_theta(drive_t *d, float theta)
{
  d->theta_hat = drive_wrap(theta);
  d->theta_cmd = d->theta_hat;
}


const char *drive_set_mode(drive_t *d, drive_mode_t mode, bool stage_enabled,
                           bool powered)
{
  if (mode >= DRIVE_MODES)
  {
    return "no such mode - 0 off, 1 volt, 2 hold, 3 sensorless, 4 polarity";
  }
  if (mode == DRIVE_OFF)
  {
    d->mode = DRIVE_OFF;
    loop_reset(d);
    return NULL;
  }
  /* The model needs neither a reference nor a stage: it is the currents,
     and the duties go to real gates only if MOE happens to be set. */
  if (d->source == DRIVE_SOURCE_MODEL)
  {
    powered = true;
    stage_enabled = true;
  }
  if (!powered)
  {
    return "AFE_ON is off, and it powers the converter's reference - the "
           "currents this mode would act on are mid-scale, not measurements; "
           "enable the AFE first";
  }
  if (!stage_enabled)
  {
    return "the stage is not armed - gates.arm() is what sets MOE, and a "
           "mode that switches is refused until it has";
  }
  if ((mode == DRIVE_POLARITY) && (d->sp.pol_periods == 0U))
  {
    return "polarity needs pol_periods above zero - one pulse of no length "
           "measures nothing";
  }

  loop_reset(d);
  d->fault = DRIVE_FAULT_NONE;
  if ((mode == DRIVE_HOLD) || (mode == DRIVE_VOLT))
  {
    d->theta_cmd = drive_wrap(d->sp.theta);
    d->omega_cmd = 0.0f;
  }
  if (mode == DRIVE_POLARITY)
  {
    d->pol_step = 0U;
    d->pol_pos = 0.0f;
    d->pol_neg = 0.0f;
  }
  d->mode = mode;
  return NULL;
}


/* ---- the window ------------------------------------------------------- */

static void acc_add(drive_acc_t *a, float x)
{
  a->n++;
  a->sum += (double)x;
  a->sumsq += (double)x * (double)x;
}


static void window_innovation(drive_window_t *w, float e)
{
  /* The lagged products of the innovation, for the whiteness test the
     host runs: a residual that still correlates with itself carries a
     model error, and rho_j = lag[j] / lag[0] is what says so. */
  w->e_ring[w->e_head] = e;
  for (uint8_t j = 0U; j <= DRIVE_LAGS; j++)
  {
    const uint8_t back = (uint8_t)((w->e_head + (DRIVE_LAGS + 1U) - j)
                                   % (DRIVE_LAGS + 1U));

    w->lag[j] += (double)e * (double)w->e_ring[back];
  }
  w->e_head = (uint8_t)((w->e_head + 1U) % (DRIVE_LAGS + 1U));
  acc_add(&w->acc[DRIVE_ACC_EPS], e);
}


void drive_window_take(drive_t *d, drive_window_t *out)
{
  *out = d->win;
  memset(&d->win, 0, sizeof(d->win));
}


/* ---- the moments ------------------------------------------------------ */

void drive_moments_arm(drive_t *d, uint32_t periods)
{
  memset(&d->mom, 0, sizeof(d->mom));
  d->mom.want = periods;
}


void drive_moments_feed(drive_t *d, const int32_t *codes)
{
  drive_moments_t *m = &d->mom;

  if ((m->want == 0U) || (m->n >= m->want))
  {
    return;
  }
  for (uint8_t k = 0U; k < DRIVE_MOMENT_CHANNELS; k++)
  {
    const int32_t c = codes[k];

    if (m->n == 0U)
    {
      m->lo[k] = c;
      m->hi[k] = c;
    }
    m->lo[k] = (c < m->lo[k]) ? c : m->lo[k];
    m->hi[k] = (c > m->hi[k]) ? c : m->hi[k];
    m->sum[k] += (int64_t)c;
    m->sumsq[k] += (uint64_t)((int64_t)c * (int64_t)c);
  }
  m->n++;
}


/* ---- one period ------------------------------------------------------- */

static float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}


/** How much of the angle error comes from the back-EMF, 0..1 by speed.
  * `speed` is the command frame's under I/f - the rotor observer has not found
  * the rotor yet, and a weight on its own estimate never let it start. */
static float bemf_weight(const drive_t *d, float speed)
{
  const float w = fabsf(speed);

  if (d->p.w_hi <= d->p.w_lo)
  {
    return (w > d->p.w_lo) ? 1.0f : 0.0f;
  }
  return clampf((w - d->p.w_lo) / (d->p.w_hi - d->p.w_lo), 0.0f, 1.0f);
}


/** The feedback the loop acts on: the raw dq, or their mean over one
  * injection cycle so the HF ripple does not reach the PI and come back
  * out as an fs/2 voltage the demodulator would read as inductance. */
static void feedback(drive_t *d, float id_raw, float iq_raw, bool injecting,
                     uint16_t n, float *id, float *iq)
{
  d->fb_d[d->fb_head] = id_raw;
  d->fb_q[d->fb_head] = iq_raw;
  d->fb_head = (uint8_t)((d->fb_head + 1U) % DRIVE_FB_RING);
  if (d->fb_fill < DRIVE_FB_RING)
  {
    d->fb_fill++;
  }

  if (!injecting)
  {
    *id = id_raw;
    *iq = iq_raw;
    return;
  }

  const uint16_t span = (uint16_t)(2U * n);
  const uint16_t use = (d->fb_fill < span) ? d->fb_fill : span;
  float sd = 0.0f;
  float sq = 0.0f;

  for (uint16_t k = 0U; k < use; k++)
  {
    const uint8_t at = (uint8_t)((d->fb_head + DRIVE_FB_RING - 1U - k)
                                 % DRIVE_FB_RING);

    sd += d->fb_d[at];
    sq += d->fb_q[at];
  }
  *id = sd / (float)use;
  *iq = sq / (float)use;
}


/** The demodulator. True at the end of a whole injection cycle. `id_raw`
  * and `iq_raw` are the frame's own dq, reused when the injection axis is
  * the d axis - the usual case, and one trig pair fewer. */
static bool demodulate(drive_t *d, float alpha, float beta, float th,
                       float id_raw, float iq_raw, uint16_t n)
{
  float idi = id_raw;
  float iqi = iq_raw;

  if (d->p.inj_phase != 0.0f)
  {
    drive_park(alpha, beta, th + d->p.inj_phase, &idi, &iqi);
  }

  const float s = d->sign_hist[(d->periods - PIPELINE) & 3U];

  /* Every difference counts and the cycles abut: 2n consecutive
     differences see n of each sign whatever the alignment, which is what
     cancels the fundamental. Re-seeding per cycle made the window 2n+1
     against a pattern of 2n, and the two walked - at fs/4 a window could
     hold two of one sign and the slope leaked in as inductance. */
  if (d->have_prev)
  {
    d->acc_q += s * (iqi - d->iq_prev);
    d->acc_d += s * (idi - d->id_prev);
    d->cyc_count++;
  }
  d->iq_prev = iqi;
  d->id_prev = idi;
  d->have_prev = true;

  if (d->cyc_count < (uint16_t)(2U * n))
  {
    return false;
  }

  const float per = 1.0f / (float)(2U * n);

  d->demod_q = d->acc_q * per;
  d->demod_d = d->acc_d * per;
  d->acc_q = 0.0f;
  d->acc_d = 0.0f;
  d->cyc_count = 0U;
  if (d->inj_warm < 2U)
  {
    d->inj_warm++;
  }
  d->inj_valid = (d->inj_warm >= 2U);
  if (d->inj_valid)
  {
    d->ih = fabsf(d->demod_d);
    d->eps_amps = d->demod_q;
    acc_add(&d->win.acc[DRIVE_ACC_IH], d->ih);
  }
  return d->inj_valid;
}


/** The back-EMF's angle error in the rotor observer's frame, or 0 below w_lo.
  * `c`/`s` are cos/sin of theta_hat when the caller has them (the loop
  * frame is the rotor observer's), else NAN and taken here. */
static float bemf_error(drive_t *d, float alpha, float beta, float w,
                        float c, float s)
{
  float idh, iqh, vdh, vqh;

  if (w <= 0.0f)
  {
    return 0.0f;
  }
  if (c != c)                                    /* NAN: not the loop frame */
  {
    drive_sincos(d->theta_hat, &s, &c);
  }
  drive_park_cs(alpha, beta, c, s, &idh, &iqh);
  drive_park_cs(d->va_out, d->vb_out, c, s, &vdh, &vqh);

  const float ed = vdh - d->p.r * idh + d->omega_hat * d->p.lq * iqh;
  const float eq = vqh - d->p.r * iqh - d->omega_hat * d->p.ld * idh;
  const float sg = (d->omega_hat >= 0.0f) ? 1.0f : -1.0f;

  if ((fabsf(ed) + fabsf(eq)) < 1e-6f)
  {
    return 0.0f;
  }
  return atan2f(-ed * sg, eq * sg);
}


static void rotor_observer(drive_t *d, float alpha, float beta, bool injecting,
                           bool cycle_done, float w, float c, float s)
{
  const float e_b = bemf_error(d, alpha, beta, w, c, s);
  bool have = false;
  float e = 0.0f;

  d->e_bemf = e_b;
  if ((d->mode == DRIVE_SENSORLESS) && injecting)
  {
    if (cycle_done && (d->p.eps_gain != 0.0f))
    {
      e = (1.0f - w) * (d->eps_amps / d->p.eps_gain) + w * e_b;
      have = true;
    }
  }
  else if (w > 0.0f)
  {
    e = e_b;
    have = true;
  }

  if (have)
  {
    e = e - TWO_PI_F * floorf((e + PI_F) / TWO_PI_F);   /* to (-pi, pi] */
    d->theta_hat += d->p.l1 * e;
    d->omega_hat += d->p.l2 * e;
    d->eps = e;
    window_innovation(&d->win, e);
  }
  d->theta_hat = drive_wrap(d->theta_hat + d->omega_hat * d->ts);
}


/** The PI with decoupling, and the vector limit with the integrators
  * held when it bites. Returns the demand in the loop frame. */
static void current_loop(drive_t *d, float id, float iq, float vmax,
                         float w_e, float *vd, float *vq)
{
  const float id_ref = clampf(d->sp.id_ref, -d->p.i_max, d->p.i_max);
  const float iq_ref = clampf(d->sp.iq_ref, -d->p.i_max, d->p.i_max);
  const float ed = id_ref - id;
  const float eq = iq_ref - iq;
  const float vd_u = d->p.kp * ed + d->xd - w_e * d->p.lq * iq;
  const float vq_u = d->p.kp * eq + d->xq + w_e * d->p.ld * id
                     + w_e * d->p.lambda;
  const float mag = sqrtf(vd_u * vd_u + vq_u * vq_u);

  if (mag > vmax)
  {
    const float scale = (mag > 0.0f) ? (vmax / mag) : 0.0f;

    *vd = vd_u * scale;
    *vq = vq_u * scale;
    /* Saturated: integrate only what pulls the vector back in. */
    if (ed * vd_u < 0.0f)
    {
      d->xd += d->p.ki * d->ts * ed;
    }
    if (eq * vq_u < 0.0f)
    {
      d->xq += d->p.ki * d->ts * eq;
    }
    return;
  }
  *vd = vd_u;
  *vq = vq_u;
  d->xd += d->p.ki * d->ts * ed;
  d->xq += d->p.ki * d->ts * eq;
}


/** Two pulses along theta_hat, +V then -V, a gap after each; then OFF.
  * Returns the d voltage for this period. */
static float polarity(drive_t *d, float id)
{
  const uint32_t p = d->sp.pol_periods;
  const uint32_t g = d->sp.pol_gap;
  const uint32_t at = d->pol_step++;
  float v = 0.0f;

  if (at < p)
  {
    v = d->sp.pol_volts;
    d->pol_pos = (fabsf(id) > d->pol_pos) ? fabsf(id) : d->pol_pos;
  }
  else if (at < (p + g))
  {
    v = 0.0f;
  }
  else if (at < (2U * p + g))
  {
    v = -d->sp.pol_volts;
    d->pol_neg = (fabsf(id) > d->pol_neg) ? fabsf(id) : d->pol_neg;
  }
  else if (at >= (2U * p + 2U * g))
  {
    d->mode = DRIVE_OFF;
    loop_reset(d);
  }
  return v;
}


static void command_frame(drive_t *d)
{
  if ((d->mode != DRIVE_HOLD) && (d->mode != DRIVE_VOLT))
  {
    return;
  }
  const float step = d->sp.accel * d->ts;
  const float gap = d->sp.omega_target - d->omega_cmd;

  if (fabsf(gap) <= step)
  {
    d->omega_cmd = d->sp.omega_target;
  }
  else
  {
    d->omega_cmd += (gap > 0.0f) ? step : -step;
  }
  d->theta_cmd = drive_wrap(d->theta_cmd + d->omega_cmd * d->ts);
}


bool drive_step(drive_t *d, const drive_sample_t *in, bool stage_enabled,
                drive_out_t *out)
{
  float i[DRIVE_PHASES];
  float alpha, beta, id_raw, iq_raw, id, iq;

  d->periods++;
  d->vdc = in->vdc;
  for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
  {
    i[k] = in->i[k] * d->p.sign;
    out->duty[k] = 0.0f;
  }

  /* The one judgement this makes on its own: a current past the trip it
     was given drops the stage, the same way the thermal ceiling does. */
  if (stage_enabled)
  {
    for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
    {
      if (fabsf(i[k]) > d->p.i_trip)
      {
        d->fault = DRIVE_FAULT_OVERCURRENT;
        d->mode = DRIVE_OFF;
        loop_reset(d);
        return true;
      }
    }
  }
  else if (d->mode != DRIVE_OFF)
  {
    d->fault = DRIVE_FAULT_STAGE;
    d->mode = DRIVE_OFF;
    loop_reset(d);
  }

  drive_clarke(i, &alpha, &beta);

  const bool cmd_frame = (d->mode == DRIVE_HOLD) || (d->mode == DRIVE_VOLT);
  const float th = cmd_frame ? d->theta_cmd : d->theta_hat;
  const float w = bemf_weight(d, cmd_frame ? d->omega_cmd : d->omega_hat);
  uint16_t n = d->p.inj_periods;

  n = (n == 0U) ? 1U : ((n > (DRIVE_FB_RING / 2U)) ? (DRIVE_FB_RING / 2U) : n);

  const float amp = d->p.inj_volts * (1.0f - w);
  const bool injecting = (amp > 0.0f) && (d->mode != DRIVE_OFF)
                         && (d->mode != DRIVE_POLARITY);

  /* One trig pair for the frame; the rotor observer reuses it when the frame is
     its own, and every other transform below is built on it. */
  float c, s;

  drive_sincos(th, &s, &c);
  drive_park_cs(alpha, beta, c, s, &id_raw, &iq_raw);

  bool cycle_done = false;

  if (injecting)
  {
    cycle_done = demodulate(d, alpha, beta, th, id_raw, iq_raw, n);
  }
  else
  {
    d->inj_valid = false;
    d->inj_warm = 0U;
    d->cyc_count = 0U;
    d->have_prev = false;
    d->acc_q = 0.0f;
    d->acc_d = 0.0f;
    d->ih = 0.0f;
  }
  feedback(d, id_raw, iq_raw, injecting, n, &id, &iq);
  d->id = id;
  d->iq = iq;

  rotor_observer(d, alpha, beta, injecting, cycle_done, w,
                 cmd_frame ? NAN : c, cmd_frame ? NAN : s);
  command_frame(d);

  /* the fundamental */
  const float vmax = clampf(d->p.v_frac * in->vdc * INV_SQRT3 - amp,
                            0.0f, 1e9f);
  float vd = 0.0f;
  float vq = 0.0f;

  switch (d->mode)
  {
    case DRIVE_VOLT:
      vd = d->sp.vd;
      vq = d->sp.vq;
      break;
    case DRIVE_HOLD:
      current_loop(d, id, iq, vmax, d->omega_cmd, &vd, &vq);
      break;
    case DRIVE_SENSORLESS:
      current_loop(d, id, iq, vmax, d->omega_hat, &vd, &vq);
      break;
    case DRIVE_POLARITY:
      vd = polarity(d, id_raw);
      break;
    default:
      break;
  }
  d->vd = vd;
  d->vq = vq;

  float va, vb;

  drive_inv_park_cs(vd, vq, c, s, &va, &vb);
  d->va_out = va;
  d->vb_out = vb;

  /* the inverter's error, cancelled before it happens */
  float comp[DRIVE_PHASES];
  float ca, cb;

  for (uint8_t k = 0U; k < DRIVE_PHASES; k++)
  {
    comp[k] = drive_dt_volts(&d->p, i[k]);
  }
  drive_clarke(comp, &ca, &cb);
  va += ca;
  vb += cb;

  /* the injection */
  if (injecting)
  {
    if (d->inj_count == 0U)
    {
      d->inj_sign = -d->inj_sign;
      d->inj_count = n;
    }
    d->inj_count--;
    d->sign_hist[d->periods & 3U] = d->inj_sign;
    if (d->p.inj_phase != 0.0f)
    {
      float si, ci;

      drive_sincos(th + d->p.inj_phase, &si, &ci);
      va += amp * d->inj_sign * ci;
      vb += amp * d->inj_sign * si;
    }
    else
    {
      va += amp * d->inj_sign * c;
      vb += amp * d->inj_sign * s;
    }
  }
  else
  {
    d->sign_hist[d->periods & 3U] = 0.0f;
  }

  if (stage_enabled && (d->mode != DRIVE_OFF))
  {
    (void)drive_svm(va, vb, in->vdc, out->duty);
  }
  else
  {
    d->va_out = 0.0f;
    d->vb_out = 0.0f;
  }

  acc_add(&d->win.acc[DRIVE_ACC_ID], id);
  acc_add(&d->win.acc[DRIVE_ACC_IQ], iq);
  acc_add(&d->win.acc[DRIVE_ACC_VD], vd);
  acc_add(&d->win.acc[DRIVE_ACC_VQ], vq);
  acc_add(&d->win.acc[DRIVE_ACC_VDC], in->vdc);
  d->win.n++;

  const float peak = sqrtf(id_raw * id_raw + iq_raw * iq_raw);

  d->win.i_peak = (peak > d->win.i_peak) ? peak : d->win.i_peak;
  return false;
}
