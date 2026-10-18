/** drive_thermal.c - Real-time 3-node Cauer junction observer, WEP envelope, and dynamic PWM. */
#include "drive.h"

#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi)
{
  return fminf(fmaxf(x, lo), hi);
}

void drive_thermal_defaults(drive_thermal_params_t *tp, float ts)
{
  const float f_nom = (ts > 0.0f) ? (1.0f / ts) : 50000.0f;

  memset(tp, 0, sizeof(*tp));
  tp->r_ds0 = 0.0032f;          /* 3.2 mOhm at 25 C */
  tp->alpha_r = 0.0042f;        /* +0.42 %/K silicon tempco */
  tp->t_ref = 25.0f;            /* reference temperature T_0, deg C */
  tp->e_sw = 2.5e-8f;           /* switching loss energy factor, W/(V*A*Hz) */
  tp->r_th[0] = 0.50f;          /* R_jc: junction to case, K/W */
  tp->r_th[1] = 1.20f;          /* R_cs: case to solder joint, K/W */
  tp->r_th[2] = 4.50f;          /* R_sb: solder joint to PCB copper/NTC, K/W */
  tp->c_th[0] = 0.008f;         /* C_j: silicon die thermal capacitance, J/K (~4 ms) */
  tp->c_th[1] = 0.060f;         /* C_c: package slug thermal capacitance, J/K (~72 ms) */
  tp->c_th[2] = 0.800f;         /* C_s: solder + local copper plane, J/K (~3.6 s) */
  tp->tj_nom = 125.0f;          /* continuous MTTF ceiling T_{j,nom}, deg C */
  tp->tj_max = 175.0f;          /* absolute silicon ceiling T_{j,max}, deg C */
  tp->horizon_s = 0.025f;       /* prediction horizon Delta_t = 25 ms */
  tp->f_sw_nom = f_nom;
  tp->f_sw_min = 0.25f * f_nom; /* 25 % of nominal switching frequency floor */
  tp->omega_sw_knee = 150.0f;   /* quasi-static electrical speed knee, rad/s */
}

void drive_thermal_init(drive_t *d, float t_pcb)
{
  const float f_nom = (d->tp.f_sw_nom > 0.0f) ? d->tp.f_sw_nom
                      : ((d->ts > 0.0f) ? (1.0f / d->ts) : 50000.0f);

  memset(&d->th, 0, sizeof(d->th));
  d->th.t_pcb = t_pcb;
  d->th.t_node[0] = t_pcb;
  d->th.t_node[1] = t_pcb;
  d->th.t_node[2] = t_pcb;
  d->th.gamma_wep = 0.0f;
  d->th.tj_hat = t_pcb;
  d->th.tj_limit = d->tp.tj_nom;
  d->th.r_dson = d->tp.r_ds0
                 * fmaxf(1.0f + d->tp.alpha_r * (t_pcb - d->tp.t_ref), 0.2f);
  d->th.f_sw = f_nom;
  d->th.ts_active = 1.0f / fmaxf(f_nom, 1.0f);
  d->th.v_mod_scale = 1.0f;
  d->th.i_max_dyn = 1e6f;
  d->ts = d->th.ts_active;
}

void drive_set_pcb_temp(drive_t *d, float t_ntc)
{
  d->th.t_pcb = clampf(t_ntc, -55.0f, 300.0f);
}

void drive_set_wep(drive_t *d, float gamma_wep)
{
  const float g = clampf(gamma_wep, 0.0f, 1.0f);

  d->th.gamma_wep = g;
  d->th.tj_limit = d->tp.tj_nom + g * (d->tp.tj_max - d->tp.tj_nom);
  d->th.v_mod_scale = 1.0f + g;
}

float drive_current_limit(const drive_t *d)
{
  return fminf(d->p.i_max, d->th.i_max_dyn);
}

float drive_thermal_step(drive_t *d, const float *i_abc, float vdc, float omega_e)
{
  /* Unconditionally stable 3-node Cauer RC backward-Euler solver (Thomas tridiagonal),
     coupled positive-tempco R_DS(on)(\hat{T}_j) + switching loss model, dynamic PWM
     frequency downscaling, and closed-form inverse ceiling prediction I_max(Delta_t). */
  const drive_thermal_params_t *tp = &d->tp;
  drive_thermal_state_t *th = &d->th;
  const float g_wep = clampf(th->gamma_wep, 0.0f, 1.0f);
  const float dt = fmaxf(d->ts, 1e-7f);

  /* 1. Continuous thermal ceiling interpolation: T_{j,limit}(gamma_WEP) */
  const float tj_lim = tp->tj_nom + g_wep * (tp->tj_max - tp->tj_nom);
  th->tj_limit = tj_lim;
  th->v_mod_scale = 1.0f + g_wep;

  /* Worst-case instantaneous phase current magnitude across the 3 legs */
  const float i0 = fabsf(i_abc[0]);
  const float i1 = fabsf(i_abc[1]);
  const float i2 = fabsf(i_abc[2]);
  const float i_abs = fmaxf(fmaxf(i0, i1), i2);
  const float v_link = fmaxf(vdc, 0.0f);

  /* 2. Dynamic PWM frequency downscaling at low |omega_e| and high load / WEP */
  const float f_nom = fmaxf(tp->f_sw_nom, 1000.0f);
  const float f_min = clampf(tp->f_sw_min, 500.0f, f_nom);
  const float w_knee = fmaxf(tp->omega_sw_knee, 1.0f);
  const float eta_omega = 1.0f / (1.0f + fabsf(omega_e) / w_knee);
  const float i_base = fmaxf(d->p.i_max, 1.0f);
  const float load_ratio = clampf(i_abs / i_base, 0.0f, 1.0f);
  const float span_th = fmaxf(tj_lim - th->t_pcb, 1.0f);
  const float th_load = clampf((th->t_node[0] - th->t_pcb) / span_th, 0.0f, 1.0f);
  const float s_down = clampf(eta_omega * load_ratio * fmaxf(g_wep, th_load), 0.0f, 1.0f);
  const float f_sw = f_nom - s_down * (f_nom - f_min);
  const float ts_act = 1.0f / f_sw;

  th->f_sw = f_sw;
  th->ts_active = ts_act;
  d->ts = ts_act;

  /* 3. Coupled loss model: P_loss = I^2 R_DS(on)(\hat{T}_j) + P_sw(Vdc, |I|, f_sw) */
  const float r_dson = fmaxf(tp->r_ds0, 1e-6f)
                       * fmaxf(1.0f + tp->alpha_r * (th->t_node[0] - tp->t_ref), 0.2f);
  const float p_cond = i_abs * i_abs * r_dson;
  const float p_sw = fmaxf(tp->e_sw, 0.0f) * v_link * i_abs * f_sw;
  const float p_loss = p_cond + p_sw;

  th->r_dson = r_dson;
  th->p_loss = p_loss;

  /* 4. Unconditionally stable 3x3 tridiagonal backward-Euler step anchored to T_NTC */
  const float inv_dt = 1.0f / dt;
  const float g01 = 1.0f / fmaxf(tp->r_th[0], 1e-5f);
  const float g12 = 1.0f / fmaxf(tp->r_th[1], 1e-5f);
  const float g2b = 1.0f / fmaxf(tp->r_th[2], 1e-5f);
  const float k0 = fmaxf(tp->c_th[0], 1e-6f) * inv_dt;
  const float k1 = fmaxf(tp->c_th[1], 1e-6f) * inv_dt;
  const float k2 = fmaxf(tp->c_th[2], 1e-6f) * inv_dt;

  const float d0 = k0 + g01;
  const float d1 = k1 + g01 + g12;
  const float d2 = k2 + g12 + g2b;

  const float b0 = k0 * th->t_node[0] + p_loss;
  const float b1 = k1 * th->t_node[1];
  const float b2 = k2 * th->t_node[2] + g2b * th->t_pcb;

  /* Forward elimination */
  const float inv_d0 = 1.0f / d0;
  const float m1 = -g01 * inv_d0;
  const float d1_p = d1 - m1 * (-g01);
  const float inv_d1_p = 1.0f / d1_p;
  const float b1_p = b1 - m1 * b0;

  const float m2 = -g12 * inv_d1_p;
  const float d2_p = d2 - m2 * (-g12);
  const float b2_p = b2 - m2 * b1_p;

  /* Back substitution */
  const float t2_new = b2_p / d2_p;
  const float t1_new = (b1_p + g12 * t2_new) * inv_d1_p;
  const float t0_new = (b0 + g01 * t1_new) * inv_d0;

  th->t_node[0] = t0_new;
  th->t_node[1] = t1_new;
  th->t_node[2] = t2_new;
  th->tj_hat = t0_new;

  /* 5. Inverse ceiling prediction I_max(Delta_t) over horizon_s using Schur complement
        of the 3-node Cauer network and positive-tempco R_DS(on)(T_{j,limit}) */
  const float dt_h = fmaxf(tp->horizon_s, dt);
  const float inv_dt_h = 1.0f / dt_h;
  const float kh0 = fmaxf(tp->c_th[0], 1e-6f) * inv_dt_h;
  const float kh1 = fmaxf(tp->c_th[1], 1e-6f) * inv_dt_h;
  const float kh2 = fmaxf(tp->c_th[2], 1e-6f) * inv_dt_h;

  const float dh2 = kh2 + g12 + g2b;
  const float inv_dh2 = 1.0f / dh2;
  const float g12_dh2 = g12 * inv_dh2;
  const float rh2 = kh2 * t2_new + g2b * th->t_pcb;

  const float dh1 = kh1 + g01 + g12 - g12 * g12_dh2;
  const float inv_dh1 = 1.0f / dh1;
  const float rh1 = kh1 * t1_new + g12_dh2 * rh2;

  const float t1_at_lim = (rh1 + g01 * tj_lim) * inv_dh1;
  const float p_max = fmaxf((kh0 + g01) * tj_lim - kh0 * t0_new - g01 * t1_at_lim, 0.0f);

  const float r_lim = fmaxf(tp->r_ds0, 1e-6f)
                      * fmaxf(1.0f + tp->alpha_r * (tj_lim - tp->t_ref), 0.2f);
  const float b_sw = fmaxf(tp->e_sw, 0.0f) * v_link * f_sw;
  const float disc = b_sw * b_sw + 4.0f * r_lim * p_max;
  const float i_max_dyn = fmaxf((sqrtf(fmaxf(disc, 0.0f)) - b_sw) / (2.0f * r_lim), 0.0f);

  th->i_max_dyn = i_max_dyn;
  return i_max_dyn;
}

