/** wbc_twist.c - her twist on the loop (wbc_imp.h): machine.law.se2.twisted on her law's arrays - the
    floor under her, her heading, her rhythm flipped, her gait modulated, her look - numpy's and
    Python's arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

#define PI  3.14159265358979323846
#define TAU 6.28318530717958647692

/** Python's math.hypot of two; its float %, the divisor's sign; machine.law.se2.wrap_angle. */
static double hypot2(double a, double b)
{
  const double v[2] = {a, b};

  return pyhypot(v, 2);
}

static double pymod(double x, double w)
{
  double mod = fmod(x, w);

  if (mod != 0.0)
  {
    if ((w < 0.0) != (mod < 0.0))
    {
      mod += w;
    }
  }
  else
  {
    mod = copysign(0.0, w);
  }
  return mod;
}

static double wrap(double angle)
{
  return atan2(sin(angle), cos(angle));
}

/** machine.law.se2.floored: the floor under her bearing soles' lowest corners, where it is along her
    way, its pitch under each sole. */
static void floored(const wbc_imp_t *k, const wbc_twist_t *t, double dt)
{
  const double *y = t->y;
  double       *fr = t->fr;
  double        over[2], bear[2], total, kf, under, at[2], c_min[2], c_over[2], flat[2];

  for (int s = 0; s < 2; s++)
  {
    over[s] = npmaximum(y[k->y_load + s] - t->touch_n, 0.0);
    bear[s] = y[k->y_load + s] * (over[s] / (over[s] + 1.0));
  }
  total = npsum(bear, 2);
  kf = (total / (total + 1e-9)) * npmin(1.0, t->floor_hz * dt);
  under = wbc_dot(&k->blas, bear, 1, y + k->y_lowest, 1, 2) / npmax(total, 1e-9);
  fr[k->f_floor] += kf * (under - fr[k->f_floor]);
  /* the soles' x and z picked out of y are F-ordered to numpy: its (2, 2) @ (2,) form, dgemv_t's
     tail contracted (npmv2) */
  for (int j = 0; j < 2; j++)
  {
    at[j] = (0.0 + fma(bear[0], y[k->y_sole + 2 * j], bear[1] * y[k->y_sole + 3 + 2 * j])) / npmax(total, 1e-9);
    fr[k->f_floor_at + j] += kf * (at[j] - fr[k->f_floor_at + j]);
  }
  for (int s = 0; s < 2; s++)
  {
    const double *c = y + t->y_cells + 2 * s;

    c_min[s] = npminimum(c[0], c[1]);
    c_over[s] = npmaximum(c_min[s] - t->touch_n, 0.0);
    flat[s] = ((c_over[s] * c_over[s] / (c_over[s] * c_over[s] + 1.0)) * c_min[s]) / npmaximum(npmaximum(c[0], c[1]), 1e-9);
    fr[k->f_ground + s] += (npmin(1.0, t->floor_hz * dt) * flat[s]) * (y[k->y_tilt + s] - fr[k->f_ground + s]);
  }
}

/** machine.law.rhythm.modulated: her gait's rhythm, drive, turn and arms under a twist told. */
static void modulate(const wbc_imp_t *k, const wbc_twist_t *t, double vx, double vy, double wz, double vxp,
                     const double *held, const double *prior)
{
  double       *st = t->st, *out = t->out, *state = t->state;
  const double  tw_on = t->words[t->w_active];
  const double  gain = st[t->st_now_gain], amp_d = fabs(st[t->st_to_amp + k->drive]);
  const double  w_gait = npmin(1.0, npmax(0.0, (npmax(npmax(amp_d, fabs(vy)), fabs(wz)) - t->moving) / t->moving_edge));
  const double  w_on = (tw_on * npmin(1.0, npmax(0.0, (gain - t->gain_on) / t->gain_edge))) * w_gait;
  const double  w_rest = (tw_on * (1.0 - npmin(1.0, npmax(0.0, (gain - t->gain_on) / t->gain_edge))))
                         * (double)(t->rest_lo < amp_d && amp_d < t->rest_hi);
  const double  w_fast_fwd = (npmin(1.0, npmax(0.0, (vx - t->walk_m_s) / t->edge_m_s))
                              * npmin(1.0, npmax(0.0, (t->still_rad_s - fabs(wz)) / t->edge_rad_s)))
                             * npmin(1.0, npmax(0.0, (t->edge_m_s - fabs(vy)) / t->edge_m_s));
  double        w_vy, w_wz, w_wz5, w_spin, wz_way, ts_raw, ts, w_turn, bt_d, w, w_vxp, drv, amp_t, bt, w_slip, w_vx_hi, w_vy_hi;
  double        w_back_hi, w_wz_hi, w_hz, hz_m, w_boost_fwd, w_boost_back, w_short, sw;

  st[t->st_to_amp + k->drive] += (w_rest * w_fast_fwd) * npmax(0.0, t->rest_drive - st[t->st_to_amp + k->drive]);
  w_vy = npmin(1.0, npmax(0.0, (fabs(vy) - t->side_m_s) / t->edge_m_s));
  w_wz = npmin(1.0, npmax(0.0, (fabs(wz) - t->turn_rad_s) / t->turn_edge_rad_s));
  w_wz5 = npmin(1.0, npmax(0.0, (fabs(wz) - t->spin_rad_s) / t->edge_rad_s));
  w_spin = ((w_on * w_wz5) * npmin(1.0, npmax(0.0, (t->spin_still_m_s - fabs(vx)) / t->edge_m_s))) * (1.0 - w_vy);
  wz_way = (1.0 - 2.0 * (double)(vx < -t->backing_m_s)) * wz;
  ts_raw = ((w_on * w_wz) * (t->turn_dc - t->turn_dc_side * w_vy)) * npmax(-1.0, npmin(1.0, wz_way / t->yaw_rate));
  ts = state[t->s_turn_dc] + t->turn_blend * (ts_raw - state[t->s_turn_dc]);
  state[t->s_turn_dc] = ts;
  w_turn = w_on * npmax(w_wz, npmin(1.0, npmax(0.0, (fabs(ts) - t->turning) / t->turning_edge)));
  bt_d = st[t->st_theta] + st[t->st_now_phase + k->drive];
  w = out[k->o_turns];
  w_vxp = npmin(1.0, npmax(0.0, (fabs(vxp) - t->spin_fast_m_s) / t->spin_fast_edge_m_s));
  drv = (1.0 - w_vxp) * t->spin_drive + w_vxp * npmax(t->spin_drive, t->spin_drive_s_m * (fabs(vxp) - t->spin_drive_from_m_s));
  /* told on slower than her walk, past the spot's stillness: her stride her speed's share of her walk's */
  w_short = (w_on * (1.0 - npmin(1.0, npmax(0.0, (t->spin_still_m_s - vx) / t->edge_m_s)))) * npmax(0.0, 1.0 - vx / t->walk_m_s);
  sw = out[k->o_sway + k->drive];
  out[k->o_ref + k->drive] -= (w_short * sw) * sin(bt_d);
  out[k->o_rate + k->drive] -= ((w_short * sw) * w) * cos(bt_d);
  out[k->o_acc + k->drive] += (((w_short * sw) * w) * w) * sin(bt_d);
  out[k->o_sway + k->drive] -= w_short * sw;
  out[k->o_sway + k->drive] += w_spin * (drv - out[k->o_sway + k->drive]);
  out[k->o_ref + k->drive] += w_spin * (drv * sin(bt_d) - out[k->o_ref + k->drive]);
  out[k->o_rate + k->drive] += w_spin * ((drv * w) * cos(bt_d) - out[k->o_rate + k->drive]);
  out[k->o_acc + k->drive] += w_spin * ((((-drv) * w) * w) * sin(bt_d) - out[k->o_acc + k->drive]);
  out[k->o_sway + k->lift] += w_spin * npmax(0.0, t->spin_lift - out[k->o_sway + k->lift]);
  amp_t = w_vy * fabs(ts) + (1.0 - w_vy) * t->turn_sway;
  bt = st[t->st_theta] + st[t->st_now_phase + k->turn];
  out[k->o_sway + k->turn] += w_turn * (amp_t - out[k->o_sway + k->turn]);
  out[k->o_ref + k->turn] += w_turn * ((ts + amp_t * sin(bt)) - out[k->o_ref + k->turn]);
  out[k->o_rate + k->turn] += w_turn * ((amp_t * w) * cos(bt) - out[k->o_rate + k->turn]);
  out[k->o_acc + k->turn] += w_turn * ((((-amp_t) * w) * w) * sin(bt) - out[k->o_acc + k->turn]);
  w_slip = (npmin(1.0, npmax(0.0, (t->slip_grip - npmin(held[0], held[1])) / t->slip_grip_edge))
            * npmin(1.0, npmax(0.0, (npmin(prior[0], prior[1]) - t->slip_grip) / t->slip_grip_edge)))
           * npmin(1.0, npmax(0.0, (fabs(vx) - t->slip_m_s) / t->edge_m_s));
  w_vx_hi = npmin(1.0, npmax(0.0, (fabs(vx) - t->walk_m_s) / t->edge_m_s));
  w_vy_hi = npmin(1.0, npmax(0.0, (fabs(vy) - t->strafe_m_s) / t->edge_m_s));
  w_back_hi = npmin(1.0, npmax(0.0, ((-vx) - t->back_m_s) / t->edge_m_s))
              * npmin(1.0, npmax(0.0, (t->edge_m_s - fabs(vy)) / t->edge_m_s));
  w_wz_hi = npmin(1.0, npmax(0.0, (fabs(wz) - t->yaw_rate) / t->edge_rad_s));
  w_hz = w_on * npmax(npmax(npmax(npmax(w_slip, w_vx_hi), w_vy_hi), w_back_hi), w_wz5);
  hz_m = w_on * npmax(npmax(t->slip_hz * w_slip, t->brisk_hz * npmax(npmax(w_spin, w_wz_hi), w_vx_hi)), t->quick_hz * w_hz);
  st[t->st_to_hz] = npmax(st[t->st_to_hz], hz_m);
  st[t->st_now_hz] = npmax(st[t->st_now_hz], hz_m);
  out[k->o_turns] += w_hz * (TAU * st[t->st_now_hz] - out[k->o_turns]);
  w_boost_fwd = w_hz * w_fast_fwd;
  out[k->o_sway + k->drive] += w_boost_fwd * (npmin(t->run_drive, out[k->o_sway + k->drive] * t->run_boost) - out[k->o_sway + k->drive]);
  out[k->o_sway + k->arms] += w_boost_fwd * (npmin(t->run_arms, out[k->o_sway + k->arms] * t->run_boost) - out[k->o_sway + k->arms]);
  w_boost_back = (w_hz * w_back_hi) * (1.0 - w_boost_fwd);
  out[k->o_sway + k->drive] += w_boost_back * (npmin(t->back_drive, out[k->o_sway + k->drive] * t->back_boost) - out[k->o_sway + k->drive]);
  out[k->o_ref + k->lateral] += (w_hz * w_vy_hi) * (copysign(t->side_shift, vy) - out[k->o_ref + k->lateral]);
}

void wbc_twist_step(const wbc_imp_t *k, const wbc_twist_t *t, double dt, const double *vel, const double *held,
                    const double *prior)
{
  const int     n = k->n;
  const double *y_in = t->y_in, *y = t->y, *a = t->a;
  double       *fr = t->fr, *st = t->st, *out = t->out, *state = t->state, *w = t->words;
  const double  has_thd = w[t->w_has], *tw = w + t->w_twist, *act = w + t->w_actual;
  const double  vx = tw[0], vy = tw[1], wz = tw[2];
  double        heading, sh, ch, vxp, vyp, wzp, w_ty, w_tz, kv, ky, kz, pelvis_yaw, unborne, w_yaw, l0, l1, wz_way, tz_sgn;
  double        out_st, d_yaw, w_step_yaw, w_lead_yaw, v_horz, w_align, err_v, err_v_ex, err_yaw, err_yaw_ex, sh_n, ch_n;
  double        l_psi, d_psi, l_sgn, cw, last_way, w_flip, old_psi[WBC_IMP_N], turns, drive_sway, active, w_back;
  double        tl_back, tl_fwd, tl, look, tw_on;

  heading = wrap(w[t->w_heading] + has_thd * npmax(-t->slew_rad_s * dt, npmin(t->slew_rad_s * dt, wrap(w[t->w_target] - w[t->w_heading]))));
  sh = sin(heading);
  ch = cos(heading);
  floored(k, t, dt);
  vxp = vel[0] * sh + vel[2] * ch;
  vyp = vel[0] * ch - vel[2] * sh;
  wzp = y_in[t->gyro_at + 1];
  w_ty = npmin(1.0, npmax(0.0, (fabs(vy) - t->side_told_m_s) / t->side_told_m_s));
  w_tz = npmin(1.0, npmax(0.0, (fabs(wz) - t->turn_told_rad_s) / t->turn_told_rad_s));
  kv = npmin(1.0, dt / t->along_s);
  ky = npmin(1.0, dt / (t->across_s - t->told_s * w_ty));
  kz = npmin(1.0, dt / (t->across_s - t->told_s * w_tz));
  w[t->w_actual] = act[0] + kv * (vxp - act[0]);
  w[t->w_actual + 1] = act[1] + ky * (w_ty * vyp - act[1]);
  w[t->w_actual + 2] = act[2] + kz * (w_tz * wzp - act[2]);
  pelvis_yaw = 2.0 * atan2(y_in[t->quat_at + 2], y_in[t->quat_at]);
  unborne = 1.0 - a[k->a_borne];
  w_yaw = (npmin(1.0, fabs(wz) / 1e-4) * (1.0 - has_thd)) * unborne;
  l0 = y[k->y_load];
  l1 = y[k->y_load + 1];
  wz_way = (1.0 - 2.0 * (double)(vx < -t->backing_m_s)) * wz;
  tz_sgn = copysign(1.0, wz_way);
  out_st = t->stance_turn - t->stance_turn_load * tanh(((t->stance_turn_k * (l0 - l1)) * tz_sgn) / npmax(l0 + l1, t->touch_n));
  d_yaw = wrap(pelvis_yaw - heading);
  w_step_yaw = (double)(d_yaw * tz_sgn >= -t->lag_rad);
  w_lead_yaw = (double)(d_yaw * wz_way > 0.0);
  heading = wrap(heading + (w_yaw * (((w_step_yaw * out_st) * wz_way) + ((w_lead_yaw * t->lead_1_s) * d_yaw))) * dt);
  v_horz = hypot2(y[k->y_vcom], y[k->y_vcom + 2]);
  w_align = (((w_yaw * (double)(vx >= t->align_m_s)) * (double)(fabs(vy) < t->align_side_m_s)) * (double)(v_horz > t->align_speed_m_s))
            * (double)(vxp > t->align_own_m_s);
  err_v = wrap(atan2(y[k->y_vcom], y[k->y_vcom + 2]) - heading);
  err_v_ex = copysign(npmax(0.0, fabs(err_v) - t->align_rad), err_v);
  heading = wrap(heading + ((w_align * t->align_1_s) * err_v_ex) * dt);
  err_yaw = wrap(pelvis_yaw - heading);
  err_yaw_ex = copysign(npmax(0.0, fabs(err_yaw) - t->slack_rad), err_yaw);
  heading = wrap(heading + w_yaw * err_yaw_ex);
  sh_n = sin(heading);
  ch_n = cos(heading);
  fr[t->f_heading] = heading;
  fr[k->f_rot] = sh_n;
  fr[k->f_rot + 1] = 0.0;
  fr[k->f_rot + 2] = ch_n;
  fr[k->f_rot + 3] = ch_n;
  fr[k->f_rot + 4] = 0.0;
  fr[k->f_rot + 5] = -sh_n;
  fr[k->f_rot + 6] = 0.0;
  fr[k->f_rot + 7] = 1.0;
  fr[k->f_rot + 8] = 0.0;
  /* each standing sole's turn of her way since it was planted, naught for one in the air (machine.law.se2.twisted) */
  for (int b = 0; b < 2; b++)
  {
    fr[k->f_planted + b] = a[k->a_stance + b] * (fr[k->f_planted + b] + wrap(heading - w[t->w_heading]));
  }
  /* her rhythm flipped half a turn where her sway leads the other way */
  l_psi = out[k->o_psi + k->lateral];
  d_psi = out[k->o_psi + k->drive];
  l_sgn = copysign(1.0, out[k->o_sway + k->lateral]);
  cw = copysign(1.0, sin(l_psi - d_psi) * l_sgn + 0.5);
  last_way = isnan(state[t->s_last_way]) ? cw : state[t->s_last_way];
  w_flip = (double)(cw * last_way < 0.0)
           * npmax((double)(st[t->st_now_gain] > t->flip_gain), (double)(fabs(out[k->o_sway + k->drive]) > t->flip_drive));
  for (int m = 0; m < n; m++)
  {
    old_psi[m] = out[k->o_psi + m];
  }
  st[t->st_theta] = pymod(st[t->st_theta] + w_flip * PI, TAU);
  turns = out[k->o_turns];
  for (int m = 0; m < n; m++)
  {
    out[k->o_psi + m] = st[t->st_theta] + st[t->st_now_phase + m];
    out[k->o_ref + m] += ((-2.0 * w_flip) * out[k->o_sway + m]) * sin(old_psi[m]);
    out[k->o_rate + m] += (((-2.0 * w_flip) * turns) * out[k->o_sway + m]) * cos(old_psi[m]);
    out[k->o_acc + m] *= 1.0 - 2.0 * w_flip;
  }
  state[t->s_last_way] = cw;
  modulate(k, t, vx, vy, wz, vxp, held, prior);
  /* her look into a turn, her waist and her head with it */
  drive_sway = fabs(out[k->o_sway + k->drive]);
  active = npmin(1.0, npmax(0.0, (drive_sway - t->stirs) / t->look_drive)) * unborne;
  w_back = (double)(active > t->look_on) * npmax((double)(cw < 0.0), (double)(vx < -t->backing_m_s));
  tl_back = copysign(1.0, wz + t->look_back_rad_s);
  tl_fwd = (double)(fabs(wz) > t->look_turn_rad_s) * npmax(-t->look_rad, npmin(t->look_rad, (t->look_rad * wz) / t->yaw_rate));
  tl = active * (w_back * tl_back + (1.0 - w_back) * tl_fwd);
  look = state[t->s_look_yaw] + npmin(1.0, (t->look_1_s - t->look_back_1_s * w_back) * dt) * (tl - state[t->s_look_yaw]);
  state[t->s_look_yaw] = look;
  t->posture[t->waist] = (t->waist_share * look) * unborne;
  t->posture[t->head] = (t->head_share * look) * unborne;
  w[t->w_heading] = heading;
  w[t->w_back] = w_back * (double)(fabs(look) > 1e-3);
  w[t->w_backing] = t->posture[t->waist];
  tw_on = npmax(w[t->w_active], (double)(w[t->w_stop] > w[t->w_t]));
  w[t->w_clear] = ((t->clear_m * tw_on) * unborne) * (double)(out[k->o_held] > 0.5);
}
