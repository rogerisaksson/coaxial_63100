/** harness.c - aero/'s sizes, a row of its memory, and host wing/observer passes (tools/cores/aero.py). */
#include "aero.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

#define AERO_PASS_PI          3.14159265358979323846
#define AERO_PASS_STALL_RAD   (15.0 * AERO_PASS_PI / 180.0)
#define AERO_PASS_STALL_BLEND (8.0 * AERO_PASS_PI / 180.0)
#define AERO_PASS_ALPHA_MAX   (15.0 * AERO_PASS_PI / 180.0)
#define AERO_PASS_CD_LIFT     0.07

/** sizeof the memory, the payload, the frame, a reading, an answer. */
API unsigned aeh_size(int which)
{
  const unsigned sizes[] = {sizeof(aero_t), sizeof(aero_payload_t), sizeof(aero_params_t),
                            sizeof(aero_in_t), sizeof(aero_out_t)};

  return sizes[which];
}

/** A row of the modes as blended (AERO_TARGET ..) - AERO_ROWS: the targets with the
    oscillator on them -, the rate modes' sums after them. */
API void aeh_row(const aero_t *s, int row, float out[AERO_MODES + AERO_AXES])
{
  const float *rows[AERO_ROWS + 1U] = {s->now[0], s->now[1], s->now[2], s->now[3], s->now[4],
                                       s->ask};

  memcpy(out, rows[row], AERO_MODES * sizeof(float));
  memcpy(out + AERO_MODES, s->wound, AERO_AXES * sizeof(float));
}

/** Neumaier compensated 3-term sum matching CPython 3.12's builtin sum() over 3 floats. */
static inline double sum3(double a, double b, double c)
{
  double       hi = 0.0, lo = 0.0;
  const double xs[3] = {a, b, c};

  for (int i = 0; i < 3; i++)
  {
    const double x = xs[i];
    const double s = hi + x;
    const double err = (fabs(hi) >= fabs(x)) ? ((hi - s) + x) : ((x - s) + hi);

    hi = s;
    lo += err;
  }
  return hi + lo;
}

static inline double clamp_d(double x, double lo, double hi)
{
  const double a = (x > lo) ? x : lo;

  return (a < hi) ? a : hi;
}

static inline double norm3(double x, double y, double z)
{
  return sqrt((x * x + y * y) + z * z);
}

static inline void cross3(const double a[3], const double b[3], double out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static inline void mat_mul3(const double a[9], const double b[9], double out[9])
{
  double r[9];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      r[3 * i + j] = (a[3 * i + 0] * b[0 * 3 + j] + a[3 * i + 1] * b[1 * 3 + j])
                     + a[3 * i + 2] * b[2 * 3 + j];
    }
  }
  memcpy(out, r, 9 * sizeof(double));
}

static inline void mat_t3(const double a[9], double out[9])
{
  double r[9];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      r[3 * i + j] = a[3 * j + i];
    }
  }
  memcpy(out, r, 9 * sizeof(double));
}

static inline void rot_x3(double rad, double out[9])
{
  const double c = cos(rad), s = sin(rad);

  out[0] = 1.0; out[1] = 0.0; out[2] = 0.0;
  out[3] = 0.0; out[4] = c;   out[5] = -s;
  out[6] = 0.0; out[7] = s;   out[8] = c;
}

static inline void rot_y3(double rad, double out[9])
{
  const double c = cos(rad), s = sin(rad);

  out[0] = c;   out[1] = 0.0; out[2] = s;
  out[3] = 0.0; out[4] = 1.0; out[5] = 0.0;
  out[6] = -s;  out[7] = 0.0; out[8] = c;
}

/** Airframe force in world axes (`wingborne.Wing.lift`); w = {rho, side, area, cl_alpha}. */
API void aeh_wing_lift(const double w[4], const double u[3], const double turn[9], double out[3])
{
  const double rho = w[0], side = w[1], area = w[2], cl_alpha = w[3];
  const double span[3] = {turn[0], turn[3], turn[6]};
  const double u2 = sum3(u[0] * u[0], u[1] * u[1], u[2] * u[2]);
  const double u_span = sum3(u[0] * span[0], u[1] * span[1], u[2] * span[2]);
  const double slip = (-rho * side * sqrt(u2)) * u_span;
  const double uy_raw = sum3(u[0] * turn[1], u[1] * turn[4], u[2] * turn[7]);
  const double uy = (uy_raw > 0.0) ? uy_raw : 0.0;
  const double uz = sum3(u[0] * turn[2], u[1] * turn[5], u[2] * turn[8]);
  const double v2 = uy * uy + uz * uz;

  if (uy <= 0.0 || v2 < 1e-6)
  {
    out[0] = slip * span[0];
    out[1] = slip * span[1];
    out[2] = slip * span[2];
    return;
  }
  {
    const double alpha = atan2(uz, uy);
    const double a_abs = fabs(alpha);
    const double flat = clamp_d((a_abs - AERO_PASS_STALL_RAD) / AERO_PASS_STALL_BLEND, 0.0, 1.0);
    const double sa = sin(alpha), ca = cos(alpha);
    const double c_lin = cl_alpha * alpha;
    const double c_l = (1.0 - flat) * c_lin + ((flat * 2.0) * sa) * ca;
    const double c_d = ((1.0 - flat) * AERO_PASS_CD_LIFT * c_lin) * c_lin + ((flat * 2.0) * sa) * sa;
    const double q = (0.5 * rho * area) * v2;
    const double f_l = q * c_l, f_d = q * c_d;
    const double speed = sqrt(v2);
    const double nose[3] = {turn[1], turn[4], turn[7]};
    const double back[3] = {-turn[2], -turn[5], -turn[8]};

    for (int a = 0; a < 3; a++)
    {
      const double air = (uy * nose[a] - uz * back[a]) / speed;
      const double up = (uz * nose[a] + uy * back[a]) / speed;

      out[a] = (slip * span[a] + f_l * up) - f_d * air;
    }
  }
}

/** Airframe moments in body axes (`wingborne.Wing.moments`); w = {q, arm, stab, area, span, fin}. */
API void aeh_wing_moments(const double w[6], const double u[3], const double turn[9],
                          const double spin[3], double out[3])
{
  const double ux = sum3(u[0] * turn[0], u[1] * turn[3], u[2] * turn[6]);
  const double uy = sum3(u[0] * turn[1], u[1] * turn[4], u[2] * turn[7]);
  const double uz = sum3(u[0] * turn[2], u[1] * turn[5], u[2] * turn[8]);
  const double q = w[0] * uy, qd = w[0] * fabs(uy);
  const double arm = w[1], stab = w[2], area = w[3], span = w[4], fin = w[5];
  const double quarter_span = 0.25 * span;

  out[0] = (arm * stab) * (q * uz - (qd * arm) * spin[0]);
  out[1] = ((-qd * area) * (quarter_span * quarter_span)) * spin[1];
  out[2] = (-arm * fin) * (q * ux + (qd * arm) * spin[2]);
}

/** Finite-difference wing lift Jacobian (`aerobat.observer.observe`) in one C pass. */
API void aeh_wing_jacobian(const double w[4], const double u[3], const double turn[9], double mass,
                           double out[9])
{
  const double denom = 0.2 * mass;

  for (int b = 0; b < 3; b++)
  {
    double u_pos[3] = {u[0], u[1], u[2]};
    double u_neg[3] = {u[0], u[1], u[2]};
    double f_pos[3], f_neg[3];

    u_pos[b] += 0.1;
    u_neg[b] -= 0.1;
    aeh_wing_lift(w, u_pos, turn, f_pos);
    aeh_wing_lift(w, u_neg, turn, f_neg);
    for (int a = 0; a < 3; a++)
    {
      out[3 * a + b] = (f_neg[a] - f_pos[a]) / denom;
    }
  }
}

/** Wind and spin observer step (`helicopter.kit._observe_wind_spin`).
    state_io = {wind[3], push_hat[3], spin_hat[3], turn_hat[3]},
    obs_p = {min_air_m_s, wo_wind, wind_reg, wo_turn}. */
API void aeh_observe_wind_spin(double state_io[12], const double u[3], double fast, double alpha,
                               double gamma, const double up[3], const double spin[3],
                               const double tau[3], const double inertia[3], double dt,
                               double side, const double span[3], const double wing_j[9],
                               const double obs_p[4])
{
  double      *wind = state_io;
  double      *push_hat = state_io + 3;
  double      *spin_hat = state_io + 6;
  double      *turn_hat = state_io + 9;
  const double min_air = obs_p[0], wo_wind = obs_p[1], wind_reg = obs_p[2], wo_turn = obs_p[3];
  double       uh[3], m[3][3];

  for (int a = 0; a < 3; a++)
  {
    uh[a] = (fast > min_air) ? (u[a] / fast) : 0.0;
  }
  for (int a = 0; a < 3; a++)
  {
    for (int b = 0; b < 3; b++)
    {
      const double diag = (a == b) ? 1.0 : 0.0;
      const double wj = (wing_j != NULL) ? wing_j[3 * a + b] : 0.0;

      m[a][b] = ((alpha * (diag + uh[a] * uh[b]) + (gamma * up[a]) * up[b])
                 + (side * span[a]) * span[b]) + wj;
    }
  }
  for (int idx = 0; idx < 2; idx++)
  {
    const int    a = (idx == 0) ? 0 : 2;
    const double num = sum3(m[0][a] * push_hat[0], m[1][a] * push_hat[1], m[2][a] * push_hat[2]);
    const double den = ((m[0][a] * m[0][a] + m[1][a] * m[1][a]) + m[2][a] * m[2][a]) + wind_reg;
    const double dw = ((dt * wo_wind) * num) / den;

    wind[a] += dw;
    for (int b = 0; b < 3; b++)
    {
      push_hat[b] -= m[b][a] * dw;
    }
  }
  for (int a = 0; a < 3; a++)
  {
    const double e = spin[a] - spin_hat[a];

    spin_hat[a] += dt * ((tau[a] / inertia[a] + turn_hat[a]) + (2.0 * wo_turn) * e);
    turn_hat[a] = clamp_d((1.0 - 2.5 * dt) * turn_hat[a] + ((dt * wo_turn) * wo_turn) * e,
                          -12.0, 12.0);
  }
}

/** Combined aerobat observer tail: lift, vel/push observer, moments, Jacobian, wind/spin observer.
    state_io = {wind[3], push_hat[3], spin_hat[3], turn_hat[3], vel_hat[3]}.
    params = {mass, gravity, wo_pull, prop_r, k_pitch, k_yaw, k_roll, q_p, w_p, w_v,
              elev, rud, ail, lift_inflow, alpha, fast, dt}. */
API void aeh_aerobat_observe(double state_io[15], const double vel[3], const double spin[3],
                             const double turn[9], const double w_lift[4], const double w_mom[6],
                             const double inertia[3], const double obs_p[4],
                             const double params[17])
{
  const double *wind = state_io;
  double       *push_hat = state_io + 3;
  double       *vel_hat = state_io + 12;
  const double  mass = params[0], gravity = params[1], wo_pull = params[2], prop_r = params[3];
  const double  k_pitch = params[4], k_yaw = params[5], k_roll = params[6];
  const double  q_p = params[7], w_p = params[8], w_v = params[9];
  const double  elev = params[10], rud = params[11], ail = params[12];
  const double  lift_inflow = params[13], alpha = params[14], fast = params[15], dt = params[16];
  const double  u[3] = {vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2]};
  const double  up[3] = {turn[1], turn[4], turn[7]};
  const double  span[3] = {turn[0], turn[3], turn[6]};
  double        f_wing[3], own[3], tau[3], wing_j[9];

  aeh_wing_lift(w_lift, u, turn, f_wing);
  for (int a = 0; a < 3; a++)
  {
    const double off = vel[a] - vel_hat[a];
    const double g_a = (a == 1) ? gravity : 0.0;
    const double acc = (((lift_inflow * up[a] + f_wing[a]) / mass - g_a) + push_hat[a])
                       - alpha * u[a];

    vel_hat[a] += dt * (acc + (2.0 * wo_pull) * off);
    push_hat[a] += ((dt * wo_pull) * wo_pull) * off;
  }
  aeh_wing_moments(w_mom, u, turn, spin, own);
  tau[0] = ((k_pitch * w_p) * w_p) * elev + own[0] - ((inertia[2] - inertia[1]) * spin[1]) * spin[2];
  tau[1] = (((k_yaw * w_v) * w_v) * rud - q_p) + own[1]
           - ((inertia[0] - inertia[2]) * spin[2]) * spin[0];
  tau[2] = ((k_roll * w_p) * w_p) * ail + own[2] - ((inertia[1] - inertia[0]) * spin[0]) * spin[1];
  aeh_wing_jacobian(w_lift, u, turn, mass, wing_j);
  aeh_observe_wind_spin(state_io, u, fast, alpha, prop_r, up, spin, tau, inertia, dt,
                        ((w_lift[0] * w_lift[1]) * fast) / mass, span, wing_j, obs_p);
}

/** 10-step angle-of-attack bisection in `wingborne.plane`. */
API double aeh_plane_alpha(double k_w, double cl_alpha, double along, double need, double sign)
{
  const double al_pos = (along > 0.0) ? along : 0.0;
  double       hi = (sign < 0.0) ? AERO_PASS_ALPHA_MAX
                    : ((need > k_w * AERO_PASS_ALPHA_MAX) ? atan2(need, al_pos)
                                                          : AERO_PASS_ALPHA_MAX);
  double       lo = 0.0;
  const double k_over_cla = k_w / cl_alpha;

  for (int i = 0; i < 10; i++)
  {
    const double mid = 0.5 * (lo + hi);
    const double flat = clamp_d((mid - AERO_PASS_STALL_RAD) / AERO_PASS_STALL_BLEND, 0.0, 1.0);
    const double c_lin = cl_alpha * mid;
    const double sm = sin(mid), cm = cos(mid);
    const double a_w = k_over_cla * ((1.0 - flat) * c_lin + ((flat * 2.0) * sm) * cm);
    const double d_w = k_over_cla * (((1.0 - flat) * AERO_PASS_CD_LIFT * c_lin) * c_lin
                                     + (flat * 2.0) * (sm * sm));

    if (need - (al_pos + d_w) * tan(mid) > a_w) lo = mid;
    else hi = mid;
  }
  return sign * (0.5 * (lo + hi));
}

/** SO(3) attitude error (`helicopter.kit._so3_error`). */
API void aeh_so3_error(const double r_cur[9], const double r_des[9], double out[3])
{
  double       rt[9], e[9];
  double       vx, vy, vz, sa, ca, sc;

  mat_t3(r_cur, rt);
  mat_mul3(rt, r_des, e);
  vx = 0.5 * (e[7] - e[5]);
  vy = 0.5 * (e[2] - e[6]);
  vz = 0.5 * (e[3] - e[1]);
  sa = norm3(vx, vy, vz);
  ca = clamp_d(0.5 * (((e[0] + e[4]) + e[8]) - 1.0), -1.0, 1.0);
  if (sa <= 1e-4 && ca < 0.0)
  {
    const double x0 = sqrt((0.5 * (e[0] + 1.0) > 0.0) ? (0.5 * (e[0] + 1.0)) : 0.0);
    const double x1 = sqrt((0.5 * (e[4] + 1.0) > 0.0) ? (0.5 * (e[4] + 1.0)) : 0.0);
    const double x2 = sqrt((0.5 * (e[8] + 1.0) > 0.0) ? (0.5 * (e[8] + 1.0)) : 0.0);

    out[0] = -AERO_PASS_PI * ((x0 != 0.0) ? x0 : 1.0);
    out[1] = AERO_PASS_PI * x1;
    out[2] = AERO_PASS_PI * x2;
    return;
  }
  sc = (sa > 1e-5) ? (atan2(sa, ca) / sa) : 1.0;
  out[0] = vx * sc;
  out[1] = vy * sc;
  out[2] = vz * sc;
}

/** Align `from_u` onto `to_u` applied to `mat` (`helicopter.kit._align_axis`). */
API void aeh_align_axis(const double from_u[3], const double to_u[3], const double mat[9],
                        double out[9])
{
  double       w[3];
  const double d = 1.0 + sum3(from_u[0] * to_u[0], from_u[1] * to_u[1], from_u[2] * to_u[2]);

  cross3(from_u, to_u, w);
  if (d <= 1e-4)
  {
    double rx[9];

    rot_x3(AERO_PASS_PI, rx);
    mat_mul3(rx, mat, out);
    return;
  }
  {
    double cols[9];

    for (int col = 0; col < 3; col++)
    {
      const double v[3] = {mat[col], mat[3 + col], mat[6 + col]};
      double       u[3], wu[3];

      cross3(w, v, u);
      cross3(w, u, wu);
      cols[3 * col + 0] = (v[0] + u[0]) + wu[0] / d;
      cols[3 * col + 1] = (v[1] + u[1]) + wu[1] / d;
      cols[3 * col + 2] = (v[2] + u[2]) + wu[2] / d;
    }
    mat_t3(cols, out);
  }
}

static inline void exp_so3_3(double x, double y, double z, double out[9])
{
  const double a2 = (x * x + y * y) + z * z;
  const double a = sqrt(a2);
  const double s = (a > 1e-7) ? (sin(a) / a) : (1.0 - a2 / 6.0);
  const double c = (a > 1e-7) ? ((1.0 - cos(a)) / a2) : (0.5 - a2 / 24.0);

  out[0] = 1.0 - c * (y * y + z * z); out[1] = -s * z + c * x * y; out[2] = s * y + c * x * z;
  out[3] = s * z + c * x * y;         out[4] = 1.0 - c * (x * x + z * z); out[5] = -s * x + c * y * z;
  out[6] = -s * y + c * x * z;        out[7] = s * x + c * y * z;         out[8] = 1.0 - c * (x * x + y * y);
}

static inline double allowed_rate_d(double e, double a_p, double pole)
{
  const double d = fabs(e);
  const double near = a_p / (pole * pole);
  const double far = (d > near) ? 1.0 : 0.0;
  const double rem = (d - 0.5 * near > 0.0) ? (d - 0.5 * near) : 0.0;
  const double mag = (1.0 - far) * (pole * d) + far * sqrt((2.0 * a_p) * rem);

  return copysign(mag, e);
}

/** Smooth attitude target (`helicopter.kit._smooth_target`).
    filt_io = {up_filt[3], r_prev[9], w_ff[3]}, flags = {has_up, has_r}. */
API void aeh_smooth_target(double filt_io[15], int flags[2], const double want_in[3],
                           double sign, double yaw, double rate, double pace, double dt,
                           double flip_ang, double r_des_out[9])
{
  double      *up_filt = filt_io;
  double      *r_prev = filt_io + 3;
  double      *w_ff = filt_io + 12;
  double       ry[9], rx[9], r_yx[9], r0[9], r_des[9];
  const double from0[3] = {0.0, sign, 0.0};

  (void)pace;
  rot_y3(yaw, ry);
  rot_x3(flip_ang, rx);
  mat_mul3(ry, rx, r_yx);
  aeh_align_axis(from0, want_in, r_yx, r0);
  if (!flags[1] || dt <= 1e-6)
  {
    w_ff[0] = 0.0; w_ff[1] = sign * rate; w_ff[2] = 0.0;
    memcpy(up_filt, want_in, 3 * sizeof(double));
    memcpy(r_prev, r0, 9 * sizeof(double));
    memcpy(r_des_out, r0, 9 * sizeof(double));
    flags[0] = 1; flags[1] = 1;
    return;
  }
  {
    const double prev_y[3] = {r_prev[1], r_prev[4], r_prev[7]};
    const double wy = (sign * want_in[1] > 0.0) ? (sign * want_in[1]) : 0.0;
    const double pole = 5.5, kd = 22.0, a_p = 30.25, a_max = 60.0, w_max = 6.0;
    const double wt[3] = {0.0, sign * rate, 0.0};
    double       aligned[9], ry_dt[9], r_tr[9], err0[3], ry_wy[9], r_t[9], er[3], d_exp[9];

    aeh_align_axis(prev_y, want_in, r_prev, aligned);
    rot_y3((sign * rate) * dt, ry_dt);
    mat_mul3(aligned, ry_dt, r_tr);
    aeh_so3_error(r_tr, r0, err0);
    rot_y3((wy * wy) * err0[1], ry_wy);
    mat_mul3(r_tr, ry_wy, r_t);
    aeh_so3_error(r_prev, r_t, er);
    for (int a = 0; a < 3; a++)
    {
      const double acc = kd * ((allowed_rate_d(er[a], a_p, pole) + wt[a]) - w_ff[a]);

      w_ff[a] = clamp_d(w_ff[a] + clamp_d(acc, -a_max, a_max) * dt, -w_max, w_max);
    }
    exp_so3_3(w_ff[0] * dt, w_ff[1] * dt, w_ff[2] * dt, d_exp);
    mat_mul3(r_prev, d_exp, r_des);
  }
  memcpy(up_filt, want_in, 3 * sizeof(double));
  memcpy(r_prev, r_des, 9 * sizeof(double));
  memcpy(r_des_out, r_des, 9 * sizeof(double));
  flags[0] = 1;
  flags[1] = 1;
}

/** SO(3) geodesic interpolation (`wingborne.toward`). */
API void aeh_toward(const double r[9], const double to[9], double k, double out[9])
{
  double       err[3], x, y, z, a, s, c;
  double       k_[9], k2[9], turn[9];

  aeh_so3_error(r, to, err);
  x = k * err[0]; y = k * err[1]; z = k * err[2];
  a = sqrt((x * x + y * y) + z * z);
  s = (a > 1e-9) ? (sin(a) / a) : 1.0;
  c = (a > 1e-9) ? ((1.0 - cos(a)) / (a * a)) : 0.5;
  k_[0] = 0.0; k_[1] = -z;  k_[2] = y;
  k_[3] = z;   k_[4] = 0.0; k_[5] = -x;
  k_[6] = -y;  k_[7] = x;   k_[8] = 0.0;
  mat_mul3(k_, k_, k2);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      const double diag = (i == j) ? 1.0 : 0.0;

      turn[3 * i + j] = (diag + s * k_[3 * i + j]) + c * k2[3 * i + j];
    }
  }
  mat_mul3(r, turn, out);
}

/** SO(3) attitude torques (`helicopter.kit._so3_torques`). */
API void aeh_so3_torques(const double turn[9], const double r_des[9], const double w_ff[3],
                         const double spin[3], const double turn_hat[3], const double inertia[3],
                         const double kp[3], const double kd[3], double out[3])
{
  double       er[3];
  const double ew0 = w_ff[0] - spin[0], ew1 = w_ff[1] - spin[1], ew2 = w_ff[2] - spin[2];

  aeh_so3_error(turn, r_des, er);
  out[0] = inertia[0] * ((kp[0] * er[0] + kd[0] * ew0) - turn_hat[0])
           + ((inertia[2] - inertia[1]) * spin[1]) * spin[2];
  out[1] = inertia[1] * ((kp[1] * er[1] + kd[1] * ew1) - turn_hat[1])
           + ((inertia[0] - inertia[2]) * spin[2]) * spin[0];
  out[2] = inertia[2] * ((kp[2] * er[2] + kd[2] * ew2) - turn_hat[2])
           + ((inertia[1] - inertia[0]) * spin[0]) * spin[1];
}

/** SO(3) attitude torques less wing moments (`wingborne.Wing.trimmed(_so3_torques(...))`). */
API void aeh_so3_torques_trimmed(const double turn[9], const double r_des[9],
                                 const double w_ff[3], const double spin[3],
                                 const double turn_hat[3], const double inertia[3],
                                 const double kp[3], const double kd[3],
                                 const double w_mom[6], const double u[3], double out[3])
{
  double tau[3], m[3];

  aeh_so3_torques(turn, r_des, w_ff, spin, turn_hat, inertia, kp, kd, tau);
  aeh_wing_moments(w_mom, u, turn, spin, m);
  out[0] = tau[0] - m[0];
  out[1] = tau[1] - m[1];
  out[2] = tau[2] - m[2];
}

/** Wing-borne attitude law (`wingborne.plane`).
    w_plane = {q, area, mass, cl_alpha, rho, side, gravity}.
    Returns 1 if r_des_out was written, 0 if None. */
API int aeh_plane(const double w_plane[7], const double turn[9], const double acc[3],
                  const double u[3], double w, double want_io[3], double knife,
                  double r_des_out[9])
{
  const double fast = norm3(u[0], u[1], u[2]);

  if (w <= 0.0 || fast <= 1e-3)
  {
    return 0;
  }
  {
    const double q = w_plane[0], area = w_plane[1], mass = w_plane[2], cl_alpha = w_plane[3];
    const double rho = w_plane[4], side = w_plane[5], gravity = w_plane[6];
    const double v_hat[3] = {u[0] / fast, u[1] / fast, u[2] / fast};
    const double along = sum3(acc[0] * v_hat[0], acc[1] * v_hat[1], acc[2] * v_hat[2]);
    const double perp[3] = {acc[0] - along * v_hat[0], acc[1] - along * v_hat[1],
                            acc[2] - along * v_hat[2]};
    const double size = norm3(perp[0], perp[1], perp[2]);
    const double cr = cos((0.5 * AERO_PASS_PI) * knife);
    const double sr = sin((0.5 * AERO_PASS_PI) * knife);
    double       back[3] = {sr * turn[0] - cr * turn[2], sr * turn[3] - cr * turn[5],
                            sr * turn[6] - cr * turn[8]};
    const double bv = sum3(back[0] * v_hat[0], back[1] * v_hat[1], back[2] * v_hat[2]);
    double       n, under, d[3], sign, need, ka, upp[3], up_ask, across[3], k_w, al_pos, alpha;
    double       across_n, a_cross, ca, sa, nose[3], y[3], dy, z[3], x[3], xr[3], zr[3];

    for (int a = 0; a < 3; a++) back[a] -= bv * v_hat[a];
    n = norm3(back[0], back[1], back[2]);
    if (n == 0.0) n = 1.0;
    for (int a = 0; a < 3; a++) back[a] /= n;
    under = sum3(perp[0] * back[0], perp[1] * back[1], perp[2] * back[2]);
    if (under < 0.0)
    {
      d[0] = back[0]; d[1] = back[1]; d[2] = back[2];
      sign = -1.0; need = -under;
    }
    else
    {
      const double boost = (3.0 - size > 0.0) ? (3.0 - size) : 0.0;

      for (int a = 0; a < 3; a++) d[a] = perp[a] + boost * back[a];
      n = norm3(d[0], d[1], d[2]);
      if (n == 0.0) n = 1.0;
      for (int a = 0; a < 3; a++) d[a] /= n;
      sign = 1.0; need = size;
    }
    ka = fabs(knife);
    upp[0] = 0.0 - v_hat[1] * v_hat[0];
    upp[1] = 1.0 - v_hat[1] * v_hat[1];
    upp[2] = 0.0 - v_hat[1] * v_hat[2];
    n = norm3(upp[0], upp[1], upp[2]);
    if (n == 0.0) n = 1.0;
    for (int a = 0; a < 3; a++)
    {
      upp[a] /= n;
      d[a] = (1.0 - ka) * d[a] + ka * upp[a];
    }
    n = norm3(d[0], d[1], d[2]);
    if (n == 0.0) n = 1.0;
    for (int a = 0; a < 3; a++) d[a] /= n;
    up_ask = sum3(perp[0] * upp[0], perp[1] * upp[1], perp[2] * upp[2]);
    for (int a = 0; a < 3; a++) across[a] = perp[a] - up_ask * upp[a];
    k_w = (((q * area) * fast) * fast) / mass;
    al_pos = (along > 0.0) ? along : 0.0;
    alpha = aeh_plane_alpha(k_w, cl_alpha, along, need, sign);
    alpha += ka * (atan2(up_ask, (((rho * side) * fast) * fast) / mass + al_pos) - alpha);
    across_n = norm3(across[0], across[1], across[2]);
    a_cross = clamp_d(across_n / ((k_w > 1e-9) ? k_w : 1e-9), -1e9, AERO_PASS_ALPHA_MAX)
              / ((across_n != 0.0) ? across_n : 1.0);
    {
      const double slow_a = AERO_PASS_ALPHA_MAX * clamp_d((-sign * along) / gravity, 0.0, 1.0);
      if (slow_a > alpha) alpha = slow_a;
    }
    ca = cos(alpha); sa = sin(alpha);
    for (int a = 0; a < 3; a++)
    {
      nose[a] = (ca * v_hat[a] + sa * d[a]) + (ka * a_cross) * across[a];
      want_io[a] = (1.0 - w) * want_io[a] + w * nose[a];
    }
    n = norm3(want_io[0], want_io[1], want_io[2]);
    if (n == 0.0) n = 1.0;
    for (int a = 0; a < 3; a++) y[a] = want_io[a] = want_io[a] / n;
    dy = sum3(d[0] * y[0], d[1] * y[1], d[2] * y[2]);
    for (int a = 0; a < 3; a++) back[a] = d[a] - dy * y[a];
    n = norm3(back[0], back[1], back[2]);
    if (n == 0.0) n = 1.0;
    for (int a = 0; a < 3; a++) z[a] = -back[a] / n;
    cross3(y, z, x);
    for (int a = 0; a < 3; a++)
    {
      xr[a] = cr * x[a] - sr * z[a];
      zr[a] = sr * x[a] + cr * z[a];
      r_des_out[3 * a + 0] = xr[a];
      r_des_out[3 * a + 1] = y[a];
      r_des_out[3 * a + 2] = zr[a];
    }
    return 1;
  }
}
