/** harness_so3.c - host SO(3) attitude, wing-borne plane, and trimmed torque passes (tools/cores/aero.py). */
#include "aero.h"
#include "board_units.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

#define AERO_PASS_STALL_RAD   (15.0 * DEG_TO_RAD_D)
#define AERO_PASS_STALL_BLEND (8.0 * DEG_TO_RAD_D)
#define AERO_PASS_ALPHA_MAX   (15.0 * DEG_TO_RAD_D)
#define AERO_PASS_CD_LIFT     0.07

API void aeh_wing_moments(const double w[6], const double u[3], const double turn[9],
                          const double spin[3], double out[3]);

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

static inline void unit3(double v[3])
{
  const double n = norm3(v[0], v[1], v[2]);
  const double d = (n == 0.0) ? 1.0 : n;

  v[0] /= d; v[1] /= d; v[2] /= d;
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

static inline double ease_d(double x)
{
  const double t = clamp_d(x, 0.0, 1.0);

  return 0.5 * (1.0 - cos(PI_D * t));
}

static double aeh_plane_alpha(double k_w, double cl_alpha, double along, double need, double sign)
{
  const double al_pos = (along > 0.0) ? along : 0.0;
  double       hi = AERO_PASS_ALPHA_MAX;
  double       lo = 0.0;
  const double k_over_cla = k_w / cl_alpha;
  double       a_pre;

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
  a_pre = 0.5 * (lo + hi);
  if (sign > 0.0)
  {
    const double a_post = atan2(need, al_pos + 2.0 * k_over_cla);
    const double kw_max = k_w * AERO_PASS_ALPHA_MAX;
    const double s_st = ease_d((need / ((kw_max > 1e-9) ? kw_max : 1e-9) - 0.90) / 0.20);
    const double diff = a_post - a_pre;

    if (diff > 0.0) a_pre += s_st * diff;
  }
  return sign * a_pre;
}

static void aeh_so3_error(const double r_cur[9], const double r_des[9], double out[3])
{
  double rt[9], e[9];
  double vx, vy, vz, sa, ca, sc;

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

    out[0] = -PI_D * ((x0 != 0.0) ? x0 : 1.0);
    out[1] = PI_D * x1;
    out[2] = PI_D * x2;
    return;
  }
  sc = (sa > 1e-5) ? (atan2(sa, ca) / sa) : 1.0;
  out[0] = vx * sc;
  out[1] = vy * sc;
  out[2] = vz * sc;
}

static void aeh_align_axis(const double from_u[3], const double to_u[3], const double mat[9],
                           double out[9])
{
  double       w[3];
  const double d = 1.0 + sum3(from_u[0] * to_u[0], from_u[1] * to_u[1], from_u[2] * to_u[2]);

  cross3(from_u, to_u, w);
  if (d <= 1e-4)
  {
    double rx[9];

    rot_x3(PI_D, rx);
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

API void aeh_smooth_target(double filt_io[15], int flags[2], const double want_in[3],
                           double sign, double yaw, double rate, double pace, double dt,
                           double flip_ang, double r_des_out[9])
{
  double      *up_filt = filt_io;
  double      *r_prev = filt_io + 3;
  double      *w_ff = filt_io + 12;
  double       ry[9], rx[9], r_yx[9], r0[9], r_des[9];
  const double from0[3] = {0.0, sign, 0.0};

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
    const double boost = clamp_d(0.25 * (pace - 2.0), 0.0, 1.0);
    const double pole = 5.5 + 2.0 * boost;
    const double kd = 4.0 * pole, a_p = pole * pole;
    const double a_max = 60.0 + 52.5 * boost, w_max = 6.0 + 2.0 * boost;
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

API void aeh_toward(const double r[9], const double to[9], double k, double out[9])
{
  double err[3], x, y, z, a, s, c;
  double k_[9], k2[9], turn[9];

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

API int aeh_plane(const double w_plane[8], const double turn[9], const double acc[3],
                  const double u[3], double w, double want_io[3], double knife,
                  double roll, double r_des_out[9])
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
    const double cr = cos(HALF_PI_D * knife);
    const double sr = sin(HALF_PI_D * knife);
    double       back[3] = {sr * turn[0] - cr * turn[2], sr * turn[3] - cr * turn[5],
                            sr * turn[6] - cr * turn[8]};
    const double bv = sum3(back[0] * v_hat[0], back[1] * v_hat[1], back[2] * v_hat[2]);
    double       under, d[3], sign, need, ka, kh, upp[3], up_ask, across[3], k_w, al_pos, alpha;
    double       need_w, sign_w, a_knife, k_pitch;
    double       across_n, a_cross, ca, sa, nose[3], y[3], dy, z[3], x[3], xr[3], zr[3];

    for (int a = 0; a < 3; a++) back[a] -= bv * v_hat[a];
    unit3(back);
    ka = fabs(sr);
    upp[0] = 0.0 - v_hat[1] * v_hat[0];
    upp[1] = 1.0 - v_hat[1] * v_hat[1];
    upp[2] = 0.0 - v_hat[1] * v_hat[2];
    unit3(upp);
    up_ask = sum3(perp[0] * upp[0], perp[1] * upp[1], perp[2] * upp[2]);
    for (int a = 0; a < 3; a++) across[a] = perp[a] - up_ask * upp[a];
    across_n = norm3(across[0], across[1], across[2]);
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
      unit3(d);
      sign = 1.0; need = size;
    }
    kh = fabs(roll) * 4.0;
    if (kh > 1.0) kh = 1.0;
    if (ka > kh) kh = ka;
    for (int a = 0; a < 3; a++) d[a] = (1.0 - kh) * d[a] + kh * upp[a];
    unit3(d);
    k_w = (((q * area) * fast) * fast) / mass;
    al_pos = (along > 0.0) ? along : 0.0;
    need_w = (1.0 - kh) * need + kh * ((up_ask > 0.0) ? up_ask : 0.0);
    sign_w = (kh > 0.5) ? 1.0 : sign;
    alpha = aeh_plane_alpha(k_w, cl_alpha, along, need_w, sign_w);
    {
      const double k_side = (((rho * side) * fast) * fast) / mass + al_pos;
      const double k_lat = (fabs(roll) > 1e-6) ? (0.35 * k_w + 0.65 * k_side) : k_w;

      a_knife = atan2(up_ask, k_side);
      k_pitch = (fabs(roll) > 1e-6) ? ((0.28 + 0.22 * ka) * kh) : ka;
      alpha += k_pitch * (a_knife - alpha);
      a_cross = clamp_d(across_n / ((k_lat > 1e-9) ? k_lat : 1e-9), -1e9, AERO_PASS_ALPHA_MAX)
                / ((across_n != 0.0) ? across_n : 1.0);
    }
    {
      const double slow_a = (1.0 - kh) * AERO_PASS_ALPHA_MAX * clamp_d((-sign_w * along) / gravity, 0.0, 1.0);
      if (slow_a > alpha) alpha = slow_a;
    }
    ca = cos(alpha); sa = sin(alpha);
    for (int a = 0; a < 3; a++)
    {
      nose[a] = (ca * v_hat[a] + sa * d[a]) + (kh * a_cross) * across[a];
      want_io[a] = (1.0 - w) * want_io[a] + w * nose[a];
    }
    unit3(want_io);
    for (int a = 0; a < 3; a++) y[a] = want_io[a];
    dy = sum3(d[0] * y[0], d[1] * y[1], d[2] * y[2]);
    for (int a = 0; a < 3; a++) back[a] = d[a] - dy * y[a];
    unit3(back);
    for (int a = 0; a < 3; a++) z[a] = -back[a];
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

API void aeh_trimmed_torques(const double turn[9], const double r_des[9], const double w_ff[3],
                             const double spin[3], const double turn_hat[3], const double inertia[3],
                             const double kp[3], const double kd[3], const double w_mom[6],
                             const double u[3], double out[3])
{
  double tau[3], own_m[3];

  aeh_so3_torques(turn, r_des, w_ff, spin, turn_hat, inertia, kp, kd, tau);
  aeh_wing_moments(w_mom, u, turn, spin, own_m);
  out[0] = tau[0] - own_m[0];
  out[1] = tau[1] - own_m[1];
  out[2] = tau[2] - own_m[2];
}

API double aeh_aerobat_attitude(const double w_plane[8], const double turn[9], const double acc[3],
                                const double u[3], double want_io[3], double filt_io[15], int flags[2],
                                const double scalars[10], double r_out[9])
{
  const double sign = scalars[0], yaw = scalars[1], rate = scalars[2], pace = scalars[3];
  const double dt = scalars[4], flip_ang = scalars[5], w_on = scalars[6];
  const double knife_eff = scalars[7], w_roll = scalars[8], w_whole = scalars[9];
  double       r_wing[9], r_des[9];
  int          has_wing = 0;
  double       k;

  if (w_on > 0.0)
  {
    has_wing = aeh_plane(w_plane, turn, acc, u, w_on, want_io, knife_eff, w_roll, r_wing);
  }
  aeh_smooth_target(filt_io, flags, want_io, sign, yaw, rate, pace, dt, flip_ang, r_des);
  k = w_whole * (double)(has_wing != 0);
  aeh_toward(r_des, (has_wing != 0) ? r_wing : r_des, k, r_out);
  return k;
}
