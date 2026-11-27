/** harness.c - aero/'s sizes, a row of its memory, and host wing/observer passes (tools/cores/aero.py). */
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
  {
    const double y0 = sum3(m[0][0] * push_hat[0], m[1][0] * push_hat[1], m[2][0] * push_hat[2]);
    const double y2 = sum3(m[0][2] * push_hat[0], m[1][2] * push_hat[1], m[2][2] * push_hat[2]);
    const double t00 = (m[0][0] * m[0][0] + m[1][0] * m[1][0]) + m[2][0] * m[2][0];
    const double t22 = (m[0][2] * m[0][2] + m[1][2] * m[1][2]) + m[2][2] * m[2][2];
    const double reg = wind_reg + 0.1 * (t00 + t22);
    const double s00 = t00 + reg, s22 = t22 + reg;
    const double s02 = sum3(m[0][0] * m[0][2], m[1][0] * m[1][2], m[2][0] * m[2][2]);
    const double scale = (dt * wo_wind) / (s00 * s22 - s02 * s02);
    const double dw0 = (s22 * y0 - s02 * y2) * scale;
    const double dw2 = (s00 * y2 - s02 * y0) * scale;

    wind[0] += dw0;
    wind[2] += dw2;
    for (int b = 0; b < 3; b++)
    {
      push_hat[b] -= m[b][0] * dw0 + m[b][2] * dw2;
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

/** Combined aerobat observer tail (`aerobat.observer.observe`). */
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
