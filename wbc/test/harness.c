/** harness.c - A flat C API over wbc/, so test_wbc_core.py can run the real chain on the host
    through ctypes beside MuJoCo. */
#include "wbc.h"
#include "wbc_imp.h"
#include "wbc_law.h"
#include "wbc_np.h"
#include "wbc_step.h"
#include "wbc_pass.h"

#include <string.h>
#include <time.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

static wbc_body_t body;

static void frame_of(const double *flat, wbc_frame_t *f)
{
  memcpy(f->r, flat, 9U * sizeof(double));
  memcpy(f->p, flat + 9, 3U * sizeof(double));
}

API int wbh_links(void)
{
  return WBC_LINKS;
}

API int wbh_n(void)
{
  return WBC_N;
}

API const char *wbh_name(int link)
{
  return wbc_name[link];
}

API int wbh_parent(int link)
{
  return wbc_parent[link];
}

/** Every link's frame in the world, 12 doubles each, from the pelvis's and the hinges' rad. */
API void wbh_pose(const double *base, const double *q, double *t)
{
  wbc_frame_t f;

  frame_of(base, &f);
  wbc_body_pose(&body, &f, q);
  for (int i = 0; i < WBC_LINKS; i++)
  {
    memcpy(t + 12 * i, body.t[i].r, 9U * sizeof(double));
    memcpy(t + 12 * i + 9, body.t[i].p, 3U * sizeof(double));
  }
}

API void wbh_mass(double *m)
{
  wbc_body_mass(&body);
  memcpy(m, body.m, sizeof(body.m));
}

API void wbh_bias(const double *u, double *h)
{
  wbc_body_bias(&body, u);
  memcpy(h, body.h, sizeof(body.h));
}

API void wbh_point(int link, const double *r, double *out)
{
  wbc_body_point(&body, link, r, out);
}

API void wbh_jacobian(int link, const double *r, double *j)
{
  wbc_body_jacobian(&body, link, r, (double (*)[WBC_N])j);
}

/** Seconds a step: the pose, the mass matrix, the bias and two links' Jacobians, over reps. */
API double wbh_seconds(const double *base, const double *q, const double *u, int a, int b,
                       int reps)
{
  static double j[6][WBC_N];
  const double  o[3] = {0.0, 0.0, 0.0};
  wbc_frame_t   f;
  clock_t       t0;

  frame_of(base, &f);
  t0 = clock();
  for (int k = 0; k < reps; k++)
  {
    wbc_body_pose(&body, &f, q);
    wbc_body_mass(&body);
    wbc_body_bias(&body, u);
    wbc_body_jacobian(&body, a, o, j);
    wbc_body_jacobian(&body, b, o, j);
  }
  return (double)(clock() - t0) / (double)CLOCKS_PER_SEC / (double)reps;
}

API void wbh_com(double *com, double *j)
{
  wbc_body_com(&body, com, (double (*)[WBC_N])j);
}

API void wbh_drift(int link, const double *r, double *out)
{
  wbc_body_drift(&body, link, r, out);
}

API void wbh_momentum(const double *com, double *k)
{
  wbc_body_momentum(&body, com, k);
}

static wbc_stack_t stack;

API void wbh_stack_init(void)
{
  wbc_stack_init(&stack);
}

/** One tick of the loop from the pelvis's frame, the hinges' rad and u, for `ask`. */
API void wbh_stack(const double *base, const double *q, const double *u, const wbc_ask_t *ask,
                   wbc_out_t *out)
{
  wbc_frame_t f;

  frame_of(base, &f);
  wbc_stack_step(&body, &stack, &f, q, u, ask, out);
}

/** Seconds a tick of the loop, over reps. */
API double wbh_stack_seconds(const double *base, const double *q, const double *u,
                             const wbc_ask_t *ask, int reps)
{
  static wbc_out_t out;
  wbc_frame_t      f;
  clock_t          t0;

  frame_of(base, &f);
  t0 = clock();
  for (int k = 0; k < reps; k++)
  {
    wbc_stack_step(&body, &stack, &f, q, u, ask, &out);
  }
  return (double)(clock() - t0) / (double)CLOCKS_PER_SEC / (double)reps;
}

static wbc_wrench_t wrench;

API void wbh_wrench_init(void)
{
  wbc_wrench_init(&wrench);
}

/** One tick of the step without an active set, as wbh_stack. */
API void wbh_wrench(const double *base, const double *q, const double *u, const wbc_ask_t *ask,
                    wbc_out_t *out)
{
  wbc_frame_t f;

  frame_of(base, &f);
  wbc_wrench_step(&body, &wrench, &f, q, u, ask, out);
}

/** Seconds a tick of it, over reps. */
API double wbh_wrench_seconds(const double *base, const double *q, const double *u,
                              const wbc_ask_t *ask, int reps)
{
  static wbc_out_t out;
  wbc_frame_t      f;
  clock_t          t0;

  frame_of(base, &f);
  t0 = clock();
  for (int k = 0; k < reps; k++)
  {
    wbc_wrench_step(&body, &wrench, &f, q, u, ask, &out);
  }
  return (double)(clock() - t0) / (double)CLOCKS_PER_SEC / (double)reps;
}

API int wbh_ortho_row_size(void)
{
  return (int)sizeof(ortho_row_t);
}

/** One orthokinematic row from its wire bytes into `head` (id, at, blend, flags) and `p`. */
API void wbh_ortho_row(const void *wire, uint32_t *head, float *p)
{
  ortho_row_t row;

  memcpy(&row, wire, sizeof(row));
  head[0] = row.id;
  head[1] = row.at;
  head[2] = row.blend;
  head[3] = row.flags;
  memcpy(p, row.p, sizeof(row.p));
}

API void wbh_pass_drives(wbc_drives_t *p, double dt, double wo, double wo3, double ys, int in_hop, int wb)
{
  wbc_pass_drives(p, dt, wo, wo3, ys, in_hop, wb);
}

API void wbh_pass_ctrl(wbc_world_t *w, double span, const double *ahead, const double *wrate)
{
  wbc_pass_ctrl(w, span, ahead, wrate);
}

API void wbh_pass_motor(wbc_world_t *w, double step_s)
{
  wbc_pass_motor(w, step_s);
}

API void wbh_pass_metered(wbc_world_t *w, const double *speed, double step_s)
{
  wbc_pass_metered(w, speed, step_s);
}

API void wbh_pass_felt(const wbc_world_t *w, double *y, int quat_at, int gyro_at, int accel_at, int q_at,
                       int qd_at, int qm_at)
{
  wbc_pass_felt(w, y, quat_at, gyro_at, accel_at, q_at, qd_at, qm_at);
}

API void wbh_pass_contacts(wbc_contacts_t *c, int ncon, const char *pos, const double *efc_force)
{
  wbc_pass_contacts(c, ncon, pos, efc_force);
}

API void wbh_pass_sensed(const wbc_contacts_t *c, double *y, int load_at, int touch_at, int press_at, int cop_at,
                         int cells_at, int at_at)
{
  wbc_pass_sensed(c, y, load_at, touch_at, press_at, cop_at, cells_at, at_at);
}

API void wbh_pass_heat_load(wbc_heat_t *h, const double *torques, const double *qd)
{
  wbc_pass_heat_load(h, torques, qd);
}

API void wbh_pass_heat(wbc_heat_t *h, double dt, const double *air, const double *rds, int envelope)
{
  wbc_pass_heat(h, dt, air, rds, envelope);
}

API void wbh_pass_heat_peak(const wbc_heat_t *h, double *out)
{
  wbc_pass_heat_peak(h, out);
}

API double wbh_pass_drawn(const wbc_world_t *w, const double *tau, const double *speed, const double *loss,
                          const unsigned char *driven, const double *testing, const double *ohm,
                          double housekeeping_w, double switching_w)
{
  return wbc_pass_drawn(w, tau, speed, loss, driven, testing, ohm, housekeeping_w, switching_w);
}

API void wbh_imp_ask(const wbc_imp_t *k, const double *out, const double *y, const double *fr, double *a)
{
  wbc_imp_ask(k, out, y, fr, a);
}

API int wbh_imp_size(void)
{
  return (int)sizeof(wbc_imp_t);
}

API double wbh_imp_surety(const wbc_imp_t *k, const wbc_surety_t *s, double prev, double dt, double ct, double t,
                          double *vec)
{
  return wbc_imp_surety(k, s, prev, dt, ct, t, vec);
}

API int wbh_surety_size(void)
{
  return (int)sizeof(wbc_surety_t);
}

API void wbh_touch_step(const wbc_imp_t *k, const wbc_touch_t *t, const double *asked, double *twist, double tw_on,
                        double heading, double dt, const double *vel)
{
  wbc_touch_step(k, t, asked, twist, tw_on, heading, dt, vel);
}

API int wbh_touch_size(void)
{
  return (int)sizeof(wbc_touch_t);
}

API void wbh_twist_step(const wbc_imp_t *k, const wbc_twist_t *t, double dt, const double *vel, const double *held,
                        const double *prior)
{
  wbc_twist_step(k, t, dt, vel, held, prior);
}

API int wbh_twist_size(void)
{
  return (int)sizeof(wbc_twist_t);
}

API void wbh_wait_held(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *a, const double *out,
                       double t, double dt, double held_s, double *got)
{
  wbc_wait_held(k, w, tab, n, a, out, t, dt, held_s, got);
}

API void wbh_wait_risen(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *y, const double *fr,
                        const double *a, const double *st, double t, double dt, double prop_until, double *got)
{
  wbc_wait_risen(k, w, tab, n, y, fr, a, st, t, dt, prop_until, got);
}

API int wbh_wait_size(void)
{
  return (int)sizeof(wbc_wait_t);
}

API int wbh_wait_rows(void)
{
  return WBC_WAIT_ROWS;
}

API void wbh_brace_scaled(const wbc_brace_t *b, const double *M, const double *w_up, double *Ms)
{
  wbc_brace_scaled(b, M, w_up, Ms);
}

API void wbh_brace_blend(const wbc_brace_t *b, const double *M, const double *Ms, const double *q, const double *v,
                         const double *h, const double *posture, double *qacc, double *tau, unsigned char *over,
                         double sw, double bw, double d_ax, double d_lat, double w_hop, const double *w_up,
                         const double *stance, double *jeff, double *leg_kp)
{
  wbc_brace_blend(b, M, Ms, q, v, h, posture, qacc, tau, over, sw, bw, d_ax, d_lat, w_hop, w_up, stance, jeff, leg_kp);
}

API int wbh_brace_size(void)
{
  return (int)sizeof(wbc_brace_t);
}

API void wbh_law_latent(const wbc_law_t *k, double *st, const double *tab, int ntab, double now, double dt,
                        double *out, double *hit)
{
  wbc_law_latent(k, st, tab, ntab, now, dt, out, hit);
}

API void wbh_law_couple(const wbc_law_t *k, const double *q, const double *qd, const double *ref_q,
                        const double *ref_v, double *eso_z, double *out, double *dq)
{
  wbc_law_couple(k, q, qd, ref_q, ref_v, eso_z, out, dq);
}

API int wbh_law_size(void)
{
  return (int)sizeof(wbc_law_t);
}

/** numpy's primitives as her law's C computes them, `n` draws: 0 a (3, 3) @ (3,), 1 a (2, 2) @ (2,),
    2 a (2, 3) @ (3, 3).T and 3 @ (3, 3), 4 a 1-d dot of 3, 5 einsum 'nia,ia->n' of six. */
API void wbh_np_probe(int which, const double *a, const double *b, double *out, int n)
{
  for (int k = 0; k < n; k++)
  {
    const double *x = a + 9 * k, *y = b + 9 * k;
    double       *o = out + 9 * k;

    switch (which)
    {
    case 0:
      for (int j = 0; j < 3; j++)
      {
        o[j] = npmv3(x + 3 * j, y);
      }
      break;
    case 1:
      o[0] = npmv2(x, y);
      o[1] = npmv2(x + 2, y);
      break;
    case 2:
    case 3:
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          o[3 * i + j] = which == 2 ? npmm3(x + 3 * i, y + 3 * j, 1) : npmm3(x + 3 * i, y + j, 3);
        }
      }
      break;
    case 4:
      o[0] = npdot(x, 1, y, 1, 3);
      break;
    case 5:
      o[0] = npein6(x, y);
      break;
    default:
      break;
    }
  }
}

/** libm as her law's C calls it, side by side in one body: of in (n, 5) - x, y, x within [-1, 1],
    x within [-50, 50], |x| + 1e-3 - out (n, 10) sin x, cos x, tan x, asin, acos, atan2(x, y), exp,
    log, hypot(x, y), pow(x, 2). */
API void wbh_np_libm(int n, const double *in, double *out)
{
  for (int k = 0; k < n; k++)
  {
    const double *x = in + 5 * k;
    double       *o = out + 10 * k;

    o[0] = sin(x[0]);
    o[1] = cos(x[0]);
    o[2] = tan(x[0]);
    o[3] = asin(x[2]);
    o[4] = acos(x[2]);
    o[5] = atan2(x[0], x[1]);
    o[6] = exp(x[3]);
    o[7] = log(x[4]);
    o[8] = hypot(x[0], x[1]);
    o[9] = pow(x[0], 2.0);
  }
}

API int wbh_step(wbc_step_t *p)
{
  return wbc_step(p);
}

/** wbc_step_t's sizes: the most dofs, driven, contacts, unknowns, rows, its workspace, its bytes. */
API void wbh_step_sizes(int *out)
{
  out[0] = WBC_STEP_N;
  out[1] = WBC_STEP_D;
  out[2] = WBC_STEP_C;
  out[3] = WBC_STEP_U;
  out[4] = WBC_STEP_R;
  out[5] = WBC_STEP_WORK;
  out[6] = (int)sizeof(wbc_step_t);
}

API void wbh_pass_hem(int n, double dt, const double *ap, const double *om, const double *alb, const double *c,
                      double *state, double *q, double *v, double *a, unsigned char *hit)
{
  wbc_pass_hem(n, dt, ap, om, alb, c, state, q, v, a, hit);
}
API void wbh_sil_init(wbc_world_t *w, int nbuses, const int *counts, const int *joints)
{
  wbc_sil_init(w, nbuses, counts, joints);
}

API int wbh_sil_transceive(wbc_world_t *w, int bus_id, const unsigned char *frame, int len)
{
  return wbc_sil_transceive(w, bus_id, frame, len);
}

API void wbh_sil_drive(wbc_world_t *w, int n_idx, const int *idx, const double *target,
                       const double *feed, const double *kp, const double *limit)
{
  wbc_sil_drive(w, n_idx, idx, target, feed, kp, limit);
}

API void wbh_sil_stats(int bus_id, unsigned long long *counts, double *wire_s,
                       unsigned char *last_frame, int *last_len)
{
  wbc_sil_stats(bus_id, counts, wire_s, last_frame, last_len);
}
