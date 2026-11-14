/** wbc_stride.h - her stride in C under her ask (wbc_imp.h): machine.law.stride's steps (wbc_stride.c,
    wbc_floor.c), numpy's arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#ifndef WBC_STRIDE_H
#define WBC_STRIDE_H

#include "wbc_imp.h"

/** machine.law.stride.swings: how far round her rhythm each limb swings, rad - SWING of a turn to
    pace_m_s of her drive's pace, swing_fast from pace_fast_m_s, in line between. */
double wbc_imp_swing_w(const wbc_imp_t *k, double drive, double turns, double going);
/** machine.law.stride.beat: (stepping, way, swung) out, each limb's sweep, phi, on, off, u, flying. */
void wbc_imp_beat(const wbc_imp_t *k, double drive, double psi, double sway, double lateral, double turns,
                  const double was[2], double need, const double lead[2], const double split[2], const double begun[2],
                  const double stood[2], double phi_w, double thrown, double *stepping, double *way,
                  double *swung, double sweep[2], double phi[2], double on[2], double off[2], double u[2],
                  double flying[2]);

/** machine.law.stride.hurried, cruises, meets, rocks, folds, lates, goes. */
void   wbc_imp_hurried(const wbc_imp_t *k, const double aside[2], double om, const double u[2], const double rate[2],
                       const double kp[2], double hurry[2]);
double wbc_imp_cruises(const wbc_imp_t *k, double way, double drive, double turns, double going, double phi_w);
void   wbc_imp_meets(const wbc_imp_t *k, const double xi[2], const double c[3], const double flat[2][2],
                     double cruise, double om, double turns, double wide, double across, double reach,
                     double apart, const double planted[2], double meet[2][2]);
void   wbc_imp_guarded(const wbc_imp_t *k, const double meet[2][2], const double over[2], const double flat[2][2],
                       double apart, const double planted[2], double held[2][2]);
/** machine.law.stride.ducked: her height's sidechain, (latch, envelope) `held` in place, the duck and its rate, m. */
void   wbc_imp_ducked(const double duck_p[3], double held[2], const double stance[2], const double stood[2], double dt,
                      double duck[2]);
void   wbc_imp_rocks(const wbc_imp_t *k, double rock, double rate, double acc, double tip[2], double tipping[2],
                     double tipped[2]);
void   wbc_imp_folds(const wbc_imp_t *k, const double off[2], const double loads[2], double bent, double fold[2],
                     double shorter[2]);
void   wbc_imp_lates(const wbc_imp_t *k, const double phi[2], double turns, double stepping, double phi_w, double late[2]);
void   wbc_imp_goes(const wbc_imp_t *k, const double meet[2][2], const double flat[2][2], const double shod[2][3],
                    const double v[3], const double u[2], const double off[2], const double turns[2],
                    const double kp[2], const double kd[2], double phi_w, double acc[2][2]);

/** machine.law.stride.pitched, of each sole's floor. */
void wbc_imp_pitched(const wbc_imp_t *k, const double ground[2], double pitched[2]);

/** machine.law.stride.heels: the rise asked, flatten, the edge and rounds (2, 3) out. `place` the
    soles' places (2, 3), `tilt` and `tilting` their pitch and its rate, `leg` a leg's length. */
void wbc_imp_heels(const wbc_imp_t *k, const double place[2][3], const double tilt[2], const double tilting[2],
                   const double hips[2], const double floors[2], double leg, const double shorter[2],
                   const double off[2], const double tip[2], const double stance[2], double stepping,
                   const double was[2], double dt, const double ground[2], double rocked[2], double flatten[2],
                   double edge[2], double rounds[2][3], double reach_h[2]);
/** machine.law.stride.vaults: each standing leg's hip's acceleration up, m/s^2: over a flat sole the arc about
    the ankle at the length the axial spring asks (`pressed`); its heel asked up (`heeled`), the hip held at the
    height the heel rises for (`reach_h`, heels'), on that spring (`kp`, `kd`). `leg` hip from ankle, `len`,
    `lrate`; `high` the hips over the floor. */
void wbc_imp_vaults(const wbc_imp_t *k, const double leg[2][3], const double len[2], const double lrate[2],
                    const double v[3], const double shod[2][3], const double along[2], double propped, double pressed,
                    double hips_by, double low, const double high[2], const double reach_h[2], const double duck[2],
                    const double heeled[2], const double striding[2], double asked, double kp, double kd, double risen[2]);

/** machine.law.stride.clears: a swinging sole's middle over the floor, its rate, its acceleration. */
void wbc_imp_clears(const wbc_imp_t *k, const double u[2], const double flying[2], const double swung[2],
                    const double high[2], const double tip[2], const double tipping[2], const double tilt[2],
                    const double late[2], const double land[2], double clear[3][2]);
void wbc_imp_footholds(const wbc_imp_t *k, const double *along, const double *rise, int told, const double s_meet[2],
                       const double s_fore[2], const double s_now[2], double s_com, double reach, const double hold[2],
                       double gap, double step, double s_new[2], double top[2], double land[2], double run[2], double cell[2]);
double wbc_imp_interp(double x, const double *along, const double *rise, int n);
void wbc_imp_landings(const wbc_imp_t *k, const double *y, const double *fr, double *a, const double *rot, const double c[3],
                      const double flat[2][2], const double xi[2], double *cruise, double om, double turns, double wide,
                      double across, double reach, const double stood[2], const double u[2], const double flying[2],
                      double meet[2][2], double top[2], double land[2], double adj[2], double over[2],
                      double flown, double *apart);
double wbc_imp_floor_at(const double *along, const double *rise, int told, double s);
double wbc_imp_floor_mean(const wbc_imp_t *k, const double *along, const double *rise, int told, double s0, double s1);

/** machine.law.stride.flown: which sole's her stride is with none standing - the one `strode` alone the
    pass before, the one `off` as the other left the floor (`stood`) with her centre going `up`, while
    `thrown`; else the one further on its swing (`u`). */
void wbc_imp_flown(const double strode[2], const double off[2], const double stood[2], const double u[2], double thrown,
                   double up, double flown[2]);

#endif
