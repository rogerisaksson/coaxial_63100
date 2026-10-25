/** wbc_step.h - her whole body's step on the host (wbc_step.c, wbc_rounds.c): machine.wrench.step,
    numpy's arithmetic and numpy's own OpenBLAS (wbc_blas.h) operation for operation - her law's
    step the same bit for bit, where wbc_wrench.c is the firmware's own. Portable C11, the
    host's alone. */
#ifndef WBC_STEP_H
#define WBC_STEP_H

#include "wbc_blas.h"

/** The most dofs, driven joints, contacts bearing her; unknowns and rows. */
#define WBC_STEP_N 48
#define WBC_STEP_D 32
#define WBC_STEP_C 8
#define WBC_STEP_U (WBC_STEP_D + 3 * WBC_STEP_C)
#define WBC_STEP_R (2 * WBC_STEP_U + 2 * WBC_STEP_D + 40)
/** The workspace's doubles. */
#define WBC_STEP_WORK (8 * WBC_STEP_N * WBC_STEP_U + 12 * WBC_STEP_U * WBC_STEP_U + 2 * WBC_STEP_R * WBC_STEP_U \
                       + WBC_STEP_U * WBC_STEP_C + 2 * WBC_STEP_C)

/** The step: her body and machine.wrench's constants laid once; what is sensed of her
    (machine.wbc.sense) and what is asked (machine.wbc.step's ask) pointed at or copied each
    call; what it answers. */
typedef struct
{
  /* her body, laid once: dofs, driven, unknowns before the contacts (NU), each driven joint's
     dof, each leg's six and each knee among the driven, her upper body's, the held dofs, the
     passive and the active, the parallel pairs; the master insert's buffers (machine.wbc.Body's
     buffers): each one's driven joint, stop rad and side, 1 a low end; their width rad,
     stiffness 1/s^2, damping 1/s and reach s */
  int           n, nd, nu0, nupper, nheld, npass, nact, npairs, nbuffers;
  int           act[WBC_STEP_D], legs[2][6], knees[2], upper[WBC_STEP_D];
  int           held[WBC_STEP_N], passive[WBC_STEP_N], active[WBC_STEP_N], pairs[WBC_STEP_D][2];
  int           rounds, past, buffer_at[2 * WBC_STEP_D];
  double        buffer_stop[2 * WBC_STEP_D], buffer_side[2 * WBC_STEP_D], buffer[4];
  double        mass, moment[WBC_STEP_U], room[2], arms[WBC_STEP_D], yaw_m, damp2;
  double        w_com, w_turn, w_swing, w_knee, w_posture, w_height, w_moment, w_fold, w_stop, w_part, w_air;
  double        w_pin, w_keep, w_still, grow_n, ridge, contact_k, height_band, held_k, momentum_k, mu, g, slide_m_s;
  double        posture_kd[2], swing_kd[2], turn_kd[2];
  wbc_blas_t    blas;
  /* sensed: M, h, v, q (driven), com, k; each foot's R, quat, J6 (6, n), dJv, sole; the pelvis's
     and the trunk's quat, Jr (3, n), dJr, w; each contact's J (3, n), p, dJv, f */
  const double *M, *h, *v, *q, *com, *k;
  const double *R[2], *quat[2], *J6[2], *dJv[2], *sole[2];
  const double *tquat[2], *Jr[2], *dJr[2], *tw[2];
  int           nc;
  const double *cJ[WBC_STEP_C], *cp[WBC_STEP_C], *cdJv[WBC_STEP_C], *cf[WBC_STEP_C];
  int           cpatch[WBC_STEP_C]; /**< each contact's patch, the segments one surface bears (machine.borne) */
  /* asked */
  int           stance[2], nturns, kdot_given, swing[2], fold[2], wep, let_go[2];
  double        com_acc[3], turn[2][4], kdot[3], settle[2][6], edge[2], strides, borne, hold, mus[2], rests[2];
  double        slide[3], fz_cap, slid_cap; /**< the slide her contacts may take, world m/s; what bears her off her soles'
                                                cap, N; a sliding contact's, the slide's governor's */
  double        swing_at[2][3], swing_speed[2][3], swing_acc[2][3], swing_quat[2][4];
  double        fold_knee[2], fold_rate[2], fold_acc[2], knee_rate[2];
  const double *posture, *derate, *clamp, *wep_nm, *lo, *hi; /* lo, hi: each drive's back-EMF bounds */
  /* answered: the drives' torques clipped, every acceleration, each sole's load, its centre of
     pressure where it stands (cop_n of them, none where it bears nothing), each contact's force,
     the share of the ask carried, each drive's load of its clamp, which drives were asked past it */
  double        tau[WBC_STEP_D], qacc[WBC_STEP_N], bears[2], cop[2][2], forces[WBC_STEP_C][3], scale;
  double        load[WBC_STEP_D];
  int           cop_n, cop_none[2], over[WBC_STEP_D];
  double        work[WBC_STEP_WORK];
} wbc_step_t;

/** The step on `p`'s laid, sensed and asked: 0, or 1 where numpy would raise LinAlgError. */
int wbc_step(wbc_step_t *p);

/** machine.wrench._rounds: the unknowns `at` (nu) for the rows' normal matrix H0 and right-hand
    side g0, the contacts' held rows (S (3, nu) and r (3) each, nc of them) pressing on what they
    bear, her height's band lo to hi; geo and stride as machine.wrench lays them. 0 or 1 raised. */
typedef struct
{
  int           nu, k3;
  const double *moment, *part, *up, *lift, *stance, *kinetic, *caps, *slips, *frames[2];
  double        mass, grounded, strides, edges[2], mu[2];
} wbc_geo_t;

/** machine.wrench._patch: each contact's four rows (S, 4 a contact, r and r0): its point held still, along its
    up alone where it slides along the floor's plane past slide_m_s along the said slide, and its patch's normal
    force split as felt,
    its share pinned; what it feels along the patch's normal told (`felt`, N, nc) and its slip (`slips`, 3 a
    contact: sliding, along the world's x and z; wbc_patch.c). */
void wbc_patch_rows(const wbc_step_t *p, int n, int nu, int nc, const double *full, const double *full0, double *S,
                    double *r, double *r0, double *felt, double *slips);

int wbc_rounds(wbc_step_t *p, const wbc_geo_t *geo, const double *H0, const double *g0, const double *S,
               const double *r, const double *bears, double lo, double hi, double *at, double *work);

#endif
