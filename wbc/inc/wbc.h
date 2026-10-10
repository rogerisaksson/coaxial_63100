/** wbc.h - her chain on static arrays (wbc_model.h): poses, Jacobians, mass matrix and bias,
    the product of exponentials over twists (w, v) in body frames. Portable C11. */
#ifndef WBC_H
#define WBC_H

#include "wbc_model.h"

/** A frame: R by rows, then p. */
typedef struct
{
  double r[9];
  double p[3];
} wbc_frame_t;

/** Her body at one configuration, laid once; filled by the steps below, pose first. */
typedef struct
{
  wbc_frame_t t[WBC_LINKS];        /**< each link's frame in the world */
  wbc_frame_t l[WBC_LINKS];        /**< each link's frame in its parent's */
  double      a[WBC_LINKS][36];    /**< each link's adjoint from its parent's frame to its own */
  double      q[WBC_N - 6];        /**< the hinges' angles */
  double      axis[WBC_LINKS][3];  /**< each hinge's axis in the world */
  double      anchor[WBC_LINKS][3];/**< a point on it */
  double      v[WBC_LINKS][6];     /**< each link's twist in its own frame */
  double      vd[WBC_LINKS][6];    /**< its acceleration, gravity's included */
  double      f[WBC_LINKS][6];     /**< the wrench it passes to its parent */
  double      gc[WBC_LINKS][36];   /**< the inertia of all it carries, composite */
  double      sub_m[WBC_LINKS];    /**< the mass it and its descendants carry */
  double      sub_mc[WBC_LINKS][3];/**< that mass times its centre, world */
  double      m[WBC_N][WBC_N];     /**< the mass matrix */
  double      h[WBC_N];            /**< the bias C u + g less the springs and dampers */
} wbc_body_t;

/** The frames from the pelvis's pose in the world and the hinges' angles, rad. */
void wbc_body_pose(wbc_body_t *b, const wbc_frame_t *base, const double q[WBC_N - 6]);

/** The mass matrix at the pose, the armatures on its diagonal. */
void wbc_body_mass(wbc_body_t *b);

/** The bias at the pose under u - the pelvis's twist in its own frame, then the hinges' rad/s:
    C u + g, and the springs' and dampers' torques against it. */
void wbc_body_bias(wbc_body_t *b, const double u[WBC_N]);

/** A point r of `link`, in its frame, placed in the world. */
void wbc_body_point(const wbc_body_t *b, int link, const double r[3], double out[3]);

/** The point's Jacobian over u in the world: rows w, then v. */
void wbc_body_jacobian(const wbc_body_t *b, int link, const double r[3], double j[6][WBC_N]);

/** Her centre of mass in the world and its Jacobian over u, at the pose. */
void wbc_body_com(wbc_body_t *b, double com[3], double j[3][WBC_N]);

/** The point's J-dot u in the world, rows w then v: its acceleration were u held (after the bias). */
void wbc_body_drift(const wbc_body_t *b, int link, const double r[3], double out[6]);

/** Her angular momentum about `com`, world (after the bias). */
void wbc_body_momentum(const wbc_body_t *b, const double com[3], double k[3]);

/** The commanded torques: the drives', then the held joints'. */
#define WBC_X (WBC_DRIVEN + WBC_HELD)
/** The most contact rows: two soles standing. */
#define WBC_C 12
/** The levels, and the most rows one lays. */
#define WBC_LEVELS 4
#define WBC_ROWS 64
/** A level of this many rows or fewer is solved by its Gram matrix. */
#define WBC_SMALL 16

/** What the loop is asked, R^k: a sole bears or swings; the centre of mass's acceleration; the
    pelvis's and the trunk's turns; a swinging sole's place, speed, acceleration and turn; a
    swinging knee's fold; the posture; each drive's derate; war emergency power; the angular
    momentum's rate, or none to bleed it. World frame; quaternions (w, x, y, z); the drives in
    wbc_driven's order. */
typedef struct
{
  int    stance[2];
  double com_acc[3];
  double turn[2][4];
  int    swing[2];
  double swing_at[2][3];
  double swing_speed[2][3];
  double swing_acc[2][3];
  double swing_quat[2][4];
  int    fold[2];
  double fold_knee[2];
  double fold_rate[2];
  double fold_acc[2];
  double posture[WBC_DRIVEN];
  double derate[WBC_DRIVEN];
  int    wep;
  int    kdot_given;
  double kdot[3];
} wbc_ask_t;

/** What the loop answers: the drives' torques; every acceleration (u's order); each sole's
    wrench (m, f) in the world and its load; of each level, the share the polytope let
    through; the drives asked past their clamps, each drive's load of its clamp, and the
    inertia each drive's torque meets. */
typedef struct
{
  double tau[WBC_DRIVEN];
  double udot[WBC_N];
  double wrench[2][6];
  double bears[2];
  double alpha[WBC_LEVELS];
  int    over[WBC_DRIVEN];
  double load[WBC_DRIVEN];
  double jeff[WBC_DRIVEN];        /**< each drive's effective inertia under the body and the contacts, kg m^2 */
} wbc_out_t;

/** The loop's memory, laid once. */
typedef struct
{
  double chol[WBC_N][WBC_N];      /**< M's Cholesky factor, lower */
  double w[WBC_N][WBC_X];         /**< M^-1 S^T */
  double w0[WBC_N];               /**< -M^-1 h */
  double jc[WBC_C][WBC_N];        /**< the standing soles' Jacobians */
  double c[WBC_C];                /**< their accelerations asked: still */
  double y[WBC_N][WBC_C];         /**< M^-1 Jc^T */
  double lam[WBC_C][WBC_C];       /**< (Jc M^-1 Jc^T)^-1 */
  double bb[WBC_N][WBC_X];        /**< udot = u0 + bb tau */
  double u0[WBC_N];
  double ll[WBC_C][WBC_X];        /**< lambda = l0 + ll tau */
  double l0[WBC_C];
  double a[WBC_ROWS][WBC_X];      /**< a level's rows over tau */
  double r[WBC_ROWS];
  double ab[WBC_ROWS][WBC_X];     /**< its rows in the null space so far */
  double rho[WBC_ROWS];
  double nn[WBC_X][WBC_X];        /**< the null space so far, a projector */
  double g[WBC_SMALL][WBC_SMALL]; /**< a small level's Gram matrix, then its factor */
  double qq[WBC_SMALL][WBC_X];
  double hh[WBC_X][WBC_X];        /**< the last level's normal equations */
  double tau[WBC_LEVELS][WBC_X];  /**< each level's contribution */
  double load[WBC_DRIVEN];        /**< the last step's load of each clamp: WEP where it ran near */
  double com[3];
  double jcom[3][WBC_N];
  double jac[6][WBC_N];
  int    rows;
  int    nc;
} wbc_stack_t;

void wbc_stack_init(wbc_stack_t *s);

/** One tick: her pose and velocity in, the ask in, the torques and the rest out. */
void wbc_stack_step(wbc_body_t *b, wbc_stack_t *s, const wbc_frame_t *base,
                    const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                    wbc_out_t *out);

#endif
