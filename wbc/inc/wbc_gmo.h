/** wbc_gmo.h - her generalized momentum observer (machine.observer): the external force on her and
    each manifold's normal force of it, of her drives, encoders and IMU over her chain (wbc.h).
    The residual of her momentum over a law pass, M u' + h - S'tau less her drives' friction,
    low-passed; split over her patches' points by `rounds` least squares, each normal pulling
    pinned for the next. Laid by machine.observer from the kernel's registry. Portable C11. */
#ifndef WBC_GMO_H
#define WBC_GMO_H

#include "wbc.h"

/** Her patches' points; the unknowns, a sole's six and a push's one; each point's shares of the
    manifolds; the manifolds. */
#define WBC_GMO_POINTS    16
#define WBC_GMO_F         26
#define WBC_GMO_SHARES    18
#define WBC_GMO_MANIFOLDS 5

typedef struct
{
  int         rounds;                      /**< the split's solves */
  double      alpha;                       /**< the low-pass's bandwidth, rad/s */
  double      slip;                        /**< her drives' friction tanh(u / slip), rad/s */
  double      eps;                         /**< the split's tie-break, of its largest curvature */
  double      loss[WBC_N];                 /**< each velocity's friction loss, N m */
  int         driven_at[WBC_DRIVEN];       /**< each drive's velocity, and its torque in her edge's */
  int         driven_joint[WBC_DRIVEN];
  int         link[WBC_GMO_POINTS];        /**< each point's link, its place in it, its outward */
  double      local[WBC_GMO_POINTS][3];    /**< normal there - none a sole's whole wrench -, */
  double      normal[WBC_GMO_POINTS][3];   /**< its first unknown */
  int         sole[WBC_GMO_POINTS];
  int         col[WBC_GMO_POINTS];
  int         share_point[WBC_GMO_SHARES]; /**< a share: its point, its manifold, the link whose */
  int         share_manifold[WBC_GMO_SHARES]; /**< frame holds the manifold's normal, that normal */
  int         share_link[WBC_GMO_SHARES];
  double      share_axis[WBC_GMO_SHARES][3];
  double      tau[WBC_DRIVEN];             /**< the drives' torques and the pelvis's acceleration */
  double      acc[3];                      /**< (world) summed over the passes since the law's */
  double      span;
  double      primed;                      /**< 0 before a first velocity, then 1 */
  double      u[WBC_N];                    /**< the last law pass's velocities */
  double      r[WBC_N];                    /**< out: the external generalized force, low-passed */
  double      f[WBC_GMO_F];                /**< out: each point's force */
  double      force[WBC_GMO_MANIFOLDS];    /**< out: each manifold's normal force, N */
  double      at[WBC_GMO_POINTS][3];       /**< out: each point in the world */
  wbc_body_t  body;
  double      a[WBC_N][WBC_GMO_F];
  double      h[WBC_GMO_F][WBC_GMO_F];
  double      k[WBC_GMO_F][WBC_GMO_F];
} wbc_gmo_t;

/** A drive pass of `dt` s into the sums, of her edge's array: the drives' torques (`tau`, every
    joint's), the pelvis's specific force (its own frame) turned by its quaternion (w, x, y, z) to
    the world, gravity added. */
void wbc_gmo_felt(wbc_gmo_t *o, const double quat[4], const double accel[3], const double *tau, double dt);

/** A law pass at her estimate - the pelvis's frame, the hinges, u -, `dt` s on: r, f, force; the
    sums emptied. */
void wbc_gmo_step(wbc_gmo_t *o, const wbc_frame_t *base, const double q[WBC_N - 6], const double u[WBC_N], double dt);

#endif
