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
  double      v[WBC_LINKS][6];     /**< each link's twist in its own frame */
  double      vd[WBC_LINKS][6];    /**< its acceleration, gravity's included */
  double      f[WBC_LINKS][6];     /**< the wrench it passes to its parent */
  double      gc[WBC_LINKS][36];   /**< the inertia of all it carries, composite */
  double      m[WBC_N][WBC_N];     /**< the mass matrix */
  double      h[WBC_N];            /**< the bias C u + g */
} wbc_body_t;

/** The frames from the pelvis's pose in the world and the hinges' angles, rad. */
void wbc_body_pose(wbc_body_t *b, const wbc_frame_t *base, const double q[WBC_N - 6]);

/** The mass matrix at the pose, the armatures on its diagonal. */
void wbc_body_mass(wbc_body_t *b);

/** The bias at the pose under u: the pelvis's twist in its own frame, then the hinges' rad/s. */
void wbc_body_bias(wbc_body_t *b, const double u[WBC_N]);

/** A point r of `link`, in its frame, placed in the world. */
void wbc_body_point(const wbc_body_t *b, int link, const double r[3], double out[3]);

/** The point's Jacobian over u in the world: rows w, then v. */
void wbc_body_jacobian(const wbc_body_t *b, int link, const double r[3], double j[6][WBC_N]);

#endif
