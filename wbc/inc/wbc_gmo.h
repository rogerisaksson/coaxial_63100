/** wbc_gmo.h - the robot's generalized momentum observer (machine.senses.observer): the external force on it and
    each manifold's normal force of it, of the robot's drives, encoders and IMU over its chain
    (wbc.h). The residual of the robot's momentum over a law pass, M u' + h - S'tau less the robot's
    drives' friction, low-passed; split by least squares with every unknown pushing (Lawson-Hanson): the
    robot's patches' points - a sole's four corners in its law's pyramid -, a joint on its stop
    pushed off it, the robot's capsules touching each other pushed apart. Laid by machine.senses.observer
    from the kernel's registry and the robot's model. A pass works on its working set alone
    (working_envelope_scoper.lisp): the unknowns that can push, the pairs staged within the robot's
    envelope; the same bits as the whole. Portable C11. */
#ifndef WBC_GMO_H
#define WBC_GMO_H

#include "wbc.h"

#include <stdalign.h>

/** The robot's patches' points; a point's pushes at most (a pyramid's edges), all of them at most;
    its joints' stops' pushes, one a hinge; its capsules, the pairs of them that can touch, the
    touches a pass at most; the unknowns; each point's shares of the manifolds; the manifolds. */
#define WBC_GMO_POINTS    24
#define WBC_GMO_EDGES     4
#define WBC_GMO_PUSHES    72
#define WBC_GMO_CAPSULES  48
#define WBC_GMO_PAIRS     1024
#define WBC_GMO_TOUCH     8
#define WBC_GMO_F         (WBC_GMO_PUSHES + WBC_N - 6 + WBC_GMO_TOUCH)
#define WBC_GMO_SHARES    30
/** The friction band's rounds at most a pass. */
#define WBC_GMO_ROUNDS    8
#define WBC_GMO_MANIFOLDS 8
/** A row of the split's matrices padded to whole 64-byte lines: its unknowns' columns, none past them read. */
#define WBC_GMO_LD        (((WBC_GMO_F) + 7) / 8 * 8)

typedef struct
{
  alignas(64) double a[WBC_N][WBC_GMO_LD]; /**< hot, first, each row whole 64-byte lines: J' (the robot's velocities' rows), */
  double      h[WBC_GMO_F][WBC_GMO_LD];    /**< the split's normal equations and their factor, its weighed rows */
  double      k[WBC_GMO_F][WBC_GMO_LD];    /**< gathered and its gradient over the robot's working set */
  double      ag[WBC_N][WBC_GMO_LD];
  double      w[WBC_GMO_LD];
  double      alpha;                       /**< the low-pass's bandwidth, rad/s */
  double      slip;                        /**< the robot's drives' friction tanh(u / slip), rad/s */
  double      eps;                         /**< the split's tie-break, of its largest curvature */
  double      weight;                      /**< the robot's weight, N; gravity's prior's sharpness on a */
  double      upright;                     /**< push's cosine with the vertical, and its reach */
  double      reach;                       /**< over the robot's lowest point, m; a support's speed, m/s: */
  double      still;                       /**< past it a point does not stand on what bears the robot */
  double      slide[3];                    /**< the robot's text's slide, world m/s: moving with it a point stands still */
  double      horizon;                     /**< the robot's envelope, m: a pair staged nearer is searched a pass */
  double      loss[WBC_N];                 /**< each velocity's friction loss, N m */
  int         driven_at[WBC_DRIVEN];       /**< each drive's velocity, and its torque in the robot's edge's */
  int         driven_joint[WBC_DRIVEN];
  int         link[WBC_GMO_POINTS];        /**< each point's link, its place in it, its pushes' */
  double      local[WBC_GMO_POINTS][3];    /**< directions there and how many, its first unknown */
  double      edge[WBC_GMO_POINTS][WBC_GMO_EDGES][3];
  int         edges[WBC_GMO_POINTS];
  int         col[WBC_GMO_POINTS];
  int         whole;                       /**< 1: every unknown and every pair a pass, the robot's set unscoped */
  int         pushes;                      /**< the points' pushes, the stops' first column */
  int         gated[WBC_GMO_POINTS];       /**< 1 a point not felt: off the robot's soles, unlatched (a pass's) */
  int         world[WBC_GMO_POINTS];       /**< 1 a point's pushes laid in the world, not its link */
  int         soled[WBC_GMO_POINTS];       /**< 1 a point of the robot's soles */
  double      stop_lo[WBC_N - 6];          /**< each hinge's stops, rad; 0 none */
  double      stop_hi[WBC_N - 6];
  int         stopped[WBC_N - 6];
  int         capsules;                    /**< the robot's capsules: each one's link, radius and axis's */
  int         capsule_link[WBC_GMO_CAPSULES]; /**< ends in it (a sphere's alike) */
  double      capsule_r[WBC_GMO_CAPSULES];
  double      capsule_a[WBC_GMO_CAPSULES][3];
  double      capsule_b[WBC_GMO_CAPSULES][3];
  int         pairs;                       /**< the pairs of capsules that can touch */
  int         pair[WBC_GMO_PAIRS][2];
  int         listed;                      /**< the pairs staged within the robot's envelope, in their order */
  int         list[WBC_GMO_PAIRS];
  double      staged[WBC_GMO_CAPSULES][2][3]; /**< each capsule's ends, world, as last staged */
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
  double      f[WBC_GMO_F];                /**< out: each unknown's push: the points' edges, the */
                                           /**< stops' (N m), the touches' */
  double      force[WBC_GMO_MANIFOLDS];    /**< out: each manifold's normal force, N */
  double      at[WBC_GMO_POINTS][3];       /**< out: each point in the world */
  int         touched[WBC_GMO_TOUCH];      /**< out: each touch's pair, -1 none */
  int         past[WBC_N - 6];             /**< each hinge's row past its band: 1 over, -1 under */
  int         stagings;                    /**< out: the robot's pairs staged so far */
  double      prior[WBC_GMO_POINTS];       /**< out: each point's push as gravity would have it, N */
  double      speed[WBC_GMO_POINTS];       /**< out: each point's speed in the world, m/s */
  double      expected[WBC_GMO_F];         /**< and each unknown's */
  int         rounds;                      /**< out: the band's rounds this pass */
  int         lives;                       /**< out: the unknowns that can push this pass, in their order */
  int         live[WBC_GMO_F];
  int         spare;                       /**< explicit: the robot's ints a whole number of doubles */
  wbc_body_t  body;
} wbc_gmo_t;

/** A drive pass of `dt` s into the sums, of the robot's edge's array: the drives' torques (`tau`,
    every joint's), the pelvis's specific force (its own frame) turned by its quaternion (w, x, y,
    z) to the world, gravity added. */
void wbc_gmo_felt(wbc_gmo_t *o, const double quat[4], const double accel[3], const double *tau, double dt);

/** A law pass at the robot's estimate - the pelvis's frame, the hinges, u -, `dt` s on: r, f,
    force; the sums emptied. */
void wbc_gmo_step(wbc_gmo_t *o, const wbc_frame_t *base, const double q[WBC_N - 6], const double u[WBC_N], double dt);

#endif
