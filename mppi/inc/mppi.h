/** mppi.h - a plane's rollouts for sampled predictive control: its forces, its floor, its cost */
#ifndef MPPI_H
#define MPPI_H

/* The plane's model: its propeller through its inflow, the wing to and past its stall, the side
 * as a plate across the span, the surfaces in the propwash, the drag, the floor under its contact
 * points. One source for the host's CPU (gcc, ctypes) and its card (NVRTC); machine/se3mpc.py is
 * its numpy twin. The firmware builds none of it. */

/* The model's constants, in this order (machine.se3mpc.CONSTS). */
enum {
    MP_MASS, MP_GRAVITY, MP_K_THRUST, MP_K_PITCH, MP_K_ROLL, MP_K_YAW, MP_K_DRAG, MP_RHO, MP_AREA,
    MP_CL_ALPHA, MP_SPAN, MP_STAB, MP_FIN, MP_ARM, MP_WQ, MP_STALL, MP_BLEND, MP_CD_LIFT, MP_J0,
    MP_PITCH, MP_PROP_R, MP_CL_ALPHA_A, MP_CDA, MP_AREA_A, MP_CD0, MP_CDI, MP_SHARE, MP_IX, MP_IY,
    MP_IZ, MP_K_FLOOR, MP_C_FLOOR, MP_SIDE, MP_A_DISC, MP_PROP_BASE, MP_PROP_LOAD, MP_PROP_SURF, MP_CONSTS
};

/* The cost's weights, in this order (machine.se3mpc.Mppi.W); each a sum over the steps. */
enum {
    MW_X, MW_Y, MW_Z, MW_SPEED, MW_BELLY, MW_NOSE, MW_SPIN, MW_RESERVE, MW_TOUCH, MW_CRASH, MW_EASE,
    MW_THRUST, MW_SKIM, MW_WEIGHTS
};

/* A step's reference, in this order: place, speed, the belly's and the nose's directions and
 * their weights, the height asked of the tail's lowest point and its weight, the height's own. */
enum {
    MR_PLACE = 0, MR_SPEED = 3, MR_BELLY = 6, MR_NOSE = 9, MR_W_BELLY = 12, MR_W_NOSE, MR_SKIM,
    MR_W_SKIM, MR_W_HEIGHT, MR_REF
};

#ifndef MPPI_FN
#define MPPI_FN
#endif

/* One rollout's cost: from x0 (place 3, speed 3, body to world 9 row-major, body rates 3) under
 * u (h x 4: throttle, elevator, aileron, rudder) against ref (h x MR_REF); contact points c
 * (nc x 5: body point, radius, friction), the first ns the tail's; points r (nr x 3) that must
 * stay off the floor; u0 the controls before it, wind the air's. */
MPPI_FN double mppi_plane_rollout(const double *x0, const double *u, const double *ref,
                                  const double *k, const double *c, int nc, int ns,
                                  const double *r, int nr, const double *weight, const double *u0,
                                  int h, double dt, double w_p, double w_v, const double *wind);

/* Each of n rollouts' cost, u n x h x 4. */
void mppi_plane_costs(const double *x0, const double *u, const double *ref, const double *k,
                      const double *c, int nc, int ns, const double *r, int nr,
                      const double *weight, const double *u0, int n, int h, double dt, double w_p,
                      double w_v, const double *wind, double *cost);

#endif
