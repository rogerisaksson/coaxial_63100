/** wbc.h - her chain on static arrays (wbc_model.h): poses, Jacobians, mass matrix and bias,
    the product of exponentials over twists (w, v) in body frames; and the loop over it, the
    asks of R^k into the drives' torques. Portable C11. */
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
  double      q[WBC_N - 6];        /**< the hinges' angles */
  double      axis[WBC_LINKS][3];  /**< each hinge's axis in the world */
  double      anchor[WBC_LINKS][3];/**< a point on it */
  double      u[WBC_N];            /**< her velocities, as the bias had them */
  double      v[WBC_LINKS][6];     /**< each link's twist in its own frame */
  double      vd[WBC_LINKS][6];    /**< its acceleration, gravity's included */
  double      f[WBC_LINKS][6];     /**< the wrench it passes to its parent */
  double      gc[WBC_LINKS][10];   /**< all it carries about the pelvis's origin, the world's axes:
                                        mass, that times its centre, inertia (xx xy xz yy yz zz) */
  double      s[WBC_LINKS][6];     /**< its hinge's screw there, (w, v) */
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

/** Her angular momentum about `com`, world (after the mass matrix and the bias). */
void wbc_body_momentum(const wbc_body_t *b, const double com[3], double k[3]);

/** The commanded torques: the drives', then the held joints'. */
#define WBC_X (WBC_DRIVEN + WBC_HELD)
/** The loop's variables: every acceleration, then each sole's four corners' forces, world, in
    shares of her weight (the python stack's scaling: the regularisation stays negligible). */
#define WBC_F 12
/** The slacks: each sole's four centre-of-pressure margins. */
#define WBC_S 8
#define WBC_V (WBC_N + 2 * WBC_F + WBC_S)
/** The levels - the dynamics' undriven rows, the held joints and the standing soles (exact);
    the centre of mass; the turns and a swing; nothing; the form - and the most rows one lays. */
#define WBC_LEVELS 5
#define WBC_ROWS 96
/** A level of this many rows or fewer cuts the null space under it. */
#define WBC_SMALL 48
/** The most bounds a tick, and the most of them reported at their edge. */
#define WBC_INEQ 160
#define WBC_EXTRA 32

/** What the loop is asked, R^k: a sole bears or swings; the centre of mass's acceleration; the
    pelvis's and the trunk's turns, one of all zeros unasked; a swinging sole's place, speed,
    acceleration and turn; a
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
  double settle[2][6];            /**< a standing sole's own acceleration (w, v), world: 0 still */
  double edge[2];                 /**< its centre of pressure toward its ball (1) or heel (-1) */
  double strides;                 /**< how far she strides, 0 to 1: the floor's shortfall her inertia's */
  double mu[2];                   /**< each standing sole's friction: her grip (machine.grip) */
} wbc_ask_t;

/** What the loop answers: the drives' torques; every acceleration (u's order); each sole's
    wrench (m, f) in the world about its middle and its load; the bounds at their edge, how
    many and which, the active set's changes this tick, the most any bound is over; the drives
    asked past their clamps, each drive's load of its clamp, the inertia each drive's torque
    meets alone; each sole's corner forces, world, 3 a corner; the levels whose set ran out. */
typedef struct
{
  double tau[WBC_DRIVEN];
  double udot[WBC_N];
  double wrench[2][6];
  double bears[2];
  int    held;
  int    passes;
  int    guarded[5];              /**< at their edge, by kind: a load, a corner pressing, its cone, a centre of pressure, a stop */
  int    ids[WBC_EXTRA];          /**< which */
  double alpha[3];                /**< 1: every level whole */
  double residual;                /**< the most a bound is over, unit rows */
  int    over[WBC_DRIVEN];
  double load[WBC_DRIVEN];
  double jeff[WBC_DRIVEN];
  double force[2][WBC_F];
  int    stuck;
  double clipped[2];              /**< 1: a swing's ask whole */
} wbc_out_t;

/** The loop's memory, laid once. */
typedef struct
{
  double chol[WBC_N][WBC_N];      /**< M's Cholesky factor, lower, for each drive's inertia */
  int    freed[WBC_N];            /**< the dofs no drive and no stop holds, and how many */
  int    nfree;
  double jsole[2][6][WBC_N];      /**< each standing sole's Jacobian at its middle, world (w, v) */
  double dsole[2][6];             /**< and its drift */
  double jcorner[2][4][3][WBC_N]; /**< each corner's linear Jacobian, world */
  double pcorner[2][4][3];        /**< and where it is */
  double sole_p[2][3];            /**< each sole's middle, world */
  double fs;                      /**< her weight, N: a corner force variable is a share of it */
  double tc[WBC_DRIVEN][WBC_V];   /**< each drive's torque over the variables, and its offset */
  double toff[WBC_DRIVEN];
  double la[WBC_ROWS][WBC_V];     /**< the level being laid: its rows over the variables, laid again as a solve needs them */
  double lr[WBC_ROWS];
  int    lrows;
  int    laying;                  /**< which level `la` holds */
  void (*relay)(void *ctx, int level); /**< lays a level into `la` (`wbc_stack_step`) */
  void  *ctx;
  double ab[WBC_ROWS][WBC_V];     /**< the level being solved: its rows in the reduced coordinates */
  double rho[WBC_ROWS];
  double zb[WBC_V][WBC_V];        /**< the null space so far, an orthonormal basis in its first zm columns */
  int    zm;
  double qq[WBC_V + 1][WBC_V];    /**< a level's rows orthonormalised, their span: at most the reduced dimension, one tried past it */
  double lowcol[WBC_V];           /**< the tie-break's column norms, the levels below projected */
  double kk[WBC_V][WBC_V];        /**< a level's curvature K = H + mu I, factored in place */
  double ga[WBC_INEQ][WBC_V];     /**< the bounds, ga v <= gh, unit rows; their kinds and ids */
  double gh[WBC_INEQ];
  int    gkind[WBC_INEQ];
  int    gid[WBC_INEQ];
  int    ineq;
  double gs[WBC_V][WBC_V];        /**< the held bounds in the reduced coordinates, in the set's order, and */
  double hs[WBC_V];               /**< what the variables so far leave them; every bound reduced was 89 KB */
  double gp[WBC_V];               /**< the pending bound's, the same */
  double hp;
  int    gfixed[WBC_INEQ];        /**< a bound the level being solved cannot move, found as it is tried */
  double ud[WBC_V];               /**< the held bounds' duals, and the pending bound's */
  double upend;
  double sfac[WBC_SMALL][WBC_SMALL]; /**< S factored; a level holds WBC_SMALL bounds at most */
  int    wset[WBC_LEVELS][WBC_V]; /**< each level's bounds held, warm across ticks */
  int    wn[WBC_LEVELS];
  int    wsized[WBC_LEVELS];      /**< the bounds a level's set was made among */
  int    iterations;              /**< the set's changes this tick */
  int    stuck;                   /**< levels that ran out of changes */
  double vw[WBC_SMALL][WBC_V];    /**< K^-1 of the held bounds' rows */
  double sm[WBC_SMALL][WBC_SMALL]; /**< their Schur complement S = N K^-1 N^T */
  double tau[WBC_LEVELS][WBC_V];  /**< each level's contribution */
  double total[WBC_V];            /**< their sum: the accelerations, then the corners' forces */
  double torque[WBC_X];           /**< the drives' torques they give; the held joints' 0 */
  double load_now[WBC_DRIVEN];
  double pmap[WBC_DRIVEN][WBC_X]; /**< each drive's load from the torques */
  double ceiling[WBC_DRIVEN];     /**< this tick's bound on each load, and the clamp as derated */
  double nominal[WBC_DRIVEN];
  double edge_lo[2];              /**< a sole's centre of pressure within, x then z, its foot's frame */
  double edge_hi[2];
  int    standing[2];             /**< each sole's place among the contacts, -1 swinging */
  int    nst;
  double load[WBC_DRIVEN];        /**< the last tick's load of each clamp: WEP where it ran near */
  double com[3];
  double jcom[3][WBC_N];
  double jac[6][WBC_N];
} wbc_stack_t;

void wbc_stack_init(wbc_stack_t *s);

/** One tick: her pose and velocity in, the ask in, the torques and the rest out. */
void wbc_stack_step(wbc_body_t *b, wbc_stack_t *s, const wbc_frame_t *base,
                    const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                    wbc_out_t *out);

/** The step without an active set (wbc_wrench.c): its unknowns - six a sole, its wrench where
    it stands, its acceleration where it does not, then her upper body's drives' accelerations
    - and the joints no drive and no stop holds. */
#define WBC_U (12 + WBC_DRIVEN - 12)
/** Its rounds of clipping, and of faces striding. */
#define WBC_ROUNDS 4
/** Rounds more, at most, for an answer past her height's band after them (machine.wrench.PAST). */
#define WBC_PAST 2
/** Her law's own friction, a sole's, before she has felt a floor (machine.wbc.MU). */
#define WBC_MU 0.6
#define WBC_PASSIVE (WBC_N - 6 - WBC_DRIVEN - WBC_HELD)

/** Its memory, laid once. */
typedef struct
{
  int    leg[2][6];               /**< each leg's drives, hip to sole, among the drives */
  int    upper[WBC_DRIVEN - 12];  /**< the rest of them */
  int    passive[WBC_PASSIVE];    /**< the dofs no drive and no stop holds, and how many */
  int    npassive;
  int    led[12 + WBC_PASSIVE];   /**< the dofs the pelvis's six move: her legs', the passive */
  int    reach[WBC_N][2];         /**< the unknowns a dof's row of e reaches this tick: from, before */
  double room[2];                 /**< a sole's centre of pressure within, across and along, m */
  double jsole[2][6][WBC_N];      /**< each sole's Jacobian at its middle, world (w, v) */
  double sole_p[2][3];            /**< and where it is */
  double jac[6][WBC_N];
  double jcom[3][WBC_N];
  double t[WBC_N][6];             /**< udot = t base + e x + t0 */
  double e[WBC_N][WBC_U];
  double t0[WBC_N];
  double bm[6][WBC_U];            /**< the base's rows: A base = bm x - d */
  double p[6][WBC_U];             /**< base = p x + p0 */
  double p0[6];
  double h[WBC_U][WBC_U];         /**< the rows' normal equations, and a round's, factored */
  double g[WBC_U];
  double k[WBC_U][WBC_U];
  double gk[WBC_U];
  double mt[WBC_N][6];            /**< striding: M t and M e, t^T M e, (t^T M t) p + t^T M e */
  double me[WBC_N][WBC_U];
  double tme[6][WBC_U];
  double xk[6][WBC_U];
  double gi[12][WBC_U];           /**< the metric's inverse's columns a sole's unknowns have */
  double cf[13][12];              /**< the faces met, over the soles' unknowns; the 13th her height's band */
  double z[13][WBC_U];            /**< the metric's inverse on each */
  double s[13][13];
} wbc_wrench_t;

void wbc_wrench_init(wbc_wrench_t *w);

/** One tick: as wbc_stack_step; of `out` the torques, the accelerations, the soles' wrenches
    and loads, the drives past their clamps and each one's load. */
void wbc_wrench_step(wbc_body_t *b, wbc_wrench_t *w, const wbc_frame_t *base,
                     const double q[WBC_N - 6], const double u[WBC_N], const wbc_ask_t *ask,
                     wbc_out_t *out);

/** A standing sole's wrench in its cone (wbc_cone.c): x with each inside its bounds a part at
    a time - pressing, in its friction's pyramid, its centre of pressure in its room -, into
    at, and which parts moved. */
void wbc_cone_bounded(const wbc_body_t *b, const wbc_wrench_t *w, const wbc_ask_t *ask,
                      const double stance[2], const double x[WBC_U], double at[WBC_U],
                      double moved[WBC_U]);

/** Striding: u0, the rows' answer with the floor unbounded, brought to the faces her soles'
    wrenches are past by the least change in her kinetic energy's metric, into at; her
    height inside lo to hi; mbt the pelvis's rows of M t, own each unknown's own weight. 0
    where the metric has no factor. */
int wbc_cone_taken(const wbc_body_t *b, wbc_wrench_t *w, const wbc_ask_t *ask, const double stance[2],
                   double lo, double hi, const double mbt[6][6], const double own[WBC_U],
                   const double u0[WBC_U], double at[WBC_U]);

#endif
