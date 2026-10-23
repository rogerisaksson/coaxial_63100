/** wbc_pass.h - a pass of her world on the host (wbc_pass.c): her drives told, her world's
    torques, meters and reads, her drives' heat, on arrays laid once and pointed to, numpy's
    arithmetic operation for operation. Portable C11. */
#ifndef WBC_PASS_H
#define WBC_PASS_H

#include "wbc.h"
#include "wbc_blas.h"
#include "wbc_bus.h"

/** A pass of her world on the host (wbc_pass.c): arrays laid once and pointed to, nothing allocated,
    numpy's arithmetic operation for operation. The most joints a world has. */
#define WBC_PASS_MAX 64

/** Her law's drive-side step (machine.law.drives): each driven joint's observer on its motor's
    encoder, its setpoint carried on by the loop's ask and leaking to its encoder, its feed and its
    clamp; the knees' spring. Pointers into her law's arrays, the driven joints in wbc_driven's
    order, y machine.edge's. */
typedef struct
{
  int                  n;          /**< the driven joints */
  int                  q_at;       /**< y's offsets: the joints' angles, their rates, the motors' angles */
  int                  qd_at;
  int                  qm_at;
  int                  knee[2];    /**< the knees among the driven */
  int                  cervical[2];/**< the neck and the head among them, -1 none */
  double               knee_nm;    /**< the knees' spring from straight, N m, over this much bend, rad */
  double               knee_soft;
  double               leak_s;     /**< a setpoint's leak to its encoder, s */
  double               wep_hold_s; /**< a drive asked past its clamp holds its WEP this long, s */
  const int           *joints;     /**< each driven joint's index among the world's joints */
  const int           *act;        /**< its dof among the loop's accelerations */
  const double        *y;          /**< her edge's array */
  const double        *qacc;       /**< the loop's accelerations, its torques, which drives it asked over */
  const double        *tau;
  const unsigned char *over;
  const double        *jeff;       /**< the inertia each drive's torque meets alone */
  const double        *clamp;      /**< each drive's clamp, its WEP, its WEP's limit, the world's clamp */
  const double        *wep;
  const double        *wep_lim;
  const double        *clamp_j;
  const double        *q_lo;       /**< its setpoint's room */
  const double        *q_hi;
  double              *eso;        /**< the observers: angle, rate, disturbance (3, n) */
  double              *ref_q;      /**< the setpoints and their rates */
  double              *ref_v;
  double              *hold;       /**< each drive's hold left, s */
  double              *sb;         /**< the knees' spring this pass */
  double              *feed;       /**< out: each drive's feed and its clamp this pass */
  double              *limit;
  const wbc_bus_k     *bus_k;      /**< the master bus insert (machine.supervisor): its numbers, its state */
  wbc_bus_t           *bus;
  double               posture_kd; /**< a drive's own damping over its inertia, 1/s */
} wbc_drives_t;

/** The step, `dt` s on: `wo` the observers' bandwidth and `wo3` its cube, `ys` her law's yield as
    she tilts (1 upright), `in_hop` whether she is in a hop, `wb` whether the raised WEP limit is hers. */
void wbc_pass_drives(wbc_drives_t *p, double dt, double wo, double wo3, double ys, int in_hop, int wb);

/** Her world's drives on their setpoints (machine.physics.World.advance): each joint's PD at the
    world's step, a parallel pair's shared, the motors' torques and their meters; her IMU and
    encoders read. Pointers into the world's arrays and MuJoCo's. */
typedef struct
{
  int                  n;          /**< the joints */
  int                  npairs;     /**< the parallel pairs, (joint, its roll) each */
  int                  pelvis;     /**< her pelvis's body */
  double               wound;      /**< where a drive reads its angle: 1 the motor's encoder, 0 the joint's */
  const int           *qadr;       /**< each joint's address in qpos, in qvel */
  const int           *vadr;
  const int           *pairs;
  const double        *qpos;       /**< MuJoCo's */
  const double        *qvel;
  const double        *qacc;
  const double        *xmat;
  double              *ctrl;
  const double        *target;     /**< each drive's setpoint, its rate, the torque fed forward, its gains (n, 2) */
  const double        *rate;
  const double        *feed;
  const double        *gains;
  const double        *limit;      /**< its clamp, its N m an amp, its phases shorted, its gates cut */
  const double        *kt;
  const unsigned char *shorted;
  const unsigned char *cut;
  const double        *delta;      /**< each rotor's angle past its link's, its encoder's zero */
  const double        *zero;
  const double        *armature;
  double              *motor;      /**< out: the motors' torques this step; each gearbox's peak since the reset */
  double              *geared;
  double               effort;     /**< the meters since the reset: torque held (N m s), work done and braked (J) */
  double               work;
  double               brake;
  const double        *loss;       /**< each drive's copper, W/(N m)^2; numpy's BLAS its sum */
  const wbc_blas_t    *blas;
  double               heat;       /**< the meter of its heat since the reset, J */
} wbc_world_t;

/** The torques into ctrl, `span` s past the setpoints' stamp; `ahead` and `wrate` the gearboxes'
    rotors past the setpoints and the links' speeds (boxed), NULL rigid. */
void wbc_pass_ctrl(wbc_world_t *w, double span, const double *ahead, const double *wrate);

/** The motors' torques off ctrl, the effort and the copper's heat metered over `step_s`. */
void wbc_pass_motor(wbc_world_t *w, double step_s);

/** After the world's step: the work done and braked on `speed` (NULL: the joints'), each gearbox's peak. */
void wbc_pass_metered(wbc_world_t *w, const double *speed, double step_s);

/** Whether any of `ncon` contacts pairs a geom `a` marks with one `b` marks: each contact's geom
    pair two ints at `geom`, `stride` bytes on from the one before (MuJoCo's mjContact). */
int wbc_pass_paired(const unsigned char *geom, ptrdiff_t stride, int ncon, const unsigned char *a,
                    const unsigned char *b);

/** The drives' torque (ctrl) and its power on each joint's speed eased into `torque` and `power` by
    `k` of the way (machine.worker's averages). */
void wbc_pass_averaged(const wbc_world_t *w, double k, double *torque, double *power);

/** Her IMU and encoders into `y` at its offsets (machine.edge): her turn, spin and specific force,
    each joint's angle and rate, each motor's angle through its gearbox. */
void wbc_pass_felt(const wbc_world_t *w, double *y, int quat_at, int gyro_at, int accel_at, int q_at, int qd_at,
                   int qm_at);

/** Her soles' and her skin's contacts read (machine.edge.read): the contacts between a body of hers
    and one not, a contact's force its rows of the solver's (the cone elliptic), summed a bin a
    segment in the contacts' order. The contacts' fields are read at their offsets from each
    contact's `pos`, `stride` bytes a contact (MuJoCo's mjContact, laid from its arrays). */
typedef struct
{
  int                  nbody;
  int                  nskin;      /**< the segments with skin, the insoles, the feet */
  int                  ninsoles;
  int                  nfeet;
  int                  stride;     /**< a contact's bytes, and its fields' offsets from its pos */
  int                  off_geom;
  int                  off_frame;
  int                  off_dim;
  int                  off_efc;
  double               middle_m;   /**< a sole's middle along it: its heel's cell behind, its ball's ahead */
  const int           *gb;         /**< each geom's body */
  const unsigned char *hers;       /**< each body: hers or not */
  const int           *sole_side;  /**< each body's sole's side, -1 none; its insole, its foot, its skin, -1 none */
  const int           *insole;
  const int           *foot;
  const int           *skin;
  const double        *xmat;       /**< MuJoCo's */
  const double        *xpos;
  double              *loads;      /**< the bins: each sole's load; each segment's touch (3); each insole's
                                        load and its moment (3); each foot's two cells; each segment's
                                        touch's size and its first moment (3) */
  double              *touch;
  double              *pressed;
  double              *at;
  double              *cells;
  double              *felt_n;
  double              *felt_at;
} wbc_contacts_t;

/** The bins filled from `ncon` contacts at `pos`, their forces in `efc_force`. */
void wbc_pass_contacts(wbc_contacts_t *c, int ncon, const char *pos, const double *efc_force);

/** The bins into `y` at its offsets: the loads, the touch, the insoles' loads and centres of pressure,
    the cells, where each segment's touch is. */
void wbc_pass_sensed(const wbc_contacts_t *c, double *y, int load_at, int touch_at, int press_at, int cop_at,
                     int cells_at, int at_at);

/** Her drives' heat as their boards keep it (machine.heat.Heat): four nodes a drive - the switches,
    the laminate, the stator, the winding - on their losses, the envelope's derate and trip on them. */
typedef struct
{
  int                  n;
  double               at;         /**< heat seconds since the start */
  double               haste;      /**< heat's seconds a second of the clock; the room, C */
  double               ambient_c;
  double               rds_ohm;    /**< the switches' on-resistance and its temperature coefficients, the shunts' */
  double               rds_alpha;
  double               rds_beta;
  double               shunt_ohm;
  double               switching_w;/**< the legs' dry switching loss; the switches' path into the laminate, K/W */
  double               into_k_w;
  double               rth_jc;     /**< the FETs' junction over the switches, K/W */
  double               lookahead_s;/**< the envelope: a hold inside this counted as spent; the derate 1 to here */
  double               throttle_at;
  double               recover_per_s;
  double               trip_margin;/**< a trip trims every ceiling to this of its span, given back this fast */
  double               trip_recover_per_s;
  double               tj_max_c;   /**< the FETs' junction limit */
  const double        *kt;         /**< each drive's N m an amp, its board's scale, its rotor's krpm a rad/s */
  const double        *scale;
  const double        *ratio_k;
  const double        *cloth_air;  /**< its clothing over the air, its winding's ohm, its mount's and
                                        its winding's paths to the stator, its laminate's and its
                                        winding's to the air, each node's J/K, the housekeeping's W */
  const double        *r;
  const double        *g_ls;
  const double        *g_ws;
  const double        *lam_k_w;
  const double        *win_k_w;
  const double        *capacity;   /**< (n, 4) */
  const double        *ceiling;    /**< each node's ceiling, C (4) */
  const double        *house_w;
  const unsigned char *active;     /**< a drive at all */
  double              *t;          /**< the nodes' temperatures (n, 4) */
  double              *sq;         /**< the mean square current since the last step, and its ticks */
  double              *krpm;       /**< the rotor's speed, the limb's */
  double              *limb_rad_s;
  double              *derate;     /**< the envelope's: the derate, the span spent, the FETs' junction, the trip cap and when */
  double              *spent;
  double              *die;
  double              *cap;
  double              *cap_at;
  long long           *ticks;
  long long           *worst;      /**< the node that binds */
  long long           *trips;
  unsigned char       *gates;      /**< its gates on, its phases shorted */
  unsigned char       *shorted;
} wbc_heat_t;

/** A tick's torques and joint speeds into each drive's mean square and speeds (machine.heat.Heat.step_joints). */
void wbc_pass_heat_load(wbc_heat_t *h, const double *torques, const double *qd);

/** `dt` s of the clock on: the nodes on their losses, `air` and `rds` each drive's air and
    on-resistance scales or NULL as built, the envelope derating and tripping where `envelope`. */
void wbc_pass_heat(wbc_heat_t *h, double dt, const double *air, const double *rds, int envelope);

/** Each drive's hottest: its junction or its nodes' peak. */
void wbc_pass_heat_peak(const wbc_heat_t *h, double *out);

/** What her drives draw now, W (machine.physics.World.drawn): the work they do on `speed` (NULL the
    joints'), their copper, their boards' own, the drives `driven`, switching, and a check's copper. */
double wbc_pass_drawn(const wbc_world_t *w, const double *tau, const double *speed, const double *loss,
                      const unsigned char *driven, const double *testing, const double *ohm, double housekeeping_w,
                      double switching_w);

/** machine.wear._hems' pendulum: a hem 2-DoF about its x and z hinges with their stops, `n` steps of
    `dt` on its hinge's acceleration `ap`, spin `om` and angular acceleration `alb` (n, 3 each, its
    frame): its angles, rates and accelerations (n, 2 each), the steps it met a stop, its (qx, qz,
    vx, vz) carried in and out. `c`: its arm, mass, inertia, J, stiffness, damping, the stop's
    dmin, dmax, width, stiffness and damping, its two inverse weights, lows and highs. */
void wbc_pass_hem(int n, double dt, const double *ap, const double *om, const double *alb, const double *c,
                  double *state, double *q, double *v, double *a, unsigned char *hit);

/** Her 5 extremity 10 Mbit/s RS485 Modbus RTU buses and 27 SIL coaxial_63100 boards. */
#define WBC_SIL_BUSES 5
#define WBC_SIL_BAUD  10000000U

void wbc_sil_init(wbc_world_t *w, int nbuses, const int *counts, const int *joints);
int  wbc_sil_transceive(wbc_world_t *w, int bus_id, const unsigned char *frame, int len);
void wbc_sil_drive(wbc_world_t *w, int n_idx, const int *idx, const double *target,
                   const double *feed, const double *kp, const double *limit);
void wbc_sil_stats(int bus_id, unsigned long long *counts, double *wire_s,
                   unsigned char *last_frame, int *last_len);

#endif
