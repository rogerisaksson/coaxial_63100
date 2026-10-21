/** wbc_imp.h - her modes' springs on her body in C (wbc_imp.c, wbc_stride.c): machine.impedance's
    ask, with machine.modal's reading, machine.poise, machine.stride and flight.standing under it,
    numpy's arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#ifndef WBC_IMP_H
#define WBC_IMP_H

#include "wbc_blas.h"

/** The most modes she has. */
#define WBC_IMP_N 16

/** What the ask is laid on, filled once from the modules that define it: her modes' indices,
    the offsets of latent.step's out, modal's y, fr and a, and their constants. */
typedef struct
{
  int           n;            /**< her modes, and each's index */
  int           axial, lateral, drive, pitch, sagittal, lift, width, list, turn, arms, rock;
  int           o_ref, o_rate, o_acc, o_ks, o_ds, o_psi, o_sway, o_turns, o_held, o_yawed;
  int           y_com, y_vcom, y_sole, y_vsole, y_load, y_quat, y_spin, y_moment, y_lowest, y_hip, y_tilt;
  int           y_tilt_rate, y_arm, y_prop, y_prop_lo, y_prop_hi, y_grip, y_vhead, y_hspin;
  int           f_rot, f_w0, f_floor, f_weight, f_knee, f_pitch0, f_dt, f_reach0, f_hips_by, f_arm0, f_unit;
  int           f_z0, f_mass, f_ground, f_floor_at;
  int           a_stance, a_com_acc, a_sole_acc, a_want, a_fold, a_fold_rate, a_turned, a_bend, a_kdot;
  int           a_going, a_rolled, a_headed, a_tipped, a_swung, a_folds, a_settle, a_edge, a_heeled;
  int           a_strides, a_prefold, a_swings, a_raise, a_need, a_lead, a_outside, a_borne;
  int           free_by[3];   /**< the modes holding a sole that does not stand, an axis each */
  const double *hands;        /**< each mode's hand on a limb's place (n, 2, 3) */
  /* machine.impedance */
  double        on[2], going_s, ahead, yaw_k, throw_m_s2, prop_on, low_m, low_leg, falls[3], upward[3];
  double        between[3];
  /* machine.modal, machine.wbc */
  double        g, axial_m, pitch_rad, clear_m, list_rad, turn_rad, arm_rad, leg_m, pelvis[2];
  double        momentum_k, posture_kd[2], turn_kd[2], contact_k;
  /* machine.poise, machine.flight, machine.figure */
  double        big, tiny, half_m, margin_m, capture_s, extent[2], lift_eps, gate_m, early, along_on[2];
  double        touch_n, touch_m, rest_m_s, middle, ankle_h, ball, toe_m;
  /* machine.stride */
  double        limbs[2], stride_m, rock_rad, stirs, phi_w, back, ahead_m_s, reach_m, apart_m, soon_s;
  double        rise_u, fall_u, rise_u2, fall_u2, arrive_u, leads, reach_m_s, reach_s, flat_k, heel_rad;
  double        heel_rad_s, dip_m, edge_rad, fold_rad, heel_up;
} wbc_imp_t;

/** `a`, the loop's ask this pass: latent.step's `out` rendered on her as she is sensed (`y`,
    `fr`), `a` the pass before's in and this one's out (machine.impedance.ask). */
void wbc_imp_ask(const wbc_imp_t *k, const double *out, const double *y, const double *fr, double *a);

/** machine.modal.reading: her modes `z` and their rates `zd` as she is, the soles in `stance` her
    standing ones; her legs (each hip from its ankle (2, 3), its length, that's rate), her turns
    and their rates (3, 2). */
void wbc_imp_reading(const wbc_imp_t *k, const double *y, const double *fr, const double stance[2], double *z,
                     double *zd, double leg[2][3], double len[2], double lrate[2], double turn[3][2],
                     double spun[3][2]);

/** machine.modal.shortened: her axial compression and its rate; under: the floor under each
    sole's middle, its rise along her way returned; each sole's share of those that stand. */
void   wbc_imp_shortened(const wbc_imp_t *k, const double len[2], const double rate[2], const double *fr,
                         const double stance[2], double axial[2]);
double wbc_imp_under(const wbc_imp_t *k, const double *y, const double *fr, const double stance[2], double floors[2]);
void   wbc_imp_shares(const wbc_imp_t *k, const double stance[2], double bears[2]);

/** machine.poise.boxes: each sole's box (low, high; along, across) round its middle `flat`. */
void wbc_imp_boxes(const wbc_imp_t *k, const double flat[2][2], const double edge[2], const double tilt[2],
                   double low[2][2], double high[2][2]);

/** machine.poise.spanned: lo to hi grown over prop (low, high) as far as `by` has it. */
void wbc_imp_spanned(const double lo[2], const double hi[2], const double plo[2], const double phi[2], double by,
                     double slo[2], double shi[2]);

/** machine.poise.stands: which soles stand, her ZMP into `zmp`. `beat` sweep, off, flying;
    `bears` loads, over, felt; `prop` its low, high and how far. */
void wbc_imp_stands(const wbc_imp_t *k, double zmp[2], const double xi[2], const double low[2][2],
                    const double high[2][2], double om, double held, const double up[2], int swaying,
                    const double beat[3][2], const double bears[3][2], double rising, double borne,
                    const double stood[2], const double prop[2][2], double by, double stance[2]);

/** machine.stride.beat: (stepping, way, swung) out, each limb's sweep, phi, on, off, u, flying. */
void wbc_imp_beat(const wbc_imp_t *k, double drive, double psi, double sway, double lateral, double turns,
                  const double was[2], double need, const double lead[2], double *stepping, double *way,
                  double *swung, double sweep[2], double phi[2], double on[2], double off[2], double u[2],
                  double flying[2]);

/** machine.stride.hurried, cruises, meets, rocks, folds, lates, goes. */
void   wbc_imp_hurried(const wbc_imp_t *k, const double aside[2], double om, const double u[2], double rate,
                       const double kp[2], double hurry[2]);
double wbc_imp_cruises(const wbc_imp_t *k, double way, double drive, double turns, double going);
void   wbc_imp_meets(const wbc_imp_t *k, const double xi[2], const double c[3], const double flat[2][2],
                     double cruise, double om, double turns, double wide, double across, double reach,
                     double meet[2][2]);
void   wbc_imp_rocks(const wbc_imp_t *k, double rock, double rate, double acc, double tip[2], double tipping[2],
                     double tipped[2]);
void   wbc_imp_folds(const wbc_imp_t *k, const double off[2], const double loads[2], double bent, double fold[2],
                     double shorter[2]);
void   wbc_imp_lates(const wbc_imp_t *k, const double phi[2], double turns, double stepping, double late[2]);
void   wbc_imp_goes(const wbc_imp_t *k, const double meet[2][2], const double flat[2][2], const double shod[2][3],
                    const double v[3], const double u[2], const double turns[2], const double kp[2],
                    const double kd[2], double acc[2][2]);

/** machine.stride.pitched, of each sole's floor. */
void wbc_imp_pitched(const wbc_imp_t *k, const double ground[2], double pitched[2]);

/** machine.stride.heels: the rise asked, flatten, the edge and rounds (2, 3) out. `place` the
    soles' places (2, 3), `tilt` and `tilting` their pitch and its rate, `leg` a leg's length. */
void wbc_imp_heels(const wbc_imp_t *k, const double place[2][3], const double tilt[2], const double tilting[2],
                   const double hips[2], const double floors[2], double leg, const double shorter[2],
                   const double off[2], const double tip[2], const double stance[2], double stepping,
                   const double was[2], double dt, const double ground[2], double rocked[2], double flatten[2],
                   double edge[2], double rounds[2][3]);

/** machine.stride.clears: a swinging sole's middle over the floor, its rate, its acceleration. */
void wbc_imp_clears(const wbc_imp_t *k, const double u[2], const double flying[2], const double swung[2],
                    double high, const double tip[2], const double tipping[2], const double tilt[2],
                    const double late[2], double clear[3][2]);

/** machine.motion.surety_field's reading of her law's arrays, laid once: machine.edge's offsets in
    y_in, machine.latent's in st and out, the arrays themselves. */
typedef struct
{
  int           quat_at;   /**< the IMU's turn, spin and specific force, the soles' loads in y_in */
  int           gyro_at;
  int           accel_at;
  int           load_at;
  int           k_to;      /**< the axial stiffness asked and left in st, its gain now; out's ZMP */
  int           k_from;
  int           gain_now;
  int           zmp_out;
  const double *y_in;      /**< her edge's array; modal's y, fr and a; her latent state and its out */
  const double *y;
  const double *fr;
  const double *a;
  const double *st;
  const double *out;
} wbc_surety_t;

/** Her directional surety field `dt` s on from `prev` (machine.motion.surety_field, wbc_surety.c):
    her tilt `ct` deg where given, nan her IMU's; `t` her time; `vec` out: her surety along and
    across, its direction, its probe. */
double wbc_imp_surety(const wbc_imp_t *k, const wbc_surety_t *s, double prev, double dt, double ct, double t,
                      double vec[5]);

/** machine.touch.stepped's reading of her law's arrays, laid once: the edge's touch offset in
    y_in, latent's gains in st, her rest's Z, K and D, the filter's state's offsets; her core's
    and her soles' skin indices, each's body, the toes' tips; the filter's constants; the arrays. */
typedef struct
{
  int           touch_at;  /**< the skin's touch in y_in */
  int           st_now_gain, st_to_gain;  /**< the gain now and asked in st */
  int           r_z, r_k, r_d;  /**< rest's targets, springs, dampers */
  int           s_felt, s_hold, s_brake, s_stand, s_drv, s_clear;  /**< the state's */
  int           core[4];   /**< her pelvis, torso, neck and head among the skin */
  int           soles[2][2];  /**< each sole's foot and toes among the skin */
  const int    *ids;       /**< each skin index's body */
  const double *tips;      /**< the toes' tips (2, 3) */
  double        resume_s, along_cos, felt_s, edge_m, touched_n;
  const double *y_in, *xmat, *xpos, *y, *fr, *st, *rest;
  double       *a, *out, *state;  /**< written: the ask, the latent out, the filter's state */
} wbc_touch_t;

/** A pass of the filter `dt` s on (machine.touch.stepped, wbc_touch.c): `asked` the twist told,
    `twist` hers in and out, `tw_on` whether one is told, `heading` hers, `vel` her estimate's. */
void wbc_touch_step(const wbc_imp_t *k, const wbc_touch_t *t, const double asked[3], double twist[3], double tw_on,
                    double heading, double dt, const double *vel);

/** machine.se2.twisted's reading of her law's arrays, laid once: the edge's and modal's offsets
    the ask's struct lacks, latent's in st, her waist and her head among her driven joints, the
    step's state's and its words' offsets (machine.se2), its constants, the arrays. */
typedef struct
{
  int           gyro_at, quat_at, y_cells, f_heading;
  int           st_now_gain, st_now_phase, st_to_amp, st_to_hz, st_now_hz, st_theta;
  int           waist, head;
  int           s_last_way, s_look_yaw, s_turn_dc;
  int           w_heading, w_target, w_has, w_twist, w_actual, w_active, w_stop, w_t, w_back, w_backing, w_clear;
  double        walk_m_s, back_m_s, strafe_m_s, yaw_rate, floor_hz, touch_n, stirs;
  const double *y_in, *y, *a;
  double       *fr, *st, *out, *posture, *state, *words;
} wbc_twist_t;

/** A pass of her twist `dt` s on (machine.se2.twisted, wbc_twist.c): `vel` her estimate's, `held`
    her soles' grip, `prior` what she knows of the floor; fr, st, out, her posture, the state and
    the words in place. */
void wbc_twist_step(const wbc_imp_t *k, const wbc_twist_t *t, double dt, const double *vel, const double *held,
                    const double *prior);

/** The most rows of her table the waits take. */
#define WBC_WAIT_ROWS 512

/** machine.waits.held_rows's, risen_rows's and woken_rows's reading of her table (machine.latent):
    a row's doubles, its time's place, its targets told and said, its gain told and said, its said
    block (p long), its amplitudes told, its phases said and told; her latent state's targets asked,
    its gain and phases now, her rhythm's angle; the waits' constants; numpy's BLAS. */
typedef struct
{
  int        table, at, vals_z, said_z, vals_gain, said_gain, st_to_z;
  int        p, said, vals_amp, said_phase, vals_phase, st_now_gain, st_now_phase, st_theta;
  double     edge_s, tiny, phi_w;
  wbc_blas_t blas;
} wbc_wait_t;

/** Rows that halt her rhythm wait while her capture point steps her or a swing is under way
    (machine.waits.held_rows): `tab` (n rows) its times shifted in place; `got` the shift, the
    first row shifted, her hold's seconds. Rows that raise her wait while something bears her
    off her soles and her capture point is off them (risen_rows): `got` the shift, the first,
    until when a prop counts. */
void wbc_wait_held(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *a, const double *out,
                   double t, double dt, double held_s, double got[3]);
void wbc_wait_risen(const wbc_imp_t *k, const wbc_wait_t *w, double *tab, int n, const double *y, const double *fr,
                    const double *a, const double *st, double t, double dt, double prop_until, double got[3]);

/** The rows due this pass wake her rhythm where its gain has died down (machine.waits.woken_rows):
    its angle let go, its phases those the rows say, in her latent `st`. */
void wbc_wait_woken(const wbc_imp_t *k, const wbc_wait_t *w, const double *tab, int n, double *st, double t, double dt);

/** machine.motion.scaled's and braced's reading of her body: its dofs and driven, the cervical and
    shoulder joints among the driven (cv) and their dofs (acv), the tuck's rows (3, driven), her
    drives' clamps, her legs' joints and their free stiffness, the posture gains, numpy's BLAS, a
    scratch (wbc_brace.c). */
typedef struct
{
  int           n, nd, ncv, nleg;
  const int    *act, *acv, *cv, *leg_joints, *leg_len;
  const double *brace, *clamp, *free_kp;
  double       *work;
  double        kd0, kd1, standing_kp;
  wbc_blas_t    blas;
} wbc_brace_t;

/** The solve's M: `M` copied into `Ms`, the four dofs' rows and columns against the base scaled
    by 1 - w_up (machine.motion.scaled). */
void wbc_brace_scaled(const wbc_brace_t *b, const double *M, const double *w_up, double *Ms);

/** The solve's `qacc`, `tau` and `over` blended in place with the tuck's by her surety's `sw` and
    `bw`, the cervical blend by `w_up`; her drives' inertia `jeff` and her legs' stiffness `leg_kp`
    by their `stance` out (machine.motion.braced). */
void wbc_brace_blend(const wbc_brace_t *b, const double *M, const double *Ms, const double *q, const double *v,
                     const double *h, const double *posture, double *qacc, double *tau, unsigned char *over,
                     double sw, double bw, double d_ax, double d_lat, double w_hop, const double *w_up,
                     const double *stance, double *jeff, double *leg_kp);

#endif
