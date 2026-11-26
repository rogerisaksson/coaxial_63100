/** wbc_world.h - the robot's world's steps on the host (wbc_world.c): before MuJoCo's step its drives' torques on their
    setpoints and its gearboxes stepped on them; after it the boxes moved, the drives' meters, the robot's
    DC link and its room's lamp switch - machine.world.physics.World.advance between its steps in two
    calls, numpy's arithmetic operation for operation (wbc_np.h), MuJoCo's contacts read off its data through its
    own headers. The host's alone. */
#ifndef WBC_WORLD_H
#define WBC_WORLD_H

#include <stddef.h>

#include "wbc_blas.h"
#include "wbc_pass.h"

/** MuJoCo's data (mujoco/mjdata.h), its contacts read in wbc_world.c alone. */
struct mjData_;

/** A ufunc's own inner loop (numpy's PyUFuncGenericFunction): its arguments' bytes, its length, their strides. */
typedef void (*wbc_loop_t)(char **args, const ptrdiff_t *dimensions, const ptrdiff_t *steps, void *data);

/** The robot's gearboxes (machine.body.gearbox.Boxes) laid once on its arrays, stepped in place as Boxes.step
    steps them: each joint's numbers, the state its step carries, the robot's world's braking of a shorted drive,
    and numpy's own tanh loop - an array's tanh is numpy's SIMD, not libm's, in a fifth of its values
    (tools.cores.world.numpy_tanh). */
typedef struct
{
  const unsigned char *on;          /**< each joint's box stepped (`sprung`), a byte each */
  const double        *k;           /**< its wind-up, its damping, its rotor's inertia, half its play, its shock and
                                         its mesh's friction (n each) */
  const double        *c;
  const double        *j;
  const double        *gap;
  const double        *shock;
  const double        *mesh;
  const double        *observe;     /**< its board's observer's rate, its torque loop's gain and its peak */
  const double        *track;
  const double        *peak;
  const double        *damping;     /**< a shorted drive's braking, N m s/rad (the world's) */
  double              *delta;       /**< in and out: each rotor's angle past its link, its speed, its board's estimate
                                         and its integral, its last drive, the command and its rate, the links at the
                                         step's start and their turn through it */
  double              *w;
  double              *hat;
  double              *p_int;
  double              *drive;
  double              *asked;
  double              *rate;
  double              *q_step;
  double              *went;
  wbc_loop_t           tanh;        /**< numpy's float64 tanh loop and its data */
  void                *tanh_data;
  double               ahead_wind;  /**< drives.AHEAD_WIND, drives.AHEAD_PLAY; gearbox.PLAY_NM, MESH_RAD_S, RATE_S */
  double               ahead_play;
  double               play_nm;
  double               mesh_rad_s;
  double               rate_s;
  int                  substeps;    /**< gearbox.SUBSTEPS */
  int                  spare;       /**< explicit: its ints a whole number of doubles */
} wbc_boxes_t;

/** What bears does not creep (machine.world.stiction): each of MuJoCo's contacts' friction rows' slip since the
    contact stuck, kept from step to step on its arrays. MuJoCo's friction rows are dampers alone - their reference
    acceleration -B v, their violation naught -, and a load f one holds creeps at R f / B for good. A row whose
    violation is its slip s since the contact stuck (-B v - K I s, K the stiffest its damper holds without ringing,
    (B / 2)^2, I the row's impedance) holds its load at rest, R f / (K I) off where it stuck, within its cone; past
    its cone the slip yields (wbc_world_slipped). MuJoCo lays one friction a contact: a contact stuck holds on it,
    its friction at rest; one that yielded slides on its sliding share of it, and sticks again once its pull is
    back inside that. */
typedef struct
{
  int    *pair;   /**< the last step's contacts' geoms, two a contact (cap) */
  double *at;     /**< where each bore, the world's (3 a contact) */
  double *slip;   /**< each one's slip since it stuck, the world's: along its plane, m (3), about its normal and
                       about its plane, rad (3) */
  double *row;    /**< this step's contacts' friction rows' slips between the two calls: along its two tangents,
                       about its normal, about its tangents (5 a contact) */
  double *held;   /**< what of each one's slip its cone held the last step: under 1 it yielded, it slides (cap) */
  double *borne;  /**< each one's normal force the last step, N: the reach its next step's strain is kept to (cap) */
  const double *grip;  /**< MuJoCo's geoms' friction as its model holds it: slide, spin, roll (3 a geom) */
  const double *share; /**< each geom's sliding friction over its friction at rest (a geom) */
  double  step_s; /**< a step, s */
  double  fade;   /**< 1 / v^2, v the speed where a contact's friction stands half way to its sliding share */
  int     cap;    /**< the contacts kept at most: one past them keeps MuJoCo's rows */
  int     n;      /**< in and out: the last step's contacts kept */
} wbc_stuck_t;

/** The robot's world's steps laid once (tools.cores.world): its pass's struct (wbc_pass_ctrl, wbc_pass_motor,
    wbc_pass_metered), MuJoCo's data, its setpoints and their last stamp, its DC link (machine.body.dclink), its
    room's switch (machine.world.room.touched). */
typedef struct
{
  wbc_world_t           *pass;       /**< the robot's drives on their setpoints (wbc_pass.c) */
  struct mjData_        *d;          /**< MuJoCo's data: its contacts, its friction rows' references */
  const wbc_blas_t      *blas;       /**< numpy's OpenBLAS: the link's dots */
  const double          *target;     /**< each setpoint, its rate, as last stamped (n) */
  double                *rate;
  double                *was;
  double                *link_v;     /**< the link's volts, its peak and what its boards bled, J (0-d each) */
  double                *link_peak;
  double                *link_bled;
  double                *link_id;    /**< each drive's d-axis bleed, A (n) */
  const double          *link_kt;    /**< each drive's N m an amp, its board's ohm, its peak amps, its copper's loss */
  const double          *link_ohm;
  const double          *link_amps;
  const double          *link_loss;
  const unsigned char   *switch_geom;/**< the robot's room's switch's geom and its hands', a byte a geom; NULL no room */
  const unsigned char   *hands;
  wbc_boxes_t           *boxes;      /**< the robot's gearboxes where any is stepped (`World.sprung`); NULL rigid */
  wbc_stuck_t           *stuck;      /**< the world's contacts' slips (machine.world.stiction); NULL MuJoCo's rows */
  double                 house_w;    /**< the robot's boards' housekeeping and switching, W */
  double                 onset_v;    /**< the link's bleed onset and clamp, V; its capacitance, F */
  double                 clamp_v;
  double                 link_f;
  double                 pack_v;     /**< the robot's packs' volts, ohm and most charge, A */
  double                 pack_ohm;
  double                 charge_a;
  double                 step_s;     /**< a step, s; the switch's dead time, s */
  double                 switch_s;
  double                 switched;   /**< in and out: since the switch toggled, s; the lamp on */
  int                    lamp;
  int                    spare;      /**< explicit: its ints a whole number of doubles */
} wbc_steps_t;

/** The robot's setpoints' rates over `span` s since their last stamp where it is past 1e-9 s, each
 * stamped. */
void wbc_world_rated(wbc_steps_t *s, double span);

/** Before MuJoCo's step's second half `since` s past the stamp: the robot's drives' torques, motors and gearboxes
    alone (wbc_world_drives, reading nothing mj_step1 writes), then the contacts stuck on the rows the first half
    laid (wbc_world_stuck_step); wbc_world_before runs both. */
void wbc_world_drives(wbc_steps_t *s, double since);
void wbc_world_stuck_step(wbc_steps_t *s);
void wbc_world_before(wbc_steps_t *s, double since);

/** After MuJoCo's step: the contacts' slips yielded and the room's switch read off its contacts
    (wbc_world_contacts_after, before the next step's mj_step1), then the boxes moved, the drives' meters and the
    DC link stepped (wbc_world_rest_after); wbc_world_after runs both. */
void wbc_world_contacts_after(wbc_steps_t *s);
void wbc_world_rest_after(wbc_steps_t *s);
void wbc_world_after(wbc_steps_t *s);

/** Whether any of MuJoCo's contacts now pairs a geom `a` marks with one `b` marks under `below` m up
    (wbc_pass_paired on its data's contacts). */
int wbc_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below);

/** The robot's skin's bins filled from MuJoCo's contacts as its data `d` stands (wbc_pass_contacts on its
    contacts, their forces and their rows' impedance), the contacts' layout laid on `c` at their first; 1 where the
    world applies a force to any of its bodies (its xfrc_applied: a shove, for the robot's skin to feel), else 0.
    */
int wbc_world_contacted(const struct mjData_ *d, wbc_contacts_t *c);

/** Between MuJoCo's step's halves (after mj_step1, before mj_step2): each contact's friction rows sprung on their
    slip - the slip of the last step's nearest contact of its two geoms (none: naught), carried on by each row's
    speed a step -, each row's reference acceleration less K I s; an elliptic cone's rows alone. A contact whose
    nearest yielded the last step slides: its friction its sliding share of what MuJoCo laid. */
void wbc_world_stuck(struct mjData_ *d, wbc_stuck_t *k);

/** After MuJoCo's step: each contact's slip yielded to what its cone holds - its rows' pulls K I s / R over their
    friction's, past its normal force, scaled back onto it -, kept in the world's frame with its geoms and where
    it bore. */
void wbc_world_slipped(const struct mjData_ *d, wbc_stuck_t *k);

/** The robot's loose things asleep as machine.world.loose.stick holds them a pass (its `_held`
 * arrays). */
typedef struct
{
  const int           *by_geom;  /**< each geom's thing asleep, -1 none (MuJoCo's ngeom) */
  const unsigned char *live;     /**< a byte a geom: 1 where its body moves (not the world's, welded to it) */
  const int           *vel;      /**< the things asleep's velocities along the floor and up, three a thing (nvel) */
  const int           *of;       /**< each thing asleep, in their order (nvel / 3) */
  const int           *at;       /**< their coordinates (nat) and where each lies asleep */
  const double        *lie;
  const int           *still;    /**< their velocities, held at 0 (nstill) */
  unsigned char       *woken;    /**< out: a byte a thing (nthing), 1 where it woke */
  double               low;      /**< a thing asleep moving past this, m/s, woke (half loose.SLIDE_M_S) */
  int                  nvel;
  int                  nat;
  int                  nstill;
  int                  nthing;
} wbc_held_t;

/** The robot's loose things asleep a pass (loose._woke): each that MuJoCo's contacts pair with a geom of something
    that moves, not of a thing asleep, or that moves past `low`, woken; none, each held where it lies, still. How many
    woke - those the caller wakes, then holds the rest. */
int wbc_world_held(struct mjData_ *d, const wbc_held_t *h);

#endif
