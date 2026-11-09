/** wbc_world.h - her world's steps on the host (wbc_world.c): before MuJoCo's step her drives' torques on their
    setpoints and her gearboxes stepped on them; after it the boxes moved, the drives' meters, her DC link and her
    room's lamp switch - machine.world.physics.World.advance between its steps in two calls, numpy's arithmetic
    operation for operation (wbc_np.h), MuJoCo's contacts read off its data through its own headers. The host's
    alone. */
#ifndef WBC_WORLD_H
#define WBC_WORLD_H

#include <stddef.h>

#include "wbc_blas.h"
#include "wbc_pass.h"

/** MuJoCo's data (mujoco/mjdata.h), its contacts read in wbc_world.c alone. */
struct mjData_;

/** A ufunc's own inner loop (numpy's PyUFuncGenericFunction): its arguments' bytes, its length, their strides. */
typedef void (*wbc_loop_t)(char **args, const ptrdiff_t *dimensions, const ptrdiff_t *steps, void *data);

/** Her gearboxes (machine.body.gearbox.Boxes) laid once on its arrays, stepped in place as Boxes.step steps them:
    each joint's numbers, the state its step carries, her world's braking of a shorted drive, and numpy's own tanh
    loop - an array's tanh is numpy's SIMD, not libm's, in a fifth of its values (tools.cores.world.numpy_tanh). */
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

/** Her world's steps laid once (tools.cores.world): her pass's struct (wbc_pass_ctrl, wbc_pass_motor,
    wbc_pass_metered), MuJoCo's data, her setpoints and their last stamp, her DC link (machine.body.dclink), her
    room's switch (machine.world.room.touched). */
typedef struct
{
  wbc_world_t           *pass;       /**< her drives on their setpoints (wbc_pass.c) */
  const struct mjData_  *d;          /**< MuJoCo's data: its contacts */
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
  const unsigned char   *switch_geom;/**< her room's switch's geom and her hands', a byte a geom; NULL no room */
  const unsigned char   *hands;
  wbc_boxes_t           *boxes;      /**< her gearboxes where any is stepped (`World.sprung`); NULL rigid */
  double                 house_w;    /**< her boards' housekeeping and switching, W */
  double                 onset_v;    /**< the link's bleed onset and clamp, V; its capacitance, F */
  double                 clamp_v;
  double                 link_f;
  double                 pack_v;     /**< her packs' volts, ohm and most charge, A */
  double                 pack_ohm;
  double                 charge_a;
  double                 step_s;     /**< a step, s; the switch's dead time, s */
  double                 switch_s;
  double                 switched;   /**< in and out: since the switch toggled, s; the lamp on */
  int                    lamp;
  int                    spare;      /**< explicit: its ints a whole number of doubles */
} wbc_steps_t;

/** Her setpoints' rates over `span` s since their last stamp where it is past 1e-9 s, each stamped. */
void wbc_world_rated(wbc_steps_t *s, double span);

/** Before MuJoCo's step `since` s past the stamp: her drives' torques into ctrl - on her boxes' rotors asked ahead
    and their rates past the links where boxes are laid (Boxes.ahead) -, their motors' (wbc_pass.c), and ctrl what her
    boxes pass on to their links (Boxes.step). */
void wbc_world_before(wbc_steps_t *s, double since);

/** After it: her boxes' rotors past their links where they landed (Boxes.moved), her drives' meters on her rotors'
    speeds (wbc_pass_metered), her DC link a step on (dclink.step), her room's switch toggled where a hand of hers
    touches it, not twice within its dead time (room.touched). */
void wbc_world_after(wbc_steps_t *s);

/** Whether any of MuJoCo's contacts now pairs a geom `a` marks with one `b` marks under `below` m up
    (wbc_pass_paired on its data's contacts). */
int wbc_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below);

#endif
