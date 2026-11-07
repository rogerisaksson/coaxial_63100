/** wbc_world.h - her world's steps on the host (wbc_world.c): before MuJoCo's step her drives' torques on their
    setpoints; after it their meters, her DC link and her room's lamp switch - machine.world.physics.World.advance
    between its steps in two calls, numpy's arithmetic operation for operation (wbc_np.h), MuJoCo's contacts read
    off its data through its own headers. The host's alone. */
#ifndef WBC_WORLD_H
#define WBC_WORLD_H

#include "wbc_blas.h"
#include "wbc_pass.h"

/** MuJoCo's data (mujoco/mjdata.h), its contacts read in wbc_world.c alone. */
struct mjData_;

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

/** Before MuJoCo's step `since` s past the stamp: her drives' torques into ctrl, their motors' (wbc_pass.c). */
void wbc_world_before(wbc_steps_t *s, double since);

/** After it: her drives' meters (wbc_pass_metered), her DC link a step on (dclink.step), her room's switch toggled
    where a hand of hers touches it, not twice within its dead time (room.touched). */
void wbc_world_after(wbc_steps_t *s);

/** Whether any of MuJoCo's contacts now pairs a geom `a` marks with one `b` marks under `below` m up
    (wbc_pass_paired on its data's contacts). */
int wbc_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below);

#endif
