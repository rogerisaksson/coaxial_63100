/** wbc_bus.h - the master bus insert (machine.supervisor, anti_spasm_supervisor.lisp): its stages
    on her drives' flat arrays at the drives' pass, 1 kHz - her ask's jerk, her torque command as
    her drives take it. Portable C11. */
#ifndef WBC_BUS_H
#define WBC_BUS_H

/** The insert's numbers, as its text says them (machine.supervisor lays them). */
typedef struct
{
  double gate_rad_s;    /**< the D terms' rate: its gate's deadband, rad/s, its attack and release, s; */
  double gate_attack_s;
  double gate_release_s;
  double d_b[3];        /**< its low-pass, a biquad, its a0 1 */
  double d_a[2];
  double tank_max_j;    /**< the tank: its capacity, its floor, J */
  double tank_min_j;
  double hp_b[3];       /**< its detector's window on rate and torque: a high-pass at its low edge, */
  double hp_a[2];       /**< a low-pass at its high edge, biquads; the window's longest period, s - */
  double lp_b[3];       /**< a quiet half cycle that long clears the count */
  double lp_a[2];
  double period_s;
  double reversals;     /**< reversals in a row, each half cycle's band power past trip_w (W), that trip it */
  double trip_w;
  double inject;        /**< the damping injected, times a drive's own; its gain's rollback; the release, s */
  double rollback;
  double recover_s;
} wbc_bus_k;

/** The bus's state, a drive each (n), pointers into her law's arrays. */
typedef struct
{
  int                  n;
  const double        *slew;     /**< each drive's own slew, N m/s: its peak times its current loop's
                                      bandwidth, rad/s; and jerk, rad/s^3: that over its rotor's inertia */
  const double        *jerk;
  double              *qa_out;   /**< the acceleration and her law's torque command given, last pass; */
  double              *tau_out;
  double              *bv;       /**< the detector's window's delays: rate (4, n), torque (4, n) */
  double              *bt;
  double              *sign;     /**< each rate's band's sign, its half cycle's energy and time, its count */
  double              *half_e;
  double              *half_t;
  double              *count;
  double              *gate;     /**< each rate's gate, 0 shut to 1 open; its low-pass's two delays; */
  double              *dz1;
  double              *dz2;
  double              *qd;       /**< out: the rate its D terms take, rad/s; */
  double              *alpha;    /**< how far each is given to the passivity controller, 0 to 1; */
  double              *given;    /**< what the bus changed of its torque command, N m */
  double               energy;   /**< the tank, J; its power this pass, W */
  double               power;
  int                  primed;   /**< 0 from a lay: the first pass's command taken whole */
} wbc_bus_t;

/** Drive `i`'s rate `qd` as its D terms take it (b->qd too): gated under the deadband, smooth-step
    over its attack and release, then low-passed. */
double wbc_bus_rate(const wbc_bus_k *k, wbc_bus_t *b, int i, double dt, double qd);

/** Drive `i`'s asked acceleration `qa` through the jerk's soft clip: the acceleration given - its
    torque less its inertia's share of what is cut (wbc_pass.c). */
double wbc_bus_ask(wbc_bus_t *b, int i, double dt, double qa);

/** Drive `i`'s torque command from her law `tau` through the slew's soft clip: the command given -
    her drive's observer below it, untold, restores the motion she asked at its own bandwidth. */
double wbc_bus_slew(wbc_bus_t *b, int i, double dt, double tau);

/** Drive `i`'s torque command `feed` at rate `qd`, its own damping `damping` (N m s/rad): the
    tank's detector and its passivity controller - the command given. */
double wbc_bus_feed(const wbc_bus_k *k, wbc_bus_t *b, int i, double dt, double qd, double feed, double damping);

/** The pass's end, after every drive: the tank on the pass's band power. */
void wbc_bus_pass(const wbc_bus_k *k, wbc_bus_t *b, double dt);

#endif
