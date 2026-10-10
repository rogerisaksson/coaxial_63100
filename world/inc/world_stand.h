/** world_stand.h - A stand-in board's heat: the board's own observer beside its world's truth. */
#ifndef WORLD_STAND_H
#define WORLD_STAND_H

#include "thermal_run.h"
#include "world_heat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The stand-in (coaxial.simulated.thermal) runs the code the board runs (thermal_run) on the
   truth the emulated worlds run (world_heat): one array of slots, a step a call, the host's
   Python its edge - the load, the room, the reads (the user, 2026-10-10). */

/** The stand-ins a process holds. */
#define WORLD_STANDS 64

/** A slice of the pair, model ms: the board's THERMAL_STEP_MS. */
#define WORLD_STAND_SLICE_MS 100U

/** Each leg's mean square follows its square at this constant, s: the board's sync window. */
#define WORLD_STAND_RMS_TAU_S 0.5f

/** The run's row, in order (`STAND_IN`, coaxial.simulated.thermal): AFE_ON, the bridge
    switching, the three duties, the legs' amps, the link's volts and amps (<0 off the legs),
    the rotor's rpm, the dead time s, the meter held by the drive (no MCU die), the envelope
    acting on the stage. */
#define WORLD_STAND_IN 14

/** The run's out: the derate as last applied, whether a trip dropped the stage, the steps. */
#define WORLD_STAND_OUT 3

typedef struct
{
  thermal_run_t run;            /**< the board's observer, its identification and envelope */
  world_heat_t truth;           /**< what it watches, in its room */
  thermal_t record;             /**< the record's network in still air: its cfg */
  thermal_app_t app;            /**< what the board is mounted in */
  float squares[3];             /**< each leg's mean square, A^2 */
  uint32_t clock_ms;            /**< the observer's clock, model ms */
  uint32_t sampled_ms;
  uint32_t every_ms;            /**< how often the thermometers are read, 0 never */
  uint32_t noise;               /**< xorshift32's state: the thermometers' noise */
  float noise_k;                /**< +- this */
  thermal_sense_t seen;         /**< the last sample, NAN where none */
  thermal_sense_t now;          /**< the truth's three thermometers now, noiseless */
  bool sampled;
  uint32_t wep_until_ms;
  uint32_t told_until_ms;
  float told_airspeed;          /**< the host's word, m/s, held to told_until_ms */
  float truth_airspeed;         /**< the air the truth flies in, m/s */
  float cycle[4];               /**< amps, on s, off s, from s: amps 0 none */
  int32_t cycle_trip;           /**< the cycle a trip ended early, -1 none */
  uint32_t steps;
  float speed_rpm;
} world_stand_t;

/** Slot `i` fresh: the record's winding over the core's network, mounted in `app`, every node
    of both and the room at `start_c`; the thermometers' noise +-`noise_k` from `seed`. */
void world_stand_init(int i, int app, float winding_k_per_w, float winding_j_per_k,
                      float r_phase, float start_c, float noise_k, uint32_t seed);

/** The record's envelope: each node's ceiling C (0 none), which the clamp cannot cool, where
    derating starts and the window it keeps, the margin's floor - the ceilings trimmed now. */
void world_stand_envelope(int i, const float *limit_c, uint32_t undriven, float throttle_at,
                          float lookahead_s, float margin_floor);

/** The truth's room: ambient C, its air path and capacity scaled. */
void world_stand_room(int i, float ambient, float air, float capacity);

/** Mounted in `app`: both networks laid again on it, the identification begun again. */
void world_stand_application(int i, int app);

/** The record's network changed: `what` 0 a node's first path out and capacity, 1 an edge's
    K/W (`node` its index), 2 the bulk's K/W and J/K, 3 the winding's K/W and J/K; the truth's
    network with it, the observer's under its scales. False where the core refuses. */
bool world_stand_network(int i, int what, int node, float a, float b);

/** The identification begun again at the room the observer holds. */
void world_stand_identify(int i);

/** The thermometers read every `every_ms` of the observer's clock, 0 never. */
void world_stand_every(int i, uint32_t every_ms);

/** War emergency power: the derate held at one `ms` from now. */
void world_stand_wep(int i, uint32_t ms);

/** The host's word on the airspeed, m/s, held `hold_ms`; the truth's own air, m/s. */
void world_stand_airspeed(int i, float told, uint32_t hold_ms, float truth);

/** The load cycle: `amps` on all three legs for `on_s`, idle `off_s`, from now; 0 amps none. */
void world_stand_cycle(int i, float amps, float on_s, float off_s);

/** `seconds` of model time on `in` (WORLD_STAND_IN) - the cycle's where one runs -, slice by
    slice: the truth, its thermometers when due, the board's run; `out` WORLD_STAND_OUT. No
    seconds is a slice of none: the envelope judged where the observer stands. */
void world_stand_run(int i, const float *in, float seconds, float *out);

/** The winding's resistance in both the observer's losses and the truth's, ohm. */
void world_stand_phase_r(int i, float ohm);

/** An observer's node at `celsius` where it stands: a test's way to a hot leg. */
void world_stand_place(int i, int node, float celsius);

/** Both networks `seconds` on `in`'s power with nothing read nor identified - a board after an
    hour of idling -, the identification's room the truth's. */
void world_stand_settle(int i, const float *in, float seconds);

/** The views a reader names (coaxial.simulated.thermal), each a row of floats in this order:
    STATE the observer's nodes, its room, its thermistor, settled, the last sample's NTC, AFE
    and MCU (NAN none), whether any, ms since it, the clock ms, the steps, the rpm, the told
    airspeed while it holds and the truth's, each leg's junction over its node; BUDGET each
    node's spend 0..255, the worst, its node, ms to the ceiling, throttling, tripped, the
    derate applied and asked, each node's soak J, the winding's C, spend and factor, the trips;
    IDENT the state, each recorded scale's value, sigma and whether online, the innovation K,
    the margin, the updates, the room C and its sigma, the floor, the trip cap, the margin the
    identification earns alone; NETWORK each node's J/K, K/W to the air, share, die K/W and
    forced gain, each edge's two nodes and K/W, the bulk's K/W, its radiated share, what the
    NTC sees, its lag s, the face to the stator W/K; TRUTH its nodes, its three thermometers,
    its room, the cycle's amps now and asked. */
enum { WORLD_STAND_STATE, WORLD_STAND_BUDGET, WORLD_STAND_IDENT, WORLD_STAND_NETWORK,
       WORLD_STAND_TRUTH };

/** View `what` of slot `i` into `out`; how many floats. */
int world_stand_read(int i, int what, float *out);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_STAND_H */
