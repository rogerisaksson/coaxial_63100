/** thermal_run.h - The board's thermal observer a slice at a time, its envelope held. */
#ifndef THERMAL_RUN_H
#define THERMAL_RUN_H

#include "thermal.h"
#include "thermal_ident.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One code for every executive (the user, 2026-10-10): the part runs it from board_thermal.c,
   the stand-in from coaxial.simulated.thermal; each lays its own record and gathers its own
   load, samples and clock. */

/** The thermometers' floor for the identification, kelvin: the NTC quantises at about 30 mK
    and TSEN at 125 mK, and a sample is one reading of each. */
#define THERMAL_IDENT_NOISE_K 0.1f

/** What a ceiling's span is measured up from when the margin trims it, degrees C. */
#define THERMAL_MARGIN_REF_C 25.0f

/** How far the margin moves before the ceilings are trimmed again: a thousandth, the wire's. */
#define THERMAL_MARGIN_STEP 0.001f

/** The trip cap, what the margin is held to after the envelope has dropped the stage, and how
    fast that hold lets go. */
#define THERMAL_TRIP_MARGIN        0.70f
#define THERMAL_TRIP_RECOVER_PER_S (0.30f / 1800.0f)

/** How fast the applied derate recovers, per second; down is at once. */
#define THERMAL_DERATE_RECOVER_PER_S 0.05f

/** The winding's K/W from the copper into the iron, the rest the iron's own air path. */
#define THERMAL_WINDING_INTO_IRON 0.25f

typedef struct
{
  thermal_t th;
  thermal_loss_t loss;
  thermal_power_t power;
  thermal_cfg_t base;            /**< the record's network, the scales applied over it */
  thermal_ident_t ident;
  thermal_soa_t record;          /**< the record's envelope, untrimmed */
  float margin_floor;            /**< the record's floor the margin rises from, 0..1 */
  thermal_soa_t soa;             /**< the envelope the margin trims */
  thermal_budget_t budget;
  float margin;
  float trip_cap;                /**< one with no trip in hand */
  uint32_t trip_ms;
  float winding_derate;          /**< the winding's own factor beside the whole's */
  float derate;                  /**< as applied: its recovery slewed */
  uint32_t trips;
} thermal_run_t;

typedef struct
{
  thermal_load_t load;           /**< with the airspeed the executive holds */
  thermal_sense_t seen;          /**< NAN where nothing answered this slice */
  uint32_t slice_ms;
  uint32_t clock_ms;             /**< the observer's clock */
  bool wep;                      /**< the derate held off (thermal op 14) */
  bool switching;                /**< the stage enabled: a trip drops it */
} thermal_run_in_t;

typedef struct
{
  float derate;                  /**< what the current clamp is multiplied by */
  bool drop;                     /**< the stage dropped now: a node at its record's ceiling */
} thermal_run_out_t;

/** Fresh: `base` and `loss` laid, the scales at one, every node and the room at `start_c`. */
void thermal_run_init(thermal_run_t *r, const thermal_cfg_t *base, const thermal_loss_t *loss,
                      float start_c);

/** The identification begun again at the room the observer holds, over `r->base`. */
void thermal_run_identify(thermal_run_t *r);

/** The record's network again under the identified scales: `r->base` laid anew. */
void thermal_run_refresh(thermal_run_t *r);

/** A record's winding into `cfg`: its J/K, and its K/W to the air split into the iron
    (THERMAL_WINDING_INTO_IRON) and the iron's own path where it has one. */
void thermal_run_winding(thermal_cfg_t *cfg, float k_per_w, float j_per_k);

/** The trip cap at `clock_ms`: what it was set to and what the minutes since gave back. */
float thermal_run_trip_cap(const thermal_run_t *r, uint32_t clock_ms);

/** The ceilings trimmed from `r->record` by the margin at `clock_ms`: the identification's for
    its doubt over `r->margin_floor`, or the trip cap, the less. */
void thermal_run_envelope(thermal_run_t *r, uint32_t clock_ms);

/** The ceilings trimmed again where the margin has moved a step since. */
void thermal_run_follow(thermal_run_t *r, uint32_t clock_ms);

/** A slice: the losses on the estimate's own legs, the step, the identification beside it,
    the room, the budget, the clamp's derate held - `between` run between the costly parts. */
void thermal_run_slice(thermal_run_t *r, const thermal_run_in_t *in, thermal_run_out_t *out,
                       void (*between)(void));

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_RUN_H */
