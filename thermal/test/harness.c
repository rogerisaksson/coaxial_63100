/**
  ******************************************************************************
  * @file    harness.c
  * @brief   A flat C API over thermal/, so test_thermal_core.py can run the
  *          observer and its envelope on the host through ctypes.
  *
  * Built by the Python suite with the host gcc, never by the firmware build.
  * Test scaffolding; it must not appear in the root CMakeLists.
  *
  * `check.c` beside this is a different thing and stays: it is the
  * CALIBRATION CAMPAIGN's own report, prose about whether the network
  * reproduces four camera-measured states. This is the API a suite drives
  * to ask narrow questions about the envelope - the derate ramp, the
  * lookahead, the soak joules - and those are the parts that will act on
  * the gates.
  *
  * NOTHING CROSSES AS A STRUCT. A ctypes mirror of `thermal_budget_t` would
  * be a second declaration of a layout the compiler already owns, and the
  * two would drift the first time a field was appended - which is exactly
  * what this file exists to test. Flat float arrays in the orders the
  * BUDGET_ORDER / LOAD_ORDER comments give, and the Python side names them
  * by the same lists.
  ******************************************************************************
  */
#include "thermal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

/* BUDGET_ORDER: worst, worst_node, millis_to_limit, throttling, tripped,
   derate, then used[0..N-1], then soak_j[0..N-1]. `worst` and `used` come
   back as the FRACTIONS the bytes stand for, because a test that says 216
   is testing the encoding and one that says 0.847 is testing the budget. */
#define BUDGET_SLOTS (6 + 2 * THERMAL_NODES)

/* LOAD_ORDER: phase_amps[0..2], duty[0..2], link_volts, link_amps,
   switching, afe_on, phase_sq[0..2]. */
#define LOAD_SLOTS 13


API int thm_nodes(void)
{
  return (int)THERMAL_NODES;
}


API int thm_budget_slots(void)
{
  return BUDGET_SLOTS;
}


API thermal_t *thm_new(float celsius)
{
  thermal_t *th = calloc(1U, sizeof(*th));
  thermal_cfg_t cfg;

  if (th != NULL)
  {
    thermal_defaults(&cfg);
    thermal_init(th, &cfg, celsius);
    th->ambient = celsius;
  }
  return th;
}


API void thm_free(thermal_t *th)
{
  free(th);
}


API void thm_ambient(thermal_t *th, float celsius)
{
  if (th != NULL)
  {
    th->ambient = celsius;
  }
}


/** Put a node at a temperature outright.
  *
  * A TEST NEEDS TO STAND THE MODEL SOMEWHERE, not drive it there: the
  * derate ramp is a function of where a node is, and heating one to
  * ninety points on the band would take ninety integrations and measure
  * the network rather than the ramp.
  */
API void thm_place(thermal_t *th, int node, float celsius)
{
  if ((th != NULL) && (node >= 0) && (node < (int)THERMAL_NODES))
  {
    th->t[node] = celsius;
  }
}


API float thm_at(const thermal_t *th, int node)
{
  if ((th == NULL) || (node < 0) || (node >= (int)THERMAL_NODES))
  {
    return NAN;
  }
  return th->t[node];
}


API float thm_capacity(const thermal_t *th, int node)
{
  if ((th == NULL) || (node < 0) || (node >= (int)THERMAL_NODES))
  {
    return NAN;
  }
  return th->cfg.node[node].capacity;
}


/** One integration step. `watt` is THERMAL_NODES long; a NaN in `seen`
  * means that sensor is not answering, which is what the board passes when
  * the AFE is off.
  */
API void thm_step(thermal_t *th, const float *watt,
                  float ntc_c, float afe_c, float mcu_c, float dt_s)
{
  thermal_power_t p;
  thermal_sense_t seen;
  const float ambient = (th != NULL) ? th->ambient : 0.0f;

  if ((th == NULL) || (watt == NULL))
  {
    return;
  }
  memset(&p, 0, sizeof(p));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
  }
  seen.ntc_c = ntc_c;
  seen.afe_c = afe_c;
  seen.mcu_c = mcu_c;
  thermal_step(th, &p, &seen, dt_s);
  th->ambient = ambient;               /* the room does not drift here */
}


/** The envelope, flattened. `limit_c` is THERMAL_NODES long - the ceilings
  * come from the caller because they come from the calibration record, and
  * there is no compiled-in copy to ask for (invariant 10).
  */
API void thm_budget(const thermal_t *th, const float *watt,
                    const float *limit_c, float throttle_at,
                    float lookahead_s, float *out)
{
  thermal_power_t p;
  thermal_soa_t soa;
  thermal_budget_t b;

  if ((th == NULL) || (watt == NULL) || (limit_c == NULL) || (out == NULL))
  {
    return;
  }
  memset(&p, 0, sizeof(p));
  memset(&soa, 0, sizeof(soa));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
    soa.limit_c[i] = limit_c[i];
  }
  soa.throttle_at = throttle_at;
  soa.lookahead_s = lookahead_s;
  thermal_budget(th, &p, &soa, &b);

  out[0] = (float)b.worst / 255.0f;
  out[1] = (float)b.worst_node;
  out[2] = (float)b.millis_to_limit;
  out[3] = b.throttling ? 1.0f : 0.0f;
  out[4] = b.tripped ? 1.0f : 0.0f;
  out[5] = b.derate;
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    out[6 + i] = (float)b.used[i] / 255.0f;
    out[6 + (int)THERMAL_NODES + i] = b.soak_j[i];
  }
}


/** The power estimator. `load` is LOAD_SLOTS long, `phase_c` three node
  * temperatures or NULL for the flat 25 C figure, `out` THERMAL_NODES.
  */
API void thm_power(const float *load, const float *phase_c, float *out)
{
  thermal_load_t in;
  thermal_loss_t loss;
  thermal_power_t p;

  if ((load == NULL) || (out == NULL))
  {
    return;
  }
  memset(&in, 0, sizeof(in));
  in.phase_amps[0] = load[0];
  in.phase_amps[1] = load[1];
  in.phase_amps[2] = load[2];
  in.duty[0] = load[3];
  in.duty[1] = load[4];
  in.duty[2] = load[5];
  in.link_volts = load[6];
  in.link_amps = load[7];
  in.switching = (load[8] != 0.0f);
  in.afe_on = (load[9] != 0.0f);
  in.phase_sq[0] = load[10];
  in.phase_sq[1] = load[11];
  in.phase_sq[2] = load[12];

  thermal_losses(&loss);
  thermal_power_estimate(&p, &in, &loss, phase_c);
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    out[i] = p.watt[i];
  }
}


/** The loss constants, so a test can check the split against the parts
  * rather than against a number typed twice. ORDER: rds_on, rds_alpha,
  * r_shunt, r_hotswap, switching_watt, switch_volts, driver_share,
  * mcu_watt, ldo_watt, afe_watt.
  */
API void thm_losses(float *out)
{
  thermal_loss_t loss;

  if (out == NULL)
  {
    return;
  }
  thermal_losses(&loss);
  out[0] = loss.rds_on;
  out[1] = loss.rds_alpha;
  out[2] = loss.r_shunt;
  out[3] = loss.r_hotswap;
  out[4] = loss.switching_watt;
  out[5] = loss.switch_volts;
  out[6] = loss.driver_share;
  out[7] = loss.mcu_watt;
  out[8] = loss.ldo_watt;
  out[9] = loss.afe_watt;
}
