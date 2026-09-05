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
  * to ask narrow questions about the graph and the envelope - the derate
  * ramp, the lookahead, the soak joules, a leg warming its neighbour - and
  * those are the parts that will act on the gates.
  *
  * NOTHING CROSSES AS A STRUCT. A ctypes mirror of `thermal_budget_t` would
  * be a second declaration of a layout the compiler already owns, and the
  * two would drift the first time a field was appended - which is exactly
  * what this file exists to test. Flat float arrays in the orders the
  * BUDGET_ORDER / LOAD_ORDER / LOSS_ORDER / CFG_ORDER comments give, and
  * the Python side names them by the same lists.
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
   switching, afe_on, phase_sq[0..2], speed_rpm, t_dead_s. */
#define LOAD_SLOTS 15

/* LOSS_ORDER: rds_on, rds_alpha, r_shunt, r_hotswap, switching_watt,
   switch_volts, driver_share, mcu_watt, ldo_watt, afe_watt, f_sw,
   coss_cjo, coss_m, coss_vj, t_switch_s, v_sd, q_g, v_drive, buck_eff,
   r_phase, k_iron. */
#define LOSS_SLOTS 21

/* CFG_ORDER, per node: capacity, to_ambient, area_share, rth_die, forced. */
#define CFG_PER_NODE 5


API int thm_nodes(void)
{
  return (int)THERMAL_NODES;
}


API int thm_edges(void)
{
  return THERMAL_EDGES;
}


API int thm_budget_slots(void)
{
  return BUDGET_SLOTS;
}


API int thm_load_slots(void)
{
  return LOAD_SLOTS;
}


API int thm_loss_slots(void)
{
  return LOSS_SLOTS;
}


/** Which two nodes an edge joins; -1 for no such edge. */
API int thm_edge_end(int edge, int which)
{
  if ((edge < 0) || (edge >= THERMAL_EDGES))
  {
    return -1;
  }
  return which ? (int)THERMAL_EDGE_ENDS[edge].b
               : (int)THERMAL_EDGE_ENDS[edge].a;
}


API int thm_sink_edge(int node)
{
  return thermal_sink_edge((thermal_node_t)node);
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


/** Put a node at a temperature outright: a test needs to stand the model
  * somewhere, not drive it there. */
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


API int thm_set_node(thermal_t *th, int node, float k_per_w, float capacity)
{
  if ((th == NULL) || (node < 0) || (node >= (int)THERMAL_NODES))
  {
    return 0;
  }
  return thermal_set_node(th, (thermal_node_t)node, k_per_w, capacity)
         ? 1 : 0;
}


API int thm_set_edge(thermal_t *th, int edge, float k_per_w)
{
  return ((th != NULL) && thermal_set_edge(th, edge, k_per_w)) ? 1 : 0;
}


API int thm_set_board(thermal_t *th, float to_ambient, float capacity)
{
  return ((th != NULL) && thermal_set_board(th, to_ambient, capacity)) ? 1 : 0;
}


API float thm_edge_r(const thermal_t *th, int edge)
{
  if ((th == NULL) || (edge < 0) || (edge >= THERMAL_EDGES))
  {
    return NAN;
  }
  return th->cfg.r_edge[edge];
}


/** The whole node table, CFG_ORDER per node, `out` CFG_PER_NODE * N long. */
API void thm_cfg(const thermal_t *th, float *out)
{
  if ((th == NULL) || (out == NULL))
  {
    return;
  }
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    const thermal_node_cfg_t *n = &th->cfg.node[i];

    out[CFG_PER_NODE * i + 0] = n->capacity;
    out[CFG_PER_NODE * i + 1] = n->to_ambient;
    out[CFG_PER_NODE * i + 2] = n->area_share;
    out[CFG_PER_NODE * i + 3] = n->rth_die;
    out[CFG_PER_NODE * i + 4] = n->forced;
  }
}


/** The bulk's five scalars: board_to_ambient, board_cal_rise_k,
  * board_rad_share, ntc_sees, ntc_tau_s. */
API void thm_bulk(const thermal_t *th, float *out)
{
  if ((th == NULL) || (out == NULL))
  {
    return;
  }
  out[0] = th->cfg.board_to_ambient;
  out[1] = th->cfg.board_cal_rise_k;
  out[2] = th->cfg.board_rad_share;
  out[3] = th->cfg.ntc_sees;
  out[4] = th->cfg.ntc_tau_s;
}


API void thm_set_rad_board_stator(thermal_t *th, float w_per_k)
{
  if (th != NULL)
  {
    th->cfg.rad_board_stator = w_per_k;
  }
}


/** The modelled thermistor reading - the lagged state, not the algebra. */
API float thm_ntc(const thermal_t *th)
{
  return (th == NULL) ? NAN : thermal_expected_ntc(th);
}


API float thm_capacity(const thermal_t *th, int node)
{
  if ((th == NULL) || (node < 0) || (node >= (int)THERMAL_NODES))
  {
    return NAN;
  }
  return th->cfg.node[node].capacity;
}


API float thm_to_ambient_at(const thermal_t *th, int node, float rise_k,
                            float speed_rpm)
{
  if (th == NULL)
  {
    return NAN;
  }
  return thermal_to_ambient_at(&th->cfg, (thermal_node_t)node, rise_k,
                               speed_rpm);
}


/** LOAD_ORDER into the struct - one place. */
static void load_from(thermal_load_t *in, const float *load)
{
  memset(in, 0, sizeof(*in));
  in->phase_amps[0] = load[0];
  in->phase_amps[1] = load[1];
  in->phase_amps[2] = load[2];
  in->duty[0] = load[3];
  in->duty[1] = load[4];
  in->duty[2] = load[5];
  in->link_volts = load[6];
  in->link_amps = load[7];
  in->switching = (load[8] != 0.0f);
  in->afe_on = (load[9] != 0.0f);
  in->phase_sq[0] = load[10];
  in->phase_sq[1] = load[11];
  in->phase_sq[2] = load[12];
  in->speed_rpm = load[13];
  in->t_dead_s = load[14];
}


/** One integration step at a speed. `watt` is THERMAL_NODES long; a NaN
  * in a sensor means it is not answering, which is what the board passes
  * when the AFE is off. */
API void thm_step_at(thermal_t *th, const float *watt,
                     float ntc_c, float afe_c, float mcu_c, float speed_rpm,
                     float dt_s)
{
  thermal_power_t p;
  thermal_sense_t seen;
  thermal_load_t load;
  const float ambient = (th != NULL) ? th->ambient : 0.0f;

  if ((th == NULL) || (watt == NULL))
  {
    return;
  }
  memset(&p, 0, sizeof(p));
  memset(&load, 0, sizeof(load));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
  }
  seen.ntc_c = ntc_c;
  seen.afe_c = afe_c;
  seen.mcu_c = mcu_c;
  load.speed_rpm = speed_rpm;
  thermal_step(th, &p, &seen, &load, dt_s);
  th->ambient = ambient;               /* the room does not drift here */
}


API void thm_step(thermal_t *th, const float *watt,
                  float ntc_c, float afe_c, float mcu_c, float dt_s)
{
  thm_step_at(th, watt, ntc_c, afe_c, mcu_c, 0.0f, dt_s);
}


static void soa_from(thermal_soa_t *soa, const float *limit_c,
                     float throttle_at, float lookahead_s,
                     const float *undriven)
{
  memset(soa, 0, sizeof(*soa));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    soa->limit_c[i] = limit_c[i];
    /* Floats because nothing crosses this boundary as anything else -
       NULL is every node driven, which is the struct's own default. */
    soa->undriven[i] = (undriven != NULL) && (undriven[i] != 0.0f);
  }
  soa->throttle_at = throttle_at;
  soa->lookahead_s = lookahead_s;
}


/** The envelope, flattened. `limit_c` is THERMAL_NODES long - the ceilings
  * come from the caller because they come from the calibration record, and
  * there is no compiled-in copy to ask for (invariant 10). */
API void thm_budget(const thermal_t *th, const float *watt,
                    const float *limit_c, float throttle_at,
                    float lookahead_s, const float *undriven, float *out)
{
  thermal_power_t p;
  thermal_soa_t soa;
  thermal_budget_t b;

  if ((th == NULL) || (watt == NULL) || (limit_c == NULL) || (out == NULL))
  {
    return;
  }
  memset(&p, 0, sizeof(p));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
  }
  soa_from(&soa, limit_c, throttle_at, lookahead_s, undriven);
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


/** One node's own clamp factor under the same envelope. */
API float thm_node_derate(const thermal_t *th, const float *watt,
                          const float *limit_c, float throttle_at,
                          float lookahead_s, const float *undriven, int node)
{
  thermal_power_t p;
  thermal_soa_t soa;

  if ((th == NULL) || (watt == NULL) || (limit_c == NULL))
  {
    return NAN;
  }
  memset(&p, 0, sizeof(p));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
  }
  soa_from(&soa, limit_c, throttle_at, lookahead_s, undriven);
  return thermal_node_derate(th, &p, &soa, (thermal_node_t)node);
}


/** The junction on a node at a power split. */
API float thm_junction(const thermal_t *th, const float *watt, int node)
{
  thermal_power_t p;

  if ((th == NULL) || (watt == NULL))
  {
    return NAN;
  }
  memset(&p, 0, sizeof(p));
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    p.watt[i] = watt[i];
  }
  return thermal_junction(th, &p, (thermal_node_t)node);
}


/** The power estimator. `load` is LOAD_SLOTS long, `phase_c` three node
  * temperatures or NULL for the flat 25 C figure, `out` THERMAL_NODES.
  * `r_phase` overrides the loss table's placeholder when positive, the
  * way the board's glue hands the record's in. */
API void thm_power_r(const float *load, const float *phase_c, float r_phase,
                     float *out)
{
  thermal_load_t in;
  thermal_loss_t loss;
  thermal_power_t p;

  if ((load == NULL) || (out == NULL))
  {
    return;
  }
  load_from(&in, load);

  thermal_losses(&loss);
  if (r_phase > 0.0f)
  {
    loss.r_phase = r_phase;
  }
  thermal_power_estimate(&p, &in, &loss, phase_c);
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    out[i] = p.watt[i];
  }
}


API void thm_power(const float *load, const float *phase_c, float *out)
{
  thm_power_r(load, phase_c, 0.0f, out);
}


API float thm_coss_energy(float volts)
{
  thermal_loss_t loss;

  thermal_losses(&loss);
  return thermal_coss_energy(&loss, volts);
}


/** The loss constants, LOSS_ORDER, so a test can check the split against
  * the parts rather than against a number typed twice. */
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
  out[10] = loss.f_sw;
  out[11] = loss.coss_cjo;
  out[12] = loss.coss_m;
  out[13] = loss.coss_vj;
  out[14] = loss.t_switch_s;
  out[15] = loss.v_sd;
  out[16] = loss.q_g;
  out[17] = loss.v_drive;
  out[18] = loss.buck_eff;
  out[19] = loss.r_phase;
  out[20] = loss.k_iron;
}
