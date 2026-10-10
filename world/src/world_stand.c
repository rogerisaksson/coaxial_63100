/** world_stand.c - A stand-in board's heat: the board's own observer beside its world's truth. */
#include "world_stand.h"

#include <math.h>
#include <string.h>

static world_stand_t s[WORLD_STANDS];

static world_stand_t *at(int i)
{
  return ((i >= 0) && (i < WORLD_STANDS)) ? &s[i] : NULL;
}

/** The thermometers' noise: uniform in +-k, xorshift32 - deterministic from the seed. */
static float noisy(world_stand_t *b, float value)
{
  uint32_t x = b->noise;

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  b->noise = x;
  return value + ((((float)(x >> 8) / 16777216.0f) - 0.5f) * 2.0f * b->noise_k);
}

/** Both networks on the record's: the observer's base under its scales, the truth's in its
    room. */
static void lay(world_stand_t *b)
{
  thermal_cfg_t base = b->record.cfg;

  thermal_application(&base, b->app);
  b->run.base = base;
  thermal_run_refresh(&b->run);
  world_heat_base(&b->truth, &b->record.cfg);
}

void world_stand_init(int i, int app, float winding_k_per_w, float winding_j_per_k,
                      float r_phase, float start_c, float noise_k, uint32_t seed)
{
  world_stand_t *b = at(i);
  thermal_cfg_t cfg;
  thermal_loss_t loss;

  if (b == NULL)
  {
    return;
  }
  memset(b, 0, sizeof(*b));
  thermal_defaults(&cfg);
  thermal_run_winding(&cfg, winding_k_per_w, winding_j_per_k);
  thermal_init(&b->record, &cfg, start_c);
  b->app = (thermal_app_t)app;
  thermal_application(&cfg, b->app);
  thermal_losses(&loss);
  loss.r_phase = r_phase;
  thermal_run_init(&b->run, &cfg, &loss, start_c);
  world_heat_init(&b->truth, start_c);
  b->truth.loss.r_phase = r_phase;
  world_heat_base(&b->truth, &b->record.cfg);
  world_heat_application(&b->truth, b->app);
  b->noise = (seed != 0U) ? seed : 0x9E3779B9U;
  b->noise_k = noise_k;
  b->seen.ntc_c = b->seen.afe_c = b->seen.mcu_c = NAN;
  b->cycle_trip = -1;
}

void world_stand_envelope(int i, const float *limit_c, uint32_t undriven, float throttle_at,
                          float lookahead_s, float margin_floor)
{
  world_stand_t *b = at(i);

  if (b == NULL)
  {
    return;
  }
  memset(&b->run.record, 0, sizeof(b->run.record));
  for (int n = 0; n < (int)THERMAL_NODES; n++)
  {
    b->run.record.limit_c[n] = limit_c[n];
    b->run.record.undriven[n] = ((undriven >> n) & 1U) != 0U;
  }
  b->run.record.throttle_at = throttle_at;
  b->run.record.lookahead_s = lookahead_s;
  b->run.margin_floor = margin_floor;
  thermal_run_envelope(&b->run, b->clock_ms);
}

void world_stand_room(int i, float ambient, float air, float capacity)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    world_heat_room(&b->truth, ambient, air, capacity);
  }
}

void world_stand_application(int i, int app)
{
  world_stand_t *b = at(i);

  if (b == NULL)
  {
    return;
  }
  b->app = (thermal_app_t)app;
  world_heat_application(&b->truth, b->app);
  lay(b);
  thermal_run_identify(&b->run);
  thermal_run_envelope(&b->run, b->clock_ms);
}

bool world_stand_network(int i, int what, int node, float a, float c)
{
  world_stand_t *b = at(i);
  bool took = false;

  if (b == NULL)
  {
    return false;
  }
  switch (what)
  {
    case 0:  took = thermal_set_node(&b->record, (thermal_node_t)node, a, c); break;
    case 1:  took = thermal_set_edge(&b->record, node, a); break;
    case 2:  took = thermal_set_board(&b->record, a, c); break;
    default: took = (a > 0.0f) && (c > 0.0f);
             if (took)
             {
               thermal_run_winding(&b->record.cfg, a, c);
             }
             break;
  }
  if (took)
  {
    lay(b);
  }
  return took;
}

void world_stand_identify(int i)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    thermal_run_identify(&b->run);
    thermal_run_envelope(&b->run, b->clock_ms);
  }
}

void world_stand_every(int i, uint32_t every_ms)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    b->every_ms = every_ms;
  }
}

void world_stand_wep(int i, uint32_t ms)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    b->wep_until_ms = b->clock_ms + ms;
  }
}

void world_stand_airspeed(int i, float told, uint32_t hold_ms, float truth)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    b->told_airspeed = told;
    b->told_until_ms = b->clock_ms + hold_ms;
    b->truth_airspeed = truth;
  }
}

void world_stand_cycle(int i, float amps, float on_s, float off_s)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    b->cycle[0] = amps;
    b->cycle[1] = on_s;
    b->cycle[2] = off_s;
    b->cycle[3] = (float)b->clock_ms / 1000.0f;
    b->cycle_trip = -1;
  }
}

/** Whether the load cycle is on now, and which cycle it is: idle off, and through a cycle the
    budget tripped in - which it marks. */
static bool cycle_on(world_stand_t *b, int32_t *index)
{
  const float since = ((float)b->clock_ms / 1000.0f) - b->cycle[3];
  const float whole = b->cycle[1] + b->cycle[2];

  *index = (int32_t)floorf(since / whole);
  bool on = ((since - ((float)*index * whole)) < b->cycle[1]) && (*index != b->cycle_trip);

  if (on && b->run.budget.tripped)
  {
    b->cycle_trip = *index;
    on = false;
  }
  return on;
}

/** The load a slice sees: the row's - its bridge off once the stage is dropped - or the cycle's
    phase, its amps derated, idle off and through a cycle the budget tripped in. */
static void load_of(world_stand_t *b, const float *in, float dt, bool stage, thermal_load_t *load)
{
  float amps[3] = { in[5], in[6], in[7] };
  bool switching = (in[1] != 0.0f) && stage;

  memset(load, 0, sizeof(*load));
  load->link_amps = in[9];
  for (int k = 0; k < 3; k++)
  {
    load->duty[k] = in[2 + k];
  }
  if (b->cycle[0] > 0.0f)
  {
    int32_t index = 0;
    const bool on = cycle_on(b, &index);

    /* Three legs' rms at half duty are no link's current: summed as a held vector's, 45 A
       went through the hot swap, 7.3 W of it (2026-10-05). */
    for (int k = 0; k < 3; k++)
    {
      amps[k] = on ? (b->cycle[0] * b->run.derate) : 0.0f;
      load->duty[k] = on ? 0.5f : load->duty[k];
    }
    switching = on;
    load->link_amps = on ? 0.0f : load->link_amps;
  }
  const float follow = (dt / WORLD_STAND_RMS_TAU_S < 1.0f) ? (dt / WORLD_STAND_RMS_TAU_S) : 1.0f;

  for (int k = 0; k < 3; k++)
  {
    const float sq = switching ? (amps[k] * amps[k]) : 0.0f;

    b->squares[k] += (sq - b->squares[k]) * follow;
    load->phase_amps[k] = amps[k];
    load->phase_sq[k] = b->squares[k];
  }
  load->switching = switching;
  load->afe_on = in[0] != 0.0f;
  load->link_volts = in[8];
  load->speed_rpm = in[10];
  load->t_dead_s = in[11];
}

/** The truth a slice on, and its thermometers sampled where one is due: noisy, the MCU's die
    not read while the drive holds the meter. */
static thermal_sense_t truth_slice(world_stand_t *b, const thermal_load_t *load, float dt,
                                   bool locked)
{
  thermal_load_t flown = *load;
  thermal_sense_t sample = { NAN, NAN, NAN };

  flown.airspeed_m_s = b->truth_airspeed;
  world_heat_step(&b->truth, &flown, dt, &b->now);
  if ((b->every_ms != 0U) && ((b->clock_ms - b->sampled_ms) >= b->every_ms))
  {
    b->sampled_ms = b->clock_ms;
    sample.ntc_c = noisy(b, b->now.ntc_c);
    sample.mcu_c = noisy(b, b->now.mcu_c);
    sample.afe_c = noisy(b, b->now.afe_c);
    sample.mcu_c = locked ? NAN : sample.mcu_c;
    b->seen = sample;
    b->sampled = true;
  }
  return sample;
}

void world_stand_run(int i, const float *in, float seconds, float *out)
{
  world_stand_t *b = at(i);
  uint32_t left = (uint32_t)lrintf(seconds * 1000.0f);
  bool stage = true;
  bool judged = false;
  float applied = 1.0f;

  if (b == NULL)
  {
    return;
  }
  /* No time at all is one slice of none: the envelope judged where the observer stands. */
  while ((left > 0U) || !judged)
  {
    const uint32_t slice = (left > WORLD_STAND_SLICE_MS) ? WORLD_STAND_SLICE_MS : left;

    judged = true;
    const float dt = (float)slice / 1000.0f;
    thermal_run_in_t step;
    thermal_run_out_t acted;

    left -= slice;
    load_of(b, in, dt, stage, &step.load);
    b->speed_rpm = step.load.speed_rpm;
    b->clock_ms += slice;
    step.seen = truth_slice(b, &step.load, dt, in[12] != 0.0f);
    step.load.airspeed_m_s = ((int32_t)(b->told_until_ms - b->clock_ms) > 0)
                             ? b->told_airspeed : 0.0f;
    step.slice_ms = slice;
    step.clock_ms = b->clock_ms;
    /* Not acting, the envelope holds still: no derate slewed, no stage dropped. */
    step.wep = ((int32_t)(b->wep_until_ms - b->clock_ms) > 0) || (in[13] == 0.0f);
    step.switching = stage && (in[1] != 0.0f) && (in[13] != 0.0f);
    thermal_run_slice(&b->run, &step, &acted, NULL);
    applied = acted.derate;
    stage = stage && !acted.drop;
    b->steps++;
  }
  thermal_run_follow(&b->run, b->clock_ms);
  out[0] = applied;
  out[1] = stage ? 0.0f : 1.0f;
  out[2] = (float)b->steps;
}

void world_stand_phase_r(int i, float ohm)
{
  world_stand_t *b = at(i);

  if (b != NULL)
  {
    b->run.loss.r_phase = ohm;
    b->truth.loss.r_phase = ohm;
  }
}

void world_stand_place(int i, int node, float celsius)
{
  world_stand_t *b = at(i);

  if ((b != NULL) && (node >= 0) && (node < (int)THERMAL_NODES))
  {
    b->run.th.t[node] = celsius;
  }
}

void world_stand_settle(int i, const float *in, float seconds)
{
  world_stand_t *b = at(i);
  const thermal_sense_t none = { NAN, NAN, NAN };
  uint32_t left = (uint32_t)lrintf(seconds * 1000.0f);

  if (b == NULL)
  {
    return;
  }
  while (left > 0U)
  {
    const uint32_t slice = (left > WORLD_STAND_SLICE_MS) ? WORLD_STAND_SLICE_MS : left;
    const float dt = (float)slice / 1000.0f;
    const float phase_c[3] = { b->run.th.t[THERMAL_DRIVER(0)], b->run.th.t[THERMAL_DRIVER(1)],
                               b->run.th.t[THERMAL_DRIVER(2)] };
    thermal_load_t load;

    left -= slice;
    load_of(b, in, dt, true, &load);
    world_heat_step(&b->truth, &load, dt, &b->now);
    b->run.th.ambient = b->truth.th.ambient;
    thermal_power_estimate(&b->run.power, &load, &b->run.loss, phase_c);
    thermal_step(&b->run.th, &b->run.power, &none, &load, dt);
  }
  /* An hour of idling has told the identification the room as well. */
  b->run.ident.scale[THERMAL_IDENT_AMBIENT] = b->truth.th.ambient;
}

/* ---- the views -------------------------------------------------------------- */

static int put(float *out, int n, float v)
{
  out[n] = v;
  return n + 1;
}

static int view_state(const world_stand_t *b, float *out)
{
  int n = 0;

  for (int k = 0; k < (int)THERMAL_NODES; k++)
  {
    n = put(out, n, b->run.th.t[k]);
  }
  n = put(out, n, b->run.th.ambient);
  n = put(out, n, thermal_expected_ntc(&b->run.th));
  n = put(out, n, b->run.th.settled ? 1.0f : 0.0f);
  n = put(out, n, b->seen.ntc_c);
  n = put(out, n, b->seen.afe_c);
  n = put(out, n, b->seen.mcu_c);
  n = put(out, n, b->sampled ? 1.0f : 0.0f);
  n = put(out, n, (float)(b->clock_ms - b->sampled_ms));
  n = put(out, n, (float)b->clock_ms);
  n = put(out, n, (float)b->steps);
  n = put(out, n, b->speed_rpm);
  n = put(out, n, ((int32_t)(b->told_until_ms - b->clock_ms) > 0) ? b->told_airspeed : 0.0f);
  n = put(out, n, b->truth_airspeed);
  for (int leg = 0; leg < 3; leg++)
  {
    n = put(out, n, thermal_junction(&b->run.th, &b->run.power, THERMAL_DRIVER(leg))
                    - b->run.th.t[THERMAL_DRIVER(leg)]);
  }
  return n;
}

static int view_budget(const world_stand_t *b, float *out)
{
  const thermal_budget_t *g = &b->run.budget;
  int n = 0;

  for (int k = 0; k < (int)THERMAL_NODES; k++)
  {
    n = put(out, n, (float)g->used[k]);
  }
  n = put(out, n, (float)g->worst);
  n = put(out, n, (float)g->worst_node);
  n = put(out, n, (float)g->millis_to_limit);
  n = put(out, n, g->throttling ? 1.0f : 0.0f);
  n = put(out, n, g->tripped ? 1.0f : 0.0f);
  n = put(out, n, b->run.derate);
  n = put(out, n, g->derate);
  for (int k = 0; k < (int)THERMAL_NODES; k++)
  {
    n = put(out, n, g->soak_j[k]);
  }
  n = put(out, n, b->run.th.t[THERMAL_WINDING]);
  n = put(out, n, (float)g->used[THERMAL_WINDING]);
  n = put(out, n, b->run.winding_derate);
  return put(out, n, (float)b->run.trips);
}

static int view_ident(const world_stand_t *b, float *out)
{
  const thermal_ident_t *id = &b->run.ident;
  int n = put(out, 0, (float)id->state);

  for (int k = 0; k < THERMAL_IDENT_RECORD; k++)
  {
    n = put(out, n, id->scale[k]);
    n = put(out, n, thermal_ident_sigma(id, (thermal_ident_param_t)k));
    n = put(out, n, thermal_ident_online((thermal_ident_param_t)k) ? 1.0f : 0.0f);
  }
  n = put(out, n, id->innovation_k);
  n = put(out, n, b->run.margin);
  n = put(out, n, (float)id->updates);
  n = put(out, n, thermal_ident_ambient(id));
  n = put(out, n, thermal_ident_sigma(id, THERMAL_IDENT_AMBIENT));
  n = put(out, n, b->run.margin_floor);
  n = put(out, n, thermal_run_trip_cap(&b->run, b->clock_ms));
  return put(out, n, thermal_ident_margin(id, b->run.margin_floor));
}

static int view_network(const world_stand_t *b, float *out)
{
  const thermal_cfg_t *cfg = &b->run.th.cfg;
  int n = 0;

  for (int k = 0; k < (int)THERMAL_NODES; k++)
  {
    n = put(out, n, cfg->node[k].capacity);
    n = put(out, n, cfg->node[k].to_ambient);
    n = put(out, n, cfg->node[k].area_share);
    n = put(out, n, cfg->node[k].rth_die);
    n = put(out, n, cfg->node[k].forced);
  }
  for (int e = 0; e < (int)THERMAL_EDGES; e++)
  {
    n = put(out, n, (float)thermal_edge(e).a);
    n = put(out, n, (float)thermal_edge(e).b);
    n = put(out, n, cfg->r_edge[e]);
  }
  n = put(out, n, cfg->board_to_ambient);
  n = put(out, n, cfg->board_rad_share);
  n = put(out, n, cfg->ntc_sees);
  n = put(out, n, cfg->ntc_tau_s);
  return put(out, n, cfg->rad_board_stator);
}

static int view_truth(world_stand_t *b, float *out)
{
  int32_t index = 0;
  const bool on = (b->cycle[0] > 0.0f) && cycle_on(b, &index);
  int n = 0;

  for (int k = 0; k < (int)THERMAL_NODES; k++)
  {
    n = put(out, n, b->truth.th.t[k]);
  }
  n = put(out, n, b->now.ntc_c);
  n = put(out, n, b->now.mcu_c);
  n = put(out, n, b->now.afe_c);
  n = put(out, n, b->truth.th.ambient);
  n = put(out, n, on ? (b->cycle[0] * b->run.derate) : 0.0f);
  return put(out, n, on ? b->cycle[0] : 0.0f);
}

int world_stand_read(int i, int what, float *out)
{
  world_stand_t *b = at(i);

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  switch (what)
  {
    case WORLD_STAND_STATE:   return view_state(b, out);
    case WORLD_STAND_BUDGET:  return view_budget(b, out);
    case WORLD_STAND_IDENT:   return view_ident(b, out);
    case WORLD_STAND_NETWORK: return view_network(b, out);
    case WORLD_STAND_TRUTH:   return view_truth(b, out);
    default:                  return 0;
  }
}
