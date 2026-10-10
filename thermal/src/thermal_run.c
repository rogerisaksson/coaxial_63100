/** thermal_run.c - The board's thermal observer a slice at a time, its envelope held. */
#include "thermal_run.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void thermal_run_init(thermal_run_t *r, const thermal_cfg_t *base, const thermal_loss_t *loss,
                      float start_c)
{
  thermal_cfg_t cfg;

  r->base = *base;
  r->loss = *loss;
  /* Fresh every boot: scales at one, the room at the start, doubted whole. */
  thermal_ident_init(&r->ident, start_c, THERMAL_IDENT_NOISE_K);
  thermal_ident_apply(&r->ident, &r->base, &cfg);
  thermal_init(&r->th, &cfg, start_c);
  memset(&r->power, 0, sizeof(r->power));
  memset(&r->budget, 0, sizeof(r->budget));
  r->margin = 1.0f;
  r->trip_cap = 1.0f;
  r->trip_ms = 0U;
  r->winding_derate = 1.0f;
  r->derate = 1.0f;
  r->trips = 0U;
}

void thermal_run_identify(thermal_run_t *r)
{
  thermal_ident_init(&r->ident, r->th.ambient, THERMAL_IDENT_NOISE_K);
  thermal_ident_apply(&r->ident, &r->base, &r->th.cfg);
}

void thermal_run_refresh(thermal_run_t *r)
{
  thermal_ident_apply(&r->ident, &r->base, &r->th.cfg);
}

void thermal_run_winding(thermal_cfg_t *cfg, float k_per_w, float j_per_k)
{
  const int into_iron = thermal_sink_edge(THERMAL_WINDING);

  cfg->node[THERMAL_WINDING].capacity = j_per_k;
  if ((k_per_w > 0.0f) && (into_iron >= 0))
  {
    cfg->r_edge[into_iron] = THERMAL_WINDING_INTO_IRON * k_per_w;
    cfg->node[THERMAL_STATOR].to_ambient = (1.0f - THERMAL_WINDING_INTO_IRON) * k_per_w;
  }
}

float thermal_run_trip_cap(const thermal_run_t *r, uint32_t clock_ms)
{
  if (r->trip_cap >= 1.0f)
  {
    return 1.0f;
  }
  const float back = (float)(clock_ms - r->trip_ms) / 1000.0f * THERMAL_TRIP_RECOVER_PER_S;
  const float cap = r->trip_cap + back;

  return (cap < 1.0f) ? cap : 1.0f;
}

/** The margin at `clock_ms`: the identification's for its doubt, or the trip cap, whichever
    keeps more in hand. */
static float margin_at(const thermal_run_t *r, uint32_t clock_ms)
{
  const float earned = thermal_ident_margin(&r->ident, r->margin_floor);
  const float cap = thermal_run_trip_cap(r, clock_ms);

  return (cap < earned) ? cap : earned;
}

void thermal_run_envelope(thermal_run_t *r, uint32_t clock_ms)
{
  const float margin = margin_at(r, clock_ms);

  r->soa = r->record;
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    /* The trip keeps the record's ceiling; the trim below is the throttle's. */
    r->soa.trip_c[i] = r->soa.limit_c[i];
    if (r->soa.limit_c[i] > THERMAL_MARGIN_REF_C)
    {
      r->soa.limit_c[i] = THERMAL_MARGIN_REF_C
                          + margin * (r->soa.limit_c[i] - THERMAL_MARGIN_REF_C);
    }
  }
  r->margin = margin;
}

void thermal_run_follow(thermal_run_t *r, uint32_t clock_ms)
{
  if (fabsf(margin_at(r, clock_ms) - r->margin) >= THERMAL_MARGIN_STEP)
  {
    thermal_run_envelope(r, clock_ms);
  }
}

/** The derate as applied: down at once, up at THERMAL_DERATE_RECOVER_PER_S. */
static float ramp(thermal_run_t *r, float want, uint32_t since_ms)
{
  float held = r->derate;

  if (want <= held)
  {
    held = want;
  }
  else
  {
    held += THERMAL_DERATE_RECOVER_PER_S * ((float)since_ms / 1000.0f);
    held = (held > want) ? want : held;
  }
  held = (held < 0.0f) ? 0.0f : ((held > 1.0f) ? 1.0f : held);
  r->derate = held;
  return held;
}

static void pump(void (*between)(void))
{
  if (between != NULL)
  {
    between();
  }
}

void thermal_run_slice(thermal_run_t *r, const thermal_run_in_t *in, thermal_run_out_t *out,
                       void (*between)(void))
{
  const float dt = (float)in->slice_ms / 1000.0f;
  /* The FET tempco feeds on the observer's own last estimate: the driver node a leg heats is
     the junction its on-resistance follows. */
  const float phase_c[3] = { r->th.t[THERMAL_DRIVER(0)], r->th.t[THERMAL_DRIVER(1)],
                             r->th.t[THERMAL_DRIVER(2)] };

  thermal_power_estimate(&r->power, &in->load, &r->loss, phase_c);
  thermal_step(&r->th, &r->power, &in->seen, &in->load, dt);
  pump(between);
  /* The identification beside it, on the same power and slice; a sample that moves the
     scales re-applies them at once. */
  if (thermal_ident_step(&r->ident, &r->th, &r->base, &r->power, &in->load, &in->seen, dt))
  {
    thermal_ident_apply(&r->ident, &r->base, &r->th.cfg);
  }
  pump(between);
  /* The room is the identification's: no sensor reads it. */
  r->th.ambient = thermal_ident_ambient(&r->ident);
  thermal_budget(&r->th, &r->power, &r->soa, &r->budget);
  pump(between);
  /* The winding's own factor, so a host can say which envelope holds the stage back; the
     whole's already includes it. */
  r->winding_derate = thermal_node_derate(&r->th, &r->power, &r->soa, THERMAL_WINDING);

  out->derate = in->wep ? 1.0f : ramp(r, r->budget.derate, in->slice_ms);
  out->drop = r->budget.tripped && in->switching;
  if (out->drop)
  {
    /* The envelope shrinks to the trip cap, recovering from now. */
    r->trips++;
    r->trip_cap = THERMAL_TRIP_MARGIN;
    r->trip_ms = in->clock_ms;
    thermal_run_envelope(r, in->clock_ms);
  }
}
