/** thermal_ident.c - Online identification of the graph's scales: a shadow
    run open loop, its sensitivities by finite differences, recursive least
    squares on the prediction error at every sample. */
#include "thermal_ident.h"

#include <math.h>
#include <string.h>
#ifdef THERMAL_IDENT_TRACE
#include <stdio.h>
#endif

/** The slice the shadow and its sensitivities are stepped with - the core's
    own, so the two integrate alike. */
#define IDENT_DT_SLICE 0.25f

/** Finite-difference steps: on a temperature, the largest perturbation any
    node takes in kelvin - the sensitivity vector is scaled to it, so a
    vector grown to tens of kelvin per unit is not probed tens of kelvin out
    where the air path bends; on a scale, a fraction of it. */
#define IDENT_EPS_T 0.5f
#define IDENT_EPS_S 0.02f
#define IDENT_EPS_AMB 0.5f

/** What the scales start out believed to, one sigma each: the air path and
    the spread are the ones a situation or a layout can double, the
    laminate's capacity was measured (49 J/K off a settling) and the
    thermistor's seat comes off the pick and place. */
static const float PRIOR_SIGMA[THERMAL_IDENT_PARAMS] = { 0.5f, 0.2f, 0.5f,
                                                        0.3f, 10.0f };
/* THE ROOM'S PRIOR IS TEN KELVIN, AND IT IS A WEIGHT. */

/** Below what sigma each is CONVERGING, and STABLE: three tenths of a scale,
    and a tenth - but 0.15 for the air path, since with the room identified
    beside it one transient leaves the air path known to about 0.12 (the two
    share a cooldown's evidence) and a machine that holds a steady current
    after a change has exactly one transient to learn from; at a tenth the
    rotor page never showed STABLE (bench, 2026-09-06). */
static const float SIGMA_CONVERGING[THERMAL_IDENT_PARAMS] = { 0.30f, 0.30f,
                                                             0.30f, 0.30f,
                                                             6.0f };
static const float SIGMA_STABLE[THERMAL_IDENT_PARAMS] = { 0.15f, 0.10f, 0.10f,
                                                         0.10f, 3.0f };

/** The process noise a sample: a random walk of half a percent on a scale -
    a hundred samples without excitation grow a sigma by five percent, no
    more - and 22 mK on the room, a couple of kelvin an hour at thirty-second
    samples, which is what a room does. */
static const float DRIFT_VAR[THERMAL_IDENT_PARAMS] = { 2.5e-5f, 2.5e-5f,
                                                      2.5e-5f, 2.5e-5f,
                                                      5.0e-4f };

/** How much of its prior each scale's sigma is floored at while the model is
    UNCERTAIN - kept free to move, since it is not predicting. */
static const float FLOOR_SHARE[THERMAL_IDENT_PARAMS] = { 0.5f, 0.25f, 0.25f,
                                                        0.25f, 0.5f };

/** Which scales the samples are allowed to move. */
static const bool ONLINE[THERMAL_IDENT_PARAMS] = { true, true, false, false,
                                                  true };

bool thermal_ident_online(thermal_ident_param_t which)
{
  return (which < THERMAL_IDENT_PARAMS) ? ONLINE[which] : false;
}

/** What an innovation is worth against the scales: the measurement noise the
    Kalman step divides by, as a multiple of the thermometers' floor. */
#define IDENT_NOISE_GAIN 3.0f

/** Below this much sensitivity squared a sample carries nothing about the
    scales and is not fed to the filter. */
#define IDENT_EXCITATION_MIN 1.0e-4f

/** The covariance's ceiling on any diagonal: a scale is never less known
    than plus or minus its own size, the room than its prior. */
static const float VAR_MAX[THERMAL_IDENT_PARAMS] = { 1.0f, 1.0f, 1.0f, 1.0f,
                                                    100.0f };

/** The innovation filter: a fifth of the way to each new sample. */
#define IDENT_INNOVATION_FOLLOW 0.2f

/** Samples passed unjudged after the one that ends a blind gap. */
#define IDENT_SETTLE_SAMPLES 2U

/** An innovation beyond this many sigmas of what the scales' own uncertainty
    and the noise predict is not believed to that size: the measurement noise
    is inflated until it is exactly this far out. */
#define IDENT_GATE_SIGMAS 3.0f

/** How far a thermometer must have moved since the seat, as a multiple of
    the noise floor, for the sample to say anything about the scales. */
#define IDENT_STILL_GAIN 3.0f

/** THE ERROR IS JUDGED AGAINST WHAT THE THERMOMETER DID. */
#define IDENT_MOVE_SHARE 0.05f

/** What the states are: the innovation against the noise floor that says the
    model predicts (the sigmas are SIGMA_CONVERGING and SIGMA_STABLE above,
    one each). */
#define IDENT_RATIO_STABLE     2.0f
#define IDENT_RATIO_UNCERTAIN  3.0f
#define IDENT_STABLE_RUNS      5U

/** Scratch for the finite differences: the observer is single-threaded and a
    thermal_t is too big to put on the main loop's stack nine times a slice. */
static thermal_t s_probe;

static void clamp_scales(thermal_ident_t *id)
{
  for (int k = 0; k < THERMAL_IDENT_RECORD; k++)
  {
    if (id->scale[k] < THERMAL_IDENT_SCALE_MIN)
    {
      id->scale[k] = THERMAL_IDENT_SCALE_MIN;
    }
    if (id->scale[k] > THERMAL_IDENT_SCALE_MAX)
    {
      id->scale[k] = THERMAL_IDENT_SCALE_MAX;
    }
  }
  /* The room is degrees, not a scale, and its own range. */
  float *room = &id->scale[THERMAL_IDENT_AMBIENT];

  if (*room < THERMAL_IDENT_AMBIENT_MIN_C)
  {
    *room = THERMAL_IDENT_AMBIENT_MIN_C;
  }
  if (*room > THERMAL_IDENT_AMBIENT_MAX_C)
  {
    *room = THERMAL_IDENT_AMBIENT_MAX_C;
  }
}

static void set_covariance(thermal_ident_t *id, float sigma)
{
  memset(id->p, 0, sizeof(id->p));
  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    /* The prior's own where a blanket sigma is wider than it; the room is as
       wide as its prior however the scales start. */
    const float s = ((sigma < PRIOR_SIGMA[k]) && (k != THERMAL_IDENT_AMBIENT))
                    ? sigma : PRIOR_SIGMA[k];

    id->p[k][k] = s * s;
  }
}

void thermal_ident_init(thermal_ident_t *id, float ambient_c, float noise_k)
{
  if (id == NULL)
  {
    return;
  }
  memset(id, 0, sizeof(*id));
  for (int k = 0; k < THERMAL_IDENT_RECORD; k++)
  {
    id->scale[k] = 1.0f;
  }
  id->scale[THERMAL_IDENT_AMBIENT] = ambient_c;
  clamp_scales(id);
  set_covariance(id, 0.5f);
  id->noise_k = (noise_k > 0.0f) ? noise_k : 0.1f;
  id->innovation_k = id->noise_k;
  id->state = THERMAL_IDENT_UNCERTAIN;
}

float thermal_ident_ambient(const thermal_ident_t *id)
{
  return (id != NULL) ? id->scale[THERMAL_IDENT_AMBIENT] : NAN;
}

/** `out` = `base` with `scale` applied - the same rule for the observer's
    configuration and for the probes the sensitivities are taken with. */
static void apply(const float *scale, const thermal_cfg_t *base,
                  thermal_cfg_t *out)
{
  *out = *base;
  out->board_to_ambient = base->board_to_ambient * scale[THERMAL_IDENT_AIR];
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    thermal_node_cfg_t *n = &out->node[i];

    if (n->area_share > 0.0f)
    {
      n->to_ambient = base->node[i].to_ambient * scale[THERMAL_IDENT_AIR];
      n->capacity = base->node[i].capacity * scale[THERMAL_IDENT_CAPACITY];
    }
  }
  /* The sources' edges into their patches: the first ten of the table. */
  for (int e = 0; e < 10; e++)
  {
    out->r_edge[e] = base->r_edge[e] * scale[THERMAL_IDENT_SPREAD];
  }
  out->ntc_sees = base->ntc_sees * scale[THERMAL_IDENT_NTC];
}

void thermal_ident_apply(const thermal_ident_t *id, const thermal_cfg_t *base,
                         thermal_cfg_t *out)
{
  if ((id == NULL) || (base == NULL) || (out == NULL))
  {
    return;
  }
  apply(id->scale, base, out);
}

/** The rate of every node, K/s, at a state and configuration: the net flows
    over the capacities. */
static void rates(const thermal_t *th, const thermal_power_t *p,
                  float speed_rpm, float *out)
{
  float net[THERMAL_NODES];

  thermal_net_flows(th, p, speed_rpm, net);
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    const float c = th->cfg.node[i].capacity;

    out[i] = (c > 0.0f) ? (net[i] / c) : 0.0f;
  }
}

/** One slice of the shadow and its sensitivities. */
static void propagate(thermal_ident_t *id, const thermal_cfg_t *base,
                      const thermal_power_t *p, float speed_rpm, float dt_s)
{
  float f0[THERMAL_NODES];
  float f1[THERMAL_NODES];

  rates(&id->shadow, p, speed_rpm, f0);

  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    float ds[THERMAL_NODES];

    /* J S_k, the perturbation scaled to IDENT_EPS_T at the largest component
       so the probe stays where the flows are linear. */
    float largest = 0.0f;

    for (int i = 0; i < THERMAL_NODES; i++)
    {
      largest = fmaxf(largest, fabsf(id->s[k][i]));
    }
    const float eps = (largest > 1.0e-6f) ? (IDENT_EPS_T / largest) : 1.0f;

    s_probe = id->shadow;
    for (int i = 0; i < THERMAL_NODES; i++)
    {
      s_probe.t[i] += eps * id->s[k][i];
    }
    rates(&s_probe, p, speed_rpm, f1);
    for (int i = 0; i < THERMAL_NODES; i++)
    {
      ds[i] = (f1[i] - f0[i]) / eps;
    }
    /* df/ds_k: the room nudged half a kelvin, a scale by a fraction. */
    if (k == THERMAL_IDENT_AMBIENT)
    {
      s_probe = id->shadow;
      s_probe.ambient += IDENT_EPS_AMB;
      rates(&s_probe, p, speed_rpm, f1);
      for (int i = 0; i < THERMAL_NODES; i++)
      {
        ds[i] += (f1[i] - f0[i]) / IDENT_EPS_AMB;
      }
    }
    else
    {
      float nudged[THERMAL_IDENT_PARAMS];

      memcpy(nudged, id->scale, sizeof(nudged));
      nudged[k] *= (1.0f + IDENT_EPS_S);
      s_probe = id->shadow;
      apply(nudged, base, &s_probe.cfg);
      rates(&s_probe, p, speed_rpm, f1);
      for (int i = 0; i < THERMAL_NODES; i++)
      {
        ds[i] += (f1[i] - f0[i]) / (IDENT_EPS_S * id->scale[k]);
      }
    }
    for (int i = 0; i < THERMAL_NODES; i++)
    {
      id->s[k][i] += ds[i] * dt_s;
    }
  }

  /* The shadow itself, then its thermistor and the thermistor's
     sensitivities: the element follows the weighted average at the
     laminate's lag, and for the NTC scale the target itself moves. */
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    id->shadow.t[i] += f0[i] * dt_s;
  }
  {
    thermal_t *sh = &id->shadow;
    const float tau = sh->cfg.ntc_tau_s;
    float share = (tau > 0.0f) ? (dt_s / tau) : 1.0f;
    float f = sh->cfg.ntc_sees;

    if (share > 1.0f)
    {
      share = 1.0f;
    }
    if (f < 0.0f)
    {
      f = 0.0f;
    }
    if (f > 1.0f)
    {
      f = 1.0f;
    }
    /* The observer's own rule for the element, bound included: held at a
       patch, the element reads that patch and so does its sensitivity. */
    const int held = thermal_ntc_follow(sh, dt_s);

    for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
    {
      if (held >= 0)
      {
        id->s_ntc[k] = id->s[k][held];
        continue;
      }
      float target = (1.0f - f) * id->s[k][THERMAL_BOARD]
                     + f * id->s[k][THERMAL_NTC_PATCH];

      if ((k == THERMAL_IDENT_NTC) && (id->scale[k] > 0.0f))
      {
        target += (f / id->scale[k])
                  * (sh->t[THERMAL_NTC_PATCH] - sh->t[THERMAL_BOARD]);
      }
      id->s_ntc[k] += (target - id->s_ntc[k]) * share;
    }
  }
}

/** Seat the shadow on the observer and forget the sensitivities: the next
    innovation is a prediction error from here. */
static void reseat(thermal_ident_t *id, const thermal_t *th,
                   const thermal_cfg_t *base, const thermal_power_t *p,
                   float speed_rpm, const thermal_sense_t *seen)
{
  float f0[THERMAL_NODES];

  id->shadow = *th;
  id->shadow.ambient = id->scale[THERMAL_IDENT_AMBIENT];   /* the room as identified */
  apply(id->scale, base, &id->shadow.cfg);
  rates(&id->shadow, p, speed_rpm, f0);   /* the nodes' own K/s here */
  memset(id->s, 0, sizeof(id->s));
  memset(id->s_ntc, 0, sizeof(id->s_ntc));
  id->horizon_s = 0.0f;
  for (int j = 0; j < 3; j++)
  {
    id->seated[j] = false;
  }
  if ((seen == NULL) || (p == NULL))
  {
    return;
  }
  thermal_t *sh = &id->shadow;

  if (!isnan(seen->ntc_c))
  {
    sh->ntc = seen->ntc_c;
    id->seated[0] = true;
    id->seat_reading[0] = seen->ntc_c;
  }
  static const thermal_node_t DIES[2] = { THERMAL_MCU, THERMAL_AFE };
  const float readings[2] = { seen->mcu_c, seen->afe_c };

  for (int d = 0; d < 2; d++)
  {
    if (isnan(readings[d]))
    {
      continue;
    }
    const thermal_node_t node = DIES[d];
    const int edge = thermal_sink_edge(node);
    const float at = readings[d] - p->watt[node] * sh->cfg.node[node].rth_die;

    sh->t[node] = at;
    if (edge >= 0)
    {
      const thermal_node_t patch = (thermal_node_t)thermal_edge(edge).b;
      /* The observer's own algebra for the patch under a die, lag term
         included: node - P R + C R dT/dt (thermal.c, anchor_die). */
      const float per_r = -p->watt[node] + sh->cfg.node[node].capacity * f0[node];

      sh->t[patch] = at + per_r * sh->cfg.r_edge[edge];
      /* The seat itself depends on the spread: the patch is placed the die's
         watts through a SCALED edge below the node, so a sample judged from
         here already owes that much to the scale. */
      id->s[THERMAL_IDENT_SPREAD][patch] = per_r * base->r_edge[edge];
    }
    id->seated[1 + d] = true;
    id->seat_reading[1 + d] = readings[d];
  }
}

/** One recursive least-squares update on one thermometer's innovation. */
static bool update(thermal_ident_t *id, const float *h_all, float innovation)
{
  float excitation = 0.0f;
  float h[THERMAL_IDENT_PARAMS];

  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    /* A held scale has no regressor: the filter neither moves it nor learns
       a correlation through it. */
    h[k] = ONLINE[k] ? h_all[k] : 0.0f;
    excitation += h[k] * h[k];
  }
  if (excitation < IDENT_EXCITATION_MIN)
  {
    return false;                 /* nothing about the scales in it */
  }

  float ph[THERMAL_IDENT_PARAMS];
  const float r = IDENT_NOISE_GAIN * id->noise_k;
  float hph = 0.0f;

  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    ph[k] = 0.0f;
    for (int j = 0; j < THERMAL_IDENT_PARAMS; j++)
    {
      ph[k] += id->p[k][j] * h[j];
    }
    hph += h[k] * ph[k];
  }
  /* The gate: an innovation further out than IDENT_GATE_SIGMAS of what the
     covariance and the noise predict has the noise inflated until it sits
     exactly there. */
  float denom = hph + r * r;
  const float far = IDENT_GATE_SIGMAS * IDENT_GATE_SIGMAS * denom;

  if (innovation * innovation > far)
  {
    denom = innovation * innovation / (IDENT_GATE_SIGMAS * IDENT_GATE_SIGMAS);
  }
  /* The gain, the move, then the covariance: one Kalman measurement update
     on the scales. */
  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    const float gain = ph[k] / denom;

    id->scale[k] += gain * innovation;
  }
  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    for (int j = 0; j < THERMAL_IDENT_PARAMS; j++)
    {
      id->p[k][j] -= ph[k] * ph[j] / denom;
    }
    if (id->p[k][k] > VAR_MAX[k])
    {
      id->p[k][k] = VAR_MAX[k];
    }
  }
  clamp_scales(id);
  return true;
}

/** Whether every online quantity is known to within its own threshold - a
    scale to a fraction, the room to kelvin. */
static bool known(const thermal_ident_t *id, const float *threshold)
{
  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    if (ONLINE[k]
        && (thermal_ident_sigma(id, (thermal_ident_param_t)k) >= threshold[k]))
    {
      return false;
    }
  }
  return true;
}

/** THE ROOM IS RESET WHEN THE MODEL STOPS PREDICTING. */
static void room_reset(thermal_ident_t *id)
{
  const int a = THERMAL_IDENT_AMBIENT;

  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    id->p[a][k] = 0.0f;
    id->p[k][a] = 0.0f;
  }
  id->p[a][a] = PRIOR_SIGMA[a] * PRIOR_SIGMA[a];
}

/** The state after a sample: what the covariance and the innovation say the
    model is worth, and the inflation that lets a model that has just stopped
    predicting move fast again. */
static void judge(thermal_ident_t *id)
{
  const float ratio = id->innovation_k / id->noise_k;

  switch (id->state)
  {
    case THERMAL_IDENT_STABLE:
      if (ratio > IDENT_RATIO_UNCERTAIN)
      {
        /* The situation changed under a model that was trusted: a box, a
           fan, a heat sink, a room. */
        id->state = THERMAL_IDENT_UNCERTAIN;
        id->stable_runs = 0U;
        for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
        {
          const float floor_sigma = FLOOR_SHARE[k] * PRIOR_SIGMA[k];

          id->p[k][k] += floor_sigma * floor_sigma;
        }
        room_reset(id);
      }
      break;

    case THERMAL_IDENT_CONVERGING:
      if (ratio > IDENT_RATIO_UNCERTAIN)
      {
        id->state = THERMAL_IDENT_UNCERTAIN;
        id->stable_runs = 0U;
        room_reset(id);
      }
      else if (known(id, SIGMA_STABLE) && (ratio < IDENT_RATIO_STABLE))
      {
        if (++id->stable_runs >= IDENT_STABLE_RUNS)
        {
          id->state = THERMAL_IDENT_STABLE;
        }
      }
      else
      {
        id->stable_runs = 0U;
      }
      break;

    case THERMAL_IDENT_UNCERTAIN:
    default:
      if (known(id, SIGMA_CONVERGING) && (ratio < IDENT_RATIO_UNCERTAIN))
      {
        id->state = THERMAL_IDENT_CONVERGING;
        id->stable_runs = 0U;
      }
      else if (ratio >= IDENT_RATIO_UNCERTAIN)
      {
        /* NOT PREDICTING, SO NOT SURE: while the innovation says the model
           is wrong the scales are kept free to move, each online variance
           floored at half its prior. */
        for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
        {
          const float floor_sigma = FLOOR_SHARE[k] * PRIOR_SIGMA[k];
          const float floor_var = floor_sigma * floor_sigma;

          if (ONLINE[k] && (id->p[k][k] < floor_var))
          {
            id->p[k][k] = floor_var;
          }
        }
      }
      break;
  }
}

bool thermal_ident_step(thermal_ident_t *id, const thermal_t *th,
                        const thermal_cfg_t *base, const thermal_power_t *p,
                        const thermal_load_t *load,
                        const thermal_sense_t *seen, float dt_s)
{
  if ((id == NULL) || (th == NULL) || (base == NULL) || (p == NULL)
      || (seen == NULL) || !(dt_s > 0.0f))
  {
    return false;
  }
  const float speed = (load != NULL) ? load->speed_rpm : 0.0f;

  if (!id->primed)
  {
    reseat(id, th, base, p, speed, seen);
    id->primed = true;
    return false;
  }

  /* The shadow forward, in the core's own slices. */
  float left = dt_s;

  while (left > 0.0f)
  {
    const float slice = (left > IDENT_DT_SLICE) ? IDENT_DT_SLICE : left;

    propagate(id, base, p, speed, slice);
    left -= slice;
  }
  id->horizon_s += dt_s;
  id->since_sample_s += dt_s;

  const bool any = !isnan(seen->ntc_c) || !isnan(seen->mcu_c)
                   || !isnan(seen->afe_c);
  /* Blind is measured from the last READING, not the last seat: the shadow
     is re-seated at THERMAL_IDENT_MAX_HORIZON_S whether or not anything was
     read, and a ten-minute run ends exactly there. */
  const bool blind = any && (id->since_sample_s > THERMAL_IDENT_BLIND_S);

  if (any)
  {
    id->since_sample_s = 0.0f;
  }

  if (blind)
  {
    /* THE SAMPLE THAT ENDS A BLIND RUN SEATS THE SHADOW AND NOTHING IS
       JUDGED FROM IT: the observer has just been pulled onto the
       thermometers from a state ten minutes of the wrong scales made, and
       what it still carries in the nodes they do not reach is the state's
       error. */
    reseat(id, th, base, p, speed, NULL);
    id->settle_left = IDENT_SETTLE_SAMPLES;
    return false;
  }

  if (any && (id->settle_left > 0U)
      && (id->horizon_s >= THERMAL_IDENT_MIN_HORIZON_S))
  {
    id->settle_left--;
    reseat(id, th, base, p, speed, seen);
    return false;
  }

  if (any && (id->horizon_s >= THERMAL_IDENT_MIN_HORIZON_S))
  {
    bool moved = false;
    bool judged = false;
    float worst = 0.0f;

    /* Each thermometer against the shadow's prediction of it - judged only
       where the shadow was SEATED on that thermometer's reading, so the
       innovation is the reading's change over the interval against the
       model's, and the state's error at the seat is not in it. */
    static const thermal_node_t DIES[2] = { THERMAL_MCU, THERMAL_AFE };
    const float readings[3] = { seen->ntc_c, seen->mcu_c, seen->afe_c };
    float predicted[3];
    float h[3][THERMAL_IDENT_PARAMS];

    /* A STILL BOARD TEACHES NOTHING. */
    float stirred = 0.0f;

    for (int j = 0; j < 3; j++)
    {
      if (!isnan(readings[j]) && id->seated[j])
      {
        stirred = fmaxf(stirred, fabsf(readings[j] - id->seat_reading[j]));
      }
    }
    const bool still = stirred < IDENT_STILL_GAIN * id->noise_k;

    predicted[0] = id->shadow.ntc;
    memcpy(h[0], id->s_ntc, sizeof(h[0]));
    for (int d = 0; d < 2; d++)
    {
      const thermal_node_t node = DIES[d];

      predicted[1 + d] = id->shadow.t[node]
                         + p->watt[node] * id->shadow.cfg.node[node].rth_die;
      for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
      {
        h[1 + d][k] = id->s[k][node];
      }
    }
    for (int j = 0; j < 3; j++)
    {
      if (isnan(readings[j]) || !id->seated[j])
      {
        continue;
      }
      const float e = readings[j] - predicted[j];

      if (!still)
      {
        moved |= update(id, h[j], e);
      }
#ifdef THERMAL_IDENT_TRACE
      /* Host diagnostics only: never in the firmware build. */
      printf("TRACE %d e=%+.3f h=%+.3f %+.3f %+.3f %+.3f  s=%.2f %.2f %.2f %.2f\n",
             j, (double)e, (double)h[j][0], (double)h[j][1], (double)h[j][2],
             (double)h[j][3], (double)id->scale[0], (double)id->scale[1],
             (double)id->scale[2], (double)id->scale[3]);
      fflush(stdout);           /* the C and Python streams interleave */
#endif
      /* The judged error, in the floor's own units: the raw error over one
         plus the share of the movement, floors. */
      const float moved_j = fabsf(readings[j] - id->seat_reading[j]);
      const float allowed = id->noise_k + IDENT_MOVE_SHARE * moved_j;

      worst = fmaxf(worst, fabsf(e) * id->noise_k / allowed);
      judged = true;
    }

    if (judged)
    {
      id->innovation_k += IDENT_INNOVATION_FOLLOW
                          * (worst - id->innovation_k);
      /* The scales may drift between samples: the process noise. */
      for (int k = 0; (k < THERMAL_IDENT_PARAMS) && !still; k++)
      {
        if (ONLINE[k])
        {
          id->p[k][k] += DRIFT_VAR[k];
        }
      }
      judge(id);
    }
    if (moved)
    {
      id->updates++;
    }
    reseat(id, th, base, p, speed, seen);
    return moved;
  }

  if (id->horizon_s >= THERMAL_IDENT_MAX_HORIZON_S)
  {
    reseat(id, th, base, p, speed, NULL);   /* nothing was read at this seat */
  }
  return false;
}

float thermal_ident_sigma(const thermal_ident_t *id,
                          thermal_ident_param_t which)
{
  if ((id == NULL) || (which >= THERMAL_IDENT_PARAMS))
  {
    return NAN;
  }
  const float var = id->p[which][which];

  return (var > 0.0f) ? sqrtf(var) : 0.0f;
}

static float unit(float x)
{
  return (x < 0.0f) ? 0.0f : ((x > 1.0f) ? 1.0f : x);
}

float thermal_ident_doubt(const thermal_ident_t *id)
{
  if (id == NULL)
  {
    return 1.0f;
  }
  /* THE INNOVATION, NORMALISED: none at the thermometers' floor, all of it
     at the ratio that says UNCERTAIN - the same three floors the state is
     judged on, so the two agree about what "not predicting" means. */
  const float ratio = id->innovation_k / id->noise_k;
  float doubt = unit((ratio - 1.0f) / (IDENT_RATIO_UNCERTAIN - 1.0f));

  /* AND THE COVARIANCE, the same way: each online quantity's sigma from
     where STABLE calls it known (none) up to its prior (all). */
  for (int k = 0; k < THERMAL_IDENT_PARAMS; k++)
  {
    if (!ONLINE[k])
    {
      continue;
    }
    const float sigma = thermal_ident_sigma(id, (thermal_ident_param_t)k);
    const float span = PRIOR_SIGMA[k] - SIGMA_STABLE[k];
    const float u = unit((sigma - SIGMA_STABLE[k]) / span);

    doubt = (u > doubt) ? u : doubt;
  }
  return doubt;
}

float thermal_ident_margin(const thermal_ident_t *id, float floor)
{
  /* THE POLICY, CONTINUOUS: the spans to the ceilings multiplied by a number
     that is the floor while the model is doubted whole and one when it is
     doubted not at all, and the evidence between. */
  const float f = unit(floor);

  return f + (1.0f - f) * (1.0f - thermal_ident_doubt(id));
}
