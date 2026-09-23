/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   The thermal observer: integrate the network, then correct it with
  *          whichever thermometers answered.
  ******************************************************************************
  */
#include "thermal.h"

#include <math.h>
#include <string.h>

/** Longest step the integration is allowed to take, seconds. */
#define THERMAL_DT_MAX 2.0f

/** The longest slice one Euler step takes, seconds. */
#define THERMAL_DT_SLICE 0.25f

/** How hard the sensors pull the model per second, 1/s. */
#define THERMAL_ANCHOR_HZ 0.05f

/** The longest gap since the previous sample over which a thermistor miss is
    still read as the V patch's: three of the board's thirty second
    intervals. */
#define THERMAL_NTC_INVERT_MAX_S 90.0f

/** How close to the V patch the element, read or modelled, counts as held at
    it, kelvin. */
#define THERMAL_NTC_AT_LEG_K 2.0f

/** Named edges the glue and the defaults reach for. */
#define EDGE_WINDING_STATOR 22
#define EDGE_STATOR_ROTOR   23
#define EDGE_MOUNT_FIRST    24
#define EDGE_MOUNTS         6

/** The bulk figures every laminate default is shared out of: the passive
    state's 1.2 W over 10 K, and the 6.8 minute constant it settled with. */
#define BULK_TO_AMBIENT 8.33f
#define BULK_CAPACITY   49.0f

static const thermal_edge_t THERMAL_EDGE_ENDS[THERMAL_EDGES] =
{
  /* 0..9  each source into the laminate under it */
  { THERMAL_DRIVER_U,   THERMAL_PATCH_U },
  { THERMAL_DRIVER_V,   THERMAL_PATCH_V },
  { THERMAL_DRIVER_W,   THERMAL_PATCH_W },
  { THERMAL_PHASE_U,    THERMAL_PATCH_U },
  { THERMAL_PHASE_V,    THERMAL_PATCH_V },
  { THERMAL_PHASE_W,    THERMAL_PATCH_W },
  { THERMAL_MCU,        THERMAL_BOARD },
  { THERMAL_REGULATORS, THERMAL_PATCH_LEFT },
  { THERMAL_AFE,        THERMAL_PATCH_BOTTOM },
  { THERMAL_HOTSWAP,    THERMAL_PATCH_RIGHT },
  /* 10..21 the laminate's own graph: every pair of patches that share a
     boundary, in-plane */
  { THERMAL_PATCH_U,     THERMAL_PATCH_V },
  { THERMAL_PATCH_V,     THERMAL_PATCH_W },
  { THERMAL_PATCH_U,     THERMAL_PATCH_LEFT },
  { THERMAL_PATCH_W,     THERMAL_PATCH_RIGHT },
  { THERMAL_PATCH_LEFT,  THERMAL_BOARD },
  { THERMAL_BOARD,       THERMAL_PATCH_RIGHT },
  { THERMAL_PATCH_LEFT,  THERMAL_PATCH_BOTTOM },
  { THERMAL_BOARD,       THERMAL_PATCH_BOTTOM },
  { THERMAL_PATCH_RIGHT, THERMAL_PATCH_BOTTOM },
  { THERMAL_PATCH_V,     THERMAL_BOARD },
  { THERMAL_PATCH_U,     THERMAL_BOARD },
  { THERMAL_PATCH_W,     THERMAL_BOARD },
  /* 22..23  the motor */
  { THERMAL_WINDING,     THERMAL_STATOR },
  { THERMAL_STATOR,      THERMAL_ROTOR },
  /* 24..29 the mount: the stator's back into the rim patches through the
     standoffs, open on a bench */
  { THERMAL_STATOR,      THERMAL_PATCH_U },
  { THERMAL_STATOR,      THERMAL_PATCH_V },
  { THERMAL_STATOR,      THERMAL_PATCH_W },
  { THERMAL_STATOR,      THERMAL_PATCH_LEFT },
  { THERMAL_STATOR,      THERMAL_PATCH_BOTTOM },
  { THERMAL_STATOR,      THERMAL_PATCH_RIGHT },
};


thermal_edge_t thermal_edge(int e)
{
  if ((e < 0) || (e >= THERMAL_EDGES))
  {
    const thermal_edge_t none = { (uint8_t)THERMAL_NODES,
                                  (uint8_t)THERMAL_NODES };

    return none;
  }
  return THERMAL_EDGE_ENDS[e];
}


int thermal_sink_edge(thermal_node_t node)
{
  switch (node)
  {
    case THERMAL_DRIVER_U:   return 0;
    case THERMAL_DRIVER_V:   return 1;
    case THERMAL_DRIVER_W:   return 2;
    case THERMAL_PHASE_U:    return 3;
    case THERMAL_PHASE_V:    return 4;
    case THERMAL_PHASE_W:    return 5;
    case THERMAL_MCU:        return 6;
    case THERMAL_REGULATORS: return 7;
    case THERMAL_AFE:        return 8;
    case THERMAL_HOTSWAP:    return 9;
    case THERMAL_WINDING:    return EDGE_WINDING_STATOR;
    case THERMAL_STATOR:     return EDGE_STATOR_ROTOR;
    default:                 return -1;     /* the laminate and the rotor: air */
  }
}


void thermal_defaults(thermal_cfg_t *cfg)
{
  if (cfg == NULL)
  {
    return;
  }
  memset(cfg, 0, sizeof(*cfg));

  /* CALIBRATED AGAINST A THERMAL CAMERA 2026-08-28, four states against a
     dead patch of soldermask (emissivity ~0.95, room 20 C). */
  cfg->board_to_ambient = BULK_TO_AMBIENT;
  cfg->board_cal_rise_k = 10.0f;   /* the passive state: 1.2 W, +10 K */
  cfg->board_rad_share  = 0.35f;   /* docs/papers: 30-40 % at passive */

  /* THE LAMINATE AS SEVEN PATCHES. */
  static const struct { thermal_node_t node; float share; } PATCHES[] =
  {
    { THERMAL_BOARD,        0.199f }, { THERMAL_PATCH_U,      0.109f },
    { THERMAL_PATCH_V,      0.134f }, { THERMAL_PATCH_W,      0.109f },
    { THERMAL_PATCH_LEFT,   0.126f }, { THERMAL_PATCH_BOTTOM, 0.197f },
    { THERMAL_PATCH_RIGHT,  0.126f },
  };
  for (unsigned i = 0U; i < sizeof(PATCHES) / sizeof(PATCHES[0]); i++)
  {
    thermal_node_cfg_t *n = &cfg->node[PATCHES[i].node];

    n->area_share = PATCHES[i].share;
    n->capacity   = BULK_CAPACITY * PATCHES[i].share;
    n->to_ambient = BULK_TO_AMBIENT / PATCHES[i].share;
    /* The rotor's air reaches the board's face behind the stator: an
       ESTIMATE of a third of the improvement the stator itself gets, and
       nothing on a bench, where the speed is zero. */
    n->forced     = 0.3f;
  }

  /* IN-PLANE CONDUCTANCE BETWEEN PATCHES: `G = k_sheet * L / d`, the shared
     boundary over the centre distance, with ONE sheet conductance for the
     whole laminate. */
  static const float PATCH_R[12] =
  {
    39.0f,  /* U-V */         39.0f,  /* V-W */
    62.0f,  /* U-left */      62.0f,  /* W-right */
    48.0f,  /* left-centre */ 48.0f,  /* centre-right */
    109.0f, /* left-bottom */ 32.0f,  /* centre-bottom */
    109.0f, /* right-bottom */ 67.0f, /* V-centre */
    263.0f, /* U-centre */    263.0f, /* W-centre */
  };
  for (int e = 0; e < 12; e++)
  {
    cfg->r_edge[10 + e] = PATCH_R[e];
  }

  /* THE SOURCES INTO THEIR PATCHES. */
  cfg->r_edge[0] = cfg->r_edge[1] = cfg->r_edge[2] = 12.0f;
  cfg->r_edge[3] = cfg->r_edge[4] = cfg->r_edge[5] = 8.0f;
  cfg->r_edge[6] = 22.5f;
  cfg->r_edge[7] = 15.0f;
  cfg->r_edge[8] = 41.5f;
  cfg->r_edge[9] = 12.0f;

  /* Heat capacity. */
  for (int leg = 0; leg < 3; leg++)
  {
    cfg->node[THERMAL_DRIVER(leg)].capacity = 0.35f / 3.0f;
    cfg->node[THERMAL_PHASE(leg)].capacity  = 1.20f / 3.0f;
    /* R_th,JC of the IAUCN10S7N021, datasheets/mosfet: what the datasheet's
       175 C is against. */
    cfg->node[THERMAL_DRIVER(leg)].rth_die  = 0.69f;
  }
  cfg->node[THERMAL_MCU].capacity        = 0.90f;
  cfg->node[THERMAL_REGULATORS].capacity = 0.80f;
  cfg->node[THERMAL_AFE].capacity        = 0.30f;
  cfg->node[THERMAL_HOTSWAP].capacity    = 0.50f;   /* two TDSON-8, an MSOP, a fuse */

  /* Junction over package per watt, for the two parts that report their own
     die. */
  cfg->node[THERMAL_MCU].rth_die = 40.5f;
  cfg->node[THERMAL_AFE].rth_die = 3.8f;

  /* THE MOTOR. */
  cfg->node[THERMAL_WINDING].capacity = 180.0f;
  cfg->r_edge[EDGE_WINDING_STATOR]    = 0.55f;
  cfg->node[THERMAL_STATOR].capacity  = 360.0f;
  cfg->node[THERMAL_STATOR].to_ambient = 1.65f;
  cfg->node[THERMAL_STATOR].forced    = 0.5f;
  cfg->r_edge[EDGE_STATOR_ROTOR]      = 2.0f;
  cfg->node[THERMAL_ROTOR].capacity   = 180.0f;
  cfg->node[THERMAL_ROTOR].to_ambient = 4.0f;
  cfg->node[THERMAL_ROTOR].forced     = 1.0f;
  /* The mount and the faces: OPEN on the bench, where the board lies in
     still air and nothing faces it. */
  for (int m = 0; m < EDGE_MOUNTS; m++)
  {
    cfg->r_edge[EDGE_MOUNT_FIRST + m] = 0.0f;
  }
  cfg->rad_board_stator = 0.0f;

  /* THE THERMISTOR, in the centre patch beside the V driver. */
  cfg->ntc_sees  = 0.30f;
  cfg->ntc_tau_s = sqrtf((cfg->node[THERMAL_PATCH_V].capacity * 15.0f)
                         * (cfg->node[THERMAL_BOARD].capacity * 48.0f));
  /* RECORDED, NOT APPLIED. */
  cfg->ntc_offset = 6.00f;
}


float thermal_board_to_ambient_at(const thermal_cfg_t *cfg, float rise_k)
{
  if (cfg == NULL)
  {
    return 0.0f;
  }

  const float cal = cfg->board_cal_rise_k;

  if (!(cal > 0.0f) || !(rise_k > cal))
  {
    return cfg->board_to_ambient;
  }

  /* CONVECTION, as the fourth root of the rise. */
  const float conv = powf(rise_k / cal, 0.25f);

  /* RADIATION, exactly. */
  const float t0 = 293.15f;               /* the 20 C room the fit used */
  const float now = t0 + rise_k;
  const float was = t0 + cal;
  const float rad = ((now * now + t0 * t0) * (now + t0))
                    / ((was * was + t0 * t0) * (was + t0));

  float share = cfg->board_rad_share;

  if (share < 0.0f)
  {
    share = 0.0f;
  }
  if (share > 1.0f)
  {
    share = 1.0f;
  }
  /* The two carry in parallel, so their CONDUCTANCES add. */
  const float better = (1.0f - share) * conv + share * rad;

  return (better > 0.0f) ? (cfg->board_to_ambient / better)
                         : cfg->board_to_ambient;
}


float thermal_to_ambient_at(const thermal_cfg_t *cfg, thermal_node_t node,
                            float rise_k, float speed_rpm)
{
  if ((cfg == NULL) || (node >= THERMAL_NODES))
  {
    return 0.0f;
  }
  const thermal_node_cfg_t *n = &cfg->node[node];
  float r = n->to_ambient;

  if (!(r > 0.0f))
  {
    return 0.0f;
  }
  /* A patch carries the bulk's law - the same fourth root and the same
     bracket, at its OWN rise - scaled to its share of the face. */
  if (n->area_share > 0.0f)
  {
    const float bulk = thermal_board_to_ambient_at(cfg, rise_k);

    r = (cfg->board_to_ambient > 0.0f)
        ? (r * bulk / cfg->board_to_ambient) : r;
  }
  /* FORCED CONVECTION with the rotor's speed: `Nu ~ Re^1/2`, so the air path
     improves with the square root of the speed. */
  if ((n->forced > 0.0f) && (speed_rpm > 0.0f))
  {
    r /= (1.0f + n->forced * sqrtf(speed_rpm / 1000.0f));
  }
  return r;
}


/** The bracket radiation carries with, `(T^2 + T0^2)(T + T0)`, kelvin. */
static float rad_bracket(float a_c, float b_c)
{
  const float a = a_c + 273.15f;
  const float b = b_c + 273.15f;

  return (a * a + b * b) * (a + b);
}


/** Net watts into every node at the present temperatures: what it makes,
    plus what flows in over the edges, less what it sheds to the air. */
static void net_flows(const thermal_t *th, const thermal_power_t *p,
                      float speed_rpm, float *net);

void thermal_net_flows(const thermal_t *th, const thermal_power_t *p,
                       float speed_rpm, float *net)
{
  if ((th != NULL) && (p != NULL) && (net != NULL))
  {
    net_flows(th, p, speed_rpm, net);
  }
}


static void net_flows(const thermal_t *th, const thermal_power_t *p,
                      float speed_rpm, float *net)
{
  const thermal_cfg_t *cfg = &th->cfg;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    net[i] = p->watt[i];
  }
  for (int e = 0; e < THERMAL_EDGES; e++)
  {
    const float r = cfg->r_edge[e];

    if (r > 0.0f)
    {
      const int a = THERMAL_EDGE_ENDS[e].a;
      const int b = THERMAL_EDGE_ENDS[e].b;
      const float flow = (th->t[a] - th->t[b]) / r;

      net[a] -= flow;
      net[b] += flow;
    }
  }
  /* The faces: each patch against the stator's back, by its share, the
     bracket scaled to its value at the room the figure is quoted for. */
  if (cfg->rad_board_stator > 0.0f)
  {
    const float at_room = rad_bracket(26.85f, 26.85f);     /* 300 K */

    for (int i = 0; i < THERMAL_NODES; i++)
    {
      const float share = cfg->node[i].area_share;

      if (share > 0.0f)
      {
        const float g = cfg->rad_board_stator * share
                        * rad_bracket(th->t[i], th->t[THERMAL_STATOR])
                        / at_room;
        const float flow = g * (th->t[i] - th->t[THERMAL_STATOR]);

        net[i] -= flow;
        net[THERMAL_STATOR] += flow;
      }
    }
  }
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    const float rise = th->t[i] - th->ambient;
    const float away = thermal_to_ambient_at(cfg, (thermal_node_t)i, rise,
                                             speed_rpm);

    if (away > 0.0f)
    {
      net[i] -= rise / away;
    }
  }
}


/** How long this node can stay at this power before its ceiling, seconds:
    the soak divided by what is going into it. */
static float hold_seconds(const thermal_t *th, const float *net,
                          const thermal_soa_t *soa, thermal_node_t node)
{
  const float gain = net[node];
  const float capacity = th->cfg.node[node].capacity;
  const float togo = soa->limit_c[node] - th->t[node];

  if (!(gain > 0.0f) || !(capacity > 0.0f))
  {
    return -1.0f;                      /* not heading anywhere warmer */
  }
  if (togo <= 0.0f)
  {
    return 0.0f;                       /* already there */
  }
  return togo * capacity / gain;
}


/** The clamp's factor for a spend: one below the throttle point, falling to
    zero at the ceiling, linear between. */
static float derate_of(float spent, const thermal_soa_t *soa)
{
  const float band = 1.0f - soa->throttle_at;

  if ((spent <= soa->throttle_at) || !(band > 0.0f))
  {
    return 1.0f;
  }
  const float over = (spent - soa->throttle_at) / band;

  return (over >= 1.0f) ? 0.0f : (1.0f - over);
}


/** A node's spend: where it is between ambient and its ceiling, and how far
    into the reaction window its hold has come - the bigger. */
static float spend_of(const thermal_t *th, const float *net,
                      const thermal_soa_t *soa, thermal_node_t node)
{
  const float span = soa->limit_c[node] - th->ambient;

  if (!(span > 0.0f))
  {
    return -1.0f;
  }
  float part = (th->t[node] - th->ambient) / span;

  if (part < 0.0f)
  {
    part = 0.0f;
  }
  if (part > 1.0f)
  {
    part = 1.0f;
  }
  /* AS THE WIRE SAYS IT: `used` is a byte, and the ramp acts on the same
     number a host reads, so the clamp a board applied and the spend it
     reported cannot disagree by a byte's worth of ramp. */
  part = (float)((uint8_t)(part * 255.0f)) / 255.0f;
  if (soa->lookahead_s > 0.0f)
  {
    const float hold = hold_seconds(th, net, soa, node);

    if (hold >= 0.0f)
    {
      const float pressed = 1.0f - (hold / soa->lookahead_s);

      if (pressed > part)
      {
        part = (pressed > 1.0f) ? 1.0f : pressed;
      }
    }
  }
  return part;
}


void thermal_budget(const thermal_t *th, const thermal_power_t *p,
                    const thermal_soa_t *soa, thermal_budget_t *out)
{
  if ((th == NULL) || (p == NULL) || (soa == NULL) || (out == NULL))
  {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->millis_to_limit = -1;

  float net[THERMAL_NODES];

  net_flows(th, p, th->speed_rpm, net);

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    const float limit = soa->limit_c[i];
    const float span = limit - th->ambient;

    if (!(span > 0.0f))
    {
      continue;              /* no limit set, or one below ambient */
    }

    float part = (th->t[i] - th->ambient) / span;

    if (part < 0.0f)
    {
      part = 0.0f;
    }
    if (part > 1.0f)
    {
      part = 1.0f;
    }
    out->used[i] = (uint8_t)(part * 255.0f);

    /* THE TRIP, on the record's own ceiling - any node at it, driven or not. */
    const float top = (soa->trip_c[i] > 0.0f) ? soa->trip_c[i] : limit;

    if (th->t[i] >= top)
    {
      out->tripped = true;
    }

    /* What is left in it, in joules. */
    const float left = limit - th->t[i];

    out->soak_j[i] = (left > 0.0f) ? (th->cfg.node[i].capacity * left) : 0.0f;

    if (soa->undriven[i])
    {
      continue;              /* nothing the clamp does moves this one */
    }
    if (out->used[i] >= out->worst)
    {
      out->worst = out->used[i];
      out->worst_node = (uint8_t)i;
    }
  }

  out->throttling = ((float)out->worst / 255.0f) >= soa->throttle_at;

  /* THE CLAMP'S FACTOR, on the worse of where a node is and how long it has
     - over every node the clamp reaches. */
  float spent = (float)out->worst / 255.0f;

  for (int i = 0; i < THERMAL_NODES; i++)
  {
    if (soa->undriven[i])
    {
      continue;
    }
    const float here = spend_of(th, net, soa, (thermal_node_t)i);

    if (here > spent)
    {
      spent = here;
    }
  }
  out->derate = derate_of(spent, soa);

  /* Time left, for the node that has least of it - the same hold the
     throttle acts on. */
  const float left = hold_seconds(th, net, soa,
                                  (thermal_node_t)out->worst_node);

  if (left > 0.0f)
  {
    const float millis = 1000.0f * left;

    out->millis_to_limit = (millis > 2.0e9f) ? 2000000000 : (int32_t)millis;
  }
}


float thermal_node_derate(const thermal_t *th, const thermal_power_t *p,
                          const thermal_soa_t *soa, thermal_node_t node)
{
  if ((th == NULL) || (p == NULL) || (soa == NULL) || (node >= THERMAL_NODES))
  {
    return 1.0f;
  }
  float net[THERMAL_NODES];

  net_flows(th, p, th->speed_rpm, net);
  const float spent = spend_of(th, net, soa, node);

  return (spent < 0.0f) ? 1.0f : derate_of(spent, soa);
}


float thermal_junction(const thermal_t *th, const thermal_power_t *p,
                       thermal_node_t node)
{
  if ((th == NULL) || (p == NULL) || (node >= THERMAL_NODES))
  {
    return NAN;
  }
  float watt = p->watt[node];

  /* A leg's node is two FETs and a driver; each FET carries half. */
  if (node <= THERMAL_DRIVER_W)
  {
    watt *= 0.5f;
  }
  return th->t[node] + watt * th->cfg.node[node].rth_die;
}


void thermal_losses(thermal_loss_t *loss)
{
  if (loss == NULL)
  {
    return;
  }
  memset(loss, 0, sizeof(*loss));

  /* The IAUCN10S7N021 VDMOS model in electronic_simulations: Ron 1.8 mOhm at
     25 C. */
  loss->rds_on    = 1.8e-3f;
  loss->rds_alpha = 7.8e-3f;

  /* RU1||RU2, two Vishay WSHM28187L000FEA of 7 mOhm - docs/HARDWARE.md. */
  loss->r_shunt = 3.5e-3f;

  /* The LM5069's back-to-back pass FETs, Q3 and Q4 - the bridge's own part,
     IAUCN10S7N021, two in series: 3.6 mOhm at 25 C. */
  loss->r_hotswap = 3.6e-3f;

  /* Measured 2026-08-28: three legs, 50 %, 24.6 V link, NO LOAD -> 1.20 W
     from difference 4-1 on the dead surface: the C_oss dump and the gate
     charge, and nothing else, since nothing was conducting. */
  loss->switching_watt = 1.20f;
  loss->switch_volts   = 24.6f;
  loss->driver_share   = 0.50f;

  /* Static. Consistent with the supply's 50 mA: 0.666+0.484+0.05 = 1.20 W. */
  loss->mcu_watt = 0.666f;
  loss->ldo_watt = 0.534f;
  loss->afe_watt = 0.13f;      /* from 2-1: the whole AFE chain and sensors */

  /* THE SWITCHING LOSS AS FUNCTIONS, 2026-09-05. */
  loss->f_sw       = 50.0e3f;
  loss->coss_cjo   = 15.6e-9f;
  loss->coss_m     = 0.45f;
  loss->coss_vj    = 0.7f;
  loss->t_switch_s = 14.0e-9f;
  loss->v_sd       = 0.85f;
  loss->q_g        = 81.0e-9f;
  loss->v_drive    = 12.0f;
  loss->buck_eff   = 0.85f;

  /* The winding: the record's `motor_r_uohm`, 50 mOhm as a placeholder until
     the commissioning writes it; the glue overwrites this from the record. */
  loss->r_phase = 0.05f;
  loss->k_iron  = 0.0f;
}


float thermal_coss_energy(const thermal_loss_t *loss, float volts)
{
  if ((loss == NULL) || !(volts > 0.0f) || !(loss->coss_cjo > 0.0f)
      || !(loss->coss_vj > 0.0f) || !(loss->coss_m < 1.0f))
  {
    return 0.0f;
  }
  /* E = integral of v C(v) dv with C = CJO / (1 + v/VJ)^M. */
  const float u = 1.0f + volts / loss->coss_vj;
  const float m = loss->coss_m;
  const float e = loss->coss_cjo * loss->coss_vj * loss->coss_vj
                  * ((powf(u, 2.0f - m) - 1.0f) / (2.0f - m)
                     - (powf(u, 1.0f - m) - 1.0f) / (1.0f - m));

  return (e > 0.0f) ? e : 0.0f;
}


void thermal_power_estimate(thermal_power_t *out, const thermal_load_t *load,
                            const thermal_loss_t *loss,
                            const float *phase_c)
{
  if ((out == NULL) || (load == NULL) || (loss == NULL))
  {
    return;
  }
  memset(out, 0, sizeof(*out));

  float link_from_phases = 0.0f;
  float sq_total = 0.0f;

  /* The no-load switching per driven leg: the calibrated figure scaled by
     the C_oss energy's own law between the link it was measured at and the
     link now. */
  const float link = (load->link_volts > 0.0f) ? load->link_volts
                                               : loss->switch_volts;
  float scale = 1.0f;
  const float e_cal = thermal_coss_energy(loss, loss->switch_volts);

  if (e_cal > 0.0f)
  {
    scale = thermal_coss_energy(loss, link) / e_cal;
  }
  else if (loss->switch_volts > 0.0f)
  {
    scale = link / loss->switch_volts;
  }
  const float per_leg = (loss->switching_watt / 3.0f) * scale;

  /* EACH LEG'S LOSS GOES TO THAT LEG. */
  for (int leg = 0; leg < 3; leg++)
  {
    const float a = load->phase_amps[leg];
    float rds = loss->rds_on;

    /* The FET's resistance follows the node it heats: first order off the
       datasheet chord, floored so a garbage estimate cannot make the loss
       vanish. */
    if ((phase_c != NULL) && !isnan(phase_c[leg]))
    {
      float factor = 1.0f + loss->rds_alpha * (phase_c[leg] - 25.0f);

      if (factor < 0.5f)
      {
        factor = 0.5f;
      }
      rds *= factor;
    }
    /* THE MEAN SQUARE WHERE THERE IS ONE. */
    const float sq = (load->phase_sq[leg] > 0.0f) ? load->phase_sq[leg]
                                                  : (a * a);
    const float irms = sqrtf(sq);

    /* SPLIT WHERE THE HEAT IS MADE: the FET's watts on the driver node, the
       shunt's on the phase node. */
    out->watt[THERMAL_DRIVER(leg)] += sq * rds;
    out->watt[THERMAL_PHASE(leg)] = sq * loss->r_shunt;
    link_from_phases += load->duty[leg] * a;
    sq_total += sq;

    if (load->switching && (load->duty[leg] > 0.0f))
    {
      /* The C_oss dump: the FET's, and the buck's share of what feeds it. */
      out->watt[THERMAL_DRIVER(leg)] += per_leg * loss->driver_share;
      out->watt[THERMAL_REGULATORS]  += per_leg * (1.0f - loss->driver_share);
      /* Overlap: `V I t f`, current and voltage on the FET together through
         its transitions. */
      const float overlap = link * irms * loss->t_switch_s * loss->f_sw;
      const float diode = 2.0f * loss->v_sd * (0.9f * irms) * load->t_dead_s
                          * loss->f_sw;
      const float gate = 2.0f * loss->q_g * loss->v_drive * loss->f_sw;

      out->watt[THERMAL_DRIVER(leg)] += overlap + diode + gate;
      if (loss->buck_eff > 0.0f)
      {
        out->watt[THERMAL_REGULATORS] += gate * (1.0f / loss->buck_eff - 1.0f);
      }
    }
  }

  /* Hot swap: it is in the link, so it sees link current, squared through
     its two FETs. */
  const float link_a = (load->link_amps >= 0.0f) ? load->link_amps
                                                 : link_from_phases;
  out->watt[THERMAL_HOTSWAP] += link_a * link_a * loss->r_hotswap;

  /* The winding: the three mean squares through the record's phase
     resistance - the same measurement the conduction rests on. */
  out->watt[THERMAL_WINDING] += sq_total * loss->r_phase;
  if ((loss->k_iron > 0.0f) && (load->speed_rpm > 0.0f))
  {
    const float krpm = load->speed_rpm / 1000.0f;

    out->watt[THERMAL_STATOR] += loss->k_iron * krpm * krpm;
  }

  /* Static. */
  out->watt[THERMAL_MCU]        += loss->mcu_watt;
  out->watt[THERMAL_REGULATORS] += loss->ldo_watt;
  out->watt[THERMAL_AFE]        += load->afe_on ? loss->afe_watt : 0.0f;
}


bool thermal_set_node(thermal_t *th, thermal_node_t node,
                      float k_per_w, float capacity)
{
  if ((th == NULL) || (node >= THERMAL_NODES)
      || !(k_per_w > 0.0f) || !(capacity > 0.0f))
  {
    return false;
  }
  const int edge = thermal_sink_edge(node);

  if (edge >= 0)
  {
    th->cfg.r_edge[edge] = k_per_w;
  }
  else
  {
    th->cfg.node[node].to_ambient = k_per_w;
  }
  th->cfg.node[node].capacity = capacity;
  return true;
}


bool thermal_set_edge(thermal_t *th, int edge, float k_per_w)
{
  if ((th == NULL) || (edge < 0) || (edge >= THERMAL_EDGES)
      || !(k_per_w >= 0.0f))
  {
    return false;
  }
  th->cfg.r_edge[edge] = k_per_w;
  return true;
}


bool thermal_set_board(thermal_t *th, float to_ambient, float capacity)
{
  if ((th == NULL) || !(to_ambient > 0.0f) || !(capacity > 0.0f))
  {
    return false;
  }
  th->cfg.board_to_ambient = to_ambient;
  /* Shared out by area, so the bulk's two numbers stay the bulk's. */
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    thermal_node_cfg_t *n = &th->cfg.node[i];

    if (n->area_share > 0.0f)
    {
      n->to_ambient = to_ambient / n->area_share;
      n->capacity   = capacity * n->area_share;
    }
  }
  return true;
}


void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius)
{
  if (th == NULL)
  {
    return;
  }
  memset(th, 0, sizeof(*th));
  if (cfg != NULL)
  {
    th->cfg = *cfg;
  }
  else
  {
    thermal_defaults(&th->cfg);
  }
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    th->t[i] = celsius;
  }
  th->ambient = celsius;
  th->ntc = celsius;
}


/** Where the thermistor's element is HEADING: the weighted average of the
    two patches it is tied to, `f` clamped to [0, 1] here rather than
    trusted, because a record is a thing a bench writes and an element
    outside its own interval is the defect this replaced. */
static float ntc_target(const thermal_t *th)
{
  const float centre = th->t[THERMAL_BOARD];
  const float leg = th->t[THERMAL_NTC_PATCH];
  float f = th->cfg.ntc_sees;

  if (f < 0.0f)
  {
    f = 0.0f;
  }
  if (f > 1.0f)
  {
    f = 1.0f;
  }
  return centre + f * (leg - centre);
}


float thermal_expected_ntc(const thermal_t *th)
{
  if (th == NULL)
  {
    return NAN;
  }
  return (th->cfg.ntc_tau_s > 0.0f) ? th->ntc : ntc_target(th);
}


float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float patch_rise_k)
{
  if (cfg == NULL)
  {
    return ntc_c;
  }
  return ntc_c - cfg->ntc_sees * patch_rise_k;
}


/** Pull one node to its die, and return the patch under it that implies: the
    node is patch + P * R into it, so subtracting reaches the patch without
    passing through anything else. */
static float anchor_die(thermal_t *th, thermal_node_t node, float seen,
                        const thermal_power_t *p, float k, float rate)
{
  const thermal_node_cfg_t *n = &th->cfg.node[node];
  const int edge = thermal_sink_edge(node);
  /* The die reads the junction; the node is the package. */
  const float at = seen - p->watt[node] * n->rth_die;

  th->t[node] += k * (at - th->t[node]);
  if (edge < 0)
  {
    return at;
  }
  /* AND THE NODE LAGS ITS PATCH. */
  const float r = th->cfg.r_edge[edge];

  return at - p->watt[node] * r + n->capacity * r * rate;
}


/** The patch a source sheds into, or the node itself. */
static thermal_node_t patch_under(thermal_node_t node)
{
  const int edge = thermal_sink_edge(node);

  return (edge >= 0) ? (thermal_node_t)THERMAL_EDGE_ENDS[edge].b : node;
}


/** One Euler slice over the whole graph. */
static void integrate(thermal_t *th, const thermal_power_t *p,
                      float speed_rpm, float dt_s)
{
  float net[THERMAL_NODES];

  net_flows(th, p, speed_rpm, net);
  for (int i = 0; i < THERMAL_NODES; i++)
  {
    const float c = th->cfg.node[i].capacity;

    if (c > 0.0f)
    {
      th->t[i] += net[i] * dt_s / c;
    }
  }
}


void thermal_integrate(thermal_t *th, const thermal_power_t *p,
                       float speed_rpm, float dt_s)
{
  if ((th != NULL) && (p != NULL) && (dt_s > 0.0f))
  {
    integrate(th, p, speed_rpm, dt_s);
  }
}


/** What the sensors say, folded in: each die corrects its node and the patch
    under it, the thermistor the V leg's patch; then ambient. */
static void anchor(thermal_t *th, const thermal_power_t *p,
                   const thermal_sense_t *seen, float speed_rpm,
                   float since_s)
{
  const float k = 1.0f - expf(-THERMAL_ANCHOR_HZ * since_s);
  int dies = 0;
  float common = 0.0f;
  bool held[THERMAL_NODES];
  float net[THERMAL_NODES];

  memset(held, 0, sizeof(held));
  net_flows(th, p, speed_rpm, net);      /* the dies' nodes' own rates */
  if (!isnan(seen->afe_c))
  {
    const float implied = anchor_die(th, THERMAL_AFE, seen->afe_c, p, k,
                                     net[THERMAL_AFE]
                                     / th->cfg.node[THERMAL_AFE].capacity);
    const thermal_node_t patch = patch_under(THERMAL_AFE);

    common += implied - th->t[patch];
    th->t[patch] += k * (implied - th->t[patch]);
    held[THERMAL_AFE] = true;
    held[patch] = true;
    dies++;
  }
  if (!isnan(seen->mcu_c))
  {
    const float implied = anchor_die(th, THERMAL_MCU, seen->mcu_c, p, k,
                                     net[THERMAL_MCU]
                                     / th->cfg.node[THERMAL_MCU].capacity);
    const thermal_node_t patch = patch_under(THERMAL_MCU);

    common += implied - th->t[patch];
    th->t[patch] += k * (implied - th->t[patch]);
    held[THERMAL_MCU] = true;
    held[patch] = true;
    dies++;
  }

  if (dies > 0)
  {
    /* THE REST OF THE LAMINATE MOVES WITH THE DIES. */
    common /= (float)dies;
    for (int i = 0; i < (int)THERMAL_WINDING; i++)
    {
      if (!held[i] && (th->cfg.node[i].capacity > 0.0f))
      {
        th->t[i] += k * common;
      }
    }

    /* Settled is about the LAMINATE: a die anchors a patch without guessing
       how much of the NTC is hot spot. */
    th->settled = true;

    if (!isnan(seen->ntc_c) && (th->cfg.ntc_sees > 0.01f))
    {
      /* THE THERMISTOR IS COMPARED WITH THE ELEMENT AS MODELLED, not with
         the average it is heading for. */
      const float miss = seen->ntc_c - th->ntc;
      const float leg = th->t[THERMAL_NTC_PATCH];
      const float centre = th->t[THERMAL_BOARD];
      /* HELD AT THE LEG, the element IS the leg's patch and the miss is that
         patch's, one for one; between the patches, it is the share the V
         patch shows through. */
      /* Within a band of the leg counts as at it: the element set to a
         reading a hair under the patch, then integrated below it, made the
         free gain act on a miss that was the patch's one for one - a
         twelvefold overshoot every third sample. */
      const bool at_leg = (leg >= centre)
                          && ((seen->ntc_c >= leg - THERMAL_NTC_AT_LEG_K)
                              || (th->ntc >= leg - THERMAL_NTC_AT_LEG_K));
      /* AND ONLY A MISS THAT GREW OVER ONE INTERVAL is the patch's. */
      const bool fresh = since_s <= THERMAL_NTC_INVERT_MAX_S;
      /* THROUGH THE LAG AS WELL AS THE SHARE. */
      float lag_gain = 1.0f;

      if ((th->cfg.ntc_tau_s > 0.0f) && (since_s > 0.0f))
      {
        lag_gain = 0.5f * th->cfg.ntc_tau_s / since_s;
        lag_gain = (lag_gain < 1.0f) ? 1.0f : (lag_gain > 8.0f) ? 8.0f : lag_gain;
      }
      const float through = at_leg ? 1.0f
                            : (fresh ? (lag_gain / th->cfg.ntc_sees) : 0.0f);
      const float move = k * miss * through;

      th->ntc += k * miss;
      /* THE THREE LEGS ARE ONE LAYOUT, MIRRORED. */
      th->t[THERMAL_NTC_PATCH] += move;
      for (int leg_i = 0; leg_i < 3; leg_i++)
      {
        const thermal_node_t other = THERMAL_PATCH(leg_i);

        if ((other != THERMAL_NTC_PATCH) && !held[other])
        {
          th->t[other] += move;
        }
      }
    }

    /* THE ROOM IS NOT ESTIMATED HERE. */
  }
  else if (!isnan(seen->ntc_c))
  {
    /* Degraded: no die answered, so the V patch's rise cannot be separated
       from the centre's. */
    const float miss = seen->ntc_c - th->ntc;

    th->ntc += k * miss;
    for (int i = 0; i < (int)THERMAL_WINDING; i++)
    {
      if (th->cfg.node[i].capacity > 0.0f)
      {
        th->t[i] += k * miss;
      }
    }
    th->settled = false;
  }
}


void thermal_step(thermal_t *th, const thermal_power_t *p,
                  const thermal_sense_t *seen, const thermal_load_t *load,
                  float dt_s)
{
  if ((th == NULL) || (p == NULL) || (seen == NULL) || !(dt_s > 0.0f))
  {
    return;
  }
  if (dt_s > THERMAL_DT_MAX)
  {
    dt_s = THERMAL_DT_MAX;
  }
  const float speed = (load != NULL) ? load->speed_rpm : 0.0f;

  th->speed_rpm = speed;

  /* SUB-STEPPED. */
  float left = dt_s;

  while (left > 0.0f)
  {
    const float slice = (left > THERMAL_DT_SLICE) ? THERMAL_DT_SLICE : left;

    integrate(th, p, speed, slice);
    left -= slice;
  }

  th->since_seen_s += dt_s;
  if (!isnan(seen->ntc_c) || !isnan(seen->afe_c) || !isnan(seen->mcu_c))
  {
    anchor(th, p, seen, speed, th->since_seen_s);
    th->since_seen_s = 0.0f;
  }

  (void)thermal_ntc_follow(th, dt_s);

  th->steps++;
}


int thermal_ntc_follow(thermal_t *th, float dt_s)
{
  if (th == NULL)
  {
    return -1;
  }

  /* THE THERMISTOR FOLLOWS, it does not jump. */
  if ((th->cfg.ntc_tau_s > 0.0f) && (dt_s > 0.0f))
  {
    const float share = fminf(dt_s / th->cfg.ntc_tau_s, 1.0f);

    th->ntc += (ntc_target(th) - th->ntc) * share;
  }
  else if (!(th->cfg.ntc_tau_s > 0.0f))
  {
    th->ntc = ntc_target(th);
  }

  /* AND NEVER PAST EITHER OF THEM: a passive element between two nodes
     cannot read outside the pair, whatever its own lag - the series network
     of docs/papers, 2.3. */
  const float centre = th->t[THERMAL_BOARD];
  const float leg = th->t[THERMAL_NTC_PATCH];
  const thermal_node_t low_at = (centre < leg) ? THERMAL_BOARD
                                               : THERMAL_NTC_PATCH;
  const thermal_node_t high_at = (centre < leg) ? THERMAL_NTC_PATCH
                                                : THERMAL_BOARD;

  if (th->ntc < th->t[low_at])
  {
    th->ntc = th->t[low_at];
    return (int)low_at;
  }
  if (th->ntc > th->t[high_at])
  {
    th->ntc = th->t[high_at];
    return (int)high_at;
  }
  return -1;
}
