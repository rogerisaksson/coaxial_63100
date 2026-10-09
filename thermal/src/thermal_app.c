/** thermal_app.c - The applications a board is mounted in, over the still air's network. */
#include "thermal.h"

#include <math.h>

/* Ballpark, every figure: a reason each, a measurement none, until a test cycle runs in
   the application (the user, 2026-10-09). Per application, thermal_app_t's order: still air,
   a rotor's wash, a sealed finned housing, a fan's finned sink, a liquid's plate, PAO,
   transformer oil. */

/** The laminate's air path over the still air's: a sealed housing's air rides on the
    housing, three times; oil's natural convection 100-150 W/m^2 K against air's 7 with
    its radiation, a sixteenth and a twelfth. */
static const float APP_LAMINATE_AIR[THERMAL_APPS] = { 1.0f, 1.0f, 3.0f, 1.0f, 1.0f,
                                                      0.06f, 0.08f };

/** Its forced gain per sqrt(krpm): behind the stator a third of the stator's; in the wash
    1.9 - a hover's 1 470 rpm at 2.5 K/W, the slipstream going as the rotor's speed and h as
    its root; sealed none; the oil stirred. */
static const float APP_LAMINATE_FORCED[THERMAL_APPS] = { 0.3f, 1.9f, 0.0f, 0.3f, 0.3f,
                                                         0.2f, 0.2f };

/** A leg's switches into the laminate under them, K/W, 0 the still air's 12: pressed on
    the body through a pad, the vias straight under them. */
static const float APP_LEG_INTO[THERMAL_APPS] = { 0.0f, 0.0f, 4.0f, 3.0f, 2.0f, 0.0f, 0.0f };

/** Each leg's patch onto the body, K/W, 0 open: a gap pad under the leg. */
static const float APP_LEG_MOUNT[THERMAL_APPS] = { 0.0f, 0.0f, 2.0f, 1.5f, 1.0f, 0.0f, 0.0f };

/** The other rim patches onto the body, K/W, 0 open: their standoffs. */
static const float APP_RIM_MOUNT[THERMAL_APPS] = { 0.0f, 0.0f, 20.0f, 20.0f, 20.0f,
                                                   0.0f, 0.0f };

/** The body - the stator's iron and whatever it and the board are fixed to - its air path
    over the still air's, the J/K that adds and its forced gain: a finned housing 1 K/W and
    270 J/K of aluminium, a fan's sink 0.4 and 180, a plate 0.05 to its coolant and 100; in
    the wash the motor's own air forced harder; in oil as the laminate, a little less. */
static const float APP_BODY_AIR[THERMAL_APPS] = { 1.0f, 1.0f, 0.6f, 0.25f, 0.03f,
                                                  0.1f, 0.12f };
static const float APP_BODY_CAPACITY[THERMAL_APPS] = { 0.0f, 0.0f, 270.0f, 180.0f, 100.0f,
                                                       0.0f, 0.0f };
static const float APP_BODY_FORCED[THERMAL_APPS] = { 0.5f, 1.5f, 0.0f, 0.0f, 0.0f,
                                                     0.2f, 0.2f };

/** The bell's air path over the still air's: in oil as the body's. */
static const float APP_BELL_AIR[THERMAL_APPS] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.1f, 0.12f };

/** A rotor's wash over its speed, m/s per krpm, where the wash is the air: twice the induced
    speed, 6.35 m/s at a hover's 1 470 rpm; 0 where the air is not a rotor's. */
static const float APP_WASH_M_S_PER_KRPM[THERMAL_APPS] = { 0.0f, 4.32f, 0.0f, 0.0f, 0.0f,
                                                           0.0f, 0.0f };

float thermal_air_rpm(thermal_app_t app, float speed_rpm, float airspeed_m_s)
{
  if (((unsigned)app >= (unsigned)THERMAL_APPS) || !(APP_WASH_M_S_PER_KRPM[app] > 0.0f) ||
      !(airspeed_m_s > 0.0f))
  {
    return speed_rpm;
  }
  const float flown_rpm = 1000.0f * airspeed_m_s / APP_WASH_M_S_PER_KRPM[app];

  return sqrtf((speed_rpm * speed_rpm) + (flown_rpm * flown_rpm));
}

void thermal_application(thermal_cfg_t *cfg, thermal_app_t app)
{
  if ((cfg == NULL) || ((int)app <= (int)THERMAL_APP_STILL) || (app >= THERMAL_APPS))
  {
    return;                                  /* still air is the network as it is */
  }
  for (int i = 0; i < (int)THERMAL_NODES; i++)
  {
    thermal_node_cfg_t *n = &cfg->node[i];

    if (n->area_share > 0.0f)
    {
      n->to_ambient *= APP_LAMINATE_AIR[app];
      n->forced = APP_LAMINATE_FORCED[app];
    }
  }
  cfg->board_to_ambient *= APP_LAMINATE_AIR[app];
  for (int leg = 0; leg < 3; leg++)
  {
    if (APP_LEG_INTO[app] > 0.0f)
    {
      cfg->r_edge[leg] = APP_LEG_INTO[app];      /* edges 0..2: a leg into its patch */
    }
    cfg->r_edge[THERMAL_EDGE_MOUNT_FIRST + leg] = APP_LEG_MOUNT[app];
    cfg->r_edge[THERMAL_EDGE_MOUNT_FIRST + 3 + leg] = APP_RIM_MOUNT[app];
  }
  thermal_node_cfg_t *body = &cfg->node[THERMAL_STATOR];

  body->to_ambient *= APP_BODY_AIR[app];
  body->capacity += APP_BODY_CAPACITY[app];
  body->forced = APP_BODY_FORCED[app];
  cfg->node[THERMAL_ROTOR].to_ambient *= APP_BELL_AIR[app];
}
