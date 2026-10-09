"""What a board is mounted in, over the still air's network: thermal_app.c's, mirrored.

Its tables in its order, by its rule. Ballpark, every figure - thermal_app.c has each one's
reason - until a test cycle runs in the application (2026-10-09).
"""
import copy
import math

#: thermal_app_t, in order: still air (a bench), a rotor's wash (a drone's), a sealed finned
#: aluminium housing, a fan's finned sink, a liquid's cold plate, PAO, transformer oil.
APPLICATIONS = ('still', 'airstream', 'enclosure', 'fan_sink', 'cold_plate', 'immersion_pao',
                'immersion_oil')
STILL = APPLICATIONS[0]

#: Per application: the laminate's air path over the still air's and its forced gain per
#: sqrt(krpm); a leg's switches into their patch, K/W, 0 the still air's; each leg's patch
#: and the other rim patches onto the body, K/W, 0 open; the body's (the stator's) air path
#: over the still air's, the J/K it gains and its forced gain; the bell's air path.
LAMINATE_AIR = (1.0, 1.0, 3.0, 1.0, 1.0, 0.06, 0.08)
LAMINATE_FORCED = (0.3, 1.9, 0.0, 0.3, 0.3, 0.2, 0.2)
LEG_INTO = (0.0, 0.0, 4.0, 3.0, 2.0, 0.0, 0.0)
LEG_MOUNT = (0.0, 0.0, 2.0, 1.5, 1.0, 0.0, 0.0)
RIM_MOUNT = (0.0, 0.0, 20.0, 20.0, 20.0, 0.0, 0.0)
BODY_AIR = (1.0, 1.0, 0.6, 0.25, 0.03, 0.1, 0.12)
BODY_CAPACITY = (0.0, 0.0, 270.0, 180.0, 100.0, 0.0, 0.0)
BODY_FORCED = (0.5, 1.5, 0.0, 0.0, 0.0, 0.2, 0.2)
BELL_AIR = (1.0, 1.0, 1.0, 1.0, 1.0, 0.1, 0.12)
#: A rotor's wash over its speed, m/s per krpm, where the wash is the air; 0 where it is not.
WASH_M_S_PER_KRPM = (0.0, 4.32, 0.0, 0.0, 0.0, 0.0, 0.0)

#: The host's word on the airspeed holds this long, wall s (THERMAL_AIRSPEED_HOLD_MS), and
#: goes to this at the most, m/s (THERMAL_AIRSPEED_MAX_MM_S).
AIRSPEED_HOLD_S, AIRSPEED_MAX_M_S = 1.0, 100.0

#: The first of the mount's six edges, the stator onto patches U, V, W, left, bottom, right.
EDGE_MOUNT_FIRST = 24


def air_rpm(app, speed_rpm, airspeed_m_s):
    """The rotor speed whose wash alone is the air over the board - `thermal_air_rpm`."""
    wash = WASH_M_S_PER_KRPM[APPLICATIONS.index(app)]
    if wash <= 0.0 or airspeed_m_s <= 0.0:
        return speed_rpm
    return math.hypot(speed_rpm, 1000.0 * airspeed_m_s / wash)


def applied(cfg, app):
    """`cfg`, the still air's network, as `app` has it: a copy - `thermal_application`."""
    k = APPLICATIONS.index(app)
    out = copy.deepcopy(cfg)
    if k == 0:
        return out
    for name, share in out['area_share'].items():
        if share > 0.0:
            out['to_ambient'][name] *= LAMINATE_AIR[k]
            out['forced'][name] = LAMINATE_FORCED[k]
    out['board_to_ambient'] *= LAMINATE_AIR[k]
    for leg in range(3):
        if LEG_INTO[k] > 0.0:
            out['edges'][leg] = LEG_INTO[k]
        out['edges'][EDGE_MOUNT_FIRST + leg] = LEG_MOUNT[k]
        out['edges'][EDGE_MOUNT_FIRST + 3 + leg] = RIM_MOUNT[k]
    out['to_ambient']['stator'] *= BODY_AIR[k]
    out['capacity']['stator'] += BODY_CAPACITY[k]
    out['forced']['stator'] = BODY_FORCED[k]
    out['to_ambient']['rotor'] *= BELL_AIR[k]
    return out
