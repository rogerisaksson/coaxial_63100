"""AEROBAT: two coaxial boards flying a 3D RC aerobatic monoplane in MuJoCo, stunts and gates.

A 2 kg Extra 330SC / Edge 540 style 3D RC aerobatic monoplane (`machine.aerobat`) on two stand-in
63100 boards (`PROP` and `VECT` driving the front and rear coaxial contra-rotating tractor stages
on finned aluminium heatsinks with temperature-controlled cooling fans), each armed on the master's
pilot and running sensorless on the 63100 outrunner's model; the two on one 63 V pack; the airframe
in MuJoCo under reversible 3D prop-hang thrust and propwash-deflected elevator, full-span ailerons,
and rudder (`terminal.views.aerobat.flight`). It takes off, turns back down the runway in a knife-edge, flies its 3D routine - prop-hang,
torque roll, climbing harrier, waterfall, snap roll, unwind, prop-hang burn arrest 10 cm over
the floor -, lands, and then flies two laps of its airfield's gate course (`machine.airfield`).

    python terminal/views/show_aerobat.py
    python terminal/views/show_aerobat.py --frames 60
"""
import math
import sys

from rich.text import Text

from coaxial.graphics import aerobat as aero_gfx
from machine import aerobat
from terminal.views import show_helicopter as base_view
from terminal.views.aerobat import flight as aero_flown
from terminal.views.show_quad import look as quad_look

TITLE, ROTORS = 'AIRCRAFT XE AEROBATICS', aero_flown.ROTORS
NEAR_M, FAR_M, ZOOM_M, ZOOM_K, CHASE_M, YAW, PITCH = 0.85, 0.90, 0.60, 0.022, 2.4, 58.0, 16.0
#: The spectator: by the hangar beside the runway's middle, SPOT_ACROSS off its line, the eye a
#: man's height up, turning after the plane with a fixed lens - framed as near or far as it is,
#: SIGHT reaches off. Zoomed onto the plane and chasing it, the figure was never in view.
SPOT_ALONG, SPOT_ACROSS, SPOT_Y = -2.0, -14.0, 1.7
_RC, _RS = math.cos(aerobat.RUNWAY_RAD), math.sin(aerobat.RUNWAY_RAD)
SPOT_X, SPOT_Z = SPOT_ALONG * _RS + SPOT_ACROSS * _RC, SPOT_ALONG * _RC - SPOT_ACROSS * _RS


def look(camera, law, row, frame, chased, dt):
    """The aerobat camera, `(reach, centre)`: the spectator beside the runway turning after the
    plane; chased, the quad's chase (`quad_look`)."""
    if chased:
        return quad_look(camera, law, row, frame, chased, dt, near_m=NEAR_M, far_m=FAR_M,
                         zoom_m=ZOOM_M, zoom_k=ZOOM_K, chase_m=CHASE_M, base_yaw=YAW,
                         chase_pitch=PITCH)
    x, y, z = (float(c) for c in frame['at'])
    dx, dy, dz = x - SPOT_X, y - SPOT_Y, z - SPOT_Z
    dh = max(0.5, math.hypot(dx, dz))
    camera['yaw'] = math.degrees(math.atan2(-dx, -dz))
    camera['pitch'] = math.degrees(math.atan2(-dy, dh))
    return math.hypot(dh, dy) / aero_gfx.SIGHT / camera['zoom'], (x, y, z)


def thrusts(frame, rotors):
    """`(prop_thrust_n, rudder_yaw_n)` in N as the 3D monoplane flies."""
    turn, vel = frame['turn'], frame['vel']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(vel))
    throt, _elev, _ail, rud = frame.get('controls', (0.0, 0.0, 0.0, 0.0))
    w_prop, w_vect = rotors[0]['w'], rotors[1]['w']
    signed_along = along if throt >= 0.0 else -along
    inf = aerobat._aero_inflow(signed_along, math.hypot(float(vel[0]), float(vel[2])), w_prop)
    prop_n = aerobat.K_THRUST * w_prop * w_prop * throt * inf
    rud_n = aerobat.K_YAW * w_vect * w_vect * rud
    return prop_n, rud_n


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the 3D aerobat in the viewport and the side HUD (with `SURFS` deflections)."""
    controls = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))
    elev_deg, ail_deg, rud_deg = aerobat.surface_angles(controls)
    surfs = ('SURFS', Text('%+4.0f elv %+4.0f ail %+4.0f rud' % (elev_deg, ail_deg, rud_deg),
                           style='alarm' if controls[0] < -0.15 else 'value'))
    return base_view.compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw,
                                   TITLE, aerobat, aero_flown, surfs)


def main(argv=None):
    return base_view.run_craft_view(
        argv, __doc__, aerobat, aero_gfx, aero_flown, compose, thrusts, look, YAW)


if __name__ == '__main__':
    sys.exit(main())
