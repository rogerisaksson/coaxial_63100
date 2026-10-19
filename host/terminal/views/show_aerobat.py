"""AEROBAT: two coaxial boards flying a 3D RC aerobatic monoplane in MuJoCo, stunts and gates.

A 2 kg Extra 330SC / Edge 540 style 3D RC aerobatic monoplane (`machine.aerobat`) on two stand-in
63100 boards (`PROP` driving the single tractor propeller and `VECT` the tail/vector drive on
finned aluminium heatsinks with temperature-controlled cooling fans), each armed on the master's
pilot and running sensorless on the 63100 outrunner's model; the two on one 63 V pack; the airframe
in MuJoCo under 3D prop-hang thrust and propwash-deflected elevator, full-span ailerons, and rudder
(`terminal.views.aerobat.flight`). It takes off, turns back down the runway in a knife-edge, flies
its 3D routine - prop-hang, torque roll, climbing harrier, waterfall, snap roll, unwind, prop-hang
burn arrest 10 cm over the floor -, lands, and then flies two laps of its airfield's gate course
(`machine.airfield`).

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
NEAR_M, CHASE_M, YAW, PITCH = 0.85, 2.4, 58.0, 16.0


def look(camera, law, row, frame, chased, dt):
    """The aerobat camera, `(reach, centre)`: the spectator beside the runway (`show_quad.look`);
    on its course the quad's chase."""
    return quad_look(camera, law, row, frame, chased, dt, near_m=NEAR_M, chase_m=CHASE_M,
                     base_yaw=YAW, chase_pitch=PITCH, build=aero_gfx.aero_mesh, sight=aero_gfx.SIGHT)


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
