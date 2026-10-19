"""HELICOPTER: two coaxial boards flying a 3D helicopter in MuJoCo, its stunts and gates.

A 2 kg single-main-rotor 3D helicopter (`machine.helicopter`) on two stand-in 63100 boards (`MAIN`
on the mast and `TAIL` on the boom), each armed on the master's pilot and running sensorless on the
63100 outrunner's model; the two on one 63 V pack; the airframe in MuJoCo under signed collective
pitch, cyclic swashplate hub moments, main-rotor reaction torque and tail-rotor anti-torque
(`terminal.views.helicopter.flight`). It flies flight after flight: first its 3D aerobatics routine
- hover, fast pirouette, banked funnel, climbing spiral, half-flip to sustained negative-collective
inverted hover and inverted pirouette, elevator tic-tocs, 3D piro-flip, axial roll, full-tilt
skyrocket punch-out, and collective burn stop 10 cm over the floor -, viewed from a side RC-pilot
perspective that zooms slightly to follow while keeping contact with the horizon, and then two laps
of the shared 16-gate course (`machine.course`) chased from behind.

    python terminal/views/show_helicopter.py
    python terminal/views/show_helicopter.py --frames 60
"""
import math
import sys

from rich.text import Text

from coaxial.graphics import helicopter as heli_gfx
from machine import helicopter
from terminal.views.helicopter import flight as flown
from terminal.views.show_quad import (SIDE_PITCH, ZOOM, carried, compose_craft, flown_row, hands,
                                      look as quad_look, recorded, run_craft_view, size_of, typed,
                                      wheeled)

TITLE = 'HELICOPTER 3D FLYING'
ROTORS = flown.ROTORS
NEAR_M, CHASE_M, YAW, PITCH = 0.76, 2.2, 52.0, 18.0


def look(camera, law, row, frame, chased, dt):
    """The helicopter camera `dt` s on, `(reach, centre)`: its spectator during its figures,
    behind its heading on the gate course (`show_quad.look`)."""
    return quad_look(camera, law, row, frame, chased, dt, near_m=NEAR_M, chase_m=CHASE_M, base_yaw=YAW,
                     chase_pitch=PITCH, build=heli_gfx.heli_mesh, sight=heli_gfx.SIGHT)


def thrusts(frame, rotors):
    """`(main_thrust_n, tail_thrust_n)` in N as the helicopter flies, signed with collective and
    tail pitch."""
    turn = frame['turn']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(frame['vel']))
    coll, _cyc_p, _cyc_r, tail_p = frame.get('controls', (0.0, 0.0, 0.0, 0.0))
    w_main, w_tail = rotors[0]['w'], rotors[1]['w']
    signed_along = along if coll >= 0.0 else -along
    main_n = helicopter.K_THRUST * w_main * w_main * coll * helicopter.inflow(signed_along, w_main)
    tail_n = helicopter.K_TAIL * w_tail * w_tail * tail_p
    return main_n, tail_n


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the 3D helicopter in the viewport and the side HUD."""
    coll, cyc_p, cyc_r, tail_p = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))
    cyc_deg = math.hypot(cyc_p, cyc_r) * helicopter.CYC_DEG
    swash = ('SWASH', Text('%+4.1f coll %4.1f cyc %+4.1f tail' % (
        coll * helicopter.COLL_DEG, cyc_deg, tail_p * helicopter.TAIL_DEG),
        style='alarm' if coll < -0.15 else 'value'))
    return compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw,
                         TITLE, helicopter, flown, swash)


def main(argv=None):
    return run_craft_view(argv, __doc__, helicopter, heli_gfx, flown, compose, thrusts, look,
                          YAW, NEAR_M)


HELI_REEXPORTS = (SIDE_PITCH, ZOOM, carried, compose_craft, flown_row, hands, recorded,
                  run_craft_view, size_of, typed, wheeled)


if __name__ == '__main__':
    sys.exit(main())


