"""ORNITHOPTER: two coaxial boards flying a 4-wing Dune ornithopter in MuJoCo, stunts and gates.

A 2 kg Dune-style 4-wing flapping ornithopter (`machine.ornithopter`) on two stand-in 63100 boards
(`FORE` and `AFT` driving the fore and aft articulated dragonfly wing pairs on finned aluminium
heatsinks with temperature-controlled cooling fans), each armed on the master's pilot and running
sensorless on the 63100 outrunner's model; the two on one 63 V pack; the airframe in MuJoCo under
signed wing-stroke amplitude, stroke-plane pitch tilt, differential roll stroke, and fore/aft wing
twist skew (`terminal.views.ornithopter.flight`). It flies its Dune stunt routine - desert hover,
pirouette, ridge soar, thermal spiral, backward wing-fold dive entry, inverted sandstorm dive,
wing-rock and bank-rock oscillations, Coriolis storm, and stroke burn arrest 10 cm over the desert
floor -, and then two laps of the shared 16-gate course (`machine.course`).

    python terminal/views/show_ornithopter.py
    python terminal/views/show_ornithopter.py --frames 60
"""
import math
import sys

from rich.text import Text

from coaxial.graphics import ornithopter as orni_gfx
from machine import ornithopter
from terminal.views import show_helicopter as base_view
from terminal.views.ornithopter import flight as orni_flown
from terminal.views.show_quad import look as quad_look

TITLE, ROTORS = 'ORNITHOPTER DUNE FLYING', orni_flown.ROTORS
NEAR_M, FAR_M, ZOOM_M, ZOOM_K, CHASE_M, YAW, PITCH = 0.76, 0.80, 0.60, 0.022, 2.2, 56.0, 16.0


def look(camera, law, row, frame, chased, dt):
    """The ornithopter camera `dt` s on, `(reach, centre)`: RC side perspective with horizon
    contact during the Dune aerobatics routine, and behind the heading on the gate course."""
    return quad_look(camera, law, row, frame, chased, dt, near_m=NEAR_M, far_m=FAR_M,
                     zoom_m=ZOOM_M, zoom_k=ZOOM_K, chase_m=CHASE_M, base_yaw=YAW,
                     chase_pitch=PITCH)


def thrusts(frame, rotors):
    """`(fore_thrust_n, aft_thrust_n)` in N from the fore and aft flapping wing pairs."""
    turn, vel = frame['turn'], frame['vel']
    along = sum(float(v) * float(turn[k][1]) for k, v in enumerate(vel))
    stroke = frame.get('controls', (0.0, 0.0, 0.0, 0.0))[0]
    w_fore, w_aft = rotors[0]['w'], rotors[1]['w']
    signed_along = along if stroke >= 0.0 else -along
    w_eq = math.sqrt(0.5 * (w_fore * w_fore + w_aft * w_aft))
    inf = ornithopter._flap_inflow(signed_along, math.hypot(float(vel[0]), float(vel[2])), w_eq)
    return (0.5 * ornithopter.K_THRUST * w_fore * w_fore * stroke * inf,
            0.5 * ornithopter.K_THRUST * w_aft * w_aft * stroke * inf)


def compose(console, origin, rotors, frame, flight, trace, now, art, yaw=0.0):
    """One frame: the 3D ornithopter in the viewport and the side HUD (with `WINGS` kinematics)."""
    controls = frame.get('controls', flight.get('controls', (0.0, 0.0, 0.0, 0.0)))
    beat_hz, stroke_deg, fold_deg = ornithopter.wing_kinematics(
        controls, (rotors[0]['w'], rotors[1]['w']))
    wings = ('WINGS', Text('%4.1f Hz %+4.0f strk %3.0f fold' % (beat_hz, stroke_deg, fold_deg),
                           style='alarm' if controls[0] < -0.15 else 'value'))
    return base_view.compose_craft(console, origin, rotors, frame, flight, trace, now, art, yaw,
                                   TITLE, ornithopter, orni_flown, wings)


def main(argv=None):
    return base_view.run_craft_view(
        argv, __doc__, ornithopter, orni_gfx, orni_flown, compose, thrusts, look, YAW)


if __name__ == '__main__':
    sys.exit(main())
