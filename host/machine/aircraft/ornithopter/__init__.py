"""The Dune-style 4-wing flapping ornithopter in MuJoCo: articulated wings, dive fold, and stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

An agile 2 kg 4-wing flapping ornithopter (`ornithopter.lisp`) powered by two Coaxial 63100 drives
(`FORE` and `AFT` driving the fore and aft articulated dragonfly wing pairs on finned aluminium
heatsinks with temperature-controlled cooling fans): collective stroke amplitude and wing pitch
pull upright or inverted, stroke-plane tilt pitches the carapace, differential left/right stroke
rolls it, and fore/aft wing-twist skew yaws it through pirouettes, sandstorm dives, and gates.
Its numbers and its XML (`airframe`), its flight in MuJoCo (`sky`), its law (`law`) and its
observer (`observer`).
"""
from machine.aircraft.course import TWIN_SPHERE as COURSE_SPHERE
from machine.aircraft.ornithopter.airframe import (AFT_Z, BEHAVIORS, BODY_CDA, CARD, COOLING, FLOOR_M,
                                          FOOT_M, FORE_Z, GEAR_RATIO, GRAVITY, HALF_M, IDLE_RAD_S, INERTIA,
                                          K_DRAG, K_PITCH, K_ROLL, K_THRUST, K_YAW, MANIFEST, MASS_KG,
                                          PULL_MAX, RHO, SANDSTORM_DIVE, SHOULDER_X, SHOULDER_Y, SKID_M,
                                          SKID_X, STREAMS, STRUT_Z, STUNTS, WING_CHORD, WING_SPAN,
                                          _flap_inflow, drive_loads, row_from_payload, wing_kinematics)
from machine.aircraft.ornithopter.law import Core
from machine.aircraft.ornithopter.sky import Sky
