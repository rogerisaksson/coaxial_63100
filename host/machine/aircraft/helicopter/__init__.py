"""The 3D aerobatic helicopter in MuJoCo: collective/cyclic swashplate, tail rotor, and stunts.

    sky, core = Sky(), Core()
    out = core.step(law, row, sky.state(), dt, reach)
    sky.step(out.speeds + out.controls, dt, air)

Two 63100 drives: `MAIN` on the mast, `TAIL` on the boom. Its numbers and its XML (`airframe`),
its flight in MuJoCo (`sky`), its law (`law`) and its observer (`observer`); `kit` the SO(3)
law's pieces it shares with the ornithopter and the aerobat.
"""
from machine.aircraft.course import TWIN_SPHERE as COURSE_SPHERE
from machine.aircraft.helicopter.airframe import (BEHAVIORS, BODY_CDA, CARD, COLL_DEG, COOLING,
                                         CYC_DEG, DISC_M, FLIP_IN, FLOOR_M, FOOT_M, GRAVITY, HALF_M,
                                         HOVER, IDLE_RAD_S, INERTIA, INVERTED, K_DRAG, K_TAIL,
                                         K_THRUST, LISP_PATH, MANIFEST, MASS_KG, PULL_MAX,
                                         QUAD_MANIFEST, RHO, ROTOR_R, SKID_M, SKID_X, STREAMS,
                                         STRUT_Z, STUNTS, TAIL_DEG, TAIL_M, TAIL_R, drive_loads,
                                         inflow, mjcf, row_from_payload)
from machine.aircraft.helicopter.law import Core
from machine.aircraft.helicopter.sky import Sky
