"""The unlimited 3D RC aerobatic monoplane in MuJoCo: prop-hang, big surfaces, stunts.

    out = core.step(law, row, sky.state(), dt, reach); sky.step(out.speeds + out.controls, dt, air)

An Edge 540 style 3D monoplane (`aerobat.lisp`) on two 63100 drives, `PROP` the tractor
propeller and `VECT` the coaxial contra stage: propwash over oversized surfaces gives attitude
authority from zero airspeed. Its numbers, its wing and its XML (`airframe`), its flight in
MuJoCo (`sky`), its law (`law`) and its observer (`observer`).
"""
from machine.aerobat.airframe import (BEHAVIORS, BODY_CDA, CARD, CD0, CD_INDUCED, CL_ALPHA, COOLING,
                                      COURSE_SPHERE, FLOOR_M, FOOT_M, GEAR_MAIN_Z, GEAR_TAIL_Z,
                                      GRAVITY, HALF_M, IDLE_RAD_S, INERTIA, K_DRAG, K_PITCH, K_ROLL,
                                      K_THRUST, K_YAW, LISP_PATH, MANIFEST, MASS_KG, PROP_R, PROP_Z,
                                      PROPWASH_SHARE, PULL_MAX, RHO, RUNWAY_M, RUNWAY_RAD, SKID_M,
                                      SKID_X, STAB_SPAN, STREAMS, STRUT_Z, STUNTS, TAIL_ARM, WING,
                                      WING_AREA, WING_CHORD, _aero_cda, _aero_inflow, _prop_drag,
                                      drive_loads, mjcf, row_from_payload, surface_angles)
from machine.aerobat.law import Core
from machine.aerobat.sky import Sky
