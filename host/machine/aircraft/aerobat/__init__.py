"""The unlimited 3D RC aerobatic monoplane in MuJoCo: prop-hang, big surfaces, stunts.

    out = core.step(law, row, sky.state(), dt, reach); sky.step(out.speeds + out.controls, dt, air)

An Extra 300 (`aerobat.lisp`) on two 63100 drives, `PROP` the tractor propeller and `VECT`
the coaxial contra stage: propwash over oversized surfaces gives attitude authority from zero
airspeed. Its numbers, its wing and its XML (`airframe`), its form (`form`), its flight in
MuJoCo (`sky`), its law (`law`) and its observer (`observer`).
"""
from machine.aircraft.aerobat.airframe import (A_DISC, BEHAVIORS, BLADE_PITCH, BODY_CDA, CARD, CD0, CD_INDUCED,
                                      CL_ALPHA, COOLING, COURSE_SPHERE, FLOOR_M, FOOT_M, FORM,
                                      GEAR_MAIN_Z, GEAR_TAIL_Z, GRAVITY, HALF_M, IDLE_RAD_S, INERTIA,
                                      INFLOW_J0, K_DRAG, K_PITCH, K_PITCH_Q, K_ROLL, K_ROLL_Q,
                                      K_THRUST, K_YAW, LISP_PATH, MANIFEST, MASS_KG, PROP_BASE,
                                      PROP_LOAD, PROP_R, PROP_SURF, PROP_Z, PROPWASH_SHARE, PULL_MAX,
                                      RHO, ROTOR, RUNWAY_M, RUNWAY_RAD, SKID_M, SKID_X, STAB_SPAN,
                                      STREAMS, STRUT_Z, STUNTS, TAIL_ARM, W_TOP, WING, WING_AREA,
                                      WING_CHORD, _aero_cda, _aero_inflow, _prop_drag, drive_loads,
                                      mjcf, prop_speed, row_from_payload, slipstream, surface_angles)
from machine.aircraft.aerobat.law import Core
from machine.aircraft.aerobat.sky import Sky
