; The quad on a body a quarter heavier, 2.5 kg, its inertia with its mass: its other blocks `quadcopter.lisp`'s,
; its flights its kind's (`flights/quadcopter.lisp`), nothing of them tuned to it (tools/sim/flights.py flies it).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defblock rigid_body quad_x_body_heavy
  (use quad_x_body)
  (mass 2.5kg))

(defvehicle quadcopter_heavy
  (use standard_airframe)
  (use quad_x_16x14)
  (use quad_x_body_heavy)
  (use finned_aluminium_fan_sink)
  (use quad_direct_governor)
  (use quad_attitude_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use base_tictoc_oscillator)
  (use hold_tolerances))

(include "flights/quadcopter.lisp")
