; Quadcopter puzzled together from general Lisp blocks (`quadcopter.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle quadcopter
  (use standard_airframe)
  (use quad_x_16x14)
  (use quad_x_body)
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
