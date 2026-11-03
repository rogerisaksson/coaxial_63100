; The quad on 15 in props for its 16s, their pitch with their diameter (a 15x13.1): its thrust at a speed by the
; diameter's fourth power, its drag by its fifth, its disc and its pitch's speed by the first - its other blocks
; `quadcopter.lisp`'s, its flights its kind's (`flights/quadcopter.lisp`), nothing of them tuned to it
; (tools/sim/flights.py flies it).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defblock rotors quad_x_15x13
  (use quad_x_16x14)
  (disc_radius 0.1905m)
  (disc_half_height 0.0075m)
  (k_thrust 8.49338e-05)
  (k_drag 2.99528e-06)
  (blade_pitch 0.3334m))

(defvehicle quadcopter_15in
  (use standard_airframe)
  (use quad_x_15x13)
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
