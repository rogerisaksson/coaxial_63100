; 3D aerobatic helicopter puzzled together from general Lisp blocks (`helicopter.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle helicopter_3d
  (use standard_airframe)
  (use swashplate_550_3d)
  (use tail_110_antitorque)
  (use swashplate_tail_load)
  (use heli_pod_boom_body)
  (use finned_aluminium_fan_sink)
  (use heli_3d_governor)
  (use heli_3d_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use heli_tictoc_oscillator)
  (use hold_tolerances))

(include "flights/helicopter.lisp")
