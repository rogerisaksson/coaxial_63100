; The helicopter on a body a quarter heavier, 2.5 kg, its inertia with its mass: its other blocks `helicopter.lisp`'s,
; its flights its kind's (`flights/helicopter.lisp`), nothing of them tuned to it (tools/sim/flights.py flies it).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defblock rigid_body heli_pod_boom_body_heavy
  (use heli_pod_boom_body)
  (mass 2.5kg)
  (inertia_xx 0.0425)
  (inertia_yy 0.05875)
  (inertia_zz 0.02875))

(defvehicle helicopter_3d_heavy
  (use standard_airframe)
  (use swashplate_550_3d)
  (use tail_110_antitorque)
  (use swashplate_tail_load)
  (use heli_pod_boom_body_heavy)
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
