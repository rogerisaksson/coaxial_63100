; The ornithopter on a body a quarter heavier, 2.5 kg, its inertia with its mass: its other blocks `ornithopter.lisp`'s,
; its flights its kind's (`flights/ornithopter.lisp`), nothing of them tuned to it (tools/sim/flights.py flies it).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defblock rigid_body ornithopter_carapace_body_heavy
  (use ornithopter_carapace_body)
  (mass 2.5kg)
  (inertia_xx 0.03)
  (inertia_yy 0.04)
  (inertia_zz 0.0225))

(defvehicle ornithopter_dune_heavy
  (use standard_airframe)
  (use dune_4wing_flapper)
  (use ornithopter_wing_load)
  (use ornithopter_carapace_body_heavy)
  (use finned_aluminium_fan_sink)
  (use ornithopter_governor)
  (use ornithopter_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use ornithopter_flap_oscillator)
  (use hold_tolerances))

(include "flights/ornithopter.lisp")
