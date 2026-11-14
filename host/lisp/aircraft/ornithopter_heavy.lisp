; The ornithopter on a 2.5 kg frame (`crafts.toml` `[ornithopter_heavy]`, compiled by `aircraft.compile`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

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
