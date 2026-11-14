; The aerobat on a 2.3 kg frame (`crafts.toml` `[aerobat_heavy]`, compiled by `aircraft.compile`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle aerobat_3d_heavy
  (use monoplane_airframe)
  (use unlimited_3d_monoplane)
  (use aerobat_prop_load)
  (use aerobat_monoplane_body_heavy)
  (use finned_aluminium_fan_sink)
  (use aerobat_3d_governor)
  (use aerobat_3d_impedance)
  (use eso_3d_observers)
  (use aerobat_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use aerobat_harrier_oscillator)
  (use hold_tolerances))

(include "flights/aerobat.lisp")
