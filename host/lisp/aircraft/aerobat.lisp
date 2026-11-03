; Unlimited 3D RC aerobatic monoplane puzzled together from general Lisp blocks (`aerobat.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle aerobat_3d
  (use monoplane_airframe)
  (use unlimited_3d_monoplane)
  (use aerobat_prop_load)
  (use aerobat_monoplane_body)
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
