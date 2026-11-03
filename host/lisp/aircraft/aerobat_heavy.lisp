; The aerobat on a body 15 percent heavier, 2.3 kg, its inertia with its mass: its other blocks `aerobat.lisp`'s,
; its flights its kind's (`flights/aerobat.lisp`), nothing of them tuned to it (tools/sim/flights.py flies it).
; At 2.5 kg - on its own 17 in prop, and on the 18 in its weight takes (its thrust a quarter more) - its knife
; pass sank at 2.1-2.5 m/s and laid its lower tip on the runway at 15 s: its wing and fuselage hold that figure;
; at 2.2 and 2.3 kg it flew the whole card.

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defblock rigid_body aerobat_monoplane_body_heavy
  (use aerobat_monoplane_body)
  (mass 2.3kg)
  (inertia_xx 0.0805)
  (inertia_yy 0.0529)
  (inertia_zz 0.10695))

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
