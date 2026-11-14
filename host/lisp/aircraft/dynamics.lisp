; Shared rigid-body, cooling, governor, impedance and observer defaults (`dynamics.lisp`);
; craft blocks come from `aircraft/crafts.toml` and `library/*.toml` via `machine.aircraft.compile`.

(defdynamics shared
  (use base_airframe_2kg)
  (use finned_aluminium_fan_sink)
  (use base_governor)
  (use base_attitude_impedance)
  (use eso_3d_observers))
