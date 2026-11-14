; Shared aerodynamic defaults (`aerodynamics.lisp`); craft blocks come from `aircraft/crafts.toml`
; via the static meta-compiler (`machine.aircraft.compile`).

(defaerodynamics shared
  (use standard_airframe))
