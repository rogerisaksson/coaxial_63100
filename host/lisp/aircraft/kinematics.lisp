; Shared kinematic envelope, trajectory, oscillator and stunt primitives (`kinematics.lisp`);
; kinematic blocks come from `aircraft/crafts.toml` via `machine.aircraft.compile`.

(defkinematics shared
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use base_tictoc_oscillator)
  (use hold_tolerances))

(defstunt hover
  (height 0.3m) (pace 1.5m_s) (climb 0.0m_s) (push 0.0m_s2)
  (speed 0.0m_s) (slide 0.0m_s) (surge 0.0m_s2) (sway 0.0m_s2)
  (turn 0.0deg_s) (nose 1.0) (x 0.0m) (z 0.0m) (home 1.0) (rail 0.0)
  (lean 8.0m_s2) (roll 0.0) (flip 0.0) (light 0.5) (wing 1.0)
  (plan 0.0) (skim 0.0) (knife 0.0) (lined 1.0))

(defstunt sky
  (base hover) (height 1000.0m) (pace 100.0m_s))

(defstunt stop
  (base hover) (height 0.1m) (pace 100.0m_s))

(defstunt launch
  (base hover) (pace 100.0m_s))

(defstunt down
  (base stop) (height -0.5m))

(defstunt pressed
  (base hover) (height -0.02m))

(defstunt over
  (base hover) (height 0.1m) (pace 2.0m_s))

(defstunt orbit
  (base hover) (height 0.45m) (slide 3.0m_s) (turn FUNNEL_TURN_DEG_S) (home 0.0))

(defstunt high
  (base hover) (height 0.75m) (pace 4.0m_s))

(defstunt roll
  (base high) (roll 1.25))

(defstunt unwind
  (base orbit) (height 0.45m) (pace 1.5m_s) (slide -3.0m_s) (turn UNWIND_TURN_DEG_S))
