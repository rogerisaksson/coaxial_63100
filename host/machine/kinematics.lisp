; General kinematic envelope, trajectory, oscillator, and stunt blocks (`kinematics.lisp`).

(defblock envelope standard_arena_envelope
  (floor_height 0.1m)
  (hover_height 1.5m)
  (high_height 4.0m)
  (top_height 4.0m)
  (least_pull 1.5)
  (hover_lean 8.0m_s2)
  (sky_lean 8.0m_s2))

(defblock envelope aerobat_envelope
  (use standard_arena_envelope)
  (floor_height 0.74m)
  (high_height 14.0m)
  (top_height 14.0m))

(defblock vertical vertical_climb_brake
  (climb_speed 4.0m_s)
  (descent_speed 2.0m_s)
  (settle_kp 6.4)
  (settle_kd 12.8))

(defblock horizontal orbit_circle
  (orbit_radius 2.0m)
  (orbit_speed 3.0m_s)
  (lap_duration 4.1887902047863905s)
  (spot_kp 1.95)
  (spot_kd 2.94))

(defblock oscillator base_tictoc_oscillator
  (tictoc_frequency 1.25Hz)
  (tictoc_amplitude 45.0deg))

(defblock oscillator heli_tictoc_oscillator
  (use base_tictoc_oscillator)
  (tictoc_amplitude 52.0deg))

(defblock oscillator ornithopter_flap_oscillator
  (use base_tictoc_oscillator)
  (tictoc_frequency 1.35Hz)
  (tictoc_amplitude 48.0deg))

(defblock oscillator aerobat_harrier_oscillator
  (use base_tictoc_oscillator)
  (tictoc_frequency 1.25Hz)
  (tictoc_amplitude 54.0deg))

(defblock hold hold_tolerances
  (height_tol 0.03m)
  (speed_tol 0.15m_s)
  (spot_tol 0.15m)
  (tilt_tol 0.5rad))

(defkinematics shared
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use base_tictoc_oscillator)
  (use hold_tolerances))

(defstunt hover
  (height 1.5m) (pace 1.5m_s) (climb 0.0m_s) (push 0.0m_s2)
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
  (base hover) (height 2.0m) (slide 3.0m_s) (turn FUNNEL_TURN_DEG_S) (home 0.0))

(defstunt high
  (base hover) (height 4.0m) (pace 4.0m_s))

(defstunt roll
  (base high) (roll 1.25))

(defstunt unwind
  (base orbit) (height 1.5m) (pace 1.5m_s) (slide -3.0m_s) (turn UNWIND_TURN_DEG_S))
