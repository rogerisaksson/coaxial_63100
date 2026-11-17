; The aerobat's flights (`aerobat.lisp` and its variants fly them): its streams, stunts and routine,
; its vehicle the text's that includes this one.

(stream_inject
  (sequence_id 501)
  (blend_duration 250ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch 0.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s)
    (mode_pitch_rate 0.0rad_s)
    (mode_roll_rate 0.0rad_s))
  (impedance
    (stiffness mode_collective_thrust 120.0)
    (damping mode_collective_thrust 25.0)
    (stiffness mode_attitude_pitch 270.0)
    (damping mode_attitude_pitch 22.0)
    (stiffness mode_attitude_roll 275.0)
    (damping mode_attitude_roll 22.0)
    (stiffness mode_yaw_rate 150.0)
    (damping mode_yaw_rate 16.0)
    (stiffness mode_pitch_rate 45.0)
    (damping mode_pitch_rate 8.0)
    (stiffness mode_roll_rate 45.0)
    (damping mode_roll_rate 8.0))
  (cpg_config
    (enable false))
  (task_constraints
    (maintain_altitude 1.5m)))

(stream_inject
  (sequence_id 502)
  (blend_duration 200ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch -24.0deg)
    (mode_attitude_roll 22.0deg)
    (mode_yaw_rate -1.5rad_s))
  (task_constraints
    (maintain_altitude 1.5m)))

(stream_inject
  (sequence_id 503)
  (blend_duration 120ms)
  (latent_targets
    (mode_collective_thrust INVERTED_THRUST)
    (mode_attitude_pitch 180.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 504)
  (blend_duration 80ms)
  (latent_targets
    (mode_collective_thrust 0.0)
    (mode_attitude_pitch 90.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (cpg_config
    (enable true)
    (frequency 1.25Hz)
    (amplitude mode_attitude_pitch 54.0deg)
    (amplitude mode_collective_thrust 24.0)
    (phase_offset mode_collective_thrust mode_attitude_pitch 90.0deg))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 505)
  (blend_duration 80ms)
  (latent_targets
    (mode_collective_thrust 0.0)
    (mode_yaw_rate 6.2831853rad_s)
    (mode_pitch_rate -6.2831853rad_s))
  (cpg_config
    (enable true)
    (frequency 1.0Hz)
    (amplitude mode_attitude_pitch 180.0deg)
    (amplitude mode_attitude_roll 180.0deg))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 506)
  (blend_duration 40ms)
  (latent_targets
    (mode_collective_thrust MAX_SYS_THRUST)
    (mode_attitude_pitch 0.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s)
    (mode_pitch_rate 0.0rad_s)
    (mode_roll_rate 0.0rad_s))
  (cpg_config
    (enable false))
  (task_constraints
    (limit_vertical_ascent 12.0m)))

(defstunt hover
  (base hover) (height 0.9m))

(defstunt high
  (base high) (height 6.0m))

(defstunt torque_roll
  (base hover) (turn 180.0deg_s))

(defstunt knife_circle
  (base orbit) (height 0.9m) (slide 2.0m_s) (turn FUNNEL_TURN_DEG_S))

(defstunt climbing_harrier
  (base knife_circle) (height 4.5m) (pace 2.0m_s))

(defstunt waterfall
  (base high) (nose -2.0))

(defstunt knife_pass
  (base hover) (height 0.46m) (pace 5.0m_s) (speed 20.0m_s)
  (knife -1.0) (lean 147.15m_s2) (home 0.0) (lined 0.0))

(defstunt rolling_harrier
  (base hover) (height 1.25m) (pace 5.0m_s) (speed 9.5m_s)
  (roll 0.70) (lean 147.15m_s2) (home 0.0) (lined 0.0))

(defstunt unwind
  (base unwind) (height 0.9m) (pace 2.5m_s) (slide -2.0m_s))

(defstunt takeoff
  (base hover) (height 1.0m) (pace 4.0m_s) (speed 12.0m_s) (home 0.0))

(defstunt parachute
  (base stop) (height 0.74m) (pace 4.0m_s))

(defroutine aerobat_3d
  (stage idle (stunt down) (duration 2.0s) (ramp 0.0s) (gate fit))
  (stage spool (stunt pressed) (duration 2.5s) (ramp 2.5s))
  (stage lift (stunt takeoff) (duration 3.6s) (ramp 1.0s))
  (stage prop-hang (stunt hover) (duration 2.5s) (ramp 2.0s) (gate held))
  (stage torque-roll (stunt torque_roll) (duration 2.0s) (ramp 1.0s))
  (stage poise (stunt hover) (duration 1.8s) (ramp 1.5s))
  (stage harrier (stunt climbing_harrier) (duration LAP_S) (ramp 1.5s))
  (stage home (stunt high) (duration 1.8s) (ramp 1.2s) (gate held))
  (stage waterfall (stunt waterfall) (duration 0.8s) (ramp 0.0s))
  (stage catch (stunt high) (duration 2.2s) (ramp 0.0s))
  (stage snap-roll (stunt roll) (duration 0.8s) (ramp 0.0s))
  (stage catch (stunt high) (duration 2.2s) (ramp 0.0s))
  (stage unwind (stunt unwind) (duration LAP_S) (ramp 1.5s))
  (stage level (stunt hover) (duration 2.0s) (ramp 1.5s) (gate held))
  (stage full_tilt (stunt sky) (duration 1.2s) (ramp 0.0s))
  (stage burn (stunt parachute) (duration 0.0s) (ramp 0.0s) (gate held))
  (stage hold (stunt parachute) (duration 1.5s) (ramp 0.0s))
  (stage land (stunt pressed) (duration 8.0s) (ramp 0.0s)))
