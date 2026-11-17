; The helicopter's flights (`helicopter.lisp` and its variants fly them): its streams, stunts and routine,
; its vehicle the text's that includes this one.

(stream_inject
  (sequence_id 301)
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
    (stiffness mode_attitude_pitch 260.0)
    (damping mode_attitude_pitch 22.0)
    (stiffness mode_attitude_roll 260.0)
    (damping mode_attitude_roll 22.0)
    (stiffness mode_yaw_rate 140.0)
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
  (sequence_id 302)
  (blend_duration 200ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch -22.0deg)
    (mode_attitude_roll 18.0deg)
    (mode_yaw_rate -1.5rad_s))
  (task_constraints
    (maintain_altitude 1.5m)))

(stream_inject
  (sequence_id 303)
  (blend_duration 120ms)
  (latent_targets
    (mode_collective_thrust INVERTED_THRUST)
    (mode_attitude_pitch 180.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 304)
  (blend_duration 80ms)
  (latent_targets
    (mode_collective_thrust 0.0)
    (mode_attitude_pitch 90.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (cpg_config
    (enable true)
    (frequency 1.25Hz)
    (amplitude mode_attitude_pitch 52.0deg)
    (amplitude mode_collective_thrust 24.0)
    (phase_offset mode_collective_thrust mode_attitude_pitch 90.0deg))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 305)
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
  (sequence_id 306)
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

(defstunt pirouette
  (base hover) (turn 180.0deg_s))

(defstunt funnel
  (base orbit) (height 0.4m) (slide 3.0m_s) (turn FUNNEL_TURN_DEG_S))

(defstunt spiral
  (base funnel) (height 1.5m) (pace 2.0m_s))

(defstunt flip_in
  (base high) (flip -0.5) (nose -1.0))

(defstunt inverted
  (base high) (nose -1.0) (height 0.2m))

(defstunt inv_piro
  (base inverted) (turn 240.0deg_s) (height 0.95m))

(defstunt tictoc
  (base high) (nose -2.0) (height 0.95m))

(defstunt piro_flip
  (base high) (flip -1.0) (turn 360.0deg_s) (height 0.95m))

(defroutine helicopter_3d
  (stage idle (stunt down) (duration 2.0s) (ramp 0.0s) (gate fit))
  (stage spool (stunt pressed) (duration 2.5s) (ramp 2.5s))
  (stage lift (stunt launch) (duration 3.0s) (ramp 0.0s))
  (stage hover (stunt hover) (duration 2.5s) (ramp 0.0s))
  (stage pirouette (stunt pirouette) (duration 2.0s) (ramp 1.0s))
  (stage poise (stunt hover) (duration 1.8s) (ramp 1.5s))
  (stage funnel (stunt funnel) (duration LAP_S) (ramp 1.5s))
  (stage spiral (stunt spiral) (duration LAP_S) (ramp 1.5s))
  (stage home (stunt high) (duration 1.8s) (ramp 1.2s) (gate held))
  (stage flip (stunt flip_in) (duration 1.0s) (ramp 0.0s))
  (stage inverted (stunt inverted) (duration 2.2s) (ramp 0.0s))
  (stage piro-inv (stunt inv_piro) (duration 1.5s) (ramp 0.0s))
  (stage tic-toc (stunt tictoc) (duration 2.0s) (ramp 0.0s))
  (stage piro-flip (stunt piro_flip) (duration 1.0s) (ramp 0.0s))
  (stage catch (stunt high) (duration 2.2s) (ramp 0.0s))
  (stage roll (stunt roll) (duration 0.8s) (ramp 0.0s))
  (stage catch (stunt high) (duration 2.2s) (ramp 0.0s))
  (stage unwind (stunt unwind) (duration LAP_S) (ramp 1.5s))
  (stage level (stunt hover) (duration 2.0s) (ramp 1.5s) (gate held))
  (stage full_tilt (stunt sky) (duration 1.4s) (ramp 0.0s))
  (stage burn (stunt stop) (duration 0.0s) (ramp 0.0s) (gate held))
  (stage hold (stunt stop) (duration 1.5s) (ramp 0.0s))
  (stage descend (stunt over) (duration 1.5s) (ramp 1.5s) (gate held))
  (stage land (stunt pressed) (duration 3.0s) (ramp 3.0s)))
