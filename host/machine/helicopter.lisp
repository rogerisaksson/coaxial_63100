; 3D aerobatic helicopter puzzled together from general Lisp blocks (`helicopter.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle helicopter_3d
  (use standard_airframe)
  (use swashplate_500_3d)
  (use tail_150_antitorque)
  (use swashplate_tail_load)
  (use heli_pod_boom_body)
  (use finned_aluminium_fan_sink)
  (use heli_3d_governor)
  (use heli_3d_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use heli_tictoc_oscillator)
  (use hold_tolerances))

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
  (base orbit) (height 1.5m) (slide 3.0m_s) (turn FUNNEL_TURN_DEG_S))

(defstunt spiral
  (base funnel) (height 8.0m) (pace 2.0m_s))

(defstunt flip_in
  (base high) (flip -0.5) (nose -1.0))

(defstunt inverted
  (base high) (nose -1.0))

(defstunt inv_piro
  (base inverted) (turn 240.0deg_s))

(defstunt tictoc
  (base high) (nose -2.0))

(defstunt aileron_tictoc
  (base high) (nose -3.0))

(defstunt piro_flip
  (base high) (flip -1.0) (turn 360.0deg_s))

(defstunt chaos
  (base high) (flip -1.0) (roll 1.0) (turn 540.0deg_s))

(defstunt hurricane
  (base funnel) (height 6.0m) (pace 2.5m_s) (slide -3.0m_s) (turn UNWIND_TURN_DEG_S))

(defbehavior tareq_smackdown
  (phase punch (stunt high) (duration 1.2s) (blend 100ms))
  (phase flip_entry (stunt flip_in) (duration 1.0s) (blend 80ms))
  (phase elevator_tictoc (stunt tictoc) (duration 2.0s) (blend 80ms))
  (phase chaos_burn (stunt chaos) (duration 1.2s) (blend 100ms))
  (phase catch_high (stunt high) (duration 1.6s) (blend 150ms) (gate held)))

(defbehavior szabo_piro_routine
  (phase inverted_entry (stunt flip_in) (duration 1.0s) (blend 100ms))
  (phase inverted_poise (stunt inverted) (duration 1.6s) (blend 120ms))
  (phase inverted_pirouette (stunt inv_piro) (duration 1.8s) (blend 100ms))
  (phase piro_flip_out (stunt piro_flip) (duration 1.0s) (blend 80ms))
  (phase recovery (stunt high) (duration 1.8s) (blend 150ms) (gate held)))

(defbehavior aileron_rainbow
  (phase climb_entry (stunt high) (duration 1.2s) (blend 120ms))
  (phase knife_tictoc (stunt aileron_tictoc) (duration 2.2s) (blend 80ms))
  (phase snap_roll (stunt roll) (duration 0.8s) (blend 80ms))
  (phase settle (stunt high) (duration 1.6s) (blend 150ms) (gate held)))

(defbehavior funnel_to_spiral
  (phase carve_funnel (stunt funnel) (duration LAP_S) (blend 200ms))
  (phase climbing_spiral (stunt spiral) (duration LAP_S) (blend 200ms))
  (phase hurricane_back (stunt hurricane) (duration 2.5s) (blend 180ms))
  (phase unwind_home (stunt unwind) (duration LAP_S) (blend 200ms)))

(defbehavior inverted_pirouette_circuit
  (phase flip_over (stunt flip_in) (duration 1.0s) (blend 100ms))
  (phase inverted_hold (stunt inverted) (duration 1.5s) (blend 100ms))
  (phase inverted_spin (stunt inv_piro) (duration 2.0s) (blend 100ms))
  (phase flip_upright (stunt piro_flip) (duration 1.0s) (blend 100ms))
  (phase lock_high (stunt high) (duration 1.5s) (blend 150ms) (gate held)))

(defroutine helicopter_3d
  (stage idle (stunt down) (duration 2.0s) (ramp 0.0s) (gate fit))
  (stage spool (stunt pressed) (duration 2.5s) (ramp 2.5s))
  (stage lift (stunt hover) (duration 3.0s) (ramp 3.0s))
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
  (stage full_tilt (stunt sky) (duration 2.0s) (ramp 0.0s))
  (stage burn (stunt stop) (duration 0.0s) (ramp 0.0s) (gate held))
  (stage hold (stunt stop) (duration 1.5s) (ramp 0.0s))
  (stage descend (stunt over) (duration 1.5s) (ramp 1.5s) (gate held))
  (stage land (stunt pressed) (duration 3.0s) (ramp 3.0s)))
