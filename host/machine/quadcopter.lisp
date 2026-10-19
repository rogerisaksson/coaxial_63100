; Quadcopter puzzled together from general Lisp blocks (`quadcopter.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle quadcopter
  (use standard_airframe)
  (use quad_x_16x14)
  (use quad_x_body)
  (use finned_aluminium_fan_sink)
  (use quad_direct_governor)
  (use quad_attitude_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use base_tictoc_oscillator)
  (use hold_tolerances))

(stream_inject
  (sequence_id 101)
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
    (stiffness mode_attitude_pitch 300.0)
    (damping mode_attitude_pitch 17.0)
    (stiffness mode_attitude_roll 300.0)
    (damping mode_attitude_roll 17.0)
    (damping mode_yaw_rate 4.0)
    (damping mode_pitch_rate 17.0)
    (damping mode_roll_rate 17.0))
  (cpg_config
    (enable false))
  (task_constraints
    (maintain_altitude 1.5m)))

(stream_inject
  (sequence_id 102)
  (blend_duration 200ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch -15.0deg)
    (mode_attitude_roll 25.0deg)
    (mode_yaw_rate -1.5rad_s))
  (task_constraints
    (maintain_altitude 2.0m)))

(stream_inject
  (sequence_id 103)
  (blend_duration 20ms)
  (latent_targets
    (mode_collective_thrust MAX_SYS_THRUST)
    (mode_attitude_pitch 0.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (task_constraints
    (limit_vertical_ascent 8.0m)))

(stream_inject
  (sequence_id 104)
  (blend_duration 50ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch 0.0deg)
    (mode_attitude_roll ignore)
    (mode_roll_rate -7.8539816rad_s))
  (impedance
    (stiffness mode_roll_rate 201.0)
    (damping mode_roll_rate 17.0))
  (task_constraints
    (maintain_altitude 8.0m)))

(defstunt pirouette
  (base hover) (turn 120.0deg_s))

(defstunt corkscrew
  (base orbit) (height 4.0m) (pace 2.0m_s))

(defstunt toss
  (base sky) (pace 5.0m_s))

(defstunt flip
  (base high) (flip -1.25))

(defbehavior corkscrew_climb
  (phase enter_orbit (stunt orbit) (duration LAP_S) (blend 200ms))
  (phase spiral_up (stunt corkscrew) (duration LAP_S) (blend 200ms))
  (phase settle_top (stunt high) (duration 2.0s) (blend 150ms) (gate held)))

(defbehavior toss_roll_flip
  (phase toss_up (stunt toss) (duration 1.0s) (blend 50ms))
  (phase snap_roll (stunt roll) (duration 0.8s) (blend 50ms))
  (phase catch_roll (stunt high) (duration 3.0s) (blend 150ms))
  (phase toss_again (stunt toss) (duration 1.0s) (blend 50ms))
  (phase snap_flip (stunt flip) (duration 0.8s) (blend 50ms))
  (phase catch_flip (stunt high) (duration 3.0s) (blend 150ms) (gate held)))

(defbehavior sky_punch_stop
  (phase full_tilt (stunt sky) (duration 2.0s) (blend 50ms))
  (phase burn_stop (stunt stop) (duration 1.5s) (blend 50ms) (gate held))
  (phase descend_over (stunt over) (duration 1.5s) (blend 150ms) (gate held)))

(defroutine quadcopter
  (stage idle (stunt down) (duration 2.0s) (ramp 0.0s) (gate fit))
  (stage spool (stunt pressed) (duration 2.5s) (ramp 2.5s))
  (stage lift (stunt launch) (duration 3.0s) (ramp 0.0s))
  (stage hover (stunt hover) (duration 3.0s) (ramp 0.0s))
  (stage pirouette (stunt pirouette) (duration 3.0s) (ramp 2.0s))
  (stage poise (stunt hover) (duration 2.0s) (ramp 2.0s))
  (stage orbit (stunt orbit) (duration LAP_S) (ramp 1.5s))
  (stage corkscrew (stunt corkscrew) (duration LAP_S) (ramp 1.5s))
  (stage home (stunt high) (duration 2.0s) (ramp 1.5s) (gate held))
  (stage toss (stunt toss) (duration 1.0s) (ramp 0.0s))
  (stage roll (stunt roll) (duration 0.8s) (ramp 0.0s))
  (stage catch (stunt high) (duration 3.0s) (ramp 0.0s))
  (stage toss (stunt toss) (duration 1.0s) (ramp 0.0s))
  (stage flip (stunt flip) (duration 0.8s) (ramp 0.0s))
  (stage catch (stunt high) (duration 3.0s) (ramp 0.0s))
  (stage unwind (stunt unwind) (duration LAP_S) (ramp 1.5s))
  (stage level (stunt hover) (duration 2.0s) (ramp 1.5s) (gate held))
  (stage full_tilt (stunt sky) (duration 0.45s) (ramp 0.0s))
  (stage burn (stunt stop) (duration 0.0s) (ramp 0.0s) (gate held))
  (stage hold (stunt stop) (duration 1.5s) (ramp 0.0s))
  (stage descend (stunt over) (duration 1.5s) (ramp 1.5s) (gate held))
  (stage land (stunt pressed) (duration 3.0s) (ramp 3.0s)))
