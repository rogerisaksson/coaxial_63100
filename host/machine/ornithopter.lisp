; Dune-style 4-wing flapping ornithopter puzzled together from general Lisp blocks (`ornithopter.lisp`).

(include "aerodynamics.lisp")
(include "dynamics.lisp")
(include "kinematics.lisp")

(defvehicle ornithopter_dune
  (use standard_airframe)
  (use dune_4wing_flapper)
  (use ornithopter_wing_load)
  (use ornithopter_carapace_body)
  (use finned_aluminium_fan_sink)
  (use ornithopter_governor)
  (use ornithopter_impedance)
  (use eso_3d_observers)
  (use standard_arena_envelope)
  (use vertical_climb_brake)
  (use orbit_circle)
  (use ornithopter_flap_oscillator)
  (use hold_tolerances))

(stream_inject
  (sequence_id 401)
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
    (stiffness mode_attitude_pitch 265.0)
    (damping mode_attitude_pitch 22.0)
    (stiffness mode_attitude_roll 265.0)
    (damping mode_attitude_roll 22.0)
    (stiffness mode_yaw_rate 145.0)
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
  (sequence_id 402)
  (blend_duration 200ms)
  (latent_targets
    (mode_collective_thrust HOVER_THRUST)
    (mode_attitude_pitch -24.0deg)
    (mode_attitude_roll 20.0deg)
    (mode_yaw_rate -1.5rad_s))
  (task_constraints
    (maintain_altitude 1.5m)))

(stream_inject
  (sequence_id 403)
  (blend_duration 120ms)
  (latent_targets
    (mode_collective_thrust INVERTED_THRUST)
    (mode_attitude_pitch 180.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 404)
  (blend_duration 80ms)
  (latent_targets
    (mode_collective_thrust 0.0)
    (mode_attitude_pitch 90.0deg)
    (mode_attitude_roll 0.0deg)
    (mode_yaw_rate 0.0rad_s))
  (cpg_config
    (enable true)
    (frequency 1.35Hz)
    (amplitude mode_attitude_pitch 48.0deg)
    (amplitude mode_collective_thrust 24.0)
    (phase_offset mode_collective_thrust mode_attitude_pitch 90.0deg))
  (task_constraints
    (maintain_altitude 8.0m)))

(stream_inject
  (sequence_id 405)
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
  (sequence_id 406)
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

(defstunt ridge_soar
  (base orbit) (height 1.5m) (slide 3.0m_s) (turn FUNNEL_TURN_DEG_S))

(defstunt thermal_spiral
  (base ridge_soar) (height 8.0m) (pace 2.0m_s))

(defstunt fold_entry
  (base high) (flip -0.5) (nose -1.0))

(defstunt sandstorm_dive
  (base high) (nose -1.0))

(defstunt dive_piro
  (base sandstorm_dive) (turn 240.0deg_s))

(defstunt wing_rock
  (base high) (nose -2.0))

(defstunt bank_rock
  (base high) (nose -3.0))

(defstunt dune_slash
  (base high) (flip -1.0) (turn 360.0deg_s))

(defstunt coriolis_storm
  (base high) (flip -1.0) (roll 1.0) (turn 540.0deg_s))

(defstunt desert_unwind
  (base ridge_soar) (height 6.0m) (pace 2.5m_s) (slide -3.0m_s) (turn UNWIND_TURN_DEG_S))

(defbehavior atreides_dive_attack
  (phase high_beat (stunt high) (duration 1.2s) (blend 100ms))
  (phase fold_wings (stunt fold_entry) (duration 1.0s) (blend 80ms))
  (phase stroke_rock (stunt wing_rock) (duration 2.0s) (blend 80ms))
  (phase coriolis_burn (stunt coriolis_storm) (duration 1.2s) (blend 100ms))
  (phase catch_high (stunt high) (duration 1.6s) (blend 150ms) (gate held)))

(defbehavior arrakis_ridge_soar
  (phase carve_ridge (stunt ridge_soar) (duration LAP_S) (blend 200ms))
  (phase rising_thermal (stunt thermal_spiral) (duration LAP_S) (blend 200ms))
  (phase desert_return (stunt desert_unwind) (duration 2.5s) (blend 180ms))
  (phase unwind_home (stunt unwind) (duration LAP_S) (blend 200ms)))

(defbehavior sandstorm_recovery
  (phase fold_over (stunt fold_entry) (duration 1.0s) (blend 100ms))
  (phase dive_hold (stunt sandstorm_dive) (duration 1.6s) (blend 120ms))
  (phase dive_pirouette (stunt dive_piro) (duration 1.8s) (blend 100ms))
  (phase slash_out (stunt dune_slash) (duration 1.0s) (blend 80ms))
  (phase recovery (stunt high) (duration 1.8s) (blend 150ms) (gate held)))

(defroutine ornithopter_dune
  (stage idle (stunt down) (duration 0.4s) (ramp 0.0s) (gate fit))
  (stage spool (stunt pressed) (duration 0.5s) (ramp 0.5s))
  (stage lift (stunt hover) (duration 1.8s) (ramp 1.8s))
  (stage hover (stunt hover) (duration 2.5s) (ramp 0.0s))
  (stage pirouette (stunt pirouette) (duration 2.0s) (ramp 1.0s))
  (stage poise (stunt hover) (duration 1.8s) (ramp 1.5s))
  (stage ridge (stunt ridge_soar) (duration LAP_S) (ramp 1.5s))
  (stage spiral (stunt thermal_spiral) (duration LAP_S) (ramp 1.5s))
  (stage home (stunt high) (duration 1.8s) (ramp 1.2s) (gate held))
  (stage fold (stunt fold_entry) (duration 1.0s) (ramp 0.0s))
  (stage dive (stunt sandstorm_dive) (duration 2.2s) (ramp 0.0s))
  (stage piro-dive (stunt dive_piro) (duration 1.5s) (ramp 0.0s))
  (stage wing-rock (stunt wing_rock) (duration 2.0s) (ramp 0.0s))
  (stage slash (stunt dune_slash) (duration 1.0s) (ramp 0.0s))
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
