;; ============================================================================
;; BASE GAIT TRAITS & CORE STRIDE PROGRESSIONS (machine/gait_base.lisp)
;; Reusable orthokinematic trait prototypes and foundational stride streams.
;; ============================================================================

;; Static support-polygon posture with CPG halted
(deftrait stance_hold
  (cpg_override (halt_oscillators))
  (task_constraints (maintain_zmp (polygon support_polygon))))

;; Standard compliant leg and foot-rock springs for dynamic stepping
(deftrait stride_spring
  (impedance
    (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
    (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
    (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
    (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
  (task_constraints (maintain_zmp (polygon dynamic_footprint))))

;; Standard forward CPG phase offsets: sway & list 108 deg ahead of drive
(deftrait forward_rhythm
  (cpg_config
    (enable true)
    (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
    (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
    (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
    (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
    (phase_offset mode_foot_rock mode_sagittal_drive 0deg)))

;; Composite forward gait prototype: posture targets + springs + CPG phase coupling
(deftrait forward_gait (extends stride_spring forward_rhythm)
  (latent_targets
    (mode_axial_compression 0.04)
    (mode_pitch_attitude 0.0)
    (mode_sagittal_shift 0.0)))

;; Composite reverse / lateral gait prototype
(deftrait reverse_gait (extends stride_spring)
  (latent_targets
    (mode_axial_compression 0.04)
    (mode_pitch_attitude 0.0)
    (mode_sagittal_shift 0.0)
    (mode_pelvis_turn 0.0)
    (mode_stance_width 0.0))
  (cpg_config
    (enable true)
    (frequency 0.85Hz)
    (amplitude mode_lateral_shift 0.28)
    (amplitude mode_sagittal_drive 0.2)
    (amplitude mode_limb_lift 0.95)
    (amplitude mode_foot_rock 0.0)
    (amplitude mode_pelvis_list 0.0)
    (amplitude mode_pelvis_turn 0.0)
    (amplitude mode_arm_swing 0.0)
    (amplitude mode_stance_width 0.0)
    (phase_offset mode_lateral_shift mode_sagittal_drive -108deg)))

;; Shared deceleration / braking phase prototype
(deftrait gait_brake
  (duration 1800ms)
  (blend_duration 500ms)
  (impedance (damping mode_lateral_shift 600) (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
  (cpg_config
    (frequency 0.85Hz)
    (amplitude mode_lateral_shift 0.12)
    (amplitude mode_sagittal_drive 0.0)
    (amplitude mode_limb_lift 0.7)
    (amplitude mode_arm_swing 0.0))
  (task_constraints (maintain_zmp (polygon dynamic_footprint))))

;; stand
(stream_inject
  (sequence_id 201)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0))
  (impedance
    (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
    (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
    (stiffness mode_sagittal_shift 300) (damping mode_sagittal_shift 150))
  (cpg_config (enable false)))

;; sway from sole to sole, no steps: her ZMP into each sole in turn
(stream_inject (use forward_rhythm)
  (sequence_id 202)
  (blend_duration 1500ms)
  (cpg_config
    (frequency 0.9Hz)
    (amplitude mode_lateral_shift 0.31)
    (amplitude mode_sagittal_drive 0.0)))

;; step: 0.12 m/s. Along her way she is damped to her cruise, held to no place
(stream_inject
  (sequence_id 203)
  (blend_duration 1500ms)
  (impedance
    (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 150)
    (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
  (cpg_config (amplitude mode_sagittal_drive 0.2) (amplitude mode_limb_lift 1.0))
  (task_constraints (maintain_zmp (polygon dynamic_footprint))))

;; walk on: 0.69 m/s. Her soles rock heel to toe, her pelvis lists and turns, her arms swing
(stream_inject
  (sequence_id 204)
  (blend_duration 2000ms)
  (latent_targets (mode_pitch_attitude 0.0))
  (cpg_config
    (amplitude mode_sagittal_drive 0.6)
    (amplitude mode_foot_rock 0.6)
    (amplitude mode_pelvis_list 0.4)
    (amplitude mode_pelvis_turn 0.4)
    (amplitude mode_arm_swing 0.8)))

;; back: the same rhythm, her sway and her list the other side of her drive
(stream_inject (use reverse_gait)
  (sequence_id 205)
  (blend_duration 1500ms))

;; 6: trip: quick tripping small steps with stiff arms and flat soles
(defbehavior trip
  (phase mincing (use forward_gait)
    (duration 60s) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.03))
    (cpg_config
      (frequency 1.08Hz)
      (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.22)
      (amplitude mode_limb_lift 0.85) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.15) (amplitude mode_pelvis_turn 0.1)
      (amplitude mode_arm_swing 0.05)))
  (phase brake (use gait_brake)))

;; 7: jog: brisk high-cadence gait with deeper knee flexion and strong arm drive
(defbehavior jog
  (phase apa (use forward_gait)
    (duration 550ms) (blend_duration 280ms)
    (impedance (stiffness mode_axial_compression 3200))
    (cpg_config
      (frequency 0.94Hz)
      (amplitude mode_lateral_shift 0.30) (amplitude mode_sagittal_drive 0.60)
      (amplitude mode_limb_lift 1.04) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.42) (amplitude mode_pelvis_turn 0.42)
      (amplitude mode_arm_swing 0.95)))
  (phase run
    (duration 60s) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.045) (mode_pitch_attitude 0.015))
    (cpg_config
      (frequency 0.95Hz)
      (amplitude mode_sagittal_drive 0.68) (amplitude mode_limb_lift 1.04)
      (amplitude mode_foot_rock 0.64) (amplitude mode_pelvis_list 0.44)
      (amplitude mode_pelvis_turn 0.44) (amplitude mode_arm_swing 1.0)))
  (phase brake (use gait_brake)
    (blend_duration 700ms)
    (latent_targets (mode_axial_compression 0.02) (mode_pitch_attitude 0.0))
    (cpg_config
      (frequency 0.83Hz)
      (amplitude mode_limb_lift 0.8) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.15) (amplitude mode_pelvis_turn 0.1)
      (amplitude mode_arm_swing 0.1))))

