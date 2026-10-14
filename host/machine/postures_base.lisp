;; ============================================================================
;; ATHLETIC & CALISTHENIC GYM POSTURES (machine/postures_base.lisp)
;; Balance, jump, and ballistic gait initiation archetypes.
;; ============================================================================

(extends "gait_base.lisp")

;; Soft-knee landing absorption spring prototype
(deftrait soft_land
  (latent_targets (mode_axial_compression 0.25) (mode_sagittal_shift 0.0) (mode_pitch_attitude 0.0))
  (impedance
    (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
    (stiffness mode_sagittal_shift 600) (damping mode_sagittal_shift 300)
    (stiffness mode_pitch_attitude 1458) (damping mode_pitch_attitude 292)))

;; 8: crouch
(defbehavior crouch (extends stance_hold)
  (phase down
    (duration 2s) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.6))
    (impedance (stiffness mode_axial_compression 8000) (damping mode_axial_compression 1200))))

;; 9: hop: loaded, thrown by a spring that lets go 17 mm under her standing height, her
;; trunk free through the throw, landed on soft knees
(defbehavior hop
  (phase load
    (duration 600ms) (blend_duration 450ms)
    (latent_targets (mode_axial_compression 0.85) (mode_sagittal_shift -0.1))
    (impedance
      (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900)
      (stiffness mode_sagittal_shift 1200) (damping mode_sagittal_shift 400))
    (cpg_override (halt_oscillators)))
  (phase throw
    (duration 150ms)
    (latent_targets (mode_axial_compression 0.15))
    (impedance
      (stiffness mode_axial_compression 20000) (damping mode_axial_compression 0)
      (stiffness mode_pitch_attitude 0) (damping mode_pitch_attitude 200)))
  (phase land (use soft_land)
    (duration 700ms)))

;; 0: on one sole, the other off the floor: it leaves once her weight is over
(defbehavior one_sole (extends stance_hold)
  (phase over
    (duration 4s) (blend_duration 800ms)
    (latent_targets (mode_lateral_shift 0.92) (mode_limb_lift -1.0) (mode_pelvis_list 0.0))
    (impedance
      (stiffness mode_axial_compression 6000) (damping mode_axial_compression 800)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600)
      (stiffness mode_pelvis_list 1500) (damping mode_pelvis_list 300))))

;; her weight over one sole
(defbehavior weight_shift (extends stance_hold)
  (phase over
    (duration 3s) (blend_duration 500ms)
    (latent_targets (mode_lateral_shift 1.0))
    (impedance (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600))))

(defbehavior bow (extends stance_hold)
  (phase forward
    (duration 2s) (blend_duration 500ms)
    (latent_targets (mode_pitch_attitude 0.5))))

;; ballistic walk from rest: crisp APA unweighting directly into stride
(defbehavior ballistic_walk
  (phase apa (use forward_gait)
    (duration 580ms) (blend_duration 300ms)
    (latent_targets (mode_axial_compression 0.065) (mode_pitch_attitude 0.025))
    (cpg_config
      (frequency 0.94Hz)
      (amplitude mode_lateral_shift 0.30) (amplitude mode_sagittal_drive 0.46)
      (amplitude mode_limb_lift 1.02) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.40) (amplitude mode_pelvis_turn 0.40)
      (amplitude mode_arm_swing 0.85)))
  (phase stride (use forward_rhythm)
    (duration 60s) (blend_duration 550ms)
    (latent_targets (mode_axial_compression 0.065) (mode_pitch_attitude 0.03))
    (cpg_config
      (frequency 0.98Hz)
      (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.70)
      (amplitude mode_limb_lift 1.08) (amplitude mode_foot_rock 0.65)
      (amplitude mode_pelvis_list 0.38) (amplitude mode_pelvis_turn 0.40)
      (amplitude mode_arm_swing 0.95))))

;; smooth initiation into terrain gaits: single APA step before full stride
(defbehavior init_walk
  (phase init (use stride_spring)
    (duration 600ms) (blend_duration 300ms)
    (latent_targets (mode_axial_compression 0.065) (mode_pitch_attitude 0.015))
    (impedance (stiffness mode_axial_compression 3200))
    (cpg_config
      (enable true) (frequency 0.90Hz)
      (amplitude mode_lateral_shift 0.30) (amplitude mode_sagittal_drive 0.44)
      (amplitude mode_limb_lift 1.02) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.36) (amplitude mode_pelvis_turn 0.38)
      (amplitude mode_arm_swing 0.82)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))))


