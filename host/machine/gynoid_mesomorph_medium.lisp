;; ============================================================================
;; GYNOID MESOMORPH MEDIUM ORTHOKINEMATIC MANIFEST
;; (machine/gynoid_mesomorph_medium.lisp, machine/ortho.py)
;; Balance, rhythm and elasticity for the mesomorph-medium gynoid locomotion
;; and stunt repertoire, composed from modular base traits and archetypes.
;;
;; CLASSIFICATION SCHEMA (gynoid_<morphology>_<scale>):
;;   * gynoid_ectomorph_petite    - Small-framed and fine-boned (waif/gamine, < 165 cm);
;;                                  yields an extremely low limb moment of inertia, but
;;                                  minimal joint volume forces tendon-driven architectures
;;                                  or low-torque actuators (the Ava archetype).
;;   * gynoid_ectomorph_tall      - Slender, statuesque column form (> 175 cm); long lever
;;                                  arms provide extended reach but demand high structural
;;                                  flexural rigidity and strictly minimized distal mass at
;;                                  the knees and ankles.
;;   * gynoid_mesomorph_medium    - Anthropometric standard (160-168 cm); a pragmatic
;;                                  compromise between statistical human proportions and
;;                                  sufficient internal and external envelope so the
;;                                  structural chassis is robust rather than overly fragile
;;                                  ("inte allt för skraltigt"), accommodating all structural
;;                                  3D prints, actuators, harmonic drives, battery packs, TPU
;;                                  gel pads, and clothing. [ACTIVE CHASSIS: 165.0 cm,
;;                                  55.0 kg de Leva (1996), size 37-38 (24 cm) sole,
;;                                  U8/MN6007 + 1:30 gearbox B]
;;   * gynoid_hourglass_full      - Hyper-gynoid geometry (WHR <= 0.70); introduces a
;;                                  severe structural bottleneck for power buses and spinal
;;                                  actuators at the waist, while offering generous
;;                                  packaging volume for cells and high-torque hip
;;                                  actuators in the pelvis and thorax.
;;   * gynoid_athletic_heavy      - High-dynamic, muscular profile; expanded thigh,
;;                                  gluteal, and shoulder envelopes dictated by
;;                                  high-torque-density QDD actuators, structural stiffness
;;                                  under impact loads, and thermal dissipation
;;                                  requirements.
;;   * gynoid_skeletal_ultralight - Hollowed truss or exoskeleton chassis stripped of
;;                                  synthetic soft tissue; reduces feminine cues strictly
;;                                  to pelvic geometry, Q-angle, and lumbar lordosis to
;;                                  minimize total system mass.
;;
;; MODULAR COMPOSITION:
;;   [1] gait_base.lisp      - Base traits, core gaits & stride progressions (#201-#205, trip, jog)
;;   [2] postures_base.lisp  - Athletic & calisthenic gym (crouch, hop, one_sole, weight_shift, bow)
;;   [3] stunts_base.lisp    - Daily stunts & terrain traversal (chair, bed, floor, stairs, ice, slope)
;;   [4] Planar SE(2)        - Curvilinear & omnidirectional gaits (strafe, turn, circle, reverse arcs)
;; ============================================================================

(extends "gait_base.lisp")
(include "postures_base.lisp")

;; Mesomorph-medium (165 cm, 55 kg) propulsion spring specialization
(defbehavior hop (extends hop)
  (phase throw
    (impedance (stiffness mode_axial_compression 21800) (damping mode_axial_compression 0))))

(include "stunts_base.lisp")

;; ----------------------------------------------------------------------------
;; CATEGORY 4: PLANAR SE(2) & CURVILINEAR GAITS
;; ----------------------------------------------------------------------------

;; strafe left: lateral stepping and CoM shift to the left
(defbehavior strafe_left
  (phase shift (use reverse_gait)
    (duration 60s) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.035) (mode_lateral_shift 0.08))
    (cpg_config
      (frequency 0.90Hz)
      (amplitude mode_sagittal_drive 0.04) (amplitude mode_pelvis_list 0.22)
      (amplitude mode_arm_swing 0.35)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))))

;; strafe right: lateral stepping and CoM shift to the right
(derive strafe_right strafe_left
  (phase shift
    (latent_targets (mode_lateral_shift -0.08))))

;; turn left: forward cruise with left yaw turn
(defbehavior turn_left
  (phase turn (use forward_gait)
    (duration 60s) (blend_duration 550ms)
    (latent_targets (mode_axial_compression 0.045) (mode_pitch_attitude 0.01)
      (mode_lateral_shift 0.02) (mode_pelvis_turn 0.38) (mode_stance_width 0.14))
    (cpg_config
      (frequency 0.90Hz)
      (amplitude mode_lateral_shift 0.27) (amplitude mode_sagittal_drive 0.48)
      (amplitude mode_limb_lift 1.00) (amplitude mode_foot_rock 0.56)
      (amplitude mode_pelvis_list 0.34) (amplitude mode_pelvis_turn 0.38)
      (amplitude mode_arm_swing 0.80) (amplitude mode_stance_width 0.10))))

;; turn right: forward cruise with right yaw turn
(derive turn_right turn_left
  (phase turn
    (latent_targets (mode_lateral_shift -0.02) (mode_pelvis_turn -0.38))))

;; circle walk: continuous circular locomotion
(derive circle_walk turn_left
  (phase turn
    (latent_targets (mode_lateral_shift 0.025) (mode_pelvis_turn 0.40))))

;; turn back left: reverse cruise with left yaw turn
(defbehavior turn_back_left
  (phase turn (use reverse_gait)
    (duration 60s) (blend_duration 600ms)
    (latent_targets (mode_pelvis_turn 0.30) (mode_lateral_shift 0.0))
    (cpg_config
      (frequency 0.90Hz)
      (amplitude mode_sagittal_drive 0.25) (amplitude mode_pelvis_turn 0.18))))

;; turn back right: reverse cruise with right yaw turn
(derive turn_back_right turn_back_left
  (phase turn
    (latent_targets (mode_pelvis_turn -0.30))))

