;; ============================================================================
;; DAILY STUNTS & TERRAIN TRAVERSALS (machine/stunts_base.lisp)
;; Furniture transitions, multi-surface gaits, and capture-point shove reflexes.
;; ============================================================================

(extends "postures_base.lisp")

;; Symmetric two-footed jump clearance trait
(deftrait two_footed
  (latent_targets (mode_limb_lift 0.0)))

;; sit down on a chair
(defbehavior sit_chair (extends stance_hold)
  (phase approach
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.35) (mode_pitch_attitude 0.18) (mode_sagittal_shift -0.05)))
  (phase lower
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.58) (mode_pitch_attitude 0.08) (mode_sagittal_shift -0.12))
    (impedance (stiffness mode_axial_compression 4500) (damping mode_axial_compression 800)))
  (phase seated (use lower)
    (duration 60s) (blend_duration 500ms)
    (latent_targets (mode_pitch_attitude 0.04))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600))))

;; rise up from a chair
(defbehavior rise_chair (extends stance_hold)
  (phase lean
    (duration 800ms) (blend_duration 400ms)
    (latent_targets (mode_axial_compression 0.55) (mode_pitch_attitude 0.32) (mode_sagittal_shift 0.14))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 800)))
  (phase thrust (use lean)
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.15) (mode_pitch_attitude 0.10) (mode_sagittal_shift 0.05))
    (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900)))
  (phase stand (use thrust)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600))))

;; lie down in a bed
(defbehavior lie_bed (extends stance_hold)
  (phase sit_edge
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.48) (mode_pitch_attitude 0.14) (mode_sagittal_shift -0.04)))
  (phase recline
    (duration 1500ms) (blend_duration 700ms)
    (latent_targets (mode_axial_compression 0.65) (mode_pitch_attitude -0.14) (mode_pelvis_turn 0.12) (mode_limb_lift 0.0) (mode_sagittal_shift -0.06))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)))
  (phase bed_rest (use recline)
    (duration 60s) (blend_duration 800ms)
    (latent_targets (mode_axial_compression 0.72) (mode_pitch_attitude -0.18) (mode_pelvis_turn 0.0) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 2500) (damping mode_axial_compression 500))))

;; rise up from a bed
(defbehavior rise_bed (extends stance_hold)
  (phase sit_up
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.52) (mode_pitch_attitude 0.20) (mode_limb_lift 0.0) (mode_sagittal_shift 0.04))
    (impedance (stiffness mode_axial_compression 4000) (damping mode_axial_compression 700)))
  (phase swing_out
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.25) (mode_pitch_attitude 0.10) (mode_limb_lift 0.0) (mode_sagittal_shift 0.02)))
  (phase stand (use sit_up)
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0))
    (impedance (damping mode_axial_compression 600))))

;; lie down on the floor
(defbehavior lie_floor (extends stance_hold)
  (phase kneel
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.70) (mode_pitch_attitude 0.18)))
  (phase lower
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.90) (mode_pitch_attitude 0.22) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 2800) (damping mode_axial_compression 600)))
  (phase floor_rest (use lower)
    (duration 60s) (blend_duration 800ms)
    (latent_targets (mode_axial_compression 0.95) (mode_pitch_attitude 0.05))
    (impedance (stiffness mode_axial_compression 2400) (damping mode_axial_compression 500))))

;; rise up from the floor
(defbehavior rise_floor (extends stance_hold)
  (phase push_arms
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.72) (mode_pitch_attitude 0.22) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 4500) (damping mode_axial_compression 800)))
  (phase half_kneel
    (duration 1200ms) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.45) (mode_pitch_attitude 0.14) (mode_lateral_shift 0.10)))
  (phase stand (use half_kneel)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_lateral_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 700))))

;; hop down from a chair
(defbehavior hop_down_chair (extends hop stance_hold)
  (phase step_off)
  (phase drop)
  (phase absorb)
  (phase recover (use soft_land)
    (duration 500ms) (blend_duration 350ms)
    (latent_targets (mode_axial_compression 0.0))))

;; two-footed jump up onto a chair
(defbehavior hop_up_chair (extends hop_down_chair two_footed)
  (phase crouch_load)
  (phase propulsion)
  (phase flight_tuck)
  (phase seat_land))

;; stairs ascent/descent: high clearance, deliberate cadence
(defbehavior stairs_walk
  (phase climb (use forward_gait)
    (duration 60s) (blend_duration 600ms)
    (latent_targets (mode_pitch_attitude 0.02))
    (impedance
      (stiffness mode_axial_compression 3200) (damping mode_axial_compression 650)
      (stiffness mode_foot_rock 2200))
    (cpg_config
      (frequency 0.86Hz)
      (amplitude mode_lateral_shift 0.28) (amplitude mode_sagittal_drive 0.48)
      (amplitude mode_limb_lift 1.0) (amplitude mode_foot_rock 0.45)
      (amplitude mode_pelvis_list 0.35) (amplitude mode_pelvis_turn 0.35)
      (amplitude mode_arm_swing 0.75))))

;; stairs descent: controlled step down, upright attitude, compliant landing
(derive stairs_down stairs_walk
  (phase descent
    (latent_targets (mode_axial_compression 0.05))
    (cpg_config
      (amplitude mode_sagittal_drive 0.24)
      (amplitude mode_foot_rock 0.40)
      (amplitude mode_arm_swing 0.70))))

;; walking on ice / low friction: narrow stance, minimal shear, flat feet
(defbehavior ice_walk
  (phase shuffle (use forward_gait)
    (duration 60s) (blend_duration 500ms)
    (impedance
      (stiffness mode_axial_compression 3200) (stiffness mode_lateral_shift 2800)
      (stiffness mode_foot_rock 3000) (damping mode_foot_rock 300))
    (cpg_config
      (frequency 1.15Hz)
      (amplitude mode_lateral_shift 0.18) (amplitude mode_sagittal_drive 0.20)
      (amplitude mode_limb_lift 0.60) (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.10) (amplitude mode_pelvis_turn 0.10)
      (amplitude mode_arm_swing 0.15))))

;; walking on an incline or slope: torso leaned into grade, compliant foot rock
(derive slope_walk stairs_walk
  (phase incline
    (latent_targets (mode_axial_compression 0.06) (mode_pitch_attitude 0.02))
    (cpg_config
      (frequency 0.90Hz)
      (amplitude mode_lateral_shift 0.30) (amplitude mode_sagittal_drive 0.55)
      (amplitude mode_limb_lift 1.0) (amplitude mode_foot_rock 0.58)
      (amplitude mode_arm_swing 0.80))))

;; active capture-point parry after an external shove
(defbehavior parry_shove (extends stance_hold)
  (phase catch
    (duration 900ms) (blend_duration 450ms)
    (latent_targets (mode_axial_compression 0.15) (mode_pitch_attitude 0.04) (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 4500) (damping mode_axial_compression 900)
      (stiffness mode_lateral_shift 3500) (damping mode_lateral_shift 500)
      (stiffness mode_sagittal_shift 1500) (damping mode_sagittal_shift 400)))
  (phase restore
    (duration 1000ms) (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0) (mode_lateral_shift 0.0))
    (impedance (stiffness mode_axial_compression 4000) (damping mode_axial_compression 700))))

;; controlled compliant fall: attempt parry first, then absorb into deep symmetric crouch
(defbehavior controlled_fall (extends stance_hold)
  (phase cushion
    (duration 900ms) (blend_duration 450ms)
    (latent_targets (mode_axial_compression 0.28) (mode_pitch_attitude 0.06) (mode_sagittal_shift 0.0) (mode_arm_swing 0.0))
    (impedance
      (stiffness mode_axial_compression 4500) (damping mode_axial_compression 1000)
      (stiffness mode_lateral_shift 3500) (damping mode_lateral_shift 550)
      (stiffness mode_sagittal_shift 1500) (damping mode_sagittal_shift 450)))
  (phase land (use cushion)
    (duration 60s) (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.35) (mode_pitch_attitude 0.04) (mode_lateral_shift 0.0))
    (impedance
      (damping mode_axial_compression 900) (damping mode_lateral_shift 450)
      (damping mode_sagittal_shift 400)
      (stiffness mode_pitch_attitude 1500) (damping mode_pitch_attitude 300))))
