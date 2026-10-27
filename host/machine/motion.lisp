;; ============================================================================
;; DAILY STUNTS & TERRAIN TRAVERSALS (machine/motion.lisp)
;; Furniture transitions, multi-surface gaits, and capture-point shove reflexes.
;; ============================================================================

(extends "postures_base.lisp")

;; Symmetric two-footed jump clearance trait
(deftrait two_footed
  (latent_targets (mode_limb_lift 0.0)))

;; Her arms held where they hang, as the suite's maneuvers hold them (`maneuver._armed`: her rest's
;; stiffness times W_STILL over W_POSTURE): let go, lying back on the bed her shoulders turned 260-500 deg
(deftrait arms_held
  (impedance (stiffness mode_arm_swing 1198000) (damping mode_arm_swing 2395)))

;; sit_chair, rise_chair, lie_bed and rise_bed: the ortho-DSP suite's maneuvers (machine/maneuver.py)


;; lie down where she stands: crouch to her modes' floor, then the drives soft and the pitch
;; let go - gravity lays her, the fall window's own quiet (45 N passive); she rests till a rise
(defbehavior lie_floor (extends stance_hold)
  (phase kneel
    (duration 2500ms) (blend_duration 1200ms)
    (latent_targets (mode_axial_compression 3.2) (mode_pitch_attitude 0.20) (mode_arm_swing 0.0)
                    (mode_arm_reach -1.4))
    (impedance (stiffness mode_arm_swing 1200) (damping mode_arm_swing 120)
               (stiffness mode_arm_reach 1200) (damping mode_arm_reach 120)))
  (phase tip
    (duration 2000ms) (blend_duration 1500ms)
    (latent_targets (mode_axial_compression 4.0) (mode_pitch_attitude -2.0)
                    (mode_arm_swing 0.0) (mode_arm_reach -3.9))
    (impedance (stiffness mode_axial_compression 400) (damping mode_axial_compression 150)
               (stiffness mode_pitch_attitude 150) (damping mode_pitch_attitude 60)
               (stiffness mode_arm_swing 1200) (damping mode_arm_swing 120)
               (stiffness mode_arm_reach 1200) (damping mode_arm_reach 120))
    (task_constraints (maintain_zmp (polygon 0))
                      (contact_manifolds (plantar 0.0) (ischial 1.0) (trochanter 0.6) (dorsal 1.0)
                                         (palmar 0.6))))
  (phase floor_rest (use tip)
    (duration 60s) (blend_duration 1000ms)
    (latent_targets (mode_axial_compression 3.2) (mode_pitch_attitude -2.4))
    (impedance (stiffness mode_axial_compression 5) (damping mode_axial_compression 120)
               (stiffness mode_pitch_attitude 5) (damping mode_pitch_attitude 60)
               (stiffness mode_sagittal_shift 5) (damping mode_sagittal_shift 60)
               (stiffness mode_lateral_shift 5) (damping mode_lateral_shift 60)
               (stiffness mode_arm_swing 5) (damping mode_arm_swing 30))
    (task_constraints (maintain_zmp (polygon 0))
                      (contact_manifolds (plantar 0.0) (ischial 1.0) (trochanter 0.6) (dorsal 1.0)))))

;; sat on the floor, she drags her butt forward on her hands: off her knees her palms down beside her hips as columns
;; (`modal.poses`) till each bears, her butt lowered on them and her soles till the floor bears it, her support's
;; polygon held throughout - let go, she threw herself back off her knees, her head down in 1.3 s; then a stroke - her
;; elbows straightened, her shoulders turned back as her butt slides - and her hands lifted forward again
(defbehavior scoot_floor (extends lie_floor)
  (phase kneel)
  (phase squat
    (duration 1500ms) (blend_duration 1500ms)
    (latent_targets (mode_axial_compression 4.0) (mode_pitch_attitude unsaid))
    (impedance (stiffness mode_pitch_attitude 0) (damping mode_pitch_attitude 0))
    (task_constraints (maintain_zmp (polygon support_polygon)) (contact_manifolds (plantar 1.0))))
  (phase tip
    (duration 500ms)
    (latent_targets (mode_pitch_attitude -0.4) (mode_arm_reach 0.0) (mode_arm_fold 0.0))
    (impedance (stiffness mode_pitch_attitude 1458) (damping mode_pitch_attitude 292))
    (task_constraints (contact_manifolds (plantar 0.7))))
  (phase sit (use tip) (wait contact)
    (duration 1500ms)
    (task_constraints (maintain_zmp (polygon support_polygon))
                      (contact_manifolds (plantar 0.0) (ischial 1.0) (palmar 0.6))))
  (phase hands (use sit)
    (duration 800ms) (blend_duration 500ms)
    (latent_targets (mode_arm_reach -0.5) (mode_arm_fold 1.0))
    (impedance (stiffness mode_arm_fold 1200) (damping mode_arm_fold 120))
    (task_constraints (maintain_zmp (polygon support_polygon))
                      (contact_manifolds (plantar 0.0) (ischial 1.0) (palmar 0.0))))
  (phase push (use hands)
    (duration 1000ms) (blend_duration 400ms)
    (latent_targets (mode_arm_reach -1.5) (mode_arm_fold 0.0))
    (impedance (stiffness mode_sagittal_shift 5) (damping mode_sagittal_shift 60))
    (task_constraints (maintain_zmp (polygon support_polygon)) (slide 0.15)
                      (contact_manifolds (plantar 0.0) (ischial 0.4) (palmar 0.6))))
  (phase place (use hands)
    (duration 800ms) (blend_duration 400ms)
    (task_constraints (maintain_zmp (polygon support_polygon)) (slide 0.0)
                      (contact_manifolds (plantar 0.0) (ischial 1.0) (palmar 0.0))))
  (phase push_2 (use push))
  (phase place_2 (use place))
  (phase push_3 (use push))
  (phase floor_rest))


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
