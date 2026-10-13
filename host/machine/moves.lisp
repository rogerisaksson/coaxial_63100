;; Her moves, in orthokinematics (machine/ortho.py): balance, rhythm and elasticity.
;; A mode is -1 to 1 (machine/modal.py); a stiffness N/m, a damping N s/m. The page's gym
;; (machine/sprung.py) says them by digit, in this order: the payloads, then the behaviours.

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

;; sway from sole to sole, no steps: her ZMP into each sole in turn. Her stride's other
;; modes are set to her drive here: her sway and her pelvis's list three tenths of a turn
;; ahead of it, its turn, her arms and her soles' rock with it
(stream_inject
  (sequence_id 202)
  (blend_duration 1500ms)
  (cpg_config
    (enable true)
    (frequency 0.9Hz)
    (amplitude mode_lateral_shift 0.31)
    (amplitude mode_sagittal_drive 0.0)
    (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
    (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
    (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
    (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
    (phase_offset mode_foot_rock mode_sagittal_drive 0deg)))

;; step: 0.12 m/s. Along her way she is damped to her cruise, held to no place; a sole in
;; the air 4 cm clear and stiff in its pitch
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
(stream_inject
  (sequence_id 205)
  (blend_duration 1500ms)
  (latent_targets (mode_pitch_attitude 0.0))
  (cpg_config
    (amplitude mode_sagittal_drive 0.2)
    (amplitude mode_foot_rock 0.0)
    (amplitude mode_pelvis_list 0.0)
    (amplitude mode_pelvis_turn 0.0)
    (amplitude mode_arm_swing 0.0)
    (phase_offset mode_lateral_shift mode_sagittal_drive -108deg)))

;; 6: trip: quick tripping small steps with stiff arms and flat soles
(defbehavior trip
  (phase mincing
    (duration 60s)
    (blend_duration 500ms)
    (latent_targets
      (mode_axial_compression 0.03)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (enable true)
      (frequency 1.08Hz)
      (amplitude mode_lateral_shift 0.28)
      (amplitude mode_sagittal_drive 0.22)
      (amplitude mode_limb_lift 0.85)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.15)
      (amplitude mode_pelvis_turn 0.1)
      (amplitude mode_arm_swing 0.05)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint))))
  (phase brake
    (duration 1800ms)
    (blend_duration 500ms)
    (impedance
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (frequency 0.85Hz)
      (amplitude mode_lateral_shift 0.28)
      (amplitude mode_sagittal_drive 0.04)
      (amplitude mode_limb_lift 0.7)
      (amplitude mode_arm_swing 0.0))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; 7: jog: brisk high-cadence gait with deeper knee flexion and strong arm drive
(defbehavior jog
  (phase apa
    (duration 700ms)
    (blend_duration 350ms)
    (latent_targets
      (mode_axial_compression 0.06)
      (mode_pitch_attitude 0.03)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3200) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (enable true)
      (frequency 0.96Hz)
      (amplitude mode_lateral_shift 0.31)
      (amplitude mode_sagittal_drive 0.50)
      (amplitude mode_limb_lift 1.0)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.45)
      (amplitude mode_pelvis_turn 0.45)
      (amplitude mode_arm_swing 0.95)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint))))
  (phase run
    (duration 60s)
    (blend_duration 800ms)
    (latent_targets
      (mode_axial_compression 0.06)
      (mode_pitch_attitude 0.03))
    (cpg_config
      (frequency 0.96Hz)
      (amplitude mode_sagittal_drive 0.58)
      (amplitude mode_limb_lift 1.0)
      (amplitude mode_foot_rock 0.65)
      (amplitude mode_pelvis_list 0.45)
      (amplitude mode_pelvis_turn 0.45)
      (amplitude mode_arm_swing 1.0)))
  (phase brake
    (duration 1800ms)
    (blend_duration 700ms)
    (latent_targets
      (mode_axial_compression 0.02)
      (mode_pitch_attitude 0.0))
    (impedance
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (frequency 0.83Hz)
      (amplitude mode_lateral_shift 0.28)
      (amplitude mode_sagittal_drive 0.04)
      (amplitude mode_limb_lift 0.8)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.15)
      (amplitude mode_pelvis_turn 0.1)
      (amplitude mode_arm_swing 0.1))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; 8: crouch
(defbehavior crouch
  (phase down
    (duration 2s)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.6))
    (impedance (stiffness mode_axial_compression 8000) (damping mode_axial_compression 1200))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; 9: hop: loaded, thrown by a spring that lets go 17 mm under her standing height, her
;; trunk free through the throw, landed on soft knees
(defbehavior hop
  (phase load
    (duration 600ms)
    (blend_duration 450ms)
    (latent_targets (mode_axial_compression 0.85) (mode_sagittal_shift -0.1))
    (impedance
      (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900)
      (stiffness mode_sagittal_shift 1200) (damping mode_sagittal_shift 400))
    (cpg_override (halt_oscillators)))
  (phase throw
    (duration 150ms)
    (latent_targets (mode_axial_compression 0.15))
    (impedance
      (stiffness mode_axial_compression 21800) (damping mode_axial_compression 0)
      (stiffness mode_pitch_attitude 0) (damping mode_pitch_attitude 200)))
  (phase land
    (duration 700ms)
    (latent_targets (mode_axial_compression 0.25) (mode_sagittal_shift 0.0) (mode_pitch_attitude 0.0))
    (impedance
      (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
      (stiffness mode_sagittal_shift 600) (damping mode_sagittal_shift 300)
      (stiffness mode_pitch_attitude 1458) (damping mode_pitch_attitude 292))))

;; 0: on one sole, the other off the floor: it leaves once her weight is over
(defbehavior one_sole
  (phase over
    (duration 4s)
    (blend_duration 800ms)
    (latent_targets (mode_lateral_shift 0.92) (mode_limb_lift -1.0) (mode_pelvis_list 0.0))
    (impedance
      (stiffness mode_axial_compression 6000) (damping mode_axial_compression 800)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600)
      (stiffness mode_pelvis_list 1500) (damping mode_pelvis_list 300))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; her weight over one sole
(defbehavior weight_shift
  (phase over
    (duration 3s)
    (blend_duration 500ms)
    (latent_targets (mode_lateral_shift 1.0))
    (impedance (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

(defbehavior bow
  (phase forward
    (duration 2s)
    (blend_duration 500ms)
    (latent_targets (mode_pitch_attitude 0.5))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; ballistic walk from rest: crisp APA unweighting directly into stride
(defbehavior ballistic_walk
  (phase apa
    (duration 800ms)
    (blend_duration 400ms)
    (latent_targets
      (mode_axial_compression 0.06)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (enable true)
      (frequency 0.9Hz)
      (amplitude mode_lateral_shift 0.31)
      (amplitude mode_sagittal_drive 0.60)
      (amplitude mode_limb_lift 1.0)
      (amplitude mode_pelvis_list 0.4)
      (amplitude mode_pelvis_turn 0.4)
      (amplitude mode_arm_swing 0.8)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint))))
  (phase stride
    (duration 20s)
    (blend_duration 1000ms)
    (latent_targets (mode_axial_compression 0.07) (mode_pitch_attitude 0.0))
    (cpg_config
      (frequency 0.9Hz)
      (amplitude mode_lateral_shift 0.31)
      (amplitude mode_sagittal_drive 0.60)
      (amplitude mode_limb_lift 1.0)
      (amplitude mode_foot_rock 0.6)
      (amplitude mode_pelvis_list 0.4)
      (amplitude mode_pelvis_turn 0.4)
      (amplitude mode_arm_swing 0.8)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))))

;; ballistic initiation into gentle walk: crisp small steps without tripping
(defbehavior init_walk
  (phase init
    (duration 700ms)
    (blend_duration 350ms)
    (latent_targets
      (mode_axial_compression 0.065)
      (mode_pitch_attitude 0.02))
    (impedance
      (stiffness mode_axial_compression 3200) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 2400) (damping mode_foot_rock 240))
    (cpg_config
      (enable true)
      (frequency 0.92Hz)
      (amplitude mode_lateral_shift 0.31)
      (amplitude mode_sagittal_drive 0.35)
      (amplitude mode_limb_lift 0.95)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.35)
      (amplitude mode_pelvis_turn 0.35)
      (amplitude mode_arm_swing 0.70)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; sit down on a chair
(defbehavior sit_chair
  (phase approach
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.35) (mode_pitch_attitude 0.18) (mode_sagittal_shift -0.05))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase lower
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.58) (mode_pitch_attitude 0.08) (mode_sagittal_shift -0.12))
    (impedance (stiffness mode_axial_compression 4500) (damping mode_axial_compression 800))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase seated
    (duration 60s)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.58) (mode_pitch_attitude 0.04) (mode_sagittal_shift -0.12))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; rise up from a chair
(defbehavior rise_chair
  (phase lean
    (duration 800ms)
    (blend_duration 400ms)
    (latent_targets (mode_axial_compression 0.55) (mode_pitch_attitude 0.32) (mode_sagittal_shift 0.14))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 800))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase thrust
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.15) (mode_pitch_attitude 0.10) (mode_sagittal_shift 0.05))
    (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase stand
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; lie down in a bed
(defbehavior lie_bed
  (phase sit_edge
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.52) (mode_pitch_attitude 0.15))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase recline
    (duration 1500ms)
    (blend_duration 700ms)
    (latent_targets (mode_axial_compression 0.85) (mode_pitch_attitude -0.40) (mode_pelvis_turn 0.30) (mode_limb_lift 0.50))
    (impedance (stiffness mode_axial_compression 2500) (damping mode_axial_compression 500))
    (cpg_override (halt_oscillators)))
  (phase bed_rest
    (duration 60s)
    (blend_duration 800ms)
    (latent_targets (mode_axial_compression 0.90) (mode_pitch_attitude -0.45) (mode_pelvis_turn 0.0) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 1500) (damping mode_axial_compression 400))
    (cpg_override (halt_oscillators))))

;; rise up from a bed
(defbehavior rise_bed
  (phase sit_up
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.60) (mode_pitch_attitude 0.35) (mode_limb_lift -0.40))
    (impedance (stiffness mode_axial_compression 4000) (damping mode_axial_compression 700))
    (cpg_override (halt_oscillators)))
  (phase swing_out
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.52) (mode_pitch_attitude 0.22) (mode_limb_lift 0.0))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase stand
    (duration 1000ms)
    (blend_duration 500ms)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0))
    (impedance (stiffness mode_axial_compression 4000) (damping mode_axial_compression 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; lie down on the floor
(defbehavior lie_floor
  (phase kneel
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.75) (mode_pitch_attitude 0.22))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase lower
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.95) (mode_pitch_attitude 0.32) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 2500) (damping mode_axial_compression 600))
    (cpg_override (halt_oscillators)))
  (phase floor_rest
    (duration 60s)
    (blend_duration 800ms)
    (latent_targets (mode_axial_compression 1.0) (mode_pitch_attitude 0.0) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 1200) (damping mode_axial_compression 400))
    (cpg_override (halt_oscillators))))

;; rise up from the floor
(defbehavior rise_floor
  (phase push_arms
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.80) (mode_pitch_attitude 0.28) (mode_arm_swing 0.0))
    (impedance (stiffness mode_axial_compression 4500) (damping mode_axial_compression 800))
    (cpg_override (halt_oscillators)))
  (phase half_kneel
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.60) (mode_pitch_attitude 0.20) (mode_lateral_shift 0.25))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon))))
  (phase stand
    (duration 1200ms)
    (blend_duration 600ms)
    (latent_targets (mode_axial_compression 0.0) (mode_pitch_attitude 0.0) (mode_lateral_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 700))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; hop down from a chair
(defbehavior hop_down_chair
  (phase step_off
    (duration 300ms)
    (blend_duration 150ms)
    (latent_targets (mode_axial_compression 0.20) (mode_sagittal_shift 0.15))
    (cpg_override (halt_oscillators)))
  (phase drop
    (duration 250ms)
    (blend_duration 100ms)
    (latent_targets (mode_axial_compression 0.15))
    (cpg_override (halt_oscillators)))
  (phase absorb
    (duration 600ms)
    (blend_duration 200ms)
    (latent_targets (mode_axial_compression 0.65) (mode_sagittal_shift 0.0))
    (impedance (stiffness mode_axial_compression 5000) (damping mode_axial_compression 1600))
    (cpg_override (halt_oscillators)))
  (phase recover
    (duration 800ms)
    (blend_duration 400ms)
    (latent_targets (mode_axial_compression 0.02))
    (impedance (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; two-footed jump up onto a chair
(defbehavior hop_up_chair
  (phase crouch_load
    (duration 500ms)
    (blend_duration 350ms)
    (latent_targets (mode_axial_compression 0.85) (mode_pitch_attitude 0.15))
    (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900))
    (cpg_override (halt_oscillators)))
  (phase propulsion
    (duration 180ms)
    (blend_duration 50ms)
    (latent_targets (mode_axial_compression -0.20))
    (impedance (stiffness mode_axial_compression MAX_SYS_K) (damping mode_axial_compression 0)))
  (phase flight_tuck
    (duration 250ms)
    (blend_duration 80ms)
    (latent_targets (mode_axial_compression 0.65) (mode_limb_lift 0.60)))
  (phase seat_land
    (duration 800ms)
    (blend_duration 300ms)
    (latent_targets (mode_axial_compression 0.58) (mode_pitch_attitude 0.05))
    (impedance (stiffness mode_axial_compression 4500) (damping mode_axial_compression 1200))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; stairs ascent/descent: high clearance, deliberate cadence
(defbehavior stairs_walk
  (phase climb
    (duration 60s)
    (blend_duration 600ms)
    (latent_targets
      (mode_axial_compression 0.04)
      (mode_pitch_attitude 0.12)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3500) (damping mode_axial_compression 700)
      (stiffness mode_lateral_shift 3200) (damping mode_lateral_shift 350)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 280)
      (stiffness mode_foot_rock 1800) (damping mode_foot_rock 200))
    (cpg_config
      (enable true)
      (frequency 0.68Hz)
      (amplitude mode_lateral_shift 0.30)
      (amplitude mode_sagittal_drive 0.42)
      (amplitude mode_limb_lift 1.0)
      (amplitude mode_foot_rock 0.3)
      (amplitude mode_pelvis_list 0.35)
      (amplitude mode_pelvis_turn 0.35)
      (amplitude mode_arm_swing 0.70)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; walking on ice / low friction: narrow stance, minimal shear, flat feet
(defbehavior ice_walk
  (phase shuffle
    (duration 60s)
    (blend_duration 500ms)
    (latent_targets
      (mode_axial_compression 0.04)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3200) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 2800) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 3000) (damping mode_foot_rock 300))
    (cpg_config
      (enable true)
      (frequency 1.15Hz)
      (amplitude mode_lateral_shift 0.18)
      (amplitude mode_sagittal_drive 0.20)
      (amplitude mode_limb_lift 0.60)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_list 0.10)
      (amplitude mode_pelvis_turn 0.10)
      (amplitude mode_arm_swing 0.15)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; walking on an incline or slope: torso leaned into grade, compliant foot rock
(defbehavior slope_walk
  (phase incline
    (duration 60s)
    (blend_duration 600ms)
    (latent_targets
      (mode_axial_compression 0.04)
      (mode_pitch_attitude 0.15)
      (mode_sagittal_shift 0.05))
    (impedance
      (stiffness mode_axial_compression 3400) (damping mode_axial_compression 650)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 0) (damping mode_sagittal_shift 260)
      (stiffness mode_foot_rock 1600) (damping mode_foot_rock 200))
    (cpg_config
      (enable true)
      (frequency 0.80Hz)
      (amplitude mode_lateral_shift 0.30)
      (amplitude mode_sagittal_drive 0.45)
      (amplitude mode_limb_lift 0.95)
      (amplitude mode_foot_rock 0.40)
      (amplitude mode_pelvis_list 0.35)
      (amplitude mode_pelvis_turn 0.35)
      (amplitude mode_arm_swing 0.75)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_list mode_sagittal_drive 108deg)
      (phase_offset mode_pelvis_turn mode_sagittal_drive 0deg)
      (phase_offset mode_arm_swing mode_sagittal_drive 0deg)
      (phase_offset mode_foot_rock mode_sagittal_drive 0deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; active capture-point parry after an external shove
(defbehavior parry_shove
  (phase catch
    (duration 600ms)
    (blend_duration 200ms)
    (latent_targets
      (mode_axial_compression 0.15)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 4500) (damping mode_axial_compression 800)
      (stiffness mode_lateral_shift 4000) (damping mode_lateral_shift 500)
      (stiffness mode_sagittal_shift 1500) (damping mode_sagittal_shift 400))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon dynamic_footprint))))
  (phase restore
    (duration 800ms)
    (blend_duration 400ms)
    (latent_targets
      (mode_axial_compression 0.0)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (impedance
      (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
      (stiffness mode_sagittal_shift 800) (damping mode_sagittal_shift 240))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; controlled compliant fall: ukemi breakfall, knees absorb descent, arms symmetrically poised, head protected
(defbehavior controlled_fall
  (phase cushion
    (duration 800ms)
    (blend_duration 250ms)
    (latent_targets
      (mode_axial_compression 0.38)
      (mode_pitch_attitude 0.12)
      (mode_sagittal_shift 0.0)
      (mode_arm_swing 0.0))
    (impedance
      (stiffness mode_axial_compression 5000) (damping mode_axial_compression 1200)
      (stiffness mode_lateral_shift 3500) (damping mode_lateral_shift 500)
      (stiffness mode_sagittal_shift 2000) (damping mode_sagittal_shift 500)
      (stiffness mode_pitch_attitude 2000) (damping mode_pitch_attitude 400))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon dynamic_footprint))))
  (phase land
    (duration 60s)
    (blend_duration 500ms)
    (latent_targets
      (mode_axial_compression 0.55)
      (mode_pitch_attitude 0.08)
      (mode_sagittal_shift 0.0)
      (mode_arm_swing 0.0))
    (impedance
      (stiffness mode_axial_compression 4500) (damping mode_axial_compression 900)
      (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 400)
      (stiffness mode_sagittal_shift 1500) (damping mode_sagittal_shift 400)
      (stiffness mode_pitch_attitude 1500) (damping mode_pitch_attitude 300))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; strafe left: lateral stepping and CoM shift to the left (Quake A)
(defbehavior strafe_left
  (phase shift
    (duration 60s)
    (blend_duration 800ms)
    (latent_targets
      (mode_lateral_shift 0.40)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (cpg_config
      (enable true)
      (frequency 1.0Hz)
      (amplitude mode_lateral_shift 0.45)
      (amplitude mode_limb_lift 0.6)
      (amplitude mode_stance_width 0.25)
      (amplitude mode_sagittal_drive 0.05)
      (phase_offset mode_lateral_shift mode_sagittal_drive 108deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; strafe right: lateral stepping and CoM shift to the right (Quake D)
(defbehavior strafe_right
  (phase shift
    (duration 60s)
    (blend_duration 800ms)
    (latent_targets
      (mode_lateral_shift -0.40)
      (mode_pitch_attitude 0.0)
      (mode_sagittal_shift 0.0))
    (cpg_config
      (enable true)
      (frequency 1.0Hz)
      (amplitude mode_lateral_shift 0.45)
      (amplitude mode_limb_lift 0.6)
      (amplitude mode_stance_width 0.25)
      (amplitude mode_sagittal_drive 0.05)
      (phase_offset mode_lateral_shift mode_sagittal_drive -108deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; turn left (Quake WA, vänstersväng): forward cruise with left yaw turn
(defbehavior turn_left
  (phase turn
    (duration 60s)
    (blend_duration 1000ms)
    (latent_targets
      (mode_pelvis_turn 0.45)
      (mode_lateral_shift 0.15)
      (mode_pitch_attitude 0.0))
    (cpg_config
      (enable true)
      (frequency 1.05Hz)
      (amplitude mode_sagittal_drive 0.50)
      (amplitude mode_foot_rock 0.50)
      (amplitude mode_pelvis_turn 0.50)
      (amplitude mode_pelvis_list 0.35)
      (amplitude mode_arm_swing 0.70))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; turn right (Quake WD, högersväng): forward cruise with right yaw turn
(defbehavior turn_right
  (phase turn
    (duration 60s)
    (blend_duration 1000ms)
    (latent_targets
      (mode_pelvis_turn -0.45)
      (mode_lateral_shift -0.15)
      (mode_pitch_attitude 0.0))
    (cpg_config
      (enable true)
      (frequency 1.05Hz)
      (amplitude mode_sagittal_drive 0.50)
      (amplitude mode_foot_rock 0.50)
      (amplitude mode_pelvis_turn 0.50)
      (amplitude mode_pelvis_list 0.35)
      (amplitude mode_arm_swing 0.70))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; turn back left (Quake SA, bakåt-vänstersväng)
(defbehavior turn_back_left
  (phase turn
    (duration 60s)
    (blend_duration 1500ms)
    (latent_targets
      (mode_pelvis_turn 0.20)
      (mode_lateral_shift 0.10)
      (mode_pitch_attitude 0.0))
    (cpg_config
      (enable true)
      (frequency 0.90Hz)
      (amplitude mode_sagittal_drive 0.20)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_turn 0.15)
      (amplitude mode_pelvis_list 0.0)
      (amplitude mode_arm_swing 0.0)
      (phase_offset mode_lateral_shift mode_sagittal_drive -108deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

;; turn back right (Quake SD, SD-högersväng)
(defbehavior turn_back_right
  (phase turn
    (duration 60s)
    (blend_duration 1500ms)
    (latent_targets
      (mode_pelvis_turn -0.20)
      (mode_lateral_shift -0.10)
      (mode_pitch_attitude 0.0))
    (cpg_config
      (enable true)
      (frequency 0.90Hz)
      (amplitude mode_sagittal_drive 0.20)
      (amplitude mode_foot_rock 0.0)
      (amplitude mode_pelvis_turn 0.15)
      (amplitude mode_pelvis_list 0.0)
      (amplitude mode_arm_swing 0.0)
      (phase_offset mode_lateral_shift mode_sagittal_drive -108deg))
    (task_constraints (maintain_zmp (polygon dynamic_footprint)))))

