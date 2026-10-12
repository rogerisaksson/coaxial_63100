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

;; walk on: 0.61 m/s. Her soles rock heel to toe, her pelvis lists and turns, her arms swing
(stream_inject
  (sequence_id 204)
  (blend_duration 2000ms)
  (latent_targets (mode_pitch_attitude 0.06))
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

(defbehavior crouch
  (phase down
    (duration 2s)
    (latent_targets (mode_axial_compression 0.6))
    (impedance (stiffness mode_axial_compression 20000) (damping mode_axial_compression 2000))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; her weight over one sole
(defbehavior weight_shift
  (phase over
    (duration 3s)
    (latent_targets (mode_lateral_shift 1.0))
    (impedance (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

(defbehavior bow
  (phase forward
    (duration 2s)
    (latent_targets (mode_pitch_attitude 0.5))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))

;; a hop: loaded, thrown by a spring that lets go 17 mm under her standing height, her
;; trunk free through the throw, landed on soft knees
(defbehavior hop
  (phase load
    (duration 600ms)
    (blend_duration 450ms)
    (latent_targets (mode_axial_compression 0.85))
    (impedance (stiffness mode_axial_compression 6000) (damping mode_axial_compression 900))
    (cpg_override (halt_oscillators)))
  (phase throw
    (duration 150ms)
    (latent_targets (mode_axial_compression 0.15))
    (impedance
      (stiffness mode_axial_compression 21800) (damping mode_axial_compression 0)
      (stiffness mode_pitch_attitude 0) (damping mode_pitch_attitude 0)))
  (phase land
    (duration 700ms)
    (latent_targets (mode_axial_compression 0.25))
    (impedance
      (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
      (stiffness mode_pitch_attitude 1458) (damping mode_pitch_attitude 292))))

;; on one sole, the other off the floor: it leaves once her weight is over
(defbehavior one_sole
  (phase over
    (duration 4s)
    (blend_duration 1500ms)
    (latent_targets (mode_lateral_shift 1.0) (mode_limb_lift -1.0))
    (impedance (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))
