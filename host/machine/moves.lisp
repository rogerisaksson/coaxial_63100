;; Her moves, in orthokinematics (machine/ortho.py): balance, rhythm and elasticity.
;; A mode is -1 to 1 (machine/modal.py); a stiffness N/m, a damping N s/m. The page's gym
;; (machine/sprung.py) says them by digit, in this order: the payloads, then the behaviours.

;; stand, her knees soft
(stream_inject
  (sequence_id 201)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.2))
  (impedance
    (stiffness mode_axial_compression 3000) (damping mode_axial_compression 600)
    (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 300)
    (stiffness mode_sagittal_shift 300) (damping mode_sagittal_shift 150))
  (cpg_config (enable false)))

;; sway from sole to sole, no steps: her ZMP into each sole in turn
(stream_inject
  (sequence_id 202)
  (blend_duration 1500ms)
  (cpg_config
    (enable true)
    (frequency 0.8Hz)
    (amplitude mode_lateral_shift 0.37)
    (amplitude mode_sagittal_drive 0.0)
    (phase_offset mode_lateral_shift mode_sagittal_drive 90deg)))

;; step: 0.13 m/s
(stream_inject
  (sequence_id 203)
  (blend_duration 1500ms)
  (cpg_config (amplitude mode_sagittal_drive 0.2)))

;; walk on: 0.22 m/s
(stream_inject
  (sequence_id 204)
  (blend_duration 1500ms)
  (cpg_config (amplitude mode_sagittal_drive 0.5))
  (task_constraints (maintain_zmp (polygon dynamic_footprint))))

;; back: the same rhythm, her drive a quarter turn the other way
(stream_inject
  (sequence_id 205)
  (blend_duration 1500ms)
  (cpg_config
    (amplitude mode_sagittal_drive 0.2)
    (phase_offset mode_lateral_shift mode_sagittal_drive -90deg)))

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

;; on one sole, the other off the floor: it leaves once her weight is over
(defbehavior one_sole
  (phase over
    (duration 4s)
    (blend_duration 1500ms)
    (latent_targets (mode_lateral_shift 1.0) (mode_limb_lift -1.0))
    (impedance (stiffness mode_lateral_shift 3000) (damping mode_lateral_shift 600))
    (cpg_override (halt_oscillators))
    (task_constraints (maintain_zmp (polygon support_polygon)))))
