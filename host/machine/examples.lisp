;; The language's first texts, as they were given: what each does on her is measured in
;; docs/findings/orthokinematics.md. The page's gym says them after `>`.

;; stand relaxed
(stream_inject
  (sequence_id 101)
  (blend_duration 0ms)
  (latent_targets
    (mode_axial_compression 0.15)
    (mode_lateral_shift 0.0)
    (mode_sagittal_drive 0.0))
  (impedance
    (stiffness mode_axial_compression 3000)
    (stiffness mode_lateral_shift 2500)
    (damping global 65))
  (cpg_config (enable false)))

;; begin the gait
(stream_inject
  (sequence_id 102)
  (blend_duration 800ms)
  (latent_targets (mode_axial_compression 0.25))
  (impedance
    (stiffness mode_lateral_shift 1200)
    (damping mode_lateral_shift 15))
  (cpg_config
    (enable true)
    (frequency 1.1Hz)
    (amplitude mode_sagittal_drive 0.4)
    (amplitude mode_lateral_shift 0.35)
    (phase_offset mode_lateral_shift mode_sagittal_drive 90deg)))

;; the gait, steady: impedance and rhythm as 102 left them
(stream_inject
  (sequence_id 103)
  (blend_duration 200ms)
  (task_constraints
    (maintain_zmp (polygon dynamic_footprint))
    (limit_yaw_momentum 15.0)))

;; a correction: the spring's damping alone
(stream_inject
  (sequence_id 104)
  (blend_duration 50ms)
  (impedance (damping mode_axial_compression 85)))

(defbehavior hop_asymmetric
  ;; Phase 1: Load the spring
  (phase pre_load
    (duration 400ms)
    (latent_targets
      (mode_axial_compression 0.8)   ; 80% vertical compression of CoM
      (mode_lateral_shift     0.5)
      (mode_pitch_attitude    0.0)   ; Keep the torso dead straight
    )
    (impedance
      (stiffness mode_axial_compression 4500)
      (damping   mode_axial_compression 25)
    )
    (cpg_override
      (halt_oscillators) ; Pause rhythmic gait for the jump
    )
  )

  ;; Phase 2: Explosion
  (phase propulsion
    (duration 150ms)
    (latent_targets
      (mode_axial_compression -1.0)  ; Full extension (negative compression)
    )
    (impedance
      (stiffness mode_axial_compression MAX_SYS_K)
      (damping   mode_axial_compression 0)
    )
    (task_constraints
      (maintain_zmp (polygon support_polygon))
    )
  )
)
