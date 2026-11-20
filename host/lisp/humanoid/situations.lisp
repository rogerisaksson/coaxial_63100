;; ============================================================================
;; SITUATIONS (lisp/humanoid/situations.lisp, machine/senses/situation.py)
;; Where it is, read from its sensors each pass: bounds on its readings - grip,
;; slope, borne, upright, pushed, raised - and what its law takes inside them,
;; whatever it is doing: its rhythm's amplitudes no more than the caps said,
;; blended in over the fifth inside each bound, and its modes no lower than the
;; latent targets said, eased in over the blend said (situation.gripped).
;; ============================================================================

;; a floor holding less than a quarter of a sole's load across - ice, a wet tile:
;; short flat steps, the robot's pelvis and arms quiet, and under whatever the robot does its centre of mass
;; down 5 cm and its track 15 % wider, eased in over the blend: a sliding sole's friction pitches the robot by
;; the height of its centre over the floor. Told so on ice the robot walked all 6 s of it in 6 of 6 laid
;; where upright and narrow it fell at 3.8-4.0 s, and fell after its stop only as the brake's and the
;; stand's rows set it upright and narrow again on the ice
(defsituation slippery (grip below 0.25)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.35) (mode_stance_width 0.15))
  (cpg_config
    (amplitude mode_sagittal_drive 0.20) (amplitude mode_limb_lift 0.60)
    (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.10)
    (amplitude mode_pelvis_turn 0.10) (amplitude mode_arm_swing 0.15)))

;; the floor rising under the robot's standing soles past 1 in 12 - a ramp, a flight -, or falling:
;; crouched for leg reach over the grade, wider in track, leaned into a climb and flat-soled descending
(defsituation uphill (slope above 0.08)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.30) (mode_stance_width 0.12))
  (cpg_config
    (amplitude mode_sagittal_drive 0.32) (amplitude mode_foot_rock 0.10)
    (amplitude mode_arm_swing 0.35)))

(defsituation downhill (slope below -0.08)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.35) (mode_stance_width 0.15))
  (cpg_config
    (amplitude mode_sagittal_drive 0.18) (amplitude mode_foot_rock 0.0)
    (amplitude mode_pelvis_list 0.15) (amplitude mode_arm_swing 0.20)))

;; a seat or a bed bearing half the robot's weight off its soles, its pelvis
;; upright; lying on it
(defsituation seated (borne above 0.5) (upright above 0.5))
(defsituation lying (upright below 0.5))

;; a push on the robot's above its soles a fiftieth of its weight - a wall, a tree, a hand:
;; lower and wider while braced against the load
(defsituation held (pushed above 0.02) (borne below 0.5)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.20) (mode_stance_width 0.15)))

;; a swing met an edge 2 cm up - a sill, a step: crouched with a wider track as the soles climb it
(defsituation stepping (raised above 0.02)
  (blend_duration 500ms)
  (latent_targets (mode_axial_compression 0.18) (mode_stance_width 0.10))
  (cpg_config
    (amplitude mode_sagittal_drive 0.40) (amplitude mode_foot_rock 0.20)))

;; frictionless skates: zero sagittal drive, foot rock and arm swing, the robot's balance held by its ankles
;; and hips
(defsituation skates (grip below 0.005)
  (cpg_config
    (amplitude mode_sagittal_drive 0.0) (amplitude mode_foot_rock 0.0)
    (amplitude mode_arm_swing 0.0)))
