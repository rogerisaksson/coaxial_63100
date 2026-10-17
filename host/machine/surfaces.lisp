;; ============================================================================
;; SURFACES (machine/surfaces.lisp, machine/ortho.py)
;; Her rhythm on floors her soles slide on: under a surface's grip (machine.grip, felt
;; as a sole slides or told) its amplitudes are the most her rhythm takes, whatever
;; she is doing.
;; ============================================================================

;; a floor holding less than a quarter of a sole's load across - ice, a wet tile:
;; short flat steps, her pelvis and arms quiet
(defsurface slippery (grip 0.25)
  (cpg_config
    (amplitude mode_sagittal_drive 0.20) (amplitude mode_limb_lift 0.60)
    (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.10)
    (amplitude mode_pelvis_turn 0.10) (amplitude mode_arm_swing 0.15)))
