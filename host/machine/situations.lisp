;; ============================================================================
;; SITUATIONS (machine/situations.lisp, machine/situation.py)
;; Where she is, read from her sensors each pass: bounds on her readings - grip,
;; slope, borne, upright, pushed, raised - and what her law takes inside them,
;; whatever she is doing: her rhythm's amplitudes no more than the caps said,
;; blended in over the fifth inside each bound.
;; ============================================================================

;; a floor holding less than a quarter of a sole's load across - ice, a wet tile:
;; short flat steps, her pelvis and arms quiet
(defsituation slippery (grip below 0.25)
  (cpg_config
    (amplitude mode_sagittal_drive 0.20) (amplitude mode_limb_lift 0.60)
    (amplitude mode_foot_rock 0.0) (amplitude mode_pelvis_list 0.10)
    (amplitude mode_pelvis_turn 0.10) (amplitude mode_arm_swing 0.15)))

;; the floor rising under her standing soles past 1 in 12 - a ramp, a flight -, or falling
(defsituation uphill (slope above 0.08))
(defsituation downhill (slope below -0.08))

;; a seat or a bed bearing half her weight off her soles, her pelvis upright; lying on it
(defsituation seated (borne above 0.5) (upright above 0.5))
(defsituation lying (upright below 0.5))

;; a push on her above her soles a fiftieth of her weight - a wall, a tree, a hand
(defsituation held (pushed above 0.02) (borne below 0.5))

;; a swing met an edge 2 cm up - a sill, a step
(defsituation stepping (raised above 0.02))
