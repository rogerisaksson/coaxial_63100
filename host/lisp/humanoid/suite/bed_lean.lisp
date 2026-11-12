;; =============================================================================
;; FILE: bed_lean.lisp
;; DOMAIN: Bed Ingress on the Back: Edge Sit -> Fists Planted Behind -> Scooted Up on Them -> Trunk over Them
;; =============================================================================
(defpackage :ortho-dsp.bed-lean
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.modulators
        :ortho-dsp.surfaces
        :ortho-dsp.docking
        :ortho-dsp.manifolds
        :ortho-dsp.substrates
        :ortho-dsp.supervisor)
  (:export #:*bed-side*
           #:specialize-recline-short-side
           #:specialize-recline-long-side
           #:transfer-carpal-recline
           #:specialize-sit-up-short-side
           #:specialize-sit-up-long-side
           #:patch-bed-lean
           #:specialize-leg-over-edge
           #:patch-leg-bed
           #:patch-rise-bed))
(in-package :ortho-dsp.bed-lean)

;; The bed's edge she docks at first where both are hers to reach: :short its end - lain along it as the take is
;; (lying/Lying Down from Sitting) -, :long its side, :any the one her walk reaches soonest (of 25 lodge errands
;; the end first fell 6 on her way round the bed, :any 1: the corners are her wayfinder's). :any since her side's
;; lie is the take's own (below: the bed bore her at the end of 9 of 12 starts across her builds, her law's own
;; stages 1). The edge she sat on picks the recline and the sit-up (`:bed-side`, read off the face she docked at).
(defparameter *bed-side* :any)

;; From its end, as the take (lying/Lying Down from Sitting: fists planted 0.2 m behind, upright; the pelvis drawn
;; 0.45 m up the bed in 1.3 s as both legs come up; the trunk down in 1.4 s; the legs stretched last). Hers: her
;; feet push her hips back over her planted fists (0.15 m, her pelvis rolling back to -0.6 rad on its ischia; the
;; fists kept where they were planted, `:carpal-ahead-m` of her pelvis's middle, `seated.kept`; her feet pushing
;; 0.30, her pelvis bearing 0.35 and her fists 0.35 through it: the feet told 0.45 it unloaded, hung on her left
;; thigh and landed 15 deg turned, the whole lie and its strokes yawed with it; the fists told 0.20 she lay square
;; but the pull drew her pelvis 0.01 m where it draws 0.13 on fists told 0.35-0.40, and told 0.40 the lodge's lie
;; landed 0.43 m in, short of its first stroke's grip). From the scoot's first tick her joints play her way in
;; (`:joint-program`, moves.lisp's `bed-entry`), told by where she lies on the bed her eyes saw, not by a count: her
;; feet planted, the shoulders flexed against the fists as the hips lift till her pelvis is 0.30 m in, the trunk
;; down, the legs raised, folded, lowered and stretched, then strokes up the bed - the heels planted 0.22 m from her
;; hips, the pelvis bridged, the heels pushed, the pelvis set down - while she is more than half a stroke short of
;; its middle (her text counted the puppet's bed_pull's strokes from the bed's depth, 0.16 m each, and she lay
;; wherever they took her; begun from the landed stage, after the law's scoot, she lay diagonal);
;; her text goes flat under it: her trunk told down and her back to bear, so her rest limps her on the foam as the
;; program ends and her law drives again. The chain's own lean, release and recline (lie.md) stood under the
;; program with asks it could not meet (705 shrunk passes) and would take her back with the knees asked 86 deg.
(defmaneuver specialize-recline-short-side [state]
  (-> state
      (stage-sequential-manifold                       ; the scoot: her feet push her hips back over her fists;
                                                     ; its fists where the plant set them (`seated.kept` holds a
                                                     ; fist that bore where it bore; one that has not is sought
                                                     ; there: sought beside her hips instead it swung forward, her
                                                     ; trunk rolled 13-18 deg and the scoot stood her up)
        :duration-ms 1200.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-sacral]
        :joint-program :bed-entry
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.25] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.30) (:ischial-bilateral 0.35) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.35)))
      (stage-sequential-manifold                       ; landed, under her way in
        :duration-ms 500.0
        :contact-chain [:ischial-sacral -> :ischial-bilateral]
        :torso-pitch-ramp [0.0 -> -0.10]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 1.25] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.25)))
      (stage-sequential-manifold                       ; flat under the program, her legs stretched along the bed:
        :duration-ms 2000.0                            ; her back told to bear, her rest limps her as the program ends
        :contact-chain [:ischial-bilateral -> :scapular-dorsal]
        :torso-pitch-ramp [-0.10 -> -1.5708]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.55) (:palmar-carpal-bilateral 0.0)))))

;; From its side, the take's own lie (lying/Lying Down from Sitting, recorded on a bed's side; gcode/
;; lying_down_from_sitting.gcode) on her joints from its landed stage's first tick, as the end plays its pull: her
;; trunk turned a little on her hands, leant back, her right leg swung up and turned at the hip, then her left,
;; supine along the long side; then flat, her back told to bear, so her rest limps her on the foam as the program
;; ends. Over 4 starts each (turned +-4 and +-8 mrad) at 0.95, 1 and 1.05 of her size the bed bore her at the end of
;; 9 of 12; her law's own stages (the trunk asked back to -1.57 over 4.5 s, yawed -0.6, the legs swept up) 1 of 12,
;; and her own size's unturned start only through a jolt off the seat (her soles 285 N at 5.25 s): her chain answers
;; her posture, not the turn asked of it. The take's misses: lain 6-12 cm off the edge, a leg over it (1.05); rolled
;; off (0.95). Her legs her law's while it plays (`:joint-keep`): the take's own swept her hips apart (right -90,
;; left to -26 deg) and rolled her 180 deg onto her belly over her right thigh, 8.2-11.2 s at up to 108 N m, and the
;; rise from her back then asked her trunk upside down.
(defmaneuver specialize-recline-long-side [state]
  (-> state
      (stage-sequential-manifold                       ; landed: the take's lie from its first tick
        :duration-ms 3350.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :joint-program :lying-down-from-sitting
        :joint-keep :legs
        :torso-pitch-ramp [0.0 -> -0.10]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.45] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.65) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.35)))
      (stage-sequential-manifold                       ; flat under the program: her back told to bear, her rest
        :duration-ms 2000.0                            ; limps her as the program ends
        :contact-chain [:ischial-bilateral -> :scapular-dorsal]
        :torso-pitch-ramp [-0.10 -> -1.5708]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.55) (:palmar-carpal-bilateral 0.0)))))

;; Sat on the mattress's edge: her fists planted on it 0.14 m behind her, upright - placed for a recline she had not
;; made (-0.40) they hung 0.17 m over the foam; for the stage's pitch, 0.11 (her pelvis sat 0.68 rad forward of
;; it) and the first fist down bore alone: each is sought to the foam under her pelvis's pitch as sensed and held
;; as it bore (`seated.kept`) -, then lie down as the edge she sat on lets her.
(defmaneuver transfer-carpal-recline
  [seated-state &key (bed-side *bed-side*) (recline-rad 0.0) (duration-ms 900.0)]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms duration-ms
        :contact-chain [:ischial-bilateral -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [0.0 -> recline-rad]
        :carpal-ahead-m -0.14
        :axial-yaw-ramp [0.0 -> 0.0]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:palmar-carpal-bilateral 0.25)))
      (case bed-side
        (:long (specialize-recline-long-side))
        (otherwise (specialize-recline-short-side)))))

;; Dock at the bed's edge, her back to it; sit on the mattress's edge over 1 400 ms as the take sits (lying/Sitting on
;; Bed, 3.2 natural times from 10 to 90 % of the way, measured from 0.5 s after the telling: at 1 050 the drop is
;; over before the window, at 1 750 her fists hang over the foam); lean back onto her hands
(defpatch patch-bed-lean [approach-gait &key (seat-z-mm 450.0) (bed-side *bed-side*)] ; the bed her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159 :fixture-face *bed-side*)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1400.0
        :seat-z-mm seat-z-mm)
      (transfer-carpal-recline :bed-side bed-side)
      (master-anti-spasm-insert)))

;; From its end, her left leg over the edge onto the bed in her law (no program): leant back onto her fists
;; (-0.60 rad, her hips holding her pelvis at its wanted pitch, `modal.seat`), the leg lifted its full reach
;; (6 units, 4 cm each over the hip, `cascade.opening`) and folded over 2 s, drawn 0.55 m back of where its knee
;; had its ankle (`:limb-back-m`, `maneuver._placed`), set down on the foam; the other leg hangs. The puppet's
;; crab (tools/sim/puppet.py) lifts, folds and sets one leg at a time this way, the trunk regulated.
(defmaneuver specialize-leg-over-edge [state]
  (-> state
      (stage-sequential-manifold                       ; the lean onto the fists behind her
        :duration-ms 600.0
        :contact-chain [:ischial-sacral -> :occipital]
        :torso-pitch-ramp [0.0 -> -0.60]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 1.25] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.22
        :trunk-hold 4.0
        :manifold-pan '((:plantar-bilateral 0.30) (:ischial-bilateral 0.40) (:palmar-carpal-bilateral 0.30)))
      (stage-sequential-manifold                       ; lifted and folded
        :duration-ms 2000.0
        :contact-chain [:occipital -> :greater-trochanter-r]
        :torso-pitch-ramp [-0.60 -> -0.60]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 2.79] :limb-lift 6.0)
        :limb-back-m 0.01
        :talocrural-rad -0.05
        :carpal-ahead-m -0.22
        :trunk-hold 4.0
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.50) (:palmar-carpal-bilateral 0.35)))
      (stage-sequential-manifold                       ; drawn back over the bed
        :duration-ms 1000.0
        :contact-chain [:greater-trochanter-r -> :greater-trochanter-l]
        :torso-pitch-ramp [-0.60 -> -0.60]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [2.79 -> 2.79] :limb-lift 6.0)
        :limb-back-m 0.55
        :talocrural-rad -0.05
        :carpal-ahead-m -0.22
        :trunk-hold 4.0
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.50) (:palmar-carpal-bilateral 0.35)))
      (stage-sequential-manifold                       ; set down on the foam
        :duration-ms 300.0
        :contact-chain [:greater-trochanter-l -> :ischial-bilateral]
        :torso-pitch-ramp [-0.60 -> -0.60]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [2.79 -> 2.79] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.22
        :trunk-hold 4.0
        :manifold-pan '((:plantar-bilateral 0.25) (:ischial-bilateral 0.45) (:palmar-carpal-bilateral 0.30)))))

(defpatch patch-leg-bed [approach-gait &key (seat-z-mm 450.0)]
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159 :fixture-face :short)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1400.0
        :seat-z-mm seat-z-mm)
      (stage-sequential-manifold                       ; onto her fists behind her, as the lie's lean
        :duration-ms 900.0
        :contact-chain [:ischial-bilateral -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [0.0 -> 0.0]
        :carpal-ahead-m -0.14
        :axial-yaw-ramp [0.0 -> 0.0]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:palmar-carpal-bilateral 0.25)))
      (specialize-leg-over-edge)
      (master-anti-spasm-insert)))

;; From supine along it from its end: her bed_exit move (machine.moves.bedexit) sits her up on her arms and scoots
;; her onto the edge, her text's rows waiting while it plays (`waits.played_back`); both stages ask the seat it leaves
;; her on - knees 1.885, shins 0.489, trunk upright, the edge bearing what her law's own sit leaves it - so her law,
;; driving again, holds her there: asked the sit-up's knees (1.40 -> 1.45) it pushed her 0.07 m back up the bed and
;; her lean stood on soles 0.52, short of the thrust's 0.82; asked 1.78 and 0.45 off the move's 108 and 28 deg, she
;; fell in her thrust. Its time after the move is what her law takes to settle her there: 0.4 s to her pelvis under
;; 2 mm/s, her joints under 17 deg/s (0, +-2e-4 rad laid); held 1.9 s - its 1200 ms and a second stage asking the same
;; seat 700 - she sat still 1.5 s and stood 11.7 s after the say
(defmaneuver specialize-sit-up-short-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 400.0
        :contact-chain [:scapular-dorsal -> :palmar-carpal-bilateral]
        :joint-program :bed-exit
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.885 -> 1.885] :limb-lift 0.0)
        :talocrural-rad 0.489
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.85) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.0)))))

;; From supine along it from its side: rotate back to the edge, plant feet, sit up
(defmaneuver specialize-sit-up-long-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:scapular-dorsal -> :greater-trochanter-r]
        :torso-pitch-ramp [-1.5708 -> -0.25]
        :axial-yaw-ramp [-0.60 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [0.1047 -> 1.40] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.06) (:ischial-bilateral 0.54) (:scapular-dorsal 0.30) (:palmar-carpal-bilateral 0.10)))
      (stage-sequential-manifold
        :duration-ms 700.0
        :contact-chain [:greater-trochanter-r -> :ischial-bilateral]
        :torso-pitch-ramp [-0.25 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.40 -> 1.45] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.70) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.15)))))

;; Sat up as the edge she lay down from lets her, then up as from a chair, her trunk leaned a chair's 0.74 rad: from
;; its end, seated on its foam over its firm core, 1.00 threw her off her feet at all 4 of 7 docks that thrust; 0.74
;; stood all 7. From its long side as from its end (her bed exit's move): lain there on her back her pelvis is 0.85 m
;; from the edge, her feet at it, and the long side's own sit-up - turned and raised in place over 1.9 s - got her
;; trunk no higher than -54 deg as her pelvis tipped back, and the rise's lean flipped her; the move sits her up on
;; her arms and scoots her toward her feet till her shanks hang, whichever edge that is
(defpatch patch-rise-bed [seated-state &key (bed-side *bed-side*)]
  (-> seated-state
      (case bed-side
        (:long (specialize-sit-up-short-side))
        (otherwise (specialize-sit-up-short-side)))
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (master-anti-spasm-insert)))
