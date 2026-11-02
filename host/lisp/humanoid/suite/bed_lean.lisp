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
           #:patch-rise-bed))
(in-package :ortho-dsp.bed-lean)

;; The bed's edge she docks at first where both are hers to reach: :short its end - lain along it as the take is
;; (lying/Lying Down from Sitting) -, :long its side, :any the one her walk reaches soonest (of 25 lodge errands
;; the end first fell 6 on her way round the bed, :any 1: the corners are her wayfinder's). The edge she sat on
;; picks the recline and the sit-up (`:bed-side`, read off the face she docked at).
(defparameter *bed-side* :short)

;; From its end, as the take (lying/Lying Down from Sitting: fists planted 0.2 m behind, upright; the pelvis drawn
;; 0.45 m up the bed in 1.3 s as both legs come up; the trunk down in 1.4 s; the legs stretched last). Hers: her
;; feet push her hips back over her planted fists (0.16 m, her pelvis rolling back to -0.6 rad on its ischia);
;; landed, she leans on the fists till her feet unload (-0.35 asked, -0.5 met, 93 N on the hands); her legs let go
;; there and drawn up (dorsal told lifts them) the chain's step lays her down on the fists at the take's 60 deg/s,
;; 0.35 m up the bed, and stretches her out. Her fists kept where they were planted (`:carpal-ahead-m` of her
;; pelvis's middle at the plant, each held where it bore as her hips pass it, `seated.kept`): the pull is her arms'
;; reach, her elbows down on the foam as her trunk comes down. Recline-only stages left her sat 0.06 m in.
(defmaneuver specialize-recline-short-side [state]
  (-> state
      (stage-sequential-manifold                       ; the scoot: her feet push her hips back over her fists;
                                                     ; its fists where the plant set them (`seated.kept` holds a
                                                     ; fist that bore where it bore; one that has not is sought
                                                     ; there: sought beside her hips instead it swung forward, her
                                                     ; trunk rolled 13-18 deg and the scoot stood her up)
        :duration-ms 1200.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-sacral]
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.25] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.45) (:ischial-bilateral 0.15) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.40)))
      (stage-sequential-manifold                       ; landed
        :duration-ms 500.0
        :contact-chain [:ischial-sacral -> :ischial-bilateral]
        :torso-pitch-ramp [0.0 -> -0.10]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 1.25] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.25)))
      (stage-sequential-manifold                       ; the lean onto her fists, her feet still down, her back told
        :duration-ms 1200.0                            ; already: let go with it half told her legs hauled her up
        :contact-chain [:ischial-bilateral -> :trochanter-lateral]
        :torso-pitch-ramp [-0.10 -> -0.35]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.25 -> 1.30] :limb-lift 0.0)
        :talocrural-rad -0.05
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.35) (:scapular-dorsal 0.10) (:palmar-carpal-bilateral 0.40)))
      (stage-sequential-manifold                       ; her legs let go and drawn up: the chain's step from here
        :duration-ms 1200.0
        :contact-chain [:trochanter-lateral -> :scapular-dorsal]
        :torso-pitch-ramp [-0.35 -> -0.35]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.30 -> 1.90] :limb-lift 0.0)
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.40) (:scapular-dorsal 0.15) (:palmar-carpal-bilateral 0.45)))
      (stage-sequential-manifold                       ; laid down on her fists
        :duration-ms 3000.0
        :contact-chain [:scapular-dorsal -> :occipital]
        :torso-pitch-ramp [-0.35 -> -1.10]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.90 -> 1.50] :limb-lift 0.0)
        :carpal-ahead-m -0.14
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.35) (:palmar-carpal-bilateral 0.30)))
      (stage-sequential-manifold                       ; flat, her legs stretched along the bed
        :duration-ms 2000.0
        :contact-chain [:occipital -> :scapular-dorsal]
        :torso-pitch-ramp [-1.10 -> -1.5708]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.50 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.55) (:palmar-carpal-bilateral 0.0)))))

;; From its side, in her own frame counter-clockwise (a negative yaw): her trunk turned a little on her hands first,
;; leant back; then her right leg swung up and turned at the hip, then her left, each toward the heading she is told
;; (machine.moves.emancipation); her hands bear till her back does; stretched out supine along the long side.
(defmaneuver specialize-recline-long-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 600.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :torso-pitch-ramp [0.0 -> -0.50]
        :axial-yaw-ramp [0.0 -> -0.35]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.45] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.55) (:palmar-carpal-bilateral 0.30)))
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:ischial-bilateral -> :greater-trochanter-r]
        :torso-pitch-ramp [-0.50 -> -0.75]
        :axial-yaw-ramp [-0.35 -> -0.50]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.35] :limb-lift -2.5)
        :manifold-pan '((:plantar-bilateral 0.08) (:ischial-bilateral 0.52) (:scapular-dorsal 0.15) (:palmar-carpal-bilateral 0.25)))
      (stage-sequential-manifold
        :duration-ms 1300.0
        :contact-chain [:greater-trochanter-r -> :greater-trochanter-l]
        :torso-pitch-ramp [-0.75 -> -1.00]
        :axial-yaw-ramp [-0.50 -> -0.60]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.35 -> 1.15] :limb-lift 2.5)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.30) (:palmar-carpal-bilateral 0.25)))
      (stage-sequential-manifold
        :duration-ms 1400.0
        :contact-chain [:greater-trochanter-l -> :scapular-dorsal]
        :torso-pitch-ramp [-1.00 -> -1.5708]
        :axial-yaw-ramp [-0.60 -> -0.60]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.15 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.40) (:scapular-dorsal 0.50) (:palmar-carpal-bilateral 0.10)))))

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

;; From supine along it from its end: sit up straight, no turn
(defmaneuver specialize-sit-up-short-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:scapular-dorsal -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [-1.5708 -> -0.25]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [0.1047 -> 1.40] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.06) (:ischial-bilateral 0.54) (:scapular-dorsal 0.30) (:palmar-carpal-bilateral 0.10)))
      (stage-sequential-manifold
        :duration-ms 700.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :torso-pitch-ramp [-0.25 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.40 -> 1.45] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.70) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.15)))))

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

;; Sat up as the edge she lay down from lets her, then up as from a chair
(defpatch patch-rise-bed [seated-state &key (bed-side *bed-side*)]
  (-> seated-state
      (case bed-side
        (:long (specialize-sit-up-long-side))
        (otherwise (specialize-sit-up-short-side)))
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (master-anti-spasm-insert)))
