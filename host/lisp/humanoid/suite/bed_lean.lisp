;; =============================================================================
;; FILE: bed_lean.lisp
;; DOMAIN: Bed Ingress on the Back: Edge Sit -> Hands Back onto the Mattress -> Trunk over Them
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

;; The bed's edge she docks at first where both are hers to reach: :short its end, :long its side, :any
;; the one her walk reaches soonest (of 25 lodge errands the end first fell 6 on her way round the bed,
;; :any 1). The edge she sat on picks the recline and the sit-up (`:bed-side`, read off the face she docked at).
(defparameter *bed-side* :any)

;; From its end (as lying/Lying Down from Sitting): lean back, both legs drawn up and stretched out,
;; supine along its length; no turn, no leg swung alone. Their stretch over 3.6 s: over the side's
;; 1.4 their swing threw her 0.25 m up and rolled her 33 deg, over 3.6 s 16 deg, her hips 1.04 legs back.
(defmaneuver specialize-recline-short-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:palmar-carpal-bilateral -> :scapular-dorsal]
        :torso-pitch-ramp [-0.40 -> -0.75]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.35] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.08) (:ischial-bilateral 0.62) (:scapular-dorsal 0.15) (:palmar-carpal-bilateral 0.15)))
      (stage-sequential-manifold
        :duration-ms 1300.0
        :contact-chain [:scapular-dorsal -> :occipital]
        :torso-pitch-ramp [-0.75 -> -1.00]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.35 -> 1.15] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.58) (:scapular-dorsal 0.42) (:palmar-carpal-bilateral 0.0)))
      (stage-sequential-manifold
        :duration-ms 3600.0
        :contact-chain [:occipital -> :scapular-dorsal]
        :torso-pitch-ramp [-1.00 -> -1.35]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.15 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.55) (:palmar-carpal-bilateral 0.0)))))

;; From its side: swing up right leg then left leg, rotate -90 deg and stretch out supine along the
;; long side of the bed.
(defmaneuver specialize-recline-long-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:palmar-carpal-bilateral -> :greater-trochanter-r]
        :torso-pitch-ramp [-0.40 -> -0.75]
        :axial-yaw-ramp [0.0 -> -0.25]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.45 -> 1.35] :limb-lift -2.5)
        :manifold-pan '((:plantar-bilateral 0.08) (:ischial-bilateral 0.62) (:scapular-dorsal 0.15) (:palmar-carpal-bilateral 0.15)))
      (stage-sequential-manifold
        :duration-ms 1300.0
        :contact-chain [:greater-trochanter-r -> :greater-trochanter-l]
        :torso-pitch-ramp [-0.75 -> -1.00]
        :axial-yaw-ramp [-0.25 -> -0.52]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.35 -> 1.15] :limb-lift 2.5)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.58) (:scapular-dorsal 0.42) (:palmar-carpal-bilateral 0.0)))
      (stage-sequential-manifold
        :duration-ms 1400.0
        :contact-chain [:greater-trochanter-l -> :scapular-dorsal]
        :torso-pitch-ramp [-1.00 -> -1.35]
        :axial-yaw-ramp [-0.52 -> -0.60]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.15 -> 0.1047] :limb-lift 0.0)
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.45) (:scapular-dorsal 0.55) (:palmar-carpal-bilateral 0.0)))))

;; Sat on the mattress's edge: lean back onto hands, then lie down as the edge she sat on lets her.
(defmaneuver transfer-carpal-recline
  [seated-state &key (bed-side *bed-side*) (recline-rad -0.40) (duration-ms 900.0)]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms duration-ms
        :contact-chain [:ischial-bilateral -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [0.0 -> recline-rad]
        :axial-yaw-ramp [0.0 -> 0.0]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:palmar-carpal-bilateral 0.25)))
      (case bed-side
        (:long (specialize-recline-long-side))
        (otherwise (specialize-recline-short-side)))))

;; Dock at the bed's edge, her back to it; sit on the mattress's edge; lean back onto her hands
(defpatch patch-bed-lean [approach-gait &key (seat-z-mm 450.0) (bed-side *bed-side*)] ; the bed her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159 :fixture-face *bed-side*)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1050.0
        :seat-z-mm seat-z-mm)
      (transfer-carpal-recline :bed-side bed-side)
      (master-anti-spasm-insert)))

;; From supine along it from its end: sit up straight, no turn
(defmaneuver specialize-sit-up-short-side [state]
  (-> state
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:scapular-dorsal -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [-1.35 -> -0.25]
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
        :torso-pitch-ramp [-1.35 -> -0.25]
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
