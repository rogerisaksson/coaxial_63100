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
  (:export #:transfer-carpal-recline
           #:patch-bed-lean
           #:patch-rise-bed))
(in-package :ortho-dsp.bed-lean)

;; Sat on the mattress's edge: lean back onto hands, swing up right leg then left leg, rotate
;; -90 deg and stretch out supine along the long side of the bed.
(defmaneuver transfer-carpal-recline
  [seated-state &key (recline-rad -0.40) (duration-ms 900.0)]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms duration-ms
        :contact-chain [:ischial-bilateral -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [0.0 -> recline-rad]
        :axial-yaw-ramp [0.0 -> 0.0]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:palmar-carpal-bilateral 0.25)))
      (stage-sequential-manifold
        :duration-ms 1200.0
        :contact-chain [:palmar-carpal-bilateral -> :greater-trochanter-r]
        :torso-pitch-ramp [recline-rad -> -0.75]
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

;; Dock at the bed's edge, her back to it; sit on the mattress's edge; lean back onto her hands
(defpatch patch-bed-lean [approach-gait &key (seat-z-mm 450.0)] ; the bed her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1050.0
        :seat-z-mm seat-z-mm)
      (transfer-carpal-recline)
      (master-anti-spasm-insert)))

;; From supine along the bed: rotate back to the edge, plant feet, sit up, then up as from a chair
(defpatch patch-rise-bed [seated-state]
  (-> seated-state
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
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.70) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.15)))
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (master-anti-spasm-insert)))
