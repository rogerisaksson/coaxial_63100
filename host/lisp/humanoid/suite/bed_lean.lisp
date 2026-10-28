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

;; Sat on the mattress's edge: her hands back onto it and her trunk back over them, the mattress and her
;; hands bearing her, her soles their residual.
(defmaneuver transfer-carpal-recline
  [seated-state &key (recline-rad -0.5) (duration-ms 1500.0)]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms duration-ms
        :contact-chain [:ischial-bilateral -> :palmar-carpal-bilateral]
        :torso-pitch-ramp [0.0 -> recline-rad]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.60) (:palmar-carpal-bilateral 0.25)))))

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

;; From her hands back on the mattress: sat up off them, her arms hanging, then up as from a chair
(defpatch patch-rise-bed [seated-state]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms 900.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :torso-pitch-ramp [-0.5 -> 0.0]
        :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.85) (:palmar-carpal-bilateral 0.0)))
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (master-anti-spasm-insert)))
