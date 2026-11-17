;; =============================================================================
;; FILE: kinetic_elegance_sequencer.lisp
;; DOMAIN: Composite Momentum-Preserving Patches
;; =============================================================================
(defpackage :ortho-dsp.elegance-sequencer
  (:use :ortho-dsp.core
        :ortho-dsp.pivots
        :ortho-dsp.ballistic-handovers
        :ortho-dsp.supervisor)
  (:export #:patch-confident-chair-sit
           #:patch-confident-bed-roll))
(in-package :ortho-dsp.elegance-sequencer)

;; 1. Rigid Chair: Single-anchor pivot -> Staggered upright drop -> Wavefront settle
(defpatch patch-confident-chair-sit [approach-gait &key (seat-z-mm 450.0)] ; the seat the robot's eyes see
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.0)
      (pivot-single-anchor-arc :target-yaw-rad 3.14159 :sagittal-stagger-mm 95.0)
      (handover-staggered-vertical-chair :seat-z-mm seat-z-mm)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)
      (master-anti-spasm-insert)))

;; 2. Elevated Bed: 135-deg diagonal pivot -> Trochanteric counterweight leg-sweep -> High-Z
(defpatch patch-confident-bed-roll [approach-gait]
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.785)
      (pivot-single-anchor-arc :target-yaw-rad 2.35619 ; 135-deg diagonal entry
                               :standoff-x-mm 35.0
                               :eccentric-overlap-rad 0.95)
      (handover-trochanteric-bed-pendulum :mattress-z-mm 540.0 :mattress-eigenfreq-hz 1.80)
      (release-impedance-wavefront :terminal-mode :supine-high-z)
      (master-anti-spasm-insert)))
