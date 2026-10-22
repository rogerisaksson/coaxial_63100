;; =============================================================================
;; FILE: kinetic_elegance_sequencer.lisp
;; DOMAIN: Composite Momentum-Preserving Patches & 1 kHz SoA Schedule
;; =============================================================================
(defpackage :ortho-dsp.elegance-sequencer
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.modulators
        :ortho-dsp.surfaces
        :ortho-dsp.pivots
        :ortho-dsp.ballistic-handovers)
  (:export #:patch-confident-chair-sit
           #:patch-confident-armchair-lounge
           #:patch-confident-sofa-drop
           #:patch-confident-bed-roll
           #:patch-confident-floor-mattress
           #:compile-elegance-showcase-soa))
(in-package :ortho-dsp.elegance-sequencer)

;; 1. Rigid Chair: Single-anchor pivot -> Staggered upright drop -> Wavefront settle
(defpatch patch-confident-chair-sit [approach-gait &key (seat-z-mm 450.0)] ; the seat her eyes see
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.0)
      (pivot-single-anchor-arc :target-yaw-rad 3.14159 :sagittal-stagger-mm 95.0)
      (handover-staggered-vertical-chair :seat-z-mm seat-z-mm)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)))

;; 2. Armchair: Single-anchor pivot -> Forearm rail-skim arc -> Leg-cross settle
(defpatch patch-confident-armchair-lounge [approach-gait]
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.15)
      (pivot-single-anchor-arc :target-yaw-rad 3.14159 :coronal-stance-mm 165.0)
      (handover-guided-armchair-arc :seat-z-mm 420.0 :cross-leg-on-settle true)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)))

;; 3. Deep Sofa: Calf-indexed pivot -> Parabolic backward shear throw -> Cushion rebound
(defpatch patch-confident-sofa-drop [approach-gait]
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad -0.10)
      (pivot-single-anchor-arc :target-yaw-rad 3.14159 :standoff-x-mm 25.0 :tactile-calf-datum true)
      (handover-parabolic-sofa-injection :uncompressed-z-mm 410.0 :throw-depth-x-mm -260.0)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)))

;; 4. Elevated Bed: 135-deg diagonal pivot -> Trochanteric counterweight leg-sweep -> High-Z
(defpatch patch-confident-bed-roll [approach-gait]
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.785)
      (pivot-single-anchor-arc :target-yaw-rad 2.35619 ; 135-deg diagonal entry
                               :standoff-x-mm 35.0
                               :eccentric-overlap-rad 0.95)
      (handover-trochanteric-bed-pendulum :mattress-z-mm 540.0 :mattress-eigenfreq-hz 1.80)
      (release-impedance-wavefront :terminal-mode :supine-high-z)))

;; 5. Floor Mattress: Cross-under curtsy corkscrew -> Lateral hip touchdown -> Vertebral unroll
(defpatch patch-confident-floor-mattress [approach-gait]
  (-> approach-gait
      (lock-horizon-gaze :room-pan-yaw-rad 0.0)
      (pivot-cross-under-corkscrew :target-yaw-rad 3.14159 :coronal-cross-offset-mm -135.0)
      (handover-corkscrew-floor-mattress :pad-z-mm 110.0 :pad-eigenfreq-hz 3.20)
      (release-impedance-wavefront :terminal-mode :supine-high-z)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION (SHOWCASE PIPELINE)
;; -----------------------------------------------------------------------------
(defpatch compile-elegance-showcase-soa
  (let [sneaker-catwalk (-> nominal-biped-manifold
                            (transducer-elastomer-flat)
                            (modulator-coronal-catwalk :posture-profile :thoracic-kyphotic-bias)
                            (surface-high-mu-planar))
        barefoot-glide  (-> nominal-biped-manifold
                            (transducer-barefoot)
                            (modulator-coronal-catwalk :posture-profile :lumbar-lordotic-bias)
                            (surface-high-mu-planar))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000
      :schedule
        ;; Scenario A: Approach in sneakers -> Parabolic lounge drop into deep sofa
        [(bars 0 4  sneaker-catwalk)
         (oneshot   (patch-confident-sofa-drop sneaker-catwalk))
         (dwell-ms  4000)

         ;; Scenario B: Barefoot approach -> Cross-under corkscrew spiral onto floor mattress
         (bars 0 4  barefoot-glide)
         (oneshot   (patch-confident-floor-mattress barefoot-glide))])))
