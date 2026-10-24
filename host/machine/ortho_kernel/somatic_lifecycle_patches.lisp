;; =============================================================================
;; FILE: somatic_lifecycle_patches.lisp
;; DOMAIN: Complete Somatic Obstacle-Course & Furniture Schedule (1 kHz SoA)
;; =============================================================================
(defpackage :ortho-dsp.somatic-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.supervisor
        :ortho-dsp.sequencer)
  (:export #:patch-shove-chair-and-mount-high-stool
           #:patch-dismount-high-stool-and-sleep
           #:patch-wake-and-traverse-stairs-up-down
           #:compile-somatic-interaction-soa))
(in-package :ortho-dsp.somatic-patches)

;; 1. Shove Blocking Chair with Forearms -> Vault & Slide onto Overheight Chair (Barstool)
(defpatch patch-shove-chair-and-mount-high-stool [base-gait]
  (-> base-gait
      (transducer-viscoelastic-flat)
      (xform-balance-biped-zmp)
      ;; Shove obstructing chair aside using non-prehensile forearm hook
      (xform-somatic-furniture-shove :shove-technique :ulnar-backrest-hook
                                     :clearing-offset-xy-mm [0.0 480.0])
      ;; Pivot at high chair, rise onto metatarsals, perch thighs, and slide back onto seat
      (lock-horizon-gaze :room-pan-yaw-rad 0.0)
      (pivot-single-anchor-arc :target-yaw-rad 3.14159 :standoff-x-mm 40.0)
      (xform-somatic-sit-ingress)
      (specialize-for-overheight-seat :direction :ingress :slide-speed-mps 0.16)
      (specialize-with-carpal-seat-boost :max-arm-push-share 0.20)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)
      (master-anti-spasm-insert)))

;; 2. Slide-Drop Dismount from Overheight Chair -> Walk to Bed -> Perch, Slide & Lie Supine
(defpatch patch-dismount-high-stool-and-sleep [seated-high-state]
  (-> seated-high-state
      ;; Slide pelvis forward off high seat until plantarflexed toes catch ground, then stand
      (specialize-for-overheight-seat :direction :egress :slide-speed-mps 0.15)
      (xform-somatic-sit-egress)
      ;; Approach bed, pivot 135 deg, perch-slide onto mattress, and roll into High-Z supine rest
      (lock-horizon-gaze :room-pan-yaw-rad 0.785)
      (pivot-single-anchor-arc :target-yaw-rad 2.35619 :standoff-x-mm 35.0)
      (xform-somatic-bed-ingress :bed-yaw-rad 1.5708)
      (master-anti-spasm-insert)))

;; 3. Rise from Bed -> Climb Stairs Up (+175mm) -> Descend Stairs Down (-175mm)
(defpatch patch-wake-and-traverse-stairs-up-down [sleeping-state]
  (let [walk-base   (-> sleeping-state
                        (xform-somatic-bed-egress)
                        (transducer-viscoelastic-flat)
                        (limiter-high-mu-planar))
        stairs-up   (-> walk-base
                        (xform-somatic-stair-traversal :nominal-riser-dz-mm +175.0 :nominal-tread-dx-mm 280.0)
                        (specialize-with-wall-or-banister-skim :side :right)
                        (limiter-high-mu-planar)
                        (master-anti-spasm-insert))
        stairs-down (-> walk-base
                        (xform-somatic-stair-traversal :nominal-riser-dz-mm -175.0 :nominal-tread-dx-mm 280.0)
                        (specialize-stair-descent-eccentric :riser-drop-mm -175.0 :tread-depth-mm 280.0)
                        (specialize-with-wall-or-banister-skim :side :left)
                        (limiter-high-mu-planar)
                        (master-anti-spasm-insert))]
    (-> walk-base
        (phase-locked-multiband-morph stairs-up :bars 1)
        (phase-locked-multiband-morph stairs-down :bars 2))))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-somatic-interaction-soa
  (let [mount-high-chair (patch-shove-chair-and-mount-high-stool nominal-biped-gait)
        dismount-to-bed  (patch-dismount-high-stool-and-sleep mount-high-chair)
        wake-and-stairs  (patch-wake-and-traverse-stairs-up-down dismount-to-bed)]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000
      :schedule
        [(oneshot  mount-high-chair) ; Forearm-shove blocking chair -> Toe-vault & slide onto high chair
         (dwell-ms 4000)             ; Seated on high chair with dangling legs in passive Null-Space
         (oneshot  dismount-to-bed)  ; Slide-drop catch onto toes -> Walk -> Slide & roll onto bed
         (dwell-ms 8000)             ; High-Z zero-torque sleep on mattress
         (oneshot  wake-and-stairs)]))) ; Roll-up & carpal push off bed -> Climb stairs -> Descend stairs
