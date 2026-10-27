;; =============================================================================
;; FILE: bedside_choreography_patches.lisp
;; DOMAIN: Complete 1 kHz SoA Schedule for Stool/Chair & Long-Side Bed Sequence
;; =============================================================================
(defpackage :ortho-dsp.bedside-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.nav-envs
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.bedside-core
        :ortho-dsp.bedside-xforms)
  (:export #:patch-turn-and-sit-high-stool
           #:patch-dismount-stool-and-mount-bed-longside
           #:patch-bed-roll-side-stomach-back-and-rise
           #:compile-complete-stool-and-bed-lifecycle-soa))
(in-package :ortho-dsp.bedside-patches)

;; 1. Stand Alongside Overheight Stool/Chair -> Turn 180 -> Calf Index -> Sit & Balance
(defpatch patch-turn-and-sit-high-stool [base-gait]
  (-> base-gait
      (transducer-viscoelastic-flat)
      (xform-balance-biped-zmp)
      ;; Stand by stool, turn 180 deg, brush her thighs' backs against its edge over her knee:
      (xform-stand-and-turn-at-fixture :turn-yaw-rad 3.14159 :standoff-clearance-mm 35.0
                                       :probe-capsules [:femoral-diaphysis-dorsal-l :femoral-diaphysis-dorsal-r])
      ;; Vault onto toes, slide thighs onto stool, halt at centroid via rim-ZCBF, balance core:
      (xform-sit-on-chair-or-stool :slide-speed-mps 0.15 :use-carpal-hand-boost true)
      (specialize-with-stool-active-core-balance :seat-diameter-mm 340.0)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; 2. Rise from Stool -> Walk to Bed Long Side -> Turn 180 -> Sit -> 90-Deg Rotate to Supine
(defpatch patch-dismount-stool-and-mount-bed-longside [stool-seated-state]
  (-> stool-seated-state
      ;; Slide forward off high stool until plantarflexed toes catch ground, then stand:
      (xform-stand-from-chair-or-stool :slide-speed-mps 0.15)
      ;; Walk to the long side of the bed, turn 180 deg, index calves against mattress edge:
      (xform-geodesic-wayfinder-base :goal-xyz [2.4 -1.6 0.0])
      (specialize-env-cluttered-interior)
      (xform-stand-and-turn-at-fixture :turn-yaw-rad 3.14159 :standoff-clearance-mm 30.0)
      ;; Sit on long side, then execute 90-deg coupled pendulum sweep to lie on back ALONG bed:
      (xform-bed-longside-sit-and-lie-supine :headboard-direction :left)
      (specialize-with-bed-headboard-pillow-align :headboard-clearance-mm 85.0)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; 3. In Bed Along Long Side: Roll to Side -> Stomach -> Back (Zero-Translation Shrimp) -> Rise!
(defpatch patch-bed-roll-side-stomach-back-and-rise [supine-bed-state]
  (-> supine-bed-state
      ;; Roll Supine -> Side -> Prone (Stomach) -> Supine (Back) pinned to bed centerline:
      (xform-bed-in-place-roll-sequence :roll-direction :clockwise-right :bed-width-mm 900.0)
      (specialize-with-shrimp-centerline-recentering :cot-width-mm 900.0 :shrimp-stroke-mm 149.0)
      ;; Reverse 90-deg pendulum: drop lower legs over long side, pivot up to seated, and stand!
      (xform-bed-supine-to-longside-stand :headboard-direction :left)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-complete-stool-and-bed-lifecycle-soa
  (let [sit-high-stool   (patch-turn-and-sit-high-stool nominal-biped-gait)
        lie-along-bed    (patch-dismount-stool-and-mount-bed-longside sit-high-stool)
        roll-and-stand   (patch-bed-roll-side-stomach-back-and-rise lie-along-bed)]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(oneshot  sit-high-stool) ; Stand by stool -> Turn -> Sit on high stool/chair -> Core balance
         (dwell-ms 3000)
         (oneshot  lie-along-bed)  ; Dismount stool -> Walk to bed long side -> Turn -> Sit -> 90-deg sweep to Supine
         (dwell-ms 2000)
         (oneshot  roll-and-stand)]))) ; Roll to Side -> Stomach -> Back (in-place) -> 90-deg sweep to Sit -> Stand
