;; =============================================================================
;; FILE: deformable_groundwork_patches.lisp
;; DOMAIN: Complete Groundwork Schedules on Mattress, Deep Snow & Trampoline
;; =============================================================================
(defpackage :ortho-dsp.deformable-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.ground-core
        :ortho-dsp.ground-xforms
        :ortho-dsp.deformable-core
        :ortho-dsp.deformable-xforms)
  (:export #:patch-groundwork-on-viscoelastic-mattress
           #:patch-groundwork-in-deep-snow
           #:patch-groundwork-on-trampoline
           #:compile-deformable-groundwork-suite-soa))
(in-package :ortho-dsp.deformable-patches)

;; -----------------------------------------------------------------------------
;; 1. MATTRESS / SOFT BED SCHEDULE:
;; Upright -> Knees -> Knees+Hands -> Buttocks (Winkler trough relief) ->
;; Supine -> 360 Roll (Archimedean trough-escape pushoff) -> Prone -> Rise
;; -----------------------------------------------------------------------------
(defpatch patch-groundwork-on-viscoelastic-mattress [base-gait]
  (-> base-gait
      (transducer-unshod)
      (xform-deformable-groundwork-base)
      (specialize-substrate-viscoelastic-mattress :trough-gradient-comp-gain 0.88)
      ;; Descent through knees -> all-fours -> buttocks -> supine -> roll -> prone -> rise:
      (xform-upright-to-kneeling)
      (xform-kneeling-to-hands-and-knees)
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support true)
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support false)
      (xform-floor-to-supine-flat)
      (xform-continuous-ground-roll :direction :clockwise-right :angle-rad 6.28318)
      (xform-floor-to-prone-flat)
      (specialize-prone-cobra-press-up)
      (xform-floor-to-upright-egress)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; 2. DEEP SNOW SCHEDULE (BEKKER TERRAMECHANICS):
;; Upright -> 2-Stage Tamp Kneel -> Broad Shin+Forearm Quad -> Buttocks ->
;; Supine/Prone Snow-Plough Roll -> Step-in-Own-Compacted-Knee-Print Egress!
;; -----------------------------------------------------------------------------
(defpatch patch-groundwork-in-deep-snow [base-gait]
  (-> base-gait
      (transducer-viscoelastic-flat)
      (xform-deformable-groundwork-base)
      (specialize-substrate-deep-snow-bekker :snow-crust-yield-kpa 14.0 :max-sinkage-depth-mm 260.0)
      (specialize-descent-staggered-lunge :lead-stance-foot :left)
      (xform-upright-to-kneeling)
      (xform-kneeling-to-hands-and-knees)
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support true)
      (xform-floor-to-supine-flat)
      (xform-continuous-ground-roll :direction :counter-clockwise-left :angle-rad 3.14159)
      (xform-floor-to-prone-flat)
      ;; Rise using compacted tibial snow pocket + hands braced on own thighs:
      (specialize-egress-snow-trench-compaction)
      (limiter-low-mu-shear)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; 3. TRAMPOLINE SCHEDULE (HIGH-Q DUFFING HARMONIC MEMBRANE):
;; Upright -> Dead-Beat Wave-Absorbing Kneel & Sit -> Anti-Phase Roll ->
;; Elastic Seat-Drop / Back-Drop Kip-Up Launch Straight to Upright Standing!
;; -----------------------------------------------------------------------------
(defpatch patch-groundwork-on-trampoline [base-gait]
  (-> base-gait
      (transducer-unshod)
      (xform-deformable-groundwork-base)
      ;; Phase A: Absorb trampoline oscillation via conjugate impedance matching
      (specialize-substrate-trampoline-harmonic :mode :absorb-and-stabilize)
      (xform-upright-to-kneeling)
      (xform-kneeling-to-hands-and-knees)
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support true)
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support false)
      (xform-floor-to-supine-flat)
      (xform-continuous-ground-roll :direction :clockwise-right :angle-rad 6.28318)
      ;; Phase B: Switch to resonant energy harvesting and bounce straight to standing!
      (specialize-egress-trampoline-elastic-kip :pump-cycles 1 :target-hop-z-mm 340.0)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-deformable-groundwork-suite-soa
  (let [mattress-run   (patch-groundwork-on-viscoelastic-mattress nominal-biped-gait)
        deep-snow-run  (patch-groundwork-in-deep-snow nominal-biped-gait)
        trampoline-run (patch-groundwork-on-trampoline nominal-biped-gait)]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(oneshot mattress-run)
         (oneshot deep-snow-run)
         (oneshot trampoline-run)])))
