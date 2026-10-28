;; =============================================================================
;; FILE: floor_lifecycle_patches.lisp
;; DOMAIN: Complete Floor Choreography Schedule (Stand -> Kneel -> Sit -> Roll -> Rise)
;; =============================================================================
(defpackage :ortho-dsp.ground-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.ground-core
        :ortho-dsp.ground-xforms)
  (:export #:patch-descent-to-knees-and-all-fours
           #:patch-transition-to-buttocks-and-supine
           #:patch-roll-supine-to-prone-and-back
           #:patch-rise-from-prone-to-standing
           #:compile-complete-groundwork-soa))
(in-package :ortho-dsp.ground-patches)

;; Phase 1: Upright -> Staggered Lunge -> Tall Knees -> Knees with Hands & Arms
(defpatch patch-descent-to-knees-and-all-fours [base-gait]
  (-> base-gait
      (transducer-unshod)
      (xform-balance-biped-zmp)
      (specialize-descent-staggered-lunge :lead-stance-foot :left)
      (xform-upright-to-kneeling)
      (xform-kneeling-to-hands-and-knees)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; Phase 2: All-Fours -> Buttocks with Hands/Arms -> Buttocks Only -> Flat on Back (Supine)
(defpatch patch-transition-to-buttocks-and-supine [kneeling-state]
  (-> kneeling-state
      ;; 90/90 hip sweep into buttocks with posterior carpal tripod:
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support true)
      (specialize-buttocks-with-carpal-tripod :tripod-mode :posterior-sunbather-tripod :arm-load-share 0.30)
      ;; Release hands to balance purely on buttocks (`:floor-ischial-unbraced`):
      (xform-to-floor-buttocks-seated :with-hands-and-arms-support false)
      ;; Unroll spine smoothly into flat supine position on the floor/mat:
      (xform-floor-to-supine-flat :terminal-high-z false)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; Phase 3: Roll 180 deg Supine -> Prone (Flat on Stomach), then 360 deg Roll Across Floor/Mattress
;; Automatically adapts to Hard Floor (spine rounding + bony landmark yield) or
;; Soft Mattress (contralateral heel/palm trough-escape pushoff)!
(defpatch patch-roll-supine-to-prone-and-back [supine-state]
  (-> supine-state
      ;; Roll 180 deg from Back to flat on Stomach (:ventral-prone-continuum):
      (xform-continuous-ground-roll :direction :clockwise-right
                                    :angle-rad 3.14159
                                    :leader :pelvic-led)
      (xform-floor-to-prone-flat)
      ;; Execute a full 360 deg barrel roll across the surface:
      (xform-continuous-ground-roll :direction :clockwise-right
                                    :angle-rad 6.28318
                                    :leader :thoracic-led)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; Phase 4: Prone on Stomach -> Cobra Press-Up -> Hands & Knees -> Turkish Get-Up / Lunge -> Upright Stand
(defpatch patch-rise-from-prone-to-standing [prone-state]
  (-> prone-state
      (specialize-prone-cobra-press-up :intermediate-sphinx-dwell-ms 300.0)
      (specialize-egress-turkish-getup-spiral :support-side :right)
      (xform-floor-to-upright-egress)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION (COMPLETE GROUNDWORK LIFECYCLE)
;; -----------------------------------------------------------------------------
(defpatch compile-complete-groundwork-soa
  (let [kneel-and-quad (patch-descent-to-knees-and-all-fours nominal-biped-gait)
        sit-and-supine (patch-transition-to-buttocks-and-supine kneel-and-quad)
        roll-sequence  (patch-roll-supine-to-prone-and-back sit-and-supine)
        stand-egress   (patch-rise-from-prone-to-standing roll-sequence)]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(oneshot  kneel-and-quad) ; Upright -> Lunge -> Tall Knees -> Knees + Hands/Arms
         (dwell-ms 1500)
         (oneshot  sit-and-supine) ; 90/90 Sweep -> Buttocks + Hands -> Buttocks Only -> Flat on Back
         (dwell-ms 1500)
         (oneshot  roll-sequence)  ; Roll Back -> Flat on Stomach -> 360 deg Roll (Hard/Soft Adaptive)
         (dwell-ms 1500)
         (oneshot  stand-egress)]))) ; Cobra Press-Up -> Knees & Hands -> Lunge Stand -> Walk Clock
