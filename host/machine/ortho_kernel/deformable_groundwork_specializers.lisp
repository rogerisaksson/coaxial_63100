;; =============================================================================
;; FILE: deformable_groundwork_specializers.lisp
;; DOMAIN: Composable Groundwork Specializers for Mattresses, Deep Snow & Trampolines
;; REQUIRES: deformable_substrate_core.lisp, ground_rolling_and_transitions.lisp
;; =============================================================================
(defpackage :ortho-dsp.deformable-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.postures-core
        :ortho-dsp.posture-xforms
        :ortho-dsp.ground-core
        :ortho-dsp.ground-xforms
        :ortho-dsp.deformable-core)
  (:export ;; A. Universal Deformable Groundwork Wrapper
           #:xform-deformable-groundwork-base
           ;; B. Substrate-Specific Plug-In Specializers
           #:specialize-substrate-deep-snow-bekker
           #:specialize-substrate-viscoelastic-mattress
           #:specialize-substrate-trampoline-harmonic
           ;; C. Specialized Compliant Egress Maneuvers
           #:specialize-egress-snow-trench-compaction
           #:specialize-egress-trampoline-elastic-kip))
(in-package :ortho-dsp.deformable-xforms)

;; =============================================================================
;; A. UNIVERSAL DEFORMABLE GROUNDWORK BASELINE (SELF-ADAPTING TO ANY SOFT SURFACE)
;; =============================================================================
(defmodifier xform-deformable-groundwork-base [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 45.0)
      (identify-rheological-work-loop :plastic-eta-ceiling 0.16 :harmonic-eta-floor 0.62)
      (decouple-sinkage-from-inekf-odometry)
      (regulate-mohr-coulomb-patch-pressure :max-contact-pressure-kpa 22.0)
      (execute-two-stage-snow-consolidation)
      (extract-limb-from-posthole-cavity)
      (lock-symplectic-membrane-resonator :interaction-mode :conjugate-wave-absorber)))


;; =============================================================================
;; B. SUBSTRATE-SPECIFIC COMPOSABLE SPECIALIZERS (SNOW, MATTRESS, TRAMPOLINE)
;; =============================================================================

;; 1. DEEP SNOW / GRANULAR SUBSTRATE SPECIALIZER (`:bekker-plastic`)
;; - Spreads weight across full shins (`:anterior-tibial`) and forearms (`:olecranon-ulnar`)
;;   so the robot never punches narrow wrists or toes deep into the snowpack.
;; - Uses 2-stage snow tamping before kneeling, sitting, or standing up.
;; - During 360 deg rolling in snow, lifts knees and elbows slightly toward the
;;   midline ("plough-clearance") so limbs don't scoop or anchor into the snowbank!
(defmodifier specialize-substrate-deep-snow-bekker
  [state &key (snow-crust-yield-kpa 14.0) (max-sinkage-depth-mm 260.0)]
  (-> state
      (regulate-mohr-coulomb-patch-pressure
        :max-contact-pressure-kpa snow-crust-yield-kpa
        :area-expansion-gain 3.2)
      (execute-two-stage-snow-consolidation
        :tamp-preload-fz-ratio 0.28
        :consolidated-stiffness-target-npmm 42.0
        :max-tamp-sinkage-mm max-sinkage-depth-mm)
      (extract-limb-from-posthole-cavity
        :min-cavity-depth-for-axial-pull-mm 35.0
        :rim-clearance-margin-mm 40.0)
      ;; Modify rolling & 90/90 leg-sweep height to clear displaced snow berms:
      (modulate-rolling-berm-clearance
        :axial-hip-lift-bias-mm 65.0
        :keep-distal-limbs-above-snow-plane true)))

;; 2. VISCOELASTIC MATTRESS / FOAM PAD SPECIALIZER (`:kelvin-viscoelastic`)
;; - Compensates for the "Winkler Gravity Trough": because the heavy pelvis sinks
;;   deeper into the mattress than the feet or hands, the mattress surface slopes
;;   inward toward the hips.
;; - Automatically tilts the contact normal cones inward toward the pelvic depression
;;   and adds phase-lead to cancel slow viscoelastic foam creep.
(defmodifier specialize-substrate-viscoelastic-mattress
  [state &key (mattress-creep-time-constant-s 0.45) (trough-gradient-comp-gain 0.85)]
  (-> state
      (specialize-with-viscoelastic-sink-compensation)
      (compensate-winkler-depression-basin
        :centroid-sinkage-source :ischial-and-pelvic-sinkage-m
        :inward-normal-tilt-gain trough-gradient-comp-gain
        :creep-phase-lead-s mattress-creep-time-constant-s
        ;; When sitting on buttocks on a soft mattress, hips sink ~90mm lower than heels;
        ;; automatically flex knees +0.35 rad in 1-Jet origin so heels don't dig in and tip torso back!
        :differential-sinkage-knee-relief-rad +0.35)))

;; 3. TRAMPOLINE / HIGH-Q SPRING-BED SPECIALIZER (`:duffing-harmonic`)
;; - Cancels cross-limb wave coupling: when one knee or hand presses the trampoline
;;   down, the surrounding fabric pulls the OTHER support points inward toward the center
;;   and transmits a transverse elastic wave.
;; - Uses `conjugate-wave-absorber` during descent, sitting, lying, and rolling so the
;;   robot settles dead-beat onto the oscillating fabric without bouncing.
(defmodifier specialize-substrate-trampoline-harmonic
  [state &key (membrane-centripetal-pull-comp true) (mode :absorb-and-stabilize)]
  (-> state
      (lock-symplectic-membrane-resonator
        :interaction-mode (if (eq mode :elastic-bounce-egress)
                              :resonant-elastic-harvest
                              :conjugate-wave-absorber))
      ;; Compensate for trampoline fabric stretching radially toward the center springs
      ;; (which creates an inward tangential shear force toward the bed centroid):
      (when-modifier membrane-centripetal-pull-comp
        (compensate-membrane-catenary-tension
          :radial-shear-feedforward-gain 0.78
          :widen-stance-polygon-scale 1.22))))


;; =============================================================================
;; C. SPECIALIZED DEFORMABLE EGRESS MANEUVERS (SNOW PACK-UP & TRAMPOLINE KIP-UP)
;; =============================================================================

;; 1. Deep-Snow Broad-Hull Compaction Egress (Prone/Supine -> Forearm-Shin Pack -> Stand)
;; In deep snow, pushing up on one hand (Turkish Get-Up) punches the arm 40cm into the drift.
;; Instead, the robot rolls to prone/all-fours on full forearms + full shins (`:olecranon-ulnar` +
;; `:anterior-tibial`), tamps a flat snow shelf with both shins, steps one flat foot onto the
;; pre-compacted knee print (stepping in its own compacted track!), and rises!
(defmodifier specialize-egress-snow-trench-compaction [snow-floor-state]
  (-> snow-floor-state
      (specialize-substrate-deep-snow-bekker :snow-crust-yield-kpa 14.0)
      ;; 1. Roll to prone and press up on broad forearms + full shins (maximum surface area):
      (xform-continuous-ground-roll :direction :clockwise-right :angle-rad 3.14159)
      (specialize-prone-cobra-press-up :intermediate-sphinx-dwell-ms 450.0)
      ;; 2. Step leading foot directly into the pre-compacted snow imprint just vacated by the knee!
      (place-foot-in-compacted-knee-imprint
        :lead-foot :calcaneus-l
        :reuse-sintered-tibial-pocket :tibial-tuberosity-l
        :pushoff-with-bilateral-carpal-on-thighs true) ; Brace hands on own thigh to avoid snow punch!
      (xform-floor-to-upright-egress)))

;; 2. Trampoline Elastic Bounce-to-Stand / Seat-Drop Kip Egress
;; Instead of fighting a wobbly trampoline to crawl up via knees, uses the trampoline's
;; own harmonic stored energy:
;; From Supine or Seated Buttocks (`:ischial-bilateral`), pumps arms/hips into the bed
;; at phase pi, rides the upward trampoline rebound into a 350mm weightless hop, tucks
;; feet underneath the pelvis in mid-air (`w_all == 0`), and lands standing in dead-beat
;; conjugate damping!
(defmodifier specialize-egress-trampoline-elastic-kip
  [trampoline-floor-state &key (pump-cycles 1) (target-hop-z-mm 340.0)]
  (-> trampoline-floor-state
      (specialize-substrate-trampoline-harmonic :mode :elastic-bounce-egress)
      (execute-harmonic-seat-drop-to-feet
        :preparation-pump-bars pump-cycles
        :launch-manifold [:ischial-bilateral :dorsal-supine-continuum :anterior-tibial-bilateral]
        :target-aerial-clearance-m (* 0.001 target-hop-z-mm)
        ;; During the 260ms aerial phase when trampoline launches the body upward:
        :midair-fast-leg-tuck
          (:target-1jet-posture :plantar-upright
           :coxofemoral-tuck-slew-radps 12.5
           :tibiofemoral-tuck-slew-radps 14.0)
        ;; Upon plantar touchdown, immediately switch back to conjugate wave absorber
        ;; so the robot sticks the landing without a second bounce!
        :touchdown-damping-handover :conjugate-wave-absorber)
      (xform-balance-biped-zmp)
      (ignite-cyclic-clock nominal-biped-gait :bars 1)))
