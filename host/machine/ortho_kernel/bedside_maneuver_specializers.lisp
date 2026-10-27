;; =============================================================================
;; FILE: bedside_maneuver_specializers.lisp
;; DOMAIN: Composable Transformers for Docking, Stool/Chair Sitting & Bed Lifecycle
;; REQUIRES: bedside_and_stool_core.lisp, somatic_interaction_specializers.lisp
;; =============================================================================
(defpackage :ortho-dsp.bedside-xforms
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
        :ortho-dsp.deformable-core
        :ortho-dsp.deformable-xforms
        :ortho-dsp.terrestrial-core
        :ortho-dsp.bedside-core)
  (:export ;; 1. Stand Alongside, Turn 180 deg & Tactile Index
           #:xform-stand-and-turn-at-fixture
           ;; 2. Universal Chair, Overheight Chair & Backless Stool Ingress/Egress
           #:xform-sit-on-chair-or-stool
           #:xform-stand-from-chair-or-stool
           ;; 3. Bed Long-Side Sit -> 90-Deg Rotate to Supine -> Side -> Prone -> Supine -> Rise
           #:xform-bed-longside-sit-and-lie-supine
           #:xform-bed-in-place-roll-sequence
           #:xform-bed-supine-to-longside-stand
           ;; 4. Composable Specializers
           #:specialize-with-stool-active-core-balance
           #:specialize-with-bed-headboard-pillow-align
           #:specialize-with-shrimp-centerline-recentering))
(in-package :ortho-dsp.bedside-xforms)

;; =============================================================================
;; A. STAND ALONGSIDE, TURN AROUND & LOCK POPLITEAL CALF DATUM
;; =============================================================================
(defmodifier xform-stand-and-turn-at-fixture
  [state &key (turn-yaw-rad 3.14159)
              (turn-style :single-anchor-arc)    ; :single-anchor-arc | :cross-under-corkscrew
              (standoff-clearance-mm 35.0)
              (probe-capsules [:shank-crural-l :shank-crural-r])]
  (-> state
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)
      (specialize-with-nullspace-limb-retraction :retraction-influence-radius-mm 260.0)
      (specialize-with-blindspot-memory-persistence :max-memory-horizon-s 60.0)
      (lock-horizon-gaze :room-pan-yaw-rad 0.0)
      ;; 1. Execute capsule-guarded pivot right next to the chair, stool, or bed long-side:
      (case turn-style
        (:cross-under-corkscrew
          (pivot-cross-under-corkscrew :target-yaw-rad turn-yaw-rad :coronal-cross-offset-mm -130.0))
        (otherwise
          (pivot-single-anchor-arc :target-yaw-rad turn-yaw-rad :standoff-x-mm standoff-clearance-mm)))
      ;; 2. Lightly brush calves against the fixture edge behind the knees to zero out any
      ;;    odometry drift before descending or vaulting!
      (index-popliteal-standoff-datum
        :target-calf-touch-force-n 9.0
        :rebound-clearance-mm 12.0
        :probe-capsules probe-capsules)))


;; =============================================================================
;; B. UNIVERSAL CHAIR, OVERHEIGHT CHAIR & BACKLESS STOOL (PALL) INGRESS/EGRESS
;; =============================================================================

;; 1. Sit on Normal Chair, Overheight Chair, or Backless Stool (Pall)
;; - Automatically morphs Plantar -> Equinus -> Thigh-Bevel Slide if height > leg length.
;; - Automatically engages `enforce-backless-stool-rim-zcbf` if no backrest is present!
(defmodifier xform-sit-on-chair-or-stool
  [state &key (slide-speed-mps 0.15) (use-carpal-hand-boost true)]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 845.0)          ; her leg, hip to sole
      (enforce-backless-stool-rim-zcbf :rim-safety-margin-mm 38.0 :auto-detect-missing-backrest true)
      (xform-somatic-sit-ingress)
      (specialize-for-overheight-seat :direction :ingress :slide-speed-mps slide-speed-mps)
      (when-modifier use-carpal-hand-boost
        (specialize-with-carpal-seat-boost :max-arm-push-share 0.22))
      (specialize-posture-dangling-crural-hang :passive-ankle-droop-rad -0.24
                                               :pendulum-natural-freq-hz 0.91)))  ; her shank's swing

;; 2. Stand Up from Normal Chair, Overheight Chair, or Backless Stool
(defmodifier xform-stand-from-chair-or-stool
  [seated-state &key (slide-speed-mps 0.15)]
  (-> seated-state
      (wake-inverter-closed-loop)
      (specialize-with-carpal-seat-boost :max-arm-push-share 0.24)
      (specialize-for-overheight-seat :direction :egress :slide-speed-mps slide-speed-mps)
      (xform-somatic-sit-egress)))


;; =============================================================================
;; C. BED LONG-SIDE INGRESS, 90-DEG ALIGNMENT, IN-PLACE ROLLING & EGRESS
;; =============================================================================

;; 1. Sit on Long Side of Bed -> Rotate 90 deg to Lie Supine ALONG the Bed's Long Side
(defmodifier xform-bed-longside-sit-and-lie-supine
  [standing-at-bedside-state &key (headboard-direction :left)]
  (-> standing-at-bedside-state
      ;; Step A: Settle onto the long-side edge of the mattress and scoot 160mm inward
      ;; so the pelvis has a wide support margin inside the mattress edge:
      (xform-deformable-groundwork-base)
      (specialize-substrate-viscoelastic-mattress :trough-gradient-comp-gain 0.88)
      (xform-sit-on-chair-or-stool :slide-speed-mps 0.14 :use-carpal-hand-boost true)
      ;; Step B: Execute the coupled 90-deg Pitch-Roll-Yaw Trochanteric Pendulum:
      ;; Lowers torso toward headboard while sweeping both legs up over the mattress edge
      ;; and rotating +90/-90 deg onto the back (`:dorsal-supine-continuum`) along the bed!
      (drive-transverse-longitudinal-bed-pendulum
        :direction :ingress-to-longitudinal-supine
        :bed-headboard-side headboard-direction
        :pendulum-coupling-gain 0.92
        :maneuver-duration-ms 1350.0)
      ;; Step C: Center spine along mattress midline and settle into supine 1-Jet posture:
      (drive-centerline-pinned-mattress-roll
        :target-roll-state :supine-back
        :roll-chirality :clockwise-right)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (specialize-posture-recumbent-archetype :archetype :supine-aligned :pillow-height-mm 65.0)))

;; 2. In-Place Bed Rolling Sequence: Back -> Side -> Stomach -> Back (Within Bed Width!)
;; Uses `drive-centerline-pinned-mattress-roll` + Helical Axial Wave so the robot
;; hip-escapes ("shrimps") toward the bed centerline during each rotation and NEVER
;; drifts over the mattress edge!
(defmodifier xform-bed-in-place-roll-sequence
  [supine-in-bed-state &key (roll-direction :clockwise-right) (bed-width-mm 900.0)]
  (-> supine-in-bed-state
      (wake-inverter-closed-loop)
      (xform-deformable-groundwork-base)
      (specialize-substrate-viscoelastic-mattress)
      ;; Phase 1: Supine (Back) -> 90 deg onto Lateral Side (`:trochanter-gluteal-lateral`)
      (drive-centerline-pinned-mattress-roll
        :target-roll-state :lateral-side
        :roll-chirality roll-direction
        :bed-width-mm bed-width-mm)
      (drive-helical-axial-roll-wave
        :roll-direction roll-direction
        :target-roll-angle-rad 1.570796          ; +90 deg to side-lying
        :propagation-leader :pelvic-led)
      ;; Phase 2: Lateral Side -> +90 deg onto Flat Stomach (`:ventral-prone-continuum`)
      (drive-centerline-pinned-mattress-roll
        :target-roll-state :prone-stomach
        :roll-chirality roll-direction
        :bed-width-mm bed-width-mm)
      (drive-helical-axial-roll-wave
        :roll-direction roll-direction
        :target-roll-angle-rad 1.570796          ; +90 deg to prone on stomach
        :propagation-leader :thoracic-led)
      (xform-floor-to-prone-flat)
      ;; Phase 3: Flat Stomach -> -180 deg (via Side) back to Supine on Back along bed!
      (drive-centerline-pinned-mattress-roll
        :target-roll-state :supine-back
        :roll-chirality (if (eq roll-direction :clockwise-right)
                            :counter-clockwise-left
                            :clockwise-right)
        :bed-width-mm bed-width-mm)
      (drive-helical-axial-roll-wave
        :roll-direction (if (eq roll-direction :clockwise-right)
                            :counter-clockwise-left
                            :clockwise-right)
        :target-roll-angle-rad 3.14159           ; 180 deg return to supine back
        :propagation-leader :pelvic-led)
      (xform-floor-to-supine-flat :terminal-high-z false)))

;; 3. Rise from Longitudinal Supine in Bed -> 90-Deg Pendulum to Long-Side Sit -> Stand Up
(defmodifier xform-bed-supine-to-longside-stand
  [supine-in-bed-state &key (headboard-direction :left)]
  (-> supine-in-bed-state
      (wake-inverter-closed-loop)
      (xform-deformable-groundwork-base)
      (specialize-substrate-viscoelastic-mattress)
      ;; Step A: Reverse 90-deg Cantilever Pendulum:
      ;; Rolls slightly toward the open room side, sweeps lower legs over the mattress
      ;; long edge, and uses the dropping legs' gravity moment + elbow/carpal push to
      ;; pivot 90 deg up into transverse seated posture on the bed's long side!
      (drive-transverse-longitudinal-bed-pendulum
        :direction :egress-to-transverse-seated
        :bed-headboard-side headboard-direction
        :pendulum-coupling-gain 0.94
        :maneuver-duration-ms 1280.0)
      ;; Step B: Slide anteriorly off mattress edge until plantarflexed toes/feet plant
      ;; on the floor, then rise to upright bipedal balance!
      (xform-stand-from-chair-or-stool :slide-speed-mps 0.14)))


;; =============================================================================
;; D. COMPOSABLE SPECIALIZATION EXTENSIONS (PLUG-IN MODIFIERS)
;; =============================================================================

;; 1. Active Core-Braced Stool Equilibrium Specializer (For Narrow/Wobbly Barstools)
;; Increases lumbar/thoracic angular momentum damping and recruits dangling legs as
;; minor coronal reaction pendulums when sitting on a narrow, backless high stool.
(defmodifier specialize-with-stool-active-core-balance
  [stool-state &key (seat-diameter-mm 320.0)]
  (-> stool-state
      (enforce-backless-stool-rim-zcbf :rim-safety-margin-mm (* 0.12 seat-diameter-mm))
      (configure-balance-feedback
        :primary-actuators [:l4-l5-lumbar :c7-t12-thoracic :iliac-coronal :coxofemoral]
        :dcm-proportional-gain 2.4
        :angular-momentum-damping 0.92)))

;; 2. Bed Headboard & Pillow Longitudinal Alignment Specializer
;; During the 90-deg sit-to-supine pendulum sweep, adjusts the longitudinal scoot
;; component along the bed's major axis so the back of her skull (`:occipital`) lands
;; centered on the pillow quadric with >= 80 mm clearance to the headboard wall!
(defmodifier specialize-with-bed-headboard-pillow-align
  [bed-state &key (headboard-clearance-mm 85.0) (pillow-elevation-mm 65.0)]
  (-> bed-state
      (modulate-longitudinal-bed-datum
        :cranial-capsule :occipital
        :headboard-wall-margin-m (* 0.001 headboard-clearance-mm)
        :pillow-neck-flexion-bias-rad (* 0.0032 pillow-elevation-mm))))

;; 3. Explicit Shrimp / Hip-Escape Amplitude Specializer (For Ultra-Narrow Cots/Bunks)
;; Tunes the stroke length of the contralateral hip-escape so the robot can roll
;; 180 deg inside a narrow camping cot or bunk bed without touching the rails: her 609 mm
;; on her back and its margins, her spine's lateral stop's stroke.
(defmodifier specialize-with-shrimp-centerline-recentering
  [bed-roll-state &key (cot-width-mm 829.0) (shrimp-stroke-mm 149.0)]
  (-> bed-roll-state
      (drive-centerline-pinned-mattress-roll
        :bed-width-mm cot-width-mm
        :edge-safety-margin-mm 110.0)
      (modulate-stick-slip-sled-ratchet
        :anchor-patches [:plantar-bilateral :olecranon-ulnar-sphinx]
        :sled-patches   [:ischial-bilateral :trochanter-gluteal-lateral]
        :vault-unweight-ratio 0.65
        :rowing-stroke-length-mm shrimp-stroke-mm)))
