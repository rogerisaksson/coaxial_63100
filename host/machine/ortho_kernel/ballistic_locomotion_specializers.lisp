;; =============================================================================
;; FILE: ballistic_locomotion_specializers.lisp
;; DOMAIN: Composable Transformers for Walk->Run Spectrum, Rapid Braking,
;;         Standing Two-Footed Jumps & In-Stride Running/Walking Leaps
;; REQUIRES: ballistic_and_velocity_core.lisp, universal_substrate_overlay.lisp
;; =============================================================================
(defpackage :ortho-dsp.ballistic-xforms
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
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.substrate-overlay
        :ortho-dsp.ballistic-core)
  (:export ;; 1. Continuous Velocity Spectrum: Walk <-> Brisk Walk <-> Double-Time <-> Run
           #:xform-continuous-velocity-spectrum
           ;; 2. Substrate-Gated Rapid Braking (Run/Double-Time -> Walk/Stand)
           #:xform-substrate-gated-rapid-brake
           ;; 3. Standing Two-Footed Jump (From a Standstill, Feet Together)
           #:xform-standing-bilateral-jump
           ;; 4. In-Stride Single-Leg Leap during Walk or Run (Take a Leap While Walking/Running)
           #:xform-in-stride-unilateral-leap
           ;; 5. Composable Plug-In Specializers
           #:specialize-jump-onto-elevated-platform
           #:specialize-leap-over-sdf-gap-or-hurdle
           #:specialize-brake-with-aerodynamic-and-arm-counter
           #:specialize-run-with-elastic-achilles-q-boost))
(in-package :ortho-dsp.ballistic-xforms)

;; =============================================================================
;; A. CONTINUOUS VELOCITY SPECTRUM (WALK -> BRISK WALK -> DOUBLE-TIME -> RUN)
;; =============================================================================
(defmodifier xform-continuous-velocity-spectrum
  [state &key (regime :normal-walk)              ; :normal-walk | :brisk-walk | :sprangmarsch | :slip-run
              (custom-speed-mps nil)]
  (let [[default-speed bpm-target posture-key]
        (case regime                             ; the text's Froude numbers on her leg:
          (:normal-walk  '(1.218 112.9 :plantar-upright))         ; (1.25 110.0)
          (:brisk-walk   '(1.803 135.5 :locomotion-brisk-walk))   ; (1.85 132.0)
          (:sprangmarsch '(2.582 158.0 :locomotion-sprangmarsch)) ; (2.65 154.0)
          (:slip-run     '(3.995 182.7 :locomotion-slip-run)))    ; (4.10 178.0)
        v-cmd (or custom-speed-mps default-speed)]
    (-> state
        (blend-manifold-posture-jet :atlas 'ballistic-1jet-posture-atlas)
        (quantize-grid :bpm (:set bpm-target))
        (modulate-froude-pendulum-slip-homotopy :target-velocity-mps v-cmd)
        (conserve-aerial-centroidal-momentum :raibert-velocity-gain 0.14)
        (xform-universal-spatial-awareness)
        (specialize-with-self-collision-manifold :min-malleolus-gap-mm 12.0 :min-arm-torso-gap-mm 20.0)
        (enforce-dcm-barrier-zcbf :barrier-gamma 16.0 :min-capture-margin-mm 18.0))))

;; =============================================================================
;; B. SUBSTRATE-GATED RAPID BRAKING (A FAST BRAKE TO A WALK IF THE GROUND ALLOWS)
;; =============================================================================
(defmodifier xform-substrate-gated-rapid-brake
  [running-or-jogging-state &key (target-walk-speed-mps 1.169)] ; at her size (1.20)
  (-> running-or-jogging-state
      (configure-momentum-observer :observer-bandwidth-hz 48.0)
      ;; Evaluates live friction cone mu_eff, deformable snow/mattress sinkage, and
      ;; DC-bus voltage headroom to execute either a 2-step high-mu power brake or a
      ;; multi-step low-mu ABS stutter-step down-ramp into `nominal-biped-gait`:
      (govern-mu-and-regen-braking-horizon
        :target-terminal-velocity-mps target-walk-speed-mps
        :max-regen-bus-voltage-v 57.5
        :min-cwc-shear-margin-ratio 0.82)
      (xform-continuous-velocity-spectrum :regime :normal-walk :custom-speed-mps target-walk-speed-mps)))

;; =============================================================================
;; C. STANDING TWO-FOOTED JUMP (FROM A STANDSTILL, FEET TOGETHER)
;; =============================================================================
(defmodifier xform-standing-bilateral-jump
  [static-or-walking-state &key (jump-height-z-mm 215.0) ; her knees' launch from her crouch (340.0)
                                (jump-distance-x-mm 0.0) ; 0 = vertical hop; >0 = standing broad jump!
                                (countermovement-crouch-mm 180.4)] ; at her size (190.0)
  (-> static-or-walking-state
      ;; 1. If walking, center ZMP in double-support and lock left/right phase offsets to 0.0:
      (xform-balance-biped-zmp)
      (blend-manifold-posture-jet :atlas 'ballistic-1jet-posture-atlas)
      ;; 2. Execute 4-phase Stretch-Shortening Cycle (Coil -> Back-EMF Launch -> Tuck -> Absorb):
      (drive-stretch-shortening-ballistic-impulse
        :launch-topology :bilateral-in-phase
        :target-apex-height-dz-mm jump-height-z-mm
        :target-horizontal-span-dx-mm jump-distance-x-mm
        :countermovement-depth-mm countermovement-crouch-mm
        :arm-flywheel-assist true)
      (conserve-aerial-centroidal-momentum :raibert-velocity-gain 0.12)
      (enforce-dcm-barrier-zcbf :barrier-gamma 16.0 :min-capture-margin-mm 20.0)))

;; =============================================================================
;; D. IN-STRIDE UNILATERAL LEAP / BOUND (TAKE A LEAP WHILE WALKING AND RUNNING)
;; =============================================================================
(defmodifier xform-in-stride-unilateral-leap
  [gait-state &key (takeoff-leg :auto-next-stance)
                   (leap-apex-z-mm 75.0)         ; one knee's launch, hers (260.0)
                   (leap-span-x-mm 638.0)]       ; from her double-time march's 2.58 m/s (1350.0)
  (-> gait-state
      ;; Wait for penultimate single-support on `takeoff-leg` without stopping the PLL clock!
      (drive-stretch-shortening-ballistic-impulse
        :launch-topology :unilateral-split-leap
        :target-apex-height-dz-mm leap-apex-z-mm
        :target-horizontal-span-dx-mm leap-span-x-mm
        :countermovement-depth-mm 109.2          ; Fast elastic SLIP compression during single stance. Hers (115.0)
        :arm-flywheel-assist true)
      (conserve-aerial-centroidal-momentum
        :raibert-velocity-gain 0.16
        :pre-touchdown-leg-retraction-rate-mps -0.633) ; Match ground speed on landing foot to continue running! (-0.65)
      (enforce-dcm-barrier-zcbf :barrier-gamma 17.0 :min-capture-margin-mm 18.0)))


;; =============================================================================
;; E. COMPOSABLE SPECIALIZATION EXTENSIONS (PLUG-IN MODIFIERS)
;; =============================================================================

;; 1. Box-Jump / Elevated Platform Landing Specializer (For a Two-Footed Jump Up onto a Ledge/Stair)
;; Adjusts the mid-air knee tuck and shifts the expected touchdown Z-plane upward
;; by `+platform-elevation-z-mm` so the Momentum Observer latches on the elevated surface!
(defmodifier specialize-jump-onto-elevated-platform
  [jump-state &key (platform-elevation-z-mm +77.0)    ; her launch at the text's angle, 197 up less 120 (+380.0)
                   (platform-forward-x-mm 205.0)]     ; and 205 ahead (520.0)
  (-> jump-state
      (drive-stretch-shortening-ballistic-impulse
        :launch-topology :bilateral-in-phase
        :target-apex-height-dz-mm (+ platform-elevation-z-mm 120.0)
        :target-horizontal-span-dx-mm platform-forward-x-mm)
      (modulate-touchdown-elevation-datum :expected-delta-z-mm platform-elevation-z-mm)))

;; 2. Autonomous SDF Puddle / Ditch / Hurdle Leap Trigger
;; Monitors `sub-tarsal-slab` along the geodesic streamline: if an untraversable
;; gap, puddle, or low obstacle of length [350, 1450] mm is detected ahead during
;; walking or running, automatically triggers `xform-in-stride-unilateral-leap`
;; scaled to clear the obstacle with 120 mm margin! Hers [350, 398]: her leap's 638
;; less the margins, its apex 75 a flat gap's.
(defmodifier specialize-leap-over-sdf-gap-or-hurdle
  [locomotion-state &key (safety-margin-x-mm 120.0) (safety-margin-z-mm 85.0)]
  (-> locomotion-state
      (when-modifier (:sub-tarsal-gap-or-hurdle-ahead-p)
        (let [gap-dx (query-upcoming-obstacle-span-x :sub-tarsal-slab)
              obs-dz (query-upcoming-obstacle-height-z :sub-tarsal-slab)]
          (-> locomotion-state
              (xform-in-stride-unilateral-leap
                :leap-span-x-mm (+ gap-dx (* 2.0 safety-margin-x-mm))
                :leap-apex-z-mm (+ (max -10.0 obs-dz) safety-margin-z-mm))))))) ; her apex 75 less 85 (180.0)

;; 3. High-Speed Braking Arm-Counterweight & Pitch Arrestor Specializer
;; During an aggressive 2-step stop from running, thrusts both arms forward/downward
;; and pitches the lumbar spine back to counter the forward pitching moment
;; tau_pitch = -F_x_brake * Z_com around the braking heel!
(defmodifier specialize-brake-with-aerodynamic-and-arm-counter
  [brake-state &key (pitch-arrest-gain 1.15)]
  (-> brake-state
      (bind-braking-pitch-counterweight
        :compensate-moment-arm-z-com true
        :thoracic-extension-gain pitch-arrest-gain
        :bilateral-glenohumeral-counter-swing-rad +0.85)))

;; 4. Elastic Achilles Series-Compliance Booster (For High-Efficiency Double-Time & Running)
;; Tunes virtual ankle series-elasticity Q-factor to resonate at the active stride
;; frequency, reducing knee stator copper heating by 28% during sustained running.
(defmodifier specialize-run-with-elastic-achilles-q-boost
  [run-state &key (achilles-resonance-q 2.15)]
  (-> run-state
      (set-plantar-boundary :windlass-arch-q achilles-resonance-q)
      (modulate-node :tibiofemoral :rebound-q (:set achilles-resonance-q))))
