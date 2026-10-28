;; =============================================================================
;; FILE: ballistic_and_velocity_core.lisp
;; DOMAIN: Froude Pendulum<->SLIP Homotopy, Aerial Momentum Conservation,
;;         Mu/Regen-Gated Braking Cascade & Stretch-Shortening Jump/Leap Engine
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp, universal_substrate_overlay.lisp
;; =============================================================================
(defpackage :ortho-dsp.ballistic-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.power-thermal)
  (:export #:ballistic-1jet-posture-atlas
           #:modulate-froude-pendulum-slip-homotopy
           #:conserve-aerial-centroidal-momentum
           #:govern-mu-and-regen-braking-horizon
           #:drive-stretch-shortening-ballistic-impulse))
(in-package :ortho-dsp.ballistic-core)

;; -----------------------------------------------------------------------------
;; 1. BALLISTIC & HIGH-VELOCITY 1-JET POSTURE ATLAS
;; Adds canonical 0-jet origins and 3-column modal synergies for:
;;   - Brisk Walk / Power-Walk (Pelvic yaw extension, stiff knee roll-over)
;;   - Double-Time March / Heavy Elastic Jog (Low-apex compliant crouch-run)
;;   - Full SLIP Running (Forefoot strike, high elbow carriage, forward lean)
;;   - Bilateral Countermovement Jump (Symmetric triple-extension coil)
;;   - Unilateral Split-Leap (Mid-air hurdle/gap split posture)
;; -----------------------------------------------------------------------------
(defstate ballistic-1jet-posture-atlas
  (:postures
    ;; A. BRISK WALK / POWER-WALK (Fr in [0.25, 0.45], Duty ~ 0.53)
    (:posture :locomotion-brisk-walk
      :coupled-manifold [:plantar-bilateral :metatarsal-equinus-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural 0.05 :tibiofemoral 0.06 :coxofemoral -0.08
         :iliac-coronal 0.00 :l4-l5-lumbar -0.10 :c7-t12-thoracic +0.10
         :glenohumeral 0.12 :cubital 0.85 :c1-c2-atlas -0.06) ; Bent elbows for fast arm pump!
      :modal-basis-columns
        (:mode-axial-compression (:talocrural +0.48 :tibiofemoral +0.95 :coxofemoral +0.52)
         :mode-sagittal-shift    (:talocrural +0.42 :coxofemoral +0.48 :c7-t12-thoracic +0.18)
         :mode-pitch-attitude    (:coxofemoral +0.60 :l4-l5-lumbar +0.35 :c1-c2-atlas -0.85)))

    ;; B. DOUBLE-TIME MARCH / DOUBLE-TIME HEAVY JOG (Fr in [0.45, 0.75], Duty ~ 0.44)
    (:posture :locomotion-sprangmarsch
      :coupled-manifold [:plantar-bilateral :metatarsal-equinus-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.14 :tibiofemoral 0.38 :coxofemoral 0.24
         :iliac-coronal 0.00 :l4-l5-lumbar -0.08 :c7-t12-thoracic +0.16
         :glenohumeral 0.18 :cubital 1.25 :c1-c2-atlas -0.12)
      :modal-basis-columns
        ;; In-phase SLIP spring compression (absorbs vertical shock without high aerial bounce):
        (:mode-axial-compression (:talocrural +0.65 :tibiofemoral +1.25 :coxofemoral +0.75
                                  :l4-l5-lumbar +0.15)
         :mode-sagittal-shift    (:talocrural +0.45 :coxofemoral +0.55 :c7-t12-thoracic +0.22)
         :mode-pitch-attitude    (:coxofemoral +0.65 :l4-l5-lumbar +0.35 :c1-c2-atlas -0.85)))

    ;; C. FULL ELASTIC SLIP RUN / RUNNING (Fr > 0.75, Duty ~ 0.32)
    (:posture :locomotion-slip-run
      :coupled-manifold :metatarsal-equinus-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.18 :tibiofemoral 0.42 :coxofemoral 0.28
         :iliac-coronal 0.00 :l4-l5-lumbar -0.06 :c7-t12-thoracic +0.22
         :glenohumeral 0.24 :cubital 1.55 :c1-c2-atlas -0.18)
      :modal-basis-columns
        (:mode-axial-compression (:talocrural +0.85 :tibiofemoral +1.35 :coxofemoral +0.80)
         :mode-sagittal-shift    (:talocrural +0.52 :coxofemoral +0.65 :c7-t12-thoracic +0.25)
         :mode-pitch-attitude    (:coxofemoral +0.70 :l4-l5-lumbar +0.38 :c1-c2-atlas -0.88)))

    ;; D. BILATERAL COUNTERMOVEMENT JUMP COIL & TUCK (Standing Jump, Feet Together)
    (:posture :ballistic-bilateral-jump
      :coupled-manifold [:plantar-bilateral :metatarsal-equinus-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural 0.00 :tibiofemoral 0.12 :coxofemoral 0.08
         :iliac-coronal 0.00 :l4-l5-lumbar -0.10 :c7-t12-thoracic +0.08
         :glenohumeral 0.00 :cubital 0.35 :c1-c2-atlas 0.00)
      :modal-basis-columns
        ;; Axial +1.0 = Deep countermovement squat with arms cocked back (-0.85 rad);
        ;; Axial -1.0 = Full triple-extension takeoff with arms thrown overhead (+2.1 rad)!
        (:mode-axial-compression (:talocrural +0.72 :tibiofemoral +1.48 :coxofemoral +1.15
                                  :l4-l5-lumbar +0.28 :glenohumeral -1.35 :cubital +0.45)
         ;; Sagittal shift controls forward jump vector vs. pure vertical jump:
         :mode-sagittal-shift    (:talocrural +0.48 :coxofemoral +0.55 :glenohumeral +0.65)
         ;; Pitch attitude controls mid-air knee-tuck clearance without torso rotation:
         :mode-pitch-attitude    (:coxofemoral +1.25 :tibiofemoral +1.40 :glenohumeral -0.55)))))

;; -----------------------------------------------------------------------------
;; 2. FROUDE-NUMBER PENDULUM <-> SLIP CONTINUOUS HOMOTOPY ENGINE
;; Smoothly morphs phase relationship, duty factor, pelvic yaw stride-extension,
;; and Achilles tendon stiffness as speed transitions across:
;; Walk (Fr < 0.25) <-> Brisk Walk (0.25..0.45) <-> Double-Time (0.45..0.75) <-> Run (> 0.75)
;; -----------------------------------------------------------------------------
(defprimitive modulate-froude-pendulum-slip-homotopy
  [state &key (target-velocity-mps 1.218)        ; her walk at the text's Froude 0.179 (1.25)
              (leg-reference-length-m 0.845)]    ; her hip over her soles (0.89)
  (let [froude-num (/ (* target-velocity-mps target-velocity-mps)
                      (* 9.80665 leg-reference-length-m))]
    (-> state
        (drive-froude-continuum-morph
          :froude-index froude-num
          :target-speed-mps target-velocity-mps
          ;; 1. Continuous Stance Duty Slew: 0.62 (Walk) -> 0.53 (Brisk) -> 0.44 (Double-Time) -> 0.32 (Run)
          :duty-factor-law (:hermite-froude-map :froude-knots [0.15 0.38 0.60 1.20]
                                                :duty-values  [0.62 0.53 0.44 0.32])
          ;; 2. Vertical CoM Phase Inversion: pi (Vaulting Walk) -> 0.0 (Compliant SLIP Spring)
          :com-z-phase-inversion-law
            (:froude-crossover-window [0.35 0.52]
             :walk-vault-phase-rad 3.14159
             :slip-bounce-phase-rad 0.00000
             :sprangmarsch-apex-clamp-mm 26.6)   ; Keep vertical bounce low during the double-time march! Hers (28.0)
          ;; 3. Brisk-Walk Transverse Pelvic Yaw Stride Extension (Replaces premature flight):
          :pelvic-yaw-stride-booster
            (:active-froude-window [0.22 0.50]
             :max-axial-pelvic-yaw-rad 0.22      ; +140mm effective stride length in Brisk Walk! Hers +83, hips 190 apart
             :counter-rotate-thoracic-arm-pump true)))))

;; -----------------------------------------------------------------------------
;; 3. AERIAL CENTROIDAL ANGULAR MOMENTUM CONSERVATION (FLIGHT-PHASE CONTROLLER)
;; Active whenever sum(w_m) < 0.05 (running flight phase, two-footed jump, or leap).
;; Replaces ground Wrench-QP with A_{G,omega}(q)*dq = L_launch conservation and
;; Raibert + Capture-Point foot placement pre-positioning.
;; -----------------------------------------------------------------------------
(defprimitive conserve-aerial-centroidal-momentum
  [state &key (raibert-velocity-gain 0.14)
              (pre-touchdown-leg-retraction-rate-mps -0.438)]  ; at her size (-0.45)
  (-> state
      (bind-zero-wrench-flight-connection
        :activation-condition (:sum-of-manifold-weights-below 0.05)
        ;; Enforce A_{G,omega}(q) * dq == L_takeoff so tucking/extending legs in mid-air
        ;; automatically counter-rotates arms/thorax to keep the head and torso level!
        :momentum-projector :centroidal-angular-momentum-nullspace
        :raibert-touchdown-targeting
          (:neutral-point-law :half-stance-time-prediction
           :velocity-feedback-gain raibert-velocity-gain
           ;; Late-swing ground-speed matching ("pawing" retraction just before impact)
           ;; reduces horizontal impact shock and prevents heel-strike slip!
           :swing-retraction-matching-mps pre-touchdown-leg-retraction-rate-mps)
        :touchdown-stiffness-precharge
          (:pre-tension-ankle-knee-before-impact-ms 25.0
           :target-manifold-on-land :metatarsal-equinus-bilateral))))

;; -----------------------------------------------------------------------------
;; 4. FRICTION- & REGEN-GOVERNED RAPID BRAKING HORIZON (RUN -> WALK ABS CASCADE)
;; Evaluates live surface friction mu_eff and DC-bus voltage headroom V_clamp - V_bus
;; to decelerate from running/double-time to walking as fast as physically possible
;; WITHOUT triggering tangential foot slip or inverter overvoltage High-Z trips!
;; -----------------------------------------------------------------------------
(defprimitive govern-mu-and-regen-braking-horizon
  [state &key (target-terminal-velocity-mps 1.169) ; at her size (1.20)
              (max-regen-bus-voltage-v 57.5)
              (min-cwc-shear-margin-ratio 0.82)]
  (-> state
      (solve-adaptive-deceleration-cascade
        :target-speed-mps target-terminal-velocity-mps
        :friction-source :live-gmo-and-registry-mu
        :rheology-source 'ortho-dsp.deformable-core:rheological-substrate-soa-state
        :bus-voltage-ceiling-v max-regen-bus-voltage-v
        :cone-utilization-cap min-cwc-shear-margin-ratio
        ;; Dynamic Step-Count & Posture Modulation Law:
        ;; N_steps = ceil( m * (v_current - v_target) / Impulse_max(mu_eff, V_bus) )
        :high-mu-aggressive-stop                 ; Activated when mu_eff >= 0.65 (Dry floor / asphalt)
          (:max-braking-steps 2
           :drop-com-axial-mode +0.68            ; Deep eccentric squat lowers CoM & lengthens brake stroke
           :anterior-capture-step-lead-gain 1.32 ; Plant braking foot well ahead of CoM
           :strike-topology :calcaneal-heel-dig-to-flat
           :shunt-excess-regen-to-nullspace-co-contraction true)
        :low-mu-stutter-abs-stop                 ; Activated when mu_eff < 0.35 (Ice, wet tile, loose gravel/snow)
          (:min-braking-steps 6
           :boost-cadence-bpm-scale 1.35         ; Fast, short, flat-footed stutter steps!
           :zero-strike-pitch-angle true
           :limit-per-step-dv-mps 0.341))))      ; at her size (0.35)

;; -----------------------------------------------------------------------------
;; 5. STRETCH-SHORTENING CYCLE (SSC) BALLISTIC IMPULSE ENGINE (JUMPS & LEAPS)
;; Drives both Symmetric Standing Two-Footed Jumps (`:bilateral-in-phase`) and
;; Asymmetric In-Stride Single-Leg Leaps (`:unilateral-split-leap`) with live
;; Back-EMF torque-envelope shifting from knee -> ankle during late push-off!
;; -----------------------------------------------------------------------------
(defprimitive drive-stretch-shortening-ballistic-impulse
  [state &key (launch-topology :bilateral-in-phase) ; :bilateral-in-phase (two-footed jump) | :unilateral-split-leap (leap)
              (target-apex-height-dz-mm 210.0)   ; her knees' launch from her crouch (320.0)
              (target-horizontal-span-dx-mm 0.0)
              (countermovement-depth-mm 175.6)   ; at her size (185.0)
              (arm-flywheel-assist true)]
  (-> state
      (execute-ssc-ballistic-launch
        :topology launch-topology
        :desired-apex-z-m (* 0.001 target-apex-height-dz-mm)
        :desired-range-x-m (* 0.001 target-horizontal-span-dx-mm)
        ;; Phase 1: Eccentric Unweighting & Countermovement Coil (Stores virtual tendon energy)
        :phase-1-countermovement
          (:unweight-fz-dip-ratio 0.35
           :crouch-depth-m (* 0.001 countermovement-depth-mm)
           :cock-glenohumeral-back-rad -0.82)
        ;; Phase 2: Concentric Triple-Extension with Back-EMF Proximal-to-Distal Handoff
        ;; As knee velocity dq_knee approaches Back-EMF saturation, shifts peak power
        ;; demand down to the ankle plantarflexors (`:talocrural` + `:metatarsal-equinus`)!
        ;; Hers: her knees leave at 15.8 of their 16.8 rad/s, 2.03 m/s; her ankles reach 2.15 m/s at most.
        :phase-2-concentric-launch
          (:takeoff-angle-law :ballistic-parabola-solver
           :bemf-aware-proximal-to-distal-sequencing true
           :arm-upward-momentum-injection arm-flywheel-assist
           :unlatch-ground-at-fz-ratio 0.02)
        ;; Phase 3: Mid-Air Obstacle Clearance Tuck & Split-Leg Hurdle Geometry
        :phase-3-aerial-shaping
          (:conserve-angular-momentum true
           :midair-knee-tuck-clearance-m (* 0.001 (* 0.65 target-apex-height-dz-mm))
           :split-leap-lead-hip-flex-rad (if (eq launch-topology :unilateral-split-leap) +1.15 0.0))
        ;; Phase 4: Conjugate Viscoelastic Impact Absorption & Re-Sync to Gait Clock
        :phase-4-touchdown-absorption
          (:pre-land-equinus-plantarflex-rad -0.38
           :eccentric-knee-cushion-rad 0.68
           :shunt-landing-regen-spike true
           :resume-clock-on-dcm-capture true))))
