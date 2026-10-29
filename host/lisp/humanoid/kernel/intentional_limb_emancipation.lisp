;; =============================================================================
;; FILE: intentional_limb_emancipation.lisp
;; DOMAIN: Context-Gated Contact Unloading, Pre-Emptive Observer Unanchoring,
;;         Open-Chain Gravity Admittance & Cantilever Reaction Feedforward
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp,
;;           prior_regularized_observer.lisp
;; =============================================================================
(defpackage :ortho-dsp.emancipation
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.prior-observer)
  (:export #:limb-emancipation-soa-state
           #:evaluate-residual-cwc-sufficiency
           #:evacuate-manifold-simplex-preemptively
           #:morph-stance-to-open-chain-admittance
           #:feedforward-cantilever-reaction-wrench
           #:xform-intentional-limb-emancipation
           ;; Composable Scenario Specializers
           #:specialize-emancipation-locomotion-lockout
           #:specialize-emancipation-seated-leg-stretch
           #:specialize-emancipation-high-stool-dangling
           #:specialize-emancipation-bed-leg-repositioning
           #:specialize-emancipation-compliant-human-guidance))
(in-package :ortho-dsp.emancipation)

;; -----------------------------------------------------------------------------
;; 1. LIMB EMANCIPATION SOA STATE (64B-ALIGNED PER-APPENDAGE TRACKER)
;; Tracks intentional release request iota_k, safety gate eta_release_k,
;; effective emancipation gamma_k = iota_k * eta_release_k, and open-chain state.
;; -----------------------------------------------------------------------------
(defstate limb-emancipation-soa-state
  (:memory-layout
    :num-appendages 4                        ; [0: Left-Leg, 1: Right-Leg, 2: Left-Arm, 3: Right-Arm]
    :align-bytes 64)
  (:slot-schema
    :intentional-unload-iota   :float32      ; Requested unload [0.0 = stance, 1.0 = free open-chain]
    :residual-cwc-gate-eta     :float32      ; Physics safety gate [0.0 = unsafe/walking, 1.0 = safe to release]
    :effective-gamma           :float32      ; gamma = iota * eta_release (actual emancipation factor)
    :open-chain-stiffness-kp   :float32      ; Reduced joint Kp when emancipated (Nm/rad)
    :open-chain-damping-kd     :float32      ; Joint Kd when emancipated (Nms/rad)
    :external-admittance-gain  :float32      ; rad/(Nm*s) compliance to external human/object manipulation
    :cantilever-root-wrench-6d :vec6-plucker ; Reaction wrench [tau; f] exerted by free limb on hip/shoulder
    :emancipation-stage        :keyword))    ; :locked-stance | :unanchoring-observer | :shedding-qp | :free-open-chain

;; -----------------------------------------------------------------------------
;; 2. RESIDUAL CWC SUFFICIENCY & FROUDE VELOCITY GATE (SAFETY INTERLOCK)
;; Determines whether appendage `k` CAN be safely unloaded by checking:
;;   1) Froude number Fr = v_com^2 / (g * L_0) < Fr_lockout (blocks during walk/run!)
;;   2) The complement support manifolds S \ {k} strictly enclose the centroidal
;;      gravito-inertial wrench W_GI with >= `min-cwc-margin-nm` stability margin.
;; -----------------------------------------------------------------------------
(defprimitive evaluate-residual-cwc-sufficiency
  [state &key (froude-lockout-ceiling 0.045) ; Hard lockout if walking/running (v > ~0.6 m/s)
              (min-complement-cwc-margin-nm 18.0)
              (require-minimum-anchor-sum-w 0.65)]
  (-> state
      (compute-complement-cone-feasibility
        :emancipation-bank 'limb-emancipation-soa-state
        :manifold-weights-source :active-manifold-weights ; `out[MANIFOLDED]`
        :max-allowable-froude froude-lockout-ceiling
        :min-residual-cone-margin min-complement-cwc-margin-nm
        :min-complement-weight-sum require-minimum-anchor-sum-w
        ;; Smooth barrier: eta_release_k -> 0.0 during bipedal gait/flight,
        ;;                 eta_release_k -> 1.0 when braced on buttocks/hands/back!
        :gate-output-slot :residual-cwc-gate-eta)))

;; -----------------------------------------------------------------------------
;; 3. CAUSAL PRE-EMPTIVE SIMPLEX & OBSERVER EVACUATION (ZERO-SPASM HANDOVER)
;; Executes BEFORE the limb moves in joint space:
;;   Stage 1 (gamma in [0.00, 0.25]): Detaches limb `k` from InEKF zero-velocity
;;           odometry and suppresses its prior weight pi_k in `observer.py`.
;;   Stage 2 (gamma in [0.15, 0.75]): Ramps down upper normal force bound Fz_max_k
;;           in `wrench.step`, smoothly shifting 100% of weight onto S \ {k}.
;;   Stage 3 (gamma in [0.75, 1.00]): Unlocks open-chain joint trajectory/admittance
;;           once measured Fz_k confirms < 3% m*g.
;; -----------------------------------------------------------------------------
(defprimitive evacuate-manifold-simplex-preemptively
  [state &key (evacuation-slew-ms 160.0)
              (confirm-unloaded-fz-ratio 0.03)]
  (-> state
      (drive-causal-unloading-cascade
        :emancipation-bank 'limb-emancipation-soa-state
        :slew-duration-ms evacuation-slew-ms
        ;; 1. Pre-emptive Observer Decoupling (Prevents "phantom falling" EKF spasm!):
        :observer-unanchor-window [0.00 -> 0.25]
        :inflate-inekf-contact-covariance-scale 1000.0
        :mask-simplex-prior-weight true
        ;; 2. Pre-emptive QP Normal-Force Ceiling Ramp-Down (Prevents "floor-chasing" spasm!):
        :qp-load-shed-window [0.15 -> 0.75]
        :transfer-load-to-complement-manifolds true
        ;; 3. Physical Unload Confirmation Gate:
        :kinematic-release-fz-floor confirm-unloaded-fz-ratio)))

;; -----------------------------------------------------------------------------
;; 4. STANCE-TO-OPEN-CHAIN ADMITTANCE HOMOTOPY (PASSIVE OR ACTIVE MANIPULATION)
;; As gamma_k -> 1.0, decouples limb `k` from the standing-squat modal columns,
;; switches from whole-body mass compensation (62 kg) to local limb-only gravity
;; compensation (~8 kg), projects into complement Null-Space P_{perp,-k}, and
;; routes external GMO torques into smooth compliant backdrivability!
;; -----------------------------------------------------------------------------
(defprimitive morph-stance-to-open-chain-admittance
  [state &key (open-chain-kp-nm-rad 35.0)     ; Soft open-chain tracking (vs 450 Nm/rad in stance!)
              (open-chain-kd-nms-rad 4.2)
              (external-admittance-rad-nms 0.18) ; Yield smoothly if pushed/pulled by hand or bed!
              (external-torque-deadzone-nm 1.8)]
  (-> state
      (blend-closed-to-open-chain-impedance
        :emancipation-bank 'limb-emancipation-soa-state
        ;; Zero out the emancipated limb's rows in `B_modal` so torso axial/sagittal
        ;; breathing never forces the free legs/arms to squat!
        :decouple-from-modal-basis true
        :gravity-compensation-law :isolated-limb-subtree-rnea
        :nullspace-projector :complement-support-khatib-nullspace
        :open-chain-impedance
          (:kp open-chain-kp-nm-rad
           :kd open-chain-kd-nms-rad
           :admittance-gain external-admittance-rad-nms
           :gmo-deadzone-nm external-torque-deadzone-nm))))

;; -----------------------------------------------------------------------------
;; 5. OPEN-CHAIN CANTILEVER REACTION FEEDFORWARD (ANTI-TIPPING COUNTERBALANCE)
;; Computes the 6D reaction wrench at the hip/shoulder root of every emancipated
;; limb (e.g. two legs stretched straight out in front of the buttocks) and feeds
;; it forward into the complement CWC (`:ischial` + `:ulnar-carpal`) so the hands
;; and torso automatically counter-balance the shifting leg Center of Mass!
;; -----------------------------------------------------------------------------
(defprimitive feedforward-cantilever-reaction-wrench
  [state &key (feedforward-gain 1.00)
              (auto-pitch-torso-counterweight true)]
  (-> state
      (inject-open-chain-root-wrench-into-cwc
        :emancipation-bank 'limb-emancipation-soa-state
        :rnea-subtree-source :emancipated-limb-chains
        :wrench-bus-gain feedforward-gain
        :compensate-with-complement-anchors true
        :enable-thoracic-counter-cantilever auto-pitch-torso-counterweight)))

;; -----------------------------------------------------------------------------
;; 6. MASTER INTENTIONAL LIMB EMANCIPATION TRANSFORMER
;; -----------------------------------------------------------------------------
(defmodifier xform-intentional-limb-emancipation [state]
  (-> state
      (evaluate-residual-cwc-sufficiency
        :froude-lockout-ceiling 0.045
        :min-complement-cwc-margin-nm 18.0)
      (evacuate-manifold-simplex-preemptively :evacuation-slew-ms 160.0)
      (morph-stance-to-open-chain-admittance
        :open-chain-kp-nm-rad 35.0
        :external-admittance-rad-nms 0.18)
      (feedforward-cantilever-reaction-wrench :feedforward-gain 1.00)))


;; =============================================================================
;; COMPOSABLE SCENARIO SPECIALIZERS (PLUG-IN EXTENSIONS)
;; =============================================================================

;; 1. Hard Locomotion Lockout Specializer (Walk, Brisk Walk, Double-Time, Run)
;; Explicitly clamps iota_k = 0.0 and eta_release_k = 0.0 on stance limbs during
;; dynamic bipedal locomotion so stance legs NEVER yield or drop out of the CWC.
(defmodifier specialize-emancipation-locomotion-lockout [gait-state]
  (-> gait-state
      (evaluate-residual-cwc-sufficiency
        :froude-lockout-ceiling 0.015
        :min-complement-cwc-margin-nm 45.0)
      (set-appendage-unload-intent
        :appendages [:left-leg :right-leg]
        :intent-iota 0.00)))

;; 2. Seated on Buttocks + Hands Tripod -> Stretch Out Both Legs in Front
;; ("Sit down on the buttocks and stretch the legs out, supported on the arms, hands and buttocks")
;; - Verifies that `:ischial-bilateral` + `:ulnar-carpal-shove-bilateral` hold the CoM.
;; - Pre-emptively evacuates `:plantar-bilateral` from the observer and QP.
;; - Stretches both knees (`:tibiofemoral` -> 0.05 rad) in open-chain admittance while
;;   feeding the +56 Nm forward leg-cantilever torque into the rear hands and spine!
(defmodifier specialize-emancipation-seated-leg-stretch
  [seated-tripod-state &key (target-knee-extension-rad 0.05)
                            (target-hip-flexion-rad 1.42)
                            (target-ankle-dorsiflex-rad -0.12)]
  (-> seated-tripod-state
      (xform-intentional-limb-emancipation)
      ;; Request full intentional emancipation of both legs while hands + buttocks anchor:
      (set-appendage-unload-intent
        :appendages [:left-leg :right-leg]
        :intent-iota 1.00
        :require-active-anchors [:ischial-bilateral :ulnar-carpal-shove-bilateral])
      ;; Command smooth open-chain leg extension in front of the torso:
      (drive-emancipated-limb-posture
        :appendages [:left-leg :right-leg]
        :target-angles `((:coxofemoral ,target-hip-flexion-rad)
                         (:tibiofemoral ,target-knee-extension-rad)
                         (:talocrural ,target-ankle-dorsiflex-rad))
        :open-chain-kp-nm-rad 28.0
        :external-admittance-rad-nms 0.25))) ; Compliant if feet touch floor or an obstacle!

;; 3. Overheight Chair / Stool Dangling & Free Leg Manipulation
;; When seated on a high stool where feet don't reach the floor (or barely graze it),
;; emancipates both legs into gravity-compensated pendulum admittance so they hang
;; naturally and can be swung or pushed by hand without disturbing core balance on the stool!
(defmodifier specialize-emancipation-high-stool-dangling
  [high-seated-state &key (pendulum-stiffness-kp 12.0) (admittance-compliance 0.35)]
  (-> high-seated-state
      (xform-intentional-limb-emancipation)
      (set-appendage-unload-intent
        :appendages [:left-leg :right-leg]
        :intent-iota 1.00
        :require-active-anchors [:ischial-bilateral :posterior-femoral-bilateral])
      (morph-stance-to-open-chain-admittance
        :open-chain-kp-nm-rad pendulum-stiffness-kp
        :open-chain-kd-nms-rad 2.5
        :external-admittance-rad-nms admittance-compliance)))

;; 4. Bedside Sequential Leg Lift & Repositioning (One Leg or Both Legs in Bed)
;; Used when sitting on the bed edge leaning on forearms (`:olecranon-ulnar-sphinx`)
;; to unload and lift one leg at a time onto the mattress without observer spikes.
(defmodifier specialize-emancipation-bed-leg-repositioning
  [bedside-state &key (active-lifting-leg :left-leg)]
  (-> bedside-state
      (xform-intentional-limb-emancipation)
      (set-appendage-unload-intent
        :appendages [active-lifting-leg]
        :intent-iota 1.00
        :require-active-anchors [:ischial-bilateral :olecranon-ulnar-sphinx])
      (morph-stance-to-open-chain-admittance
        :open-chain-kp-nm-rad 42.0
        :external-admittance-rad-nms 0.22)))

;; 5. Zero-Gravity "Lead-by-Hand" Compliant Limb Guidance (Physical Human Manipulation)
;; Sets open-chain position stiffness Kp -> 0.0 on the emancipated limb(s) while keeping
;; exact subtree gravity + Coriolis compensation active. A human operator can grab the
;; robot's foot, shin, or hand and place it anywhere in its workspace with zero resistance!
(defmodifier specialize-emancipation-compliant-human-guidance
  [state &key (guided-appendages [:left-leg :right-leg]) (viscous-damping-nms 1.8)]
  (-> state
      (xform-intentional-limb-emancipation)
      (set-appendage-unload-intent :appendages guided-appendages :intent-iota 1.00)
      (morph-stance-to-open-chain-admittance
        :open-chain-kp-nm-rad 0.0               ; Pure zero-G gravity compensation!
        :open-chain-kd-nms-rad viscous-damping-nms
        :external-admittance-rad-nms 0.65
        :external-torque-deadzone-nm 0.8)))
