;; =============================================================================
;; FILE: prior_regularized_observer.lisp
;; DOMAIN: Laplace Innovation Collapse, Tri-IMU (Head/Torso/Pelvis) Gravity
;;         Simplex Priors & Active Weight-Shift Compliance Probing
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp
;; Hers (`machine.senses.observer`, wbc_gmo.c): her whole body's balance - her pelvis's IMU and her momentum - bears the
;; sum, so no 0/0: what her joints cannot tell, gravity, height and stillness do, her soles their last pass's.
;; =============================================================================
(defpackage :ortho-dsp.prior-observer
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.postures-core)
  (:export #:tri-imu-observer-soa-state
           #:collapse-static-equilibrium-innovation
           #:fuse-tri-imu-axial-kinematics
           #:regularize-manifold-simplex-with-priors
           #:gate-supports-by-stillness
           #:probe-unobserved-substrate-via-weight-shift
           #:xform-prior-regularized-observer
           #:specialize-observer-for-sensorless-soft-bed
           #:specialize-observer-with-active-edge-probing))
(in-package :ortho-dsp.prior-observer)

;; -----------------------------------------------------------------------------
;; 1. TRI-IMU & PRIOR OBSERVER SOA STATE (64B-ALIGNED REAL-TIME TELEMETRY)
;; Integrates the 3 IMUs (:pelvis-imu, :thorax-imu, :head-imu) with prior-backed
;; manifold weights `out[MANIFOLDED]` and directional compliance estimates.
;; Hers: her observer reads her pelvis's IMU alone; each of her boards carries a BNO085.
;; -----------------------------------------------------------------------------
(defstate tri-imu-observer-soa-state
  (:memory-layout
    :num-imus 3                              ; [0: Pelvis/Hip, 1: Thorax/Torso, 2: Head/Atlas]
    :num-manifolds 8
    :align-bytes 64)
  (:imu-nodes
    :pelvis-hip-imu   (:frame :sacroiliac-z    :gyro-sigma 0.0012 :accel-sigma 0.010)
    :thorax-torso-imu (:frame :c7-t12-thoracic :gyro-sigma 0.0015 :accel-sigma 0.012)
    :head-atlas-imu   (:frame :c1-c2-atlas     :gyro-sigma 0.0010 :accel-sigma 0.008))
  (:observer-fields
    :static-energy-mahalanobis :float32      ; E_motion -> 0.0 when upright & still
    :innovation-shrinkage-gain :float32      ; 0.0 = total collapse (locked), 1.0 = full open loop
    :prior-trust-ratio-kappa   :float32      ; 0.02 in sensorized stance -> 1.0 in High-Z soft bed
    :probed-directional-stiff  :vec4-npmrad  ; [Anterior, Posterior, Left, Right] rotational stiffness
    :active-probe-com-shift-mm :vec2-mm))    ; Live sub-threshold CoM test perturbation

;; -----------------------------------------------------------------------------
;; 2. LAPLACE / STEIN INNOVATION COLLAPSE AT STATIC EQUILIBRIUM
;; Evaluates Mahalanobis motion energy across all 3 IMUs and joint encoders.
;; When standing upright and still (E_motion < 1.0), applies soft-thresholding
;; shrinkage to collapse filter innovation to EXACT 0.0, locking gyro biases!
;; Hers: a joint's row within its friction's band - its loss, 1.23-2.24 N m, less what its speed spends
;; (`slip` 0.03 rad/s) - is read as friction and tells the split nothing; past it, the band's edge.
;; -----------------------------------------------------------------------------
(defprimitive collapse-static-equilibrium-innovation
  [state &key (static-energy-threshold 1.0)
              (deadzone-lambda-max 0.045)
              (collapse-slew-ms 12.0)]
  (-> state
      (apply-laplace-innovation-shrinkage
        :observer-bank 'tri-imu-observer-soa-state
        :imu-sources [:pelvis-hip-imu :thorax-torso-imu :head-atlas-imu]
        :encoder-velocity-source :all-actuated-dq
        :energy-gate-threshold static-energy-threshold
        ;; Shrinkage Law: nu_eff = sign(nu) * max(0, |nu| - lambda_max * exp(-E_motion))
        :soft-threshold-lambda deadzone-lambda-max
        :collapse-attack-ms collapse-slew-ms
        :on-full-static-collapse
          (:freeze-manifold-simplex-weights true
           :lock-gyro-bias-integrator true
           :zero-spurious-shear-wrenches true))))

;; -----------------------------------------------------------------------------
;; 3. TRI-IMU AXIAL SKELETON TRIANGULATION (HIP <-> TORSO <-> HEAD)
;; Compares relative gravity vectors [g_pelvis, g_thorax, g_head] against spinal
;; encoder forward kinematics to observe unmodeled mattress cradling, pillow
;; support, and trunk pitch/roll even when all leg/arm torques are zero!
;; Hers: none of it on her loop.
;; -----------------------------------------------------------------------------
(defprimitive fuse-tri-imu-axial-kinematics
  [state &key (complementary-cutoff-hz 18.0)
              (cross-imu-consistency-weight 2.5)]
  (-> state
      (solve-tri-imu-kinematic-chain
        :pelvis-imu :pelvis-hip-imu
        :thorax-imu :thorax-torso-imu
        :head-imu   :head-atlas-imu
        :inter-imu-encoders [:l4-l5-lumbar :c7-t12-thoracic :c1-c2-atlas]
        :fusion-bandwidth-hz complementary-cutoff-hz
        :coupling-gain cross-imu-consistency-weight
        :outputs [:segment-gravity-vectors-s2
                  :spinal-deflection-residual-rad
                  :centroidal-tilt-rate-radps])))

;; -----------------------------------------------------------------------------
;; 4. TIKHONOV-DIRICHLET SIMPLEX REGULARIZER (PREVENTS 0/0 COLLAPSE ON SOFT BED)
;; Computes `out[MANIFOLDED]` by blending measured GMO joint-torque forces with
;; a morphological 3-IMU gravity-alignment prior pi_m(q, g_P, g_T, g_H).
;; Guarantees sum(w_m) == 1.0 and smooth CWC matrices even with zero force sensors!
;; Hers: the prior her split's tie-break, over her points, not her manifolds' weights: each pressing up as
;; far as its pushes' cosine with her pelvis IMU's vertical, lying low as far as its height over her
;; lowest point in her ischia's radius; trusted as a floor, sat on the chair her seat read 94 N off.
;; -----------------------------------------------------------------------------
(defprimitive regularize-manifold-simplex-with-priors
  [state &key (min-prior-floor-mg-ratio 1.0e-6)  ; a tie-break, of her split's largest curvature (0.02)
              (uninstrumented-prior-boost 0.85)
              (gravity-alignment-beta 7.02)      ; her law's pyramid's edge at e^-1: 1 / (1 - cos atan 0.6) (6.5)
              (height-reach-m 0.07)]             ; her ischia's radius (the text's 12 per m)
  (-> state
      (compute-map-simplex-manifold-weights
        :manifolds [:plantar-bilateral :metatarsal-equinus-bilateral
                    :posterior-femoral-bilateral :ischial-bilateral
                    :trochanter-gluteal-lateral :dorsal-supine-continuum
                    :ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :measured-wrench-source :generalized-momentum-observer
        ;; Morphological Prior pi_m(q, g_IMU):
        ;; Evaluates dot(-normal_m(q), g_IMU) across Pelvis & Thorax IMUs + kinematic Z-height
        :prior-distribution-law
          (:gravity-cosine-sharpness gravity-alignment-beta
           :height-reach height-reach-m
           :pelvic-patches  [:ischial-bilateral :posterior-femoral-bilateral :trochanter-gluteal-lateral]
           :thoracic-patches [:dorsal-supine-continuum :ventral-prone-continuum]
           :appendage-patches [:plantar-bilateral :metatarsal-equinus-bilateral :ulnar-carpal-shove-bilateral])
        ;; Tikhonov Regularization Denominator Guard:
        ;; w_m = (Fz_meas_m + kappa * m*g * pi_m) / sum_k(Fz_meas_k + kappa * m*g * pi_k)
        :adaptive-kappa-law
          (:baseline-kappa min-prior-floor-mg-ratio
           :unobserved-root-contact-kappa uninstrumented-prior-boost
           :min-denominator-newton-floor 85.0)))) ; Hard mathematical guard against 0/0!

;; -----------------------------------------------------------------------------
;; 4b. A SUPPORT STANDS STILL (HERS)
;; What bears her does not move through what it stands on: a point of hers moving past her grip's slip -
;; a hand she lifts, a sole in swing, a limb she moves herself - takes no prior share and joins the split
;; only where nothing still would. Her feet and hands on the floor bear her; her seat once it lands.
;; -----------------------------------------------------------------------------
(defprimitive gate-supports-by-stillness
  [state &key (support-slip-mps 0.15)]          ; her law's slip: a contact slides past it (`wbc.SLIDE_M_S`)
  (-> state
      (gate-manifold-simplex-by-patch-speed
        :observer-bank 'tri-imu-observer-soa-state
        :patch-speed-source :encoder-forward-kinematics
        :still-below-mps support-slip-mps
        :on-moving-patch
          (:zero-prior-share true
           :hold-out-of-free-support-set true))))

;; -----------------------------------------------------------------------------
;; 5. ACTIVE PROPRIOCEPTIVE WEIGHT-SHIFT PROBER (TEST BY REDISTRIBUTING WEIGHT!)
;; When sitting or lying on a soft, unmeasured surface (kappa > 0.45), injects a
;; gentle, bounded weight redistribution (via limb cantilever or trunk shift)
;; and demodulates the 3-IMU pitch/roll admittance response d(theta_IMU)/d(tau_probe)
;; to verify support stiffness and detect hidden mattress/cushion edges!
;; Hers: none of it on her loop.
;; -----------------------------------------------------------------------------
(defprimitive probe-unobserved-substrate-via-weight-shift
  [state &key (probe-trigger-kappa 0.45)
              (max-probe-com-shift-mm 16.0)
              (probe-frequency-hz 1.8)
              (edge-drop-tilt-ceiling-rad 0.042)]
  (-> state
      (run-active-proprioceptive-weight-probe
        :observer-bank 'tri-imu-observer-soa-state
        :activate-when-prior-kappa-above probe-trigger-kappa
        ;; 1. Inject bounded test weight-shift across [Anterior, Posterior, Left, Right]:
        :excitation-law
          (:com-excursion-limit-m (* 0.001 max-probe-com-shift-mm)
           :probe-carrier-hz probe-frequency-hz
           :effector-synergy :nullspace-limb-and-thoracic-cantilever)
        ;; 2. Demodulate 3-IMU Rotational Admittance Response:
        :imu-response-evaluator
          (:sensors [:pelvis-hip-imu :thorax-torso-imu :head-atlas-imu]
           :max-allowable-tilt-rad edge-drop-tilt-ceiling-rad
           ;; If tilting toward a quadrant causes Pelvis/Thorax IMU to pitch > 0.042 rad,
           ;; mark that direction as a soft/missing edge and retract CoM immediately!
           :on-excessive-imu-yield
             (:shrink-cwc-polygon-in-probe-direction true
              :retract-com-toward-stiffest-quadrant true)
           ;; If IMU response is firm and bounded, update posterior stiffness & confirm support:
           :on-firm-imu-resistance
             (:confirm-prior-manifold-weight true
              :write-estimated-rotational-stiffness-to-cwc true)))))

;; -----------------------------------------------------------------------------
;; 6. MASTER PRIOR-REGULARIZED OBSERVER & COMPOSABLE SPECIALIZERS
;; -----------------------------------------------------------------------------
(defmodifier xform-prior-regularized-observer [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 35.0)
      (fuse-tri-imu-axial-kinematics :complementary-cutoff-hz 18.0)
      (collapse-static-equilibrium-innovation :static-energy-threshold 1.0)
      (regularize-manifold-simplex-with-priors
        :min-prior-floor-mg-ratio 1.0e-6                ; (0.02)
        :uninstrumented-prior-boost 0.85)
      (gate-supports-by-stillness :support-slip-mps 0.15)
      (probe-unobserved-substrate-via-weight-shift
        :probe-trigger-kappa 0.45
        :max-probe-com-shift-mm 16.0)))

;; Specializer A: Sensorless Soft Bed / Sofa Mode (High Prior Trust + Continuous Probing)
(defmodifier specialize-observer-for-sensorless-soft-bed
  [state &key (prior-dominance 0.92) (probe-shift-mm 22.0)]
  (-> state
      (regularize-manifold-simplex-with-priors
        :min-prior-floor-mg-ratio 0.15
        :uninstrumented-prior-boost prior-dominance
        :gravity-alignment-beta 8.0)
      (probe-unobserved-substrate-via-weight-shift
        :probe-trigger-kappa 0.30
        :max-probe-com-shift-mm probe-shift-mm
        :probe-frequency-hz 1.4)))

;; Specializer B: Active Edge-Seeking Probe Before Sit-to-Stand or Roll
(defmodifier specialize-observer-with-active-edge-probing
  [state &key (test-direction :anterior-egress) (max-test-lean-rad 0.035)]
  (-> state
      (probe-unobserved-substrate-via-weight-shift
        :probe-trigger-kappa 0.20
        :max-probe-com-shift-mm 28.0
        :edge-drop-tilt-ceiling-rad max-test-lean-rad)))
