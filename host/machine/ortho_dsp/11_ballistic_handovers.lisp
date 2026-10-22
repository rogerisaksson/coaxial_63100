;; =============================================================================
;; FILE: 11_ballistic_handovers.lisp
;; DOMAIN: Geometry-Parameterized Descent Envelopes & Wavefront Impedance Release
;; =============================================================================
(defpackage :ortho-dsp.ballistic-handovers
  (:use :ortho-dsp.core)
  (:export #:handover-staggered-vertical-chair
           #:handover-guided-armchair-arc
           #:handover-parabolic-sofa-injection
           #:handover-trochanteric-bed-pendulum
           #:handover-corkscrew-floor-mattress
           #:release-impedance-wavefront))
(in-package :ortho-dsp.ballistic-handovers)

;; -----------------------------------------------------------------------------
;; 1. RIGID CHAIR: STAGGERED-FOOTPRINT UPRIGHT DESCENT (ZERO IMPACT VELOCITY)
;; Trailing foot under seat edge supports CoM, avoiding awkward 45-deg torso bow
;; -----------------------------------------------------------------------------
(defmaneuver handover-staggered-vertical-chair
  [pivoting-state &key (seat-z-mm 450.0) (backrest-pitch-rad -0.06)]
  (-> pivoting-state
      (trigger-one-shot-envelope
        :mode :eccentric-feathered-zero
        :duration-ms 820.0
        :curve :minimum-jerk-deceleration
        :apex-deceleration-height-mm (+ seat-z-mm 18.0) ; Peak braking 18mm above hard seat
        :joint-targets '((:tibiofemoral 1.54) (:coxofemoral 1.46) (:talocrural 0.38))
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad +0.14) ; Nearly vertical torso
        :appendage-counterweight (:node :glenohumeral :flexion-rad +0.10)) ; Relaxed arms

      (on-contact-threshold
        :sensor-bus :ischial-bilateral
        :trigger-fz-ratio 0.12
        :max-impact-velocity-mps 0.02           ; Silent, zero-thump contact
        (crossfade-support-manifold
          :from-bus :plantar-bilateral
          :to-bus :ischial-bilateral
          :transfer-time-ms 240.0
          :curve :equal-power
          :thoracic-settle-rad backrest-pitch-rad))))

;; -----------------------------------------------------------------------------
;; 2. ARMCHAIR: ULNAR RAIL-GUIDED ARC & OPTIONAL CONTRALATERAL LEG CROSS
;; Uses light forearm skim on armrests as coronal centering rails
;; -----------------------------------------------------------------------------
(defmaneuver handover-guided-armchair-arc
  [pivoting-state &key (seat-z-mm 420.0) (recline-pitch-rad -0.18) (cross-leg-on-settle true)]
  (-> pivoting-state
      (trigger-one-shot-envelope
        :mode :eccentric-arc-glide
        :duration-ms 900.0
        :curve :hermite-soft-landing
        :joint-targets '((:tibiofemoral 1.48) (:coxofemoral 1.35) (:talocrural 0.28))
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad +0.22)
        :tactile-rail-guide (:node :glenohumeral-ulnar-bilateral
                             :abduction-rad 0.28
                             :target-skim-fz-ratio 0.04)) ; 4% weight skim (tactile guide, not crutch)

      (on-contact-threshold
        :sensor-bus :ischial-bilateral
        :trigger-fz-ratio 0.18
        (crossfade-support-manifold
          :from-bus :plantar-bilateral
          :to-bus :ischial-dorsal-coupled
          :transfer-time-ms 340.0
          :curve :logarithmic-swell
          :thoracic-settle-rad recline-pitch-rad
          :post-settle-flourish (if cross-leg-on-settle
                                    (:coxofemoral-l-adduct-rad 0.32 :tibiofemoral-l-extend-rad 0.45)
                                    nil)))))

;; -----------------------------------------------------------------------------
;; 3. DEEP COMPLIANT SOFA: PARABOLIC SHEAR INJECTION & CUSHION REBOUND
;; Projects CoM diagonally backward (-260mm X) into the cushion crevice
;; -----------------------------------------------------------------------------
(defmaneuver handover-parabolic-sofa-injection
  [pivoting-state &key (uncompressed-z-mm 410.0) (dynamic-sinkage-mm 75.0) (throw-depth-x-mm -260.0)]
  (-> pivoting-state
      (trigger-one-shot-envelope
        :mode :ballistic-parabolic-capture
        :duration-ms 760.0
        :curve :parabolic-shear-glide
        :sagittal-com-injection-mm throw-depth-x-mm    ; Diagonal throw deep into seat
        :target-capture-z-mm (- uncompressed-z-mm dynamic-sinkage-mm)
        :joint-targets '((:tibiofemoral 1.38) (:coxofemoral 1.32))
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad +0.26))

      ;; Cushion compression absorbs horizontal momentum and rebounds torso into backrest
      (on-contact-threshold
        :sensor-bus :ischial-sacral-coupled
        :trigger-fz-ratio 0.22
        :allow-viscoelastic-travel-mm dynamic-sinkage-mm
        (absorb-and-rebound-manifold
          :horizontal-damping-ratio 0.85
          :rebound-lumbar-pitch-rad -0.16
          :rebound-thoracic-pitch-rad -0.22
          :transfer-time-ms 480.0))))

;; -----------------------------------------------------------------------------
;; 4. ELEVATED BED: 135-DEG DIAGONAL TROCHANTERIC PENDULUM ROLL
;; Torso descent levers the legs off the floor in a single coupled sweep
;; -----------------------------------------------------------------------------
(defmaneuver handover-trochanteric-bed-pendulum
  [pivoting-state &key (mattress-z-mm 540.0) (mattress-eigenfreq-hz 1.80) (bed-axis-yaw-rad 1.5708)]
  (-> pivoting-state
      (configure-contact-compliance
        :biquad-notch-hz mattress-eigenfreq-hz
        :notch-attenuation-db -18.0)

      ;; Continuous 3-stage diagonal roll: Trochanter Touch -> Torso Lever -> Axial Alignment
      (stage-coupled-pendulum
        :total-duration-ms 1550.0
        :fulcrum-node :greater-trochanter-r
        :fulcrum-touchdown-z-mm mattress-z-mm

        ;; Torso pitching down toward pillow drives contralateral leg lift via pelvic coupling
        :coupled-lever-dynamics
          (:torso-pitch-slew-rad [+0.15 -> -1.5708]
           :pelvic-yaw-slew-rad  [0.785 -> bed-axis-yaw-rad] ; Complete remaining 45 deg yaw on mattress
           :leg-pendulum-gain    1.15                        ; Passive counterweight sweep
           :knee-extension-window-ms [600.0 -> 1500.0])

        :contact-wavefront [:greater-trochanter-r -> :ischial-sacral -> :scapular-dorsal -> :occipital])))

;; -----------------------------------------------------------------------------
;; 5. FLOOR MATTRESS: HANDS-FREE CROSS-STEP CORKSCREW TO SUPINE UNROLL
;; Descends to Z = 110mm via lateral shank/hip spiral without ground hand-bracing
;; -----------------------------------------------------------------------------
(defmaneuver handover-corkscrew-floor-mattress
  [corkscrew-state &key (pad-z-mm 110.0) (pad-eigenfreq-hz 3.20)]
  (-> corkscrew-state
      (configure-contact-compliance
        :biquad-notch-hz pad-eigenfreq-hz
        :notch-attenuation-db -14.0)

      ;; Stage A: Spiral lower onto lateral gluteus/thigh with upright thoracic carriage
      (trigger-one-shot-envelope
        :mode :helical-eccentric-spiral
        :duration-ms 1050.0
        :curve :constant-angular-momentum
        :target-capture-z-mm pad-z-mm
        :joint-targets '((:tibiofemoral-r 2.15) (:tibiofemoral-l 1.95)
                         (:coxofemoral-r 1.85)  (:coxofemoral-l 1.60))
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad +0.18 :coronal-tilt-rad -0.22)
        :forbid-palmar-ground-contact true) ; Never touch hands to floor

      ;; Stage B: Sequential vertebral unroll along mattress while uncrossing legs
      (on-contact-threshold
        :sensor-bus :trochanter-gluteal-lateral
        :trigger-fz-ratio 0.25
        (stage-sequential-manifold
          :duration-ms 1250.0
          :contact-chain [:trochanter-gluteal-lateral -> :sacral -> :l4-l5-lumbar
                          -> :c7-t12-thoracic -> :occipital]
          :uncross-lower-limbs (:coronal-track-mm 140.0 :tibiofemoral-rad 0.04)
          :torso-pitch-ramp [0.18 -> -1.5708]))))

;; -----------------------------------------------------------------------------
;; 6. PROGRESSIVE WAVEFRONT IMPEDANCE RELEASE (POST-CONTACT MUSCLE MELT)
;; Decays joint stiffness in an ascending wave matched to cushion compression
;; -----------------------------------------------------------------------------
(defmodifier release-impedance-wavefront
  [state &key (terminal-mode :seated-active-rest) ; :seated-active-rest | :supine-high-z
              (wave-speed-ms-per-node 55.0)]
  (-> state
      (cascade-impedance-envelope
        :propagation-order [:calcaneus :talocrural :tibiofemoral :coxofemoral
                            :l4-l5-lumbar :c7-t12-thoracic :glenohumeral :c1-c2-atlas]
        :node-stagger-ms wave-speed-ms-per-node
        :damping-ratio-ramp [0.707 -> 0.95]            ; Eliminate post-contact spring bounce
        :stiffness-atten-db (case terminal-mode
                              (:seated-active-rest -26.0)
                              (:supine-high-z      -inf))
        :inverter-gate-mode (case terminal-mode
                              (:seated-active-rest :low-bias-gravity-comp)
                              (:supine-high-z      :high-z-passive-bypass)))))
