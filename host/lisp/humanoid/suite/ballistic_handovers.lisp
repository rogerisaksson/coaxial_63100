;; =============================================================================
;; FILE: ballistic_handovers.lisp
;; DOMAIN: Geometry-Parameterized Descent Envelopes & Wavefront Impedance Release
;; =============================================================================
(defpackage :ortho-dsp.ballistic-handovers
  (:use :ortho-dsp.core)
  (:export #:handover-staggered-vertical-chair
           #:handover-trochanteric-bed-pendulum
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
        :trigger-fz-ratio 0.15             ; the robot's seat bears it from 0.15 of its weight (impedance.PROP_ON)
        :max-impact-velocity-mps 0.02           ; Silent, zero-thump contact
        (crossfade-support-manifold
          :from-bus :plantar-bilateral
          :to-bus :ischial-bilateral
          :transfer-time-ms 240.0
          :curve :equal-power
          :thoracic-settle-rad backrest-pitch-rad))))

;; -----------------------------------------------------------------------------
;; 2. ELEVATED BED: 135-DEG DIAGONAL TROCHANTERIC PENDULUM ROLL
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
;; 3. PROGRESSIVE WAVEFRONT IMPEDANCE RELEASE (POST-CONTACT MUSCLE MELT)
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
