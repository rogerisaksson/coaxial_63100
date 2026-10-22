;; =============================================================================
;; FILE: 08_substrate_impedance.lisp
;; DOMAIN: External Furniture Compliance Filters & High-Z Inverter Gating
;; =============================================================================
(defpackage :ortho-dsp.substrates
  (:use :ortho-dsp.core)
  (:export #:substrate-rigid-fixture
           #:substrate-viscoelastic-mattress
           #:substrate-compliant-cushion
           #:gate-inverter-high-z
           #:wake-inverter-closed-loop))
(in-package :ortho-dsp.substrates)

;; 1. Rigid Unyielding Fixture (Wood/Metal Bench): High damping, zero resonance notch
(defmodifier substrate-rigid-fixture [state]
  (-> state
      (configure-contact-compliance
        :surface-stiffness-npmm 1500.0
        :impact-pad-saturation :hard-clip
        :post-contact-limb-gain-db -24.0)))

;; 2. Viscoelastic Spring Mattress: Notch-filters mattress eigenfrequency (1.5 - 2.2 Hz)
(defmodifier substrate-viscoelastic-mattress [state &key (eigenfreq-hz 1.80)]
  (-> state
      (configure-contact-compliance
        :surface-stiffness-npmm 18.5
        :impact-pad-saturation :tanh-viscoelastic
        :biquad-notch-hz eigenfreq-hz
        :notch-attenuation-db -18.0
        :notch-q-factor 1.20
        :post-contact-limb-gain-db -36.0)))

;; 3. Deep Compliant Upholstery (Sofa): Compensates for slow creep and sinkage offset
(defmodifier substrate-compliant-cushion [state &key (sinkage-allowance-mm 65.0)]
  (-> state
      (configure-contact-compliance
        :surface-stiffness-npmm 42.0
        :creep-relaxation-ms 450.0
        :z-sinkage-offset-mm (- sinkage-allowance-mm)
        :post-contact-limb-gain-db -28.0)))

;; -----------------------------------------------------------------------------
;; HIGH-Z INVERTER GATE MANAGEMENT (ZERO-TORQUE STANDBY <-> ACTIVE PRECHARGE)
;; -----------------------------------------------------------------------------
(defmodifier gate-inverter-high-z [state &key (verify-dwell-ms 400.0)]
  (-> state
      (on-manifold-equilibrium
        :min-distributed-fz-ratio 0.96
        :dwell-verify-ms verify-dwell-ms

        (cascade-actuator-gates
          :direction :distal-to-proximal
          :nodes [:calcaneus :tibiofemoral :coxofemoral :l4-l5-lumbar
                  :c7-t12-thoracic :glenohumeral :c1-c2-atlas :craniofacial]
          :stagger-ms 75.0
          :torque-decay-ms 300.0
          :bridge-state :high-z-passive-bypass
          :master-clock :halted))))

(defmodifier wake-inverter-closed-loop [state]
  (-> state
      (latch-current-encoders :set-as-zero-error-reference true)
      (cascade-actuator-gates
        :direction :proximal-to-distal
        :nodes [:l4-l5-lumbar :c7-t12-thoracic :coxofemoral :tibiofemoral
                :calcaneus :glenohumeral :c1-c2-atlas :craniofacial]
        :stagger-ms 50.0
        :torque-ramp-ms 250.0
        :bridge-state :pwm-active-closed-loop
        :master-clock :armed)))
