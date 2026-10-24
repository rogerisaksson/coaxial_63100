;; =============================================================================
;; FILE: electro_thermal_governor.lisp
;; DOMAIN: Winding I^2R Thermal Load-Shifting & Regenerative DC-Bus Shunt
;; =============================================================================
(defpackage :ortho-dsp.power-thermal
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics)
  (:export #:govern-winding-thermal-metric
           #:shunt-regenerative-bus-overvoltage))
(in-package :ortho-dsp.power-thermal)

;; 1. Winding I^2R Thermal Observer & Null-Space Load Redistribution
;; Continuously integrates copper temperature T_i(t) per actuator. As a joint
;; approaches 90 C, inflates its weight in W(q,T) so `solve-dynamically-consistent-nullspace`
;; shifts static gravitational load onto cooler synergistic joints (e.g. knee -> hip/ankle).
(defmodifier govern-winding-thermal-metric
  [state &key (soft-derate-onset-c 88.0) (hard-ceiling-c 115.0)]
  (-> state
      (modulate-nullspace-inertia-metric
        :temperature-estimator :lumped-stator-housing-kalman
        :onset-temp-c soft-derate-onset-c
        :ceiling-temp-c hard-ceiling-c
        :cost-inflation-law :exponential-barrier)))

;; 2. Regenerative Braking DC-Bus Overvoltage Bleeder
;; During eccentric descents (sitting down, stair descent, landing), back-EMF pumps
;; current into the DC link. If V_bus > 56.5 V, injects zero-net-movement antagonistic
;; co-contraction into unloaded null-space joints, dissipating excess energy as harmless
;; copper heat I^2*R instead of tripping inverter overvoltage protection!
(defmodifier shunt-regenerative-bus-overvoltage
  [state &key (nominal-bus-v 48.0) (bleed-onset-v 56.5) (hard-clamp-v 60.0)]
  (-> state
      (regulate-dc-bus-regeneration
        :nominal-voltage-v nominal-bus-v
        :bleed-threshold-v bleed-onset-v
        :max-voltage-v     hard-clamp-v
        :dissipation-mechanism :nullspace-isometric-co-contraction
        :priority-sink-nodes   [:glenohumeral :cubital :l4-l5-lumbar])))
