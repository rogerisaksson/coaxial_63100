;; -----------------------------------------------------------------------------
;; MODULE 1.16: electro_thermal_governor
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.power-thermal
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics)
  (:export #:govern-winding-thermal-metric
           #:shunt-regenerative-bus-overvoltage))
(in-package :ortho-dsp.power-thermal)

(defmodifier govern-winding-thermal-metric
  [state &key (soft-derate-onset-c 88.0) (hard-ceiling-c 115.0)]
  (-> state
      (modulate-nullspace-inertia-metric
        :temperature-estimator :lumped-stator-housing-kalman
        :onset-temp-c soft-derate-onset-c :ceiling-temp-c hard-ceiling-c
        :cost-inflation-law :exponential-barrier)))

(defmodifier shunt-regenerative-bus-overvoltage
  [state &key (nominal-bus-v 48.0) (bleed-onset-v 56.5) (hard-clamp-v 60.0)]
  (-> state
      (regulate-dc-bus-regeneration
        :nominal-voltage-v nominal-bus-v :bleed-threshold-v bleed-onset-v :max-voltage-v hard-clamp-v
        :dissipation-mechanism :nullspace-isometric-co-contraction
        :priority-sink-nodes [:glenohumeral :cubital :l4-l5-lumbar])))
