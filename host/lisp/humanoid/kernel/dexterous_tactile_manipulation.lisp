;; -----------------------------------------------------------------------------
;; MODULE 1.15: dexterous_tactile_manipulation
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.manipulation
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:allocate-grasp-matrix-wrench
           #:engage-incipient-slip-reflex))
(in-package :ortho-dsp.manipulation)

(defprimitive allocate-grasp-matrix-wrench
  [state &key (contact-frames [:palmar-carpal-l :palmar-carpal-r])
              (min-internal-squeeze-n 8.0)
              (fragility-crush-ceiling-n 95.0)]
  (-> state
      (solve-grasp-wrench-decomposition
        :active-hands contact-frames
        :external-object-wrench :coupled-quadric-inertial-demand
        :internal-nullspace-squeeze
          (:baseline-preload-n min-internal-squeeze-n
           :load-proportional-gain 1.25
           :hard-crush-limit-n fragility-crush-ceiling-n))))

(defmodifier engage-incipient-slip-reflex [state]
  (-> state
      (monitor-tactile-vibrometry
        :sensors [:palmar-shear-piezo-l :palmar-shear-piezo-r]
        :bandpass-window-hz [80.0 250.0] :energy-threshold-g 0.12
        :on-incipient-ripple
          (:boost-internal-squeeze-rate-nps 450.0
           :decrement-estimated-mu-step -0.06
           :hold-decay-ms 600.0))))
