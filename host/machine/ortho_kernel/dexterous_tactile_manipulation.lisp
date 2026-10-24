;; =============================================================================
;; FILE: dexterous_tactile_manipulation.lisp
;; DOMAIN: Grasp-Matrix Null-Space Squeeze & Incipient Shear-Slip Vibrometry
;; =============================================================================
(defpackage :ortho-dsp.manipulation
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:allocate-grasp-matrix-wrench
           #:engage-incipient-slip-reflex))
(in-package :ortho-dsp.manipulation)

;; 1. Grasp Matrix Decomposition: f_contact = G^+ * W_obj + (I - G^+ * G) * f_internal
;; Separates object transport wrench from internal grasping squeeze force so lifting
;; or rotating an object never crushes fragile items or loses friction lock.
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
           :load-proportional-gain 1.25          ; Squeeze scales with tangential load / mu
           :hard-crush-limit-n fragility-crush-ceiling-n))))

;; 2. High-Frequency Incipient Slip Reflex (80-250 Hz Tactile Shear Vibrometry)
;; Detects micro-slip acoustic/shear ripples inside the palmar pads and boosts
;; normal squeeze force in < 4 ms before macro-slip occurs.
(defmodifier engage-incipient-slip-reflex [state]
  (-> state
      (monitor-tactile-vibrometry
        :sensors [:palmar-shear-piezo-l :palmar-shear-piezo-r]
        :bandpass-window-hz [80.0 250.0]
        :energy-threshold-g 0.12
        :on-incipient-ripple
          (:boost-internal-squeeze-rate-nps 450.0
           :decrement-estimated-mu-step -0.06
           :hold-decay-ms 600.0))))
