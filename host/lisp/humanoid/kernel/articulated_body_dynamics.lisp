;; -----------------------------------------------------------------------------
;; MODULE 1.2: articulated_body_dynamics
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.aba-dynamics
  (:use :ortho-dsp.core)
  (:export #:compute-screw-mass-matrix
           #:update-asymmetric-bemf-bounds
           #:solve-dynamically-consistent-nullspace
           #:allocate-whole-body-torque))
(in-package :ortho-dsp.aba-dynamics)

(defprimitive compute-screw-mass-matrix [state &key (include-rotor-reflected-inertia true)]
  (-> state
      (evaluate-lie-articulated-dynamics
        :kinematic-tree :full-humanoid-branch-chain
        :plucker-basis-order :angular-first-featherstone ; [omega^T, v^T]^T and [tau^T, f^T]^T
        :twist-propagate :se3-adjoint-exp
        :include-armature-j-g2 include-rotor-reflected-inertia
        :outputs [:mass-matrix-m :nonlinear-bias-cg :centroidal-momentum-ag])))

(defprimitive update-asymmetric-bemf-bounds [state]
  (-> state
      (compute-dynamic-voltage-torque-envelope
        :bus-voltage-source :live-dc-link-sensor
        :svpwm-modulation-limit 0.57735027         ; V_bus / sqrt(3)
        :quadrant-law :asymmetric-bemf-shift       ; tau_max/min(dq) = (N*kt/R)*(+/-V_max - N*ke*dq)
        :export-to-qp-box-constraints true)))

(defprimitive solve-dynamically-consistent-nullspace
  [state &key (weighting-metric :joint-mass-and-thermal-w)]
  (-> state
      (compute-operational-space-projector
        :contact-jacobian :active-weighted-jc
        :inertia-metric   weighting-metric
        :svd-damping-floor 1e-4
        :projector-form   :khatib-dynamically-consistent)))

(defmodifier allocate-whole-body-torque [state]
  (-> state
      (compute-screw-mass-matrix :include-rotor-reflected-inertia true)
      (update-asymmetric-bemf-bounds)
      (solve-dynamically-consistent-nullspace :weighting-metric :joint-mass-and-thermal-w)
      (synthesize-joint-torques
        :primary-task   (:wrench-source :optimal-cwc-contact-forces)
        :secondary-task (:posture-source :kinematic-channel-strip-targets
                         :evasion-source :ecbf-nullspace-repulsion-torques
                         :projection     :dynamically-consistent-nullspace)
        :feedforward    (:coriolis-centrifugal true :gravity-vector true)
        :box-clamp      :live-asymmetric-bemf-bounds)))
