;; =============================================================================
;; FILE: articulated_body_dynamics.lisp
;; DOMAIN: Lie-Algebraic RNEA/ABA, Operational-Space Inertia & Whole-Body Torque
;; =============================================================================
(defpackage :ortho-dsp.aba-dynamics
  (:use :ortho-dsp.core)
  (:export #:compute-screw-mass-matrix
           #:solve-dynamically-consistent-nullspace
           #:allocate-whole-body-torque))
(in-package :ortho-dsp.aba-dynamics)

;; 1. Lie-Group Recursive Newton-Euler & Composite-Rigid-Body Mass Matrix M(q)
;; Evaluates spatial inertia tensors I_i, Coriolis matrix C(q,dq), and gravity g(q)
;; in O(N) via Lie-bracket adjoint twists [ad_V_i] on se(3).
(defprimitive compute-screw-mass-matrix [state &key (include-rotor-reflected-inertia true)]
  (-> state
      (evaluate-lie-articulated-dynamics
        :kinematic-tree :full-humanoid-branch-chain
        :twist-propagate :se3-adjoint-exp
        :include-armature-j-g2 include-rotor-reflected-inertia
        :outputs [:mass-matrix-m :nonlinear-bias-cg :centroidal-momentum-ag])))

;; 2. Dynamically Consistent Generalized Inverse: J_bar = W^-1 * J_c^T * (J_c * W^-1 * J_c^T)^-1
;; Guarantees that any posture torque projected into (I - J_c^T * J_bar^T) generates
;; strictly ZERO operational-space acceleration at active contact manifolds!
(defprimitive solve-dynamically-consistent-nullspace
  [state &key (weighting-metric :joint-mass-and-thermal-w)]
  (-> state
      (compute-operational-space-projector
        :contact-jacobian :active-weighted-jc
        :inertia-metric   weighting-metric          ; W(q, T) = M(q) * diag(thermal_derating)
        :svd-damping-floor 1e-4
        :projector-form   :khatib-dynamically-consistent))) ; P_dyn = I - J_c^T * J_bar^T

;; 3. Master Whole-Body Torque Allocator (Bridges 6D Wrench + Null-Space -> Joint Nm)
;; tau_cmd = J_c^T * F_cwc + (I - J_c^T * J_bar^T) * (M * ddq_des + K_p*e + K_d*de) + C*dq + g
(defmodifier allocate-whole-body-torque [state]
  (-> state
      (compute-screw-mass-matrix :include-rotor-reflected-inertia true)
      (solve-dynamically-consistent-nullspace :weighting-metric :joint-mass-and-thermal-w)
      (synthesize-joint-torques
        :primary-task   (:wrench-source :optimal-cwc-contact-forces)
        :secondary-task (:posture-source :kinematic-channel-strip-targets
                         :projection     :dynamically-consistent-nullspace)
        :feedforward    (:coriolis-centrifugal true :gravity-vector true))))
