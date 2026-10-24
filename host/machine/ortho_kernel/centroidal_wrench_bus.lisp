;; =============================================================================
;; FILE: centroidal_wrench_bus.lisp
;; DOMAIN: SE(3)* Dual-Adjoint Wrench Homotopy, Poinsot Ray & Null-Space Engine
;; =============================================================================
(defpackage :ortho-dsp.balance-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics)
  (:export #:pan-wrench-homotopy
           #:splice-auxiliary-manifold
           #:project-nullspace-posture))
(in-package :ortho-dsp.balance-core)

(defprimitive pan-wrench-homotopy
  [state &key (from-manifolds '((:plantar-bilateral 1.0)))
              (to-manifolds   '((:ischial-bilateral 0.85) (:plantar-bilateral 0.15)))
              (pan-law :equal-wrench-power)
              (poinsot-ray-slew :minimum-jerk)
              (sync-trigger :contact-comparator-or-phase)
              (duration-ms 450.0)]
  (-> state
      (interpolate-adjoint-cones
        :source-weights from-manifolds
        :target-weights to-manifolds
        :interpolation-law pan-law
        :duration-ms duration-ms
        :trigger sync-trigger
        :central-axis-homotopy
          (:trajectory poinsot-ray-slew
           :enforce-interior-margin-nm 14.0
           :compensate-coriolis-centrifugal true))
      (reallocate-nullspace-projector
        :projector-type :khatib-dynamically-consistent
        :weight-source :active-manifold-weights
        :unloaded-dof-damping 0.85
        :continuity-guard :c2-bumpless)))

(defprimitive splice-auxiliary-manifold
  [state &key (patch-id :ulnar-armrest-bilateral)
              (anatomical-frames [:ulna-l :ulna-r])
              (surface-normal [0.0 0.0 1.0])
              (cop-box-mm '(:x [-80.0 80.0] :y [-25.0 25.0]))
              (mu 0.50)
              (bilateral-grasp-pull false)
              (weight-law (:envelope [0.0 -> 0.20 -> 0.0]))
              (window-ms [200.0 -> 850.0])]
  (-> state
      (inject-adjoint-cone
        :id patch-id
        :frames anatomical-frames
        :normal surface-normal
        :cop-bounds cop-box-mm
        :mu-static mu
        :unilateral-only (not bilateral-grasp-pull)
        :gain-law weight-law
        :active-window-ms window-ms)))
