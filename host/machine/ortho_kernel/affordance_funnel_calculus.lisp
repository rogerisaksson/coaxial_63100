;; -----------------------------------------------------------------------------
;; MODULE 2.5: affordance_funnel_calculus
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.affordance-core
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.body-schema
        :ortho-dsp.manipulation
        :ortho-dsp.nav-core
        :ortho-dsp.perception-core)
  (:export #:evaluate-situational-affordances
           #:define-postural-funnel-topology
           #:solve-minimum-action-funnel-chain
           #:splice-manipulated-object-adjoint))
(in-package :ortho-dsp.affordance-core)

(defprimitive evaluate-situational-affordances
  [state &key (registry 'spatial-object-soa-registry) (min-salience-log-odds +1.50)]
  (-> state
      (compute-affordance-compatibility-tensor
        :object-registry registry :salience-floor min-salience-log-odds
        :morphological-kernels
          ((:affordance :sit-on-fixture
             :anatomical-patch [:ischial-bilateral :posterior-femoral-bilateral]
             :normal-alignment-cos-min 0.88 :elevation-window-z-mm [280.0 920.0]
             :min-polygon-area-mm2 75000.0 :headroom-slab-clear [:mid-pelvic-slab :high-thoracic-slab])
           (:affordance :lie-supine-on-fixture
             :anatomical-patch :dorsal-supine-continuum
             :normal-alignment-cos-min 0.92 :elevation-window-z-mm [0.0 750.0]
             :min-major-axis-length-mm 1350.0 :headroom-slab-clear [:high-thoracic-slab])
           (:affordance :relocate-mobile-object
             :anatomical-patch [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
             :grasp-aperture-window-mm [12.0 120.0] :grasp-height-window-z-mm [420.0 1150.0]
             :max-estimated-mass-kg 22.0 :max-breakaway-shear-n 140.0)
           (:affordance :recruit-brace-support
             :anatomical-patch [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
             :elevation-window-z-mm [380.0 1050.0] :min-stiffness-npmm 120.0)))))

(defprimitive define-postural-funnel-topology [state]
  (-> state
      (register-funnel-preimage-graph
        :nodes
          ((:funnel :bipedal-upright
             :inlet-preimage  (:w-plantar-or-equinus [0.75 1.00] :dcm-margin-min-mm 12.0)
             :outlet-manifold (:w-plantar 1.00 :com-velocity-max-mps 1.80))
           (:funnel :docking-pivot
             :inlet-preimage  (:w-plantar [0.85 1.00] :target-standoff-dist-m [0.05 0.85])
             :outlet-manifold (:w-plantar 1.00 :yaw-error-max-rad 0.25 :com-velocity-max-mps 0.35))
           (:funnel :ischial-seated
             :inlet-preimage  (:or (:and (:w-plantar-or-equinus [0.75 1.00]) (:yaw-error-max-rad 0.785))
                                   (:w-dorsal [0.60 1.00]))
             :outlet-manifold (:w-ischial [0.70 1.00] :w-plantar [0.00 0.30]))
           (:funnel :dorsal-supine
             :inlet-preimage  (:w-ischial [0.55 0.95] :trochanter-to-surface-max-mm 140.0)
             :outlet-manifold (:w-dorsal [0.95 1.00] :inverter-state :high-z-or-low-bias))
           (:funnel :somatic-object-coupled
             :inlet-preimage  (:w-plantar [0.85 1.00] :ulnar-or-palmar-to-object-max-mm 180.0)
             :outlet-manifold (:w-plantar 1.00 :object-wrench-latched true))))))

(defprimitive solve-minimum-action-funnel-chain
  [state &key (goal-affordance :lie-supine-on-fixture)
              (target-object-id :auto-salient)
              (occlusion-work-threshold-j 85.0)]
  (-> state
      (synthesize-funnel-trajectory
        :initial-state :live-manifold-weights-and-se3
        :terminal-goal (:affordance goal-affordance :object target-object-id)
        :cost-functional
          (:mechanical-work-weight 1.0 :wrench-margin-risk-weight 2.4 :manifold-transition-smoothness 1.8)
        :occlusion-resolution-law
          (:detect-sdf-funnel-blockage true
           :max-relocation-work-joules occlusion-work-threshold-j
           :displacement-vector-law :orthogonal-to-geodesic-corridor))))

(defprimitive splice-manipulated-object-adjoint
  [state &key (object-slot :active-grasped-quadric) (transport-mode :auto-slide-or-lift)]
  (-> state
      (allocate-grasp-matrix-wrench :min-internal-squeeze-n 12.0)
      (engage-incipient-slip-reflex)
      (splice-carried-payload-envelope :payload-slot object-slot :safety-buffer-mm 30.0)
      (couple-external-rigid-body-to-cwc
        :quadric-slot object-slot
        :grasp-frames [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :adjoint-map  :se3-dual-wrench-transform
        :online-inertial-id
          (:estimate-mass-and-com true :estimate-floor-mu true
           :mode-selection-law (case transport-mode
                                 (:auto-slide-or-lift (:lift-if-shear-exceeds-n 45.0 :lift-z-mm 35.0))
                                 (otherwise transport-mode)))
        :centroidal-counterbalance
          (:pelvic-sagittal-counter-shift true :widen-coronal-track-mm 35.0))))
