;; =============================================================================
;; FILE: blind_admittance_fields.lisp
;; DOMAIN: Momentum-Observer Wrench Homotopy, Tactile Admittance & DCM-ZCBF
;; =============================================================================
(defpackage :ortho-dsp.blind-fields
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core)
  (:export #:configure-momentum-observer
           #:bind-wrench-driven-homotopy
           #:project-blind-admittance-field
           #:enforce-dcm-barrier-zcbf))
(in-package :ortho-dsp.blind-fields)

(defprimitive configure-momentum-observer [state &key (observer-bandwidth-hz 35.0)]
  (-> state
      (enable-generalized-momentum-observer
        :filter-cutoff-hz observer-bandwidth-hz
        :friction-model :lugre-dynamic-stribeck
        :coriolis-centrifugal-comp true
        :monitored-manifolds [:plantar-bilateral
                              :ischial-bilateral
                              :trochanter-gluteal-lateral
                              :dorsal-supine-continuum
                              :palmar-carpal-bilateral])))

(defprimitive bind-wrench-driven-homotopy
  [state &key (active-manifolds [:plantar-bilateral :ischial-bilateral :dorsal-supine-continuum])
              (force-normalization :body-weight-mg)
              (slew-rate-up-per-s 4.5)
              (slew-rate-down-per-s 45.0)
              (couple-impedance-melt true)]
  (-> state
      (map-wrench-to-manifold-weights
        :manifolds active-manifolds
        :gain-scaling force-normalization
        :asymmetric-slew (:attack-rate slew-rate-up-per-s
                          :release-rate slew-rate-down-per-s)
        :sum-of-weights-invariant 1.0)
      (when-modifier couple-impedance-melt
        (bind-stiffness-to-support-share
          :law :inverse-wrench-power
          :full-support-atten-db -28.0
          :unloaded-rebound-rate-ms 15.0))))

(defprimitive project-blind-admittance-field
  [state &key (seek-velocity-xyz-mps [-0.14 0.0 -0.32])
              (virtual-mass-kg 12.0)
              (virtual-damping-nsm 280.0)
              (axial-pitch-compliance 0.65)
              (horizon-gaze-invariant true)]
  (-> state
      (drive-admittance-vector-field
        :task-space-frame :pelvic-ischial-centroid
        :free-air-seek-velocity seek-velocity-xyz-mps
        :virtual-inertia-kg virtual-mass-kg
        :damping-matrix-nsm virtual-damping-nsm
        :zero-velocity-equilibrium-fz 1.00
        :thoracic-conform-admittance
          (:dorsal-sensor :c7-t12-thoracic
           :pitch-yield-gain axial-pitch-compliance
           :restoring-spring-rad 0.08))
      (when-modifier horizon-gaze-invariant
        (modulate-node :c1-c2-atlas
          :reference-frame (:set :world-horizon)
          :vestibular-gain (:set 1.00)))))

(defprimitive enforce-dcm-barrier-zcbf
  [state &key (barrier-gamma 14.0)
              (min-capture-margin-mm 18.0)
              (kinematic-singularity-floor-rad 0.28)]
  (-> state
      (inject-qp-barrier-constraint
        :state-variable :three-dimensional-dcm
        :feasible-set :active-minkowski-wrench-hull
        :barrier-decay-rate barrier-gamma
        :safety-margin-mm min-capture-margin-mm
        :workspace-floor-barrier
          (:min-tibiofemoral-clearance-rad kinematic-singularity-floor-rad
           :deflection-mode :hover-or-capture-step))))
