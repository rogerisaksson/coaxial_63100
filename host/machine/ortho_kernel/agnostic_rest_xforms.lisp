;; -----------------------------------------------------------------------------
;; MODULE 1.12: agnostic_rest_xforms
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.agnostic-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:xform-blind-posterior-settle
           #:xform-blind-recumbent-flow
           #:specialize-with-reactive-capture-step
           #:specialize-with-tactile-arm-sweep
           #:specialize-with-deep-squat-suspension
           #:specialize-with-viscoelastic-sink-compensation))
(in-package :ortho-dsp.agnostic-xforms)

(defmodifier xform-blind-posterior-settle [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 35.0)
      (bind-wrench-driven-homotopy
        :active-manifolds [:plantar-bilateral :metatarsal-equinus-bilateral
                           :posterior-femoral-bilateral :ischial-bilateral]
        :slew-rate-up-per-s 4.5 :slew-rate-down-per-s 45.0 :couple-impedance-melt true)
      (project-blind-admittance-field
        :seek-velocity-xyz-mps [-0.12 0.0 -0.30] :virtual-damping-nsm 280.0 :axial-pitch-compliance 0.70)
      (enforce-dcm-barrier-zcbf :barrier-gamma 14.0 :min-capture-margin-mm 18.0)))

(defmodifier xform-blind-recumbent-flow [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 35.0)
      (bind-wrench-driven-homotopy
        :active-manifolds [:plantar-bilateral :ischial-bilateral
                           :trochanter-gluteal-lateral :dorsal-supine-continuum]
        :slew-rate-up-per-s 3.8 :slew-rate-down-per-s 40.0 :couple-impedance-melt true)
      (project-blind-admittance-field
        :seek-velocity-xyz-mps [-0.18 0.12 -0.26] :virtual-damping-nsm 240.0 :axial-pitch-compliance 0.92)
      (enforce-dcm-barrier-zcbf :barrier-gamma 12.0 :min-capture-margin-mm 15.0)))

(defmodifier specialize-with-reactive-capture-step
  [blind-state &key (step-clearance-z-mm 45.0) (capture-lead-gain 1.12)]
  (-> blind-state
      (configure-tangential-unlatch-law
        :patch :plantar-bilateral
        :unlatch-condition (:dcm-boundary-flux-mps :positive-outward)
        :swing-foot-attractor
          (:target-xy-law :instantaneous-capture-point :lead-multiplier capture-lead-gain
           :apex-clearance-z-mm step-clearance-z-mm :re-anchor-on-fz-ratio 0.15))))

(defmodifier specialize-with-tactile-arm-sweep
  [blind-state &key (max-arm-support-share 0.18)]
  (-> blind-state
      (splice-auxiliary-manifold
        :patch-id :tactile-ulnar-palmar-probe
        :anatomical-frames [:ulna-distal-l :ulna-distal-r :carpal-heel-l :carpal-heel-r]
        :surface-normal [0.0 0.0 1.0] :cop-box-mm '(:x [-110.0 110.0] :y [-40.0 40.0])
        :mu 0.50 :weight-law (:wrench-proportional :max-share max-arm-support-share))
      (drive-appendage-admittance
        :nodes [:glenohumeral :cubital] :seek-velocity-mps [0.0 0.0 -0.25]
        :contact-yield-damping 45.0 :free-air-fallback :centroidal-momentum-flywheel)))

(defmodifier specialize-with-deep-squat-suspension
  [blind-state &key (hover-floor-z-mm 210.0) (allow-tibial-touchdown true)]
  (-> blind-state
      (modulate-barrier-floor
        :min-com-z-mm hover-floor-z-mm :bottom-out-elasticity-npmm 85.0
        :bottom-out-damping-ratio 0.95 :counterweight-arm-cantilever true)
      (when-modifier allow-tibial-touchdown
        (splice-auxiliary-manifold
          :patch-id :contingency-tibial-keel
          :anatomical-frames [:tibial-tuberosity-r :tibial-tuberosity-l]
          :surface-normal [0.0 0.0 1.0] :cop-box-mm '(:x [-140.0 40.0] :y [-35.0 35.0])
          :mu 0.75 :weight-law (:wrench-proportional :max-share 0.60)))))

(defmodifier specialize-with-viscoelastic-sink-compensation [blind-state]
  (-> blind-state
      (bind-adaptive-substrate-notch
        :stiffness-estimator :recursive-dF-dz-gradient
        :eigenfreq-law (:sqrt-k-over-m :clamp-hz [1.10 8.50])
        :notch-attenuation-db -18.0 :creep-velocity-compensation true)))
