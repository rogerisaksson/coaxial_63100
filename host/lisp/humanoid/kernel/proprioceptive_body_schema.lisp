;; -----------------------------------------------------------------------------
;; MODULE 1.3: proprioceptive_body_schema
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.body-schema
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics)
  (:export #:evaluate-lse-clearance-field
           #:extrude-velocity-braking-cocoon
           #:enforce-second-order-ecbf-shield
           #:gate-affordance-contact-permeability
           #:splice-carried-payload-envelope
           #:xform-universal-spatial-awareness
           #:specialize-with-self-collision-manifold
           #:specialize-with-nullspace-limb-retraction
           #:specialize-with-blindspot-memory-persistence
           #:specialize-with-tactile-surprise-reflex))
(in-package :ortho-dsp.body-schema)

(defprimitive evaluate-lse-clearance-field
  [state &key (schema 'anatomical-capsule-schema)
              (object-registry :active-dual-quadric-registry)
              (lse-alpha 25.0)]
  (-> state
      (compute-smooth-capsule-distances
        :body-schema schema
        :world-quadrics object-registry
        :world-sdf-slabs [:sub-tarsal-slab :mid-pelvic-slab :high-thoracic-slab]
        :aggregation-kernel (:log-sum-exp :alpha lse-alpha)
        :outputs [:min-distance-vector-d
                  :clearance-jacobian-jc
                  :clearance-hessian-velocity-product
                  :crural-foothold-sensitivity-df])))

(defprimitive extrude-velocity-braking-cocoon
  [state &key (static-skin-margin-mm 12.0)
              (braking-time-horizon-s 0.18)
              (couple-bemf-decel-limits true)]
  (-> state
      (inflate-capsules-by-stopping-distance
        :base-margin-m (* 0.001 static-skin-margin-mm)
        :lookahead-s braking-time-horizon-s
        :deceleration-model (if couple-bemf-decel-limits
                                :live-bemf-voltage-bounded-torque
                                :nominal-peak-torque)
        :uncertainty-inflation-sigma 2.0)))

(defprimitive enforce-second-order-ecbf-shield
  [state &key (ecbf-natural-freq-hz 11.0)
              (ecbf-damping-ratio 1.05)
              (nullspace-priority-ratio 0.92)]
  (let [omega-n (* 2.0 3.14159265 ecbf-natural-freq-hz)
        kp      (* omega-n omega-n)
        kd      (* 2.0 ecbf-damping-ratio omega-n)]
    (-> state
        (inject-second-order-ecbf-constraints
          :position-gain-kp kp
          :velocity-gain-kd kd
          :allocation-hierarchy
            (:primary-evasion-space :khatib-dynamically-consistent-nullspace
             :nullspace-saturation-threshold (- 1.0 nullspace-priority-ratio)
             :centroidal-spillover-bus :centroidal-balance-bus
             :export-kkt-multipliers-to-wayfinder true)))))

(defprimitive gate-affordance-contact-permeability
  [state &key (approach-window-mm 160.0) (preserve-tangential-guard true)
              (allow-contact-zones []) (target-object-id :active-interaction-quadric)]
  (-> state
      (modulate-ecbf-permeability-mask
        :active-contact-manifolds :live-and-scheduled-patches
        :allowed-contact-zones    allow-contact-zones
        :target-quadric-slot      target-object-id
        :normal-attenuation-ramp-m (* 0.001 approach-window-mm)
        :unscheduled-limb-protection 1.00
        :keep-self-collision-active  preserve-tangential-guard)))

(defprimitive splice-carried-payload-envelope
  [state &key (payload-slot :active-coupled-quadric)
              (attach-frame :ulnar-carpal-shove-bilateral)
              (safety-buffer-mm 28.0)]
  (-> state
      (attach-dynamic-capsule-to-schema
        :schema 'anatomical-capsule-schema
        :source-quadric payload-slot
        :parent-frame attach-frame
        :extra-margin-m (* 0.001 safety-buffer-mm)
        :enable-payload-vs-body-self-check true
        :enable-payload-vs-world-check     true)))

(defmodifier xform-universal-spatial-awareness [state]
  (-> state
      (evaluate-lse-clearance-field :lse-alpha 25.0)
      (extrude-velocity-braking-cocoon :static-skin-margin-mm 14.0 :couple-bemf-decel-limits true)
      (gate-affordance-contact-permeability :approach-window-mm 160.0)
      (enforce-second-order-ecbf-shield :ecbf-natural-freq-hz 11.0 :ecbf-damping-ratio 1.05)))

(defmodifier specialize-with-self-collision-manifold
  [spatial-state &key (min-malleolus-gap-mm 7.5) (min-arm-torso-gap-mm 15.0)]
  (-> spatial-state
      (configure-self-collision-barriers
        :coronal-foot-pair   (:capsules [:foot-shell-l :foot-shell-r]
                              :min-clearance-m (* 0.001 min-malleolus-gap-mm)
                              :evasion-dof :swing-circumduction-bow)
        :crural-cross-pair   (:capsules [:shank-crural-l :shank-crural-r :knee-patellar-l :knee-patellar-r]
                              :min-clearance-m 0.012
                              :evasion-dof :coxofemoral-external-rotation)
        :appendage-core-pair (:capsules [:forearm-ulnar-l :forearm-ulnar-r :pelvic-girdle :thoracic-ribcage]
                              :min-clearance-m (* 0.001 min-arm-torso-gap-mm)
                              :evasion-dof :glenohumeral-abduction))))

(defmodifier specialize-with-nullspace-limb-retraction
  [spatial-state &key (retraction-influence-radius-mm 280.0)]
  (-> spatial-state
      (bind-nullspace-evasion-synergies
        :activation-distance-m (* 0.001 retraction-influence-radius-mm)
        :synergy-primitives
          ((:capsules [:upper-arm-humeral-l :upper-arm-humeral-r :forearm-ulnar-l :forearm-ulnar-r]
             :repulsion-mapping [:glenohumeral-adduction :cubital-flexion :thoracic-axial-yaw])
           (:capsules [:pelvic-girdle :lumbar-column]
             :repulsion-mapping [:l4-l5-lumbar-flexion :iliac-coronal-shift])
           (:capsules [:cranial-vault]
             :repulsion-mapping [:c7-t12-thoracic-flexion :c1-c2-atlas-pitch-comp :sacroiliac-z-duck])))))

(defmodifier specialize-with-blindspot-memory-persistence
  [spatial-state &key (max-memory-horizon-s 45.0) (uncertainty-growth-mmps 1.5)]
  (-> spatial-state
      (maintain-out-of-frustum-ecbf-anchors
        :registry :active-dual-quadric-registry
        :persistence-horizon-s max-memory-horizon-s
        :se3-ego-propagation :right-invariant-ekf-coupled
        :blindspot-margin-inflation-rate-mps (* 0.001 uncertainty-growth-mmps)
        :max-inflation-cap-m 0.120)))

(defmodifier specialize-with-tactile-surprise-reflex
  [spatial-state &key (unexpected-force-thresh-n 9.5) (phantom-decay-s 12.0)]
  (-> spatial-state
      (bind-unexpected-wrench-recoil
        :observer-source :generalized-momentum-observer
        :unscheduled-force-threshold-n unexpected-force-thresh-n
        :on-unscheduled-contact
          (:yield-admittance-along-wrench true
           :spawn-virtual-quadric-at-contact-point true
           :virtual-quadric-radius-m 0.150
           :virtual-quadric-ttl-s phantom-decay-s))))
