;; =============================================================================
;; FILE: situational_action_specializers.lisp
;; DOMAIN: Composable Situational Behaviors (Clear Obstacles, Sit->Lie, Egress)
;; =============================================================================
(defpackage :ortho-dsp.affordance-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.perception-core
        :ortho-dsp.perception-xforms
        :ortho-dsp.affordance-core)
  (:export #:xform-situational-affordance-engine
           #:specialize-with-autonomous-obstacle-relocation
           #:specialize-with-furniture-pose-alignment
           #:specialize-with-bidirectional-postural-chaining
           #:specialize-with-assistive-egress-bracing))
(in-package :ortho-dsp.affordance-xforms)

(defmodifier xform-situational-affordance-engine [state &key (intent :lie-supine-on-fixture)]
  (-> state
      (xform-perceptual-slam-base)
      (specialize-with-saccadic-active-gaze :max-saccade-dwell-ms 240.0)
      (specialize-with-dynamic-agent-bifurcation :prediction-horizon-s 2.5)
      (specialize-with-affordance-field-projection)
      (specialize-with-visuo-tactile-assimilation)
      (evaluate-situational-affordances)
      (define-postural-funnel-topology)
      (solve-minimum-action-funnel-chain :goal-affordance intent)))

(defmodifier specialize-with-autonomous-obstacle-relocation
  [situational-state &key (clearance-margin-mm 280.0)]
  (-> situational-state
      (when-funnel-active :bimanual-object-coupled
        (-> situational-state
            (xform-geodesic-wayfinder-base :goal-xyz (:active-obstacle-grasp-datum))
            (drive-appendage-admittance
              :nodes [:glenohumeral :cubital :palmar-carpal-l :palmar-carpal-r]
              :target-attractor (:active-obstacle-handle-se3)
              :latch-grip-force-n 28.0)
            (splice-manipulated-object-adjoint
              :object-slot :active-occluding-quadric
              :transport-mode :auto-slide-or-lift)
            (xform-geodesic-wayfinder-base
              :goal-xyz (:compute-non-occluding-parking-pocket :margin-mm clearance-margin-mm))
            (release-bimanual-coupling
              :verify-object-static-fz true
              :update-registry-pose true)))))

(defmodifier specialize-with-furniture-pose-alignment
  [situational-state &key (preferred-room-gaze-yaw-rad 0.0) (min-legroom-x-mm 420.0)]
  (-> situational-state
      (when-target-affordance-occluded (:sit-on-fixture :reason [:insufficient-legroom :suboptimal-yaw])
        (-> situational-state
            (splice-manipulated-object-adjoint :object-slot :target-sit-quadric)
            (reorient-coupled-quadric
              :target-yaw-rad preferred-room-gaze-yaw-rad
              :extract-from-under-overhang-mm min-legroom-x-mm
              :max-wrist-torque-nm 18.0)
            (release-bimanual-coupling :update-registry-pose true)))))

(defmodifier specialize-with-bidirectional-postural-chaining [situational-state]
  (-> situational-state
      (bind-funnel-transition-executors
        :transitions
          ((:from :bipedal-upright :to :ischial-seated
             :execute (-> (lock-horizon-gaze)
                          (pivot-single-anchor-arc :target-yaw-rad (:target-quadric-anterior-yaw))
                          (xform-blind-posterior-settle)
                          (specialize-with-tactile-arm-sweep)
                          (specialize-with-reactive-capture-step)))

           (:from :ischial-seated :to :dorsal-supine
             :execute (-> (xform-blind-recumbent-flow)
                          (specialize-with-viscoelastic-sink-compensation)
                          (release-impedance-wavefront :terminal-mode :supine-high-z)))

           (:from :dorsal-supine :to :ischial-seated
             :execute (-> (wake-inverter-closed-loop)
                          (xform-balance-supine-to-biped :ascent-ms 850.0)
                          (configure-momentum-observer)))

           (:from :ischial-seated :to :bipedal-upright
             :execute (-> (xform-balance-supine-to-biped :ascent-ms 950.0)
                          (ignite-cyclic-clock nominal-biped-gait :bars 1)))))))

(defmodifier specialize-with-assistive-egress-bracing
  [situational-state &key (max-brace-reach-mm 680.0) (max-brace-load-share 0.25)]
  (-> situational-state
      (when-funnel-transition (:from [:dorsal-supine :ischial-seated] :to :bipedal-upright)
        (let [brace-candidate (query-nearest-affordance 'spatial-object-soa-registry
                                :affordance :recruit-brace-support
                                :max-radius-mm max-brace-reach-mm)]
          (when-modifier brace-candidate
            (splice-auxiliary-manifold
              :patch-id :opportunistic-egress-brace
              :anatomical-frames [:palmar-carpal-l :palmar-carpal-r]
              :surface-normal (slot-surface-normal brace-candidate)
              :mu (slot-estimated-mu brace-candidate)
              :weight-law (:wrench-proportional :max-share max-brace-load-share)))))))
