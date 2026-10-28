;; -----------------------------------------------------------------------------
;; MODULE 2.6: situational_action_specializers
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.affordance-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
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
      (when-funnel-active :somatic-object-coupled
        (-> situational-state
            (xform-geodesic-wayfinder-base :goal-xyz (:active-obstacle-grasp-datum))
            (xform-somatic-furniture-shove
              :shove-technique :ulnar-backrest-hook
              :clearing-offset-xy-mm (:compute-non-occluding-parking-pocket :margin-mm clearance-margin-mm))
            (release-bimanual-coupling :verify-object-static-fz true :update-registry-pose true)))))

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
                          (xform-somatic-sit-ingress)
                          (specialize-for-overheight-seat :direction :ingress :slide-speed-mps 0.16)
                          (specialize-with-carpal-seat-boost :max-arm-push-share 0.20)
                          (specialize-with-tactile-arm-sweep)))

           (:from :ischial-seated :to :dorsal-supine
             :execute (-> (xform-somatic-bed-ingress)))

           (:from :dorsal-supine :to :ischial-seated
             :execute (-> (xform-somatic-bed-egress)))

           (:from :ischial-seated :to :bipedal-upright
             :execute (-> (specialize-for-overheight-seat :direction :egress :slide-speed-mps 0.15)
                          (xform-somatic-sit-egress)))))))

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
              :anatomical-frames [:carpal-heel-l :carpal-heel-r :ulna-distal-l :ulna-distal-r]
              :surface-normal (slot-surface-normal brace-candidate)
              :mu (slot-estimated-mu brace-candidate)
              :weight-law (:wrench-proportional :max-share max-brace-load-share)))))))
