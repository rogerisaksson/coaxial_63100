;; =============================================================================
;; FILE: semantic_scene_specializers.lisp
;; DOMAIN: Composable Scene-Graph Builders & Cross-Modal Perception Specializers
;; =============================================================================
(defpackage :ortho-dsp.perception-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.nav-core
        :ortho-dsp.perception-core)
  (:export #:xform-perceptual-slam-base
           #:specialize-with-visuo-tactile-assimilation
           #:specialize-with-saccadic-active-gaze
           #:specialize-with-dynamic-agent-bifurcation
           #:specialize-with-affordance-field-projection))
(in-package :ortho-dsp.perception-xforms)

(defmodifier xform-perceptual-slam-base [state]
  (-> state
      (configure-phase-gated-stereo :baseline-b-mm 120.0 :frame-rate-hz 90.0)
      (propagate-invariant-ekf-se3)
      (assimilate-semantic-quadrics)
      (maintain-spatial-memory-decay)))

(defmodifier specialize-with-visuo-tactile-assimilation [perceptual-state]
  (-> perceptual-state
      (bind-tactile-to-registry-writeback
        :registry 'spatial-object-soa-registry
        :wrench-source :generalized-momentum-observer
        :trigger-fz-ratio 0.08
        :kalman-measurement-override
          (:contact-position-sigma-mm 0.40
           :surface-normal-sigma-rad  0.008
           :write-measured-stiffness  true
           :write-measured-eigenfreq  true
           :write-measured-mu-bound   true
           :mark-tactile-verified     true))))

(defmodifier specialize-with-saccadic-active-gaze
  [perceptual-state &key (max-saccade-dwell-ms 280.0) (entropy-trigger-thresh 18.0)]
  (-> perceptual-state
      (drive-information-gain-gaze
        :node :c1-c2-atlas
        :registry 'spatial-object-soa-registry
        :objective :minimize-expected-trace-covariance
        :trigger-entropy-threshold entropy-trigger-thresh
        :lookahead-horizon-steps 2.5
        :saccade-dwell-limit-ms max-saccade-dwell-ms
        :inhibit-during-terminal-pivot true
        :default-return-attractor :world-horizon)))

(defmodifier specialize-with-dynamic-agent-bifurcation
  [perceptual-state &key (motion-chi2-thresh 16.27) (prediction-horizon-s 2.5)]
  (-> perceptual-state
      (bifurcate-moving-objects
        :registry 'spatial-object-soa-registry
        :static-violation-chi2 motion-chi2-thresh
        :kinematic-tracker :imm-constant-turn-velocity
        :exclude-from-ego-ekf true
        :feed-wayfinder-vortex
          (:target-slab :mid-pelvic-slab
           :horizon-s prediction-horizon-s
           :wake-passing-bias :pass-behind-agent))))

(defmodifier specialize-with-affordance-field-projection [perceptual-state]
  (-> perceptual-state
      (project-registry-to-control-buses
        :registry 'spatial-object-soa-registry
        :min-salience-log-odds +1.80
        :routing-rules
          ((:when-affordance :obstacle
             :inject-into :foliated-sdf-slabs
             :inflate-by-uncertainty-sigma 2.0)
           (:when-affordance :step-on
             :inject-into :sub-tarsal-slab
             :quantize-stair-risers true)
           (:when-affordance [:sit-manifold :supine-manifold :brace-rail]
             :inject-into :candidate-docking-manifolds
             :preload-learned-impedance :if-tactile-verified)))))
