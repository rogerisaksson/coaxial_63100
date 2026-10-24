;; =============================================================================
;; FILE: lie_group_slam_core.lisp
;; DOMAIN: Right-Invariant SE_k(3) SLAM, Phase-Gated Stereo & Dual-Quadric Memory
;; =============================================================================
(defpackage :ortho-dsp.perception-core
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core)
  (:export #:spatial-object-soa-registry
           #:configure-phase-gated-stereo
           #:propagate-invariant-ekf-se3
           #:assimilate-semantic-quadrics
           #:maintain-spatial-memory-decay))
(in-package :ortho-dsp.perception-core)

(defstate spatial-object-soa-registry
  (:memory-layout
    :max-tracked-objects 256
    :align-bytes 64
    :eviction-policy :mahalanobis-least-salient)
  (:slot-schema
    :object-id          :uint32
    :semantic-class     :keyword
    :affordance-mask    :bitfield
    :pose-se3           :mat4x4
    :dual-quadric-axes  :vec3-mm
    :sqrt-info-matrix   :upper-tri-6x6
    :physical-priors    (:stiffness-npmm 150.0
                         :eigenfreq-hz   2.50
                         :friction-mu    0.65
                         :estimated-mass-kg 6.0
                         :tactile-verified-p false)
    :kinematic-velocity :vec6-se3-twist
    :salience-log-odds  :float32))

(defprimitive configure-phase-gated-stereo
  [state &key (baseline-b-mm 120.0)
              (focal-length-px 840.0)
              (frame-rate-hz 90.0)
              (strike-jerk-blanking-gain 6.5)]
  (-> state
      (bind-stereo-vision-frontend
        :rig-frame :c1-c2-atlas-optical
        :stereo-baseline-mm baseline-b-mm
        :focal-px focal-length-px
        :capture-rate-hz frame-rate-hz
        :adaptive-noise-covariance
          (:base-disparity-sigma-px 0.35
           :calcaneal-strike-blanking-gain strike-jerk-blanking-gain
           :motion-blur-velocity-scale 1.80
           :prefer-phase-window [0.12 -> 0.45])
        :feature-pipeline [:semi-global-matching-disparity
                           :oriented-fast-keypoint-anchors
                           :semantic-affordance-classifier])))

(defprimitive propagate-invariant-ekf-se3
  [state &key (imu-gyro-sigma 0.0015)
              (imu-accel-sigma 0.012)
              (contact-slip-rejection-thresh 0.08)]
  (-> state
      (run-right-invariant-ekf
        :lie-group :se-k-3
        :prediction-rate-hz 1000.0
        :kinematic-measurement-law
          (:contact-weight-source :active-manifold-weights
           :shear-slip-gate-mu contact-slip-rejection-thresh
           :encoder-fk-chain [:calcaneus-l :calcaneus-r :ischial-bilateral])
        :inertial-noise (:gyro-radps-sqrt-hz imu-gyro-sigma
                         :accel-mps2-sqrt-hz imu-accel-sigma)
        :visual-innovation-law
          (:error-metric :right-invariant-se3-log
           :robust-kernel :huber-cauchy-m-estimator
           :mahalanobis-gate-chi2 11.34))))

(defprimitive assimilate-semantic-quadrics
  [state &key (association-gate-m2 5.99)
              (hit-log-odds-boost +0.85)
              (miss-frustum-decay -0.22)]
  (-> state
      (update-dual-quadric-registry
        :registry 'spatial-object-soa-registry
        :data-association :wasserstein-gaussian-metric
        :gate-threshold association-gate-m2
        :filter-topology :joseph-stabilized-sr-if
        :occupancy-bayes-law
          (:on-frustum-match hit-log-odds-boost
           :on-frustum-occlusion 0.0
           :on-frustum-empty-miss miss-frustum-decay
           :confirm-salience-thresh +2.20
           :prune-salience-floor -3.50))))

(defprimitive maintain-spatial-memory-decay
  [state &key (movable-diffusion-mm2ps 0.45) (structural-lock-classes [:stair :door :wall :bed])]
  (-> state
      (diffuse-unobserved-object-covariance
        :registry 'spatial-object-soa-registry
        :static-locked-classes structural-lock-classes
        :semi-static-drift-rate movable-diffusion-mm2ps
        :transform-with-ego-loop-closure true)))
