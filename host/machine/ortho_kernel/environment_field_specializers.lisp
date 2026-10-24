;; =============================================================================
;; FILE: environment_field_specializers.lisp
;; DOMAIN: Composable Wayfinding Specializers (Room, City, Rural, Forest)
;; =============================================================================
(defpackage :ortho-dsp.nav-envs
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.nav-core)
  (:export #:xform-geodesic-wayfinder-base
           #:specialize-env-cluttered-interior
           #:specialize-env-urban-streetscape
           #:specialize-env-rural-countryside
           #:specialize-env-dense-forest
           #:specialize-with-tactile-foliage-parting))
(in-package :ortho-dsp.nav-envs)

(defmodifier xform-geodesic-wayfinder-base [gait &key (goal-xyz [10.0 0.0 0.0])]
  (-> gait
      (configure-foliated-sdf-bands)
      (synthesize-riemannian-metric :goal-attractor-xyz goal-xyz)
      (inject-gyroscopic-obstacle-curl :detection-horizon-m 4.0)
      (snap-foothold-to-plantar-gradient :search-radius-mm 120.0)
      (slice-narrow-aperture-axial)))

(defmodifier specialize-env-cluttered-interior [nav-state]
  (-> nav-state
      (modulate-node :tarsal-coronal
        :track-width-mm (:sdf-adaptive-clamp :min-mm 52.0 :nominal-mm 103.0)
        :swing-circumduction-mm (:sdf-adaptive-bow :clearance-guard-mm 7.5))
      (snap-foothold-to-plantar-gradient
        :search-radius-mm 160.0
        :edge-repulsion-gain 4.2
        :step-over-margin-z-mm 42.0)
      (slice-narrow-aperture-axial
        :bi-deltoid-width-mm 480.0
        :headroom-margin-mm 30.0)
      (splice-auxiliary-manifold
        :patch-id :furniture-graze-buffer
        :anatomical-frames [:greater-trochanter-l :greater-trochanter-r :palmar-carpal-l :palmar-carpal-r]
        :surface-normal [0.0 1.0 0.0]
        :mu 0.30
        :weight-law (:envelope [0.0 -> 0.08 -> 0.0]))))

(defmodifier specialize-env-urban-streetscape [nav-state &key (crowd-density :moderate)]
  (-> nav-state
      (synthesize-riemannian-metric
        :roughness-cost-weight 1.2
        :cross-slope-shear-weight 2.5
        :social-lane-bias 0.75)
      (inject-gyroscopic-obstacle-curl
        :detection-horizon-m (case crowd-density (:sparse 6.0) (:moderate 4.5) (:dense 2.8))
        :vortex-circulation-gain 2.10
        :pass-behind-moving-agents true)
      (sample-and-hold-terrain :target :tarsal-sagittal
        :curb-step-quantizer true
        :elevation-deadband-mm 15.0
        :edge-margin-z-mm 35.0)))

(defmodifier specialize-env-rural-countryside [nav-state]
  (-> nav-state
      (synthesize-riemannian-metric
        :roughness-cost-weight 2.0
        :upslope-metabolic-weight 3.2
        :cross-slope-shear-weight 4.8
        :social-lane-bias 0.40)
      (adapt-to-surface-normal
        :nodes [:talocrural :iliac-coronal :l4-l5-lumbar]
        :pitch-grade-feedforward-gain 0.85
        :coronal-camber-feedforward-gain 0.90)
      (snap-foothold-to-plantar-gradient
        :search-radius-mm 280.0
        :edge-repulsion-gain 3.0
        :step-over-margin-z-mm 65.0)))

(defmodifier specialize-env-dense-forest [nav-state]
  (-> nav-state
      (synthesize-riemannian-metric
        :roughness-cost-weight 3.5
        :upslope-metabolic-weight 2.8
        :cross-slope-shear-weight 4.2
        :social-lane-bias 0.60)
      (modulate-node :tarsal-sagittal :swing-clearance-z-mm (:add 55.0))
      (snap-foothold-to-plantar-gradient
        :search-radius-mm 220.0
        :edge-repulsion-gain 4.8
        :step-over-margin-z-mm 85.0)
      (slice-narrow-aperture-axial :bi-deltoid-width-mm 480.0 :headroom-margin-mm 70.0)
      (configure-momentum-observer :observer-bandwidth-hz 45.0)
      (bind-wrench-driven-homotopy
        :active-manifolds [:plantar-bilateral]
        :slew-rate-up-per-s 8.0
        :slew-rate-down-per-s 50.0)))

(defmodifier specialize-with-tactile-foliage-parting
  [nav-state &key (yield-force-threshold-n 18.0)]
  (-> nav-state
      (configure-compliant-obstacle-filter
        :slab-target [:mid-pelvic-slab :high-thoracic-slab]
        :tactile-probe-nodes [:ulna-l :ulna-r :palmar-carpal-l :palmar-carpal-r]
        :pass-through-force-ceiling-n yield-force-threshold-n
        :rigid-deflection-fallback :inject-gyroscopic-curl)))
