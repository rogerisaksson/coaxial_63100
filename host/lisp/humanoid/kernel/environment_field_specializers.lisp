;; -----------------------------------------------------------------------------
;; MODULE 2.2: environment_field_specializers
;; SYNOPSIS: Wayfinding by setting: cluttered interiors, a payload's clearance, a furniture shove.
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.nav-envs
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.body-schema
        :ortho-dsp.somatic-core
        :ortho-dsp.nav-core)
  (:export #:xform-geodesic-wayfinder-base
           #:specialize-env-cluttered-interior
           #:specialize-with-encumbered-payload-clearance
           #:xform-somatic-furniture-shove))
(in-package :ortho-dsp.nav-envs)

(defmodifier xform-geodesic-wayfinder-base [gait &key (goal-xyz [10.0 0.0 0.0])]
  (-> gait
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)
      (specialize-with-nullspace-limb-retraction :retraction-influence-radius-mm 280.0)
      (specialize-with-blindspot-memory-persistence :max-memory-horizon-s 45.0)
      (specialize-with-tactile-surprise-reflex :unexpected-force-thresh-n 9.5)
      (configure-foliated-sdf-bands)
      (synthesize-riemannian-metric
        :goal-attractor-xyz goal-xyz
        :ecbf-pullback-stiffness-eta 18.5
        :gauge-lookahead-s 0.60)
      (inject-gyroscopic-obstacle-curl :detection-horizon-m 4.0)
      (snap-foothold-to-plantar-gradient
        :search-radius-mm 120.0
        :crural-capsule-repulsion-weight 3.2
        :min-shin-clearance-mm 28.0)
      (slice-narrow-aperture-axial)))

(defmodifier specialize-env-cluttered-interior [nav-state]
  (-> nav-state
      (modulate-node :tarsal-coronal
        :track-width-mm (:sdf-adaptive-clamp :min-mm 52.0 :nominal-mm 103.0)
        :swing-circumduction-mm (:sdf-adaptive-bow :clearance-guard-mm 7.5))
      (snap-foothold-to-plantar-gradient
        :search-radius-mm 160.0
        :edge-repulsion-gain 4.2
        :step-over-margin-z-mm 42.0
        :crural-capsule-repulsion-weight 4.0
        :min-shin-clearance-mm 32.0)
      (slice-narrow-aperture-axial
        :bi-deltoid-width-mm 480.0
        :headroom-margin-mm 30.0)
      (splice-auxiliary-manifold
        :patch-id :furniture-graze-buffer
        :anatomical-frames [:greater-trochanter-l :greater-trochanter-r :palmar-carpal-l :palmar-carpal-r]
        :surface-normal [0.0 1.0 0.0]
        :mu 0.30
        :weight-law (:envelope [0.0 -> 0.08 -> 0.0]))))

(defmodifier specialize-with-encumbered-payload-clearance
  [nav-state &key (corridor-extra-margin-mm 85.0)]
  (-> nav-state
      (when-modifier (:payload-coupled-p)
        (-> nav-state
            (splice-carried-payload-envelope
              :payload-slot :active-occluding-quadric
              :safety-buffer-mm 30.0)
            (modulate-wayfinder-aperture-width
              :coupled-bounding-span :robot-plus-payload-minkowski-hull
              :extra-wall-clearance-m (* 0.001 corridor-extra-margin-mm))))))

(defmodifier xform-somatic-furniture-shove
  [state &key (shove-technique :ulnar-backrest-hook)
              (clearing-offset-xy-mm [0.0 450.0])]
  (let [active-patch (case shove-technique
                       (:trochanter-hip-bump :trochanter-gluteal-lateral)
                       (otherwise            :ulnar-carpal-shove-bilateral))]
    (-> state
        (configure-momentum-observer :observer-bandwidth-hz 40.0)
        (couple-non-prehensile-shove
          :contact-patch active-patch
          :coupling-topology shove-technique
          :max-shove-shear-n 115.0)
        (specialize-with-encumbered-payload-clearance :corridor-extra-margin-mm 85.0)
        (xform-geodesic-wayfinder-base :goal-xyz clearing-offset-xy-mm))))
