;; -----------------------------------------------------------------------------
;; MODULE 2.1: foliated_riemannian_nav (COUPLED CAPSULE-SCHEMA WAYFINDER)
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.nav-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.body-schema)
  (:export #:configure-foliated-sdf-bands
           #:synthesize-riemannian-metric
           #:inject-gyroscopic-obstacle-curl
           #:snap-foothold-to-plantar-gradient
           #:guard-foothold-with-crural-capsules
           #:slice-narrow-aperture-axial))
(in-package :ortho-dsp.nav-core)

(defprimitive configure-foliated-sdf-bands
  [state &key (sub-band-z-mm  [0.0   260.0])
              (mid-band-z-mm  [260.0 980.0])
              (high-band-z-mm [980.0 1920.0])
              (update-rate-hz 120.0)]
  (-> state
      (bind-multiband-sdf-manifolds
        :refresh-rate-hz update-rate-hz
        :coupled-capsule-schema 'anatomical-capsule-schema
        :slabs
          ((:sub-tarsal-slab    :z-window-mm sub-band-z-mm
                                :target-nodes [:calcaneus :tarsal-sagittal :tarsal-coronal]
                                :capsule-group [:foot-shell-l :foot-shell-r :shank-crural-l :shank-crural-r])
           (:mid-pelvic-slab    :z-window-mm mid-band-z-mm
                                :target-nodes [:sacroiliac-z :iliac-coronal :coxofemoral]
                                :capsule-group [:thigh-femoral-l :thigh-femoral-r :pelvic-girdle :lumbar-column])
           (:high-thoracic-slab :z-window-mm high-band-z-mm
                                :target-nodes [:l4-l5-lumbar :c7-t12-thoracic :glenohumeral :c1-c2-atlas]
                                :capsule-group [:thoracic-ribcage :cranial-vault
                                                :upper-arm-humeral-l :upper-arm-humeral-r
                                                :forearm-ulnar-l :forearm-ulnar-r])))))

(defprimitive synthesize-riemannian-metric
  [state &key (goal-attractor-xyz [10.0 0.0 0.0])
              (roughness-cost-weight 1.4)
              (upslope-metabolic-weight 2.2)
              (cross-slope-shear-weight 3.8)
              (social-lane-bias 0.35)
              (ecbf-pullback-stiffness-eta 18.5)
              (gauge-lookahead-s 0.60)]
  (-> state
      (solve-eikonal-geodesic-flow
        :goal-point goal-attractor-xyz
        :metric-tensor-law
          (:base-isotropic-cost 1.0
           :sub-band-roughness-gain roughness-cost-weight
           :longitudinal-grade-gain upslope-metabolic-weight
           :coronal-camber-gain     cross-slope-shear-weight
           :trail-thalweg-attractor social-lane-bias)
        ;; BACKWARD COUPLING (1 kHz ECBF -> Geodesic Streamline & Metric via Sherman-Morrison):
        :ecbf-stress-pullback
          (:source-schema 'anatomical-capsule-schema
           :rank1-sherman-morrison true
           :coupling-gain-eta ecbf-pullback-stiffness-eta
           :nullspace-exhaustion-threshold 0.85
           :persistent-blockage-bake-ms 140.0
           :voxel-obstruction-multiplier 50.0)
        ;; FORWARD COUPLING (120 Hz Eikonal -> 1 kHz Capsule Gauge Pre-Shaping):
        :minkowski-gauge-projection
          (:lookahead-horizon-s gauge-lookahead-s
           :preload-nullspace-posture-when-between [:w-min-contorted :w-nominal]
           :mark-impassable-below :w-min-contorted)
        :curvature-clock-modulation
          (:centripetal-banking-gain 0.85
           :high-curvature-bpm-scale [0.78 1.00]))))

(defprimitive inject-gyroscopic-obstacle-curl
  [state &key (detection-horizon-m 4.5)
              (vortex-circulation-gain 1.65)
              (pass-behind-moving-agents true)]
  (-> state
      (apply-lorentz-steering-field
        :slab-source :mid-pelvic-slab
        :horizon-m detection-horizon-m
        :circulation-strength vortex-circulation-gain
        :chirality-law (if pass-behind-moving-agents
                           :trailing-wake-dominant
                           :minimum-deflection-angle)
        :preserve-kinetic-energy true)))

(defprimitive snap-foothold-to-plantar-gradient
  [state &key (search-radius-mm 140.0)
              (edge-repulsion-gain 2.4)
              (step-over-margin-z-mm 35.0)
              (crural-capsule-repulsion-weight 3.2)
              (min-shin-clearance-mm 28.0)]
  (-> state
      (modulate-swing-landing-attractor
        :slab-source :sub-tarsal-slab
        :active-phase-window [0.05 -> 0.38]
        :max-xy-adjustment-mm search-radius-mm
        :surface-cost-functional
          (:local-convexity-penalty edge-repulsion-gain
           :normal-deviation-penalty 1.8
           :estimated-mu-reward      2.0)
        :crural-capsule-fk-guard
          (:monitored-capsules [:foot-shell-l :foot-shell-r
                                :shank-crural-l :shank-crural-r
                                :knee-patellar-l :knee-patellar-r
                                :thigh-femoral-l :thigh-femoral-r]
           :pullback-gain crural-capsule-repulsion-weight
           :min-shin-clearance-m (* 0.001 min-shin-clearance-mm))
        :dynamic-bridge-clearance
          (:node :tarsal-sagittal
           :clearance-law (:max-intervening-sdf-z :add step-over-margin-z-mm)
           :contour :asymmetric-parabolic))))

(defprimitive guard-foothold-with-crural-capsules
  [state &key (checked-limb-capsules [:foot-shell-l :foot-shell-r :shank-crural-l :shank-crural-r
                                      :knee-patellar-l :knee-patellar-r :thigh-femoral-l :thigh-femoral-r])
              (crural-repulsion-weight 3.2)
              (min-shin-clearance-mm 28.0)]
  (-> state
      (snap-foothold-to-plantar-gradient
        :crural-capsule-repulsion-weight crural-repulsion-weight
        :min-shin-clearance-mm min-shin-clearance-mm)))

(defprimitive slice-narrow-aperture-axial
  [state &key (bi-deltoid-width-mm 480.0)
              (thoracic-depth-mm 235.0)
              (headroom-margin-mm 45.0)]
  (-> state
      (drive-nullspace-gauge-posture
        :slab-source :high-thoracic-slab
        :capsule-gradient-source 'anatomical-capsule-schema
        :aperture-yaw-slicer
          (:node :c7-t12-thoracic
           :shoulder-span-mm bi-deltoid-width-mm
           :chest-thickness-mm thoracic-depth-mm
           :max-axial-twist-rad 1.309
           :arm-tuck-adduction-rad -0.18
           :counter-rotate-atlas-gaze true)
        :overhead-canopy-ducker
          (:nodes [:l4-l5-lumbar :c7-t12-thoracic :c1-c2-atlas]
           :clearance-margin-mm headroom-margin-mm
           :max-duck-depth-mm -380.0
           :sagittal-com-counter-comp true))))
