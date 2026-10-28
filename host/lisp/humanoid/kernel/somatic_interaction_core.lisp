;; -----------------------------------------------------------------------------
;; MODULE 1.13: somatic_interaction_core
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.somatic-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:modulate-kinematic-reachability
           #:drive-tangential-coulomb-slide
           #:arpeggiate-elevation-energy
           #:couple-non-prehensile-shove))
(in-package :ortho-dsp.somatic-core)

(defprimitive modulate-kinematic-reachability
  [state &key (max-leg-reach-mm 890.0) (equinus-transition-rho [0.86 0.98]) (dangle-detach-rho 0.99)]
  (-> state
      (bind-reachability-foliation
        :chain [:coxofemoral :tibiofemoral :talocrural :calcaneus]
        :nominal-max-length-mm max-leg-reach-mm
        :equinus-morph-window
          (:rho-bounds equinus-transition-rho
           :shift-patch (:from :plantar-bilateral :to :metatarsal-equinus-bilateral)
           :talocrural-plantarflex-slew-rad [0.0 -> 0.58]
           :achilles-series-elastic-q 1.65)
        :unloading-horizon
          (:detach-rho dangle-detach-rho
           :force-weight-zero [:plantar-bilateral :metatarsal-equinus-bilateral]
           :dangling-limb-damping-ratio 0.65))))

(defprimitive drive-tangential-coulomb-slide
  [state &key (sliding-patches [:posterior-femoral-bilateral :ischial-bilateral])
              (slide-velocity-xyz-mps [-0.18 0.0 0.0])
              (max-normal-fz-during-slide 0.55)
              (stop-on-ischial-centering true)]
  (-> state
      (configure-sliding-manifold-regime
        :patches sliding-patches
        :tangent-velocity-attractor slide-velocity-xyz-mps
        :normal-load-governor
          (:cap-fz-ratio max-normal-fz-during-slide
           :offload-excess-to [:metatarsal-equinus-bilateral :ulnar-carpal-shove-bilateral])
        :friction-boundary-law
          (:mode :kinetic-coulomb-surfer :anti-stick-slip-dither-hz 65.0)
        :termination-condition
          (if stop-on-ischial-centering
              (:cop-centered-in-patch :ischial-bilateral :margin-mm 25.0)
              :external-admittance-equilibrium))))

(defprimitive arpeggiate-elevation-energy
  [state &key (step-dz-mm :from-sub-tarsal-sdf) (step-dx-mm 280.0) (nosing-clearance-mm 34.0)]
  (-> state
      (drive-hamiltonian-step-cycle
        :elevation-delta-mm step-dz-mm
        :tread-depth-mm step-dx-mm
        :nosing-radial-guard-mm nosing-clearance-mm
        :ascent-branch
          (:strike-patch :plantar-bilateral
           :lead-thoracic-pitch-rad +0.22
           :concentric-work-pulse-j (:scale-by-mg-dz 1.05)
           :trailing-ankle-pushoff-gain 1.45)
        :descent-branch
          (:strike-patch :metatarsal-equinus-bilateral
           :swing-ankle-plantarflex-rad +0.36
           :pelvic-coronal-drop-rad 0.08
           :stance-knee-eccentric-damping 0.94
           :prevent-calcaneal-riser-scrape true))))

(defprimitive couple-non-prehensile-shove
  [state &key (contact-patch :ulnar-carpal-shove-bilateral)
              (coupling-topology :ulnar-backrest-hook)
              (target-object-slot :active-occluding-quadric)
              (max-shove-shear-n 115.0)]
  (-> state
      (splice-somatic-object-interaction
        :patch contact-patch
        :object-slot target-object-slot
        :topology coupling-topology
        :unilateral-contact-maintenance
          (:min-normal-bias-n 18.0 :max-tangential-force-n max-shove-shear-n :allow-finger-closure false)
        :centroidal-shove-compensation
          (:lean-poinsot-into-load true :shunt-high-friction-to-upward-bevel-lift 25.0))))
