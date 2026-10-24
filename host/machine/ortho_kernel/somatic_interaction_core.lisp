;; =============================================================================
;; FILE: somatic_interaction_core.lisp
;; DOMAIN: Reachability Foliation, Coulomb Slide Homotopy & Non-Prehensile Shove
;; =============================================================================
(defpackage :ortho-dsp.somatic-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:somatic-contact-patch-extensions
           #:modulate-kinematic-reachability
           #:drive-tangential-coulomb-slide
           #:arpeggiate-elevation-energy
           #:couple-non-prehensile-shove))
(in-package :ortho-dsp.somatic-core)

;; -----------------------------------------------------------------------------
;; 1. EXTENDED SOMATIC CONTACT PATCHES (EQUINUS FOREFOOT, THIGH BEVEL, ULNAR HOOK)
;; No finger/dexterous grasp frames; strictly corporal & flat-pad surfaces.
;; -----------------------------------------------------------------------------
(defcontact-patch somatic-contact-patch-extensions
  ;; Forefoot/Metatarsal Equinus Patch (active when heel cannot reach floor)
  (:patch :metatarsal-equinus-bilateral
    :frames          [:metatarsal-head-l :metatarsal-head-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-22.0 45.0] :y [-42.0 42.0])
    :default-mu      0.80
    :unilateral-fz   true)

  ;; Posterior Femoral / Proximal Hamstring Patch (perching on high chair/bed edges)
  (:patch :posterior-femoral-bilateral
    :frames          [:femoral-diaphysis-dorsal-l :femoral-diaphysis-dorsal-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-110.0 110.0] :y [-135.0 135.0])
    :default-mu      0.45                       ; Lower mu permits controlled sliding on upholstery/wood
    :unilateral-fz   true)

  ;; Non-Prehensile Forearm / Carpal Heel Push-Hook Patch (moving chairs without fingers)
  (:patch :ulnar-carpal-shove-bilateral
    :frames          [:ulna-distal-l :ulna-distal-r :carpal-heel-l :carpal-heel-r]
    :local-normal    [1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-65.0 65.0] :y [-35.0 35.0])
    :default-mu      0.60
    :unilateral-fz   true))                     ; Pure unilateral push or geometric backrest hook!

;; -----------------------------------------------------------------------------
;; 2. KINEMATIC REACHABILITY FOLIATION (HANDLES HIGH CHAIRS & DEEP DROP STEPS)
;; Monitors limb extension ratio rho = ||p_hip - p_contact|| / L_max.
;; Automatically morphs Plantigrade -> Equinus Toe-Tip -> Dangling Null-Space
;; as seat height exceeds leg length, preventing knee hyperextension singularities.
;; -----------------------------------------------------------------------------
(defprimitive modulate-kinematic-reachability
  [state &key (max-leg-reach-mm 890.0)
              (equinus-transition-rho [0.86 0.98])
              (dangle-detach-rho 0.99)]
  (-> state
      (bind-reachability-foliation
        :chain [:coxofemoral :tibiofemoral :talocrural :calcaneus]
        :nominal-max-length-mm max-leg-reach-mm
        ;; Band 1 (rho < 0.86): Full plantigrade heel+toe polygon
        ;; Band 2 (0.86 <= rho < 0.99): Pitch ankle into plantarflexion, shift CWC to metatarsals
        :equinus-morph-window
          (:rho-bounds equinus-transition-rho
           :shift-patch (:from :plantar-bilateral :to :metatarsal-equinus-bilateral)
           :talocrural-plantarflex-slew-rad [0.0 -> 0.58]
           :achilles-series-elastic-q 1.65)
        ;; Band 3 (rho >= 0.99): Ground is out of reach; smoothly zero plantar weight w -> 0
        ;; and release legs into passive pendulum Null-Space (dangling feet on high barstool)
        :unloading-horizon
          (:detach-rho dangle-detach-rho
           :force-weight-zero [:plantar-bilateral :metatarsal-equinus-bilateral]
           :dangling-limb-damping-ratio 0.65))))

;; -----------------------------------------------------------------------------
;; 3. TANGENTIAL COULOMB SLIDE HOMOTOPY (PERCH, SCOOT & BEVEL SLIDING)
;; Overrides the rigid sticking constraint (v_contact = 0) on selected patches,
;; allowing the thighs/pelvis to slide smoothly across a high chair bevel or bed
;; surface by regulating shear force right at the kinetic friction boundary.
;; -----------------------------------------------------------------------------
(defprimitive drive-tangential-coulomb-slide
  [state &key (sliding-patches [:posterior-femoral-bilateral :ischial-bilateral])
              (slide-velocity-xyz-mps [-0.18 0.0 0.0]) ; Negative X = slide backward onto seat
              (max-normal-fz-during-slide 0.55)        ; Keep 45% weight on toes/arms to avoid friction lock
              (stop-on-ischial-centering true)]
  (-> state
      (configure-sliding-manifold-regime
        :patches sliding-patches
        :tangent-velocity-attractor slide-velocity-xyz-mps
        :normal-load-governor
          (:cap-fz-ratio max-normal-fz-during-slide
           :offload-excess-to [:metatarsal-equinus-bilateral :ulnar-carpal-shove-bilateral])
        :friction-boundary-law
          (:mode :kinetic-coulomb-surfer        ; Maintain ||f_xy|| = mu_k * F_z along slide vector
           :anti-stick-slip-dither-hz 65.0)     ; Prevent upholstery stick-slip judder
        :termination-condition
          (if stop-on-ischial-centering
              (:cop-centered-in-patch :ischial-bilateral :margin-mm 25.0)
              :external-admittance-equilibrium))))

;; -----------------------------------------------------------------------------
;; 4. BIDIRECTIONAL PORT-HAMILTONIAN ELEVATION ARPEGGIATOR (STAIRS UP & DOWN)
;; Unified energy-routing engine for discrete step risers (+dZ ascent, -dZ descent).
;; -----------------------------------------------------------------------------
(defprimitive arpeggiate-elevation-energy
  [state &key (step-dz-mm :from-sub-tarsal-sdf) ; Positive = Ascent, Negative = Descent
              (step-dx-mm 280.0)
              (nosing-clearance-mm 34.0)]
  (-> state
      (drive-hamiltonian-step-cycle
        :elevation-delta-mm step-dz-mm
        :tread-depth-mm step-dx-mm
        :nosing-radial-guard-mm nosing-clearance-mm

        ;; A. ASCENT LAW (dZ > 0): Concentric potential energy injection (+m*g*dZ)
        :ascent-branch
          (:strike-patch :plantar-bilateral
           :lead-thoracic-pitch-rad +0.22       ; Project CoM into upper step cone early
           :concentric-work-pulse-j (:scale-by-mg-dz 1.05)
           :trailing-ankle-pushoff-gain 1.45)

        ;; B. DESCENT LAW (dZ < 0): Controlled eccentric lowering & sub-plane foot reach
        :descent-branch
          (:strike-patch :metatarsal-equinus-bilateral ; Land forefoot-first to absorb impact!
           :swing-ankle-plantarflex-rad +0.36          ; Point toes down to reach lower tread
           :pelvic-coronal-drop-rad 0.08               ; Controlled Trendelenburg hip drop extends reach
           :stance-knee-eccentric-damping 0.94         ; Smoothly bleed -m*g*|dZ| without knee lock
           :prevent-calcaneal-riser-scrape true))))

;; -----------------------------------------------------------------------------
;; 5. NON-PREHENSILE SOMATIC SHOVE & HOOK (BODY-DRIVEN FURNITURE RELOCATION)
;; Moves chairs or doors using forearms, carpal heels, or hips—zero finger grasp.
;; -----------------------------------------------------------------------------
(defprimitive couple-non-prehensile-shove
  [state &key (contact-patch :ulnar-carpal-shove-bilateral)
              (coupling-topology :unilateral-push-or-hook) ; :flat-push | :ulnar-backrest-hook
              (target-object-slot :active-occluding-quadric)
              (max-shove-shear-n 115.0)]
  (-> state
      (splice-somatic-object-interaction
        :patch contact-patch
        :object-slot target-object-slot
        :topology coupling-topology
        ;; Maintain positive normal contact pressure (> 15 N) so flat palms/forearms
        ;; never slip off the chair frame while walking/pushing
        :unilateral-contact-maintenance
          (:min-normal-Bias-n 18.0
           :max-tangential-force-n max-shove-shear-n
           :allow-finger-closure false)         ; Strictly non-prehensile!
        ;; Route object sliding resistance directly into torso lean & Poinsot axis
        :centroidal-shove-compensation
          (:lean-poinsot-into-load true
           :shunt-high-friction-to-upward-bevel-lift 25.0)))) ; Scoop upward 25mm with forearms if chair legs snag
