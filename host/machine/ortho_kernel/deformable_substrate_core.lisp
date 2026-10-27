;; =============================================================================
;; FILE: deformable_substrate_core.lisp
;; DOMAIN: Rheological Work-Loop ID, Bekker Snow Terramechanics, Winkler Trough
;;         Compensation & Symplectic Trampoline Membrane Phase-Locking
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, floor_groundwork_core.lisp
;; =============================================================================
(defpackage :ortho-dsp.deformable-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.ground-core)
  (:export #:rheological-substrate-soa-state
           #:identify-rheological-work-loop
           #:decouple-sinkage-from-inekf-odometry
           #:regulate-mohr-coulomb-patch-pressure
           #:execute-two-stage-snow-consolidation
           #:extract-limb-from-posthole-cavity
           #:lock-symplectic-membrane-resonator))
(in-package :ortho-dsp.deformable-core)

;; -----------------------------------------------------------------------------
;; 1. RHEOLOGICAL SUBSTRATE SOA STATE (PER-MANIFOLD CONSTITUTIVE TRACKER)
;; Stores live sinkage depth z_sink, restitution ratio eta_rest, Bekker crust
;; yield threshold, and trampoline membrane wave phase theta_mem at 1 kHz.
;; -----------------------------------------------------------------------------
(defstate rheological-substrate-soa-state
  (:memory-layout
    :num-monitored-patches 8
    :align-bytes 64)
  (:slot-schema
    :sinkage-depth-m          :float32         ; Irreversible + elastic depression below initial touch Z_0
    :compression-work-j       :float32         ; Integral of Fz * dz during downward stroke (dz < 0)
    :restitution-work-j       :float32         ; Integral of Fz * dz during upward rebound (dz > 0)
    :restitution-ratio-eta    :float32         ; eta = W_rebound / W_compress in [0.0 .. 1.0]
    :tangent-stiffness-npmm   :float32         ; Instantaneous d(Fz)/d(z)
    :crust-yield-pressure-kpa :float32         ; Mohr-Coulomb snow/crust punch-through ceiling
    :membrane-eigenfreq-hz    :float32         ; Dominant resonant frequency for trampoline/spring-bed
    :membrane-phase-rad       :float32         ; Instantaneous Hilbert/quadrature phase of oscillating bed
    :constitutive-regime      :keyword))       ; :bekker-plastic | :kelvin-viscoelastic | :duffing-harmonic

;; -----------------------------------------------------------------------------
;; 2. ONLINE RHEOLOGICAL HYSTERESIS IDENTIFIER (WORK-LOOP INTEGRATOR)
;; Classifies any unknown soft surface within 35 ms of contact by evaluating the
;; energy hysteresis loop H = \oint F_z dz and d(Fz)/d(z) curvature.
;; -----------------------------------------------------------------------------
(defprimitive identify-rheological-work-loop
  [state &key (plastic-eta-ceiling 0.16)       ; Snow, sand, mud: < 16% energy return
              (harmonic-eta-floor 0.62)]       ; Trampoline, spring-floor: > 62% energy return
  (-> state
      (evaluate-constitutive-hysteresis
        :state-bank 'rheological-substrate-soa-state
        :wrench-source :generalized-momentum-observer
        :kinematic-source :inertial-minus-fk-sinkage
        :regime-classifier
          ((:when-eta-below plastic-eta-ceiling
             :set-regime :bekker-plastic)      ; Deep snow / loose granular
           (:when-eta-between [plastic-eta-ceiling harmonic-eta-floor]
             :set-regime :kelvin-viscoelastic) ; Mattress / foam / soft turf
           (:when-eta-above harmonic-eta-floor
             :set-regime :duffing-harmonic))))) ; Trampoline / high-Q spring bed

;; -----------------------------------------------------------------------------
;; 3. DEFORMABLE-GROUND InEKF ODOMETRY DECOUPLER (ZERO SINKAGE HALLUCINATION)
;; Prevents the Right-Invariant SE_k(3) EKF from mistaking downward foot/knee
;; snow sinkage or trampoline deflection (-dz_sink) for upward CoM flight (+dz_com)!
;; -----------------------------------------------------------------------------
(defprimitive decouple-sinkage-from-inekf-odometry [state]
  (-> state
      (augment-invariant-ekf-contact-model
        :substrate-state 'rheological-substrate-soa-state
        ;; Model contact anchor position p_c(t) as a moving vertical Kelvin-Bekker state
        ;; driven by measured Fz rather than a rigid zero-velocity constraint:
        :vertical-anchor-kinematics :rheological-compliance-observer
        :reject-rebound-velocity-from-static-map true)))

;; -----------------------------------------------------------------------------
;; 4. MOHR-COULOMB PATCH PRESSURE REGULATOR ("SNOWSHOE" AREA FOLIATION)
;; Enforces Fz_i / Area_i(q) <= sigma_crust. If a small patch (foot, knee-tip,
;; palm) approaches punch-through pressure in snow or deep foam, automatically
;; rotates the 1-Jet posture to lay flat the entire shin, forearm, or thigh!
;; -----------------------------------------------------------------------------
(defprimitive regulate-mohr-coulomb-patch-pressure
  [state &key (max-contact-pressure-kpa 18.0)  ; ~18 kPa for uncompacted snow; ~35 kPa for soft mattress
              (area-expansion-gain 2.4)]
  (-> state
      (enforce-patch-pressure-barrier
        :substrate-state 'rheological-substrate-soa-state
        :pressure-ceiling-kpa max-contact-pressure-kpa
        :area-foliation-synergies
          ;; 1. Foot punch-through guard -> Flatten ankle & recruit full shin (`:anterior-tibial-bilateral`)
          ((:narrow-patch [:plantar-bilateral :metatarsal-equinus-bilateral]
             :broad-snowshoe-patch :anterior-tibial-bilateral
             :area-multiplier 4.8)
           ;; 2. Carpal-heel punch-through guard -> Lay down full ulna & elbow (`:olecranon-ulnar-sphinx`)
           (:narrow-patch [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
             :broad-snowshoe-patch :olecranon-ulnar-sphinx
             :area-multiplier 5.5)
           ;; 3. Ischial buttock sinkage guard -> Spread load across full hamstrings (`:posterior-femoral`)
           (:narrow-patch [:ischial-bilateral]
             :broad-snowshoe-patch :posterior-femoral-bilateral
             :area-multiplier 3.2)))))

;; -----------------------------------------------------------------------------
;; 5. TWO-STAGE BEKKER SNOW CONSOLIDATION ("TAMP-AND-COMMIT" HOMOTOPY)
;; In deep snow or sand (:bekker-plastic), a new support patch cannot bear full
;; Poinsot shear until the granular medium is compacted. Applies a 25-35% Fz
;; pre-tamp pulse until d(Fz)/d(z) hardens, then commits full balance weight w_i!
;; -----------------------------------------------------------------------------
(defprimitive execute-two-stage-snow-consolidation
  [state &key (tamp-preload-fz-ratio 0.30)
              (consolidated-stiffness-target-npmm 48.0)
              (max-tamp-sinkage-mm 220.0)]
  (-> state
      (gate-homotopy-by-bekker-compaction
        :substrate-state 'rheological-substrate-soa-state
        :active-in-regime :bekker-plastic
        :stage-1-pre-tamp
          (:preload-weight-cap tamp-preload-fz-ratio
           :vertical-tamp-velocity-mps -0.35
           :max-allowable-sinkage-m (* 0.001 max-tamp-sinkage-mm))
        :stage-2-commit-trigger
          (:require-dF-dz-above-npmm consolidated-stiffness-target-npmm
           :lock-compacted-z-datum true
           :boost-internal-friction-angle-deg +14.0)))) ; Sintered snow increases shear cone mu!

;; -----------------------------------------------------------------------------
;; 6. AXIAL POST-HOLE CAVITY EXTRACTION (ZERO SNOW-WALL DRAG ON SWING/ROLL)
;; When a shin, foot, or arm is sunk deep inside a snow post-hole or foam trough,
;; blocks horizontal XY swing until the limb has retracted straight out along its
;; entry axis above the surrounding snow rim!
;; -----------------------------------------------------------------------------
(defprimitive extract-limb-from-posthole-cavity
  [state &key (min-cavity-depth-for-axial-pull-mm 45.0)
              (rim-clearance-margin-mm 30.0)]
  (-> state
      (modulate-swing-extraction-contour
        :substrate-state 'rheological-substrate-soa-state
        :trigger-sinkage-m (* 0.001 min-cavity-depth-for-axial-pull-mm)
        :retraction-phase-window [0.00 -> 0.24] ; First 24% of swing is pure vertical/axial pullout!
        :lock-xy-translation-until-z-clears (* 0.001 rim-clearance-margin-mm)
        :suction-breakaway-dither-hz 55.0)))    ; Break wet-snow/mud vacuum suction on lift!

;; -----------------------------------------------------------------------------
;; 7. SYMPLECTIC MEMBRANE PHASE-LOCK & IMPEDANCE ABSORBER (TRAMPOLINE ENGINE)
;; Controls interaction with high-Q oscillating membranes (:duffing-harmonic):
;; - `:conjugate-wave-absorber` -> Steals kinetic energy from the trampoline bed
;;   via anti-phase axial compression so the robot can kneel, sit, and roll calmly.
;; - `:resonant-elastic-harvest` -> Phase-locks the PLL clock to the trampoline
;;   to launch from sitting, kneeling, or supine straight to standing!
;; -----------------------------------------------------------------------------
(defprimitive lock-symplectic-membrane-resonator
  [state &key (interaction-mode :conjugate-wave-absorber) ; :conjugate-wave-absorber | :resonant-elastic-harvest
              (max-aerial-apogee-mm 380.0)]
  (-> state
      (couple-port-hamiltonian-to-membrane
        :substrate-state 'rheological-substrate-soa-state
        :active-in-regime :duffing-harmonic
        :mode-law
          (case interaction-mode
            ;; A. DAMPING MODE (Kneeling, sitting, rolling, or walking on a bouncing trampoline):
            (:conjugate-wave-absorber
              (:virtual-damping-match :critical-2-sqrt-mk
               :anti-phase-modal-compensation
                 (:drive-channel :mode-axial-compression
                  :phase-offset-rad 3.14159      ; Flex knees/hips in exact anti-phase to bed uplift!
                  :keep-world-com-z-invariant true)
               :suppress-cross-limb-wave-disturbance true))

            ;; B. HARVEST / LAUNCH MODE (Bounce from Buttocks/Back/Knees -> Upright Standing!):
            (:resonant-elastic-harvest
              (:pll-phase-lock-to-membrane true  ; Master clock BPM locks to sqrt(K_mem / m)!
               :concentric-pump-phase-rad 3.14159 ; Push into bed at maximum downward deflection
               :target-apogee-clearance-m (* 0.001 max-aerial-apogee-mm)
               :aerial-nullspace-reconfiguration
                 (:trigger-when-fz-below 0.08    ; At weightless apex of bounce, whip legs under CoM!
                  :morph-to-posture :plantar-upright
                  :landing-absorption-mode :conjugate-wave-absorber)))))))
