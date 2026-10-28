;; =============================================================================
;; FILE: hitchin_isomonodromic_bus.lisp
;; DOMAIN: Genus-0 Parabolic Hitchin System, Schlesinger Contact Transport &
;;         Hyperkahler Twistor Regularization for Closed Kinematic Loops
;; REQUIRES: ortho_dsp_somatic_substrate.lisp
;; =============================================================================
(defpackage :ortho-dsp.hitchin
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields)
  (:export #:construct-parabolic-higgs-field
           #:extract-hitchin-spectral-base
           #:propagate-schlesinger-isomonodromy
           #:regularize-via-hyperkahler-twistor
           #:xform-hitchin-integrable-balance))
(in-package :ortho-dsp.hitchin)

;; -----------------------------------------------------------------------------
;; 1. GENUS-0 PARABOLIC HIGGS FIELD ON CP^1 \ {z_0, z_1 ... z_m}
;; Maps each active contact manifold i to a puncture z_i = r_i * exp(j * phi_i)
;; on the Riemann sphere, with residue mu_i = Ad^T_{T_0,i} * W_i in se(3)*.
;; Cauchy's Residue Theorem enforces sum(mu_i) - W_CoM == 0 identically.
;; -----------------------------------------------------------------------------
(defprimitive construct-parabolic-higgs-field
  [state &key (active-manifolds [:plantar-bilateral
                                 :metatarsal-equinus-bilateral
                                 :posterior-femoral-bilateral
                                 :ischial-bilateral
                                 :ulnar-carpal-shove-bilateral])]
  (-> state
      (bind-meromorphic-lax-matrix
        :riemann-surface :cp1-punctured-genus-0
        :lie-algebra     :se3-dual-plucker
        :puncture-mapping
          (:manifolds active-manifolds
           ;; Complex pole z_i encodes kinematic lever arm r_i and PLL phase phi_i
           :pole-coordinate-law :lever-arm-and-pll-phase
           ;; Parabolic weight alpha_i in [0, 1] bound directly to GMO normal force w_i(t)
           :parabolic-weight-source :active-manifold-weights
           :infinity-pole-residue   :negated-centroidal-poinsot-wrench))))

;; -----------------------------------------------------------------------------
;; 2. HITCHIN FIBRATION SPECTRAL CURVE: det(Phi(z) - lambda * I) = 0
;; Evaluates quadratic/cubic Casimir trace invariants Tr(Phi(z)^2) to project
;; coupled nonlinear limb dynamics onto flat Action-Angle torus coordinates T^d.
;; -----------------------------------------------------------------------------
(defprimitive extract-hitchin-spectral-base
  [state &key (spectral-evaluation-points 8)
              (lock-angle-flows-to-pll true)]
  (-> state
      (solve-hitchin-fibration-map
        :quadrature-nodes spectral-evaluation-points
        ;; Base coordinates b in B: commuting Gaudin Hamiltonians H_i = Res_{z_i} Tr(Phi(z)^2)/2
        :base-invariants-output :gaudin-commuting-hamiltonians
        ;; Fiber coordinates theta in Jac(S) = T^d: linearize Coriolis cross-coupling!
        :abel-jacobi-torus-flow
          (:synchronize-with-master-clock lock-angle-flows-to-pll
           :export-spectral-residuals-to-autotuner true))))

;; -----------------------------------------------------------------------------
;; 3. SCHLESINGER ISOMONODROMIC DEFORMATION (SLIDING MULTI-CONTACT TRANSPORT)
;; Evolves contact wrenches via d(mu_i)/dt = sum_{j!=i} [mu_i, mu_j] * d(log(z_i - z_j))/dt
;; as contact patches slide (e.g. thigh scoot on 0.82m chair or shoving furniture),
;; guaranteeing zero internal wrench fighting and constant spectral monodromy.
;; -----------------------------------------------------------------------------
(defprimitive propagate-schlesinger-isomonodromy
  [state &key (coadjoint-bracket-damping 0.98)
              (min-pole-separation-eps 0.025)]
  (-> state
      (integrate-schlesinger-wrench-transport
        :higgs-source :active-meromorphic-lax-matrix
        :pole-velocity-source :contact-patch-tangential-velocity
        :lie-bracket-law :se3-coadjoint-commutator
        :collinear-pole-regularization min-pole-separation-eps
        :monodromy-preservation-gain coadjoint-bracket-damping)))

;; -----------------------------------------------------------------------------
;; 4. HYPERKAHLER TWISTOR ROTATION (CLOSED-CHAIN SINGULARITY BYPASS)
;; When closed kinematic loops (feet + hands braced) approach rank deficiency
;; under complex structure I (position Jacobian), rotates along the twistor S^2
;; toward complex structure J (momentum/wrench space) without energy dissipation.
;; -----------------------------------------------------------------------------
(defprimitive regularize-via-hyperkahler-twistor
  [state &key (singular-value-onset 0.055)
              (max-twistor-angle-rad 1.0472)]     ; Up to 60 deg rotation from I toward J
  (-> state
      (apply-twistor-sphere-rotation
        :target-projector :khatib-dynamically-consistent
        :complex-structures [:i-configuration-tangent
                             :j-coadjoint-momentum
                             :k-symplectic-cross]
        :rotation-trigger-sigma singular-value-onset
        :max-phase-angle-rad max-twistor-angle-rad
        :preserve-hyperkahler-volume-form true)))

;; -----------------------------------------------------------------------------
;; 5. COMPOSITE HITCHIN-INTEGRABLE BALANCE & SLIDING MODIFIER
;; Drop-in upgrade for high-DoF multi-contact sliding & closed-chain maneuvers.
;; -----------------------------------------------------------------------------
(defmodifier xform-hitchin-integrable-balance [state]
  (-> state
      (construct-parabolic-higgs-field)
      (extract-hitchin-spectral-base :lock-angle-flows-to-pll true)
      (propagate-schlesinger-isomonodromy :min-pole-separation-eps 0.025)
      (regularize-via-hyperkahler-twistor :singular-value-onset 0.055)))
