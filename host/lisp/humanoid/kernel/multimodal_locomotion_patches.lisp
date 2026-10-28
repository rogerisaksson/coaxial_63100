;; =============================================================================
;; FILE: multimodal_locomotion_patches.lisp
;; DOMAIN: Autonomous Multi-Modal Gait Switching Schedule (1 kHz SoA)
;; =============================================================================
(defpackage :ortho-dsp.quad-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.sequencer
        :ortho-dsp.quad-core
        :ortho-dsp.quad-xforms)
  (:export #:patch-pronograde-dog-trot
           #:patch-low-table-hands-and-knees-crawl
           #:patch-supine-crab-and-underbed-scoot
           #:compile-multimodal-locomotion-soa))
(in-package :ortho-dsp.quad-patches)

;; 1. Dynamic Pronograde "Dog/Bear" Trot (Hands + Equinus Forefeet, Diagonal 2-Beat Sync)
(defpatch patch-pronograde-dog-trot [base-gait &key (goal-xyz [6.0 0.0 0.0])]
  (-> base-gait
      (transducer-viscoelastic-flat)
      (specialize-transition-biped-to-quadruped :target-orientation :pronograde-belly-down)
      (specialize-pronograde-contact-interface :contact-regime :digitigrade-bear)
      (specialize-quad-phase-topology :gait-symmetry :diagonal-trot)
      (xform-geodesic-wayfinder-base :goal-xyz goal-xyz)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; 2. Low-Ceiling Under-Table Crawl (Hands + Tibial Knees, Lateral Sequence 3-Point Stability)
(defpatch patch-low-table-hands-and-knees-crawl [quad-state &key (goal-xyz [9.5 1.2 0.0])]
  (-> quad-state
      (specialize-pronograde-contact-interface :contact-regime :hands-and-knees)
      (specialize-quad-phase-topology :gait-symmetry :crawl-lateral-sequence)
      (xform-geodesic-wayfinder-base :goal-xyz goal-xyz)
      (specialize-env-cluttered-interior)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; 3. Supine Inverted Crab Walk -> Peristaltic Under-Bed Scoot (Belly Up, Hands + Feet + Ischial Bridge)
(defpatch patch-supine-crab-and-underbed-scoot [base-gait &key (goal-xyz [3.2 -2.0 0.0])]
  (-> base-gait
      (transducer-unshod)
      (specialize-transition-biped-to-quadruped :target-orientation :supine-belly-up)
      (specialize-quad-phase-topology :gait-symmetry :diagonal-trot)
      (specialize-supine-peristaltic-underbed-scoot :max-ceiling-z-mm 275.0)
      (xform-geodesic-wayfinder-base :goal-xyz goal-xyz)
      (specialize-env-cluttered-interior)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-multimodal-locomotion-soa
  (let [dog-trot    (patch-pronograde-dog-trot nominal-biped-gait :goal-xyz [6.0 0.0 0.0])
        table-crawl (patch-low-table-hands-and-knees-crawl dog-trot :goal-xyz [9.5 1.2 0.0])
        supine-crab (patch-supine-crab-and-underbed-scoot nominal-biped-gait :goal-xyz [3.2 -2.0 0.0])]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(bars 0  8  dog-trot)
         (bars 8  10 (phase-locked-multiband-morph dog-trot table-crawl :bars 2))
         (bars 10 18 table-crawl)
         (bars 18 26 supine-crab)])))
