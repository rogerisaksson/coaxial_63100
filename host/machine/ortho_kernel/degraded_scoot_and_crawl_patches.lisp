;; =============================================================================
;; FILE: degraded_scoot_and_crawl_patches.lisp
;; DOMAIN: Complete 1 kHz SoA Schedule for Healthy & Broken-Actuator Scoot/Crawl
;; =============================================================================
(defpackage :ortho-dsp.degraded-crawl-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.ground-xforms
        :ortho-dsp.substrate-overlay
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.sequencer
        :ortho-dsp.degraded-crawl-core
        :ortho-dsp.degraded-crawl-xforms)
  (:export #:compile-scoot-and-lowcrawl-suite-soa))
(in-package :ortho-dsp.degraded-crawl-patches)

(defpatch compile-scoot-and-lowcrawl-suite-soa
  (let [;; 1. Healthy Seated Butt-Scoot (All 4 limbs working: arms + both heels row on buttocks)
        healthy-butt-scoot (-> nominal-biped-gait
                               (transducer-unshod)
                               (xform-to-floor-buttocks-seated :with-hands-and-arms-support true)
                               (xform-locomotion-seated-butt-scoot
                                 :velocity-vx-vy-wz [0.28 0.0 0.0]
                                 :bpm 80.0
                                 :buttock-unweight-ratio 0.72)
                               (specialize-butt-scoot-propulsion-mode :technique :anterograde-hamstring-row)
                               (apply-universal-deformable-overlay)
                               (limiter-high-mu-planar)
                               (master-anti-spasm-insert))

        ;; 2. Degraded Butt-Scoot with BROKEN Left Leg (Left knee/hip in High-Z freewheel!):
        ;;    Pushes with both arms + RIGHT heel only, automatically cancels yaw spin,
        ;;    and tilts pelvis +12.5 deg right to tow the dead left leg safely!
        broken-leg-scoot   (-> healthy-butt-scoot
                               (specialize-with-actuator-fault-injection
                                 :affected-limb :hind-left
                                 :fault-type :high-z-freewheel
                                 :residual-authority 0.00)
                               (specialize-butt-scoot-propulsion-mode :technique :retrograde-quad-push)
                               (apply-universal-deformable-overlay)
                               (master-anti-spasm-insert))

        ;; 3. Healthy Prone Low-Crawl ("Leopard Crawl" flat on stomach under 260mm obstacle)
        healthy-low-crawl  (-> nominal-biped-gait
                               (transducer-viscoelastic-flat)
                               (xform-floor-to-prone-flat)
                               (xform-locomotion-prone-low-crawl
                                 :velocity-vx-vy-wz [0.25 0.0 0.0]
                                 :bpm 72.0
                                 :belly-unweight-ratio 0.45)
                               (specialize-low-crawl-frog-piston-topology
                                 :drive-topology :alternating-diagonal-leopard)
                               (apply-universal-deformable-overlay)
                               (limiter-high-mu-planar)
                               (master-anti-spasm-insert))

        ;; 4. Degraded Prone Low-Crawl with BROKEN Left Leg (Unilateral Single-Piston Frog Crawl):
        ;;    Both forearms pull in front while the healthy RIGHT frog-leg drives along the floor!
        broken-low-crawl   (-> healthy-low-crawl
                               (specialize-with-actuator-fault-injection
                                 :affected-limb :hind-left
                                 :fault-type :high-z-freewheel
                                 :residual-authority 0.00)
                               (specialize-low-crawl-frog-piston-topology
                                 :drive-topology :unilateral-single-piston)
                               (apply-universal-deformable-overlay)
                               (master-anti-spasm-insert))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(bars 0  8  healthy-butt-scoot)
         (bars 8  16 broken-leg-scoot)
         (bars 16 24 healthy-low-crawl)
         (bars 24 32 broken-low-crawl)])))
