;; =============================================================================
;; FILE: terrestrial_locomotion_patches.lisp
;; DOMAIN: Complete Non-Bipedal Locomotion Suite Schedule (1 kHz SoA)
;; =============================================================================
(defpackage :ortho-dsp.terrestrial-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.sequencer
        :ortho-dsp.substrate-overlay
        :ortho-dsp.terrestrial-core
        :ortho-dsp.terrestrial-xforms)
  (:export #:compile-terrestrial-multimodal-suite-soa))
(in-package :ortho-dsp.terrestrial-patches)

(defpatch compile-terrestrial-multimodal-suite-soa
  (let [;; 1. Pronograde Dog Trot -> Hands-and-Knees Crawl
        dog-crawl         (-> nominal-biped-gait
                              (transducer-viscoelastic-flat)
                              (xform-gait-pronograde-dog-crawl :bpm 104.0 :velocity-vx-vy-wz [0.50 0.0 0.0])
                              (specialize-pronograde-knee-vs-bear-mode :mode :digitigrade-bear)
                              (apply-universal-deformable-overlay)
                              (limiter-high-mu-planar)
                              (master-anti-spasm-insert))

        ;; 2. Supine Back-to-Floor Crawl UP (+175mm) and DOWN (-175mm) a Grade/Staircase
        supine-up-down    (-> nominal-biped-gait
                              (transducer-unshod)
                              (xform-gait-supine-back-crawl :bpm 86.0 :velocity-vx-vy-wz [-0.32 0.0 0.0])
                              (specialize-supine-grade-and-stair-ratchet :slope-or-stair-dz-mm +175.0)
                              (apply-universal-deformable-overlay)
                              (limiter-high-mu-planar)
                              (master-anti-spasm-insert))

        ;; 3. Buttock Scooting & Asymmetric Hip-Drag Around Obstacles
        butt-scoot-drag   (-> nominal-biped-gait
                              (transducer-unshod)
                              (xform-gait-ischial-butt-scoot :bpm 78.0 :velocity-vx-vy-wz [0.28 0.08 0.0]
                                                             :vault-intensity 0.70)
                              (specialize-scoot-asymmetric-hip-drag :drag-side :right)
                              (apply-universal-deformable-overlay)
                              (limiter-high-mu-planar)
                              (master-anti-spasm-insert))

        ;; 4. Pure Lateral Crabwalk Sideways (Right then Left in Supine & Pronograde)
        sideways-crabwalk (-> nominal-biped-gait
                              (transducer-viscoelastic-flat)
                              (xform-gait-lateral-crabwalk :side-direction :right
                                                           :posture-mode :supine-belly-up
                                                           :crab-speed-mps 0.38)
                              (apply-universal-deformable-overlay)
                              (limiter-high-mu-planar)
                              (master-anti-spasm-insert))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(bars 0  8  dog-crawl)
         (bars 8  16 supine-up-down)
         (bars 16 24 butt-scoot-drag)
         (bars 24 32 sideways-crabwalk)])))
