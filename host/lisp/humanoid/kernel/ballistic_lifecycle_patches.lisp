;; =============================================================================
;; FILE: ballistic_lifecycle_patches.lisp
;; DOMAIN: Complete 1 kHz SoA Schedule: Walk -> Brisk Walk -> Double-Time ->
;;         In-Stride Leap -> Run -> Substrate-Gated Rapid Brake -> Standing Jump
;; =============================================================================
(defpackage :ortho-dsp.ballistic-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.sequencer
        :ortho-dsp.substrate-overlay
        :ortho-dsp.ballistic-core
        :ortho-dsp.ballistic-xforms)
  (:export #:compile-ballistic-velocity-lifecycle-soa))
(in-package :ortho-dsp.ballistic-patches)

(defpatch compile-ballistic-velocity-lifecycle-soa
  (let [;; 1. Normal Walk (1.25 m/s, 110 BPM, Duty 0.62; hers 1.22 m/s, 112.9 BPM)
        stage-1-walk         (-> nominal-biped-gait
                                 (transducer-viscoelastic-flat)
                                 (xform-continuous-velocity-spectrum :regime :normal-walk)
                                 (apply-universal-deformable-overlay)
                                 (limiter-high-mu-planar)
                                 (master-anti-spasm-insert))

        ;; 2. Brisk Walk / Power-Walk (1.85 m/s, 132 BPM, Duty 0.53, Pelvic Yaw Stride Boost; hers 1.80 m/s, 135.5 BPM)
        stage-2-brisk-walk   (-> stage-1-walk
                                 (xform-continuous-velocity-spectrum :regime :brisk-walk)
                                 (specialize-leap-over-sdf-gap-or-hurdle) ; Can leap over puddles during brisk walk!
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))

        ;; 3. Double-Time March / Double-Time Jog (2.65 m/s, 154 BPM, Duty 0.44, Low-Apex SLIP; hers 2.58 m/s, 158 BPM)
        stage-3-sprangmarsch (-> stage-2-brisk-walk
                                 (xform-continuous-velocity-spectrum :regime :sprangmarsch)
                                 (specialize-run-with-elastic-achilles-q-boost :achilles-resonance-q 1.85)
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))

        ;; 4. In-Stride Single-Leg Leap / Bound (1.45m bound over obstacle mid-stride! Hers 0.64 m, one knee)
        stage-4-stride-leap  (-> stage-3-sprangmarsch
                                 (xform-in-stride-unilateral-leap
                                   :leap-apex-z-mm 75.0    ; one knee's launch, hers (280.0)
                                   :leap-span-x-mm 638.0)  ; at her 2.58 m/s (1450.0)
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))

        ;; 5. Full Elastic SLIP Run / Running (4.10 m/s, 178 BPM, Duty 0.32; hers 4.00 m/s, 182.7 BPM)
        stage-5-full-run     (-> stage-3-sprangmarsch
                                 (xform-continuous-velocity-spectrum :regime :slip-run)
                                 (specialize-run-with-elastic-achilles-q-boost :achilles-resonance-q 2.25)
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))

        ;; 6. Rapid Substrate-Gated Braking back to Walk (2-step stop on high-mu, ABS stutter on low-mu)
        stage-6-rapid-brake  (-> stage-5-full-run
                                 (xform-substrate-gated-rapid-brake :target-walk-speed-mps 1.169) ; hers (1.20)
                                 (specialize-brake-with-aerodynamic-and-arm-counter :pitch-arrest-gain 1.15)
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))

        ;; 7. Stop & Execute Standing Two-Footed Jump (From a Standstill, Feet Together, 360mm vertical + 600mm forward;
        ;;    hers 187 + 311 at its angle)
        stage-7-standing-jmp (-> stage-6-rapid-brake
                                 (xform-standing-bilateral-jump
                                   :jump-height-z-mm 187.0            ; her knees' launch at its angle (360.0)
                                   :jump-distance-x-mm 311.0          ; (600.0)
                                   :countermovement-crouch-mm 185.1)  ; at her size (195.0)
                                 (apply-universal-deformable-overlay)
                                 (master-anti-spasm-insert))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(bars 0  4  stage-1-walk)
         (bars 4  6  (phase-locked-multiband-morph stage-1-walk stage-2-brisk-walk :bars 2))
         (bars 6  10 stage-2-brisk-walk)
         (bars 10 12 (phase-locked-multiband-morph stage-2-brisk-walk stage-3-sprangmarsch :bars 2))
         (bars 12 16 stage-3-sprangmarsch)
         (oneshot    stage-4-stride-leap)       ; Single-leg bound over obstacle mid-stride!
         (bars 17 23 stage-5-full-run)          ; Accelerate into full 4.1 m/s SLIP running (hers 4.0)
         (oneshot    stage-6-rapid-brake)       ; Substrate-gated rapid brake back to 1.2 m/s walk! (hers 1.17)
         (bars 25 27 stage-1-walk)
         (oneshot    stage-7-standing-jmp)])))  ; Center ZMP -> Standing two-footed jump -> Absorb & Stand
