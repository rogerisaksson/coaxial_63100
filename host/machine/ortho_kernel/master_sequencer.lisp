;; =============================================================================
;; FILE: master_sequencer.lisp
;; DOMAIN: Phase-Locked Multiband Crossfade & Full Autonomous Lifecycle SoA
;; =============================================================================
(defpackage :ortho-dsp.sequencer
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.transducers
        :ortho-dsp.regimes
        :ortho-dsp.autonomics
        :ortho-dsp.surfaces
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.manipulation
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.perception-core
        :ortho-dsp.perception-xforms
        :ortho-dsp.affordance-core
        :ortho-dsp.affordance-xforms
        :ortho-dsp.power-thermal
        :ortho-dsp.supervisor
        :ortho-dsp.autotuner)
  (:export #:phase-locked-multiband-morph
           #:compile-complete-humanoid-soa))
(in-package :ortho-dsp.sequencer)

;; Staged Sub -> Mid -> High Multiband Transition Sequencer
(deftransition phase-locked-multiband-morph [source-state target-state &key (bars 2)]
  (sync-trigger
    :wait-for :double-support-zero-crossing
    :lock-zmp-margin-mm 18.0
    :duration-bars bars)
  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp  [(get-bpm source-state)  -> (get-bpm target-state)]
    :duty-ramp [(get-duty source-state) -> (get-duty target-state)]
    :window-bars [0.0 -> 1.0])
  (band-split-crossfade
    (sub-band  :nodes [:tarsal-coronal :tarsal-sagittal :calcaneus :tibiofemoral]
               :curve :equal-power
               :window-bars [0.0 -> 1.0]
               :enforce-guard :min-coronal-clearance-mm)
    (mid-band  :nodes [:sacroiliac-z :iliac-coronal :glenohumeral :cubital]
               :curve :logarithmic-swell
               :window-bars [0.5 -> 1.5]
               :enforce-guard :min-normal-fz-ratio)
    (high-band :nodes [:l4-l5-lumbar :c7-t12-thoracic :c1-c2-atlas :craniofacial]
               :curve :exponential-fast
               :window-bars [0.75 -> 2.0])))

;; Full Autonomous Humanoid Schedule:
;; Wilderness -> Rural -> Urban -> Cluttered Apartment -> Clear Blocking Chair ->
;; Blind Sit & Lie in Bed (High-Z) -> Assisted Egress -> Reorient Chair & Lounge
(defpatch compile-complete-humanoid-soa
  (let [forest-traverse (-> nominal-biped-gait
                            (transducer-viscoelastic-flat)
                            (xform-balance-biped-zmp)
                            (xform-perceptual-slam-base)
                            (specialize-with-affordance-field-projection)
                            (xform-geodesic-wayfinder-base :goal-xyz [250.0 80.0 12.0])
                            (specialize-env-dense-forest)
                            (specialize-with-tactile-foliage-parting :yield-force-threshold-n 18.0)
                            (limiter-high-mu-planar)
                            (master-anti-spasm-insert))

        urban-ice-walk  (-> nominal-biped-gait
                            (transducer-viscoelastic-flat)
                            (regime-coronal-catwalk :axial-profile :thoracic-kyphotic)
                            (xform-balance-biped-zmp)
                            (xform-perceptual-slam-base)
                            (specialize-with-dynamic-agent-bifurcation :prediction-horizon-s 2.5)
                            (xform-geodesic-wayfinder-base :goal-xyz [85.0 15.0 0.0])
                            (specialize-env-urban-streetscape :crowd-density :moderate)
                            (limiter-low-mu-shear)
                            (master-anti-spasm-insert))

        room-bed-rest   (-> nominal-biped-gait
                            (transducer-unshod)
                            (xform-balance-biped-zmp)
                            (xform-situational-affordance-engine :intent :lie-supine-on-fixture)
                            (specialize-env-cluttered-interior)
                            (specialize-with-autonomous-obstacle-relocation :clearance-margin-mm 320.0)
                            (specialize-with-bidirectional-postural-chaining)
                            (limiter-high-mu-planar)
                            (master-anti-spasm-insert))

        egress-to-chair (-> room-bed-rest
                            (xform-situational-affordance-engine :intent :sit-on-fixture)
                            (specialize-with-assistive-egress-bracing :max-brace-load-share 0.20)
                            (specialize-env-cluttered-interior)
                            (specialize-with-furniture-pose-alignment :preferred-room-gaze-yaw-rad 0.0)
                            (specialize-with-bidirectional-postural-chaining)
                            (specialize-with-contralateral-leg-cross :cross-leg :left)
                            (limiter-high-mu-planar)
                            (master-anti-spasm-insert))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000
      :schedule
        [(bars 0  16 forest-traverse)
         (bars 16 18 (phase-locked-multiband-morph forest-traverse urban-ice-walk :bars 2))
         (bars 18 32 urban-ice-walk)
         (bars 32 34 (phase-locked-multiband-morph urban-ice-walk room-bed-rest :bars 2))
         (oneshot    room-bed-rest)
         (dwell-ms   10000)
         (oneshot    egress-to-chair)])))
