;; -----------------------------------------------------------------------------
;; MODULE 2.9: master_sequencer
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.sequencer
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.transducers
        :ortho-dsp.regimes
        :ortho-dsp.autonomics
        :ortho-dsp.surfaces
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.manipulation
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.perception-core
        :ortho-dsp.perception-xforms
        :ortho-dsp.affordance-core
        :ortho-dsp.affordance-xforms
        :ortho-dsp.power-thermal
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.autotuner)
  (:export #:phase-locked-multiband-morph
           #:compile-complete-humanoid-soa))
(in-package :ortho-dsp.sequencer)

(deftransition phase-locked-multiband-morph [source-state target-state &key (bars 2)]
  (sync-trigger :wait-for :double-support-zero-crossing :lock-zmp-margin-mm 18.0 :duration-bars bars)
  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp  [(get-bpm source-state)  -> (get-bpm target-state)]
    :duty-ramp [(get-duty source-state) -> (get-duty target-state)]
    :window-bars [0.0 -> 1.0])
  (band-split-crossfade
    (sub-band  :nodes [:tarsal-coronal :tarsal-sagittal :calcaneus :tibiofemoral]
               :curve :equal-power :window-bars [0.0 -> 1.0] :enforce-guard :min-coronal-clearance-mm)
    (mid-band  :nodes [:sacroiliac-z :iliac-coronal :glenohumeral :cubital]
               :curve :logarithmic-swell :window-bars [0.5 -> 1.5] :enforce-guard :min-normal-fz-ratio)
    (high-band :nodes [:l4-l5-lumbar :c7-t12-thoracic :c1-c2-atlas :craniofacial]
               :curve :exponential-fast :window-bars [0.75 -> 2.0])))

(defpatch compile-complete-humanoid-soa
  (let [door-and-room-nav (-> nominal-biped-gait
                              (transducer-viscoelastic-flat)
                              (xform-balance-biped-zmp)
                              (xform-perceptual-slam-base)
                              (specialize-with-affordance-field-projection)
                              (xform-geodesic-wayfinder-base :goal-xyz [4.5 -1.2 0.0])
                              (specialize-env-cluttered-interior)
                              (limiter-high-mu-planar)
                              (master-anti-spasm-insert))

        shove-and-high-sit (-> door-and-room-nav
                               (xform-situational-affordance-engine :intent :sit-on-fixture)
                               (specialize-with-autonomous-obstacle-relocation :clearance-margin-mm 320.0)
                               (specialize-with-bidirectional-postural-chaining)
                               (release-impedance-wavefront :terminal-mode :seated-active-rest)
                               (master-anti-spasm-insert))

        dismount-to-bed    (-> shove-and-high-sit
                               (xform-situational-affordance-engine :intent :lie-supine-on-fixture)
                               (specialize-with-bidirectional-postural-chaining)
                               (master-anti-spasm-insert))

        wake-and-stairs    (-> dismount-to-bed
                               (xform-situational-affordance-engine :intent :bipedal-upright)
                               (specialize-with-assistive-egress-bracing :max-brace-load-share 0.22)
                               (specialize-with-bidirectional-postural-chaining)
                               (xform-somatic-stair-traversal :nominal-riser-dz-mm +175.0 :nominal-tread-dx-mm 280.0)
                               (specialize-stair-descent-eccentric :riser-drop-mm -175.0 :tread-depth-mm 280.0)
                               (specialize-with-wall-or-banister-skim :side :right)
                               (limiter-high-mu-planar)
                               (master-anti-spasm-insert))]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(bars 0 8  door-and-room-nav)
         (oneshot   shove-and-high-sit)
         (dwell-ms  4000)
         (oneshot   dismount-to-bed)
         (dwell-ms  8000)
         (oneshot   wake-and-stairs)])))
