;; =============================================================================
;; FILE: 05_universal_sequencer.lisp
;; DOMAIN: Phase-Locked Topology Morphing & 64-Byte Aligned SoA Compilation
;; =============================================================================
(defpackage :ortho-dsp.sequencer
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.topologies
        :ortho-dsp.modulators
        :ortho-dsp.surfaces)
  (:export #:universal-topology-morph
           #:compile-omnidirectional-suite))
(in-package :ortho-dsp.sequencer)

;; -----------------------------------------------------------------------------
;; UNIVERSAL MULTIBAND MORPH (HANDLES DUTY-FACTOR & INTER-LIMB PHASE SLEWING)
;; -----------------------------------------------------------------------------
(deftransition universal-topology-morph [source-state target-state &key (bars 2)]
  (sync-trigger
    :wait-for :stance-nadir-zero-crossing ; Works across both double-support and aerial gaits
    :lock-zmp-margin-mm 18.0
    :duration-bars bars)

  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp        [(get-bpm source-state)        -> (get-bpm target-state)]
    :phase-diff-ramp [(get-phase-diff source-state) -> (get-phase-diff target-state)]
    :duty-l-ramp     [(get-duty-l source-state)     -> (get-duty-l target-state)]
    :duty-r-ramp     [(get-duty-r source-state)     -> (get-duty-r target-state)]
    :window-bars     [0.0 -> 1.0])

  ;; Staged Sub -> Mid -> High crossover prevents coronal clipping and sagittal ZMP trips
  (band-split-crossfade
    (sub-band  :nodes [:tarsal-coronal :tarsal-sagittal :calcaneus :tibiofemoral]
               :curve :equal-power
               :window-bars [0.0 -> 1.0]
               :collision-guard :min-coronal-clearance-mm)

    (mid-band  :nodes [:com-dynamics :iliac-coronal :glenohumeral]
               :curve :logarithmic-swell
               :window-bars [0.4 -> 1.5]
               :contact-guard :min-normal-force-ratio)

    (high-band :nodes [:l4-l5-lumbar :c7-t12-thoracic :c1-c2-atlas :craniofacial]
               :curve :exponential-fast
               :window-bars [0.75 -> 2.0])))

;; -----------------------------------------------------------------------------
;; COMPOSITE PATCH LIBRARY (ARBITRARY ORTHOGONAL COMBINATIONS)
;; -----------------------------------------------------------------------------
(defpatch patch-sneaker-runway
  (-> nominal-biped-manifold
      (transducer-elastomer-flat)
      (topology-pendular-walk :bpm 110.0)
      (modulator-coronal-catwalk :posture-profile :thoracic-kyphotic-bias)
      (surface-high-mu-planar)))

(defpatch patch-stealth-crouch-on-ice
  (-> nominal-biped-manifold
      (transducer-elastomer-flat)
      (topology-level-com-stealth :crouch-depth-mm 165.0)
      (surface-low-mu-ice)))

(defpatch patch-loaded-boot-snow-ascent
  (-> nominal-biped-manifold
      (transducer-heavy-lugged-boot)
      (topology-pendular-walk :bpm 96.0)
      (modulator-continuous-slope :grade-angle-rad 0.21)
      (modulator-inertial-payload :mass-kg 25.0 :mount :dorsal-pack)
      (surface-granular-dissipative :sink-depth-mm 75.0)))

(defpatch patch-barefoot-stair-descent
  (-> nominal-biped-manifold
      (transducer-barefoot)
      (topology-pendular-walk)
      (modulator-discrete-stairs :riser-z-mm -175.0 :tread-x-mm 280.0) ; Negative Z = Eccentric Descent
      (surface-high-mu-planar)))

(defpatch patch-asymmetric-gallop-trail
  (-> nominal-biped-manifold
      (transducer-elastomer-flat)
      (topology-asymmetric-gallop :lead-side :right)
      (surface-stochastic-uneven :rugosity-mm 40.0)))

(defpatch patch-ballistic-sprint
  (-> nominal-biped-manifold
      (transducer-elastomer-flat)
      (topology-ballistic-slip-run :bpm 188.0 :stride-mm 1420.0)
      (surface-high-mu-planar)))

;; -----------------------------------------------------------------------------
;; STATIC DMA RINGBUFFER COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-omnidirectional-suite
  (compile-to-soa
    :align-bytes 64
    :sample-rate-hz 1000
    :schedule
      [(bars 0  4  patch-sneaker-runway)
       (bars 4  6  (universal-topology-morph patch-sneaker-runway patch-stealth-crouch-on-ice :bars 2))
       (bars 6  10 patch-stealth-crouch-on-ice)
       (bars 10 12 (universal-topology-morph patch-stealth-crouch-on-ice patch-barefoot-stair-descent :bars 2))
       (bars 12 16 patch-barefoot-stair-descent)
       (bars 16 18 (universal-topology-morph patch-barefoot-stair-descent patch-asymmetric-gallop-trail :bars 2))
       (bars 18 22 patch-asymmetric-gallop-trail)
       (bars 22 24 (universal-topology-morph patch-asymmetric-gallop-trail patch-ballistic-sprint :bars 2))
       (bars 24 32 patch-ballistic-sprint)]))
