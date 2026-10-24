;; =============================================================================
;; FILE: plantar_transducers.lisp
;; DOMAIN: Plantar Interface Impedance & Contact Geometry Modifiers
;; =============================================================================
(defpackage :ortho-dsp.transducers
  (:use :ortho-dsp.core)
  (:export #:transducer-unshod
           #:transducer-viscoelastic-flat
           #:transducer-point-plantarflexed))
(in-package :ortho-dsp.transducers)

(deftransducer transducer-unshod [gait]
  (-> gait
      (set-plantar-boundary
        :sole-stack-z-mm 0.0
        :foot-collision-w-mm 84.0
        :talocrural-pitch-bias-rad 0.0
        :support-polygon-x-mm 195.0
        :windlass-arch-q 1.40
        :metatarsal-splay-gain 1.35)
      (shape-transients :target :calcaneus
        :strike-mode (:set :midfoot-biased)
        :saturation  (:set :biological-pad-clip)
        :attack-ms   (:scale 0.45))))

(deftransducer transducer-viscoelastic-flat [gait]
  (-> gait
      (set-plantar-boundary
        :sole-stack-z-mm 32.0
        :foot-collision-w-mm 96.5
        :talocrural-pitch-bias-rad 0.035
        :support-polygon-x-mm 265.0
        :windlass-arch-q 0.60
        :metatarsal-splay-gain 0.0)
      (shape-transients :target :calcaneus
        :strike-mode (:set :heel-to-toe)
        :saturation  (:set :tanh-viscoelastic)
        :attack-ms   (:add 12.0)
        :decay-ms    (:scale 0.65)
        :sustain     (:set 0.25))))

(deftransducer transducer-point-plantarflexed [gait]
  (-> gait
      (set-plantar-boundary
        :sole-stack-z-mm 105.0
        :foot-collision-w-mm 78.0
        :talocrural-pitch-bias-rad 0.52
        :support-polygon-x-mm 55.0
        :windlass-arch-q 0.0
        :metatarsal-splay-gain 0.0)
      (shape-transients :target :calcaneus
        :strike-mode (:set :point-dirac)
        :saturation  (:set :hard-clip)
        :attack-ms   (:set 0.05)
        :decay-ms    (:set 6.0)
        :sustain     (:set 0.0))
      (modulate-node :tibiofemoral
        :flexion-cushion-rad (:set 0.0)
        :compliance          (:set 0.0))))
