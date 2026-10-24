;; -----------------------------------------------------------------------------
;; MODULE 1.6: autonomic_idle_physiology
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.autonomics
  (:use :ortho-dsp.core)
  (:export #:inject-autonomic-physiology))
(in-package :ortho-dsp.autonomics)

(defmodifier inject-autonomic-physiology
  [state &key (respiratory-rate-hz 0.22) (metabolic-exertion-scale 1.0) (stiction-dither-hz 180.0)]
  (-> state
      (modulate-nullspace-oscillator
        :carrier-freq-hz (* respiratory-rate-hz metabolic-exertion-scale)
        :waveform :asymmetric-inhalation-swell
        :coupled-nodes
          ((:c7-t12-thoracic :pitch-amp-rad 0.018 :phase-rad 0.0)
           (:l4-l5-lumbar    :pitch-amp-rad -0.008 :phase-rad 3.14159)
           (:glenohumeral    :elevation-amp-mm 3.2 :phase-rad 0.15)
           (:c1-c2-atlas     :pitch-amp-rad -0.010 :phase-rad 0.0)))
      (inject-stochastic-com-drift
        :spectrum :pink-noise-1-over-f :bandwidth-hz [0.05 0.65]
        :max-excursion-mm 6.5 :scale-by-manifold-weight :plantar-bilateral)
      (apply-zero-velocity-dither
        :trigger-below-radps 0.008 :dither-freq-hz stiction-dither-hz
        :torque-amp-nm 0.18 :disable-in-high-z true)))
