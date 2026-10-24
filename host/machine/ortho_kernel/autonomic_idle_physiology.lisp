;; =============================================================================
;; FILE: autonomic_idle_physiology.lisp
;; DOMAIN: Respiratory Sub-Bass Carrier, 1/f Postural Sway & Stiction Dither
;; =============================================================================
(defpackage :ortho-dsp.autonomics
  (:use :ortho-dsp.core)
  (:export #:inject-autonomic-physiology))
(in-package :ortho-dsp.autonomics)

;; Eliminates the "frozen statue" uncanny valley and drivetrain Coulomb stiction
;; when master locomotion BPM -> 0 (standing idle, seated lounge, or supine rest).
(defmodifier inject-autonomic-physiology
  [state &key (respiratory-rate-hz 0.22)
              (metabolic-exertion-scale 1.0)
              (stiction-dither-hz 180.0)]
  (-> state
      ;; 1. Coupled Thoracic-Lumbar-Clavicular Respiratory Wave (Null-Space Projected)
      (modulate-nullspace-oscillator
        :carrier-freq-hz (* respiratory-rate-hz metabolic-exertion-scale)
        :waveform :asymmetric-inhalation-swell     ; 40% inhale, 60% exponential exhale
        :coupled-nodes
          ((:c7-t12-thoracic :pitch-amp-rad 0.018 :phase-rad 0.0)
           (:l4-l5-lumbar    :pitch-amp-rad -0.008 :phase-rad 3.14159) ; Counter-nutation keeps head still
           (:glenohumeral    :elevation-amp-mm 3.2 :phase-rad 0.15)
           (:c1-c2-atlas     :pitch-amp-rad -0.010 :phase-rad 0.0)))   ; Stabilize gaze against breath

      ;; 2. 1/f Pink-Noise Centroidal Postural Sway (Strictly Bounded Inside 15% CWC Core)
      (inject-stochastic-com-drift
        :spectrum :pink-noise-1-over-f
        :bandwidth-hz [0.05 0.65]
        :max-excursion-mm 6.5                      ; Subtle biological weight shifting when standing
        :scale-by-manifold-weight :plantar-bilateral) ; Automatically zeroes out when lying down

      ;; 3. Sub-Threshold High-Frequency Stiction Dither (Keeps Harmonic Drives Lubricated)
      (apply-zero-velocity-dither
        :trigger-below-radps 0.008
        :dither-freq-hz stiction-dither-hz
        :torque-amp-nm 0.18                        ; Zero net movement; breaks static friction breakaway spike
        :disable-in-high-z true)))
