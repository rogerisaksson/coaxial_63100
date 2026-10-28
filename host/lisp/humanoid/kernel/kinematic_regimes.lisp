;; -----------------------------------------------------------------------------
;; MODULE 1.5: kinematic_regimes
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.regimes
  (:use :ortho-dsp.core)
  (:export #:regime-coronal-catwalk
           #:regime-ballistic-slip
           #:regime-step-quantizer))
(in-package :ortho-dsp.regimes)

(defmodifier regime-coronal-catwalk [gait &key (axial-profile :thoracic-kyphotic)]
  (let [foot-w      (get-param gait :foot-collision-w-mm)
        pitch-bias  (get-param gait :talocrural-pitch-bias-rad)
        target-tr-w (* 0.44 foot-w)
        req-bow-mm  (+ (- foot-w target-tr-w) 7.5)]
    (-> gait
        (quantize-grid :bpm (:scale 1.03636) :stance-duty-pwm (:add -0.04)
                       :swing-gate (if (> pitch-bias 0.3) 0.0 0.04) :strip-jitter true)
        (modulate-node :tibiofemoral
          :flexion-cushion-rad (if (> pitch-bias 0.3) (:set 0.0) (:set 0.14))
          :compliance          (if (> pitch-bias 0.3) (:set 0.0) (:set 0.45)))
        (mid-side-rebalance
          :narrow-stereo (:node :tarsal-coronal :track-width-mm target-tr-w
                          :swing-circumduction-mm req-bow-mm
                          :circumduction-window-phase [0.10 -> 0.40] :circumduction-shape :raised-cosine)
          :stereo-mod    (:node :iliac-coronal :wave :sine-fundamental :lfo-amp-rad (:scale 1.92) :stereo-width 0.60))
        (route-sidechain
          :key-input :calcaneus :target :sacroiliac-z :topology :second-order-damped
          :ratio 4.5 :attack-ms (:clamp-min 105.0) :release-ms 260.0 :release-curve :logarithmic
          :duck-depth-mm (if (> pitch-bias 0.3) -34.0 -26.0) :min-fz-guard :min-normal-fz-ratio)
        (apply-filter-bank
          (dc-bias         :target :l4-l5-lumbar
                           :offset-rad (case axial-profile (:thoracic-kyphotic -0.04) (:lumbar-lordotic -0.32))
                           :q-factor   (case axial-profile (:thoracic-kyphotic 0.35)  (:lumbar-lordotic 0.95)))
          (dc-bias         :target :c7-t12-thoracic
                           :offset-rad (case axial-profile (:thoracic-kyphotic +0.20) (:lumbar-lordotic -0.08))
                           :q-factor 0.80
                           :zmp-counter-bias-x-mm (case axial-profile (:thoracic-kyphotic -24.0) (:lumbar-lordotic +14.0)))
          (butterworth-lpf :target :c7-t12-thoracic :cutoff-hz 2.2 :slope-db 12)
          (phase-delay     :target :glenohumeral    :phase-lag-rad 0.25 :gain-db -4.5)
          (dc-bias         :target :c1-c2-atlas     :offset-rad 0.12 :ceiling-rad 0.01)
          (hard-mute       :target :craniofacial    :velocity-sensitivity 0.0)))))

(defmodifier regime-ballistic-slip [gait &key (target-bpm 164.0)]
  (-> gait
      (quantize-grid :bpm (:set target-bpm) :stance-duty-pwm (:set 0.36) :swing-gate 0.0)
      (shape-transients :target :calcaneus
        :strike-mode (:set :midfoot-elastic) :attack-ms (:set 18.0) :decay-ms (:set 140.0) :sustain (:set 0.0))
      (modulate-node :tibiofemoral :flexion-cushion-rad (:set 0.48) :compliance (:set 0.92) :rebound-q (:set 2.40))
      (modulate-node :tarsal-sagittal :stride-length-mm (:set 1180.0) :swing-clearance-z-mm (:set 145.0))
      (engage-sub-oscillator :target :sacroiliac-z :mode :ballistic-slip :freq-multiplier 2.0
                             :stance-compression-mm -65.0 :aerial-apex-mm +58.0)
      (apply-filter-bank
        (dc-bias         :target :c7-t12-thoracic :offset-rad 0.18 :cutoff-hz 4.5 :slope-db 12)
        (compressor-gain :target :glenohumeral    :gain-db +4.5 :elbow-flex-rad 1.45)
        (brickwall-limit :target :c1-c2-atlas     :ceiling-rad 0.02 :vor-stabilize true))))

(defmodifier regime-step-quantizer [gait &key (riser-z-mm 175.0) (tread-x-mm 280.0)]
  (-> gait
      (quantize-grid :bpm (:set 92.0) :stance-duty-pwm (:set 0.68) :swing-gate 0.0)
      (sample-and-hold-terrain :target :tarsal-sagittal
        :stride-length-mm (:set tread-x-mm) :step-elevation-z-mm (:set riser-z-mm)
        :edge-margin-z-mm 32.0 :trajectory-envelope :parabolic-step-over)
      (shape-transients :target :calcaneus :strike-mode (:set :forefoot-flat-latch)
                        :attack-ms (:set 45.0) :sustain (:set 0.85))
      (modulate-node :tibiofemoral :initial-flexion-rad 1.05
                     :extension-makeup-gain-j (* 9.80665 (/ riser-z-mm 1000.0)))
      (apply-filter-bank
        (dc-bias :target :c7-t12-thoracic :offset-rad 0.24 :zmp-lead-comp true)
        (dc-bias :target :iliac-coronal   :lfo-amp-rad 0.04 :anti-trendelenburg-gain 1.6))))
