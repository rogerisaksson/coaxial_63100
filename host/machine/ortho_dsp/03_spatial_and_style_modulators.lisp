;; =============================================================================
;; FILE: 03_spatial_and_style_modulators.lisp
;; DOMAIN: Terrain Elevation (Stairs/Slopes), Style Overlays & Inertial Payloads
;; =============================================================================
(defpackage :ortho-dsp.modulators
  (:use :ortho-dsp.core)
  (:export #:modulator-coronal-catwalk
           #:modulator-discrete-stairs
           #:modulator-continuous-slope
           #:modulator-inertial-payload
           #:modulator-antalgic-limp))
(in-package :ortho-dsp.modulators)

;; 1. Coronal Catwalk (Footwear-Adaptive Track Narrowing & Gravity-Clamped Sidechain)
(defmodifier modulator-coronal-catwalk [gait &key (posture-profile :thoracic-kyphotic-bias)]
  (let [foot-w      (get-param gait :foot-collision-w-mm)
        pitch-bias  (get-param gait :talocrural-pitch-bias-rad)
        target-tr-w (* 0.44 foot-w)
        req-bow-mm  (+ (- foot-w target-tr-w) 7.5)]
    (-> gait
        (modulate-oscillator
          :bpm (:scale 1.03636)
          :duty-factor-l (:add -0.04)
          :duty-factor-r (:add -0.04)
          :even-step-swing-offset (if (> pitch-bias 0.3) 0.0 0.04))
        (modulate-node :tarsal-coronal
          :track-width-mm (:set target-tr-w)
          :swing-circumduction-mm (:set req-bow-mm))
        (modulate-node :iliac-coronal
          :waveform (:set :sine-fundamental)
          :lfo-amplitude-rad (:scale 1.92)
          :stereo-width (:set 0.60))
        (route-envelope-sidechain
          :trigger-source :calcaneus
          :target-node :com-dynamics
          :attack-ms (:clamp-min 105.0)    ; Prevents Fz < 0.35g liftoff
          :release-ms 260.0
          :modulation-depth-mm (if (> pitch-bias 0.3) -34.0 -26.0)
          :normal-force-guard :min-normal-force-ratio)
        (modulate-node :l4-l5-lumbar
          :dc-offset-rad (:set (case posture-profile (:thoracic-kyphotic-bias -0.04) (:lumbar-lordotic-bias -0.32)))
          :q-factor      (:set (case posture-profile (:thoracic-kyphotic-bias 0.35)  (:lumbar-lordotic-bias 0.95))))
        (modulate-node :c7-t12-thoracic
          :dc-offset-rad (:set (case posture-profile (:thoracic-kyphotic-bias +0.20) (:lumbar-lordotic-bias -0.08)))
          :lpf-cutoff-hz (:set 2.2)
          :lpf-slope-db  (:set 12))
        (modulate-node :glenohumeral :phase-lag-rad (:set 0.25) :gain-db (:set -4.5))
        (modulate-node :c1-c2-atlas  :dc-offset-rad (:set 0.12) :angular-ceiling-rad (:set 0.01))
        (modulate-node :craniofacial :velocity-gain (:set 0.0)))))

;; 2. Discrete Stair Quantizer (Supports both Concentric Ascent and Eccentric Descent)
(defmodifier modulator-discrete-stairs [gait &key (riser-z-mm 175.0) (tread-x-mm 280.0)]
  ;; Note: Pass positive riser-z-mm for Ascent, negative riser-z-mm for Descent
  (let [ascent-p (> riser-z-mm 0.0)]
    (-> gait
        (modulate-oscillator
          :bpm (:set (if ascent-p 92.0 102.0))
          :duty-factor-l (:set 0.66)
          :duty-factor-r (:set 0.66))
        (modulate-node :tarsal-sagittal
          :stride-length-mm (:set tread-x-mm)
          :step-elevation-z-mm (:set riser-z-mm)
          :swing-clearance-z-mm (:set (+ (abs riser-z-mm) 32.0)))
        (modulate-node :calcaneus
          :strike-topology (:set (if ascent-p :forefoot-to-plantigrade :toe-first-eccentric-drop))
          :attack-ms (:set (if ascent-p 45.0 28.0)))
        (modulate-node :tibiofemoral
          :work-mode (:set (if ascent-p :concentric-lift :eccentric-dissipation))
          :energy-injection-j (:set (* 9.80665 (/ riser-z-mm 1000.0))))
        (modulate-node :c7-t12-thoracic
          :dc-offset-rad (:set (if ascent-p 0.24 0.08))))))

;; 3. Continuous Incline / Decline Ramp Slew
(defmodifier modulator-continuous-slope [gait &key (grade-angle-rad 0.175)]
  (-> gait
      (modulate-plantar-boundary :talocrural-pitch-bias-rad (:add (- grade-angle-rad)))
      (modulate-node :tarsal-sagittal :stride-length-mm (:scale (- 1.0 (* 0.8 (abs grade-angle-rad)))))
      (modulate-node :c7-t12-thoracic :dc-offset-rad (:add (* 1.15 grade-angle-rad)))
      (modulate-node :tibiofemoral
        :work-mode (:set (if (> grade-angle-rad 0.0) :concentric-lift :eccentric-dissipation))
        :stance-yield-rad (:add (if (< grade-angle-rad 0.0) 0.18 0.08)))))

;; 4. Inertial Payload Compensator (Backpack / Anterior Carry / Unilateral Suitcase)
(defmodifier modulator-inertial-payload [gait &key (mass-kg 20.0) (mount :dorsal-pack)]
  (let [load-ratio (/ mass-kg 75.0)]
    (-> gait
        (modulate-oscillator
          :duty-factor-l (:add (* 0.08 load-ratio))
          :duty-factor-r (:add (* 0.08 load-ratio)))
        (modulate-node :tarsal-coronal
          :track-width-mm (:add (* 35.0 load-ratio)))
        (modulate-node :c7-t12-thoracic
          :dc-offset-rad (:add (case mount
                                 (:dorsal-pack    (* +0.45 load-ratio)) ; Lean forward against pack
                                 (:anterior-carry (* -0.25 load-ratio)) ; Lean back against front load
                                 (otherwise       0.0)))
          :lpf-cutoff-hz (:scale (- 1.0 (* 0.5 load-ratio))))           ; Stiffen trunk rotation
        (modulate-node :l4-l5-lumbar
          :coronal-roll-bias-rad (:add (case mount
                                         (:unilateral-left  (* +0.18 load-ratio))
                                         (:unilateral-right (* -0.18 load-ratio))
                                         (otherwise         0.0))))
        (modulate-node :glenohumeral
          :gain-db (:add (if (eq mount :anterior-carry) -24.0 0.0)))))) ; Lock arms if carrying in front

;; 5. Antalgic / Injured Limb Bypass (Unilateral Pain-Avoidance Limp)
(defmodifier modulator-antalgic-limp [gait &key (compromised-side :left) (severity 0.65)]
  (-> gait
      (modulate-oscillator
        :inter-limb-phase-rad (:add (if (eq compromised-side :left) (* -0.45 severity) (* 0.45 severity)))
        :duty-factor-l (:add (if (eq compromised-side :left)  (* -0.16 severity) (* +0.12 severity)))
        :duty-factor-r (:add (if (eq compromised-side :right) (* -0.16 severity) (* +0.12 severity))))
      (modulate-node :l4-l5-lumbar
        :coronal-roll-bias-rad (:set (* 0.16 severity))))) ; Compensatory Trendelenburg lurch
