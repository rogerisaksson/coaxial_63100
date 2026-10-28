;; =============================================================================
;; FILE: core_manifold.lisp
;; DOMAIN: Physical Invariants, Universal Phase Oscillator & Baseline Manifold
;; =============================================================================
(defpackage :ortho-dsp.core
  (:export #:physical-invariants
           #:nominal-biped-manifold))
(in-package :ortho-dsp.core)

(defconstants physical-invariants
  :gravity-mps2             9.80665
  :nominal-foot-width-mm    96.5     ; Derived: 103.0mm track - 6.5mm inner clearance
  :min-coronal-clearance-mm 6.5      ; Hard medial malleolus collision guard
  :min-normal-force-ratio   0.35     ; Fz >= 0.35*m*g (prevents vertical liftoff during stance)
  :min-zmp-margin-mm        15.0)    ; Minimum distance to support polygon boundary

(defstate nominal-biped-manifold
  (:oscillator
    :bpm 110.0                       ; Master cycle clock (1.8333 Hz)
    :inter-limb-phase-rad 3.14159    ; pi = alternating | 0.0 = synchronous hop | ~2.0 = gallop
    :duty-factor-l 0.62              ; > 0.50 = double support | < 0.50 = aerial flight phase
    :duty-factor-r 0.62              ; Independent R/L duty allows limping / asymmetric gaits
    :direction-angle-rad 0.0         ; 0 = forward | pi = retro-walk | +/-pi/2 = lateral step
    :yaw-rate-radps 0.0              ; Curvilinear turning rate
    :even-step-swing-offset 0.0      ; Groove / temporal asymmetry
    :phase-jitter-ms 0.0)

  (:plantar-boundary
    :sole-thickness-z-mm 0.0
    :foot-collision-w-mm 96.5
    :talocrural-pitch-bias-rad 0.0
    :support-polygon-x-mm 240.0
    :windlass-arch-q 1.0
    :metatarsal-splay-gain 1.0)

  (:kinematics
    :com-dynamics    (:topology :inverted-pendulum :base-height-z-mm 920.0
                      :vertical-oscillation-mm 22.7 :damping-ratio 0.707)
    :calcaneus       (:strike-topology :heel-to-toe :saturation :linear
                      :attack-ms 35.0 :decay-ms 120.0 :sustain 0.40)
    :tibiofemoral    (:stance-yield-rad 0.24 :compliance 0.80 :rebound-q 0.50
                      :work-mode :isometric-vault :energy-injection-j 0.0)
    :tarsal-coronal  (:track-width-mm 103.0 :swing-circumduction-mm 0.0 :crossover-weave false)
    :tarsal-sagittal (:stride-length-mm 640.0 :step-elevation-z-mm 0.0 :swing-clearance-z-mm 38.0)
    :iliac-coronal   (:lfo-amplitude-rad 0.065 :waveform :triangle :stereo-width 0.35)
    :l4-l5-lumbar    (:dc-offset-rad -0.14 :q-factor 0.40 :coronal-roll-bias-rad 0.0)
    :c7-t12-thoracic (:dc-offset-rad 0.04 :lpf-cutoff-hz 8.0 :lpf-slope-db 6)
    :glenohumeral    (:phase-shift-rad 3.14159 :gain-db -3.0 :phase-lag-rad 0.0
                      :abduction-rad 0.05 :elbow-flexion-rad 0.35)
    :c1-c2-atlas     (:dc-offset-rad 0.0 :angular-ceiling-rad 0.12 :vor-lock false)
    :craniofacial    (:velocity-gain 0.80)))
