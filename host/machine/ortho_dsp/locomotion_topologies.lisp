;; =============================================================================
;; FILE: locomotion_topologies.lisp
;; DOMAIN: Fundamental Phase & CoM Engines (Walk, Run, Crouch, Skip, Hop, Retro, Lateral)
;; =============================================================================
(defpackage :ortho-dsp.topologies
  (:use :ortho-dsp.core)
  (:export #:topology-pendular-walk
           #:topology-ballistic-slip-run
           #:topology-level-com-stealth
           #:topology-asymmetric-gallop
           #:topology-synchronous-hop
           #:topology-retro-locomotion
           #:topology-coronal-sidestep))
(in-package :ortho-dsp.topologies)

;; 1. Nominal Pendular Walk (Out-of-phase PE/KE exchange, double-support present)
(defmodifier topology-pendular-walk [gait &key (bpm 110.0) (stride-mm 640.0)]
  (-> gait
      (modulate-oscillator
        :bpm (:set bpm)
        :inter-limb-phase-rad (:set 3.14159)
        :duty-factor-l (:set 0.62)
        :duty-factor-r (:set 0.62))
      (modulate-node :com-dynamics :topology (:set :inverted-pendulum))
      (modulate-node :tarsal-sagittal :stride-length-mm (:set stride-mm))))

;; 2. Ballistic SLIP Run / Sprint (In-phase elastic energy storage, duty < 0.50)
(defmodifier topology-ballistic-slip-run [gait &key (bpm 164.0) (stride-mm 1220.0)]
  (let [duty (if (> bpm 185.0) 0.28 0.36)] ; Shorter ground contact during sprinting
    (-> gait
        (modulate-oscillator
          :bpm (:set bpm)
          :inter-limb-phase-rad (:set 3.14159)
          :duty-factor-l (:set duty)
          :duty-factor-r (:set duty))
        (modulate-node :com-dynamics
          :topology (:set :ballistic-slip)
          :vertical-oscillation-mm (:set 62.0)
          :damping-ratio (:set 0.15))
        (modulate-node :calcaneus
          :strike-topology (:set (if (> bpm 185.0) :forefoot-digitigrade :midfoot-elastic))
          :attack-ms (:set 14.0)
          :sustain (:set 0.0))
        (modulate-node :tibiofemoral
          :stance-yield-rad (:set 0.48)
          :compliance (:set 0.92)
          :rebound-q (:set 2.45)
          :work-mode (:set :elastic-storage))
        (modulate-node :tarsal-sagittal
          :stride-length-mm (:set stride-mm)
          :swing-clearance-z-mm (:set 155.0))
        (modulate-node :c7-t12-thoracic :dc-offset-rad (:set 0.18) :lpf-cutoff-hz (:set 4.5))
        (modulate-node :glenohumeral    :gain-db (:set 4.5) :elbow-flexion-rad (:set 1.48))
        (modulate-node :c1-c2-atlas     :vor-lock (:set true)))))

;; 3. Level-CoM Crouch / Stealth Walk (Groucho Mechanics: Zero Vertical Bounce)
(defmodifier topology-level-com-stealth [gait &key (crouch-depth-mm 165.0)]
  (-> gait
      (modulate-oscillator
        :bpm (:set 84.0)
        :duty-factor-l (:set 0.72)         ; Extended double-support for silent weight transfer
        :duty-factor-r (:set 0.72))
      (modulate-node :com-dynamics
        :topology (:set :constant-height-groucho)
        :base-height-z-mm (:add (- crouch-depth-mm))
        :vertical-oscillation-mm (:set 1.5) ; Flat-line vertical trajectory
        :damping-ratio (:set 2.0))          ; Overdamped
      (modulate-node :calcaneus
        :strike-topology (:set :forefoot-to-plantigrade)
        :attack-ms (:set 95.0)             ; Ultra-slow silent transient
        :decay-ms (:set 250.0))
      (modulate-node :tibiofemoral
        :stance-yield-rad (:set 0.68)      ; Continuous deep knee flexion
        :compliance (:set 0.95))
      (modulate-node :c7-t12-thoracic
        :dc-offset-rad (:set 0.28)         ; Forward torso pitch to keep CoM centered
        :lpf-cutoff-hz (:set 1.5))))

;; 4. Asymmetric Gallop / Skip (Broken Phase Symmetry: Delta-Phi = 0.62*pi)
(defmodifier topology-asymmetric-gallop [gait &key (lead-side :right)]
  (-> gait
      (modulate-oscillator
        :bpm (:set 138.0)
        :inter-limb-phase-rad (:set (if (eq lead-side :right) 1.95 4.33))
        :duty-factor-l (:set 0.42)
        :duty-factor-r (:set 0.45))
      (modulate-node :com-dynamics
        :topology (:set :double-peak-slip)
        :vertical-oscillation-mm (:set 54.0))
      (modulate-node :tibiofemoral
        :rebound-q (:set 1.85))))

;; 5. Synchronous Bipedal Hop / Kangoo Bounce (In-Phase Lock: Delta-Phi = 0.0)
(defmodifier topology-synchronous-hop [gait &key (bpm 126.0) (stride-mm 450.0)]
  (-> gait
      (modulate-oscillator
        :bpm (:set bpm)
        :inter-limb-phase-rad (:set 0.0)   ; Both legs fire simultaneously
        :duty-factor-l (:set 0.38)
        :duty-factor-r (:set 0.38))
      (modulate-node :com-dynamics
        :topology (:set :ballistic-slip)
        :vertical-oscillation-mm (:set 85.0))
      (modulate-node :tarsal-coronal
        :track-width-mm (:clamp-min 140.0)) ; Widen stance for synchronous landing stability
      (modulate-node :tarsal-sagittal
        :stride-length-mm (:set stride-mm))
      (modulate-node :glenohumeral
        :phase-shift-rad (:set 0.0)        ; Arms swing synchronously in-phase with jump
        :gain-db (:set 3.0))))

;; 6. Retro-Locomotion (Backward Walk: Time-Reversed Plantar Roll & Hip Extension Drive)
(defmodifier topology-retro-locomotion [gait]
  (-> gait
      (modulate-oscillator
        :bpm (:scale 0.85)
        :direction-angle-rad (:set 3.14159)
        :duty-factor-l (:add 0.06)
        :duty-factor-r (:add 0.06))
      (modulate-node :calcaneus
        :strike-topology (:set :toe-to-heel-reverse)
        :attack-ms (:set 45.0))
      (modulate-node :tarsal-sagittal
        :stride-length-mm (:scale 0.68))
      (modulate-node :c7-t12-thoracic
        :dc-offset-rad (:set -0.06))))     ; Slight backward lean to align CoM over rear foot

;; 7. Coronal Sidestep & Carioca Crossover Weave
(defmodifier topology-coronal-sidestep [gait &key (direction :left) (carioca-weave false)]
  (-> gait
      (modulate-oscillator
        :direction-angle-rad (:set (if (eq direction :left) 1.5708 -1.5708))
        :inter-limb-phase-rad (:set 3.14159))
      (modulate-node :tarsal-coronal
        :track-width-mm (if carioca-weave (:set -35.0) (:set 220.0))
        :crossover-weave (:set carioca-weave)
        :swing-circumduction-mm (if carioca-weave (:set 85.0) (:set 0.0)))
      (modulate-node :iliac-coronal
        :waveform (:set (if carioca-weave :transverse-figure-eight :sawtooth)))))
