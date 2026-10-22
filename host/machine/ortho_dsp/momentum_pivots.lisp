;; =============================================================================
;; FILE: momentum_pivots.lisp
;; DOMAIN: Continuous-Momentum Pivots, Horizon Gaze Lock & Corkscrew Spirals
;; =============================================================================
(defpackage :ortho-dsp.pivots
  (:use :ortho-dsp.core)
  (:export #:lock-horizon-gaze
           #:pivot-single-anchor-arc
           #:pivot-cross-under-corkscrew))
(in-package :ortho-dsp.pivots)

;; -----------------------------------------------------------------------------
;; 1. VESTIBULO-OCULAR HORIZON LOCK (ELIMINATES VISUAL POLLING OF TARGET)
;; Decouples head orientation from torso pitch/yaw during terminal approach
;; -----------------------------------------------------------------------------
(defmodifier lock-horizon-gaze [state &key (room-pan-yaw-rad 0.0)]
  (-> state
      (modulate-node :c1-c2-atlas
        :reference-frame (:set :world-horizon)
        :dc-offset-rad   (:set 0.0)            ; Strictly horizontal sightline
        :yaw-target-rad  (:set room-pan-yaw-rad)
        :vestibular-gain (:set 1.00)           ; Full counter-rotation against torso pitch
        :ceiling-rad     (:set 0.015))
      (modulate-node :craniofacial
        :velocity-sensitivity (:set 0.05))))   ; Calm, unhurried facial micro-motorics

;; -----------------------------------------------------------------------------
;; 2. SINGLE-ANCHOR MOMENTUM PIVOT (ZERO-SHUFFLE YAW + OVERLAPPED DESCENT)
;; Redirects forward translational velocity into rotational yaw around stance foot
;; -----------------------------------------------------------------------------
(deftransition pivot-single-anchor-arc
  [source-gait &key (target-yaw-rad 3.14159)
                    (standoff-x-mm 55.0)
                    (sagittal-stagger-mm 95.0) ; Asymmetric heel depth keeps torso upright
                    (coronal-stance-mm 185.0)
                    (eccentric-overlap-rad 0.785) ; Begin Z-drop 45 deg before yaw completion
                    (tactile-calf-datum true)]
  (sync-trigger
    :wait-for :penultimate-single-support
    :anchor-node :calcaneus-r
    :duration-bars 1)

  ;; Convert linear momentum into smooth angular slew without zero-velocity pause
  (redirect-kinetic-vector
    :linear-to-angular-coupling 0.88
    :yaw-slew-curve :minimum-jerk
    :yaw-target-rad target-yaw-rad
    :sweep-foot-node :calcaneus-l
    :terminal-footprint (:standoff-x-mm standoff-x-mm
                         :sagittal-stagger-mm sagittal-stagger-mm
                         :coronal-track-mm coronal-stance-mm)
    :calf-contact-datum (if tactile-calf-datum
                            (:sensor :gastrocnemius-dorsal :target-pressure-kpa 3.5)
                            nil))

  ;; Phase-overlap gate: unlock knee/hip eccentric flexion before yaw completes
  (phase-overlap-trigger
    :monitor-channel :pelvic-yaw-error-rad
    :fire-below-rad eccentric-overlap-rad
    :unlatch-nodes [:tibiofemoral :coxofemoral :talocrural]
    :initial-descent-velocity-mps -0.28))

;; -----------------------------------------------------------------------------
;; 3. CROSS-UNDER CORKSCREW PIVOT (ASYMMETRIC SPIRAL FOR FLOOR-LEVEL TARGETS)
;; Crosses trailing leg behind stance leg to descend to Z <= 200mm with upright torso
;; -----------------------------------------------------------------------------
(deftransition pivot-cross-under-corkscrew
  [source-gait &key (target-yaw-rad 3.14159)
                    (coronal-cross-offset-mm -135.0) ; Negative track = cross-behind curtsy
                    (eccentric-overlap-rad 1.047)]   ; Begin spiral drop 60 deg into turn
  (sync-trigger
    :wait-for :penultimate-single-support
    :anchor-node :calcaneus-r
    :duration-bars 1)

  (redirect-kinetic-vector
    :linear-to-angular-coupling 0.92
    :yaw-slew-curve :helical-spiral
    :yaw-target-rad target-yaw-rad
    :sweep-foot-node :calcaneus-l
    :terminal-footprint (:coronal-track-mm coronal-cross-offset-mm
                         :trailing-foot-supination-rad 0.38 ; Outer edge of foot/shank bears load
                         :sagittal-stagger-mm 160.0))

  (phase-overlap-trigger
    :monitor-channel :pelvic-yaw-error-rad
    :fire-below-rad eccentric-overlap-rad
    :unlatch-nodes [:tibiofemoral :coxofemoral :talocrural :iliac-coronal]
    :spiral-helicity :counter-clockwise))
