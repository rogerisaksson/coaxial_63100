;; =============================================================================
;; FILE: 06_transport_docking.lisp
;; DOMAIN: Phase-Locked Deceleration, Terminal Foot Placement & Cycle Re-Ignition
;; =============================================================================
(defpackage :ortho-dsp.docking
  (:use :ortho-dsp.core)
  (:export #:dock-terminal-stance
           #:ignite-cyclic-clock))
(in-package :ortho-dsp.docking)

;; -----------------------------------------------------------------------------
;; 1. PHASE-LOCKED BRAKING (CYCLIC LOCOMOTION -> STATIC DOUBLE-SUPPORT PARK)
;; -----------------------------------------------------------------------------
(deftransition dock-terminal-stance [source-gait &key (standoff-x-mm 80.0)
                                                      (target-yaw-rad 3.14159)
                                                      (stance-width-mm 210.0)
                                                      (bars 2)]
  (sync-trigger
    :wait-for :double-support-zero-crossing
    :lock-zmp-margin-mm 22.0
    :duration-bars bars)

  ;; Slew master oscillator to 0.0 Hz and duty factor to 1.0 (continuous stance)
  (pll-clock-slew
    :curve :logarithmic-brake
    :bpm-ramp  [(get-bpm source-gait)  -> 0.0]
    :duty-ramp [(get-duty source-gait) -> 1.00]
    :window-bars [0.0 -> bars])

  ;; Align calcaneal nodes parallel to target fixture boundary
  (align-terminal-footprint
    :target-yaw-rad target-yaw-rad
    :sagittal-standoff-mm standoff-x-mm
    :coronal-track-mm (:set stance-width-mm)
    :swing-circumduction-mm (:set 0.0)
    :lock-plantigrade true))

;; -----------------------------------------------------------------------------
;; 2. MASTER CLOCK RE-IGNITION (STATIC DOUBLE-SUPPORT -> CYCLIC LOCOMOTION)
;; -----------------------------------------------------------------------------
(deftransition ignite-cyclic-clock [static-state target-gait &key (bars 2)]
  (sync-trigger
    :wait-for :plantar-zmp-centered
    :lock-zmp-margin-mm 18.0
    :duration-bars bars)

  ;; Pre-shift CoM onto initial stance foot before unlatching contralateral swing
  (initiate-weight-shift
    :initial-stance-node :calcaneus-l
    :coronal-com-shift-mm (* 0.45 (get-param target-gait :track-width-mm))
    :lead-time-ms 220.0)

  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp  [0.0  -> (get-bpm target-gait)]
    :duty-ramp [1.00 -> (get-duty target-gait)]
    :window-bars [0.0 -> bars]))
