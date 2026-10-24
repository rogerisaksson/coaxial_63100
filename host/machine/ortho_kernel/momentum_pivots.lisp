;; =============================================================================
;; FILE: momentum_pivots.lisp
;; DOMAIN: Continuous-Momentum Pivots, Horizon Gaze Lock & Clock Re-Ignition
;; =============================================================================
(defpackage :ortho-dsp.pivots
  (:use :ortho-dsp.core)
  (:export #:lock-horizon-gaze
           #:pivot-single-anchor-arc
           #:pivot-cross-under-corkscrew
           #:ignite-cyclic-clock
           #:release-impedance-wavefront
           #:wake-inverter-closed-loop))
(in-package :ortho-dsp.pivots)

(defmodifier lock-horizon-gaze [state &key (room-pan-yaw-rad 0.0)]
  (-> state
      (modulate-node :c1-c2-atlas
        :reference-frame (:set :world-horizon)
        :dc-offset-rad   (:set 0.0)
        :yaw-target-rad  (:set room-pan-yaw-rad)
        :vestibular-gain (:set 1.00)
        :ceiling-rad     (:set 0.015))
      (modulate-node :craniofacial
        :velocity-sensitivity (:set 0.05))))

(deftransition pivot-single-anchor-arc
  [source-gait &key (target-yaw-rad 3.14159)
                    (standoff-x-mm 55.0)
                    (sagittal-stagger-mm 95.0)
                    (coronal-stance-mm 185.0)
                    (eccentric-overlap-rad 0.785)
                    (tactile-calf-datum true)]
  (sync-trigger
    :wait-for :penultimate-single-support
    :anchor-node :calcaneus-r
    :duration-bars 1)
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
  (phase-overlap-trigger
    :monitor-channel :pelvic-yaw-error-rad
    :fire-below-rad eccentric-overlap-rad
    :unlatch-nodes [:tibiofemoral :coxofemoral :talocrural]
    :initial-descent-velocity-mps -0.28))

(deftransition pivot-cross-under-corkscrew
  [source-gait &key (target-yaw-rad 3.14159)
                    (coronal-cross-offset-mm -135.0)
                    (eccentric-overlap-rad 1.047)]
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
                         :trailing-foot-supination-rad 0.38
                         :sagittal-stagger-mm 160.0))
  (phase-overlap-trigger
    :monitor-channel :pelvic-yaw-error-rad
    :fire-below-rad eccentric-overlap-rad
    :unlatch-nodes [:tibiofemoral :coxofemoral :talocrural :iliac-coronal]
    :spiral-helicity :counter-clockwise))

(deftransition ignite-cyclic-clock [static-state target-gait &key (bars 2)]
  (sync-trigger
    :wait-for :plantar-zmp-centered
    :lock-zmp-margin-mm 18.0
    :duration-bars bars)
  (initiate-weight-shift
    :initial-stance-node :calcaneus-l
    :coronal-com-shift-mm (* 0.45 (get-param target-gait :track-width-mm))
    :lead-time-ms 220.0)
  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp  [0.0  -> (get-bpm target-gait)]
    :duty-ramp [1.00 -> (get-duty target-gait)]
    :window-bars [0.0 -> bars]))

(defmodifier release-impedance-wavefront
  [state &key (terminal-mode :seated-active-rest) (wave-speed-ms-per-node 55.0)]
  (-> state
      (cascade-impedance-envelope
        :propagation-order [:calcaneus :talocrural :tibiofemoral :coxofemoral
                            :l4-l5-lumbar :c7-t12-thoracic :glenohumeral :c1-c2-atlas]
        :node-stagger-ms wave-speed-ms-per-node
        :damping-ratio-ramp [0.707 -> 0.95]
        :stiffness-atten-db (case terminal-mode
                              (:seated-active-rest -26.0)
                              (:supine-high-z      -inf))
        :inverter-gate-mode (case terminal-mode
                              (:seated-active-rest :low-bias-gravity-comp)
                              (:supine-high-z      :high-z-passive-bypass)))))

(defmodifier wake-inverter-closed-loop [state]
  (-> state
      (latch-current-encoders :set-as-zero-error-reference true)
      (cascade-actuator-gates
        :direction :proximal-to-distal
        :nodes [:l4-l5-lumbar :c7-t12-thoracic :coxofemoral :tibiofemoral
                :calcaneus :glenohumeral :c1-c2-atlas :craniofacial]
        :stagger-ms 50.0
        :torque-ramp-ms 250.0
        :bridge-state :pwm-active-closed-loop
        :master-clock :armed)))
