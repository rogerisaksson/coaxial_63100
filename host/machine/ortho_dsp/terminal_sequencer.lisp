;; =============================================================================
;; FILE: terminal_sequencer.lisp
;; DOMAIN: Composable Postural Macros & Full Round-Trip DMA Schedule
;; =============================================================================
(defpackage :ortho-dsp.terminals
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.modulators
        :ortho-dsp.surfaces
        :ortho-dsp.docking
        :ortho-dsp.manifolds
        :ortho-dsp.substrates
        :ortho-dsp.supervisor)
  (:export #:patch-sit-rigid-chair
           #:patch-supine-mattress-sleep
           #:patch-wake-and-stand
           #:patch-rise-chair
           #:compile-full-lifecycle-soa))
(in-package :ortho-dsp.terminals)

;; Composite 1: Dock and sit on a rigid chair
(defpatch patch-sit-rigid-chair [approach-gait &key (seat-z-mm 450.0)] ; the seat her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1050.0
        :seat-z-mm seat-z-mm)
      (master-anti-spasm-insert)))

;; Composite 2: Dock at bed, sit on mattress edge, roll to supine, and gate off inverters
(defpatch patch-supine-mattress-sleep [approach-gait &key (seat-z-mm 450.0)] ; the bed her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 70.0 :target-yaw-rad 3.14159)
      (substrate-viscoelastic-mattress :eigenfreq-hz 1.80)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1150.0
        :seat-z-mm seat-z-mm)
      (sequence-recumbent-cascade :direction :to-supine :duration-ms 1750.0)
      (gate-inverter-high-z :verify-dwell-ms 400.0)
      (master-anti-spasm-insert)))

;; Composite 3: Precharge bridges from High-Z sleep, roll to seated, stand up, ignite clock
(defpatch patch-wake-and-stand [sleeping-state target-walk-gait]
  (-> sleeping-state
      (wake-inverter-closed-loop)
      (substrate-viscoelastic-mattress :eigenfreq-hz 1.80)
      (sequence-recumbent-cascade :direction :to-seated :duration-ms 1650.0)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (ignite-cyclic-clock target-walk-gait :bars 2)
      (master-anti-spasm-insert)))

;; Composite 4: Precharge from the chair's settled rest and stand up
(defpatch patch-rise-chair [seated-state]
  (-> seated-state
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; COMPLETE LIFECYCLE SCHEDULE: CATWALK -> SLEEP (HIGH-Z) -> WAKE -> CATWALK
;; -----------------------------------------------------------------------------
(defpatch compile-full-lifecycle-soa
  (let [runway-gait (-> nominal-biped-manifold
                        (transducer-elastomer-flat)
                        (modulator-coronal-catwalk :posture-profile :thoracic-kyphotic-bias)
                        (surface-high-mu-planar)
                        (master-anti-spasm-insert))
        bed-sleep   (patch-supine-mattress-sleep runway-gait)
        bed-egress  (patch-wake-and-stand bed-sleep runway-gait)]

    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000
      :schedule
        [(bars 0 6   runway-gait)   ; Approach bed at 114 BPM coronal catwalk
         (oneshot    bed-sleep)     ; Dock -> Sit -> Supine -> High-Z Zero-Torque Standby
         (dwell-ms   10000)         ; 10 seconds passive rest on mattress (0 W actuator draw)
         (oneshot    bed-egress)    ; Precharge -> Sit -> Concentric Stand -> Clock Ignition
         (bars 0 8   runway-gait)]))) ; Resume 114 BPM catwalk
