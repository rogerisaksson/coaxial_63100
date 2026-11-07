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
  (:export #:transfer-carpal-seat-boost
           #:specialize-mount-overheight-seat
           #:transfer-egress-scoot
           #:patch-sit-rigid-chair
           #:patch-supine-mattress-sleep
           #:patch-wake-and-stand
           #:patch-rise-chair
           #:compile-full-lifecycle-soa))
(in-package :ortho-dsp.terminals)

;; Sat on a seat over her shins: her fists sought onto it beside her hips, bearing her (the kernel's carpal seat boost)
(defmaneuver transfer-carpal-seat-boost
  [seated-state &key (max-arm-push-share 0.20)]
  (-> seated-state
      (seek-auxiliary-manifold
        :patch-id :carpal-seat-vault
        :anatomical-frames [:carpal-heel-l :carpal-heel-r]
        :surface-normal [0.0 0.0 1.0] :mu 0.65 :bilateral-grasp-pull false
        :weight-law (:wrench-proportional :max-share max-arm-push-share))))

;; Up onto a seat too high for her feet, as the kernel's arm vault (`xform-overheight-arm-vault-ingress`): sat on its
;; front edge, her fists on it beside her hips, her joints play her way up onto it (`:joint-program`, moves.lisp's
;; `chair-mount`) - her shoulders flexed against her fists vault her pelvis off the edge and draw it back on
;; her arms, then let her down onto the seat's middle -; her law then holds her there, the seat and her fists bearing
;; her - her fists kept where they bore as her hips went back past them (`seated.kept`) -, her knees and ankles asked
;; the kernel's hang (`specialize-posture-dangling-crural-hang`'s origin). Left to the sit's own targets her arms drew
;; her back to its front edge (0.05 m behind its middle to 0.16 ahead in 5 s); her arms let go to their rest, her trunk
;; sank 85 -> 57 deg in 12 s.
(defmaneuver specialize-mount-overheight-seat [seated-state]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms 1000.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :joint-program :chair-mount
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.5708 -> 1.5708] :limb-lift 0.0)
        :talocrural-rad -0.24
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.80) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.20)))))

;; Down from a seat too high for her feet to its front edge, as the kernel's arm-dip egress scoots her
;; (`xform-overheight-arm-dip-egress`): her legs let go, her fists boosting her, her seat slid ahead at the kernel's
;; tangent speed - her sled's twist in her closed chains' step, as far as her soles do not yet bear what her legs
;; weigh - till they bear it, then those soles told it and her slide over: the sit-to-stand take's start, her feet on
;; the floor under her knees. Risen from the seat's middle her soles bore 0 N through her lean and thrust: locked 5.2 s;
;; stroked open on her arms and knees, a puppet slid 0.29 m a stroke past the edge onto the floor.
(defmaneuver transfer-egress-scoot [seated-state &key (slide-speed-mps 0.15) (reach-fz-ratio 0.15)]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms 350.0
        :contact-chain [:palmar-carpal-bilateral -> :ischial-bilateral]
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.5708 -> 1.5708] :limb-lift 0.0)
        :talocrural-rad 0.0
        :slide-speed-mps slide-speed-mps
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 0.55) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.45)))
      (on-contact-threshold
        :sensor-bus :plantar-bilateral
        :trigger-fz-ratio reach-fz-ratio
        (stage-sequential-manifold
          :duration-ms 350.0
          :contact-chain [:ischial-bilateral -> :plantar-bilateral]
          :torso-pitch-ramp [0.0 -> 0.0]
          :axial-yaw-ramp [0.0 -> 0.0]
          :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.5708 -> 1.5708] :limb-lift 0.0)
          :talocrural-rad 0.0
          :slide-speed-mps 0.0
          :manifold-pan '((:plantar-bilateral 0.15) (:ischial-bilateral 0.65) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.20))))))

;; Composite 1: Dock and sit on a rigid chair, her hands on it where it stands over her shins, then up onto it
(defpatch patch-sit-rigid-chair [approach-gait &key (seat-z-mm 450.0)] ; the seat her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1050.0
        :seat-z-mm seat-z-mm)
      (transfer-carpal-seat-boost)
      (specialize-mount-overheight-seat)
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
      (transfer-egress-scoot)
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
