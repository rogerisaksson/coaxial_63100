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
           #:transfer-egress-dip
           #:patch-sit-rigid-chair
           #:patch-supine-mattress-sleep
           #:patch-wake-and-stand
           #:patch-rise-chair
           #:compile-full-lifecycle-soa))
(in-package :ortho-dsp.terminals)

;; Sat on a seat over the robot's shins: its fists sought onto it beside the robot's hips, bearing it
;; (the kernel's carpal seat boost)
(defmaneuver transfer-carpal-seat-boost
  [seated-state &key (max-arm-push-share 0.20)]
  (-> seated-state
      (seek-auxiliary-manifold
        :patch-id :carpal-seat-vault
        :anatomical-frames [:carpal-heel-l :carpal-heel-r]
        :surface-normal [0.0 0.0 1.0] :mu 0.65 :bilateral-grasp-pull false
        :weight-law (:wrench-proportional :max-share max-arm-push-share))))

;; Up onto a seat too high for the robot's feet, as the kernel's arm vault (`xform-overheight-arm-vault-ingress`): sat
;; on its front edge, the robot's fists briefly steady it, then `chair-mount` draws its pelvis deep onto the
;; seat's middle, lifts its hands off the seat, abducts its hips, and sets both hands lightly onto mid-thighs
;; while panning full body weight onto `:ischial-bilateral` (`:palmar-carpal-bilateral` 0.0 so `seated.kept` lets go).
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
        :manifold-pan '((:plantar-bilateral 0.0) (:ischial-bilateral 1.0) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.0)))))

;; Down from a seat too high for the robot's feet onto them, as the kernel's egress from it lowers the robot on the
;; robot's arms (`specialize-for-overheight-seat` :egress, `xform-overheight-arm-dip-egress`): its joints
;; play its dismount (moves.lisp `chair-dismount`), its text's rows waiting, till its soles
;; bear it; what bears it then all its soles, its shins 0.44 rad ahead of plumb where
;; its dismount lands them, as the kernel's homotopy pans its seat onto its feet and the
;; robot's wake latches it where its dismount left it. Told plumb ankles instead, the robot's
;; scoot and lean asked its centre of mass 14 cm behind its soles' middle, pulling it back onto the chair's edge or
;; into a step at stand.
(defmaneuver transfer-egress-dip [seated-state]
  (-> seated-state
      (stage-sequential-manifold
        :duration-ms 340.0
        :contact-chain [:ischial-bilateral -> :plantar-bilateral]
        :joint-program :chair-dismount
        :torso-pitch-ramp [0.0 -> 0.0]
        :axial-yaw-ramp [0.0 -> 0.0]
        :counterweight-limb-sweep (:tibiofemoral-flex-rad [1.5708 -> 1.5708] :limb-lift 0.0)
        :talocrural-rad 0.44
        :manifold-pan '((:plantar-bilateral 1.0) (:ischial-bilateral 0.0) (:scapular-dorsal 0.0) (:palmar-carpal-bilateral 0.0)))))

;; Composite 1: Dock and sit on a rigid chair, the robot's hands on it where it stands over the robot's shins, then up
;; onto it where it is too high for the robot's feet - over its popliteal height -, as the kernel's sit morphs
;; by the seat's height (`xform-sit-on-chair-or-stool`). Mounted on the robot's low chair, at its popliteal
;; height, its pelvis went 0.22 m back along its seat as the robot's feet stayed at its edge, the robot's
;; ankles asked the hang tipped its soles 0.27 rad onto their heels and it sat on its sacrum,
;; its soles bearing 0.45 of it.
(defpatch patch-sit-rigid-chair [approach-gait &key (seat-z-mm 450.0) (popliteal-mm 397.0)] ; the seat the robot's eyes see, its knees' height
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159)
      (substrate-rigid-fixture)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1050.0
        :source-residual-weight (if (> seat-z-mm popliteal-mm) 0.55 0.15)
        :seat-z-mm seat-z-mm)
      (transfer-carpal-seat-boost)
      (when-modifier (> seat-z-mm popliteal-mm)
        (specialize-mount-overheight-seat))
      (master-anti-spasm-insert)))

;; Composite 2: Dock at bed, sit on mattress edge, roll to supine, and gate off inverters
(defpatch patch-supine-mattress-sleep [approach-gait &key (seat-z-mm 450.0)] ; the bed the robot's eyes see
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

;; Composite 4: Precharge from the chair's settled rest and stand up - off a seat over the robot's popliteal height
;; down onto its feet on its arms, as the kernel's egress morphs by the seat's height (already on its soles at 0.44
;; rad shins, a 0.56 rad lean clears its palms and capture point over its soles without pitching its hips back onto
;; the seat edge as 0.74 did, whereas 0.28 left its capture point behind its heels); from one its feet reach, where it
;; sits (0.74 rad lean, 0.82 liftoff), an S-curve drive uncurls its trunk in phase with its knees instead of snapping
;; its upright over its heels at liftoff.
(defpatch patch-rise-chair [seated-state &key (seat-z-mm 450.0) (popliteal-mm 397.0)] ; the seat the robot's eyes see, its knees' height
  (-> seated-state
      (when-modifier (> seat-z-mm popliteal-mm)
        (transfer-egress-dip))
      (wake-inverter-closed-loop)
      (transfer-concentric-handover
        :source-manifold :ischial-bilateral
        :target-manifold :plantar-bilateral
        :ascent-duration-ms 950.0
        :ascent-curve :hermite-s-curve
        :pre-pitch-thorax-rad (if (> seat-z-mm popliteal-mm) 0.56 0.74)
        :liftoff-fz-threshold (if (> seat-z-mm popliteal-mm) 0.65 0.82))
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
