;; =============================================================================
;; FILE: manifold_handover.lisp
;; DOMAIN: Eccentric/Concentric Bus Transfers & Support Manifold Crossfaders
;; =============================================================================
(defpackage :ortho-dsp.manifolds
  (:use :ortho-dsp.core)
  (:export #:transfer-eccentric-handover
           #:transfer-concentric-handover
           #:sequence-recumbent-cascade))
(in-package :ortho-dsp.manifolds)

;; -----------------------------------------------------------------------------
;; 1. ECCENTRIC MANIFOLD HANDOVER (HIGH-Z POTENTIAL -> LOW-Z FIXTURE CONTACT)
;; Generalizes Stand-to-Sit, Stand-to-Kneel, and Sit-to-Inclined-Rest
;; -----------------------------------------------------------------------------
(defmaneuver transfer-eccentric-handover
  [state &key (source-manifold :plantar-bilateral)
              (target-manifold :ischial-bilateral)
              (descent-duration-ms 1100.0)
              (target-flexion-profile '((:tibiofemoral 1.52) (:coxofemoral 1.48) (:talocrural 0.32)))
              (com-counter-pitch-rad +0.68)
              (contact-threshold-fz 0.15)
              (seat-z-mm 450.0)] ; the seat her eyes see
  (-> state
      ;; Stage A: Controlled eccentric descent with sagittal CoM counterweight
      (trigger-one-shot-envelope
        :mode :eccentric-damping
        :duration-ms descent-duration-ms
        :curve :hermite-soft-landing
        :joint-targets target-flexion-profile
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad com-counter-pitch-rad)
        :appendage-counterweight (:node :glenohumeral :flexion-rad +0.45))

      ;; Stage B: Load-cell comparator triggers normal-force bus crossfade
      (on-contact-threshold
        :sensor-bus target-manifold
        :trigger-fz-ratio contact-threshold-fz
        :max-impact-velocity-mps 0.08

        (crossfade-support-manifold
          :from-bus source-manifold
          :to-bus target-manifold
          :transfer-time-ms 320.0
          :curve :equal-power
          :release-source-zmp true
          :thoracic-settle-rad 0.0)))) ; the spine straightened as the seat takes her

;; -----------------------------------------------------------------------------
;; 2. CONCENTRIC MANIFOLD HANDOVER (LOW-Z FIXTURE -> HIGH-Z BIPEDAL SUPPORT)
;; Generalizes Sit-to-Stand and Kneel-to-Stand via Momentum-Transfer Pre-Load
;; -----------------------------------------------------------------------------
(defmaneuver transfer-concentric-handover
  [state &key (source-manifold :ischial-bilateral)
              (target-manifold :plantar-bilateral)
              (ascent-duration-ms 950.0)
              (pre-pitch-thorax-rad +0.74)
              (liftoff-fz-threshold 0.82)]
  (-> state
      ;; Stage A: Anterior CoM projection over plantar polygon before seat unlatch
      (trigger-one-shot-envelope
        :mode :momentum-preload
        :duration-ms 350.0
        :curve :minimum-jerk-deceleration
        :joint-targets '((:talocrural 0.36))
        :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad pre-pitch-thorax-rad))

      ;; Stage B: Unlatch source manifold when plantar normal force exceeds 82% mg
      (on-contact-threshold
        :sensor-bus target-manifold
        :trigger-fz-ratio liftoff-fz-threshold

        (crossfade-support-manifold
          :from-bus source-manifold
          :to-bus target-manifold
          :transfer-time-ms 180.0
          :curve :linear-fast
          :engage-target-zmp true)

        ;; Stage C: Concentric extensor drive to nominal upright equilibrium
        (trigger-one-shot-envelope
          :mode :concentric-drive
          :duration-ms ascent-duration-ms
          :curve :logarithmic-settle
          :joint-targets '((:tibiofemoral 0.1047) (:coxofemoral 0.0) (:talocrural 0.0524)) ; her stand: knee 6 deg, ankle 3
          :axial-counterweight (:node :c7-t12-thoracic :dc-offset-rad 0.04)))))

;; -----------------------------------------------------------------------------
;; 3. RECUMBENT AXIAL CASCADE (ISCHIAL SEATED <-> DORSAL SUPINE / LATERAL)
;; -----------------------------------------------------------------------------
(defmaneuver sequence-recumbent-cascade
  [state &key (direction :to-supine) ; :to-supine | :to-seated
              (duration-ms 1750.0)
              (bed-yaw-align-rad 0.0)] ; across her 0.9 m bed: her pelvis turns 1.2 rad at most on her soles
  (let [forward-p (eq direction :to-supine)]
    (-> state
        (stage-sequential-manifold
          :duration-ms duration-ms
          :contact-chain (if forward-p
                             [:ischial-bilateral -> :trochanter-lateral -> :scapular-dorsal -> :occipital]
                             [:occipital -> :scapular-dorsal -> :trochanter-lateral -> :ischial-bilateral])
          :torso-pitch-ramp (if forward-p [0.0 -> -1.5708] [-1.5708 -> 0.0])
          :axial-yaw-ramp   (if forward-p [0.0 -> bed-yaw-align-rad] [bed-yaw-align-rad -> 0.0])
          :counterweight-limb-sweep
            (:plantar-fz-target 0.25))))) ; her knees as she sits, her feet on the floor
