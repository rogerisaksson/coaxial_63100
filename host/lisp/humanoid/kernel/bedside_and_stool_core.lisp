;; =============================================================================
;; FILE: bedside_and_stool_core.lisp
;; DOMAIN: Popliteal Tactile Datum, Backless Stool Rim ZCBF, 90-Deg Bed-Edge
;;         Pendulum Sweep & Zero-Translation "Shrimp" Mattress Rolling
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp, floor_groundwork_core.lisp
;; =============================================================================
(defpackage :ortho-dsp.bedside-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.ground-core)
  (:export #:index-popliteal-standoff-datum
           #:enforce-backless-stool-rim-zcbf
           #:drive-transverse-longitudinal-bed-pendulum
           #:drive-centerline-pinned-mattress-roll))
(in-package :ortho-dsp.bedside-core)

;; -----------------------------------------------------------------------------
;; 1. POPLITEAL / CALF TACTILE STANDOFF INDEXER (ZERO-DRIFT BLIND DOCKING)
;; After turning 180 deg with the chair, stool, or bed behind the robot, uses a
;; gentle posterior calf brush (`:shank-crural` dorsal surface) to collapse InEKF
;; standoff uncertainty to < 1.5 mm before initiating knee flexion or toe-vault!
;; -----------------------------------------------------------------------------
(defprimitive index-popliteal-standoff-datum
  [state &key (target-calf-touch-force-n 9.0)
              (max-posterior-seek-mm 110.0)
              (rebound-clearance-mm 12.0)
              (probe-capsules [:shank-crural-l :shank-crural-r])] ; her calves meet an edge under her knee
  (-> state
      (bind-posterior-crural-datum-probe
        :probe-capsules probe-capsules
        :observer-source :generalized-momentum-observer
        :posterior-creep-velocity-mps -0.06
        :max-search-travel-m (* 0.001 max-posterior-seek-mm)
        :tactile-threshold-n target-calf-touch-force-n
        :on-tactile-threshold
          (:zero-inekf-standoff-covariance true
           :latch-fixture-anterior-edge-x true
           :set-standoff-offset-m (* 0.001 rebound-clearance-mm)))))

;; -----------------------------------------------------------------------------
;; 2. BACKLESS STOOL & OVERHEIGHT STOOL RIM ZCBF (PREVENTS REARWARD TOPPLE)
;; Distinguishes a chair (has backrest in `high-thoracic-slab`) from a stool/pall
;; (empty `high-thoracic-slab`). Bounds the posterior thigh/ischial slide within
;; the stool's top ellipse and pitches the lumbar/thoracic spine to self-balance!
;; -----------------------------------------------------------------------------
(defprimitive enforce-backless-stool-rim-zcbf
  [state &key (rim-safety-margin-mm 38.0)
              (auto-detect-missing-backrest true)]
  (-> state
      (inject-elliptical-seat-rim-barrier
        :target-quadric :active-interaction-quadric
        :sliding-patches [:posterior-femoral-bilateral :ischial-bilateral]
        :perimeter-margin-m (* 0.001 rim-safety-margin-mm)
        :backrest-presence-check (if auto-detect-missing-backrest
                                     :verify-high-thoracic-slab-occupancy
                                     :force-backless-stool-mode)
        ;; When no backrest exists (Stool / Pall mode):
        ;; 1. Clamp backward Coulomb slide strictly at quadric centroid X_0.
        ;; 2. Shift 1-Jet `:ischial-seated` thoracic pitch forward so Poinsot's central axis
        ;;    pierces the center of the stool top with zero backrest force - as far as her
        ;;    ischia keep the rim's margin on a 340 mm stool:
        :on-backless-stool-detected
          (:halt-slide-at-quadric-centroid true
           :disable-thoracic-dorsal-admittance-seek true
           :self-balancing-lumbar-pitch-bias-rad +0.045))))

;; -----------------------------------------------------------------------------
;; 3. 90-DEGREE TRANSVERSE <-> LONGITUDINAL BED PENDULUM (L-SWEEP MANEUVER)
;; Couples torso lateral lowering with a 90-deg rotational yaw pivot on the
;; ischial/trochanteric patch and a simultaneous double-pendulum leg sweep:
;;   - `:ingress-to-longitudinal-supine` (Sitting across long side -> Lying along bed)
;;   - `:egress-to-transverse-seated`    (Lying along bed -> Sitting across long side)
;; -----------------------------------------------------------------------------
(defprimitive drive-transverse-longitudinal-bed-pendulum
  [state &key (direction :ingress-to-longitudinal-supine) ; or :egress-to-transverse-seated
              (bed-headboard-side :left)                  ; :left (+90 deg yaw) | :right (-90 deg yaw)
              (pendulum-coupling-gain 0.92)
              (maneuver-duration-ms 1350.0)]
  (let [yaw-delta-rad (case bed-headboard-side (:left +1.570796) (:right -1.570796))
        pivot-arm     (if (eq bed-headboard-side :left) :olecranon-l :olecranon-r)]
    (-> state
        (execute-coupled-so3-cantilever-sweep
          :direction direction
          :target-yaw-rotation-rad (if (eq direction :ingress-to-longitudinal-supine)
                                       yaw-delta-rad
                                       (- yaw-delta-rad))
          :duration-ms maneuver-duration-ms
          ;; A. Torso-Leg Seesaw Counterweight Law (Conserves centroidal angular momentum):
          ;; As trunk pitches down onto `pivot-arm` along the bed's major axis, the pelvis
          ;; tilts to lift both legs clear of the mattress edge without hip-flexor overload!
          :cantilever-seesaw-coupling
            (:fulcrum-patches [:ischial-bilateral :trochanter-gluteal-lateral]
             :upper-body-prop-frame pivot-arm
             :torso-to-crural-momentum-transfer pendulum-coupling-gain
             :leg-clearance-over-mattress-rim-mm 65.0)
          ;; B. Tangential Spin Friction Cone on Mattress (Rotational Coulomb Surfer):
          :rotational-pivot-cone
            (:allow-yaw-slip-on-patches [:ischial-bilateral :trochanter-gluteal-lateral]
             :schlesinger-isomonodromic-transport true
             :align-final-spine-with :bed-quadric-major-axis)))))

;; -----------------------------------------------------------------------------
;; 4. CENTERLINE-PINNED MATTRESS ROLL ("SHRIMP" / HIP-ESCAPE ZERO-DRIFT ROLL)
;; Cancels the non-holonomic lateral displacement (dy = R_torso * d_theta) during
;; Supine -> Side -> Prone -> Side -> Supine rolling on a narrow bed by executing
;; a synchronous contralateral hip/shoulder escape scoot toward the bed centerline!
;; -----------------------------------------------------------------------------
(defprimitive drive-centerline-pinned-mattress-roll
  [state &key (target-roll-state :lateral-side) ; :lateral-side | :prone-stomach | :supine-back
              (roll-chirality :clockwise-right)
              (bed-width-mm 900.0)
              (edge-safety-margin-mm 140.0)]
  (-> state
      (execute-zero-translation-bed-roll
        :target-orientation target-roll-state
        :chirality roll-chirality
        :bed-quadric-slot :active-interaction-quadric
        ;; Hard 2nd-Order ECBF Barrier against rolling off the left/right mattress edges:
        :mattress-drop-off-barrier
          (:bed-width-m (* 0.001 bed-width-mm)
           :min-capsule-to-edge-margin-m (* 0.001 edge-safety-margin-mm))
        ;; Holonomic Phase-Slip ("Shrimp" / Hip-Escape) Counter-Scoot:
        ;; v_y_escape = -R_eff(theta) * d(theta)/dt - k_center * y_offset_from_bed_axis
        :shrimp-counter-translation
          (:enabled true
           :effective-torso-radius-m 0.197      ; her drift onto her side over its quarter turn
           :centerline-restoring-gain 4.5
           ;; Plant contralateral heel + shoulder briefly to scoot pelvis BACK toward
           ;; bed centerline before completing the roll over the ipsilateral shoulder!
           :hip-escape-anchor-frames [:calcaneus-bilateral :glenohumeral-bilateral]
           :pelvic-scoot-stroke-mm 185.0))))
