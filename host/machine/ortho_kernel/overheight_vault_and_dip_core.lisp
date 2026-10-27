;; =============================================================================
;; FILE: overheight_vault_and_dip_core.lisp
;; DOMAIN: 1-Jet Dip-Suspension Posture, Two-Body Stool Tipping Cone, Capstan
;;         Bevel Unweighting, Carpal Hand-Walk Ratchet & Overhang Dip Descent
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp, bedside_and_stool_core.lisp
;; =============================================================================
(defpackage :ortho-dsp.high-vault-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.bedside-core)
  (:export #:high-vault-1jet-posture-atlas
           #:enforce-fixture-overturning-cone
           #:drive-capstan-bevel-arm-vault
           #:ratchet-carpal-seat-replant
           #:drive-anterior-overhang-dip-descent))
(in-package :ortho-dsp.high-vault-core)

;; -----------------------------------------------------------------------------
;; 1. 1-JET POSTURE ATLAS EXTENSION: UPPER-BODY DIP & L-VAULT SUSPENSION
;; Defines the canonical posture and 3-column modal synergy when 70-100% of body
;; weight is suspended on the hands/forearms during high-stool mounting/dismounting.
;; -----------------------------------------------------------------------------
(defstate high-vault-1jet-posture-atlas
  (:postures
    (:posture :carpal-dip-suspension
      :coupled-manifold [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.52       ; Point toes down in active equinus readiness
         :tibiofemoral 0.95 :coxofemoral 1.15    ; Compact L-tuck so thighs clear high seat bevel!
         :iliac-coronal 0.00 :l4-l5-lumbar +0.16 :c7-t12-thoracic +0.22
         :glenohumeral -0.45 :glenohumeral-abduct 0.24 :cubital 0.75
         :c1-c2-atlas -0.15)
      :modal-basis-columns
        ;; 1. Axial Compression: Directly drives vertical Triceps/Latissimus Dip height!
        ;;    (+1.0 = deep elbow flexion lowering pelvis; -1.0 = full lockout lifting pelvis high)
        (:mode-axial-compression (:glenohumeral -0.62 :cubital +1.15
                                  :coxofemoral +0.42 :tibiofemoral +0.35 :l4-l5-lumbar +0.18)
         ;; 2. Sagittal Shift: Rows the suspended pelvis posteriorly (-X) over the seat edge
         ;;    or anteriorly (+X) clear of the seat edge by pivoting around the wrists/shoulders!
         :mode-sagittal-shift    (:glenohumeral +0.78 :cubital -0.28
                                  :coxofemoral -0.45 :l4-l5-lumbar -0.22 :c1-c2-atlas -0.35)
         ;; 3. Pitch Attitude: Tilts torso forward to keep CoM inside stool base polygon:
         :mode-pitch-attitude    (:coxofemoral +0.55 :l4-l5-lumbar +0.35 :c7-t12-thoracic +0.30
                                  :c1-c2-atlas -0.80))
      ;; When feet are dangling in mid-air during the vault/dip, decouple lower legs from
      ;; standing-squat constraints while keeping toes primed for floor capture:
      :gravity-hang-law
        (:enabled true
         :trigger-on-unloaded-patch [:plantar-bilateral :metatarsal-equinus-bilateral]
         :dangling-q0 (:talocrural -0.48 :tibiofemoral 0.85 :coxofemoral 1.05)
         :stiffness-atten-db -18.0))))

;; -----------------------------------------------------------------------------
;; 2. TWO-BODY FIXTURE OVERTURNING & SLIP WRENCH CONE BARRIER
;; Guarantees that pulling/pushing on a light or tall stool never tips the stool
;; over or slides it out from under the robot!
;; Constraints net reaction wrench applied ON the stool to lie strictly inside
;; the stool's own floor-support polygon and static friction cone.
;; -----------------------------------------------------------------------------
(defprimitive enforce-fixture-overturning-cone
  [state &key (fixture-quadric-slot :active-interaction-quadric)
              (min-stool-zmp-margin-mm 28.0)
              (vertical-preload-boost-n 140.0)]
  (-> state
      (inject-two-body-stability-barrier
        :object-slot fixture-quadric-slot
        :active-robot-patches [:ulnar-carpal-shove-bilateral
                               :palmar-carpal-bilateral
                               :posterior-femoral-bilateral
                               :ischial-bilateral]
        :object-floor-zmp-margin-m (* 0.001 min-stool-zmp-margin-mm)
        ;; Self-Stabilizing Preload Law:
        ;; Before applying horizontal rowing shear F_x to pull the pelvis onto the stool,
        ;; force the hands to press DOWNWARD (+F_z >= 140 N) near the stool's centroidal
        ;; axis, pinning the stool's four legs firmly to the floor!
        :axial-pinning-law
          (:min-downward-hand-fz-before-shear-n vertical-preload-boost-n
           :max-shear-to-normal-ratio-on-fixture 0.38
           :abort-to-plantar-if-fixture-tilts-rad 0.035))))

;; -----------------------------------------------------------------------------
;; 3. CAPSTAN BEVEL-UNWEIGHTING ARM VAULT (PULL/PUSH PAST SEAT EDGE IN PLACE)
;; Solves the edge-snag problem when mounting an overheight chair/stool:
;; As the thighs/buttocks cross the front edge coordinate s = x_ischial - x_bevel,
;; attenuates thigh/buttock normal force via a Gaussian unweighting window and
;; transfers 70-90% of body weight to the arms (`:carpal-dip-suspension`) while
;; rowing the pelvis up and backward past the edge!
;; -----------------------------------------------------------------------------
(defprimitive drive-capstan-bevel-arm-vault
  [state &key (bevel-unweight-fraction 0.82)     ; Unload 82% of thigh normal force at the sharp edge!
              (bevel-half-width-mm 65.0)
              (vault-clear-lift-mm 31.0)                 ; her lockout from the dip's origin
              (rowing-velocity-mps -0.18)]
  (-> state
      (execute-bevel-arch-vault-homotopy
        :hand-anchor-patches [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :pelvic-sled-patches [:posterior-femoral-bilateral :ischial-bilateral]
        :bevel-transition-halfwidth-m (* 0.001 bevel-half-width-mm)
        ;; Capstan Unweighting Curve: Fz_sled_max(s) = m*g * (1 - alpha * exp(-s^2 / 2*sigma^2))
        :capstan-edge-unweighting-alpha bevel-unweight-fraction
        :vertical-arch-clearance-m (* 0.001 vault-clear-lift-mm)
        :posterior-rowing-velocity-mps rowing-velocity-mps
        ;; Synchronize 1-Jet modal channels during the vault stroke:
        ;; Compress axial mode (-0.85 = elbow lockout lift) + negative sagittal shift (row back)
        :coupled-1jet-vault-trajectory
          (:atlas 'high-vault-1jet-posture-atlas
           :lockout-axial-command -0.72                ; her elbow's -5 deg stop
           :posterior-row-sagittal-command -0.75
           :schlesinger-isomonodromic-transport true))))

;; -----------------------------------------------------------------------------
;; 4. CARPAL SEAT-WALK RATCHET (MULTI-STROKE HAND REPLANT FOR DEEP/HIGH SEATS)
;; Once the initial arm-vault clears the front bevel, temporarily settles weight
;; onto the posterior thighs/buttocks, walks the hands 180mm further back on the
;; seat (alternating left/right or bimanual), and executes a centering scoot.
;; -----------------------------------------------------------------------------
(defprimitive ratchet-carpal-seat-replant
  [state &key (replant-mode :alternating-unilateral) ; :alternating-unilateral (safer on narrow stool) | :bimanual-sync
              (hand-step-back-mm 175.0)
              (stop-at-seat-centroid true)]
  (-> state
      (execute-appendage-replant-ratchet
        :replant-topology replant-mode
        :hand-patches [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :temporary-holding-patches [:posterior-femoral-bilateral :ischial-bilateral]
        :replant-stride-x-m (* -0.001 hand-step-back-mm)
        :swing-hand-clearance-z-m 0.035
        :follow-up-scoot-to-centroid stop-at-seat-centroid)))

;; -----------------------------------------------------------------------------
;; 5. ANTERIOR OVERHANG DIP & TOE-SEEK DESCENT (DISMOUNT FROM HIGH SEAT/STOOL)
;; Safely lowers the robot from a high seat where feet cannot reach the floor:
;;   1) Anchors hands on the front/sides of the seat and scoots pelvis forward
;;      until buttocks clear the front edge (s <= -50 mm).
;;   2) Suspends 80-95% of weight on the arms and performs a controlled eccentric
;;      triceps/shoulder dip (`mode_axial_compression`: -0.6 -> +0.95), lowering the
;;      pelvis vertically IN FRONT OF the stool face while dorsal thighs lightly skim
;;      the front bevel for sagittal pendulum damping.
;;   3) When reachability `rho_plantar` drops below 0.96, catches weight on
;;      plantarflexed forefeet (`:metatarsal-equinus`) and unlatches hands!
;; -----------------------------------------------------------------------------
(defprimitive drive-anterior-overhang-dip-descent
  [state &key (overhang-clearance-x-mm 55.0)     ; Clear buttocks 55mm in front of seat edge before lowering
              (max-dip-lowering-z-mm 167.0)      ; Up to 235mm vertical arm-lowering travel! Hers 167
              (descent-velocity-mps -0.16)
              (skim-dorsal-thigh-on-bevel true)]
  (-> state
      (execute-overhang-triceps-dip-egress
        :hand-anchor-patches [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :ground-catch-patches [:metatarsal-equinus-bilateral :plantar-bilateral]
        :anterior-bevel-overhang-m (* 0.001 overhang-clearance-x-mm)
        :max-vertical-dip-stroke-m (* 0.001 max-dip-lowering-z-mm)
        :eccentric-descent-rate-mps descent-velocity-mps
        ;; Keep a light 15-25 N normal skim between posterior thighs/calves and the
        ;; front edge of the stool during the dip so the body never swings outward like a pendulum!
        :posterior-bevel-stabilizer-skim
          (:enabled skim-dorsal-thigh-on-bevel
           :skim-patches [:posterior-femoral-bilateral]
           :target-normal-force-n 20.0
           :kinetic-coulomb-damping true)
        ;; Hand-to-Foot Wrench Homotopy Trigger:
        ;; Do NOT release hands from the stool until the lowered forefoot confirms Fz >= 0.28*m*g!
        :ground-capture-handover
          (:require-reachability-rho-below 0.96
           :require-forefoot-fz-ratio 0.28
           :handover-duration-ms 280.0))))
