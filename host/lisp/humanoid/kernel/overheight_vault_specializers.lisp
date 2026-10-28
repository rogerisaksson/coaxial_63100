;; =============================================================================
;; FILE: overheight_vault_specializers.lisp
;; DOMAIN: Composable Transformers & Specializers for High Stools, Chairs & Benches
;; REQUIRES: overheight_vault_and_dip_core.lisp
;; =============================================================================
(defpackage :ortho-dsp.high-vault-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.postures-core
        :ortho-dsp.posture-xforms
        :ortho-dsp.bedside-core
        :ortho-dsp.bedside-xforms
        :ortho-dsp.high-vault-core)
  (:export ;; A. General In-Place Arm-Vault Ingress & Overhang-Dip Egress
           #:xform-overheight-arm-vault-ingress
           #:xform-overheight-arm-dip-egress
           ;; B. Composable Grip, Geometry & Contingency Specializers
           #:specialize-vault-hand-anchor-topology
           #:specialize-vault-with-unilateral-leg-hook
           #:specialize-vault-for-light-tippy-stool
           #:specialize-dip-egress-with-footrest-rung
           #:specialize-vault-abort-to-plantar-drop))
(in-package :ortho-dsp.high-vault-xforms)

;; =============================================================================
;; A. GENERAL IN-PLACE ARM-VAULT INGRESS & OVERHANG-DIP EGRESS
;; =============================================================================

;; 1. Universal High-Seat / High-Stool Arm-Haul Ingress (In-Place Pull/Push Past Edge)
;; - Automatically indexes calf distance, plants hands on seat/rails, pins the stool's
;;   stability cone, rises onto toes, vaults pelvis up and over the front bevel on
;;   arm support (unweighting thigh Capstan friction), walks hands back if needed,
;;   and settles into balanced seated posture with dangling legs in Null-Space!
(defmodifier xform-overheight-arm-vault-ingress
  [state &key (bevel-unweight-ratio 0.82) (rowing-speed-mps 0.16)]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 42.0)
      (modulate-kinematic-reachability :max-leg-reach-mm 845.0)          ; her leg, hip to sole
      ;; 1. Enforce both Backless-Stool Rim ZCBF and Two-Body Stool Overturning Cone:
      (enforce-backless-stool-rim-zcbf :rim-safety-margin-mm 38.0 :auto-detect-missing-backrest true)
      (enforce-fixture-overturning-cone :min-stool-zmp-margin-mm 28.0 :vertical-preload-boost-n 140.0)
      ;; 2. Blend into `:carpal-dip-suspension` 1-Jet and vault pelvis up & over the front bevel:
      (blend-manifold-posture-jet :atlas 'high-vault-1jet-posture-atlas)
      (drive-capstan-bevel-arm-vault
        :bevel-unweight-fraction bevel-unweight-ratio
        :vault-clear-lift-mm 31.0                    ; her lockout from the dip's origin
        :rowing-velocity-mps (- rowing-speed-mps))
      ;; 3. Walk hands back one ratchet step if seat depth requires further posterior centering:
      (ratchet-carpal-seat-replant
        :replant-mode :alternating-unilateral
        :hand-step-back-mm 165.0
        :stop-at-seat-centroid true)
      ;; 4. Morph 1-Jet from `:carpal-dip-suspension` to `:ischial-seated` with dangling legs:
      (blend-manifold-posture-jet :atlas 'canonical-posture-soa-atlas)
      (specialize-posture-dangling-crural-hang :passive-ankle-droop-rad -0.24)
      (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 18.0)))

;; 2. Universal High-Seat / High-Stool Arm-Dip Egress (Controlled Lowering Past Edge)
;; - Plants hands beside hips, scoots buttocks forward clear of the front bevel,
;;   suspends body weight on the arms in `:carpal-dip-suspension`, lowers the pelvis
;;   eccentrically in front of the stool until plantarflexed toes touch the floor,
;;   transfers the 6D Wrench Cone to the feet, and stands up!
(defmodifier xform-overheight-arm-dip-egress
  [seated-high-state &key (dip-lowering-speed-mps 0.16) (max-dip-travel-mm 167.0)]   ; hers
  (-> seated-high-state
      (wake-inverter-closed-loop)
      (configure-momentum-observer :observer-bandwidth-hz 42.0)
      (modulate-kinematic-reachability :max-leg-reach-mm 845.0)          ; her leg, hip to sole
      (enforce-fixture-overturning-cone :min-stool-zmp-margin-mm 28.0 :vertical-preload-boost-n 120.0)
      ;; 1. Scoot anteriorly to the front bevel using tangential Coulomb slide + carpal boost:
      (specialize-with-carpal-seat-boost :max-arm-push-share 0.45)
      (drive-tangential-coulomb-slide
        :sliding-patches [:ischial-bilateral :posterior-femoral-bilateral]
        :slide-velocity-xyz-mps [0.15 0.0 0.0]
        :max-normal-fz-during-slide 0.55
        :stop-on-ischial-centering false)
      ;; 2. Shift weight onto arms (`:carpal-dip-suspension`), clear buttocks in front of
      ;;    the seat edge, and lower eccentrically until toes establish ground capture!
      (blend-manifold-posture-jet :atlas 'high-vault-1jet-posture-atlas)
      (drive-anterior-overhang-dip-descent
        :overhang-clearance-x-mm 55.0
        :max-dip-lowering-z-mm max-dip-travel-mm
        :descent-velocity-mps (- dip-lowering-speed-mps)
        :skim-dorsal-thigh-on-bevel true)
      ;; 3. Settle heels to plantigrade and restore upright bipedal ZMP balance:
      (blend-manifold-posture-jet :atlas 'canonical-posture-soa-atlas)
      (xform-balance-biped-zmp)
      (ignite-cyclic-clock nominal-biped-gait :bars 1)))


;; =============================================================================
;; B. COMPOSABLE SPECIALIZATION EXTENSIONS (PLUG-IN MODIFIERS)
;; =============================================================================

;; 1. Hand-Anchor Geometry Specializer (Flat Seat Surface vs. Lateral Rim vs. Armrests vs. Forearm)
;; Adapts how the upper limbs anchor onto the chair/stool during the vault & dip:
;; - `:flat-carpal-beside-hips` -> Palms/carpal-heels flat on seat surface outside thighs
;; - `:lateral-seat-rim-cup`    -> Palms cupped over the left/right edges of a narrow stool
;; - `:dual-armrest-parallel-bar` -> Parallel-bar dip on left/right chair armrests
;; - `:ulnar-forearm-bench-vault` -> Full forearm ulna blades on a very high workbench/bed
(defmodifier specialize-vault-hand-anchor-topology
  [vault-state &key (anchor-style :flat-carpal-beside-hips) (seat-width-mm 380.0)]
  (let [coronal-span-mm (* 0.5 (+ seat-width-mm 40.0))]
    (case anchor-style
      (:flat-carpal-beside-hips
        (-> vault-state
            (splice-auxiliary-manifold
              :patch-id :vault-carpal-flat-anchors
              :anatomical-frames [:carpal-heel-l :carpal-heel-r]
              :surface-normal [0.0 0.0 1.0]
              :cop-box-mm `(:x [-30.0 30.0] :y [(- ,coronal-span-mm) ,coronal-span-mm]) ; her fist
              :mu 0.70 :bilateral-grasp-pull false
              :weight-law (:wrench-proportional :max-share 0.90))))

      (:lateral-seat-rim-cup
        (-> vault-state
            (splice-auxiliary-manifold
              :patch-id :vault-rim-cup-anchors
              :anatomical-frames [:palmar-carpal-l :palmar-carpal-r]
              :surface-normal [0.0 0.0 1.0]
              :mu 0.85 :bilateral-grasp-pull false ; Non-prehensile friction/form cup over rim!
              :weight-law (:wrench-proportional :max-share 0.92))
            ;; Add inward coronal adduction bias (25 N) to pinch the left/right stool rims:
            (configure-multi-limb-wrench-allocation
              :grasp-matrix-topology :quadrupedal-virtual-truss
              :nullspace-internal-wrench-law (:edge-tension-target-n 25.0))))

      (:dual-armrest-parallel-bar
        (-> vault-state
            (specialize-with-armrest-rails :rail-load-share 0.90 :grasp-pull-allowed false)
            (specialize-posture-appendage-brace :brace-style :carpal-seat-push :triceps-extension-gain 1.05)))

      (:ulnar-forearm-bench-vault
        (-> vault-state
            (splice-auxiliary-manifold
              :patch-id :vault-ulnar-bench-anchors
              :anatomical-frames [:olecranon-l :olecranon-r :ulna-distal-l :ulna-distal-r]
              :surface-normal [0.0 0.0 1.0]
              :mu 0.75 :bilateral-grasp-pull false
              :weight-law (:wrench-proportional :max-share 0.88)))))))

;; 2. Asymmetric Unilateral Leg-Hook / Heel-Shelf Assist Specializer
;; When a seat or bench is extremely high (> 0.88 m) or arm torque is thermally derated,
;; hooks one flexed heel/calf (`:posterior-femoral-r` or `:calcaneus-r`) onto the seat
;; edge or a lower stool rung FIRST while the other foot pushes from the floor,
;; creating a 3-limb truss (2 hands + 1 hooked leg) that halves required triceps torque!
(defmodifier specialize-vault-with-unilateral-leg-hook
  [vault-state &key (hook-side :right)]
  (let [hook-frames (if (eq hook-side :right)
                        [:femoral-diaphysis-dorsal-r :calcaneus-r]
                        [:femoral-diaphysis-dorsal-l :calcaneus-l])]
    (-> vault-state
        (splice-auxiliary-manifold
          :patch-id :unilateral-bevel-leg-hook
          :anatomical-frames hook-frames
          :surface-normal [0.0 0.0 1.0]
          :mu 0.65
          :weight-law (:wrench-proportional :max-share 0.45))
        (drive-capstan-bevel-arm-vault :bevel-unweight-fraction 0.55))))

;; 3. Light / Narrow / Wobbly Barstool Stabilization Specializer
;; Tightens the two-body overturning cone for lightweight or narrow 3-legged stools:
;; - Forces deeper forward torso pitch (+0.28 rad) during both vault and dip so the
;;   combined Robot+Stool CoM stays pinned over the center of the stool base.
;; - Prohibits simultaneous bimanual hand lifting during the seat-walk ratchet.
(defmodifier specialize-vault-for-light-tippy-stool
  [vault-state &key (estimated-stool-mass-kg 4.5) (base-radius-mm 190.0)]
  (-> vault-state
      (enforce-fixture-overturning-cone
        :min-stool-zmp-margin-mm (* 0.22 base-radius-mm)
        :vertical-preload-boost-n (+ 160.0 (* 10.0 (- 10.0 estimated-stool-mass-kg))))
      (ratchet-carpal-seat-replant
        :replant-mode :alternating-unilateral
        :hand-step-back-mm 120.0)
      (apply-filter-bank
        (dc-bias :target :c7-t12-thoracic :offset-rad +0.26 :zmp-lead-comp true)
        (dc-bias :target :c1-c2-atlas     :offset-rad -0.26 :vor-stabilize true))))

;; 4. Footrest-Rung Intermediate Step Specializer (Barstool with Cross-Rung)
;; If `sub-tarsal-slab` detects a horizontal footrest bar/rung on the high stool
;; (`Z_rung` in [180, 340] mm), uses the rung as an intermediate plantar/equinus step
;; during both ingress and egress!
(defmodifier specialize-dip-egress-with-footrest-rung
  [vault-or-dip-state &key (rung-elevation-z-mm :auto-from-sub-tarsal-sdf)]
  (-> vault-or-dip-state
      (when-modifier (:sub-tarsal-rung-detected-p)
        (-> vault-or-dip-state
            (arpeggiate-elevation-energy
              :step-dz-mm rung-elevation-z-mm
              :step-dx-mm 140.0
              :nosing-clearance-mm 28.0)))))

;; 5. Instant Slip / Tipping Abort-to-Plantar Contingency Specializer
;; If a hand slips on a polished stool edge or the stool begins to tilt (> 2.0 deg)
;; during the arm-vault ingress, immediately aborts the backward row, reverses the
;; triceps dip to drop the plantarflexed feet straight back under the CoM, and
;; executes a reactive capture step on the floor!
(defmodifier specialize-vault-abort-to-plantar-drop [vault-state]
  (-> vault-state
      (bind-vault-contingency-reflex
        :trigger-conditions [:hand-tangential-slip-exceeds-mm 18.0
                             :fixture-tilt-angle-exceeds-rad 0.035
                             :bemf-elbow-torque-saturation]
        :abort-action
          (-> (pan-wrench-homotopy
                :from-manifolds :current
                :to-manifolds '((:metatarsal-equinus-bilateral 0.70) (:plantar-bilateral 0.30))
                :duration-ms 140.0)
              (specialize-with-reactive-capture-step :step-clearance-z-mm 40.0 :capture-lead-gain 1.18)
              (xform-balance-biped-zmp)))))
