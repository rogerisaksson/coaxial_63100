;; =============================================================================
;; FILE: sequential_bedside_ingress.lisp
;; DOMAIN: Sit on Bed Long-Side -> Lean Back on Forearms -> Swing Up One Leg
;;         at a Time in Null-Space -> Align & Settle Supine Along Bed
;; REQUIRES: ortho_dsp_somatic_substrate.lisp,
;;           ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp,
;;           hitchin_isomonodromic_bus.lisp,
;;           floor_groundwork_core.lisp,
;;           deformable_substrate_core.lisp,
;;           terrestrial_multimodal_gaits.lisp,
;;           bedside_and_stool_core.lisp
;; =============================================================================
(defpackage :ortho-dsp.bed-sequential
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
        :ortho-dsp.hitchin
        :ortho-dsp.ground-core
        :ortho-dsp.ground-xforms
        :ortho-dsp.deformable-core
        :ortho-dsp.deformable-xforms
        :ortho-dsp.substrate-overlay
        :ortho-dsp.terrestrial-core
        :ortho-dsp.terrestrial-xforms
        :ortho-dsp.bedside-core
        :ortho-dsp.bedside-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.supervisor
        :ortho-dsp.lowering)
  (:export #:drive-sequential-unilateral-leg-mount
           #:xform-bed-longside-sequential-ingress
           #:compile-sequential-bed-ingress-soa))
(in-package :ortho-dsp.bed-sequential)

;; -----------------------------------------------------------------------------
;; 1. SEQUENTIAL UNILATERAL LEG-SWEEP PRIMITIVE (ONE LEG AT A TIME IN NULL-SPACE)
;; While reclined on the buttocks + forearms (`:ischial` + `:olecranon-ulnar`),
;; lifts and swings the Lead (inner/headboard) leg onto the mattress first,
;; latches its heel/thigh weight on the bed, rotates the pelvis in yaw via
;; Schlesinger isomonodromy, and then lifts and swings the Trail (outer) leg up!
;; -----------------------------------------------------------------------------
(defprimitive drive-sequential-unilateral-leg-mount
  [state &key (headboard-side :left)            ; :left -> Left leg goes up first, then Right leg
              (mattress-rim-clearance-mm 68.0)  ; Parabolic heel/shin clearance over mattress edge
              (per-leg-sweep-duration-ms 780.0)]
  (let [lead-leg-chain   (if (eq headboard-side :left)
                             [:coxofemoral-l :tibiofemoral-l :talocrural-l :calcaneus-l]
                             [:coxofemoral-r :tibiofemoral-r :talocrural-r :calcaneus-r])
        trail-leg-chain  (if (eq headboard-side :left)
                             [:coxofemoral-r :tibiofemoral-r :talocrural-r :calcaneus-r]
                             [:coxofemoral-l :tibiofemoral-l :talocrural-l :calcaneus-l])
        lead-foot-patch  (if (eq headboard-side :left) :calcaneus-l :calcaneus-r)
        trail-foot-patch (if (eq headboard-side :left) :calcaneus-r :calcaneus-l)
        bed-yaw-sign     (if (eq headboard-side :left) +1.570796 -1.570796)]
    (-> state
        ;; STAGE A: Unload Lead (Inner) Leg & Sweep onto Mattress in J_bar_c Null-Space
        ;; Outer leg remains planted on floor (or dangling counterweight) while forearms + buttocks hold CWC!
        (rebalance-subpatch-symmetry
          :patch :plantar-bilateral
          :active-subframe trail-foot-patch)
        (guard-foothold-with-crural-capsules
          :checked-limb-capsules [:foot-shell-l :foot-shell-r :shank-crural-l :shank-crural-r]
          :crural-repulsion-weight 3.8)
        (project-nullspace-posture
          :free-nodes lead-leg-chain
          :trajectory-contour (:asymmetric-parabolic-arc :apex-z-mm mattress-rim-clearance-mm)
          :target-angles `((:coxofemoral-flex 0.95)
                           (:coxofemoral-abduct ,(* 0.55 bed-yaw-sign))
                           (:tibiofemoral-flex 0.35)
                           (:talocrural-pitch -0.15))
          :nullspace-leakage-guard 0.0
          :duration-ms per-leg-sweep-duration-ms)

        ;; STAGE B: Anchor Lead Leg on Mattress & Pivot Pelvis 90 Deg on Ischial Patch
        ;; Transfer support to [:ischial + :olecranon-ulnar + lead-leg-on-bed], freeing Trail Leg!
        (rebalance-subpatch-symmetry
          :patch :plantar-bilateral
          :active-subframe lead-foot-patch)
        (propagate-schlesinger-isomonodromy :min-pole-separation-eps 0.025)
        (drive-tangential-coulomb-slide
          :sliding-patches [:ischial-bilateral :trochanter-gluteal-lateral]
          :slide-velocity-xyz-mps [0.0 0.0 0.0] ; Pure in-place rotational yaw slip on mattress
          :max-normal-fz-during-slide 0.65)

        ;; STAGE C: Unload Trail (Outer) Leg & Sweep Up Alongside Lead Leg in Null-Space
        (project-nullspace-posture
          :free-nodes trail-leg-chain
          :trajectory-contour (:asymmetric-parabolic-arc :apex-z-mm mattress-rim-clearance-mm)
          :target-angles '((:coxofemoral-flex 0.42)
                           (:coxofemoral-abduct 0.00)
                           (:tibiofemoral-flex 0.25)
                           (:talocrural-pitch -0.18))
          :nullspace-leakage-guard 0.0
          :duration-ms per-leg-sweep-duration-ms))))

;; -----------------------------------------------------------------------------
;; 2. COMPLETE LONGSIDE SIT -> LEAN BACK -> ONE-LEG-AT-A-TIME -> SETTLE SUPINE
;; -----------------------------------------------------------------------------
(defmodifier xform-bed-longside-sequential-ingress
  [standing-at-bedside-state &key (headboard-direction :left)
                                  (bed-width-mm 900.0)
                                  (pillow-height-mm 65.0)
                                  (terminal-sleep-high-z true)]
  (-> standing-at-bedside-state
      ;; 1. Stand by long side of bed, turn 180 deg, brush calves against mattress edge:
      (xform-stand-and-turn-at-fixture
        :turn-yaw-rad 3.14159
        :standoff-clearance-mm 30.0)

      ;; 2. Activate Viscoelastic Mattress Rheology & Sit + Scoot 165mm inward from rim:
      (apply-universal-deformable-overlay :allow-elastic-kip-egress false)
      (xform-sit-on-chair-or-stool
        :slide-speed-mps 0.14
        :use-carpal-hand-boost true)

      ;; 3. Lean Back & Diagonal onto Broad Forearms/Elbows (`:olecranon-ulnar-sphinx`):
      ;;    Shifts CoM well inside the mattress polygon and distributes arm pressure
      ;;    so wrists don't punch into the foam!
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (specialize-posture-appendage-brace
        :brace-style :mattress-elbow-prop
        :triceps-extension-gain 0.85)
      (pan-wrench-homotopy
        :from-manifolds '((:ischial-bilateral 0.80) (:plantar-bilateral 0.20))
        :to-manifolds   '((:ischial-bilateral 0.58)
                          (:olecranon-ulnar-sphinx 0.30)
                          (:plantar-bilateral 0.12))
        :duration-ms 520.0)

      ;; 4. Swing Up One Leg at a Time (Inner/Headboard Leg -> Latch -> Outer Leg):
      (xform-hitchin-integrable-balance)
      (drive-sequential-unilateral-leg-mount
        :headboard-side headboard-direction
        :mattress-rim-clearance-mm 68.0
        :per-leg-sweep-duration-ms 780.0)

      ;; 5. "Settle In" (Lagga sig tillratta utmed sangen):
      ;;    a) Execute 1-2 gentle stick-slip pelvic scoots ("shrimp" recentering) using
      ;;       heels + elbows to center the torso on the bed midline and align head on pillow:
      (drive-centerline-pinned-mattress-roll
        :target-roll-state :supine-back
        :bed-width-mm bed-width-mm
        :edge-safety-margin-mm 140.0)
      (modulate-stick-slip-sled-ratchet
        :anchor-patches [:plantar-bilateral :olecranon-ulnar-sphinx]
        :sled-patches   [:ischial-bilateral :dorsal-supine-continuum]
        :vault-unweight-ratio 0.55
        :rowing-stroke-length-mm 140.0)
      (specialize-with-bed-headboard-pillow-align
        :headboard-clearance-mm 85.0
        :pillow-elevation-mm pillow-height-mm)

      ;;    b) Unroll spine from forearm prop into aligned supine rest and release impedance:
      (xform-floor-to-supine-flat :terminal-high-z terminal-sleep-high-z)
      (specialize-posture-recumbent-archetype
        :archetype :supine-aligned
        :pillow-height-mm pillow-height-mm)))

;; -----------------------------------------------------------------------------
;; 3. DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-sequential-bed-ingress-soa
  (let [sequential-bed-settle (-> nominal-biped-gait
                                  (transducer-unshod)
                                  (xform-balance-biped-zmp)
                                  (xform-bed-longside-sequential-ingress
                                    :headboard-direction :left
                                    :bed-width-mm 900.0
                                    :pillow-height-mm 65.0
                                    :terminal-sleep-high-z true)
                                  (limiter-high-mu-planar)
                                  (master-anti-spasm-insert))]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(oneshot sequential-bed-settle)])))
