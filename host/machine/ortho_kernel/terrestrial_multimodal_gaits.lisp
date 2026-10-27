;; =============================================================================
;; FILE: terrestrial_multimodal_gaits.lisp
;; DOMAIN: T^5 x se(2) Ehresmann Vectoring, Coxa-Turnout Crabwalk, Stick-Slip
;;         Buttock Scooting/Dragging & Supine/Pronograde Grade Ratcheting
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp
;; =============================================================================
(defpackage :ortho-dsp.terrestrial-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core)
  (:export #:terrestrial-1jet-posture-atlas
           #:drive-5torus-ehresmann-clock
           #:align-hinge-joints-to-se2-vector
           #:modulate-stick-slip-sled-ratchet
           #:arpeggiate-supine-pronograde-grade))
(in-package :ortho-dsp.terrestrial-core)

;; -----------------------------------------------------------------------------
;; 1. TERRESTRIAL 1-JET POSTURE ATLAS (DOG, SUPINE-CRAWL, BUTT-SCOOT, CRABWALK)
;; Defines canonical 0-jet origins and 1-jet modal columns for all non-upright
;; locomotion topologies, including coronal hip turnout for crabwalks.
;; -----------------------------------------------------------------------------
(defstate terrestrial-1jet-posture-atlas
  (:postures
    ;; A. PRONOGRADE DOG / BEAR CRAWL (Belly down, hands + equinus feet or knees)
    (:posture :terrestrial-pronograde-dog
      :coupled-manifold [:metatarsal-equinus-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.42 :tibiofemoral 1.46 :coxofemoral 1.30
         :iliac-coronal 0.00 :l4-l5-lumbar +0.16 :c7-t12-thoracic +0.22
         :glenohumeral +1.36 :cubital 0.24 :c1-c2-atlas -0.785)            ; her neck's 45 deg stop
      :modal-basis-columns
        (:mode-axial-compression (:tibiofemoral +0.48 :coxofemoral +0.40 :glenohumeral +0.25 :cubital +0.88)
         :mode-sagittal-shift    (:coxofemoral -0.45 :tibiofemoral -0.30 :glenohumeral +0.42)
         :mode-pitch-attitude    (:coxofemoral +0.55 :glenohumeral -0.55 :c1-c2-atlas -0.45)))

    ;; B. SUPINE BACK-TO-FLOOR CRAWL (Belly up, heels/feet + externally rotated hands)
    (:posture :terrestrial-supine-bridge
      :coupled-manifold [:plantar-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.24 :tibiofemoral 1.38 :coxofemoral 0.45
         :iliac-coronal 0.00 :l4-l5-lumbar -0.10 :c7-t12-thoracic -0.06
         :glenohumeral -0.80 :glenohumeral-ext-rot +1.18 :cubital 0.30
         :c1-c2-atlas +0.68) ; Chin tucked so stereo rig looks across torso toward horizon
      :modal-basis-columns
        (:mode-axial-compression (:coxofemoral +0.80 :tibiofemoral +0.42 :glenohumeral -0.25 :cubital +0.82)
         :mode-sagittal-shift    (:talocrural +0.35 :tibiofemoral -0.40 :glenohumeral +0.38)
         :mode-pitch-attitude    (:coxofemoral -0.45 :l4-l5-lumbar -0.25 :c1-c2-atlas +0.38)))

    ;; C. ISCHIAL BUTTOCK-SCOOT & DRAG (Seated on buttocks + heels + posterior/lateral hands)
    (:posture :terrestrial-ischial-scooter
      :coupled-manifold [:ischial-bilateral :plantar-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.15 :tibiofemoral 1.25 :coxofemoral 1.35
         :iliac-coronal 0.00 :l4-l5-lumbar +0.08 :c7-t12-thoracic +0.14
         :glenohumeral -0.45 :glenohumeral-ext-rot +0.65 :cubital 0.48 :c1-c2-atlas 0.05)
      :modal-basis-columns
        ;; Axial compression pushes down through palms & heels to vault/unweight the buttocks!
        (:mode-axial-compression (:glenohumeral -0.35 :cubital -0.45 :coxofemoral -0.28 :l4-l5-lumbar -0.18)
         ;; Sagittal shift rows the pelvis forward or backward relative to anchored hands/heels:
         :mode-sagittal-shift    (:tibiofemoral -0.65 :coxofemoral -0.55 :glenohumeral +0.60)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.35 :c7-t12-thoracic +0.30 :c1-c2-atlas -0.45)))

    ;; D. LATERAL CRAB-STANCE (Wide-abducted external-turnout posture for pure sideways motion)
    (:posture :terrestrial-coronal-crab
      :coupled-manifold [:plantar-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.18 :tibiofemoral 1.28
         :coxofemoral 0.85 :coxofemoral-abduct 0.62 :coxofemoral-ext-rot 0.436 ; her hip's 25 deg turn
         :iliac-coronal 0.00 :l4-l5-lumbar -0.05 :c7-t12-thoracic 0.06
         :glenohumeral -1.04 :cubital 0.00 :c1-c2-atlas 0.00)              ; straight arms plumb, her shoulder no turn
      :modal-basis-columns
        (:mode-axial-compression (:tibiofemoral +0.65 :coxofemoral +0.45 :cubital +0.65)
         :mode-sagittal-shift    (:iliac-coronal +0.45 :coxofemoral-abduct +0.35)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.30 :c1-c2-atlas -0.40)))))

;; -----------------------------------------------------------------------------
;; 2. 5-TORUS EHRESMANN CLOCK (4 APPENDAGES + 1 AXIAL PELVIC/TORSO OSCILLATOR)
;; Drives [LH, LF, RH, RF, Axial-Sled] phases and prevents mid-line limb clashes
;; during lateral crabwalking by enforcing Outer-Limb-Leads phase ordering.
;; -----------------------------------------------------------------------------
(defprimitive drive-5torus-ehresmann-clock
  [state &key (master-bpm 94.0)
              (appendage-duty-pwm 0.68)
              (axial-sled-duty-pwm 0.00)        ; 0.0 = pelvis suspended; >0.0 = intermittent buttock scoot!
              (phase-5vec [0.00 0.50 0.50 0.00 0.25]) ; [LH, LF, RH, RF, Axial]
              (velocity-twist-vx-vy-wz [0.35 0.0 0.0])]
  (-> state
      (bind-5torus-locomotion-oscillator
        :bpm master-bpm
        :limb-duty appendage-duty-pwm
        :axial-duty axial-sled-duty-pwm
        :nominal-phases phase-5vec
        :desired-se2-twist velocity-twist-vx-vy-wz
        ;; OUTER-LEAD CORONAL CHIRALITY GUARD:
        ;; When moving sideways (|vy| > |vx|), automatically shifts phase offsets so the
        ;; leading outer side (Right limbs if vy < 0, Left limbs if vy > 0) steps outward
        ;; at phi = 0.0 BEFORE the trailing inner limbs step inward at phi = 0.50!
        :coronal-non-crossing-phase-lock true)))

;; -----------------------------------------------------------------------------
;; 3. HINGE-JOINT TO SE(2) VECTOR ALIGNER (CRABWALK COXA-TURNOUT FOLIO)
;; Turns her hips out kappa_lat = |vy| / sqrt(vx^2 + vy^2) of their 25 deg stop: her
;; hip abduction carries the sideways stroke, her knees help; her shoulder only flexes,
;; her arms bear as plumb columns.
;; -----------------------------------------------------------------------------
(defprimitive align-hinge-joints-to-se2-vector
  [state &key (orientation-topology :pronograde-belly-down) ; :pronograde-belly-down | :supine-belly-up | :ischial-seated | :deep-biped-squat
              (max-coxa-turnout-rad 0.436)                  ; Her hip's 25 deg turn
              (hindlimb-load-share 0.66)]
  (-> state
      (modulate-proximal-balljoint-turnout
        :topology orientation-topology
        :turnout-scaling-law :coronal-velocity-ratio-kappa
        :max-hip-external-rot-rad max-coxa-turnout-rad
        :max-hip-abduction-widen-rad 0.52
        :max-shoulder-external-rot-rad 0.0
        ;; Ensure virtual limb lengths match across all 360 deg travel directions:
        :equalize-hind-fore-lever-arms true
        :hind-to-fore-thermal-load-bias hindlimb-load-share
        :zero-internal-truss-shear true)))

;; -----------------------------------------------------------------------------
;; 4. STICK-SLIP SLED RATCHET (BUTTOCK SCOOTING & ASYMMETRIC FLOOR DRAGGING)
;; Partitions manifolds into Anchors (static mu_s, v=0) and Sleds (kinetic mu_k,
;; controlled sliding) synchronized with the 5th torus phase `phi_axial`.
;; -----------------------------------------------------------------------------
(defprimitive modulate-stick-slip-sled-ratchet
  [state &key (anchor-patches [:ulnar-carpal-shove-bilateral :plantar-bilateral])
              (sled-patches   [:ischial-bilateral :posterior-femoral-bilateral])
              (vault-unweight-ratio 0.85)       ; 1.0 = full aerial pelvic hop; 0.35 = continuous heavy drag
              (rowing-stroke-length-mm 260.0)]
  (-> state
      (configure-peristaltic-stick-slip-cycle
        :anchors anchor-patches
        :sleds   sled-patches
        :pelvic-unweighting-gain vault-unweight-ratio
        :stroke-amplitude-m (* 0.001 rowing-stroke-length-mm)
        ;; Phase 1 (phi_axial in [0.0, 0.5]): Anchors stick (w_anchor -> 0.85), pelvis unweights
        ;; and slides along desired SE(2) vector via `kinetic-coulomb-surfer`.
        ;; Phase 2 (phi_axial in [0.5, 1.0]): Pelvis anchors on floor (w_sled -> 0.90), freeing
        ;; hands and feet to reach to their next anchor points in Null-Space!
        :schlesinger-isomonodromic-torsion-guard true
        :anti-carpet-snag-pelvic-tilt-rad 0.14))) ; Lift leading edge of pelvis so tailbone/trochanter never snags!

;; -----------------------------------------------------------------------------
;; 5. SUPINE & PRONOGRADE GRADE RATCHET (CRAWLING UP & DOWN SLOPES / STAIRS)
;; Protects shoulder motors when crawling uphill or upstairs with back to floor
;; by using the buttocks (`:ischial`) as a directional mechanical pawl.
;; -----------------------------------------------------------------------------
(defprimitive arpeggiate-supine-pronograde-grade
  [state &key (grade-dz-mm :from-sub-tarsal-sdf)
              (body-orientation :supine-belly-up)]
  (-> state
      (drive-terrestrial-elevation-ratchet
        :elevation-delta-mm grade-dz-mm
        :orientation body-orientation
        ;; A. SUPINE ASCENT (Climbing UP on the back, feet-first or head-first):
        :supine-ascent-law
          (:primary-work-actuators [:coxofemoral :tibiofemoral] ; Legs provide 80% of +m*g*dZ!
           :ratchet-pawl-patch :ischial-bilateral               ; Latch buttocks on step/slope to rest arms
           :max-glenohumeral-extension-torque-nm 28.0)
        ;; B. SUPINE DESCENT (Crawling DOWN on the back):
        :supine-descent-law
          (:skim-clearance-z-mm 18.0                            ; Keep buttocks 18mm from floor as zero-drop brake!
           :eccentric-braking-patches [:plantar-bilateral :ulnar-carpal-shove-bilateral]
           :prevent-sacral-step-nosing-impact true)
        ;; C. PRONOGRADE ASCENT/DESCENT (All-fours scrambling up/down):
        :pronograde-grade-law
          (:pitch-torso-parallel-to-slope true
           :inter-limb-wedge-tension-n 24.0))))
