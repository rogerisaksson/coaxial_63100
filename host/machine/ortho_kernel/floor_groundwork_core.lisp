;; =============================================================================
;; FILE: floor_groundwork_core.lisp
;; DOMAIN: Floor Contact Hulls, 1-Jet Ground Postures & Helical Rolling Physics
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp
;; =============================================================================
(defpackage :ortho-dsp.ground-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core)
  (:export #:groundwork-manifold-extensions
           #:groundwork-1jet-posture-atlas
           #:drive-helical-axial-roll-wave
           #:adapt-rolling-substrate-mechanics
           #:route-90-90-hip-inversion-bridge))
(in-package :ortho-dsp.ground-core)

;; -----------------------------------------------------------------------------
;; 1. EXTENDED SOMATIC FLOOR MANIFOLDS (KNEES, VENTRAL PRONE, LATERAL ROLL HULL)
;; Completes the 360-degree circumferential body contact hull on Z = 0.
;; -----------------------------------------------------------------------------
(defcontact-patch groundwork-manifold-extensions
  ;; Bilateral Anterior Tibial / Patellar Keel (Kneeling upright & Hands-and-Knees)
  (:patch :anterior-tibial-bilateral
    :frames          [:tibial-tuberosity-l :tibial-tuberosity-r :tarsal-dorsum-l :tarsal-dorsum-r]
    :local-normal    [1.0 0.0 0.0]              ; Anterior shin normal becomes vertical [0,0,1] at 90 deg knee flex
    :cop-bounds-mm   (:x [-220.0 45.0] :y [-135.0 135.0])
    :default-mu      0.75
    :unilateral-fz   true)

  ;; Ventral Prone Continuum (Lying flat on stomach: sternum, ribs, anterior pelvis, thighs)
  (:patch :ventral-prone-continuum
    :frames          [:sternal-pectoral :iliac-anterior-spine :femoral-diaphysis-ventral]
    :local-normal    [1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-440.0 460.0] :y [-185.0 185.0])
    :default-mu      0.60
    :unilateral-fz   true)

  ;; Olecranon / Forearm Sphinx Prop (Prone on stomach propped on forearms/elbows)
  (:patch :olecranon-ulnar-sphinx
    :frames          [:olecranon-l :olecranon-r :ulna-distal-l :ulna-distal-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-120.0 120.0] :y [-160.0 160.0])
    :default-mu      0.70
    :unilateral-fz   true))

;; -----------------------------------------------------------------------------
;; 2. GROUNDWORK 1-JET POSTURE ATLAS (ORIGINS q_0 + MODAL SYNERGY BASES B_m)
;; Covers all requested floor states without singular standing-squat assumptions:
;;   1) Tall Kneeling (Knees only)
;;   2) Quadruped Kneeling (Knees + Hands/Arms)
;;   3) Floor Long-Sit / Cross-Sit (Buttocks only)
;;   4) Tripod Seated (Buttocks + Posterior/Lateral Hands & Arms)
;;   5) Flat Supine (On back)
;;   6) Flat Prone & Sphinx Prone (On stomach)
;; -----------------------------------------------------------------------------
(defstate groundwork-1jet-posture-atlas
  (:postures
    ;; 1. TALL KNEELING (On her knees, trunk upright, hands free)
    (:posture :kneeling-tall-biped
      :coupled-manifold :anterior-tibial-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.58 :tibiofemoral 1.5708 :coxofemoral 0.00
         :iliac-coronal 0.00 :l4-l5-lumbar -0.10 :c7-t12-thoracic 0.04
         :glenohumeral 0.05 :cubital 0.25 :c1-c2-atlas 0.00)
      :modal-basis-columns
        ;; Axial compression sits hips down toward heels (Seiza fold) + cantilevers arms forward:
        (:mode-axial-compression (:tibiofemoral +0.72 :coxofemoral +0.95
                                  :l4-l5-lumbar +0.15 :glenohumeral +0.55)
         :mode-sagittal-shift    (:coxofemoral +0.48 :l4-l5-lumbar +0.22 :c1-c2-atlas -0.35)
         :mode-pitch-attitude    (:coxofemoral +0.65 :l4-l5-lumbar +0.35 :c1-c2-atlas -0.80)))

    ;; 2. KNEES WITH ARMS AND HANDS (On her knees, arms and hands on the floor)
    (:posture :kneeling-quadruped-braced
      :coupled-manifold [:anterior-tibial-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.55 :tibiofemoral 1.52 :coxofemoral 1.38
         :iliac-coronal 0.00 :l4-l5-lumbar 0.14 :c7-t12-thoracic 0.18
         :glenohumeral 1.32 :cubital 0.18 :c1-c2-atlas -0.85)
      :modal-basis-columns
        ;; Axial compression bends elbows and lowers chest+hips toward floor (child's pose / prone prep):
        (:mode-axial-compression (:tibiofemoral +0.45 :coxofemoral +0.42
                                  :glenohumeral +0.28 :cubital +1.10)
         ;; Sagittal shift rocks weight forward over hands or backward over knees:
         :mode-sagittal-shift    (:coxofemoral -0.55 :tibiofemoral -0.35 :glenohumeral +0.45)
         :mode-pitch-attitude    (:coxofemoral +0.40 :glenohumeral -0.45 :c1-c2-atlas -0.40)))

    ;; 3. FLOOR BUTTOCKS ONLY (On her buttocks on the floor, legs straight or slightly bent ahead)
    (:posture :floor-ischial-unbraced
      :coupled-manifold [:ischial-bilateral :posterior-femoral-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.15 :tibiofemoral 0.28 :coxofemoral 1.42
         :iliac-coronal 0.00 :l4-l5-lumbar -0.18 :c7-t12-thoracic 0.08
         :glenohumeral 0.25 :cubital 0.55 :c1-c2-atlas 0.00)
      :modal-basis-columns
        ;; Axial compression tucks knees toward chest and rounds spine into a rolling ball:
        (:mode-axial-compression (:tibiofemoral +1.45 :coxofemoral +0.65
                                  :l4-l5-lumbar +0.38 :c7-t12-thoracic +0.35)
         :mode-sagittal-shift    (:coxofemoral +0.45 :l4-l5-lumbar +0.32 :glenohumeral +0.45)
         :mode-pitch-attitude    (:coxofemoral +0.50 :l4-l5-lumbar +0.40 :c1-c2-atlas -0.75)))

    ;; 4. FLOOR BUTTOCKS WITH ARMS AND HANDS (On her buttocks, hands and arms braced behind or beside)
    (:posture :floor-ischial-carpal-tripod
      :coupled-manifold [:ischial-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.20 :tibiofemoral 0.65 :coxofemoral 1.12
         :iliac-coronal 0.00 :l4-l5-lumbar 0.05 :c7-t12-thoracic 0.10
         :glenohumeral -0.68 :glenohumeral-ext-rot 0.85 :cubital 0.28 :c1-c2-atlas 0.15)
      :modal-basis-columns
        ;; Axial compression bends elbows behind back to lower torso toward supine OR extends to lift pelvis:
        (:mode-axial-compression (:glenohumeral -0.35 :cubital +0.95
                                  :coxofemoral -0.45 :l4-l5-lumbar +0.25)
         :mode-sagittal-shift    (:coxofemoral +0.65 :glenohumeral +0.55 :cubital -0.25)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.35 :c7-t12-thoracic +0.30 :c1-c2-atlas -0.50)))

    ;; 5. FLAT SUPINE ON BACK (Lying flat on her back)
    (:posture :floor-supine-flat
      :coupled-manifold :dorsal-supine-continuum
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.22 :tibiofemoral 0.04 :coxofemoral 0.02
         :iliac-coronal 0.00 :l4-l5-lumbar -0.02 :c7-t12-thoracic 0.02
         :glenohumeral 0.04 :cubital 0.12 :c1-c2-atlas 0.04)
      :modal-basis-columns
        (:mode-axial-compression (:tibiofemoral +1.35 :coxofemoral +0.75 :l4-l5-lumbar +0.22)
         :mode-sagittal-shift    (:coxofemoral +0.35 :l4-l5-lumbar +0.42 :c7-t12-thoracic +0.48)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.25 :c7-t12-thoracic +0.55 :c1-c2-atlas +0.48)))

    ;; 6. FLAT PRONE ON STOMACH (Lying flat on her stomach + Cobra/Sphinx press-up basis)
    (:posture :floor-prone-flat
      :coupled-manifold [:ventral-prone-continuum :olecranon-ulnar-sphinx]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.52 :tibiofemoral 0.05 :coxofemoral 0.00
         :iliac-coronal 0.00 :l4-l5-lumbar -0.08 :c7-t12-thoracic -0.04
         :glenohumeral 0.45 :glenohumeral-abduction 0.55 :cubital 1.85 ; Hands tucked by chest/shoulders
         :c1-c2-atlas -0.35 :c1-c2-atlas-yaw 0.785)                   ; Turn cheek/chin to clear floor!
      :modal-basis-columns
        ;; Axial compression executes a Prone Cobra / Push-Up press (extending arms, arching spine, flexing knees):
        (:mode-axial-compression (:glenohumeral +0.65 :cubital -1.15
                                  :l4-l5-lumbar -0.38 :c7-t12-thoracic -0.32
                                  :coxofemoral +0.85 :tibiofemoral +0.95)
         :mode-sagittal-shift    (:coxofemoral +0.55 :tibiofemoral +0.60 :glenohumeral +0.35)
         :mode-pitch-attitude    (:l4-l5-lumbar -0.45 :c7-t12-thoracic -0.40 :c1-c2-atlas -0.55)))))

;; -----------------------------------------------------------------------------
;; 3. HELICAL AXIAL ROLL WAVE ENGINE (SUPINE <-> LATERAL <-> PRONE <-> SUPINE)
;; Drives continuous 180 or 360 deg rolling without jamming the downside shoulder:
;;   1) Tucks the ipsilateral (downside) arm overhead into a zero-obstruction slot.
;;   2) Swings the contralateral leg & arm across the sagittal midline as a
;;      gravity cantilever to shift CoM past the lateral roll axis.
;;   3) Propagates a phase-delayed axial twist between pelvis and thorax.
;; -----------------------------------------------------------------------------
(defprimitive drive-helical-axial-roll-wave
  [state &key (roll-direction :clockwise-right)  ; :clockwise-right | :counter-clockwise-left
              (target-roll-angle-rad 3.14159)    ; pi = 180 deg (Back<->Stomach), 2*pi = 360 deg full roll
              (propagation-leader :pelvic-led)   ; :pelvic-led (lower body initiates) | :thoracic-led
              (roll-angular-velocity-radps 1.45)]
  (let [ipsi-side   (if (eq roll-direction :clockwise-right) :right :left)
        contra-side (if (eq roll-direction :clockwise-right) :left  :right)]
    (-> state
        (execute-nonholonomic-body-roll
          :chirality roll-direction
          :total-angle-rad target-roll-angle-rad
          :nominal-omega-radps roll-angular-velocity-radps
          :axial-wave-leader propagation-leader
          :inter-girdle-phase-lag-rad 0.38       ; 22 deg helical twist between pelvis and ribcage!

          ;; A. Ipsilateral Overhead Arm-Slot (Prevents crushing the downside arm under ribs!)
          :ipsilateral-limb-clearance
            (:side ipsi-side
             :glenohumeral-flexion-rad 2.88      ; Sweep downside arm straight overhead along floor
             :cubital-flexion-rad 0.15
             :scapular-protraction-m 0.035
             :tuck-chin-and-craniofacial true)   ; Keep cranium 35mm off floor during roll!

          ;; B. Contralateral Pendulum Cantilever (Generates internal rolling torque)
          :contralateral-momentum-driver
            (:side contra-side
             :coxofemoral-flexion-rad 0.95       ; Lift opposite knee & sweep across midline
             :coxofemoral-adduction-rad 0.52
             :glenohumeral-cross-reach-rad 1.25)

          ;; C. Circumferential Manifold Homotopy around the Torso Perimeter:
          :circumferential-manifold-ring
            [:dorsal-supine-continuum
             :trochanter-gluteal-lateral
             :ventral-prone-continuum
             :trochanter-gluteal-lateral
             :dorsal-supine-continuum]))))

;; -----------------------------------------------------------------------------
;; 4. SUBSTRATE-ADAPTIVE ROLLING MECHANICS (HARD FLOOR VS. SOFT VISCOELASTIC BED)
;; Reads real-time contact stiffness k_sub = d(Fz)/d(z) from the Momentum Observer:
;; - On HARD floors (k > 400 N/mm): Rounds the spine/pelvis to eliminate bony point
;;   impacts (iliac crest, acromion, patella) and softens tangential shear stiffness.
;; - On SOFT surfaces (k < 60 N/mm): Injects a contralateral heel+palm push-off
;;   impulse to climb out of the viscoelastic mattress sinkage trough!
;; -----------------------------------------------------------------------------
(defprimitive adapt-rolling-substrate-mechanics
  [state &key (hard-floor-threshold-npmm 380.0)
              (soft-mattress-threshold-npmm 65.0)]
  (-> state
      (bind-substrate-dependent-roll-law
        :stiffness-source :generalized-momentum-observer-df-dz
        ;; Branch 1: Hard Unyielding Surface (Concrete, Wood, Tile)
        :hard-substrate-compensation
          (:active-above-npmm hard-floor-threshold-npmm
           :kyphotic-rounding-bias-rad +0.24     ; Curve spine like a wheel rim (no flat-back slap!)
           :bony-landmark-pressure-shield
             (:protected-frames [:greater-trochanter-l :greater-trochanter-r
                                 :glenohumeral-l :glenohumeral-r
                                 :knee-patellar-l :knee-patellar-r]
              :local-admittance-yield-mm 28.0
              :atten-stiffness-on-transit-db -18.0))
        ;; Branch 2: Soft Viscoelastic Surface (Mattress, Gym Mat, Deep Carpet, Sand)
        :soft-substrate-compensation
          (:active-below-npmm soft-mattress-threshold-npmm
           :sinkage-trough-escape-boost
             (:contralateral-heel-pushoff-n 110.0 ; Push heel into mattress to lever hips out of trough!
              :contralateral-palmar-pushoff-n 75.0
              :archimedean-hip-lift-clearance-mm 48.0
              :viscoelastic-phase-lead-rad 0.26)))))

;; -----------------------------------------------------------------------------
;; 5. ASYMMETRIC 90/90 HIP-INVERSION & TRIPOD SWEEP BRIDGE
;; Solves the topological collision between Kneeling (shins behind hips) and
;; Buttock-Sitting (shins in front of hips) via a unilateral carpal tripod and
;; 90/90 internal/external hip rotation sweep.
;; -----------------------------------------------------------------------------
(defprimitive route-90-90-hip-inversion-bridge
  [state &key (direction :kneel-to-buttocks)     ; :kneel-to-buttocks | :buttocks-to-kneel
              (pivot-side :left)]
  (let [post-hand  (if (eq pivot-side :left) :carpal-heel-l :carpal-heel-r)
        sweep-legs [:coxofemoral-l :tibiofemoral-l :coxofemoral-r :tibiofemoral-r]]
    (-> state
        (execute-topological-leg-inversion
          :direction direction
          :support-tripod-anchor post-hand
          ;; Shift 35% weight onto ipsilateral hand + hip while sweeping shins through
          ;; a 45mm elevated null-space arc clear of the floor!
          :tripod-load-share 0.35
          :pelvic-clearance-lift-mm 45.0
          :hip-90-90-rotational-pathway
            (:lead-hip-internal-rot-rad +0.68
             :trail-hip-external-rot-rad -0.68
             :sweep-duration-ms 680.0
             :enforce-22-capsule-shin-guard true)))))
