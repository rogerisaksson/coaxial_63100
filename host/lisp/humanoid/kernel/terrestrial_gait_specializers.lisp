;; =============================================================================
;; FILE: terrestrial_gait_specializers.lisp
;; DOMAIN: Composable Non-Bipedal Gaits (Dog, Supine-Crawl, Butt-Scoot, Crabwalk)
;; REQUIRES: terrestrial_multimodal_gaits.lisp, ortho_dsp_cognitive_autonomy.lisp
;; =============================================================================
(defpackage :ortho-dsp.terrestrial-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.posture-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.terrestrial-core)
  (:export ;; 1. Four Canonical Non-Upright Gait Generators
           #:xform-gait-pronograde-dog-crawl
           #:xform-gait-supine-back-crawl
           #:xform-gait-ischial-butt-scoot
           #:xform-gait-lateral-crabwalk
           ;; 2. Composable Specializers & Modifiers
           #:specialize-terrestrial-se2-vector
           #:specialize-scoot-asymmetric-hip-drag
           #:specialize-supine-grade-and-stair-ratchet
           #:specialize-pronograde-knee-vs-bear-mode))
(in-package :ortho-dsp.terrestrial-xforms)

;; =============================================================================
;; A. FOUR CANONICAL TERRESTRIAL GAIT GENERATORS
;; =============================================================================

;; 1. PRONOGRADE DOG / BEAR CRAWL (Like a dog: feet, legs and arms, belly down)
(defmodifier xform-gait-pronograde-dog-crawl
  [state &key (bpm 102.0) (velocity-vx-vy-wz [0.45 0.0 0.0]) (gait-pattern :diagonal-trot)]
  (let [[phases duty] (case gait-pattern
                        (:diagonal-trot    '([0.00 0.50 0.50 0.00 0.00] 0.54))
                        (:lateral-sequence '([0.00 0.25 0.50 0.75 0.00] 0.76)))]
    (-> state
        (blend-manifold-posture-jet :atlas 'terrestrial-1jet-posture-atlas)
        (drive-5torus-ehresmann-clock
          :master-bpm bpm
          :appendage-duty-pwm duty
          :axial-sled-duty-pwm 0.00
          :phase-5vec phases
          :velocity-twist-vx-vy-wz velocity-vx-vy-wz)
        (align-hinge-joints-to-se2-vector
          :orientation-topology :pronograde-belly-down
          :hindlimb-load-share 0.68)
        (xform-universal-spatial-awareness)
        (specialize-with-self-collision-manifold :min-arm-torso-gap-mm 22.0)
        (enforce-dcm-barrier-zcbf :barrier-gamma 16.0 :min-capture-margin-mm 18.0))))

;; 2. SUPINE BACK-TO-FLOOR CRAWL (Back to the floor, forward/back and up/down on legs and arms)
(defmodifier xform-gait-supine-back-crawl
  [state &key (bpm 86.0)
              (velocity-vx-vy-wz [-0.35 0.0 0.0]) ; Negative vx = feet-first; Positive vx = head-first!
              (bridge-clearance-mm 155.0)]
  (-> state
      (blend-manifold-posture-jet :atlas 'terrestrial-1jet-posture-atlas)
      (drive-5torus-ehresmann-clock
        :master-bpm bpm
        :appendage-duty-pwm 0.66
        :axial-sled-duty-pwm 0.00
        :phase-5vec [0.00 0.50 0.50 0.00 0.00]
        :velocity-twist-vx-vy-wz velocity-vx-vy-wz)
      (align-hinge-joints-to-se2-vector
        :orientation-topology :supine-belly-up
        :hindlimb-load-share 0.65)
      ;; Regulate pelvic bridge height above floor via 1-Jet axial mode:
      (reconstruct-modal-pose
        :mode-axial-compression (- 1.0 (/ bridge-clearance-mm 240.0)))
      (arpeggiate-supine-pronograde-grade :body-orientation :supine-belly-up)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)
      (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 18.0)))

;; 3. ISCHIAL BUTT-SCOOT & DRAG (Scooting / dragging on the buttocks in any direction)
(defmodifier xform-gait-ischial-butt-scoot
  [state &key (bpm 78.0)
              (velocity-vx-vy-wz [0.28 0.0 0.0])  ; Supports forward, backward, or sideways scooting!
              (vault-intensity 0.75)]             ; 1.0 = full lift between hands; 0.30 = continuous butt-drag
  (-> state
      (blend-manifold-posture-jet :atlas 'terrestrial-1jet-posture-atlas)
      (drive-5torus-ehresmann-clock
        :master-bpm bpm
        :appendage-duty-pwm 0.55
        :axial-sled-duty-pwm 0.48               ; Buttocks anchor for 48% of cycle, slide/vault for 52%!
        :phase-5vec [0.00 0.00 0.00 0.00 0.50]  ; Limbs row in sync while pelvis vaults in anti-phase
        :velocity-twist-vx-vy-wz velocity-vx-vy-wz)
      (align-hinge-joints-to-se2-vector
        :orientation-topology :ischial-seated
        :hindlimb-load-share 0.55)
      (modulate-stick-slip-sled-ratchet
        :anchor-patches [:ulnar-carpal-shove-bilateral :plantar-bilateral]
        :sled-patches   [:ischial-bilateral :posterior-femoral-bilateral]
        :vault-unweight-ratio vault-intensity
        :rowing-stroke-length-mm 260.0)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)
      (enforce-dcm-barrier-zcbf :barrier-gamma 14.0 :min-capture-margin-mm 15.0)))

;; 4. LATERAL CRABWALK SIDEWAYS (Works in a supine bridge, pronograde all-fours or a low squat!)
(defmodifier xform-gait-lateral-crabwalk
  [state &key (side-direction :right)            ; :right (vy = -0.38 m/s) | :left (vy = +0.38 m/s)
              (posture-mode :supine-belly-up)    ; :supine-belly-up | :pronograde-belly-down | :deep-biped-squat
              (crab-speed-mps 0.38)
              (bpm 96.0)]
  (let [vy-signed (if (eq side-direction :left) crab-speed-mps (- crab-speed-mps))]
    (-> state
        (blend-manifold-posture-jet :atlas 'terrestrial-1jet-posture-atlas)
        ;; Engage outer-limb-leading coronal phase lock so legs & arms never cross midline:
        (drive-5torus-ehresmann-clock
          :master-bpm bpm
          :appendage-duty-pwm 0.64
          :axial-sled-duty-pwm 0.00
          :phase-5vec (if (eq side-direction :right)
                          [0.50 0.50 0.00 0.00 0.00]  ; Right (outer) limbs step out first at phi=0!
                          [0.00 0.00 0.50 0.50 0.00]) ; Left (outer) limbs step out first at phi=0!
          :velocity-twist-vx-vy-wz [0.0 vy-signed 0.0])
        ;; Rotate hips & shoulders into 45 deg external frog/crab turnout so knee & elbow
        ;; hinge axes align with the lateral Y travel vector - her hips to their 25 deg stop;
        ;; her legs bear what her plumb arms leave them on her back, the dog's stance belly down:
        (align-hinge-joints-to-se2-vector
          :orientation-topology posture-mode
          :max-coxa-turnout-rad 0.436
          :hindlimb-load-share (case posture-mode
                                 (:deep-biped-squat      1.00)
                                 (:pronograde-belly-down 0.27)
                                 (otherwise              0.38)))
        (xform-universal-spatial-awareness)
        (specialize-with-self-collision-manifold :min-malleolus-gap-mm 18.0 :min-arm-torso-gap-mm 24.0)
        (enforce-dcm-barrier-zcbf :barrier-gamma 16.0 :min-capture-margin-mm 20.0))))


;; =============================================================================
;; B. COMPOSABLE SPECIALIZATION EXTENSIONS (PLUG-IN MODIFIERS)
;; =============================================================================

;; 1. Omnidirectional SE(2) Steering & Crab-Spin Specializer
;; Allows any of the 4 gaits above to blend smoothly between forward, backward,
;; sideways crabwalk, and on-the-spot tank-turn spinning (`wz != 0`).
(defmodifier specialize-terrestrial-se2-vector
  [terrestrial-state &key (vx-mps 0.0) (vy-mps 0.0) (wz-radps 0.0)]
  (-> terrestrial-state
      (drive-5torus-ehresmann-clock :velocity-twist-vx-vy-wz [vx-mps vy-mps wz-radps])
      (align-hinge-joints-to-se2-vector :max-coxa-turnout-rad 0.436)))

;; 2. Asymmetric One-Sided Hip/Forearm Drag Specializer ("Injured / Low-Profile Drag")
;; Specializes `xform-gait-ischial-butt-scoot` for scenarios where one arm is carrying
;; an object (or disabled) or ceiling clearance is < 320 mm:
;; Rolls onto one hip/trochanter (`:trochanter-gluteal-lateral`) and drags the body
;; using the ipsilateral forearm (`:olecranon-ulnar-sphinx`) + contralateral heel push!
(defmodifier specialize-scoot-asymmetric-hip-drag
  [scoot-state &key (drag-side :right) (keep-free-hand-for-payload true)]
  (let [active-ulna (if (eq drag-side :right) :olecranon-r :olecranon-l)
        push-heel   (if (eq drag-side :right) :calcaneus-l :calcaneus-r)]
    (-> scoot-state
        (modulate-stick-slip-sled-ratchet
          :anchor-patches [active-ulna push-heel]
          :sled-patches   [:trochanter-gluteal-lateral :ischial-bilateral]
          :vault-unweight-ratio 0.28            ; Continuous low-profile sliding contact!
          :rowing-stroke-length-mm 290.0)
        (when-modifier keep-free-hand-for-payload
          (project-nullspace-posture
            :free-nodes (if (eq drag-side :right) [:glenohumeral-l :cubital-l] [:glenohumeral-r :cubital-r])
            :target-angles '((:glenohumeral-flex 0.65) (:cubital-flex 1.15)))))))

;; 3. Supine Grade & Stair Ratchet Specializer (Crawling up and down stairs and slopes on the back)
;; Couples `sub-tarsal-slab` stair/slope detection with the 5th-torus ischial pawl:
;; On ascent, plants buttocks on each tread so arms never overheat; on descent,
;; skims buttocks 18mm over stair nosings with forefoot + carpal-heel eccentric braking.
(defmodifier specialize-supine-grade-and-stair-ratchet
  [supine-state &key (slope-or-stair-dz-mm +175.0)]
  (-> supine-state
      (arpeggiate-supine-pronograde-grade
        :grade-dz-mm slope-or-stair-dz-mm
        :body-orientation :supine-belly-up)
      ;; Automatically recruit the 5th-torus ischial ratchet when climbing > 80mm risers:
      (when-modifier (> (abs slope-or-stair-dz-mm) 80.0)
        (modulate-stick-slip-sled-ratchet
          :anchor-patches [:plantar-bilateral :ulnar-carpal-shove-bilateral]
          :sled-patches   [:ischial-bilateral]
          :vault-unweight-ratio 0.65
          :rowing-stroke-length-mm 280.0))))

;; 4. Pronograde Knee-Crawl vs. Bear-Crawl Interface Switcher
;; Switches pronograde dog-crawl between fast digitigrade bear-crawl (hands + toes),
;; armoured hands-and-knees crawl (palms + tibial knee-pads), and ultra-low army crawl
;; (forearm ulna blades + tibial knees + ventral belly skim).
(defmodifier specialize-pronograde-knee-vs-bear-mode
  [dog-state &key (mode :hands-and-knees)] ; :digitigrade-bear | :hands-and-knees | :forearm-belly-army-crawl
  (case mode
    (:digitigrade-bear
      (-> dog-state (reconstruct-modal-pose :mode-axial-compression 0.0)))
    (:hands-and-knees
      (-> dog-state
          (bind-wrench-driven-homotopy
            :active-manifolds [:anterior-tibial-bilateral :ulnar-carpal-shove-bilateral])
          (specialize-posture-dangling-crural-hang :passive-ankle-droop-rad -0.35)))
    (:forearm-belly-army-crawl
      (-> dog-state
          (bind-wrench-driven-homotopy
            :active-manifolds [:anterior-tibial-bilateral :olecranon-ulnar-sphinx :ventral-prone-continuum])
          (modulate-stick-slip-sled-ratchet
            :anchor-patches [:olecranon-ulnar-sphinx :anterior-tibial-bilateral]
            :sled-patches   [:ventral-prone-continuum]
            :vault-unweight-ratio 0.35
            :rowing-stroke-length-mm 220.0)))))
