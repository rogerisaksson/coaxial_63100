;; =============================================================================
;; FILE: degraded_scoot_and_crawl_specializers.lisp
;; DOMAIN: General & Degraded Butt-Scoot and Prone Low-Crawl Transformers
;; REQUIRES: degraded_scoot_and_crawl_core.lisp, ortho_dsp_cognitive_autonomy.lisp
;; =============================================================================
(defpackage :ortho-dsp.degraded-crawl-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.somatic-xforms
        :ortho-dsp.postures-core
        :ortho-dsp.posture-xforms
        :ortho-dsp.hitchin
        :ortho-dsp.ground-core
        :ortho-dsp.ground-xforms
        :ortho-dsp.substrate-overlay
        :ortho-dsp.terrestrial-core
        :ortho-dsp.terrestrial-xforms
        :ortho-dsp.prior-observer
        :ortho-dsp.degraded-crawl-core)
  (:export ;; A. Two Core Terrestrial Sled Gaits (Work Nominally AND with Broken Limbs!)
           #:xform-locomotion-seated-butt-scoot
           #:xform-locomotion-prone-low-crawl
           ;; B. Composable Fault, Direction & Splinting Specializers
           #:specialize-with-actuator-fault-injection
           #:specialize-butt-scoot-propulsion-mode
           #:specialize-low-crawl-frog-piston-topology
           #:specialize-with-contralateral-leg-splinting))
(in-package :ortho-dsp.degraded-crawl-xforms)

;; =============================================================================
;; A. CORE SEATED BUTT-SCOOT & PRONE LOW-CRAWL GENERATORS
;; =============================================================================

;; 1. UNIVERSAL SEATED BUTT-SCOOT ("Drag / scoot on the butt with arms and feet")
;; Works seamlessly with 2 healthy legs OR with 1/both legs degraded/broken!
;; - Buttocks (`:ischial-bilateral`) act as the continuous or intermittent Coulomb sled.
;; - Hands (`:ulnar-carpal`) and available heel(s) (`:plantar`) act as synchronized
;;   or alternating rowing anchors.
(defmodifier xform-locomotion-seated-butt-scoot
  [state &key (velocity-vx-vy-wz [0.25 0.0 0.0]) ; +vx = Forward scoot, -vx = Backward push-scoot!
              (bpm 76.0)
              (buttock-unweight-ratio 0.65)]     ; 0.30 = heavy continuous butt-drag; 0.85 = high vault
  (-> state
      (xform-prior-regularized-observer)
      (partition-fault-tolerant-dynamics :health-mask 'actuator-health-soa-mask)
      (blend-manifold-posture-jet :atlas 'scoot-and-lowcrawl-1jet-atlas)
      (drive-5torus-ehresmann-clock
        :master-bpm bpm
        :appendage-duty-pwm 0.56
        :axial-sled-duty-pwm 0.48
        :phase-5vec [0.00 0.00 0.00 0.00 0.50]   ; Active limbs row at 0.0, buttocks anchor at 0.50
        :velocity-twist-vx-vy-wz velocity-vx-vy-wz)
      (modulate-stick-slip-sled-ratchet
        :anchor-patches [:ulnar-carpal-shove-bilateral :plantar-bilateral]
        :sled-patches   [:ischial-bilateral :posterior-femoral-bilateral]
        :vault-unweight-ratio buttock-unweight-ratio
        :rowing-stroke-length-mm 250.0)
      (nullify-asymmetric-propulsion-yaw
        :desired-yaw-rate-radps (elt velocity-vx-vy-wz 2)
        :allow-contralateral-arm-overdrive true
        :allow-diagonal-foot-vectoring true)
      (escort-passive-towed-limb :towing-strategy :pelvic-roll-unweight)
      (xform-hitchin-integrable-balance)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)))

;; 2. UNIVERSAL PRONE LOW-CRAWL ("Low-crawl on the belly with forearms and legs/feet")
;; Keeps the entire body silhouette below Z = 260 mm!
;; - Belly and pelvis (`:ventral-prone-continuum`) slide along the floor.
;; - Elbows/forearms (`:olecranon-ulnar-sphinx`) pull in front while externally rotated
;;   frog-legs (`:medial-crural-frog-piston` + `:metatarsal-equinus`) push from the sides!
(defmodifier xform-locomotion-prone-low-crawl
  [state &key (velocity-vx-vy-wz [0.22 0.0 0.0])
              (bpm 68.0)
              (belly-unweight-ratio 0.45)]       ; Keeps chest/pelvis skimming low to the ground; her anchors hold her belly from 0.441 of her (0.42)
  (-> state
      (xform-prior-regularized-observer)
      (partition-fault-tolerant-dynamics :health-mask 'actuator-health-soa-mask)
      (blend-manifold-posture-jet :atlas 'scoot-and-lowcrawl-1jet-atlas)
      ;; Diagonal 2-beat reptile/leopard phase: Left Forearm + Right Frog-Leg pull/push together,
      ;; then Right Forearm + Left Frog-Leg, while Axial belly sled modulates at 2x frequency!
      (drive-5torus-ehresmann-clock
        :master-bpm bpm
        :appendage-duty-pwm 0.64
        :axial-sled-duty-pwm 0.50
        :phase-5vec [0.00 0.50 0.50 0.00 0.25]
        :velocity-twist-vx-vy-wz velocity-vx-vy-wz)
      (modulate-stick-slip-sled-ratchet
        :anchor-patches [:olecranon-ulnar-sphinx
                         :medial-crural-frog-piston
                         :metatarsal-equinus-bilateral]
        :sled-patches   [:ventral-prone-continuum]
        :vault-unweight-ratio belly-unweight-ratio
        :rowing-stroke-length-mm 230.0)
      (nullify-asymmetric-propulsion-yaw
        :desired-yaw-rate-radps (elt velocity-vx-vy-wz 2)
        :allow-contralateral-arm-overdrive true
        :allow-diagonal-foot-vectoring true)
      (escort-passive-towed-limb :towing-strategy :pelvic-roll-unweight)
      (xform-hitchin-integrable-balance)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)))


;; =============================================================================
;; B. COMPOSABLE FAULT, DIRECTION & SPLINTING SPECIALIZERS (PLUG-IN EXTENSIONS)
;; =============================================================================

;; 1. Actuator Fault / Degradation Specializer (Broken Leg or Arm Handler)
;; Can be applied to EITHER Butt-Scoot or Low-Crawl (or omitted when 100% healthy!).
;; Supports:
;;   - `:high-z-freewheel`    (Burned inverter / zero torque / floppy joint)
;;   - `:mechanically-jammed` (Seized gearbox / locked angle)
;;   - `:thermal-derated`     (Partial torque authority, e.g., alpha = 0.25)
;; Hers: her floppy joint still holds its backdrive; her derate is her thermal envelope's.
(defmodifier specialize-with-actuator-fault-injection
  [crawl-state &key (affected-limb :hind-left)   ; :hind-left | :hind-right | :fore-left | :fore-right
                    (fault-type :high-z-freewheel)
                    (residual-authority 0.00)]
  (-> crawl-state
      (set-limb-health-mask
        :mask 'actuator-health-soa-mask
        :limb affected-limb
        :authority residual-authority
        :fault-mode fault-type)
      (partition-fault-tolerant-dynamics :health-mask 'actuator-health-soa-mask)
      ;; Automatically tilt pelvis/thorax 12 deg away from the broken limb so it drags
      ;; with minimal normal force Fz_dead:
      (escort-passive-towed-limb
        :towing-strategy (if (eq fault-type :high-z-freewheel)
                             :proximal-hip-sling
                             :pelvic-roll-unweight)
        :coronal-unweight-tilt-rad (case affected-limb
                                     ((:hind-left :fore-left)   +0.22)
                                     ((:hind-right :fore-right) -0.22)))))

;; 2. Butt-Scoot Propulsion Technique Specializer
;; Specializes how the hands and feet coordinate during Seated Butt-Scoot:
;; - `:anterograde-hamstring-row` -> Pulling forward with heels + pushing behind hips with hands
;; - `:retrograde-quad-push`      -> Pushing backward powerfully with legs + catching with hands
;; - `:lateral-hip-hitch-scoot`   -> Sideways scoot on one buttock + forearm (combat side-drag)
(defmodifier specialize-butt-scoot-propulsion-mode
  [scoot-state &key (technique :anterograde-hamstring-row)]
  (case technique
    (:anterograde-hamstring-row
      (-> scoot-state
          (reconstruct-modal-pose :mode-sagittal-shift +0.65 :mode-pitch-attitude +0.18)))

    (:retrograde-quad-push
      (-> scoot-state
          (specialize-terrestrial-se2-vector :vx-mps -0.32 :vy-mps 0.0 :wz-radps 0.0)
          (reconstruct-modal-pose :mode-sagittal-shift -0.70 :mode-pitch-attitude -0.12)))

    (:lateral-hip-hitch-scoot
      (-> scoot-state
          (specialize-scoot-asymmetric-hip-drag :drag-side :right :keep-free-hand-for-payload false)))))

;; 3. Prone Low-Crawl Frog-Piston Topology Specializer
;; Adapts the prone low-crawl leg drive between:
;; - `:alternating-diagonal-leopard` (Both legs healthy: left/right frog-knees alternate with elbows)
;; - `:unilateral-single-piston`     (One leg broken: both forearms pull in sync while the
;;                                    single healthy leg drives as a centered frog-piston!)
;; - `:arms-only-seal-drag`          (Both legs dead/paralyzed: pure bimanual forearm drag
;;                                    with passive lower body trailing in streamlined extension!)
(defmodifier specialize-low-crawl-frog-piston-topology
  [crawl-state &key (drive-topology :alternating-diagonal-leopard)]
  (case drive-topology
    (:alternating-diagonal-leopard
      (-> crawl-state
          (drive-5torus-ehresmann-clock :phase-5vec [0.00 0.50 0.50 0.00 0.25])))

    (:unilateral-single-piston
      (-> crawl-state
          ;; Synchronize both forearms with the single working hindlimb at phi = 0.0:
          (drive-5torus-ehresmann-clock :phase-5vec [0.00 0.00 0.00 0.00 0.50])
          (nullify-asymmetric-propulsion-yaw :allow-diagonal-foot-vectoring true)))

    (:arms-only-seal-drag
      (-> crawl-state
          (set-limb-health-mask :mask 'actuator-health-soa-mask :limb :hind-left  :authority 0.0 :fault-mode :high-z-freewheel)
          (set-limb-health-mask :mask 'actuator-health-soa-mask :limb :hind-right :authority 0.0 :fault-mode :high-z-freewheel)
          (partition-fault-tolerant-dynamics)
          (modulate-stick-slip-sled-ratchet
            :anchor-patches [:olecranon-ulnar-sphinx :ulnar-carpal-shove-bilateral]
            :sled-patches   [:ventral-prone-continuum :anterior-tibial-bilateral]
            :vault-unweight-ratio 0.51          ; her forearms hold her belly and shins from 0.509 of her (0.48)
            :rowing-stroke-length-mm 270.0)))))

;; 4. Self-Splinting / Cross-Ankle Hook Specializer (Carry Dead Leg Off the Floor!)
;; When a leg actuator is completely dead (`:high-z-freewheel`) on rough terrain or gravel
;; where dragging the dead foot would snag on rocks, uses the healthy leg (or hands prior
;; to crawling) to hook the dead ankle OVER the healthy shin/achilles or into a figure-4
;; pelvic lock so the dead limb is carried 100% off the ground!
(defmodifier specialize-with-contralateral-leg-splinting
  [crawl-state &key (broken-leg :hind-left)]
  (let [dead-chain (if (eq broken-leg :hind-left)
                       [:coxofemoral-l :tibiofemoral-l :talocrural-l]
                       [:coxofemoral-r :tibiofemoral-r :talocrural-r])]
    (-> crawl-state
        (escort-passive-towed-limb :towing-strategy :contralateral-ankle-splint)
        ;; Lock passive figure-4 / cross-ankle hook in dynamically consistent Null-Space:
        (project-nullspace-posture
          :free-nodes dead-chain
          :target-angles '((:coxofemoral-flex 0.85)
                           (:coxofemoral-ext-rot 0.436) ; hers at her hip yaw's stop (0.72)
                           (:coxofemoral-adduct 0.48)
                           (:tibiofemoral-flex 1.25))
          :nullspace-leakage-guard 0.0))))
