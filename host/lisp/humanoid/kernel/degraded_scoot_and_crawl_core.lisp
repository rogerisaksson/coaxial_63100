;; =============================================================================
;; FILE: degraded_scoot_and_crawl_core.lisp
;; DOMAIN: Fault-Tolerant Schur-Complement Null-Space, Asymmetric Yaw
;;         Cancellation, Seated Butt-Scoot & Prone Leopard Low-Crawl Physics
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp,
;;           terrestrial_multimodal_gaits.lisp, prior_regularized_observer.lisp
;; Hers: fitted to her body (`machine.moves.crawl`, test_crawl): her hip turns out 25 deg at its stop, her shoulders do not
;; abduct; a drive in High-Z still holds its gearbox's backdrive (1.2-2.2 N m, `drives.backdrive`). None of it on
;; her loop.
;; =============================================================================
(defpackage :ortho-dsp.degraded-crawl-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.terrestrial-core
        :ortho-dsp.prior-observer)
  (:export #:actuator-health-soa-mask
           #:low-crawl-manifold-extensions
           #:scoot-and-lowcrawl-1jet-atlas
           #:partition-fault-tolerant-dynamics
           #:nullify-asymmetric-propulsion-yaw
           #:escort-passive-towed-limb))
(in-package :ortho-dsp.degraded-crawl-core)

;; -----------------------------------------------------------------------------
;; 1. ACTUATOR HEALTH & FAULT-MODE SOA MASK (1 kHz HARDWARE TELEMETRY)
;; Tracks live authority alpha_i in [0..1] and fault topology per joint/limb.
;; Nominal operation has authority == 1.0 and mode == :healthy-closed-loop.
;; Hers: her 63100's gates off is :high-z-freewheel - her joint then turns against its gearbox's backdrive alone.
;; -----------------------------------------------------------------------------
(defstate actuator-health-soa-mask
  (:memory-layout
    :num-limb-chains 4                       ; [:hind-left, :hind-right, :fore-left, :fore-right]
    :align-bytes 64)
  (:limb-status-defaults
    :hind-left  (:authority 1.00 :fault-mode :healthy-closed-loop) ; or :high-z-freewheel | :mechanically-jammed
    :hind-right (:authority 1.00 :fault-mode :healthy-closed-loop)
    :fore-left  (:authority 1.00 :fault-mode :healthy-closed-loop)
    :fore-right (:authority 1.00 :fault-mode :healthy-closed-loop)))

;; -----------------------------------------------------------------------------
;; 2. LOW-CRAWL & SCOOT SOMATIC MANIFOLD EXTENSIONS
;; Adds the Medial Tibial/Epicondyle Frog-Piston patch used when pushing with
;; the inside of the leg flattened against the ground during Prone Low-Crawl!
;; Hers: on her shank, 380 mm knee to ankle, its capsule 50 mm round.
;; -----------------------------------------------------------------------------
(defcontact-patch low-crawl-manifold-extensions
  (:patch :medial-crural-frog-piston
    :frames          [:medial-condyle-l :medial-malleolus-l :medial-condyle-r :medial-malleolus-r]
    :local-normal    [0.0 1.0 0.0]           ; Medial side of knee/shin faces [0,0,1] when hip externally rotated 75 deg! Hers 25 deg
    :cop-bounds-mm   (:x [-180.0 180.0] :y [-45.0 45.0])
    :default-mu      0.78
    :unilateral-fz   true))

;; -----------------------------------------------------------------------------
;; 3. 1-JET POSTURE ATLAS FOR BUTT-SCOOT & PRONE LOW-CRAWL (LEOPARD CRAWL)
;; -----------------------------------------------------------------------------
(defstate scoot-and-lowcrawl-1jet-atlas
  (:postures
    ;; A. SEATED BUTT-SCOOT SLED ("Scoot butt down", Arms + Heels + Buttocks)
    (:posture :seated-butt-scoot-sled
      :coupled-manifold [:ischial-bilateral :posterior-femoral-bilateral
                         :plantar-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.22 :tibiofemoral 1.32 :coxofemoral 1.38
         :iliac-coronal 0.00 :l4-l5-lumbar +0.12 :c7-t12-thoracic +0.18
         :glenohumeral -0.38 :glenohumeral-abduct 0.00 :cubital 0.55 :c1-c2-atlas -0.10) ; her shoulders do not abduct (0.28)
      :modal-basis-columns
        ;; 1. Axial Compression: Presses hands and active heel(s) down to unweight the buttocks!
        (:mode-axial-compression (:glenohumeral -0.42 :cubital -0.52
                                  :tibiofemoral -0.25 :coxofemoral -0.30 :l4-l5-lumbar -0.15)
         ;; 2. Sagittal Shift: Drives the rowing stroke (hamstring pull + triceps push):
         :mode-sagittal-shift    (:tibiofemoral +0.75 :coxofemoral -0.60
                                  :glenohumeral +0.68 :cubital -0.35)
         ;; 3. Pitch Attitude: Tilts pelvis/torso to unweight a broken leg on one side:
         :mode-pitch-attitude    (:l4-l5-lumbar +0.35 :c7-t12-thoracic +0.30 :c1-c2-atlas -0.45)))

    ;; B. PRONE LEOPARD LOW-CRAWL ("Low-crawl on the belly", Forearms + Flattened Frog-Legs + Belly)
    (:posture :prone-leopard-low-crawl
      :coupled-manifold [:ventral-prone-continuum :olecranon-ulnar-sphinx :medial-crural-frog-piston]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.35       ; Dorsiflex ankle so medial big-toe/arch grips floor!
         :tibiofemoral 1.15                      ; Flex knee 66 deg out to the side
         :coxofemoral 0.78 :coxofemoral-abduct 0.88 :coxofemoral-ext-rot 0.436 ; 75 deg frog turnout flat to floor! Hers 25 deg, her hip yaw's stop (1.31)
         :iliac-coronal 0.00 :l4-l5-lumbar -0.05 :c7-t12-thoracic +0.08
         :glenohumeral +1.65 :glenohumeral-abduct 0.00 :cubital 1.45 ; Forearms flat in front of chest; her shoulders do not abduct (0.65)
         :c1-c2-atlas -0.42 :c1-c2-atlas-yaw 0.52) ; Head low, chin cleared to side, eyes forward!
      :modal-basis-columns
        ;; 1. Axial Compression: Shifts load between ventral belly sled and elbows/inner-knees:
        (:mode-axial-compression (:glenohumeral -0.35 :cubital +0.40 :coxofemoral-abduct +0.25)
         ;; 2. Sagittal Shift: Horizontal piston stroke! Pulls elbows back under ribs while
         ;;    extending the externally rotated frog-leg along the floor plane:
         :mode-sagittal-shift    (:glenohumeral -0.75 :cubital +0.55
                                  :coxofemoral -0.82 :tibiofemoral -0.90 :l4-l5-lumbar +0.28)
         ;; 3. Pitch Attitude: Lateral spinal undulation (salamander C-bend) to extend reach!
         :mode-pitch-attitude    (:iliac-coronal +0.35 :c1-c2-atlas -0.30)))))

;; -----------------------------------------------------------------------------
;; 4. FAULT-TOLERANT SCHUR-COMPLEMENT DYNAMICS & WRENCH PARTITIONER
;; Automatically adapts `allocate-whole-body-torque` and the CWC QP when one or
;; more limb joints have reduced authority (alpha_i < 1) or are dead/jammed:
;;   - `:high-z-freewheel`    -> Eliminates joint via Schur complement M_ff^-1;
;;                               sets its active propulsive wrench bound to 0 N.
;;   - `:mechanically-jammed` -> Zeroes column in J_c; uses locked limb as a
;;                               passive rigid skid or hook if geometry permits!
;; Hers: a freewheeling joint of hers keeps its backdrive's friction in c_f.
;; -----------------------------------------------------------------------------
(defprimitive partition-fault-tolerant-dynamics
  [state &key (health-mask 'actuator-health-soa-mask)
              (min-active-authority-floor 0.08)]
  (-> state
      (reconfigure-underactuated-operational-space
        :health-source health-mask
        :freewheel-threshold min-active-authority-floor
        ;; 1. Schur-Complement Reduction for High-Z / Dead Joints:
        ;;    M_bar_aa = M_aa - M_af * M_ff^-1 * M_fa
        :schur-complement-passive-elimination true
        ;; 2. Jammed-Joint Jacobian Deflation:
        ;;    J_eff = J_c * (I - P_jammed)
        :deflate-jammed-jacobian-columns true
        ;; 3. Contact Wrench Cone (CWC) Re-Bound:
        ;;    Dead limbs cannot generate active push/pull shear, only passive Coulomb drag!
        :passive-limb-wrench-law :coulomb-drag-only)))

;; -----------------------------------------------------------------------------
;; 5. ASYMMETRIC PROPULSION YAW-MOMENT NULLIFIER (STRAIGHT-LINE 3-LIMB DRIVE)
;; When pushing with 2 arms + only 1 working leg (or 1 arm + 2 legs), cancels
;; the parasitic yaw torque tau_z = r_x * F_y - r_y * F_x so the robot scoots
;; or crawls straight ahead without spinning in circles!
;; -----------------------------------------------------------------------------
(defprimitive nullify-asymmetric-propulsion-yaw
  [state &key (desired-yaw-rate-radps 0.0)
              (allow-contralateral-arm-overdrive true)
              (allow-diagonal-foot-vectoring true)]
  (-> state
      (enforce-zero-net-yaw-wrench-allocation
        :target-wz-radps desired-yaw-rate-radps
        :sled-pivot-patches [:ischial-bilateral :ventral-prone-continuum]
        :compensation-mechanisms
          ;; A. Diagonal Coupling: Boost stroke & force on the arm opposite the working leg
          ;;    Hers: her hands 148 mm off her midline, her heel 30 mm once shifted, pushing as an arm (1.45)
          ((:when allow-contralateral-arm-overdrive
             :boost-ipsilateral-to-fault-arm-gain 1.20)
           ;; B. Centerline Foot/Knee Placement: Plant the single working heel/knee closer to
           ;;    the sagittal midline so its push vector passes through the sled's CoM!
           ;;    Hers: her heel 95 mm off her midline, her hips' half width
           (:when allow-diagonal-foot-vectoring
             :shift-working-hindlimb-toward-midline-mm 65.0)
           ;; C. Lateral Spine C-Arch Counter-Torque:
           (:use-lumbar-coronal-rudder true)))))

;; -----------------------------------------------------------------------------
;; 6. PASSIVE / BROKEN LIMB TOWING ESCORT (ANTI-FOLDING & CABLE PROTECTION)
;; Prevents a High-Z or jammed leg from folding underneath the sliding buttocks
;; or belly during forward scooting or low-crawling.
;; -----------------------------------------------------------------------------
(defprimitive escort-passive-towed-limb
  [state &key (towing-strategy :pelvic-roll-unweight) ; :pelvic-roll-unweight | :contralateral-ankle-splint | :proximal-hip-sling
              (coronal-unweight-tilt-rad 0.22)]
  (-> state
      (guard-underactuated-appendage-envelope
        :health-source 'actuator-health-soa-mask
        :strategy towing-strategy
        ;; 1. Pelvic/Thoracic Roll Tilt: Tilt 12.5 deg onto the healthy hip/buttock so the
        ;;    broken side is lifted 35mm clear of floor friction! Hers 35.6 mm, her far hip 163 mm across
        :unweight-roll-bias-rad coronal-unweight-tilt-rad
        ;; 2. Proximal Hip Sling: If only the knee/ankle actuator is broken but the hip
        ;;    (`:coxofemoral`) still works, hold the thigh slightly abducted/elevated:
        :use-surviving-proximal-joints-to-lift true
        ;; 3. Capsule Self-Collision Barrier: Ensure the working leg never kicks or crushes
        ;;    the passive towed leg during its power stroke!
        :inflate-passive-limb-ecbf-margin-mm 32.0)))
