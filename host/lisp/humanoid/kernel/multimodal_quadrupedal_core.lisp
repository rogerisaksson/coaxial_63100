;; =============================================================================
;; FILE: multimodal_quadrupedal_core.lisp
;; DOMAIN: T^4 Coupled Phase Clock, Limb-Length Equalization & Virtual Truss QP
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, canonical_posture_bundle.lisp
;; =============================================================================
(defpackage :ortho-dsp.quad-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core)
  (:export #:quadrupedal-posture-extensions
           #:configure-4torus-symplectic-clock
           #:equalize-heterogeneous-limb-kinematics
           #:project-virtual-truss-internal-wrenches
           #:couple-fifth-limb-axial-peristalsis))
(in-package :ortho-dsp.quad-core)

;; -----------------------------------------------------------------------------
;; 1. EXTENDED 1-JET POSTURE ATLAS FOR PRONOGRADE & SUPINE QUADRUPEDAL REGIMES
;; Adds canonical origins q_0 and 3-column modal bases for belly-down (dog/bear)
;; and belly-up (supine crab/inverted) quadrupedal stances.
;; -----------------------------------------------------------------------------
(defstate quadrupedal-posture-extensions
  (:postures
    ;; A. PRONOGRADE ALL-FOURS (Belly down: hands/carpal-heels + equinus feet or knees)
    (:posture :pronograde-quadruped
      :coupled-manifold [:metatarsal-equinus-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.42 :tibiofemoral 1.48 :coxofemoral 1.32
         :iliac-coronal 0.00 :l4-l5-lumbar +0.18 :c7-t12-thoracic +0.24
         :glenohumeral +1.38 :cubital 0.22 :c1-c2-atlas -0.95) ; Extend neck to look forward along floor!
      :modal-basis-columns
        ;; Axial compression crouches all 4 limbs closer to the ground (under-table clearance):
        (:mode-axial-compression (:tibiofemoral +0.45 :coxofemoral +0.38
                                  :glenohumeral +0.25 :cubital +0.85 :l4-l5-lumbar +0.15)
         ;; Sagittal shift transfers weight between hindlimbs (pelvis) and forelimbs (shoulders):
         :mode-sagittal-shift    (:coxofemoral -0.45 :tibiofemoral -0.30
                                  :glenohumeral +0.42 :cubital -0.15)
         ;; Pitch attitude tilts the trunk line (uphill/downhill crawl or rear-up):
         :mode-pitch-attitude    (:coxofemoral +0.55 :glenohumeral -0.55 :c1-c2-atlas -0.50)))

    ;; B. SUPINE INVERTED CRAB (Belly up / on back: heels/feet + externally rotated palms)
    (:posture :supine-inverted-quadruped
      :coupled-manifold [:plantar-bilateral :ulnar-carpal-shove-bilateral]
      :origin-q0
        (:calcaneus 0.00 :talocrural +0.25 :tibiofemoral 1.35 :coxofemoral 0.42
         :iliac-coronal 0.00 :l4-l5-lumbar -0.12 :c7-t12-thoracic -0.08
         :glenohumeral -0.82 :glenohumeral-ext-rot +1.15 :cubital 0.32
         :c1-c2-atlas +0.65) ; Flex neck forward (+pitch) so cameras look across the abdomen/horizon!
      :modal-basis-columns
        ;; Axial compression lowers suspended pelvis toward floor (0.0 = high bridge, 1.0 = buttock skim):
        (:mode-axial-compression (:coxofemoral +0.75 :tibiofemoral +0.40
                                  :glenohumeral -0.25 :cubital +0.78 :l4-l5-lumbar +0.22)
         ;; Sagittal shift rocks suspended pelvis between heels and carpal heels:
         :mode-sagittal-shift    (:talocrural +0.35 :tibiofemoral -0.42
                                  :glenohumeral +0.38 :cubital +0.20)
         :mode-pitch-attitude    (:coxofemoral -0.45 :l4-l5-lumbar -0.25 :c1-c2-atlas +0.40)))))

;; -----------------------------------------------------------------------------
;; 2. SYMPLECTIC 4-TORUS OSCILLATOR NETWORK (GOLUBITSKY-STEWART D_4 CLOCK)
;; Drives 4 independent limb phases [LH, LF, RH, RF] on T^4 with continuous
;; phase-offset vector `delta-psi-rad` and per-gait duty factor `stance-duty`.
;; -----------------------------------------------------------------------------
(defprimitive configure-4torus-symplectic-clock
  [state &key (master-bpm 96.0)
              (stance-duty-pwm 0.72)            ; 0.75+ = 3-limb static stability; 0.50 = dynamic trot
              (phase-offsets-lh-lf-rh-rf [0.00 0.25 0.50 0.75]) ; Normalized [0..1] cycle offsets
              (kuramoto-coupling-gain 8.5)
              (propulsion-vector-sign +1.0)]    ; +1.0 = cranial-first, -1.0 = podalic/feet-first crab
  (-> state
      (bind-coupled-torus-oscillators
        :torus-dimension 4
        :limb-channels
          ((:hind-left  :frames [:coxofemoral-l :tibiofemoral-l :calcaneus-l]
                        :phase-offset (elt phase-offsets-lh-lf-rh-rf 0))
           (:fore-left  :frames [:glenohumeral-l :cubital-l :palmar-carpal-l]
                        :phase-offset (elt phase-offsets-lh-lf-rh-rf 1))
           (:hind-right :frames [:coxofemoral-r :tibiofemoral-r :calcaneus-r]
                        :phase-offset (elt phase-offsets-lh-lf-rh-rf 2))
           (:fore-right :frames [:glenohumeral-r :cubital-r :palmar-carpal-r]
                        :phase-offset (elt phase-offsets-lh-lf-rh-rf 3)))
        :bpm master-bpm
        :duty-cycle stance-duty-pwm
        :sync-stiffness-k kuramoto-coupling-gain
        :sagittal-direction-sign propulsion-vector-sign
        ;; Automatically scale each limb's contact manifold weight w_a(phi_a) by its stance window
        ;; multiplied by live Generalized Momentum Observer normal force Fz_a!
        :phase-to-wrench-homotopy :gmo-gated-raised-cosine)))

;; -----------------------------------------------------------------------------
;; 3. HETEROGENEOUS LIMB-LENGTH EQUALIZER & SCAPULAR/PELVIC ROLL COMPENSATOR
;; Resolves the human leg-vs-arm length disparity (0.89m vs 0.58m) by regulating
;; virtual leg length L_virt(q) so the trunk maintains target pitch without
;; hyperextending shoulders or bottoming out knee flexion limits.
;; -----------------------------------------------------------------------------
(defprimitive equalize-heterogeneous-limb-kinematics
  [state &key (orientation-regime :pronograde-belly-down) ; :pronograde-belly-down | :supine-belly-up
              (hindlimb-contact-mode :equinus-digitigrade) ; :equinus-digitigrade | :plantigrade-flat | :tibial-knee-pad
              (forelimb-contact-mode :carpal-heel-palm)    ; :carpal-heel-palm | :ulnar-forearm-plank
              (target-trunk-height-z-mm 440.0)
              (target-trunk-pitch-rad -0.08)]
  (-> state
      (solve-virtual-limb-length-matching
        :orientation orientation-regime
        :hind-interface hindlimb-contact-mode
        :fore-interface forelimb-contact-mode
        :nominal-pelvis-z-m (* 0.001 target-trunk-height-z-mm)
        :nominal-thorax-pitch-rad target-trunk-pitch-rad
        ;; Map swing-clearance contours according to joint knee/elbow chirality:
        :swing-kinematic-chirality
          (:hind-knee-bend-direction (case orientation-regime
                                       (:pronograde-belly-down :anterior-cranial)
                                       (:supine-belly-up       :dorsal-zenith))
           :fore-elbow-bend-direction (case orientation-regime
                                        (:pronograde-belly-down :posterior-caudal)
                                        (:supine-belly-up       :caudal-outward-splay))
           :carpal-external-yaw-rad   (case orientation-regime
                                        (:pronograde-belly-down 0.26)   ; 15 deg outward turnout
                                        (:supine-belly-up       1.85))) ; 106 deg fingers-out/back turnout!
        ;; Counter-rotate pelvic and thoracic roll (coronal LFO) during diagonal support
        ;; so the suspended spine never twists past intervertebral torque limits:
        :axial-torsion-damping
          (:couple-iliac-to-thoracic-roll true
           :max-intervertebral-twist-rad 0.22))))

;; -----------------------------------------------------------------------------
;; 4. VIRTUAL-TRUSS INTERNAL WRENCH PROJECTOR & HINDLIMB THERMAL BIAS
;; Eliminates internal shear fighting between grounded hands and feet in ker(G_quad)
;; and shifts 65-72% of gravitational load onto the larger lower-body actuators.
;; -----------------------------------------------------------------------------
(defprimitive project-virtual-truss-internal-wrenches
  [state &key (hindlimb-load-share-bias 0.68)   ; 68% weight on legs, 32% on arms!
              (internal-truss-tension-n 0.0)    ; 0.0 N = zero internal floor-scrubbing shear
              (protect-glenohumeral-windings true)]
  (-> state
      (configure-multi-limb-wrench-allocation
        :grasp-matrix-topology :quadrupedal-virtual-truss
        ;; Shift Poinsot central axis posteriorly toward the pelvic girdle so smaller
        ;; shoulder/elbow motors never saturate thermally during sustained crawling:
        :sagittal-load-distribution
          (:hind-to-fore-weight-ratio hindlimb-load-share-bias
           :thermal-adaptive-shift    protect-glenohumeral-windings
           :max-forelimb-fz-ratio     0.42)
        ;; Constrain internal tangential forces along inter-limb edges (p_a - p_b):
        :nullspace-internal-wrench-law
          (:edge-tension-target-n internal-truss-tension-n
           :minimize-tangential-mu-utilization true))))

;; -----------------------------------------------------------------------------
;; 5. FIFTH-LIMB AXIAL PERISTALSIS (INTERMITTENT PELVIC / DORSAL BRIDGE-SCOOT)
;; Couples `:ischial-bilateral` or `:dorsal-supine-continuum` as a 5th phase-locked
;; oscillator for ultra-low-clearance supine/seated scooting or army crawling.
;; -----------------------------------------------------------------------------
(defprimitive couple-fifth-limb-axial-peristalsis
  [state &key (axial-patch :ischial-bilateral)
              (bridge-lift-clearance-mm 38.0)
              (axial-duty-cycle 0.45)]
  (-> state
      (splice-fifth-peristaltic-oscillator
        :patch axial-patch
        :phase-lock-to-torus :antiphase-to-limb-swing
        :ground-dwell-duty axial-duty-cycle
        :aerial-bridge-lift-m (* 0.001 bridge-lift-clearance-mm)
        ;; When pelvis rests on floor (`w_ischial > 0.7`), release all 4 limbs simultaneously
        ;; in dynamically consistent Null-Space to advance them rapidly with zero balance risk!
        :simultaneous-limb-advance-on-axial-stance true)))
