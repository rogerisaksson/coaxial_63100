;; =============================================================================
;; FILE: multimodal_gait_specializers.lisp
;; DOMAIN: General Non-Bipedal Gaits & Composable Topological Specializers
;; REQUIRES: multimodal_quadrupedal_core.lisp, ortho_dsp_cognitive_autonomy.lisp
;; =============================================================================
(defpackage :ortho-dsp.quad-xforms
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
        :ortho-dsp.quad-core)
  (:export ;; A. Core Multi-Limbed Gait Transformers
           #:xform-gait-pronograde-quadruped
           #:xform-gait-supine-inverted-crab
           ;; B. Composable Phase, Contact & Clearance Specializers
           #:specialize-quad-phase-topology
           #:specialize-pronograde-contact-interface
           #:specialize-supine-peristaltic-underbed-scoot
           #:specialize-quad-steep-grade-scramble
           #:specialize-transition-biped-to-quadruped))
(in-package :ortho-dsp.quad-xforms)

;; =============================================================================
;; A. BASELINE MULTI-LIMBED LOCOMOTION TRANSFORMERS
;; =============================================================================

;; 1. PRONOGRADE QUADRUPEDAL GAIT ("Dog / Bear" All-Fours: Hands + Feet/Knees, Belly Down)
;; - Uses `:pronograde-quadruped` 1-Jet posture
;; - Equalizes long legs vs. short arms via digitigrade/knee flexion
;; - Biases 68% of mass onto hindlimbs to protect shoulder/elbow windings
;; - Enforces 22-capsule ECBF self-collision (knees never strike elbows during mid-swing!)
(defmodifier xform-gait-pronograde-quadruped
  [state &key (bpm 98.0) (stride-length-mm 460.0) (clearance-z-mm 45.0)]
  (-> state
      (blend-manifold-posture-jet :atlas 'quadrupedal-posture-extensions)
      (configure-4torus-symplectic-clock
        :master-bpm bpm
        :stance-duty-pwm 0.72
        :phase-offsets-lh-lf-rh-rf [0.00 0.25 0.50 0.75] ; Default: stable lateral-sequence walk
        :propulsion-vector-sign +1.0)
      (equalize-heterogeneous-limb-kinematics
        :orientation-regime :pronograde-belly-down
        :hindlimb-contact-mode :equinus-digitigrade
        :forelimb-contact-mode :carpal-heel-palm
        :target-trunk-height-z-mm 460.0
        :target-trunk-pitch-rad -0.06)
      (project-virtual-truss-internal-wrenches
        :hindlimb-load-share-bias 0.68
        :internal-truss-tension-n 0.0)
      (modulate-node :tarsal-sagittal
        :stride-length-mm (:set stride-length-mm)
        :swing-clearance-z-mm (:set clearance-z-mm))
      ;; Guard knee-patellar vs. cubital-elbow self-collision during ipsilateral/diagonal swing:
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold :min-arm-torso-gap-mm 22.0)
      (enforce-dcm-barrier-zcbf :barrier-gamma 16.0 :min-capture-margin-mm 20.0)))

;; 2. SUPINE INVERTED CRAB GAIT ("On the back with feet, legs and arms", Belly Up)
;; - Uses `:supine-inverted-quadruped` 1-Jet posture with external shoulder/wrist turnout
;; - Supports both cranial-first (head-leading) and podalic-first (feet-leading) vectors
;; - Keeps head flexed (+0.65 rad) with horizon gaze stabilization across the torso
(defmodifier xform-gait-supine-inverted-crab
  [state &key (bpm 84.0)
              (travel-direction :podalic-feet-first) ; :podalic-feet-first | :cranial-head-first | :coronal-crab-sidestep
              (pelvic-bridge-clearance-mm 165.0)]
  (let [dir-sign (if (eq travel-direction :cranial-head-first) +1.0 -1.0)]
    (-> state
        (blend-manifold-posture-jet :atlas 'quadrupedal-posture-extensions)
        (configure-4torus-symplectic-clock
          :master-bpm bpm
          :stance-duty-pwm 0.68
          :phase-offsets-lh-lf-rh-rf [0.00 0.50 0.50 0.00] ; Diagonal trot pairing minimizes pelvic twist!
          :propulsion-vector-sign dir-sign)
        (equalize-heterogeneous-limb-kinematics
          :orientation-regime :supine-belly-up
          :hindlimb-contact-mode :plantigrade-flat
          :forelimb-contact-mode :carpal-heel-palm
          :target-trunk-height-z-mm pelvic-bridge-clearance-mm
          :target-trunk-pitch-rad +0.05)
        (project-virtual-truss-internal-wrenches
          :hindlimb-load-share-bias 0.64        ; Heels carry 64%, carpal heels carry 36%
          :internal-truss-tension-n 0.0)
        (xform-universal-spatial-awareness)
        (specialize-with-self-collision-manifold)
        (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 18.0))))


;; =============================================================================
;; B. COMPOSABLE TOPOLOGICAL & ENVIRONMENTAL SPECIALIZERS (PLUG-IN EXTENSIONS)
;; =============================================================================

;; 1. Phase-Topology & Symmetry Bifurcation Specializer
;; Smoothly shifts the 4-Torus phase offsets and duty cycle between:
;; - `:crawl-lateral-sequence` (Max stability, 3-point support triangle)
;; - `:diagonal-trot`          (Fast, zero-net-yaw diagonal synchronization)
;; - `:ipsilateral-pace`       (Lateral body-roll momentum conservation)
;; - `:synchronous-bound`      (Dynamic obstacle/ditch bounding)
(defmodifier specialize-quad-phase-topology
  [quad-state &key (gait-symmetry :diagonal-trot)]
  (let [[offsets duty bpm-scale]
        (case gait-symmetry
          (:crawl-lateral-sequence '([0.00 0.25 0.50 0.75] 0.78 0.85))
          (:diagonal-trot          '([0.00 0.50 0.50 0.00] 0.52 1.25))
          (:ipsilateral-pace       '([0.00 0.00 0.50 0.50] 0.54 1.15))
          (:synchronous-bound      '([0.00 0.50 0.05 0.55] 0.42 1.45)))]
    (-> quad-state
        (configure-4torus-symplectic-clock
          :master-bpm (:scale bpm-scale)
          :stance-duty-pwm duty
          :phase-offsets-lh-lf-rh-rf offsets))))

;; 2. Pronograde Contact-Interface Specializer (Bear-Walk vs. Hands-and-Knees vs. Forearm-Plank)
;; Adapts which anatomical patches touch the floor based on ceiling height & surface hardness:
;; - `:digitigrade-bear`  -> Metatarsal toes + flat palms (Fast, high hip clearance)
;; - `:hands-and-knees`   -> Tibial knee pads + flat palms (Low ceiling < 650mm, zero ankle strain)
;; - `:ulnar-tibial-low`  -> Forearm ulna blades + tibial knees (Crawl under 420mm table/obstacle)
(defmodifier specialize-pronograde-contact-interface
  [quad-state &key (contact-regime :hands-and-knees)]
  (case contact-regime
    (:digitigrade-bear
      (-> quad-state
          (equalize-heterogeneous-limb-kinematics
            :orientation-regime :pronograde-belly-down
            :hindlimb-contact-mode :equinus-digitigrade
            :forelimb-contact-mode :carpal-heel-palm
            :target-trunk-height-z-mm 520.0)))

    (:hands-and-knees
      (-> quad-state
          (equalize-heterogeneous-limb-kinematics
            :orientation-regime :pronograde-belly-down
            :hindlimb-contact-mode :tibial-knee-pad
            :forelimb-contact-mode :carpal-heel-palm
            :target-trunk-height-z-mm 360.0)
          ;; Release unloaded ankles/feet into passive trailing Null-Space behind knees:
          (specialize-posture-dangling-crural-hang :passive-ankle-droop-rad -0.38)))

    (:ulnar-tibial-low
      (-> quad-state
          (equalize-heterogeneous-limb-kinematics
            :orientation-regime :pronograde-belly-down
            :hindlimb-contact-mode :tibial-knee-pad
            :forelimb-contact-mode :ulnar-forearm-plank
            :target-trunk-height-z-mm 240.0)))))

;; 3. Supine Peristaltic Under-Bed / Low-Clearance Scoot (5-Point Bridge & Slide)
;; When roof height in `high-thoracic-slab` drops below 320 mm (e.g. scooting under a
;; bed frame or low pipe on the back), recruits `:ischial-bilateral` and `:dorsal-supine-continuum`
;; as a 5th peristaltic support phase so the robot never has to hold a high crab bridge!
(defmodifier specialize-supine-peristaltic-underbed-scoot
  [supine-state &key (max-ceiling-z-mm 275.0) (scoot-slide-mu 0.40)]
  (-> supine-state
      (reconstruct-modal-pose :mode-axial-compression 0.78) ; Compress bridge height to fit < 275mm
      (couple-fifth-limb-axial-peristalsis
        :axial-patch :ischial-bilateral
        :bridge-lift-clearance-mm 28.0
        :axial-duty-cycle 0.48)
      (drive-tangential-coulomb-slide
        :sliding-patches [:ischial-bilateral :dorsal-supine-continuum]
        :slide-velocity-xyz-mps [-0.22 0.0 0.0]
        :max-normal-fz-during-slide 0.45)))

;; 4. Steep-Grade Scrambling & Chimney Opposition (Stairs/Slopes > 35 deg on All Fours)
;; On steep stairs, rubble, or slopes where bipedal ZMP would tip backward, switches
;; to pronograde scrambling and injects positive internal virtual-truss tension
;; (`internal-truss-tension-n > 0`) to wedge hands and feet securely against risers/rocks.
(defmodifier specialize-quad-steep-grade-scramble
  [quad-state &key (slope-pitch-rad 0.65) (wedge-preload-n 32.0)]
  (-> quad-state
      (equalize-heterogeneous-limb-kinematics
        :orientation-regime :pronograde-belly-down
        :hindlimb-contact-mode :equinus-digitigrade
        :forelimb-contact-mode :carpal-heel-palm
        :target-trunk-pitch-rad slope-pitch-rad)
      (project-virtual-truss-internal-wrenches
        :hindlimb-load-share-bias 0.74
        :internal-truss-tension-n wedge-preload-n) ; Actively pinch stair treads/rocks between hands & feet!
      (arpeggiate-elevation-energy :step-dz-mm :from-sub-tarsal-sdf)))

;; 5. Bidirectional Biped <-> Pronograde / Supine Quadruped Morphing Funnel
;; Smoothly lowers from upright bipedal walking to either pronograde all-fours
;; (via deep squat + forward palmar touchdown) or supine crab (via blind posterior
;; sit + carpal back-brace), and reverses back to upright bipedal walking.
(defmodifier specialize-transition-biped-to-quadruped
  [state &key (target-orientation :pronograde-belly-down)]
  (case target-orientation
    (:pronograde-belly-down
      (-> state
          ;; 1. Deep squat via 1-Jet `:plantar-upright` axial mode until hands are 140mm from floor
          (reconstruct-modal-pose :mode-axial-compression 0.88 :mode-pitch-attitude 0.55)
          ;; 2. Sweep palms forward/downward in tactile admittance field until Fz_palmar latches
          (specialize-with-tactile-arm-sweep :max-arm-support-share 0.32)
          ;; 3. Morph wrench cone to 4-limb virtual truss and ignite T^4 clock
          (xform-gait-pronograde-quadruped)))

    (:supine-belly-up
      (-> state
          ;; 1. Settle backward via blind admittance + carpal behind-hip touchdown
          (xform-blind-posterior-settle)
          (specialize-with-carpal-seat-boost :max-arm-push-share 0.36)
          ;; 2. Unweight ischial patch by extending hips into inverted bridge and ignite T^4 crab clock
          (xform-gait-supine-inverted-crab)))))
