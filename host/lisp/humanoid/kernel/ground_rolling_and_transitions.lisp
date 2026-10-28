;; =============================================================================
;; FILE: ground_rolling_and_transitions.lisp
;; DOMAIN: Composable Floor Descent, Ground Transfers, Rolling & Standing Egress
;; REQUIRES: floor_groundwork_core.lisp, ortho_dsp_cognitive_autonomy.lisp
;; =============================================================================
(defpackage :ortho-dsp.ground-xforms
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
        :ortho-dsp.quad-core
        :ortho-dsp.quad-xforms
        :ortho-dsp.ground-core)
  (:export ;; A. Upright -> Knees -> All-Fours -> Buttocks -> Supine/Prone
           #:xform-upright-to-kneeling
           #:xform-kneeling-to-hands-and-knees
           #:xform-to-floor-buttocks-seated
           #:xform-floor-to-supine-flat
           #:xform-floor-to-prone-flat
           ;; B. Rolling & Floor Egress back to Upright
           #:xform-continuous-ground-roll
           #:xform-floor-to-upright-egress
           ;; C. Composable Specializers
           #:specialize-descent-staggered-lunge
           #:specialize-buttocks-with-carpal-tripod
           #:specialize-prone-cobra-press-up
           #:specialize-egress-turkish-getup-spiral))
(in-package :ortho-dsp.ground-xforms)

;; =============================================================================
;; A. SEQUENTIAL & DIRECT FLOOR DESCENT TRANSFORMERS
;; =============================================================================

;; 1. Upright Standing -> Tall Kneeling on Both Knees (`:anterior-tibial-bilateral`)
;; Uses blind admittance + DCM-ZCBF so knees touch down with zero impact spike
;; regardless of floor height or mat thickness.
(defmodifier xform-upright-to-kneeling [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 40.0)
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      ;; Morph plantar support to forefoot equinus while lowering patellas along admittance field
      (bind-wrench-driven-homotopy
        :active-manifolds [:plantar-bilateral
                           :metatarsal-equinus-bilateral
                           :anterior-tibial-bilateral]
        :slew-rate-up-per-s 5.5
        :slew-rate-down-per-s 45.0
        :couple-impedance-melt false)
      (project-blind-admittance-field
        :seek-velocity-xyz-mps [0.06 0.0 -0.28]  ; Slight forward tibia tilt to land on patellar pad
        :virtual-damping-nsm 320.0)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (reconstruct-modal-pose :mode-axial-compression 0.0 :mode-pitch-attitude 0.0)
      (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 16.0)))

;; 2. Tall Kneeling -> Knees with Arms and Hands (`:anterior-tibial` + `:ulnar-carpal`)
(defmodifier xform-kneeling-to-hands-and-knees [state]
  (-> state
      ;; Hinge hips and sweep flat palms/carpal heels forward to the floor via admittance
      (reconstruct-modal-pose :mode-axial-compression 0.35 :mode-pitch-attitude 0.65)
      (specialize-with-tactile-arm-sweep :max-arm-support-share 0.34)
      (bind-wrench-driven-homotopy
        :active-manifolds [:anterior-tibial-bilateral :ulnar-carpal-shove-bilateral]
        :slew-rate-up-per-s 6.0
        :slew-rate-down-per-s 40.0)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (project-virtual-truss-internal-wrenches :hindlimb-load-share-bias 0.66 :internal-truss-tension-n 0.0)))

;; 3. Any State (Standing, Kneeling, or All-Fours) -> Seated on Floor Buttocks (`:ischial`)
;; - If entering from Kneeling/All-Fours: automatically executes the 90/90 hip-inversion
;;   sweep so shins clear the floor into front long-sit/cross-sit!
;; - If entering from Upright Standing: performs a deep squat + posterior carpal-assisted
;;   settle directly onto `:ischial-bilateral` at Z = 0.
(defmodifier xform-to-floor-buttocks-seated
  [state &key (with-hands-and-arms-support false)]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 40.0)
      (route-90-90-hip-inversion-bridge :direction :kneel-to-buttocks :pivot-side :left)
      (bind-wrench-driven-homotopy
        :active-manifolds (if with-hands-and-arms-support
                              [:ischial-bilateral :posterior-femoral-bilateral
                               :ulnar-carpal-shove-bilateral :plantar-bilateral]
                              [:ischial-bilateral :posterior-femoral-bilateral :plantar-bilateral])
        :slew-rate-up-per-s 5.0
        :slew-rate-down-per-s 45.0)
      (project-blind-admittance-field
        :seek-velocity-xyz-mps [-0.14 0.0 -0.26]
        :virtual-damping-nsm 290.0)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (reconstruct-modal-pose
        :mode-axial-compression 0.0
        :mode-sagittal-shift (if with-hands-and-arms-support -0.25 0.0))
      (enforce-dcm-barrier-zcbf :barrier-gamma 14.0 :min-capture-margin-mm 15.0)))

;; 4. Floor Seated / Kneeling -> Flat on Back (Supine, `:dorsal-supine-continuum`)
;; Unrolls the lumbar and thoracic spine sequentially onto the floor while extending legs.
(defmodifier xform-floor-to-supine-flat [state &key (terminal-high-z false)]
  (-> state
      (xform-blind-recumbent-flow)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (reconstruct-modal-pose :mode-axial-compression 0.0 :mode-pitch-attitude 0.0)
      (specialize-with-viscoelastic-sink-compensation)
      (when-modifier terminal-high-z
        (release-impedance-wavefront :terminal-mode :supine-high-z))))

;; 5. Knees+Hands OR Supine -> Flat on Stomach (Prone, `:ventral-prone-continuum`)
;; Lowers eccentrically through triceps/pectoralis from `:kneeling-quadruped-braced`
;; (or rolls 180 deg from supine) into full ventral alignment with neck yaw protection.
(defmodifier xform-floor-to-prone-flat [state]
  (-> state
      (configure-momentum-observer :observer-bandwidth-hz 40.0)
      (bind-wrench-driven-homotopy
        :active-manifolds [:anterior-tibial-bilateral
                           :ulnar-carpal-shove-bilateral
                           :olecranon-ulnar-sphinx
                           :ventral-prone-continuum]
        :slew-rate-up-per-s 4.5
        :slew-rate-down-per-s 40.0
        :couple-impedance-melt true)
      ;; Eccentric push-up lowering field until sternum & anterior pelvis rest on floor:
      (project-blind-admittance-field
        :seek-velocity-xyz-mps [0.08 0.0 -0.22]
        :virtual-damping-nsm 310.0)
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (reconstruct-modal-pose :mode-axial-compression 0.0)))


;; =============================================================================
;; B. CONTINUOUS GROUND ROLLING & FULL FLOOR-TO-STANDING EGRESS
;; =============================================================================

;; 1. Universal Ground Roll (Supine <-> Lateral <-> Prone <-> Supine on Hard or Soft Floor)
(defmodifier xform-continuous-ground-roll
  [state &key (direction :clockwise-right)
              (angle-rad 3.14159)                ; 3.14159 = 180 deg (Back<->Stomach), 6.28318 = 360 deg
              (leader :pelvic-led)]
  (-> state
      (wake-inverter-closed-loop)
      (configure-momentum-observer :observer-bandwidth-hz 45.0)
      ;; Automatically detects Hard Floor vs. Soft Mattress via d(Fz)/d(z) and adapts
      ;; spine rounding, bony landmark shielding, and mattress-trough push-off impulse!
      (adapt-rolling-substrate-mechanics
        :hard-floor-threshold-npmm 380.0
        :soft-mattress-threshold-npmm 65.0)
      (drive-helical-axial-roll-wave
        :roll-direction direction
        :target-roll-angle-rad angle-rad
        :propagation-leader leader
        :roll-angular-velocity-radps 1.45)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)))

;; 2. Universal Floor-to-Upright Standing Egress (From Supine, Prone, Buttocks, or Knees)
;; Uses the Funnel Graph to chain the minimum-work biomechanical ascent:
;;   - If Prone: Cobra Press-Up -> Hands-and-Knees -> Tall Kneel / Half-Kneel Lunge -> Stand
;;   - If Supine: Axial Roll-Up or Side-Roll -> Tripod Buttocks -> 90/90 Kneel -> Stand
(defmodifier xform-floor-to-upright-egress [floor-state]
  (-> floor-state
      (wake-inverter-closed-loop)
      (configure-momentum-observer :observer-bandwidth-hz 40.0)
      (sequence-wrench-homotopy
        :total-duration-ms 2100.0
        :keyframes
          '((0.00 ((:dorsal-supine-continuum 0.50) (:ventral-prone-continuum 0.50)))
            (0.28 ((:ischial-bilateral 0.60) (:ulnar-carpal-shove-bilateral 0.40)))
            (0.55 ((:anterior-tibial-bilateral 0.65) (:ulnar-carpal-shove-bilateral 0.35)))
            (0.80 ((:plantar-bilateral 0.70) (:metatarsal-equinus-bilateral 0.30)))
            (1.00 ((:plantar-bilateral 1.00)))))
      (blend-manifold-posture-jet :atlas 'groundwork-1jet-posture-atlas)
      (xform-balance-biped-zmp)
      (ignite-cyclic-clock nominal-biped-gait :bars 1)))


;; =============================================================================
;; C. COMPOSABLE SPECIALIZATION EXTENSIONS (PLUG-IN MODIFIERS)
;; =============================================================================

;; 1. Staggered Split-Lunge Kneel Specializer (Asymmetric Knight's Kneel Descent/Ascent)
;; Lowers one knee to the ground first while the leading foot remains flat in
;; `:plantar-bilateral`, halving peak patellar impact and doubling sagittal CWC length!
(defmodifier specialize-descent-staggered-lunge
  [state &key (lead-stance-foot :left) (stagger-step-x-mm 340.0)]
  (let [trail-knee (if (eq lead-stance-foot :left) :tibial-tuberosity-r :tibial-tuberosity-l)]
    (-> state
        (modulate-node :tarsal-sagittal :stride-length-mm (:set stagger-step-x-mm))
        (splice-auxiliary-manifold
          :patch-id :unilateral-knight-keel
          :anatomical-frames [trail-knee]
          :surface-normal [0.0 0.0 1.0]
          :cop-box-mm '(:x [-160.0 45.0] :y [-40.0 40.0])
          :mu 0.75
          :weight-law (:wrench-proportional :max-share 0.60)))))

;; 2. Seated Buttocks + Bilateral/Unilateral Carpal Tripod Specializer
;; Configures how the arms/hands support the torso when sitting on the floor on the buttocks:
;; - `:posterior-sunbather-tripod` (Both hands planted behind hips, chest reclined)
;; - `:unilateral-side-lean`       (One hand/forearm planted beside hip)
;; - `:anterior-combat-guard`      (Hands planted in front of shins for rapid forward crawl)
(defmodifier specialize-buttocks-with-carpal-tripod
  [seated-floor-state &key (tripod-mode :posterior-sunbather-tripod) (arm-load-share 0.28)]
  (-> seated-floor-state
      (specialize-posture-appendage-brace :brace-style :carpal-seat-push :triceps-extension-gain 0.90)
      (splice-auxiliary-manifold
        :patch-id :floor-tripod-hand-anchor
        :anatomical-frames (case tripod-mode
                             (:unilateral-side-lean [:carpal-heel-l :ulna-distal-l])
                             (otherwise             [:carpal-heel-l :carpal-heel-r]))
        :surface-normal [0.0 0.0 1.0]
        :mu 0.70
        :weight-law (:wrench-proportional :max-share arm-load-share))))

;; 3. Prone Sphinx / Cobra Press-Up Specializer (From Flat Stomach to Hands-and-Knees)
;; Presses flat palms or forearms into the floor from `:ventral-prone-continuum`
;; to peel the ribcage and pelvis off the ground into `:kneeling-quadruped-braced`.
(defmodifier specialize-prone-cobra-press-up
  [prone-state &key (intermediate-sphinx-dwell-ms 350.0)]
  (-> prone-state
      (reconstruct-modal-pose
        :mode-axial-compression 0.92             ; Activates triceps press + hip/knee flexion synergy
        :mode-pitch-attitude -0.35)
      (pan-wrench-homotopy
        :from-manifolds '((:ventral-prone-continuum 1.00))
        :to-manifolds   '((:anterior-tibial-bilateral 0.62) (:ulnar-carpal-shove-bilateral 0.38))
        :duration-ms (+ 650.0 intermediate-sphinx-dwell-ms))))

;; 4. Turkish Get-Up Spiral Egress Specializer (Heavy / Low-Torque Floor Ascent)
;; When battery bus voltage is low or knee motors are thermally derated, rises from
;; flat supine via a continuous unilateral diagonal spiral:
;; Supine -> Ipsilateral Elbow Prop -> Carpal Tripod -> High Hip Bridge -> Sweep Knee Under -> Lunge Stand!
(defmodifier specialize-egress-turkish-getup-spiral
  [floor-state &key (support-side :right)]
  (let [prop-frames (if (eq support-side :right)
                        [:olecranon-r :carpal-heel-r]
                        [:olecranon-l :carpal-heel-l])]
    (-> floor-state
        (splice-auxiliary-manifold
          :patch-id :tgu-unilateral-column
          :anatomical-frames prop-frames
          :surface-normal [0.0 0.0 1.0]
          :mu 0.75
          :weight-law (:wrench-proportional :max-share 0.42))
        (route-90-90-hip-inversion-bridge :direction :buttocks-to-kneel :pivot-side support-side)
        (specialize-descent-staggered-lunge :lead-stance-foot (if (eq support-side :right) :left :right)))))
