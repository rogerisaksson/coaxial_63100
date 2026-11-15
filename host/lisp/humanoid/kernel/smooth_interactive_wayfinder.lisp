;; =============================================================================
;; FILE: smooth_interactive_wayfinder.lisp
;; DOMAIN: Object-Agnostic SO(3) Orientation, SE(2) Clothoid Curvature Filter,
;;         Zero-Distance Affordance Docking Funnel & Anti-Trip Swing Foliation
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           agnostic_terrain_and_clutter_nav.lisp
;; The robot's own: somatic_interaction_core.lisp, lie_group_slam_core.lisp and the bridge
;; agnostic_terrain_and_clutter_nav.lisp (`:ortho-dsp.agnostic-nav`).
;; =============================================================================
(defpackage :ortho-dsp.smooth-interactive-nav
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.nav-core
        :ortho-dsp.nav-envs
        :ortho-dsp.agnostic-nav)
  (:export #:clothoid-and-docking-soa-state
           #:carve-anisotropic-affordance-docking-funnel
           #:filter-streamline-via-se2-clothoid
           #:allow-sub-overhang-interactive-footholds
           #:xform-smooth-interactive-stereo-navigator
           ;; Composable Specializers
           #:specialize-nav-zero-distance-furniture-dock
           #:specialize-nav-zero-distance-object-pickup
           #:specialize-nav-outdoor-smooth-trail-cruiser))
(in-package :ortho-dsp.smooth-interactive-nav)

;; -----------------------------------------------------------------------------
;; 1. CLOTHOID STREAMLINE & INTERACTION DOCKING SOA STATE (64B-ALIGNED)
;; Maintains the continuous C^2 path states [psi_heading, kappa_curvature] and
;; the active interaction target descriptor for zero-distance approach.
;; -----------------------------------------------------------------------------
(defstate clothoid-and-docking-soa-state
  (:memory-layout :align-bytes 64)
  (:slot-schema
    ;; SE(2) Clothoid Filter States (Guarantees C^2 smooth walking curves!):
    :smooth-heading-psi-rad    :float32      ; Integrated path tangent angle psi(t)
    :path-curvature-kappa-m    :float32      ; Instantaneous curvature kappa(t) = d(psi)/ds [1/m]
    :curvature-rate-dkappa     :float32      ; Clothoid sharpness d(kappa)/dt [1/(m*s)]
    :centripetal-speed-scale   :float32      ; Automatic speed reduction in tight curves [0.2 .. 1.0]
    ;; Active Interaction Target Descriptor (For Zero-Distance Approach):
    :target-cluster-id         :int32        ; Index in `agnostic-hazard-soa-registry` (-1 = none)
    :docking-face-normal-xy    :vec2-float32 ; Outward unit normal of the chosen interaction face
    :target-touch-standoff-m   :float32      ; 0.000 m = physical contact, -0.015 m = preload press!
    :docking-funnel-aperture   :float32))    ; Cosine power gamma controlling funnel width

;; -----------------------------------------------------------------------------
;; 2. ANISOTROPIC AFFORDANCE DOCKING FUNNEL (ZERO-DISTANCE TARGET EXCISION)
;; For the designated interaction cluster `target-cluster-id`:
;;   - Keeps the rear and side corners of the object 100% protected by the
;;     oriented Sherman-Morrison barrier so the robot smoothly curves AROUND
;;     the object if approaching from behind.
;;   - Excises the barrier margin down to `target-touch-standoff-m` (0.0 mm!)
;;     inside the frontal docking cone aligned with `docking-face-normal-xy`,
;;     guiding the robot smoothly into zero-clearance contact perpendicular
;;     to the object's oriented face!
;; The robot's own: its slot's margin from its reach across its way
;; (0.20-0.33 m in the bands it meets) to the robot's reach ahead (0.10-0.32 m) and the standoff;
;; the face its dock's (`machine.way.dock`, by what the thing affords), the cone's apex its
;; centre. -----------------------------------------------------------------------------
(defprimitive carve-anisotropic-affordance-docking-funnel
  [state &key (target-selection :nearest-goal-cluster) ; :nearest-goal-cluster | :recognized-semantic-class | :none
              (preferred-face :minor-axis-front)       ; :minor-axis-front (broad side) | :major-axis-end
              (terminal-standoff-mm 0.0)               ; 0.0 mm = exact surface touch!
              (funnel-sharpness-gamma 2.5)]
  (-> state
      (modulate-target-obstacle-permeability-cone
        :hazard-registry 'agnostic-hazard-soa-registry
        :docking-state   'clothoid-and-docking-soa-state
        :selector        target-selection
        :face-eigenvector preferred-face
        :touch-standoff-m (* 0.001 terminal-standoff-mm)
        :cone-exponent    funnel-sharpness-gamma
        ;; Inside the docking cone (sigma_afford -> 1.0):
        ;; 1. Collapse pelvic & thoracic radial repulsion margin from 150 mm -> `terminal-standoff-mm`.
        ;; 2. Align terminal approach streamline orthogonally into `-docking-face-normal-xy`.
        ;; 3. Open 1 kHz capsule ECBF permeability (`sigma_intent -> 1.0`) for tactile contact!
        :couple-capsule-affordance-permeability true)))

;; -----------------------------------------------------------------------------
;; 3. SE(2) CLOTHOID CURVATURE & CENTRIPETAL VELOCITY GOVERNOR (C^2 SMOOTH PATHS)
;; Replaces jerky C^0 velocity deflections with an Euler-spiral (clothoid)
;; differential filter on SE(2) combined with harmonic dipole circulation:
;;   d(psi)/dt = v * kappa,  |kappa| <= mu * g / v^2,  |d(kappa)/dt| <= sigma_max
;; The robot's own: |kappa| within its walk's arc (WALK_SPEED / YAW_RATE, 1.63 m) - it has one
;; walk's speed, so its arc bounds kappa, not its speed -; d(kappa)/dt its arc taken up
;; over a step, where a sole lands and turns it (`machine.way.errand`).
;; -----------------------------------------------------------------------------
(defprimitive filter-streamline-via-se2-clothoid
  [state &key (curvature-bandwidth-radps 6.5)
              (max-curvature-inv-m 0.61)       ; Min turning radius R_min ~ 0.54 m during normal walk! The robot's own 1.63 m, its arc
              (max-clothoid-rate-inv-ms 1.41)  ; Bounds lateral jerk d^3(y)/dt^3! Text 2.4; the robot's own 0.61 /m over a step's 0.43 s
              (harmonic-dipole-circulation 0.42)]
  (-> state
      (integrate-se2-clothoid-streamline
        :docking-state 'clothoid-and-docking-soa-state
        :omega-n curvature-bandwidth-radps
        :damping-zeta 0.92
        :kappa-max max-curvature-inv-m
        :dkappa-max max-clothoid-rate-inv-ms
        ;; Harmonic Dipole Circulation: adds a smooth tangential swirl around oriented
        ;; obstacles at long range (1.8 m) so curves initiate early and gracefully!
        ;; The robot's own none: its way is planned round its things, its aim a pursuit's ahead.
        :dipole-swirl-gain harmonic-dipole-circulation
        ;; Centripetal Friction-Cone Governor: automatically slows translational speed v(t)
        ;; in tight turns so lateral acceleration v^2 * |kappa| never challenges the ZMP!
        ;; The robot's own: one walk speed (asked 0.25-0.69 m/s it walks 0.81): kappa bounded
        ;; instead.
        :centripetal-velocity-scaling true
        :bank-torso-into-curve-gain 0.14)))    ; Subtle human-like inward torso lean in curves!

;; -----------------------------------------------------------------------------
;; 4. SUB-OVERHANG INTERACTIVE FOOTHOLD ALLOWANCE (STEP UNDER TABLES / BEDS)
;; When walking up to zero distance at a table, desk, bed, or chair whose frame
;; is recessed at floor level (`sub-tarsal-slab` smaller than `pelvic-crural-slab`),
;; allows the feet `p_td` to step UNDER the overhang so the torso/pelvis can
;; reach true 0.0 mm contact with the upper edge without leaning off-balance!
;; The robot's own by its bands: a thing meets its reach in the bands its box spans
;; (`footprint.met`), a surface told apart from the legs under it (`machine.eyes.voxels`).
;; -----------------------------------------------------------------------------
(defprimitive allow-sub-overhang-interactive-footholds
  [state &key (shin-to-bevel-min-clearance-mm 14.0)]
  (-> state
      (decouple-subtarsal-from-pelvic-footprint-on-target
        :hazard-registry 'agnostic-hazard-soa-registry
        :docking-state   'clothoid-and-docking-soa-state
        :min-crural-capsule-margin-m (* 0.001 shin-to-bevel-min-clearance-mm)
        :align-terminal-feet-parallel-to-dock-face true)))

;; -----------------------------------------------------------------------------
;; 5. MASTER SMOOTH & INTERACTIVE STEREO NAVIGATOR TRANSFORMER
;; -----------------------------------------------------------------------------
(defmodifier xform-smooth-interactive-stereo-navigator
  [state &key (goal-xyz [4.0 0.0 0.0])
              (interact-with-goal-object true)
              (terminal-standoff-mm 0.0)]
  (-> state
      (foliate-local-curved-ground-manifold :grid-extent-m 7.0) ; 6 m! The robot's own 7 m, its map's side
      (extract-agnostic-frustum-so3-clusters
        :registry 'agnostic-hazard-soa-registry
        :min-positive-trip-height-mm 75.0     ; 24 mm! The robot's own 75 mm, its ankle: its map's floor 80 mm
        :min-negative-ditch-depth-mm -45.0)   ; The robot's own: its map holds nothing under its floor
      (carve-anisotropic-affordance-docking-funnel
        :target-selection (if interact-with-goal-object :nearest-goal-cluster :none)
        :preferred-face :minor-axis-front
        :terminal-standoff-mm terminal-standoff-mm)
      (solve-permeability-sherman-morrison-streamline
        :goal-xyz goal-xyz
        :pelvic-clearance-margin-mm 155.0     ; The robot's own: its reach across its way in each thing's bands
        :shoulder-aperture-slotting true)
      (filter-streamline-via-se2-clothoid
        :curvature-bandwidth-radps 6.5
        :max-curvature-inv-m 0.61             ; 1.85! The robot's own 0.61, its arc
        :harmonic-dipole-circulation 0.42)
      (allow-sub-overhang-interactive-footholds :shin-to-bevel-min-clearance-mm 14.0)
      (guard-3d-swing-and-prealign-ankle-so3
        :max-step-over-height-mm 210.0
        :swing-clearance-margin-mm 52.0
        :prealign-ankle-to-terrain-normal true)
      (xform-universal-spatial-awareness)
      (specialize-with-self-collision-manifold)
      (specialize-with-nullspace-limb-retraction :retraction-influence-radius-mm 260.0)))


;; =============================================================================
;; COMPOSABLE INTERACTION & TERRAIN SPECIALIZERS (PLUG-IN EXTENSIONS)
;; =============================================================================

;; 1. Zero-Distance Furniture Docking Specializer (Chair, High Stool, Bed Long-Side)
;; Guides the robot along a C^2 clothoid curve around clutter, approaches the
;; furniture's front/long face orthogonally, steps feet right up to the base,
;; and brings the calves/thighs to -5 mm (light 9 N tactile preload!) for sitting/lying.
;; The robot's own: its stage where its turn onto the face clears from any arrival
;; (`machine.way.dock`), its heels then the stunt's standoff off the edge (`maneuver.standoff`).
(defmodifier specialize-nav-zero-distance-furniture-dock
  [nav-state &key (target-class :chair-or-bed) (tactile-preload-standoff-mm -5.0)]
  (-> nav-state
      (carve-anisotropic-affordance-docking-funnel
        :target-selection :nearest-goal-cluster
        :preferred-face :minor-axis-front
        :terminal-standoff-mm tactile-preload-standoff-mm
        :funnel-sharpness-gamma 3.0)
      (gate-affordance-contact-permeability
        :allow-contact-zones [:buttocks-ischial :thigh-femoral :shank-crural :palm-carpal]
        :target-object-id :active-interaction-quadric)))

;; 2. Zero-Distance Table / Counter / Crate Manipulation Specializer
;; Walks up to 0 mm chest/hand distance from a workbench, table, or box on the floor,
;; stepping feet underneath any bottom recess while keeping hands ready for bimanual grasp.
(defmodifier specialize-nav-zero-distance-object-pickup
  [nav-state &key (standoff-mm 0.0)]
  (-> nav-state
      (carve-anisotropic-affordance-docking-funnel
        :target-selection :nearest-goal-cluster
        :preferred-face :minor-axis-front
        :terminal-standoff-mm standoff-mm)
      (allow-sub-overhang-interactive-footholds :shin-to-bevel-min-clearance-mm 12.0)
      (gate-affordance-contact-permeability
        :allow-contact-zones [:palm-carpal :forearm-ulnar :anterior-thorax]
        :target-object-id :active-interaction-quadric)))

;; 3. Outdoor Smooth Trail & Boulder Cruiser Specializer
;; Widens the harmonic dipole swirl (0.58) so the robot initiates long, sweeping
;; S-curves well ahead of boulders and trees while stepping over roots and ditches.
(defmodifier specialize-nav-outdoor-smooth-trail-cruiser [nav-state]
  (-> nav-state
      (filter-streamline-via-se2-clothoid
        :curvature-bandwidth-radps 5.2
        :max-curvature-inv-m 1.01               ; 1.35! The robot's own 1.01, its arc
        :harmonic-dipole-circulation 0.58)
      (specialize-nav-forest-roots-and-boulders :max-root-rock-height-mm 245.0)
      (specialize-nav-negative-ditches-and-curbs :ditch-depth-trigger-mm -40.0)))
