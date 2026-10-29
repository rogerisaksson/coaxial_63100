;; -----------------------------------------------------------------------------
;; MODULE 2.5: agnostic_terrain_and_clutter_nav (OBJECT-AGNOSTIC CLUTTER ROUTE)
;; The package smooth_interactive_wayfinder.lisp uses and no text sent defines:
;; its primitives laid on her map, her costed cells and her planner apart.
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.agnostic-nav
  (:use :ortho-dsp.core
        :ortho-dsp.body-schema
        :ortho-dsp.nav-core
        :ortho-dsp.perception-core)
  (:export #:foliate-local-curved-ground-manifold
           #:extract-agnostic-frustum-so3-clusters
           #:lay-banded-traversability-costmap
           #:solve-permeability-sherman-morrison-streamline
           #:run-wayfinder-apart
           #:guard-3d-swing-and-prealign-ankle-so3
           #:specialize-nav-forest-roots-and-boulders
           #:specialize-nav-negative-ditches-and-curbs
           #:specialize-terrain-outdoor
           #:specialize-with-recognized-classes))
(in-package :ortho-dsp.agnostic-nav)

;; Her ground: her soles' floor, flat under her map; the bands over it its foliation.
(defprimitive foliate-local-curved-ground-manifold
  [state &key (grid-extent-m 7.0)]
  (-> state
      (fit-local-ground-manifold
        :anchor :stance-soles
        :band-source :foliated-sdf-slabs
        :grid-extent-m grid-extent-m)))

;; A thing is whatever her stereo's voxels lump together: a box on the floor, its turn its
;; footprint's least-area rectangle; recognition names it, the route needs no name. Under her
;; trip height it is floor she steps over.
(defprimitive extract-agnostic-frustum-so3-clusters
  [state &key (registry 'spatial-object-soa-registry)
              (min-positive-trip-height-mm 75.0)
              (min-negative-ditch-depth-mm -45.0)
              (footprint-fit :min-area-rectangle)]
  (-> state
      (fit-oriented-obstacle-boxes
        :registry registry
        :point-source :stereo-voxel-footprint
        :footprint-fit footprint-fit
        :trip-height-m (* 0.001 min-positive-trip-height-mm)
        :ditch-depth-m (* 0.001 min-negative-ditch-depth-mm))))

;; Each floor cell's cost a metre walked: blocked where a thing lies within her reach across her
;; way in the bands it meets (`configure-foliated-sdf-bands`), else 1 plus each layer's in order;
;; the thing she goes to opened before its face (`carve-anisotropic-affordance-docking-funnel`).
;; A layer is a name and its keys; an extension appends its name, `:layers (:add [:its-name])`.
(defprimitive lay-banded-traversability-costmap
  [state &key (clearance-weight 2.0)
              (clearance-decay-mm 150.0)
              (unknown-weight 0.5)
              (class-margins-mm ())]
  (-> state
      (weigh-traversability-layers
        :band-source :foliated-sdf-slabs
        :layers [:class-margin :affordance-aperture :clearance :unknown]
        :class-margins-mm class-margins-mm
        :clearance-weight clearance-weight
        :clearance-decay-mm clearance-decay-mm
        :unknown-weight unknown-weight)))

;; Her planner in a process of its own, as her camera's map: a dock or a way she stands for told
;; at once, her way watched against her things each period while she walks it, told a latency on.
(defprimitive run-wayfinder-apart
  [state &key (watch-latency-s 0.25)
              (watch-period-s 0.5)]
  (-> state
      (spawn-wayfinder-process
        :transport :pickled-stdio
        :watch-latency-s watch-latency-s
        :watch-period-s watch-period-s)))

;; Her way: A* over the costed cells, a corner cut only where the straight line costs no more
;; than the cells it replaces, the curve along the corners her walk's (`machine.route`); her
;; trunk twisted through a gap under her shoulders (`slice-narrow-aperture-axial`); apart.
(defprimitive solve-permeability-sherman-morrison-streamline
  [state &key (goal-xyz [4.0 0.0 0.0])
              (pelvic-clearance-margin-mm 155.0)
              (shoulder-aperture-slotting true)]
  (-> state
      (synthesize-riemannian-metric :goal-attractor-xyz goal-xyz)
      (lay-banded-traversability-costmap)
      (search-costed-cell-geodesic
        :connectivity 8
        :heuristic :euclidean-least-cost
        :shortcut-law :no-dearer-than-replaced
        :pelvic-clearance-margin-m (* 0.001 pelvic-clearance-margin-mm)
        :shoulder-aperture-slotting shoulder-aperture-slotting)
      (run-wayfinder-apart)))

;; Her swing over what she steps over, her landing sole tilted to the ground's normal: her
;; foothold's text (`snap-foothold-to-plantar-gradient`), its lift this one's clearance.
(defprimitive guard-3d-swing-and-prealign-ankle-so3
  [state &key (max-step-over-height-mm 210.0)
              (swing-clearance-margin-mm 52.0)
              (prealign-ankle-to-terrain-normal true)]
  (-> state
      (snap-foothold-to-plantar-gradient :step-over-margin-z-mm swing-clearance-margin-mm)
      (adapt-to-surface-normal
        :max-step-over-height-m (* 0.001 max-step-over-height-mm)
        :prealign-landing-sole prealign-ankle-to-terrain-normal)))

;; Open ground: roots and rocks stepped over up to their height.
(defmodifier specialize-nav-forest-roots-and-boulders
  [nav-state &key (max-root-rock-height-mm 245.0)]
  (-> nav-state
      (adapt-to-surface-normal :max-step-over-height-m (* 0.001 max-root-rock-height-mm))))

;; A ditch or a curb's drop: a hole in her floor from its depth on.
(defmodifier specialize-nav-negative-ditches-and-curbs
  [nav-state &key (ditch-depth-trigger-mm -40.0)]
  (-> nav-state
      (fit-oriented-obstacle-boxes :ditch-depth-m (* 0.001 ditch-depth-trigger-mm))))

;; Open ground: room to keep off things, what lies unseen mostly ground.
(defmodifier specialize-terrain-outdoor [nav-state]
  (-> nav-state
      (weigh-traversability-layers
        :clearance-weight 3.0
        :clearance-decay-mm 400.0
        :unknown-weight 0.2)))

;; Recognition's names widen a thing's reach where its kind needs it: glass her stereo sees by
;; its frame and its specks, not its pane.
(defmodifier specialize-with-recognized-classes [nav-state]
  (-> nav-state
      (weigh-traversability-layers
        :class-margins-mm (:glass 100.0))))
