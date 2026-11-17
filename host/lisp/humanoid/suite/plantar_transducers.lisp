;; =============================================================================
;; FILE: plantar_transducers.lisp
;; DOMAIN: Footwear Mechanical Impedance & Contact Geometry
;; =============================================================================
(defpackage :ortho-dsp.transducers
  (:use :ortho-dsp.core)
  (:export #:transducer-barefoot
           #:transducer-elastomer-flat
           #:transducer-heavy-lugged-boot))
(in-package :ortho-dsp.transducers)

;; 1. Unshod / Barefoot: Midfoot protection, active plantar aponeurosis windlass spring
(deftransducer transducer-barefoot [gait]
  (-> gait
      (set-plantar-boundary
        :sole-thickness-z-mm 0.0
        :foot-collision-w-mm 84.0
        :talocrural-pitch-bias-rad 0.0
        :support-polygon-x-mm 195.0
        :windlass-arch-q 1.40
        :metatarsal-splay-gain 1.35)
      (modulate-node :calcaneus
        :strike-topology (:set :midfoot-biased)
        :saturation      (:set :biological-pad-clip)
        :attack-ms       (:scale 0.45))))

;; 2. Viscoelastic Elastomer Sole (Sneaker): Second-order low-pass impact attenuation
(deftransducer transducer-elastomer-flat [gait]
  (-> gait
      (set-plantar-boundary
        :sole-thickness-z-mm 32.0
        :foot-collision-w-mm 96.5
        :talocrural-pitch-bias-rad 0.035
        :support-polygon-x-mm 265.0
        :windlass-arch-q 0.60
        :metatarsal-splay-gain 0.0)
      (modulate-node :calcaneus
        :strike-topology (:set :heel-to-toe)
        :saturation      (:set :tanh-viscoelastic)
        :attack-ms       (:add 12.0)
        :decay-ms        (:scale 0.65)
        :sustain         (:set 0.25))))

;; 3. Heavy Tactical / Alpine Boot: High distal mass, immobilized talocrural joint
(deftransducer transducer-heavy-lugged-boot [gait]
  (-> gait
      (set-plantar-boundary
        :sole-thickness-z-mm 45.0
        :foot-collision-w-mm 112.0         ; Wide lugged outsole requires wider track
        :talocrural-pitch-bias-rad 0.05
        :support-polygon-x-mm 295.0
        :windlass-arch-q 0.10              ; Rigid steel/carbon shank
        :metatarsal-splay-gain 0.0)
      (modulate-node :tarsal-coronal
        :track-width-mm (:clamp-min 118.5))
      (modulate-node :tarsal-sagittal
        :swing-clearance-z-mm (:add 18.0)) ; Compensate for heavy distal pendulum
      (modulate-node :calcaneus
        :strike-topology (:set :rocker-sole-roll)
        :attack-ms       (:add 18.0))))
