;; =============================================================================
;; FILE: 04_surface_wrench_limiters.lisp
;; DOMAIN: Master-Bus Contact Wrench-Cone & Medium Impedance Limiters
;; =============================================================================
(defpackage :ortho-dsp.surfaces
  (:use :ortho-dsp.core)
  (:export #:surface-high-mu-planar
           #:surface-low-mu-ice
           #:surface-granular-dissipative
           #:surface-stochastic-uneven))
(in-package :ortho-dsp.surfaces)

;; 1. High-Friction Rigid Planar (Dry Concrete / Hardwood Runway)
(deflimiter surface-high-mu-planar [gait]
  (-> gait
      (enforce-wrench-cone :mu-static 0.85 :mu-kinetic 0.70 :max-shear-slew-nps 2500.0)))

;; 2. Low-Friction Shear Boundary (Black Ice: mu = 0.08)
(deflimiter surface-low-mu-ice [gait]
  (let [splay-active (> (get-param gait :metatarsal-splay-gain) 1.0)
        mu-eff       (if splay-active 0.14 0.08)
        foot-w       (get-param gait :foot-collision-w-mm)]
    (-> gait
        (enforce-wrench-cone :mu-static mu-eff :mu-kinetic (* 0.75 mu-eff) :max-shear-slew-nps 120.0)
        (modulate-node :tarsal-sagittal :stride-length-mm (:scale 0.58) :strike-angle-rad (:set 0.0))
        (modulate-node :tarsal-coronal  :track-width-mm (:clamp-min (+ foot-w 18.0)) :swing-circumduction-mm (:set 0.0))
        (modulate-node :iliac-coronal   :lfo-amplitude-rad (:clamp-max 0.055))
        (modulate-node :com-dynamics    :base-height-z-mm (:add -45.0))
        (modulate-node :glenohumeral    :gain-db (:clamp-min -1.5) :abduction-rad (:add 0.22)))))

;; 3. Granular Dissipative Medium (Dry Sand / Deep Snow: Plastic Sinkage & Zero Elastic Return)
(deflimiter surface-granular-dissipative [gait &key (sink-depth-mm 65.0)]
  (-> gait
      (enforce-wrench-cone :mu-static 0.45 :mu-kinetic 0.35 :max-shear-slew-nps 450.0)
      (modulate-plantar-boundary :windlass-arch-q (:scale 0.15)) ; Medium swallows elastic recoil
      (modulate-node :tibiofemoral
        :rebound-q (:clamp-max 0.35)                             ; Kill SLIP elastic bounce
        :energy-injection-j (:add (* 9.80665 (/ sink-depth-mm 1000.0))))
      (modulate-node :tarsal-sagittal
        :swing-clearance-z-mm (:add (* 1.35 sink-depth-mm))      ; High-step over snow/sand surface
        :stride-length-mm (:scale 0.82))
      (modulate-node :calcaneus
        :attack-ms (:add 40.0)                                   ; Plastic medium crushes slowly
        :decay-ms (:add 85.0))))

;; 4. Stochastic Uneven Terrain (Rocky Trail / Rubble: High Clearance & Ankle Compliance)
(deflimiter surface-stochastic-uneven [gait &key (rugosity-mm 45.0)]
  (-> gait
      (modulate-node :tarsal-sagittal :swing-clearance-z-mm (:add (* 1.5 rugosity-mm)))
      (modulate-node :tarsal-coronal  :track-width-mm (:add 28.0))
      (modulate-node :tibiofemoral    :compliance (:clamp-min 0.85) :stance-yield-rad (:add 0.12))
      (modulate-node :c1-c2-atlas     :dc-offset-rad (:add 0.22) :vor-lock (:set true)))) ; Gaze downward
