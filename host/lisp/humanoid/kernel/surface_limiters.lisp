;; -----------------------------------------------------------------------------
;; MODULE 1.7: surface_limiters
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.surfaces
  (:use :ortho-dsp.core)
  (:export #:limiter-high-mu-planar
           #:limiter-low-mu-shear))
(in-package :ortho-dsp.surfaces)

(deflimiter limiter-high-mu-planar [gait]
  (-> gait (enforce-wrench-cone :mu-static 0.85 :mu-kinetic 0.70 :shear-slew-limit-nps 2500.0)))

(deflimiter limiter-low-mu-shear [gait]
  (let [splay-active (> (get-param gait :metatarsal-splay-gain) 1.0)
        mu-eff       (if splay-active 0.14 0.08)
        foot-w       (get-param gait :foot-collision-w-mm)]
    (-> gait
        (enforce-wrench-cone :mu-static mu-eff :mu-kinetic (* 0.75 mu-eff) :shear-slew-limit-nps 120.0)
        (modulate-node :tarsal-sagittal :stride-length-mm (:scale 0.58)
                       :strike-angle-rad (:set 0.0) :toe-off-shear (:scale 0.20))
        (modulate-node :tarsal-coronal :track-width-mm (:clamp-min (+ foot-w 18.0))
                       :swing-circumduction-mm (:set 0.0))
        (modulate-node :iliac-coronal :lfo-amp-rad (:clamp-max 0.055) :ankle-impedance-dither true)
        (modulate-node :tibiofemoral :flexion-cushion-rad (:add 0.18))
        (modulate-node :glenohumeral :gain-db (:clamp-min -1.5) :abduction-rad (:add 0.22)))))
