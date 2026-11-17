;; -----------------------------------------------------------------------------
;; MODULE 1.9: balance_xforms
;; SYNOPSIS: Balance: biped ZMP, biped to seated and supine to biped; armrests and a wall brace.
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.balance-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core)
  (:export #:xform-balance-biped-zmp
           #:xform-balance-biped-to-seated
           #:xform-balance-supine-to-biped
           #:specialize-with-armrest-rails
           #:specialize-with-unilateral-wall-brace))
(in-package :ortho-dsp.balance-xforms)

(defmodifier xform-balance-biped-zmp [state]
  (-> state
      (pan-wrench-homotopy :from-manifolds :current :to-manifolds '((:plantar-bilateral 1.00))
                           :pan-law :linear-convex :duration-ms 180.0)
      (configure-balance-feedback
        :primary-actuators [:talocrural :coxofemoral :iliac-coronal :l4-l5-lumbar]
        :dcm-proportional-gain 4.2 :angular-momentum-damping 0.85)))

(defmodifier xform-balance-biped-to-seated
  [state &key (seat-pitch-rad -0.06) (foot-residual-weight 0.15) (transfer-ms 420.0)]
  (-> state
      (orient-manifold-normal :patch :ischial-bilateral :pitch-rad seat-pitch-rad)
      (pan-wrench-homotopy
        :from-manifolds '((:plantar-bilateral 1.00))
        :to-manifolds   `((:ischial-bilateral ,(- 1.0 foot-residual-weight))
                          (:plantar-bilateral ,foot-residual-weight))
        :pan-law :equal-wrench-power :poinsot-ray-slew :parabolic-capture
        :sync-trigger (:contact-fz :ischial-bilateral 0.12) :duration-ms transfer-ms)
      (configure-balance-feedback
        :primary-actuators [:l4-l5-lumbar :c7-t12-thoracic :coxofemoral]
        :nullspace-released-nodes [:calcaneus :talocrural :tibiofemoral]
        :dcm-proportional-gain 1.6)))

(defmodifier xform-balance-supine-to-biped [state &key (ascent-ms 1600.0)]
  (-> state
      (sequence-wrench-homotopy
        :total-duration-ms ascent-ms
        :keyframes
          '((0.00 ((:dorsal-supine-continuum 1.00)))
            (0.45 ((:ischial-bilateral 0.75) (:plantar-bilateral 0.25)))
            (0.75 ((:plantar-bilateral 0.85) (:ischial-bilateral 0.15)))
            (1.00 ((:plantar-bilateral 1.00)))))
      (xform-balance-biped-zmp)))

(defmodifier specialize-with-armrest-rails
  [balance-state &key (rail-load-share 0.12) (grasp-pull-allowed false)]
  (-> balance-state
      (splice-auxiliary-manifold
        :patch-id :ulnar-armrest-bilateral :anatomical-frames [:ulna-l :ulna-r]
        :surface-normal [0.0 0.0 1.0] :cop-box-mm '(:x [-95.0 95.0] :y [-30.0 30.0])
        :mu 0.45 :bilateral-grasp-pull grasp-pull-allowed
        :weight-law (:envelope [0.0 -> rail-load-share -> 0.04]) :window-ms [150.0 -> 950.0])))

(defmodifier specialize-with-unilateral-wall-brace
  [balance-state &key (side :left) (wall-normal [0.0 -1.0 0.0]) (max-shear-mu 0.60)]
  (let [frame-node (if (eq side :left) :palmar-carpal-l :palmar-carpal-r)]
    (-> balance-state
        (splice-auxiliary-manifold
          :patch-id :unilateral-palmar-brace :anatomical-frames [frame-node]
          :surface-normal wall-normal :cop-box-mm '(:x [-45.0 45.0] :y [-45.0 45.0])
          :mu max-shear-mu :bilateral-grasp-pull false
          :weight-law (:envelope [0.0 -> 0.22 -> 0.0]) :window-ms [0.0 -> 1100.0]))))
