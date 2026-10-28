;; -----------------------------------------------------------------------------
;; MODULE 1.14: somatic_interaction_specializers
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.somatic-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.somatic-core)
  (:export #:xform-somatic-sit-ingress
           #:xform-somatic-sit-egress
           #:specialize-for-overheight-seat
           #:specialize-with-carpal-seat-boost
           #:xform-somatic-bed-ingress
           #:xform-somatic-bed-egress
           #:xform-somatic-stair-traversal
           #:specialize-stair-descent-eccentric
           #:specialize-with-wall-or-banister-skim))
(in-package :ortho-dsp.somatic-xforms)

(defmodifier xform-somatic-sit-ingress [state]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-blind-posterior-settle)
      (specialize-with-viscoelastic-sink-compensation)
      (specialize-with-reactive-capture-step)))

(defmodifier xform-somatic-sit-egress [state]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-balance-supine-to-biped :ascent-ms 920.0)
      (ignite-cyclic-clock nominal-biped-gait :bars 1)))

(defmodifier specialize-for-overheight-seat
  [somatic-state &key (direction :ingress) (slide-speed-mps 0.16)]
  (case direction
    (:ingress
      (-> somatic-state
          (splice-auxiliary-manifold
            :patch-id :thigh-bevel-perch
            :anatomical-frames [:femoral-diaphysis-dorsal-l :femoral-diaphysis-dorsal-r]
            :surface-normal [0.0 0.0 1.0] :mu 0.45
            :weight-law (:wrench-proportional :max-share 0.65))
          (drive-tangential-coulomb-slide
            :sliding-patches [:posterior-femoral-bilateral :ischial-bilateral]
            :slide-velocity-xyz-mps [(- slide-speed-mps) 0.0 0.0]
            :max-normal-fz-during-slide 0.55 :stop-on-ischial-centering true)))
    (:egress
      (-> somatic-state
          (drive-tangential-coulomb-slide
            :sliding-patches [:ischial-bilateral :posterior-femoral-bilateral]
            :slide-velocity-xyz-mps [slide-speed-mps 0.0 -0.05]
            :max-normal-fz-during-slide 0.70 :stop-on-ischial-centering false)
          (pan-wrench-homotopy
            :from-manifolds '((:ischial-bilateral 0.60) (:posterior-femoral-bilateral 0.40))
            :to-manifolds   '((:metatarsal-equinus-bilateral 0.75) (:plantar-bilateral 0.25))
            :duration-ms 340.0)
          (xform-balance-biped-zmp)))))

(defmodifier specialize-with-carpal-seat-boost [somatic-state &key (max-arm-push-share 0.28)]
  (-> somatic-state
      (splice-auxiliary-manifold
        :patch-id :carpal-seat-vault
        :anatomical-frames [:carpal-heel-l :carpal-heel-r]
        :surface-normal [0.0 0.0 1.0] :mu 0.65 :bilateral-grasp-pull false
        :weight-law (:wrench-proportional :max-share max-arm-push-share))))

(defmodifier xform-somatic-bed-ingress [state]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-blind-posterior-settle)
      (drive-tangential-coulomb-slide
        :sliding-patches [:posterior-femoral-bilateral :ischial-bilateral]
        :slide-velocity-xyz-mps [-0.15 0.0 0.0] :max-normal-fz-during-slide 0.60)
      (xform-blind-recumbent-flow)
      (specialize-with-viscoelastic-sink-compensation)
      (release-impedance-wavefront :terminal-mode :supine-high-z)))

(defmodifier xform-somatic-bed-egress [state]
  (-> state
      (wake-inverter-closed-loop)
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (sequence-wrench-homotopy
        :total-duration-ms 1100.0
        :keyframes
          '((0.00 ((:dorsal-supine-continuum 1.00)))
            (0.50 ((:trochanter-gluteal-lateral 0.55) (:ischial-bilateral 0.45)))
            (1.00 ((:ischial-bilateral 0.85) (:posterior-femoral-bilateral 0.15)))))
      (specialize-with-carpal-seat-boost :max-arm-push-share 0.22)
      (specialize-for-overheight-seat :direction :egress :slide-speed-mps 0.14)
      (xform-somatic-sit-egress)))

(defmodifier xform-somatic-stair-traversal
  [gait &key (nominal-riser-dz-mm 175.0) (nominal-tread-dx-mm 280.0)]
  (-> gait
      (quantize-grid :bpm (:set 90.0) :stance-duty-pwm (:set 0.66) :swing-gate 0.0)
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (arpeggiate-elevation-energy
        :step-dz-mm nominal-riser-dz-mm :step-dx-mm nominal-tread-dx-mm :nosing-clearance-mm 34.0)
      (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 16.0)))

(defmodifier specialize-stair-descent-eccentric
  [stair-state &key (riser-drop-mm -175.0) (tread-depth-mm 280.0)]
  (-> stair-state
      (arpeggiate-elevation-energy
        :step-dz-mm riser-drop-mm :step-dx-mm tread-depth-mm :nosing-clearance-mm 38.0)
      (apply-filter-bank
        (dc-bias :target :c7-t12-thoracic :offset-rad +0.06 :zmp-counter-bias-x-mm -18.0)
        (dc-bias :target :glenohumeral    :abduction-rad 0.18 :gain-db -1.0))))

(defmodifier specialize-with-wall-or-banister-skim
  [stair-state &key (side :right) (skim-normal [0.0 1.0 0.0]) (max-skim-share 0.10)]
  (let [frame-node (if (eq side :right) :ulna-distal-r :ulna-distal-l)]
    (-> stair-state
        (splice-auxiliary-manifold
          :patch-id :stairwell-sliding-rail :anatomical-frames [frame-node]
          :surface-normal skim-normal :mu 0.25 :bilateral-grasp-pull false
          :weight-law (:wrench-proportional :max-share max-skim-share)))))
