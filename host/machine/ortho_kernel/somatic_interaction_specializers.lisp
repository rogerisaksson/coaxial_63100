;; =============================================================================
;; FILE: somatic_interaction_specializers.lisp
;; DOMAIN: Composable Modifiers for High Chairs, Beds, Stairs & Corporal Shoving
;; =============================================================================
(defpackage :ortho-dsp.somatic-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.balance-core
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.blind-fields
        :ortho-dsp.agnostic-xforms
        :ortho-dsp.nav-core
        :ortho-dsp.somatic-core)
  (:export ;; 1. High-Chair & Standard-Chair Ingress/Egress
           #:xform-somatic-sit-ingress
           #:xform-somatic-sit-egress
           #:specialize-for-overheight-seat
           #:specialize-with-carpal-seat-boost
           ;; 2. Bed Ingress/Egress
           #:xform-somatic-bed-ingress
           #:xform-somatic-bed-egress
           ;; 3. Bidirectional Stair Ascent/Descent
           #:xform-somatic-stair-traversal
           #:specialize-stair-descent-eccentric
           #:specialize-with-wall-or-banister-skim
           ;; 4. Non-Prehensile Furniture Clearing
           #:xform-somatic-furniture-shove))
(in-package :ortho-dsp.somatic-xforms)

;; =============================================================================
;; A. UNIVERSAL CHAIR INGRESS & EGRESS (WITH OVERHEIGHT SEAT SPECIALIZATION)
;; =============================================================================

;; 1. Base Somatic Sit Ingress (Reduces to normal blind sit if Z_seat <= L_leg,
;;    automatically engages Equinus-Vault + Thigh-Slide if Z_seat > L_leg!)
(defmodifier xform-somatic-sit-ingress [state]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-blind-posterior-settle)
      (specialize-with-viscoelastic-sink-compensation)
      (specialize-with-reactive-capture-step)))

;; 2. Base Somatic Sit Egress (Stand up from any seated state)
(defmodifier xform-somatic-sit-egress [state]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-balance-supine-to-biped :ascent-ms 920.0)
      (ignite-cyclic-clock nominal-biped-gait :bars 1)))

;; 3. SPECIALIZER: Overheight Seat / Barstool / High Bench (Z_seat > Knee Height)
;; - On Ingress: Rises onto toes (:metatarsal-equinus), perches posterior thighs on
;;   the high front edge, and slides backward onto the seat while feet detach cleanly.
;; - On Egress: Scoots pelvis forward along seat bevel until toes reach the floor in
;;   plantarflexion, catches weight on elastic ankle springs, and settles heels down.
(defmodifier specialize-for-overheight-seat
  [somatic-state &key (direction :ingress) ; :ingress | :egress
                      (slide-speed-mps 0.16)]
  (case direction
    (:ingress
      (-> somatic-state
          (splice-auxiliary-manifold
            :patch-id :thigh-bevel-perch
            :anatomical-frames [:femoral-diaphysis-dorsal-l :femoral-diaphysis-dorsal-r]
            :surface-normal [0.0 0.0 1.0]
            :mu 0.45
            :weight-law (:wrench-proportional :max-share 0.65))
          ;; Vault on metatarsals and slide posteriorly until ischial tuberosities center
          (drive-tangential-coulomb-slide
            :sliding-patches [:posterior-femoral-bilateral :ischial-bilateral]
            :slide-velocity-xyz-mps [(- slide-speed-mps) 0.0 0.0]
            :max-normal-fz-during-slide 0.55
            :stop-on-ischial-centering true)))

    (:egress
      (-> somatic-state
          ;; Slide anteriorly (+X) off the high seat until reachability rho < 0.96
          ;; so the forefoot (:metatarsal-equinus) establishes ground capture cone FIRST!
          (drive-tangential-coulomb-slide
            :sliding-patches [:ischial-bilateral :posterior-femoral-bilateral]
            :slide-velocity-xyz-mps [slide-speed-mps 0.0 -0.05]
            :max-normal-fz-during-slide 0.70
            :stop-on-ischial-centering false)
          ;; Catch descending CoM on plantarflexed forefoot and lower heels smoothly
          (pan-wrench-homotopy
            :from-manifolds '((:ischial-bilateral 0.60) (:posterior-femoral-bilateral 0.40))
            :to-manifolds   '((:metatarsal-equinus-bilateral 0.75) (:plantar-bilateral 0.25))
            :duration-ms 340.0)
          (xform-balance-biped-zmp)))))

;; 4. SPECIALIZER: Carpal-Heel Seat Push-Off / Vault Assist (No Gripping)
;; Places flat palms/wrists on the seat surface or armrests beside the hips to push
;; the pelvis up onto a high chair or boost egress from a deep/high fixture.
(defmodifier specialize-with-carpal-seat-boost
  [somatic-state &key (max-arm-push-share 0.28)]
  (-> somatic-state
      (splice-auxiliary-manifold
        :patch-id :carpal-seat-vault
        :anatomical-frames [:carpal-heel-l :carpal-heel-r]
        :surface-normal [0.0 0.0 1.0]
        :mu 0.65
        :bilateral-grasp-pull false             ; Strictly flat-palm downward push (Fz >= 0)
        :weight-law (:wrench-proportional :max-share max-arm-push-share))))


;; =============================================================================
;; B. SOMATIC BED INGRESS & EGRESS (PERCH -> SLIDE -> TROCHANTERIC PENDULUM)
;; =============================================================================

;; 1. Bed Ingress: Perch on mattress edge -> Slide pelvis deep -> Pendulum leg sweep -> Supine
(defmodifier xform-somatic-bed-ingress [state &key (bed-yaw-rad 1.5708)]
  (-> state
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (xform-blind-posterior-settle)
      ;; Slide 180mm deeper onto mattress so pelvis isn't perched precariously on the edge
      (drive-tangential-coulomb-slide
        :sliding-patches [:posterior-femoral-bilateral :ischial-bilateral]
        :slide-velocity-xyz-mps [-0.15 0.0 0.0]
        :max-normal-fz-during-slide 0.60)
      ;; Coupled torso lowering levers legs off the floor into supine alignment
      (xform-blind-recumbent-flow)
      (specialize-with-viscoelastic-sink-compensation)
      (release-impedance-wavefront :terminal-mode :supine-high-z)))

;; 2. Bed Egress: Wake Inverters -> Supine-to-Ischial Roll -> Edge Slide-Drop -> Stand
(defmodifier xform-somatic-bed-egress [state]
  (-> state
      (wake-inverter-closed-loop)
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      ;; Roll from dorsal spine onto lateral trochanter and ischial tuberosities while
      ;; dropping lower legs over the mattress edge as a gravity counterweight!
      (sequence-wrench-homotopy
        :total-duration-ms 1100.0
        :keyframes
          '((0.00 ((:dorsal-supine-continuum 1.00)))
            (0.50 ((:trochanter-gluteal-lateral 0.55) (:ischial-bilateral 0.45)))
            (1.00 ((:ischial-bilateral 0.85) (:posterior-femoral-bilateral 0.15)))))
      ;; Use flat carpal push against mattress + anterior slide until feet plant on floor
      (specialize-with-carpal-seat-boost :max-arm-push-share 0.22)
      (specialize-for-overheight-seat :direction :egress :slide-speed-mps 0.14)
      (xform-somatic-sit-egress)))


;; =============================================================================
;; C. BIDIRECTIONAL STAIR TRAVERSAL (ASCENT & DESCENT)
;; =============================================================================

;; 1. Base Stair Engine (Automatically switches Ascent vs Descent from Sub-Band SDF dZ)
(defmodifier xform-somatic-stair-traversal
  [gait &key (nominal-riser-dz-mm 175.0) (nominal-tread-dx-mm 280.0)]
  (-> gait
      (quantize-grid :bpm (:set 90.0) :stance-duty-pwm (:set 0.66) :swing-gate 0.0)
      (modulate-kinematic-reachability :max-leg-reach-mm 890.0)
      (arpeggiate-elevation-energy
        :step-dz-mm nominal-riser-dz-mm
        :step-dx-mm nominal-tread-dx-mm
        :nosing-clearance-mm 34.0)
      (enforce-dcm-barrier-zcbf :barrier-gamma 15.0 :min-capture-margin-mm 16.0)))

;; 2. SPECIALIZER: Steep Stair Descent (dZ < 0 Eccentric Braking & Forefoot Catch)
;; Reverses potential energy flow: lowers CoM via stance-knee eccentric damping,
;; tilts pelvis coronally to extend swing reach below stance plane, and lands on
;; metatarsal equinus patch to absorb impact without heel-clipping the stair riser.
(defmodifier specialize-stair-descent-eccentric
  [stair-state &key (riser-drop-mm -175.0) (tread-depth-mm 280.0)]
  (-> stair-state
      (arpeggiate-elevation-energy
        :step-dz-mm riser-drop-mm               ; Negative dZ triggers descent branch
        :step-dx-mm tread-depth-mm
        :nosing-clearance-mm 38.0)
      ;; Shift sagittal CoM bias slightly posterior so braking shear stays inside tread cone
      (apply-filter-bank
        (dc-bias :target :c7-t12-thoracic :offset-rad +0.06 :zmp-counter-bias-x-mm -18.0)
        (dc-bias :target :glenohumeral    :abduction-rad 0.18 :gain-db -1.0))))

;; 3. SPECIALIZER: Non-Prehensile Wall / Banister Skim (Sliding Forearm/Palm Stabilizer)
;; Slides open palm or forearm lightly along a stairwell wall or handrail (no gripping!)
;; to bound coronal sway and assist eccentric braking during steep stair traversal.
(defmodifier specialize-with-wall-or-banister-skim
  [stair-state &key (side :right) (skim-normal [0.0 1.0 0.0]) (max-skim-share 0.10)]
  (let [frame-node (if (eq side :right) :ulna-distal-r :ulna-distal-l)]
    (-> stair-state
        (splice-auxiliary-manifold
          :patch-id :stairwell-sliding-rail
          :anatomical-frames [frame-node]
          :surface-normal skim-normal
          :mu 0.25                              ; Low friction allows continuous sliding along rail
          :bilateral-grasp-pull false
          :weight-law (:wrench-proportional :max-share max-skim-share)))))


;; =============================================================================
;; D. NON-PREHENSILE SOMATIC FURNITURE SHOVING (FOREARM / HIP / CARPAL PUSH)
;; =============================================================================

;; Displaces an obstructing chair or light table out of the way using forearm hooks,
;; flat carpal-heel pushes, or lateral hip bumps—zero finger actuation required.
(defmodifier xform-somatic-furniture-shove
  [state &key (shove-technique :ulnar-backrest-hook) ; :ulnar-backrest-hook | :carpal-flat-push | :trochanter-hip-bump
              (clearing-offset-xy-mm [0.0 450.0])]
  (let [active-patch (case shove-technique
                       (:trochanter-hip-bump :trochanter-gluteal-lateral)
                       (otherwise            :ulnar-carpal-shove-bilateral))]
    (-> state
        (configure-momentum-observer :observer-bandwidth-hz 40.0)
        (couple-non-prehensile-shove
          :contact-patch active-patch
          :coupling-topology shove-technique
          :max-shove-shear-n 115.0)
        (xform-geodesic-wayfinder-base :goal-xyz clearing-offset-xy-mm))))
