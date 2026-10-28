;; =============================================================================
;; FILE: posture_specializers.lisp
;; DOMAIN: Composable Null-Space & Modal-Column Specializers for Canonical Poses
;; REQUIRES: canonical_posture_bundle.lisp
;; =============================================================================
(defpackage :ortho-dsp.posture-xforms
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core)
  (:export #:specialize-posture-dangling-crural-hang
           #:specialize-posture-appendage-brace
           #:specialize-posture-seated-archetype
           #:specialize-posture-recumbent-archetype
           #:specialize-posture-protective-breakfall))
(in-package :ortho-dsp.posture-xforms)

;; -----------------------------------------------------------------------------
;; 1. DANGLING CRURAL PENDULUM SPECIALIZER (Formally replaces `HUNG` in motion.py)
;; Continuously monitors plantar normal force Fz_plantar and leg extension rho.
;; When sitting on an overheight stool (0.75m - 0.82m) or bed edge where feet
;; leave the floor, decouples the lower legs from `mode_axial_compression` and
;; lets them swing as lightly damped passive gravity pendulums.
;; -----------------------------------------------------------------------------
(defmodifier specialize-posture-dangling-crural-hang
  [posture-state &key (pendulum-natural-freq-hz 1.15)
                      (passive-ankle-droop-rad -0.24)
                      (swing-damping-ratio 0.65)]
  (-> posture-state
      (modulate-unloaded-limb-jet
        :monitored-patches [:plantar-bilateral :metatarsal-equinus-bilateral]
        :target-chain      [:tibiofemoral :talocrural :calcaneus]
        ;; Smooth activation weight eta_hang = (1 - w_plantar) * smoothstep(0.88, 0.99, rho)
        :activation-law    :unloaded-reach-exhaustion
        :override-origin   (:tibiofemoral 1.5708 :talocrural passive-ankle-droop-rad)
        :zero-modal-columns [:mode-axial-compression :mode-sagittal-shift]
        :passive-pendulum-dynamics
          (:eigenfreq-hz pendulum-natural-freq-hz
           :damping-ratio swing-damping-ratio))))

;; -----------------------------------------------------------------------------
;; 2. APPENDAGE BRACE & VAULT SPECIALIZER (Formally replaces `brace` in motion.py)
;; Automatically warps the upper-extremity rows of B_modal(w) whenever the hands
;; or forearms carry load (`w_palmar > 0` or `w_ulnar > 0`), turning
;; `mode_axial_compression` into active arm push-off / eccentric lowering!
;; -----------------------------------------------------------------------------
(defmodifier specialize-posture-appendage-brace
  [posture-state &key (brace-style :carpal-seat-push) ; :carpal-seat-push | :femoral-knee-rest | :mattress-elbow-prop
                      (triceps-extension-gain 0.85)]
  (-> posture-state
      (warp-modal-basis-submatrix
        :active-when-manifold [:ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :target-nodes [:glenohumeral :cubital :c7-t12-thoracic]
        :style-preset
          (case brace-style
            ;; Hands pushing flat against chair seat or bed beside hips:
            (:carpal-seat-push
              (:q0-shift   (:glenohumeral -0.35 :cubital 0.68 :c7-t12-thoracic 0.14)
               :axial-col  (:glenohumeral (- (* 0.50 triceps-extension-gain))
                            :cubital      (* 0.95 triceps-extension-gain))))
            ;; Hands braced on own knees during deep squat or exhausted sit-to-stand:
            (:femoral-knee-rest
              (:q0-shift   (:glenohumeral +0.42 :cubital 0.45 :l4-l5-lumbar 0.28)
               :axial-col  (:glenohumeral +0.30 :cubital +0.65 :l4-l5-lumbar -0.25)))
            ;; Propped on elbows in supine/recumbent bed state:
            (:mattress-elbow-prop
              (:q0-shift   (:glenohumeral -0.65 :cubital 1.52 :c7-t12-thoracic 0.48)
               :pitch-col  (:glenohumeral -0.30 :c7-t12-thoracic +0.50)))))))

;; -----------------------------------------------------------------------------
;; 3. SEATED ARCHETYPE SPECIALIZER (Executive Upright, Lounge Slouch, Cross-Leg)
;; Modulates the `:ischial-seated` 1-jet in Null-Space without disturbing CWC.
;; -----------------------------------------------------------------------------
(defmodifier specialize-posture-seated-archetype
  [posture-state &key (archetype :executive-attentive)] ; :executive-attentive | :relaxed-lounge | :sartorius-cross-leg
  (-> posture-state
      (inject-nullspace-posture-delta
        :gate-by-manifold-weight (:ischial-bilateral :min-w 0.50)
        :delta-angles
          (case archetype
            (:executive-attentive
              '((:l4-l5-lumbar -0.14) (:c7-t12-thoracic +0.02)
                (:glenohumeral 0.04)  (:cubital 0.65) (:c1-c2-atlas 0.00)))
            (:relaxed-lounge
              '((:coxofemoral 1.22)   (:tibiofemoral 1.18) (:l4-l5-lumbar +0.18)
                (:c7-t12-thoracic +0.22) (:glenohumeral -0.18) (:cubital 0.35)))
            (:sartorius-cross-leg
              '((:coxofemoral-l 1.44) (:coxofemoral-adduct-l 0.34)
                (:coxofemoral-ext-rot-l 0.28) (:tibiofemoral-l 1.38)))))))

;; -----------------------------------------------------------------------------
;; 4. RECUMBENT BED ARCHETYPE SPECIALIZER (Supine Aligned, Lateral Fetal, Propped)
;; -----------------------------------------------------------------------------
(defmodifier specialize-posture-recumbent-archetype
  [posture-state &key (archetype :supine-aligned) (pillow-height-mm 65.0)]
  (let [neck-flex-rad (* 0.0032 pillow-height-mm)]
    (-> posture-state
        (inject-nullspace-posture-delta
          :gate-by-manifold-weight (:dorsal-supine-continuum :min-w 0.40)
          :delta-angles
            (case archetype
              (:supine-aligned
                `((:coxofemoral 0.04) (:tibiofemoral 0.06) (:talocrural -0.20)
                  (:glenohumeral 0.05) (:cubital 0.22) (:c1-c2-atlas ,neck-flex-rad)))
              (:lateral-fetal-curl
                `((:coxofemoral 0.92) (:tibiofemoral 1.15) (:l4-l5-lumbar 0.24)
                  (:c7-t12-thoracic 0.28) (:glenohumeral 0.45) (:cubital 1.25)
                  (:c1-c2-atlas ,(+ 0.12 neck-flex-rad))))
              (:semi-fowler-propped
                `((:coxofemoral 0.65) (:tibiofemoral 0.38) (:l4-l5-lumbar 0.15)
                  (:c7-t12-thoracic 0.42) (:c1-c2-atlas ,(+ 0.18 neck-flex-rad)))))))))

;; -----------------------------------------------------------------------------
;; 5. UNLOADED FREE-FALL / UKEMI BREAKFALL POSTURE (Zero-Manifold Contingency)
;; If sum(w_m) drops below 0.08 for > 35 ms (unrecoverable slip or catastrophic
;; support collapse), morphs posture into a spherical rolling tuck (chin tucked,
;; forearms shielding cranium, knees flexed) to minimize peak impact impulse.
;; -----------------------------------------------------------------------------
(defmodifier specialize-posture-protective-breakfall
  [posture-state &key (freefall-sum-w-thresh 0.08)]
  (-> posture-state
      (bind-zero-support-contingency-posture
        :sum-of-weights-floor freefall-sum-w-thresh
        :dwell-trigger-ms 35.0
        :ukemi-tuck-q0
          (:talocrural 0.15 :tibiofemoral 1.25 :coxofemoral 1.10
           :l4-l5-lumbar 0.45 :c7-t12-thoracic 0.52
           :glenohumeral 0.85 :cubital 1.65 :c1-c2-atlas 0.48) ; Chin to chest, forearms guard head
        :isotropic-damping-ratio 1.20)))
