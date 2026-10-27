;; =============================================================================
;; FILE: canonical_posture_bundle.lisp
;; DOMAIN: 1-Jet Posture Atlas (Origin q_0 + Modal Basis B_m) & Manifold Blending
;; REQUIRES: ortho_dsp_somatic_substrate.lisp
;; =============================================================================
(defpackage :ortho-dsp.postures-core
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core)
  (:export #:canonical-posture-soa-atlas
           #:blend-manifold-posture-jet
           #:reconstruct-modal-pose
           #:xform-posture-manifold-synthesizer))
(in-package :ortho-dsp.postures-core)

;; -----------------------------------------------------------------------------
;; 1. CANONICAL 1-JET POSTURE ATLAS (STATIC 64B-ALIGNED SOA TABLE)
;; Maps each contact manifold in `ortho.MANIFOLDS` to:
;;   - :origin-q0          -> Canonical resting angles when modes == [0, 0, 0]
;;   - :modal-basis-columns -> How `acts.py` [axial, sagittal, pitch] map to joints
;;   - :gravity-hang-law   -> Replaces `HUNG` in `motion.py` when limb unloads
;; -----------------------------------------------------------------------------
(defstate canonical-posture-soa-atlas
  (:memory-layout
    :num-canonical-postures 7
    :num-modal-columns 3                     ; [mode_axial_compression, mode_sagittal_shift, mode_pitch_attitude]
    :align-bytes 64)

  (:postures
    ;; -------------------------------------------------------------------------
    ;; POSTURE 1: PLANTAR UPRIGHT STAND (Replaces static `figure.stand()`)
    ;; -------------------------------------------------------------------------
    (:posture :plantar-upright
      :coupled-manifold :plantar-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural 0.02 :tibiofemoral 0.08 :coxofemoral -0.06
         :iliac-coronal 0.00 :l4-l5-lumbar -0.12 :c7-t12-thoracic 0.04
         :glenohumeral 0.05 :cubital 0.24 :c1-c2-atlas 0.00)
      :modal-basis-columns
        ;; Closed-chain bipedal squat & ankle-hip strategy:
        (:mode-axial-compression (:talocrural +0.52 :tibiofemoral +1.05 :coxofemoral +0.58
                                  :l4-l5-lumbar +0.12 :glenohumeral -0.15)
         :mode-sagittal-shift    (:talocrural +0.38 :tibiofemoral -0.10 :coxofemoral +0.42
                                  :l4-l5-lumbar +0.18 :c1-c2-atlas -0.20)
         :mode-pitch-attitude    (:coxofemoral +0.65 :l4-l5-lumbar +0.35 :c7-t12-thoracic +0.25
                                  :c1-c2-atlas -0.85)) ; Counter-pitch atlas to preserve horizon gaze!
      :gravity-hang-law (:enabled false))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 2: METATARSAL EQUINUS PERCH (Toe-tip vault / stair descent catch)
    ;; -------------------------------------------------------------------------
    (:posture :equinus-digitigrade
      :coupled-manifold :metatarsal-equinus-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.48 :tibiofemoral 0.28 :coxofemoral -0.18
         :iliac-coronal 0.00 :l4-l5-lumbar -0.16 :c7-t12-thoracic 0.08
         :glenohumeral 0.14 :cubital 0.45 :c1-c2-atlas 0.00)
      :modal-basis-columns
        (:mode-axial-compression (:talocrural -0.15 :tibiofemoral +1.15 :coxofemoral +0.72
                                  :glenohumeral +0.35) ; Cantilever arms forward as knees flex on toes
         :mode-sagittal-shift    (:talocrural +0.22 :coxofemoral +0.55 :l4-l5-lumbar +0.20)
         :mode-pitch-attitude    (:coxofemoral +0.60 :l4-l5-lumbar +0.40 :c1-c2-atlas -0.80))
      :gravity-hang-law (:enabled false))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 3: ISCHIAL SEATED (Replaces `seated.posed` + `HUNG` overrides!)
    ;; Pelvis is anchored on seat; axial compression acts on SPINE, not knees!
    ;; -------------------------------------------------------------------------
    (:posture :ischial-seated
      :coupled-manifold :ischial-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural 0.00 :tibiofemoral 1.52 :coxofemoral 1.48
         :iliac-coronal 0.00 :l4-l5-lumbar -0.06 :c7-t12-thoracic 0.06
         :glenohumeral 0.08 :cubital 0.55 :c1-c2-atlas 0.00)
      :modal-basis-columns
        ;; In seated posture, axial compression settles the lumbar/thoracic spine & shoulders
        ;; while leaving knee/ankle completely unperturbed!
        (:mode-axial-compression (:talocrural 0.00 :tibiofemoral 0.00 :coxofemoral +0.12
                                  :l4-l5-lumbar +0.28 :c7-t12-thoracic +0.32 :glenohumeral +0.10)
         ;; Sagittal shift hinges torso forward over femurs (for sit-to-stand prep) and
         ;; slides tibias slightly back WITHOUT standing-squat ankle distortion:
         :mode-sagittal-shift    (:talocrural +0.18 :tibiofemoral +0.22 :coxofemoral +0.75
                                  :l4-l5-lumbar +0.25 :glenohumeral +0.30 :c1-c2-atlas -0.45)
         :mode-pitch-attitude    (:coxofemoral +0.55 :l4-l5-lumbar +0.45 :c7-t12-thoracic +0.30
                                  :c1-c2-atlas -0.85))
      ;; Passive Gravity-Pendulum Law (Eliminates `HUNG` in `motion.py`):
      ;; When sitting on a high chair (0.75m - 0.82m) where w_plantar -> 0 and rho -> 1.0,
      ;; shanks relax to vertical gravity hang (1.57 rad) and ankles drop into slight plantarflexion (-0.22 rad).
      :gravity-hang-law
        (:enabled true
         :trigger-on-unloaded-patch [:plantar-bilateral :metatarsal-equinus-bilateral]
         :dangling-q0 (:talocrural -0.22 :tibiofemoral 1.5708 :coxofemoral 1.38)
         :stiffness-atten-db -24.0))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 4: POSTERIOR-FEMORAL BEVEL PERCH (High-chair / bed-edge scoot)
    ;; -------------------------------------------------------------------------
    (:posture :femoral-bevel-perch
      :coupled-manifold :posterior-femoral-bilateral
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.32 :tibiofemoral 1.15 :coxofemoral 1.10
         :iliac-coronal 0.00 :l4-l5-lumbar -0.10 :c7-t12-thoracic 0.12
         :glenohumeral -0.20 :cubital 0.35 :c1-c2-atlas 0.00)
      :modal-basis-columns
        (:mode-axial-compression (:tibiofemoral +0.35 :coxofemoral +0.40 :l4-l5-lumbar +0.20)
         :mode-sagittal-shift    (:coxofemoral +0.65 :l4-l5-lumbar +0.25 :glenohumeral -0.35)
         :mode-pitch-attitude    (:coxofemoral +0.50 :l4-l5-lumbar +0.40 :c1-c2-atlas -0.75))
      :gravity-hang-law
        (:enabled true
         :trigger-on-unloaded-patch [:plantar-bilateral :metatarsal-equinus-bilateral]
         :dangling-q0 (:talocrural -0.28 :tibiofemoral 1.35 :coxofemoral 1.15)
         :stiffness-atten-db -20.0))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 5: TROCHANTER LATERAL RECUMBENT (Side-lying transition bridge)
    ;; -------------------------------------------------------------------------
    (:posture :trochanter-lateral
      :coupled-manifold :trochanter-gluteal-lateral
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.10 :tibiofemoral 0.85 :coxofemoral 0.78
         :iliac-coronal 0.28 :l4-l5-lumbar 0.10 :c7-t12-thoracic 0.14
         :glenohumeral 0.35 :cubital 0.95 :c1-c2-atlas 0.18)
      :modal-basis-columns
        (:mode-axial-compression (:tibiofemoral +0.55 :coxofemoral +0.50 :l4-l5-lumbar +0.30) ; Curl into fetal tuck
         :mode-sagittal-shift    (:coxofemoral +0.35 :iliac-coronal -0.25 :c7-t12-thoracic +0.30)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.35 :c7-t12-thoracic +0.40 :c1-c2-atlas +0.30))
      :gravity-hang-law (:enabled false))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 6: DORSAL SUPINE REST (Lying flat on bed / floor)
    ;; -------------------------------------------------------------------------
    (:posture :dorsal-supine
      :coupled-manifold :dorsal-supine-continuum
      :origin-q0
        (:calcaneus 0.00 :talocrural -0.18 :tibiofemoral 0.06 :coxofemoral 0.04
         :iliac-coronal 0.00 :l4-l5-lumbar -0.04 :c7-t12-thoracic 0.02
         :glenohumeral 0.02 :cubital 0.18 :c1-c2-atlas 0.04)
      :modal-basis-columns
        ;; In supine posture, axial compression draws knees up toward chest / flattens lumbar
        ;; while pitch attitude props the upper thorax and head up (reading/waking in bed):
        (:mode-axial-compression (:tibiofemoral +0.85 :coxofemoral +0.45 :l4-l5-lumbar +0.18)
         :mode-sagittal-shift    (:coxofemoral +0.30 :l4-l5-lumbar +0.40 :c7-t12-thoracic +0.45)
         :mode-pitch-attitude    (:l4-l5-lumbar +0.25 :c7-t12-thoracic +0.55 :c1-c2-atlas +0.45))
      :gravity-hang-law (:enabled false))

    ;; -------------------------------------------------------------------------
    ;; POSTURE 7: ULNAR-PALMAR BRACED (Replaces ad-hoc `brace` in `motion.py`)
    ;; -------------------------------------------------------------------------
    (:posture :appendage-braced
      :coupled-manifold :ulnar-carpal-shove-bilateral
      :origin-q0
        (:glenohumeral -0.38 :cubital 0.62 :l4-l5-lumbar 0.08 :c7-t12-thoracic 0.16)
      :modal-basis-columns
        ;; Maps axial compression directly into elbow/shoulder triceps push-off extension!
        (:mode-axial-compression (:glenohumeral -0.45 :cubital +0.80 :c7-t12-thoracic +0.15)
         :mode-sagittal-shift    (:glenohumeral +0.55 :cubital -0.35 :l4-l5-lumbar +0.20)
         :mode-pitch-attitude    (:glenohumeral -0.25 :c7-t12-thoracic +0.30))
      :gravity-hang-law (:enabled false))))

;; -----------------------------------------------------------------------------
;; 2. CONTINUOUS BARYCENTRIC 1-JET BLENDER (1 kHz BRANCHLESS HOMOTOPY)
;; Reads `out[MANIFOLDED]` weights w_m(t) and reachability rho(t) to synthesize
;; the composite baseline posture `g['posture']` and the composite 3-column
;; modal matrix B_modal(w, rho) in a single SIMD pass.
;; -----------------------------------------------------------------------------
(defprimitive blend-manifold-posture-jet
  [state &key (atlas 'canonical-posture-soa-atlas)
              (weight-source :active-manifold-weights) ; `out[MANIFOLDED]` from observer.py
              (reachability-source :live-limb-rho)
              (slew-filter-cutoff-hz 14.0)]
  (-> state
      (synthesize-1jet-posture-bundle
        :posture-atlas atlas
        :manifold-weights weight-source
        :reachability-foliation reachability-source
        ;; Geodesic / barycentric interpolation of origin q_0(w) and tangent basis B(w):
        ;; q_base(w)  = sum_m w_m * ((1 - eta_hang_m) * q_0_m + eta_hang_m * q_grav_m)
        ;; B_modal(w) = sum_m w_m * (I - diag(eta_hang_m)) * B_m
        :interpolation-space :so3-product-lie-algebra
        :lowpass-smoothing-hz slew-filter-cutoff-hz
        :outputs [:composite-base-posture-q0         ; Direct drop-in for `g['posture']`!
                  :composite-modal-jacobian-3xn      ; Direct drop-in for `modal.poses` `bend` basis!
                  :composite-joint-stiffness-mask])))

;; -----------------------------------------------------------------------------
;; 3. UNIFIED MODAL POSE RECONSTRUCTOR (DROP-IN REPLACEMENT FOR `modal.poses`)
;; Decodes the 3 scalar modes from `acts.py` through the manifold-blended
;; tangent basis B_modal(w, rho) instead of a hardcoded standing-squat vector.
;; -----------------------------------------------------------------------------
(defprimitive reconstruct-modal-pose
  [state &key (mode-axial-compression 0.0)
              (mode-sagittal-shift 0.0)
              (mode-pitch-attitude 0.0)]
  (-> state
      (evaluate-1jet-modal-expansion
        :base-origin     :composite-base-posture-q0
        :tangent-basis   :composite-modal-jacobian-3xn
        :modal-vector-m3 [mode-axial-compression
                          mode-sagittal-shift
                          mode-pitch-attitude]
        ;; Project any secondary postural specializers strictly into P_dyn = I - J_c^T * J_bar^T
        ;; so stylistic posture tweaks never corrupt active contact wrenches!
        :nullspace-overlay :active-posture-specializer-delta
        :output-channel    :kinematic-channel-strip-targets)))

;; -----------------------------------------------------------------------------
;; 4. MASTER POSTURE SYNTHESIZER MODIFIER
;; -----------------------------------------------------------------------------
(defmodifier xform-posture-manifold-synthesizer
  [state &key (axial-mode 0.0) (sagittal-mode 0.0) (pitch-mode 0.0)]
  (-> state
      (blend-manifold-posture-jet :atlas 'canonical-posture-soa-atlas)
      (reconstruct-modal-pose
        :mode-axial-compression axial-mode
        :mode-sagittal-shift    sagittal-mode
        :mode-pitch-attitude    pitch-mode)))
