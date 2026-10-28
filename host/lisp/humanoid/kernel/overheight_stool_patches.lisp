;; =============================================================================
;; FILE: overheight_stool_patches.lisp
;; DOMAIN: Complete 1 kHz SoA Schedule for High Stool & High Chair Vault/Dip
;; =============================================================================
(defpackage :ortho-dsp.high-vault-patches
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.surfaces
        :ortho-dsp.balance-xforms
        :ortho-dsp.pivots
        :ortho-dsp.supervisor
        :ortho-dsp.lowering
        :ortho-dsp.bedside-xforms
        :ortho-dsp.high-vault-core
        :ortho-dsp.high-vault-xforms)
  (:export #:patch-mount-overheight-stool-with-arm-vault
           #:patch-dismount-overheight-stool-with-arm-dip
           #:compile-overheight-vault-and-dip-soa))
(in-package :ortho-dsp.high-vault-patches)

;; 1. Stand by High Stool/Chair -> Turn 180 -> Calf Datum -> In-Place Arm-Vault Past Edge -> Balance
(defpatch patch-mount-overheight-stool-with-arm-vault [base-gait]
  (-> base-gait
      (transducer-viscoelastic-flat)
      (xform-balance-biped-zmp)
      ;; Stand alongside high stool, turn 180 deg, index calves against front frame:
      (xform-stand-and-turn-at-fixture :turn-yaw-rad 3.14159 :standoff-clearance-mm 30.0)
      ;; Plant hands on seat rim/surface, pin stool stability cone, lift & row pelvis
      ;; up past the front bevel on arm support, walk hands back, and settle at centroid:
      (xform-overheight-arm-vault-ingress :bevel-unweight-ratio 0.84 :rowing-speed-mps 0.16)
      (specialize-vault-hand-anchor-topology :anchor-style :lateral-seat-rim-cup :seat-width-mm 340.0)
      (specialize-vault-for-light-tippy-stool :estimated-stool-mass-kg 5.0 :base-radius-mm 200.0)
      (specialize-vault-abort-to-plantar-drop)
      (specialize-with-stool-active-core-balance :seat-diameter-mm 340.0)
      (release-impedance-wavefront :terminal-mode :seated-active-rest)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; 2. Dismount High Stool/Chair -> Hand Anchor -> Scoot Clear of Edge -> Eccentric Arm-Dip -> Stand
(defpatch patch-dismount-overheight-stool-with-arm-dip [seated-high-stool-state]
  (-> seated-high-stool-state
      ;; Anchor hands on seat corners, scoot buttocks 55mm past front bevel, lower pelvis
      ;; vertically via eccentric triceps dip until plantarflexed toes catch the floor, then stand!
      (xform-overheight-arm-dip-egress :dip-lowering-speed-mps 0.15 :max-dip-travel-mm 167.0) ; her dip
      (specialize-vault-hand-anchor-topology :anchor-style :flat-carpal-beside-hips :seat-width-mm 340.0)
      (specialize-vault-for-light-tippy-stool :estimated-stool-mass-kg 5.0 :base-radius-mm 200.0)
      (specialize-dip-egress-with-footrest-rung)
      (limiter-high-mu-planar)
      (master-anti-spasm-insert)))

;; -----------------------------------------------------------------------------
;; DETERMINISTIC 1 kHz SOA COMPILATION
;; -----------------------------------------------------------------------------
(defpatch compile-overheight-vault-and-dip-soa
  (let [mount-high-stool    (patch-mount-overheight-stool-with-arm-vault nominal-biped-gait)
        dismount-high-stool (patch-dismount-overheight-stool-with-arm-dip mount-high-stool)]
    (compile-to-soa
      :align-bytes 64
      :sample-rate-hz 1000.0
      :target-bank :primary-bank-a
      :schedule
        [(oneshot  mount-high-stool)    ; Turn -> Calf Datum -> Arm-Vault Past Edge -> Hand Replant -> Sit
         (dwell-ms 4000)                ; Active core balance on backless high stool (dangling legs in Null-Space)
         (oneshot  dismount-high-stool)]))) ; Hand Anchor -> Scoot Past Edge -> Eccentric Triceps Dip -> Toe Catch -> Stand
