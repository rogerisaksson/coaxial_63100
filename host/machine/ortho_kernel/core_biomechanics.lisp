;; =============================================================================
;; FILE: core_biomechanics.lisp
;; DOMAIN: Physical Invariants, Contact Patch Registry & Nominal Baseline State
;; =============================================================================
(defpackage :ortho-dsp.core
  (:export #:physical-invariants
           #:anatomical-manifold-registry
           #:nominal-biped-gait))
(in-package :ortho-dsp.core)

(defconstants physical-invariants
  :gravity-mps2             9.80665
  :nominal-foot-width-mm    96.5    ; Derived from telemetry: 103.0mm track - 6.5mm clearance
  :min-coronal-clearance-mm 6.5     ; Medial malleolus anti-collision floor at mid-swing
  :min-normal-fz-ratio      0.35    ; Fz >= 0.35*m*g (clamps vertical accel d2z/dt2 >= -6.37 m/s^2)
  :min-zmp-margin-mm        15.0    ; Planar projection safety margin
  :min-wrench-margin-nm     14.0    ; 6D SE(3)* polyhedral CWC margin
  :max-dc-bus-voltage-v     60.0    ; Inverter bus overvoltage regeneration threshold
  :max-winding-temp-c       115.0)  ; Stator copper thermal derating ceiling

(defcontact-patch anatomical-manifold-registry
  (:patch :plantar-bilateral
    :frames          [:calcaneus-l :calcaneus-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-65.0 175.0] :y [-48.0 48.0])
    :default-mu      0.80
    :unilateral-fz   true)

  (:patch :ischial-bilateral
    :frames          [:ischial-tuberosity-l :ischial-tuberosity-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-90.0 110.0] :y [-145.0 145.0])
    :default-mu      0.65
    :unilateral-fz   true)

  (:patch :trochanter-gluteal-lateral
    :frames          [:greater-trochanter-l :greater-trochanter-r]
    :local-normal    [0.0 1.0 0.0]
    :cop-bounds-mm   (:x [-110.0 110.0] :y [-85.0 85.0])
    :default-mu      0.60
    :unilateral-fz   true)

  (:patch :dorsal-supine-continuum
    :frames          [:sacral :l4-l5-lumbar :c7-t12-thoracic :occipital]
    :local-normal    [-1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-420.0 480.0] :y [-190.0 190.0])
    :default-mu      0.55
    :unilateral-fz   true)

  (:patch :palmar-carpal-bilateral
    :frames          [:palmar-carpal-l :palmar-carpal-r]
    :local-normal    [1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-45.0 45.0] :y [-45.0 45.0])
    :default-mu      0.90
    :unilateral-fz   false)) ; Capable of bilateral form/force closure grasp

(defstate nominal-biped-gait
  (:clock
    :bpm 110.0
    :stance-duty-pwm 0.62
    :swing-gate 0.00
    :phase-jitter-ms 0.0)

  (:centroidal-balance
    :state-space :se3-dual-wrench
    :central-axis :poinsot-screw-axis
    :capture-lead-time-s 0.32
    :qp-solver :active-set-warmstart
    :manifold-weights (:plantar-bilateral          1.00
                       :ischial-bilateral          0.00
                       :trochanter-gluteal-lateral 0.00
                       :dorsal-supine-continuum    0.00
                       :palmar-carpal-bilateral    0.00)
    :nullspace-projector (:consistency :inertia-weighted-bar-j
                          :auto-release-unloaded-limbs true
                          :passive-cone-atten-db -32.0))

  (:plantar-boundary
    :sole-stack-z-mm 0.0
    :foot-collision-w-mm 96.5
    :talocrural-pitch-bias-rad 0.0
    :support-polygon-x-mm 240.0
    :windlass-arch-q 1.0
    :metatarsal-splay-gain 1.0)

  (:kinematics
    :calcaneus       (:strike-mode :heel-to-toe :saturation :linear
                      :attack-ms 35.0 :decay-ms 120.0 :sustain 0.40)
    :tibiofemoral    (:flexion-cushion-rad 0.24 :compliance 0.80 :rebound-q 0.50)
    :tarsal-coronal  (:track-width-mm 103.0 :swing-circumduction-mm 0.0)
    :tarsal-sagittal (:stride-length-mm 640.0 :step-elevation-z-mm 0.0 :swing-clearance-z-mm 38.0)
    :sacroiliac-z    (:vertical-oscillation-mm 22.7 :damping-ratio 0.707)
    :iliac-coronal   (:lfo-amp-rad 0.065 :wave :triangle :stereo-width 0.35)
    :l4-l5-lumbar    (:dc-offset-rad -0.14 :q-factor 0.40)
    :c7-t12-thoracic (:dc-offset-rad 0.04 :cutoff-hz 8.0 :slope-db 6)
    :glenohumeral    (:phase-shift-rad 3.14159 :gain-db -3.0 :phase-lag-rad 0.0 :abduction-rad 0.05)
    :cubital         (:flexion-bias-rad 0.28 :compliance 0.65)
    :c1-c2-atlas     (:dc-offset-rad 0.0 :ceiling-rad 0.12)
    :craniofacial    (:velocity-sensitivity 0.80)))
