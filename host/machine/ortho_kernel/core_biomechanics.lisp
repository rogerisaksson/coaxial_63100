;; -----------------------------------------------------------------------------
;; MODULE 1.1: core_biomechanics
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.core
  (:export #:physical-invariants
           #:anatomical-manifold-registry
           #:anatomical-capsule-schema
           #:nominal-biped-gait))
(in-package :ortho-dsp.core)

(defconstants physical-invariants
  :gravity-mps2             9.80665
  :nominal-foot-width-mm    96.5    ; Telemetry derived: 103.0mm track - 6.5mm clearance
  :min-coronal-clearance-mm 6.5     ; Medial malleolus anti-collision floor at mid-swing
  :min-normal-fz-ratio      0.35    ; Fz >= 0.35*m*g (clamps vertical accel d2z/dt2 >= -6.37 m/s^2)
  :min-zmp-margin-mm        15.0    ; Planar projection safety margin
  :min-wrench-margin-nm     14.0    ; 6D SE(3)* polyhedral CWC margin
  :min-phase-margin-deg     38.0    ; Discrete 1 kHz loop phase margin floor at 0 dB crossover
  :max-dc-bus-voltage-v     60.0    ; Inverter bus overvoltage regeneration ceiling
  :max-winding-temp-c       115.0)  ; Stator copper thermal derating ceiling

(defcontact-patch anatomical-manifold-registry
  (:patch :plantar-bilateral
    :frames          [:calcaneus-l :calcaneus-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-65.0 175.0] :y [-48.0 48.0])
    :default-mu      0.80
    :unilateral-fz   true)

  (:patch :metatarsal-equinus-bilateral
    :frames          [:metatarsal-head-l :metatarsal-head-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-22.0 45.0] :y [-42.0 42.0])
    :default-mu      0.80
    :unilateral-fz   true)

  (:patch :posterior-femoral-bilateral
    :frames          [:femoral-diaphysis-dorsal-l :femoral-diaphysis-dorsal-r]
    :local-normal    [0.0 0.0 1.0]
    :cop-bounds-mm   (:x [-110.0 110.0] :y [-135.0 135.0])
    :default-mu      0.45
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

  (:patch :ulnar-carpal-shove-bilateral
    :frames          [:ulna-distal-l :ulna-distal-r :carpal-heel-l :carpal-heel-r]
    :local-normal    [1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-65.0 65.0] :y [-35.0 35.0])
    :default-mu      0.60
    :unilateral-fz   true)

  (:patch :palmar-carpal-bilateral
    :frames          [:palmar-carpal-l :palmar-carpal-r]
    :local-normal    [1.0 0.0 0.0]
    :cop-bounds-mm   (:x [-45.0 45.0] :y [-45.0 45.0])
    :default-mu      0.90
    :unilateral-fz   false))

(defstate anatomical-capsule-schema
  (:memory-layout
    :num-body-capsules 18
    :num-payload-slots 4
    :align-bytes 64
    :lse-smoothing-alpha 25.0)

  (:capsules
    :foot-shell-l        (:frame :calcaneus-l      :p0 [-0.065 0.0 0.0]   :p1 [0.175 0.0 0.0]  :r0-m 0.048 :r1-m 0.045)
    :foot-shell-r        (:frame :calcaneus-r      :p0 [-0.065 0.0 0.0]   :p1 [0.175 0.0 0.0]  :r0-m 0.048 :r1-m 0.045)
    :shank-crural-l      (:frame :tibiofemoral-l   :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.410] :r0-m 0.068 :r1-m 0.052)
    :shank-crural-r      (:frame :tibiofemoral-r   :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.410] :r0-m 0.068 :r1-m 0.052)
    :thigh-femoral-l     (:frame :coxofemoral-l    :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.420] :r0-m 0.095 :r1-m 0.072)
    :thigh-femoral-r     (:frame :coxofemoral-r    :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.420] :r0-m 0.095 :r1-m 0.072)
    :pelvic-girdle       (:frame :sacroiliac-z     :p0 [0.0 -0.145 0.0]   :p1 [0.0 +0.145 0.0] :r0-m 0.115 :r1-m 0.115)
    :lumbar-column       (:frame :l4-l5-lumbar     :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 +0.220] :r0-m 0.110 :r1-m 0.120)
    :thoracic-ribcage    (:frame :c7-t12-thoracic  :p0 [0.0 -0.155 0.18]  :p1 [0.0 +0.155 0.18] :r0-m 0.130 :r1-m 0.130)
    :cranial-vault       (:frame :c1-c2-atlas      :p0 [0.0 0.0 0.04]     :p1 [0.0 0.0 +0.190] :r0-m 0.095 :r1-m 0.105)
    :upper-arm-humeral-l (:frame :glenohumeral-l   :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.310] :r0-m 0.062 :r1-m 0.050)
    :upper-arm-humeral-r (:frame :glenohumeral-r   :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.310] :r0-m 0.062 :r1-m 0.050)
    :forearm-ulnar-l     (:frame :cubital-l        :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.260] :r0-m 0.048 :r1-m 0.038)
    :forearm-ulnar-r     (:frame :cubital-r        :p0 [0.0 0.0 0.0]      :p1 [0.0 0.0 -0.260] :r0-m 0.048 :r1-m 0.038)
    :hand-carpal-l       (:frame :palmar-carpal-l  :p0 [-0.04 0.0 0.0]    :p1 [0.09 0.0 0.0]   :r0-m 0.036 :r1-m 0.032)
    :hand-carpal-r       (:frame :palmar-carpal-r  :p0 [-0.04 0.0 0.0]    :p1 [0.09 0.0 0.0]   :r0-m 0.036 :r1-m 0.032)
    :knee-patellar-l     (:frame :tibiofemoral-l   :p0 [0.02 0.0 -0.02]   :p1 [0.02 0.0 +0.02] :r0-m 0.060 :r1-m 0.060)
    :knee-patellar-r     (:frame :tibiofemoral-r   :p0 [0.02 0.0 -0.02]   :p1 [0.02 0.0 +0.02] :r0-m 0.060 :r1-m 0.060))

  (:self-collision-pairs
    [(:foot-shell-l :foot-shell-r)
     (:foot-shell-l :shank-crural-r)
     (:foot-shell-r :shank-crural-l)
     (:shank-crural-l :shank-crural-r)
     (:knee-patellar-l :knee-patellar-r)
     (:thigh-femoral-l :thigh-femoral-r)
     (:forearm-ulnar-l :pelvic-girdle)
     (:forearm-ulnar-r :pelvic-girdle)
     (:forearm-ulnar-l :thoracic-ribcage)
     (:forearm-ulnar-r :thoracic-ribcage)
     (:hand-carpal-l :hand-carpal-r)
     (:hand-carpal-l :thigh-femoral-l)
     (:hand-carpal-r :thigh-femoral-r)]))

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
    :manifold-weights (:plantar-bilateral            1.00
                       :metatarsal-equinus-bilateral 0.00
                       :posterior-femoral-bilateral  0.00
                       :ischial-bilateral            0.00
                       :trochanter-gluteal-lateral   0.00
                       :dorsal-supine-continuum      0.00
                       :ulnar-carpal-shove-bilateral 0.00
                       :palmar-carpal-bilateral      0.00)
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
