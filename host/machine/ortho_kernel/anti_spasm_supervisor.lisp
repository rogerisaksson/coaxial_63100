;; -----------------------------------------------------------------------------
;; MODULE 1.17: anti_spasm_supervisor
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.supervisor
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.autonomics
        :ortho-dsp.power-thermal)
  (:export #:suppress-derivative-chatter
           #:regularize-kinematic-singularities
           #:latch-hysteretic-contact
           #:bridge-backlash-zero-crossing
           #:enforce-passivity-energy-tank
           #:monitor-discrete-phase-margin
           #:master-anti-spasm-insert))
(in-package :ortho-dsp.supervisor)

(defmodifier suppress-derivative-chatter [state]
  (-> state
      (apply-velocity-noise-gate
        :target-nodes :all-actuated-joints :deadband-radps 0.0035
        :gate-open-attack-ms 4.0 :gate-close-release-ms 25.0 :curve :smooth-step)
      (filter-derivative-channel :topology :butterworth-2nd-order :cutoff-hz 42.0 :zero-phase-compensation false)
      (limit-torque-transients   :max-torque-slew-nmps 850.0 :max-jerk-radps3 12000.0 :clip-topology :cubic-soft-knee)))

(defmodifier regularize-kinematic-singularities [state]
  (-> state
      (configure-jacobian-solver
        :method :damped-least-squares :min-singular-value-sigma 0.045
        :max-damping-lambda 0.18 :activation-curve :raised-cosine)
      (enforce-joint-boundary-buffer
        :nodes [:tibiofemoral :glenohumeral :cubital :talocrural :c1-c2-atlas]
        :extension-soft-knee-rad 0.06 :boundary-repulsion-stiffness 140.0 :boundary-damping-ratio 1.00)))

(defmodifier latch-hysteretic-contact [state]
  (-> state
      (configure-contact-comparators
        :manifolds [:plantar-bilateral :metatarsal-equinus-bilateral :posterior-femoral-bilateral
                    :ischial-bilateral :trochanter-gluteal-lateral :dorsal-supine-continuum
                    :ulnar-carpal-shove-bilateral :palmar-carpal-bilateral]
        :schmitt-high-engage-fz 0.18 :schmitt-low-release-fz 0.06
        :min-state-dwell-ms 45.0 :impact-blanking-window-ms 18.0)
      (enforce-bumpless-handover
        :continuity-order :c2-acceleration-continuous
        :state-mismatch-decay-ms 85.0 :decay-envelope :exponential-critically-damped)))

(defmodifier bridge-backlash-zero-crossing [state]
  (-> state
      (apply-zero-crossing-governor
        :nodes [:coxofemoral :tibiofemoral :talocrural :l4-l5-lumbar]
        :torque-deadzone-window-nm 0.85 :crossing-slew-attenuation 0.35 :co-contraction-bias-nm 1.20)))

(defmodifier monitor-discrete-phase-margin [state]
  (-> state
      (run-quadrature-goertzel-probe
        :probe-frequencies-hz [15.0 22.0 30.0 38.0]
        :demod-window-ms 250.0
        :min-phase-margin-deg 38.0
        :on-margin-deficit (:auto-rollback-kd-and-zcbf-gamma true
                            :min-kd-scale-floor 0.35
                            :min-gamma-scale-floor 0.50))))

(defmodifier enforce-passivity-energy-tank [state]
  (-> state
      (monitor-energy-tank
        :tank-capacity-joules 12.0 :initial-energy-joules 4.0 :lower-depletion-bound-joules 0.50
        :limit-cycle-detector (:bandpass-window-hz [6.0 -> 35.0] :max-consecutive-reversals 3 :trigger-power-threshold-watts 4.5)
        :on-passivity-violation
          (engage-virtual-dissipative-load :damping-injection-gain 3.2 :stiffness-rollback-db -9.0 :recovery-release-ms 450.0))))

(defmodifier master-anti-spasm-insert [stream]
  (-> stream
      (inject-autonomic-physiology)
      (govern-winding-thermal-metric)
      (allocate-whole-body-torque)
      (shunt-regenerative-bus-overvoltage)
      (latch-hysteretic-contact)
      (regularize-kinematic-singularities)
      (bridge-backlash-zero-crossing)
      (suppress-derivative-chatter)
      (monitor-discrete-phase-margin)
      (enforce-passivity-energy-tank)))
