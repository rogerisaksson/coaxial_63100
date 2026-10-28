;; =============================================================================
;; FILE: anti_spasm_supervisor.lisp
;; DOMAIN: Passivity Enforcement, Singularity Soft-Clipping & Chatter Suppression
;; =============================================================================
(defpackage :ortho-dsp.supervisor
  (:use :ortho-dsp.core)
  (:export #:suppress-derivative-chatter
           #:regularize-kinematic-singularities
           #:latch-hysteretic-contact
           #:bridge-backlash-zero-crossing
           #:enforce-passivity-energy-tank
           #:master-anti-spasm-insert))
(in-package :ortho-dsp.supervisor)

;; -----------------------------------------------------------------------------
;; 1. TORQUE DE-ESSER & ENCODER NOISE GATE (DERIVATIVE KICK SUPPRESSION)
;; Prevents high Kd gains from amplifying encoder quantization steps into motor
;; whine or high-frequency mechanical tremor.
;; -----------------------------------------------------------------------------
(defmodifier suppress-derivative-chatter [state]
  (-> state
      ;; Noise gate on velocity estimation: ignore sub-LSB encoder hunting
      (apply-velocity-noise-gate
        :target-nodes :all-actuated-joints
        :deadband-radps 0.0035
        :gate-open-attack-ms 4.0
        :gate-close-release-ms 25.0
        :curve :smooth-step)

      ;; Biquad low-pass on the derivative (D) term before torque summation
      (filter-derivative-channel
        :topology :butterworth-2nd-order
        :cutoff-hz 42.0                   ; Strip structural frame resonances > 42 Hz
        :zero-phase-compensation false)   ; Prioritize causal phase margin over group delay

      ;; Hard slew-rate & jerk limiter on output torque command (dTau/dt, d2Tau/dt2)
      (limit-torque-transients
        :max-torque-slew-nmps :drive-current-loop ; each drive's peak x 2 pi x its current loop's bandwidth:
                                          ; 850 N m/s felled her walk, her law asks hips 10 kN m/s at p99
        :max-jerk-radps3 :drive-current-loop ; each drive's slew over its rotor's inertia: 12000
                                          ; ramped her law's 3 ms asks out - on stilts, 465 J/m
        :clip-topology :cubic-soft-knee)))

;; -----------------------------------------------------------------------------
;; 2. SINGULARITY SOFT-CLIPPER (DAMPED LEAST-SQUARES JACOBIAN REGULARIZATION)
;; Prevents infinite joint-velocity commands when knees or elbows approach
;; straight-leg geometric lock (Manipulability index w -> 0).
;; -----------------------------------------------------------------------------
(defmodifier regularize-kinematic-singularities [state]
  (-> state
      (configure-jacobian-solver
        :method :damped-least-squares     ; J^* = J^T * (J * J^T + lambda^2 * I)^-1
        :min-singular-value-sigma 0.045
        :max-damping-lambda 0.18
        :activation-curve :raised-cosine)

      ;; Soft-knee anatomical buffer prior to mechanical hyperextension hard-stops
      (enforce-joint-boundary-buffer
        :nodes [:tibiofemoral :glenohumeral :talocrural :c1-c2-atlas]
        :extension-soft-knee-rad 0.06     ; ~3.4 deg viscoelastic cushion before 0.0 rad lock
        :boundary-repulsion-stiffness 140.0
        :boundary-damping-ratio 1.00)))   ; Critically damped: zero rebound off the limit

;; -----------------------------------------------------------------------------
;; 3. HYSTERETIC CONTACT SCHMITT-TRIGGER & BUMPLESS STATE TRANSFER
;; Eliminates violent stance/swing chattering when normal force hovers near the
;; threshold, and forces C^2 spline continuity across all regime switches.
;; -----------------------------------------------------------------------------
(defmodifier latch-hysteretic-contact [state]
  (-> state
      ;; Dual-threshold Schmitt trigger on all load-bearing manifolds
      (configure-contact-comparators
        :manifolds [:plantar-bilateral :ischial-bilateral :dorsal-supine-array]
        :schmitt-high-engage-fz 0.18      ; Must exceed 18% mg to latch contact ON
        :schmitt-low-release-fz 0.06      ; Must drop below 6% mg to unlatch contact OFF
        :min-state-dwell-ms 45.0          ; Debounce window: forbid state toggling < 45 ms
        :impact-blanking-window-ms 18.0)  ; Ignore piezoelectric ringing during initial strike

      ;; Bumpless transfer: sample-and-hold actual encoder/torque state on mode switch
      ;; and decay the residual error to zero so desired trajectory never step-jumps
      (enforce-bumpless-handover
        :continuity-order :c2-acceleration-continuous
        :state-mismatch-decay-ms 85.0
        :decay-envelope :exponential-critically-damped)))

;; -----------------------------------------------------------------------------
;; 4. TRANSMISSION BACKLASH & ZERO-CROSSING ANTI-HUNTING BRIDGE
;; Prevents limit-cycle oscillation inside drivetrain/tendon deadzones when
;; commanded joint torque crosses 0.0 Nm.
;; -----------------------------------------------------------------------------
(defmodifier bridge-backlash-zero-crossing [state]
  (-> state
      (apply-zero-crossing-governor
        :nodes [:coxofemoral :tibiofemoral :talocrural :l4-l5-lumbar]
        :torque-deadzone-window-nm 0.85
        :crossing-slew-attenuation 0.35   ; Slow down torque reversal across the gear/tendon gap
        :co-contraction-bias-nm :none)))  ; one reversible drive a joint, both ways: no antagonist to
                                          ; pull against; 1.2 N m in the command's sign sat her on one
                                          ; sole and she fell rising from the chair

;; -----------------------------------------------------------------------------
;; 5. PORT-HAMILTONIAN PASSIVITY OBSERVER / CONTROLLER (ENERGY-TANK LIMITER)
;; Monitors net mechanical power flow (P = dq/dt * Tau). If phase lag causes an
;; actuator to inject net energy into an oscillation, a virtual series damper
;; automatically bleeds the excess energy to ground.
;; -----------------------------------------------------------------------------
(defmodifier enforce-passivity-energy-tank [state]
  (-> state
      (monitor-energy-tank
        :tank-capacity-joules 12.0
        :initial-energy-joules 4.0
        :lower-depletion-bound-joules 0.50

        ;; Detect parasitic oscillation (bandpass 6.0 Hz -> 35.0 Hz zero-crossing counter)
        :limit-cycle-detector
          (:bandpass-window-hz [6.0 -> 35.0]
           :max-consecutive-reversals 3
           :trigger-power-threshold-watts 4.5)

        ;; Passivity Controller (PC): inject adaptive virtual damping when tank depletes
        :on-passivity-violation
          (engage-virtual-dissipative-load
            :damping-injection-gain 3.2
            :stiffness-rollback-db -9.0   ; Temporarily soften P-gain to restore phase margin
            :recovery-release-ms 450.0))))

;; -----------------------------------------------------------------------------
;; 6. MASTER ANTI-SPASM BUS INSERT (DROP-IN GLOBAL STABILIZER)
;; -----------------------------------------------------------------------------
(defmodifier master-anti-spasm-insert [gait-or-maneuver-stream]
  (-> gait-or-maneuver-stream
      (latch-hysteretic-contact)
      (regularize-kinematic-singularities)
      (bridge-backlash-zero-crossing)
      (suppress-derivative-chatter)
      (enforce-passivity-energy-tank)))
