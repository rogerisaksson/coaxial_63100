;; -----------------------------------------------------------------------------
;; MODULE 2.8: meta_ast_autotuner
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.autotuner
  (:use :ortho-dsp.core
        :ortho-dsp.lowering
        :ortho-dsp.supervisor)
  (:export #:define-residual-grammar
           #:universal-autotune-daemon))
(in-package :ortho-dsp.autotuner)

(define-residual-grammar *universal-primitive-map*
  (:residual-class :com-wrench-balance
    :telemetry-error :centroidal-wrench-margin-err-nm
    :match-ast-keys  [:dc-offset-rad :axial-counterweight :sagittal-com-injection-mm
                      :zmp-counter-bias-x-mm :sagittal-stagger-mm]
    :adjoint-sign    -1.0 :hard-invariant :min-wrench-margin-nm)
  (:residual-class :normal-force-floor
    :telemetry-error :manifold-min-fz-violation
    :match-ast-keys  [:attack-ms :duration-ms :duck-depth-mm :stance-compression-mm]
    :adjoint-sign    +1.0 :hard-invariant :min-normal-fz-ratio)
  (:residual-class :tangential-shear-slip
    :telemetry-error :integrated-micro-slip-mm
    :match-ast-keys  [:stride-length-mm :linear-to-angular-coupling :toe-off-shear
                      :stance-duty-pwm :slide-speed-mps]
    :adjoint-sign    -1.0)
  (:residual-class :spatial-sdf-clearance
    :telemetry-error :min-capsule-lse-distance-err-m
    :match-ast-keys  [:swing-circumduction-mm :swing-clearance-z-mm :edge-margin-z-mm
                      :min-shin-clearance-mm :ecbf-pullback-stiffness-eta :headroom-margin-mm]
    :adjoint-sign    +1.0 :hard-invariant :min-coronal-clearance-mm)
  (:residual-class :contact-transient-ringdown
    :telemetry-error :contact-impact-and-psd-peak
    :match-ast-keys  [:virtual-damping-nsm :compliance :rebound-q :biquad-notch-hz :transfer-ms]
    :adjoint-sign    :spectral-match)
  (:residual-class :actuator-thermal-passivity
    :telemetry-error :energy-tank-and-phase-margin-err
    :match-ast-keys  [:q-factor :cutoff-hz :stiffness-atten-db :damping-ratio :barrier-gamma]
    :adjoint-sign    -1.0 :hard-invariant :min-phase-margin-deg))

(defdaemon universal-autotune-daemon [root-schedule-symbol]
  :execution-domain :background-non-rt-core
  :telemetry-tap    :lock-free-spsc-ringbuffer
  :init
    (let [binding-table (introspect-active-ast root-schedule-symbol *universal-primitive-map*)]
      (loop-on-window-completion [telemetry-window active-ast]
        (let [deltas   (compute-adjoint-ast-deltas binding-table telemetry-window)
              next-ast (apply-homoiconic-rewrite active-ast binding-table deltas)
              next-soa (compile-to-soa :ast next-ast :target-bank :inactive-shadow-bank
                                       :live-telemetry telemetry-window)]
          (when (verify-static-invariants next-soa physical-invariants)
            (arm-rcu-pointer-swap :dma-ring *master-dma-ring* :next-buffer next-soa
                                  :sync-barrier (:or :double-support-zero-crossing :manifold-equilibrium-dwell)))
          (when (residual-norm-converged-p telemetry-window :tol 1e-3)
            (sync-mutated-packages-to-disk next-ast binding-table))))))
