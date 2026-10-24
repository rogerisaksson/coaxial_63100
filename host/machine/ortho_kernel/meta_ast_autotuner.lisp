;; =============================================================================
;; FILE: meta_ast_autotuner.lisp
;; DOMAIN: Move-Agnostic Homoiconic AST Introspection & Adjoint Residual Tuning
;; =============================================================================
(defpackage :ortho-dsp.autotuner
  (:use :ortho-dsp.core
        :ortho-dsp.supervisor)
  (:export #:define-residual-grammar
           #:introspect-active-ast
           #:compute-adjoint-ast-deltas
           #:apply-homoiconic-rewrite
           #:atomic-rcu-soa-swap
           #:sync-mutated-packages-to-disk
           #:universal-autotune-daemon))
(in-package :ortho-dsp.autotuner)

(define-residual-grammar *universal-primitive-map*
  (:residual-class :com-wrench-balance
    :telemetry-error :centroidal-wrench-margin-err-nm
    :match-ast-keys  [:dc-offset-rad :axial-counterweight :appendage-counterweight
                      :sagittal-com-injection-mm :zmp-counter-bias-x-mm :sagittal-stagger-mm]
    :adjoint-sign    -1.0
    :hard-invariant  :min-wrench-margin-nm)

  (:residual-class :normal-force-floor
    :telemetry-error :manifold-min-fz-violation
    :match-ast-keys  [:attack-ms :descent-duration-ms :duration-ms
                      :modulation-depth-mm :duck-depth-mm :stance-compression-mm]
    :adjoint-sign    +1.0
    :hard-invariant  :min-normal-fz-ratio)

  (:residual-class :tangential-shear-slip
    :telemetry-error :integrated-micro-slip-mm
    :match-ast-keys  [:stride-length-mm :linear-to-angular-coupling :propulsive-shear
                      :toe-off-shear :stance-duty-pwm :coronal-track-mm :baseline-preload-n]
    :adjoint-sign    -1.0)

  (:residual-class :spatial-sdf-clearance
    :telemetry-error :min-signed-distance-err-mm
    :match-ast-keys  [:swing-circumduction-mm :swing-clearance-z-mm :edge-margin-z-mm
                      :coronal-cross-offset-mm :abduction-rad :headroom-margin-mm]
    :adjoint-sign    +1.0
    :hard-invariant  :min-coronal-clearance-mm)

  (:residual-class :contact-transient-ringdown
    :telemetry-error :contact-impact-and-psd-peak
    :match-ast-keys  [:virtual-damping-nsm :eccentric-damping :compliance
                      :rebound-q :biquad-notch-hz :transfer-ms]
    :adjoint-sign    :spectral-match)

  (:residual-class :actuator-thermal-passivity
    :telemetry-error :energy-tank-and-winding-temp-err
    :match-ast-keys  [:q-factor :cutoff-hz :stiffness-atten-db :damping-ratio]
    :adjoint-sign    -1.0))

(defprimitive introspect-active-ast [root-schedule-symbol grammar]
  (let [active-packages (collect-referenced-packages root-schedule-symbol)
        binding-table   (make-lock-free-binding-table :max-nodes 512)]
    (dolist (pkg active-packages)
      (dolist (sym (exported-symbols pkg))
        (when (macro-or-modifier-ast-p sym)
          (walk-sexp-tree (symbol-ast-definition sym)
            (lambda (subform path-indices parent-op)
              (when (and (keywordp (car subform))
                         (numberp (cadr subform))
                         (grammar-matches-key-p grammar (car subform)))
                (register-ast-pointer binding-table
                  :package pkg
                  :symbol sym
                  :sexp-path path-indices
                  :keyword (car subform)
                  :current-val (cadr subform)
                  :residual-class (lookup-residual-class grammar (car subform)))))))))
    binding-table))

(defprimitive compute-adjoint-ast-deltas [binding-table telemetry-window]
  (let [residual-vec  (extract-residual-vector telemetry-window *universal-primitive-map*)
        sensitivity-j (estimate-empirical-jacobian binding-table telemetry-window)]
    (solve-dls-parameter-step
      :jacobian sensitivity-j
      :residuals residual-vec
      :damping-lambda 0.08
      :trust-region-scale 0.15
      :enforce-invariants physical-invariants)))

(defprimitive apply-homoiconic-rewrite [shadow-ast binding-table delta-vector]
  (let [mutated-ast (copy-sexp-tree shadow-ast)]
    (dotimes (i (binding-count binding-table))
      (let [binding   (get-binding binding-table i)
            raw-delta (aref delta-vector i)]
        (when (> (abs raw-delta) (binding-deadband binding))
          (let [new-val (clamp-to-physical-invariants
                          (+ (binding-current-val binding) raw-delta)
                          (binding-keyword binding)
                          physical-invariants)]
            (setf-sexp-at-path mutated-ast
                               (binding-package binding)
                               (binding-symbol binding)
                               (binding-sexp-path binding)
                               new-val)))))
    mutated-ast))

(defprimitive atomic-rcu-soa-swap [mutated-ast active-dma-ring]
  (let [shadow-soa (compile-to-soa
                     :ast mutated-ast
                     :align-bytes 64
                     :sample-rate-hz 1000
                     :target-bank :inactive-shadow-bank)]
    (when (verify-static-invariants shadow-soa physical-invariants)
      (arm-rcu-pointer-swap
        :dma-ring active-dma-ring
        :next-buffer shadow-soa
        :sync-barrier (:or :double-support-zero-crossing :manifold-equilibrium-dwell)
        :handover-filter (enforce-bumpless-handover
                           :continuity-order :c2-acceleration-continuous
                           :state-mismatch-decay-ms 85.0)))))

(defprimitive sync-mutated-packages-to-disk [mutated-ast binding-table]
  (dolist (dirty-pkg (collect-modified-packages binding-table))
    (let [source-filepath (package-source-file dirty-pkg)]
      (serialize-package-sexp-atomic
        :package dirty-pkg
        :ast mutated-ast
        :filepath source-filepath
        :preserve-comments true
        :float-precision 4))))

(defdaemon universal-autotune-daemon [root-schedule-symbol]
  :execution-domain :background-non-rt-core
  :telemetry-tap    :lock-free-spsc-ringbuffer
  :init
    (let [binding-table (introspect-active-ast root-schedule-symbol *universal-primitive-map*)]
      (loop-on-window-completion [telemetry-window active-ast]
        (let [deltas   (compute-adjoint-ast-deltas binding-table telemetry-window)
              next-ast (apply-homoiconic-rewrite active-ast binding-table deltas)]
          (atomic-rcu-soa-swap next-ast *master-dma-ring*)
          (when (residual-norm-converged-p telemetry-window :tol 1e-3)
            (sync-mutated-packages-to-disk next-ast binding-table))))))
