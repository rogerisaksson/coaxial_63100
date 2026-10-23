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

;; -----------------------------------------------------------------------------
;; 1. UNIVERSAL PHYSICAL RESIDUAL GRAMMAR (MOVE-AGNOSTIC COST FUNCTIONALS)
;; Maps generic AST keyword patterns to physical telemetry error channels.
;; Any present or future .lisp file using these primitives is tuned automatically.
;; -----------------------------------------------------------------------------
(define-residual-grammar *universal-primitive-map*
  ;; A. Momentum & CoM Projection Balance (applies to any posture, lift, pivot, or gait)
  (:residual-class :com-wrench-balance
    :telemetry-error :centroidal-zmp-vector-err-mm   ; [err_x, err_y] inside active support manifold
    :match-ast-keys  [:dc-offset-rad :axial-counterweight :appendage-counterweight
                      :sagittal-com-injection-mm :zmp-counter-bias-x-mm :sagittal-stagger-mm]
    :adjoint-sign    -1.0
    :hard-invariant  :min-zmp-margin-mm)

  ;; B. Normal-Force & Gravitational Unloading (applies to any drop, jump, or envelope)
  (:residual-class :normal-force-floor
    :telemetry-error :manifold-min-fz-violation      ; max(0, min_fz_ratio - measured_fz)
    :match-ast-keys  [:attack-ms :descent-duration-ms :duration-ms
                      :modulation-depth-mm :duck-depth-mm :stance-nadir-mm]
    :adjoint-sign    +1.0                            ; Lengthen duration / reduce depth on Fz drop
    :hard-invariant  :min-normal-fz-ratio)

  ;; C. Tangential Shear & Friction Cone (applies to any cut, pivot, brake, or slip)
  (:residual-class :tangential-shear-slip
    :telemetry-error :integrated-micro-slip-mm       ; Measured tangential foot/contact displacement
    :match-ast-keys  [:stride-length-mm :linear-to-angular-coupling :propulsive-shear
                      :toe-off-shear :stance-duty-pwm :coronal-track-mm]
    :adjoint-sign    -1.0)

  ;; D. Self-Collision & Environmental SDF Clearance (applies to any limb sweep or cross-step)
  (:residual-class :spatial-sdf-clearance
    :telemetry-error :min-signed-distance-err-mm     ; Violation of safety hull between links/obstacles
    :match-ast-keys  [:swing-circumduction-mm :swing-clearance-z-mm :edge-margin-z-mm
                      :coronal-cross-offset-mm :abduction-rad]
    :adjoint-sign    +1.0
    :hard-invariant  :min-coronal-clearance-mm)

  ;; E. Contact Impact & Substrate Resonance (applies to any strike, landing, or handover)
  (:residual-class :contact-transient-ringdown
    :telemetry-error :contact-impact-and-psd-peak    ; [impact_jerk, substrate_eigenfreq_hz]
    :match-ast-keys  [:apex-deceleration-height-mm :eccentric-damping :compliance
                      :rebound-q :biquad-notch-hz :transfer-time-ms]
    :adjoint-sign    :spectral-match)

  ;; F. Passivity & Cost of Transport (applies to all active actuators across any regime)
  (:residual-class :actuator-thermal-passivity
    :telemetry-error :energy-tank-bleed-joules       ; Dissipated virtual damper energy from supervisor
    :match-ast-keys  [:q-factor :cutoff-hz :stiffness-atten-db :damping-ratio]
    :adjoint-sign    -1.0))

;; -----------------------------------------------------------------------------
;; 2. DYNAMIC AST WALKER (DISCOVERS TUNABLE S-EXPRESSION NODES AT RUNTIME)
;; Traverses the active schedule and builds a flat pointer table to every numeric
;; literal inside any loaded package (00..N) that matches the residual grammar.
;; -----------------------------------------------------------------------------
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

;; -----------------------------------------------------------------------------
;; 3. EMPIRICAL SENSITIVITY & ADJOINT DELTA SOLVER
;; Computes parameter updates d_theta = -alpha * (J^T * J + lambda * I)^-1 * J^T * R
;; over the last execution window (cyclic bar or episodic one-shot maneuver).
;; -----------------------------------------------------------------------------
(defprimitive compute-adjoint-ast-deltas [binding-table telemetry-window]
  (let [residual-vec  (extract-residual-vector telemetry-window *universal-primitive-map*)
        sensitivity-j (estimate-empirical-jacobian binding-table telemetry-window)]

    ;; Damped Least-Squares Gauss-Newton step in AST parameter space
    (solve-dls-parameter-step
      :jacobian sensitivity-j
      :residuals residual-vec
      :damping-lambda 0.08
      :trust-region-scale 0.15               ; Max 15% parameter shift per iteration
      :enforce-invariants physical-invariants)))

;; -----------------------------------------------------------------------------
;; 4. HOMOICONIC AST REWRITER (MUTATES ANY LOADED S-EXPRESSION IN PLACE)
;; -----------------------------------------------------------------------------
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
            ;; Splice new numeric literal directly into the S-expression node
            (setf-sexp-at-path mutated-ast
                               (binding-package binding)
                               (binding-symbol binding)
                               (binding-sexp-path binding)
                               new-val)))))
    mutated-ast))

;; -----------------------------------------------------------------------------
;; 5. PHASE-SAFE RCU SHADOW COMPILATION & SOURCE-FILE SERIALIZATION
;; -----------------------------------------------------------------------------
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
        ;; Swap either at cyclic zero-crossing OR when static manifold is in equilibrium
        :sync-barrier (:or :double-support-zero-crossing :manifold-equilibrium-dwell)
        :handover-filter (enforce-bumpless-handover
                           :continuity-order :c2-acceleration-continuous
                           :state-mismatch-decay-ms 85.0)))))

;; Writes mutated AST definitions back to whichever .lisp file originally defined them
(defprimitive sync-mutated-packages-to-disk [mutated-ast binding-table]
  (dolist (dirty-pkg (collect-modified-packages binding-table))
    (let [source-filepath (package-source-file dirty-pkg)] ; Resolved via reader metadata
      (serialize-package-sexp-atomic
        :package dirty-pkg
        :ast mutated-ast
        :filepath source-filepath
        :preserve-comments true
        :float-precision 4))))

;; -----------------------------------------------------------------------------
;; 6. UNIVERSAL BACKGROUND AUTOTUNE DAEMON
;; -----------------------------------------------------------------------------
(defdaemon universal-autotune-daemon [root-schedule-symbol]
  :execution-domain :background-non-rt-core
  :telemetry-tap    :lock-free-spsc-ringbuffer
  :init
    (let [binding-table (introspect-active-ast root-schedule-symbol *universal-primitive-map*)]
      (loop-on-window-completion [telemetry-window active-ast] ; Fires per bar OR per oneshot
        (let [deltas   (compute-adjoint-ast-deltas binding-table telemetry-window)
              next-ast (apply-homoiconic-rewrite active-ast binding-table deltas)]
          (atomic-rcu-soa-swap next-ast *master-dma-ring*)
          (when (residual-norm-converged-p telemetry-window :tol 1e-3)
            (sync-mutated-packages-to-disk next-ast binding-table))))))
