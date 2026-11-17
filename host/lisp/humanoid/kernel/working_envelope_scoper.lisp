;; =============================================================================
;; FILE: working_envelope_scoper.lisp
;; DOMAIN: Operating-Point Horizon Foliation, Composite-Rigid-Body Subtree
;;         Condensation, Dense SoA Compaction & RCU Dynamic Capability Loader
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp
;; The robot's own (`machine.senses.observer`, wbc_gmo.c): its momentum observer's working set a law pass -
;; its split over the unknowns that can push (every point's, a hinge's on its stop, a touch found), its touches
;; searched among the pairs staged within the preload distance, restaged once a capsule's end has moved a quarter of
;; it -, the same bits as the robot's whole set. The robot's figure: 39 dofs, 21 driven, 8 manifolds, 24 points
;; pushing 54 ways, 44 capsules, 737 pairs.
;; =============================================================================
(defpackage :ortho-dsp.envelope-scoper
  (:use :ortho-dsp.core)
  (:export #:working-envelope-soa-descriptor
           #:evaluate-operating-point-horizon))
(in-package :ortho-dsp.envelope-scoper)

;; -----------------------------------------------------------------------------
;; 1. WORKING-ENVELOPE & ACTIVE-SLICE SOA DESCRIPTOR (STATIC A/B RCU ARENA)
;; Pre-allocates the full capacity once at `init()`, but restricts 1 kHz execution
;; to the compacted slice counts [n_act_dofs, m_act_manifolds, p_act_capsules,
;; k_act_kernels] in the active bank.
;; The robot's own: its observer's arena is wbc_gmo_t, laid once - 113 unknowns, 1024 pairs -, a pass's
;; set its `live` of `lives` and its pairs' `list` of `listed`.
;; -----------------------------------------------------------------------------
(defstate working-envelope-soa-descriptor
  (:memory-layout
    :max-total-dofs 30
    :max-total-manifolds 10
    :max-total-capsule-pairs 231
    :max-domain-kernels 8
    :rcu-banks [:primary-bank-a :shadow-bank-b]
    :align-bytes 64)
  (:active-slice-counters
    :num-active-dofs          :uint16        ; e.g., 12..14 in pure walk (down from 30!)
    :num-active-manifolds     :uint16        ; e.g., 2 in walk (`plantar`, `equinus`, down from 10!)
    :num-active-capsule-pairs :uint16        ; e.g., 14 reachable pairs (down from 231!)
    :num-active-kernels       :uint16)       ; e.g., 2 kernels in flat walk (down from 8!)
  (:permutation-tables
    :active-dof-indices       :u16-array-30  ; Dense gather/scatter map for M(q) and J_c
    :dormant-dof-indices      :u16-array-30  ; Subtrees folded into parent spatial inertia
    :active-manifold-indices  :u16-array-10  ; Dense map for CWC Wrench-QP
    :active-capsule-pair-ids  :u16-array-231 ; Dense map for 1 kHz LSE/ECBF
    :active-kernel-dispatch   :fn-ptr-array-8)
  (:condensed-inertia-cache
    :parent-composite-spatial-inertias :mat6x6-array-30)) ; I_p^eff = I_p + sum Ad^T * I_child * Ad

;; -----------------------------------------------------------------------------
;; 2. OPERATING-POINT & REACHABLE HORIZON EVALUATOR (SELECTS WORKING SUBSET)
;; Computes the reachable 1-Jet neighborhood E(X_0, dT) around the live operating
;; point X_0 = (q_0, dq_0, w_manifolded, Fr, d_obstacles) and marks capabilities
;; for LOADING (entering horizon) or FREEING/EVICTION (leaving horizon).
;; The robot's own: its capsules' pairs, staged within `manifold-preload-distance-mm` (`gmo.horizon`) by a
;; sphere about each; no speed bound is needed, a capsule's end moved a quarter of it restages them. The robot's
;; points all stay: blind, those off its soles join its split where its soles cannot bear it (10 pushes a pass
;; walking), none evicted. The robot's shield's 13 pairs lie 4-12 within influence every law pass (median 10, sat
;; 12): none to cull. The robot's law's parts gated by their activity (`machine.law.composer`): each at rest its
;; rest, the same bits - the robot's melt off its soles' share, its wayfinder's gauge without an
;; errand, its counterweight with no leg asked released.
;; -----------------------------------------------------------------------------
(defprimitive evaluate-operating-point-horizon
  [state &key (descriptor 'working-envelope-soa-descriptor)
              (lookahead-horizon-ms 250.0)
              (manifold-preload-distance-mm 180.0)
              (dormant-eviction-dwell-ms 400.0)]
  (-> state
      (classify-reachable-capability-subset
        :envelope-descriptor descriptor
        :prediction-horizon-s (* 0.001 lookahead-horizon-ms)
        ;; 1. Manifold Loading/Freeing Criterion:
        ;;    Load manifold `m` ONLY if w_m > 0.005 OR kinematic patch distance to
        ;;    ground/fixture SDF is <= v_patch * dT + `manifold-preload-distance-mm`!
        :manifold-activation-law
          (:preload-margin-m (* 0.001 manifold-preload-distance-mm)
           :evict-after-zero-weight-ms dormant-eviction-dwell-ms)
        ;; 2. Joint Subtree Loading/Freeing Criterion:
        ;;    Free a limb/subtree from whole-body ABA/QP when its rows in B_modal(w)
        ;;    and J_c(w) are zero AND no external GMO torque or obstacle repulsion acts on it!
        :subtree-activation-law
          (:min-modal-sensitivity-norm 0.015
           :min-gmo-external-torque-nm 2.2)
        ;; 3. Capsule-Pair Pruning Criterion:
        ;;    Load pair (a, b) into 1 kHz ECBF only if d_ab(q_0) - (v_a + v_b)*dT < d_influence!
        :capsule-horizon-culling true
        ;; 4. Domain-Kernel Gating Criterion:
        ;;    Load Hitchin/Schlesinger only during sliding multi-contact; load Rheology
        ;;    only when work-loop hysteresis > 0.08; load Wayfinder only when v_cmd > 0!
        :kernel-on-demand-gating true)))
