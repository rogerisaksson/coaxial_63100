;; =============================================================================
;; FILE: working_envelope_scoper.lisp
;; DOMAIN: Operating-Point Horizon Foliation, Composite-Rigid-Body Subtree
;;         Condensation, Dense SoA Compaction & RCU Dynamic Capability Loader
;; REQUIRES: ortho_dsp_somatic_substrate.lisp, ortho_dsp_cognitive_autonomy.lisp,
;;           canonical_posture_bundle.lisp
;; Hers (`machine.senses.observer`, wbc_gmo.c): her momentum observer's working set a law pass - its split over the
;; unknowns that can push (every point's, a hinge's on its stop, a touch found), its touches searched among the pairs
;; staged within the preload distance, restaged once a capsule's end has moved a quarter of it -, the same bits as
;; her whole set. Her figure: 39 dofs, 21 driven, 8 manifolds, 24 points pushing 54 ways, 44 capsules, 737 pairs.
;; =============================================================================
(defpackage :ortho-dsp.envelope-scoper
  (:use :ortho-dsp.core
        :ortho-dsp.aba-dynamics
        :ortho-dsp.body-schema
        :ortho-dsp.balance-core
        :ortho-dsp.blind-fields
        :ortho-dsp.somatic-core
        :ortho-dsp.postures-core
        :ortho-dsp.supervisor
        :ortho-dsp.lowering)
  (:export #:working-envelope-soa-descriptor
           #:evaluate-operating-point-horizon
           #:condense-dormant-kinematic-subtrees
           #:compact-active-soa-working-slices
           #:stage-and-swap-rcu-capability-bank
           #:xform-dynamic-working-envelope-scoper
           ;; Composable Operating-Point Specializers
           #:specialize-scope-flat-biped-cruise
           #:specialize-scope-furniture-docking-preload
           #:specialize-scope-deformable-terrain-on-demand
           #:specialize-scope-recumbent-hibernation
           #:specialize-scope-disturbance-emergency-wakeup))
(in-package :ortho-dsp.envelope-scoper)

;; -----------------------------------------------------------------------------
;; 1. WORKING-ENVELOPE & ACTIVE-SLICE SOA DESCRIPTOR (STATIC A/B RCU ARENA)
;; Pre-allocates the full capacity once at `init()`, but restricts 1 kHz execution
;; to the compacted slice counts [n_act_dofs, m_act_manifolds, p_act_capsules,
;; k_act_kernels] in the active bank.
;; Hers: her observer's arena is wbc_gmo_t, laid once - 113 unknowns, 1024 pairs -, a pass's set its `live` of
;; `lives` and its pairs' `list` of `listed`.
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
;; Hers: her capsules' pairs, staged within `manifold-preload-distance-mm` (`gmo.horizon`) by a sphere about each;
;; no speed bound is needed, a capsule's end moved a quarter of it restages them. Her points all stay: blind,
;; those off her soles join her split where her soles cannot bear her (10 pushes a pass walking), none evicted.
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

;; -----------------------------------------------------------------------------
;; 3. COMPOSITE-RIGID-BODY SUBTREE CONDENSATION (DYNAMIC FREEING OF DOFS)
;; When a kinematic subtree (e.g., arms/wrists during calm walking, or legs
;; during supine rest) is freed from the active set, folds its 6D spatial
;; inertia tensor I_child into its parent body via Adjoint transformation!
;; Keeps 100% exact CoM & Poinsot inertia while shrinking M(q) from N -> N_act!
;; Hers: none folded: her arms swing and her legs stride as she walks, so a frozen subtree's momentum is not hers;
;; her chain's M and h are 23 of her observer's 257 us.
;; -----------------------------------------------------------------------------
(defprimitive condense-dormant-kinematic-subtrees
  [state &key (descriptor 'working-envelope-soa-descriptor)
              (dormant-holding-mode :local-board-pd-latch)] ; :local-board-pd-latch | :passive-high-z
  (-> state
      (fold-unloaded-subtrees-into-parent-inertia
        :envelope-descriptor descriptor
        :spatial-inertia-law :featherstone-crba-adjoint-fold
        ;; Offload freed joints from the host 1 kHz QP directly to the STM32 board's
        ;; local 50 kHz current/position latch (or High-Z if resting in bed):
        :offloaded-joint-behavior dormant-holding-mode
        :deflate-mass-matrix-and-jacobians true)))

;; -----------------------------------------------------------------------------
;; 4. DENSE SOA WORKING-SLICE COMPACTOR (GATHER / SCATTER L1 CACHE PACKING)
;; Packs only the loaded manifolds (M_act), loaded DOFs (N_act), and loaded
;; capsule pairs (P_act) into contiguous SIMD registers so the 1 kHz QP and
;; Khatib projector never touch unallocated/dormant rows.
;; Hers: her observer's split gathered onto its working set (wbc_gmo.c `scoped`): 58 of 113 unknowns walking, its
;; normal matrix and its active set over them in their order; her step (`machine.stack.wrench`) has no QP to pack.
;; -----------------------------------------------------------------------------
(defprimitive compact-active-soa-working-slices
  [state &key (descriptor 'working-envelope-soa-descriptor)]
  (-> state
      (pack-dense-operational-submatrices
        :envelope-descriptor descriptor
        :simd-vector-width-floats 8          ; AVX-256 / AVX-512 cache-line alignment
        :compact-cwc-qp-variables true
        :compact-khatib-nullspace-projector true
        :compact-lse-capsule-distance-kernel true)))

;; -----------------------------------------------------------------------------
;; 5. BUMPLESS RCU SHADOW-BANK LOADER & ZERO-CROSSING SWAPPER
;; Stages newly loaded capabilities into Shadow Bank B, pre-seeds filter and
;; integrator states from the live kinematic state (bumpless transfer), and
;; swaps Bank B -> Bank A atomically at the next PLL phase zero-crossing!
;; Hers: staged within her law's own pass, one thread, so laid as it is swapped, the same bits either side: a
;; capsule's end moved past a quarter of her envelope restages her pairs that pass, no zero-crossing waited for.
;; The double bank (`machine.process.rcu`) is her tuner's.
;; -----------------------------------------------------------------------------
(defprimitive stage-and-swap-rcu-capability-bank
  [state &key (descriptor 'working-envelope-soa-descriptor)
              (crossfade-window-ms 35.0)
              (allow-midcycle-emergency-preempt true)]
  (-> state
      (execute-rcu-working-set-handover
        :envelope-descriptor descriptor
        ;; 1. Warm-start newly loaded joints/manifolds in Shadow Bank B:
        :state-seeding-law
          (:seed-biquad-memory-from-live-q-dq true
           :seed-qp-lagrange-multipliers-from-prior true
           :cosine-taper-window-ms crossfade-window-ms)
        ;; 2. Atomic swap trigger:
        :swap-synchronization
          (:nominal-trigger :pll-phase-zero-crossing
           :emergency-immediate-preempt allow-midcycle-emergency-preempt))))

;; -----------------------------------------------------------------------------
;; 6. MASTER DYNAMIC WORKING-ENVELOPE SCOPER MODIFIER
;; Hers: read for its envelope (`kernel.said`); her working set follows from her state each pass.
;; -----------------------------------------------------------------------------
(defmodifier xform-dynamic-working-envelope-scoper [state]
  (-> state
      (evaluate-operating-point-horizon
        :descriptor 'working-envelope-soa-descriptor
        :lookahead-horizon-ms 250.0)
      (condense-dormant-kinematic-subtrees
        :descriptor 'working-envelope-soa-descriptor
        :dormant-holding-mode :local-board-pd-latch)
      (compact-active-soa-working-slices
        :descriptor 'working-envelope-soa-descriptor)
      (stage-and-swap-rcu-capability-bank
        :descriptor 'working-envelope-soa-descriptor
        :crossfade-window-ms 35.0)))


;; =============================================================================
;; COMPOSABLE OPERATING-POINT SPECIALIZERS (PLUG-IN EXTENSIONS)
;; Hers: none pinned: her working set follows from her state - a seat's points join her split as they did.
;; =============================================================================

;; 1. Flat-Floor Bipedal Cruise Minimal Scope (~60 us 1 kHz Tick Time!)
;; Unloads all non-plantar manifolds (`ischial`, `dorsal`, `ventral`, `trochanter`,
;; `ulnar-carpal`), condenses distal arm/wrist DOFs into the thoracic rigid body,
;; and unloads Hitchin/Rheology kernels until an obstacle or surface change appears.
(defmodifier specialize-scope-flat-biped-cruise [state]
  (-> state
      (xform-dynamic-working-envelope-scoper)
      (pin-minimum-working-set
        :active-manifolds [:plantar-bilateral :metatarsal-equinus-bilateral]
        :condense-subtrees [:distal-carpal-l :distal-carpal-r]
        :evict-kernels [:hitchin-isomonodromic :bekker-snow :duffing-trampoline :bed-pendulum])))

;; 2. Furniture / Bed Proximity Pre-Loader (Lookahead Horizon Staging)
;; As soon as the wayfinder approaches within 1.1 m of an interaction target
;; (chair, high stool, or bed), pre-loads `:ischial`, `:posterior-femoral`,
;; `:ulnar-carpal`, the Hitchin sliding bus, and the Popliteal calf-datum probe
;; into Shadow Bank B so they are 100% warm-started before contact!
(defmodifier specialize-scope-furniture-docking-preload
  [state &key (preload-trigger-radius-m 1.10)]
  (-> state
      (xform-dynamic-working-envelope-scoper)
      (bind-proximity-capability-preloader
        :trigger-distance-to-target-m preload-trigger-radius-m
        :preload-manifolds [:ischial-bilateral
                            :posterior-femoral-bilateral
                            :ulnar-carpal-shove-bilateral]
        :preload-kernels   [:hitchin-isomonodromic
                            :popliteal-standoff-indexer
                            :backless-stool-rim-zcbf]
        :unfreeze-subtrees [:fore-left :fore-right])))

;; 3. On-Demand Deformable Substrate Loader (Zero Cost on Rigid Floors)
;; Keeps heavy Bekker-snow, Winkler-mattress, and Duffing-trampoline kernels
;; UNLOADED on hard floors (`k_sub > 380 N/mm`), and hot-loads ONLY the matching
;; rheological kernel if the lightweight 35ms work-loop detector trips!
(defmodifier specialize-scope-deformable-terrain-on-demand [state]
  (-> state
      (xform-dynamic-working-envelope-scoper)
      (gate-kernel-by-work-loop-hysteresis
        :rigid-floor-stiffness-npmm 380.0
        :evict-all-rheology-above-npmm 380.0
        :hotload-bekker-when-eta-below 0.16
        :hotload-winkler-when-eta-between [0.16 0.62]
        :hotload-duffing-when-eta-above 0.62)))

;; 4. Recumbent / Seated Low-Power Hibernation Scope (Minimal Standby Execution)
;; When resting in bed (`:dorsal-supine`) or seated calmly, unloads the stereo
;; wayfinder, swing-chord guards, and whole-body limb QP; condenses the skeleton
;; down to the 3-IMU Prior Observer +active weight-shift prober (< 18 us/tick!).
(defmodifier specialize-scope-recumbent-hibernation [state]
  (-> state
      (xform-dynamic-working-envelope-scoper)
      (condense-dormant-kinematic-subtrees
        :descriptor 'working-envelope-soa-descriptor
        :dormant-holding-mode :passive-high-z)
      (pin-minimum-working-set
        :active-manifolds [:dorsal-supine-continuum :ischial-bilateral]
        :condense-subtrees [:hind-left :hind-right :fore-left :fore-right]
        :active-kernels [:tri-imu-prior-observer :active-weight-shift-prober])))

;; 5. Instant Disturbance / Slip Emergency Wake-Up Preemptor
;; If the 3-IMU observer or GMO detects an unexpected external push (> 35 N) or
;; foot slip while running in a reduced working scope, immediately preempts the
;; zero-crossing wait and hot-loads all 4 limbs + reactive capture-step kernels
;; in a single 1 kHz tick!
(defmodifier specialize-scope-disturbance-emergency-wakeup [state]
  (-> state
      (bind-emergency-working-set-expander
        :gmo-impulse-threshold-n 35.0
        :imu-pitch-roll-rate-threshold-radps 1.65
        :instant-load-manifolds [:plantar-bilateral
                                 :metatarsal-equinus-bilateral
                                 :ulnar-carpal-shove-bilateral]
        :instant-unfreeze-all-limbs true)))
