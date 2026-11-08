;; =============================================================================
;; FILE: closed_chain_qp.lisp
;; DOMAIN: Multi-Point Support Regime - Closed Kinematic Chains on so(3)^n as One Quadratic Program:
;;         the Step that Takes Over when the Pendulum is Gone
;; REQUIRES: core_biomechanics.lisp
;; Hers: `machine.stack.chain` runs it on her law (`machine.stack.chain_qp` the program) and `wbc_chain.c` with
;; `wbc_chain_qp.c` bit for bit; `motion.control` picks it by her release alone - a leg let go or no sole standing,
;; and none asked to land -, never by a text's name; a sole still standing is one more contact of the chain.
;; =============================================================================
(defpackage :ortho-dsp.closed-chain
  (:use :ortho-dsp.core)
  (:export #:closed-chain-regime
           #:contact-frame
           #:closed-chain-unknowns
           #:closed-chain-level-1
           #:closed-chain-level-2
           #:closed-chain-answer))
(in-package :ortho-dsp.closed-chain)

;; With no sole standing and her knees, hands, hips or back bearing her, the inverted pendulum is gone: the physics
;; is a parallel manipulator whose contacts stick (|f_t| < mu f_n, v_t = 0), slide (|f_t| = mu f_n against v_t) or
;; lift (f_n = 0). The upright step's least squares (soles never sliding, every bearing point pinned) is the wrong
;; tool there, and so was a plan that welded every held contact as three equalities, patched broken bounds between
;; rounds with forces fixed, and let a Gauss-Seidel re-solve the same contacts apart: her fists squeezed 570 N into
;; each other, a hand's shear tilted its frame, a floor on their load lifted her seat and slid her 37 cm, a cap on it
;; starved them, and a box on a level seat released world y, the fist's own normal.
;; The regime: one QP in u = (x, zeta) - her driven joints' accelerations x and the contacts' forces zeta in their
;; frames - every other quantity affine in u through her dynamics: the dofs no drive holds, the contacts'
;; accelerations, her tasks, her torques. Level 1 answers her tasks with the anchors held still and the forces least,
;; under the laws no answer may break: a contact presses alone (z_n >= 0) and never sinks faster than its support
;; gives, friction within its pyramid, her torques within their clamps. Level 2 lays her posture in level 1's damped
;; null space under the same laws, its arm columns projected off her held hands. Her torques are those forces'.
(defconstants closed-chain-regime
  :dls-damping-m                0.02   ; lambda: level 2's damping, m, and the hands' projection's (a chain near
                                       ; singular still answers)
  :task-damping                 0.3    ; level 1's damping on x: a turn through a weak lever asks less of her drives
                                       ; (at 0.5 and 1.0 her floor sit fell over at 7.1 s)
  :contact-softness-m-s2-per-n  27.9   ; a contact's give over a pass, 1 / (k h^2): the viscoelastic mattress's k, her
                                       ; 31 kg at its 1.8 Hz (3 977 N/m), h her law's 3 ms pass - the force an
                                       ; acceleration asks of the foam comes a pass late (the compliant variant's);
                                       ; sensed, each contact's give is the solver's impedance R on its normal rows
                                       ; (edge.GIVE, a_n = a_ref - R f_n): a chair's small, never none - at none the
                                       ; plan split her seat's load as it pleased at a_n = 0. Her drives'
                                       ; wind-up is no give of a contact's: a hand's push on a seat rises as the rotor
                                       ; winds the box up while the hand stays still (counted as give it tied her hands
                                       ; to what they felt: 3 of 6 met fell to 1)
  :stick-bleed-per-s            20.0   ; a sticking contact's slip bled, 1/s (the upright step's CONTACT_K)
  :friction-mu                  0.6    ; her sled's friction on what she sits or lies on (the seat's material)
  :anchor-mu                    1.0    ; a hand's, a let-go foot's or a sole's friction on what bears it (her skin's
                                       ; material; the kernel's hands on a seat read 49 of 50 N across at it)
  :hold-n                       30.0   ; a patch bearing this much, its points together, is held still (its stick
                                       ; rows); under it a contact only presses alone - her fingers' arrival at 2-4 N,
                                       ; held, fought her arms' pose and threw her elbow 449 deg/s
  :stick-weight                 10.0   ; a held contact's three stick rows, against the tasks' 1: the hands hold and
                                       ; the sled moves by pushing against them, not by the hands sliding in the plan
  :force-reg                    0.02   ; the contacts' shear regularised toward none, rad/s^2 a N (the plan's force
                                       ; rows' scale): the least shear - at 0.001 the plan used every anchor's friction
                                       ; to its edge, forearms at 116 of 116 N - never toward last pass's felt shear,
                                       ; which fed a squeeze back on itself
  :normal-reg                   0.02   ; the normals' regularisation toward what is felt: a force is a state, changing
                                       ; it is what costs (toward none a touching hand was unloaded and its give lifted
                                       ; it off the seat: a scoot with no hands); at 0.002 the plan put forces where it
                                       ; pleased and the bench fell from 3 of 6 met to 1
  :turn-weight                  1.0    ; the tasks' weights: her pelvis's and trunk's turns and a
  :swing-weight                 1.0    ; sole's swing, weighed against each other alone
  :posture-weight               1.0    ; her posture's rows, level 2
  :turn-kp                      4.0    ; its own gains, rad/s^2 a rad and a rad/s, near critical: lying down is slow;
  :turn-kd                      4.0    ; on the gait's (100, 20) a hip's yaw told 0.44 rad asked 44 rad/s^2
  :posture-kp                   14.0   ; (at 16 and 20 her hands' arrival at the second lift threw her elbow 374-428
  :posture-kd                   7.5    ; deg/s; at 12 her floor sit's trunk sat 10.7 deg off)
  :twist-weight                 1.0    ; the twist told her (WASD, or her egress): her pelvis along its level way at
  :twist-kd                     4.0    ; its speed, 1/s - the sled's se(2) vector
  :slide-m-s                    0.02   ; her sled slides past this speed along the way told: kinetic, its friction at
                                       ; the cone's edge against its slip, free to slow; under it static, its shear
                                       ; within the pyramid, dissipative, drawn to the edge by the pin's soft row and
                                       ; never planned to slide against the way (at rest she creeps 6 mm/s)
  :pin-weight                   2.0    ; the static sled's soft pin, z_t . s_t = -mu z_n over her mass, m/s^2, above the
                                       ; twist's 1: at rest the hands push and the seat unloads until its friction
                                       ; reaches the cone and she goes; a soft pin without al_t . s_t >= 0 pulls the
                                       ; shear toward the edge anyway, z = (m a* - F - w mu N) / (1 + w) - a backward
                                       ; motor again, as the hard pin was: the plan had her at -3.7 m/s^2 standing still
  :slide-tilt                   0.2)   ; a slide's normal along the way under this; past it the contact braces (her
                                       ; sacrum on the seat's rear at 0.42 pinned braked her at mu 0.11 and pitched her)

;; Each contact's frame Q (3 x 3, rows): the normal of what bears it - her skin's capsule nearest the touch, its
;; outward radial there reversed (borne._normal), up where she has none -, then two tangents from a fixed world axis.
(defun contact-frame (n q at)
  "q (9k) from at: n (3) unit, t1 = n x e (e = z where |n_x| > 0.9, else x), t2 = n x t1."
  (declare (type (simple-array double-float (*)) n q) (type fixnum at))
  (let* ((e (if (> (abs (aref n 0)) 0.9d0) #(0.0d0 0.0d0 1.0d0) #(1.0d0 0.0d0 0.0d0)))
         (t1 (unit (cross n e))) (t2 (cross n t1)))
    (replace q n :start1 (* 9 at)) (replace q t1 :start1 (+ (* 9 at) 3)) (replace q t2 :start1 (+ (* 9 at) 6))))

;; The unknowns: x (nd) and zeta, each contact's force in its frame - three (normal, two tangents), or one for her
;; sled told a twist on a level slide (its normal up within 45 deg, along the way under slide-tilt) sliding past
;; slide-m-s along it: kinetic, it brakes at its friction's edge against its slip, z = z_n (n - mu_s sign(v . s_t) s_t),
;; s_t the way's unit tangent in its frame, and may slow; at rest such a sled is static - three unknowns, its shear
;; within the pyramid under the laws below; a back against a tilted rest braces - no slide rows, its tangents free
;; within the pyramid. Roles by
;; what she is, never by a text's name: a hand's or a let-go foot's patch (carpal on) and a standing sole are anchors
;; at anchor-mu; what she sits or lies on (a body of hers, a patch under carpal) is her sled at friction-mu. A touching
;; contact's normal is held unless let go or on a foot told free (her heels stepping in turn); an anchor's tangents
;; stick where its patch bears hold-n, its points together.
(defun closed-chain-unknowns (contacts slide regime)
  "(nz, span S (3k x nz), roles): zeta's width, the map zeta -> z in frames, each contact's role."
  (declare (ignore contacts slide regime))
  (values 0 nil nil))

;; Level 1 over u = (x, zeta). Affine maps through her dynamics on the dofs no drive holds P, M_PP qdd_P + M_PG qg
;; + h_P = J_P^T z, qg = (x, qdd_H) with the held joints' known: qdd_P = p0 + Px x + Pz zeta; the contacts'
;; accelerations in their frames al = a0 + Ax x + Az zeta; her tasks R qdd = r as Rx x + Rz zeta = rr; her torques
;; tau = t0 + Tx x + Tz zeta.
;; Least squares (each row weighed): the tasks' rows (turn-weight, swing-weight, twist-weight); a held contact's
;; stick rows at stick-weight - its normal's al_n + give (z_n - f_n) = -bleed v_n (its support gives as the force
;; grows past what is felt); an anchor's tangents' al_t = -bleed v_t (its slip bled); her sled's tangents are its
;; friction's within the pyramid: no tangential lock (held still on a mattress her thighs locked her hip and knee
;; against a leg swing); a static sled's soft pin at pin-weight over her mass, z_t . s_t = -mu_s z_n: drawn to the
;; cone's edge against the way, the hands push and the seat unloads until it goes -; the forces regularised, each
;; contact's tangents toward none at force-reg and its normal toward what is felt at normal-reg; x damped at
;; task-damping (Levenberg-Marquardt).
;; Hard (A u <= b): al_n + give z_n >= -bleed v_n + give f_n every contact (it may leave its support faster than the
;; give says, never sink into it faster); a held contact presses alone, z_n >= 0, with no cap on its growth (felt +
;; hold-n was the upright step's GROW_N and froze a scoot's hands at what they felt); a free contact - its patch under
;; hold-n, its foot told free, or let go - carries at most what it feels, 0 <= z_n <= f_n, free to unload and leave:
;; no lever the world never gave (a sole felt at 10 N was planned at 262 N while leaving the floor at 8.7 m/s^2); a
;; static sled's shear along the way dissipative alone, z_t . s_t <= 0 (the seat never helps), and never planned to
;; slide against the way, al_t . s_t >= 0: with the pin's soft row that is static friction - z_t . s_t = -F while
;; the hands' F is under the cone, a = 0, no backward motor (the hard pin had her at -3.7 m/s^2 standing still) and
;; no reaction wheel (the plan swung her free arms at -7400 deg/s^2 for the pelvis's acceleration and her trunk hit
;; the chair's back at 902 N) -; an eight-sided pyramid inscribed in its friction's
;; cone for every contact with three unknowns, d_k . z_t <= mu cos(pi/8) z_n, d_k at k pi/4 (k = 0..7); her torques
;; within [lo, hi] (her clamps as derated, her back-EMF's); her joints' stops where asked (chain.stop_rows).
;; Solved by the dual active set warm from the last pass's held rows (machine.stack.qp.solve); where no u meets the
;; laws, the least squares alone and her torques shrunk as the upright step's (machine.stack.shrunk).
;; Let go (complementarity pass by pass, from the plan's own multiplier - no release a text tells, no cap): a held
;; contact whose row z_n >= 0 the dual held (the plan would pull on it) is let go next pass - free, it carries what it
;; feels and may leave -, and stays let while the plan lifts it (its al_n > 0); its skin frees it as its force goes,
;; and bearing hold-n again while the plan presses on it (al_n <= 0) it is held again. A held contact's soft normal
;; row alone had glued her torso to a chair's back: no answer could leave it. Remembered a pass by its key (body,
;; patch, ordinal), wbc_step_t.chain_let in C.
(defun closed-chain-level-1 (maps rows bounds warm)
  "(u1, held1, met): level 1's answer, its held rows, whether the laws were met."
  (declare (ignore maps rows bounds warm))
  (values nil nil nil))

;; Level 2 in level 1's damped null space N = I - H1^-1 G1 (G1 the least squares' normal matrix, H1 with the damping):
;; u = u1 + N y, minimising posture-weight |P (x - want)| + lambda |y| - P the projector off her held hands' linear
;; Jacobians on her driven joints, I - J^T (J J^T + lambda^2 I)^-1 J: her posture asks nothing of her hands' motion
;; and still holds every joint along the rest (asked of them it squeezed her fists 570 N into each other) - under the
;; same laws shifted, A N y <= b - A u1, warm from the last pass's; none met, the least squares alone.
(defun closed-chain-level-2 (u1 maps posture bounds warm)
  "(u, held2): the answer with her posture laid in."
  (declare (ignore u1 maps posture bounds warm))
  (values nil nil))

;; The answer: x her driven joints' accelerations, z = Q^T S zeta the forces in the world, qdd_P foreseen, tau her
;; drives' torques - those forces', never a Gauss-Seidel's over them again -; the complementarity's residual the
;; largest |min(al_n, z_n)| for the trace; her torques clipped to [lo, hi].
(defun closed-chain-answer (u maps)
  "(x qdd-p z tau residual)."
  (declare (ignore u maps))
  (values nil nil nil nil 0.0d0))
