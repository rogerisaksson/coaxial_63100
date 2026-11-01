;; =============================================================================
;; FILE: closed_chain_complementarity.lisp
;; DOMAIN: Multi-Point Support Regime - Closed Kinematic Chains on so(3)^n with Stick-Slip
;;         Friction Complementarity: the Step that Takes Over when the Pendulum is Gone
;; REQUIRES: core_biomechanics.lisp
;; Hers: `machine.stack.chain` runs it on her law and `wbc_chain.c` bit for bit; `motion.control` picks it by her
;; stance alone - no sole standing and what is not her soles bearing her -, never by a text's name. The solvers
;; below are the ones they run, written out on flat row-major double-float vectors.
;; =============================================================================
(defpackage :ortho-dsp.closed-chain
  (:use :ortho-dsp.core)
  (:export #:closed-chain-regime
           #:cholesky-banachiewicz
           #:cholesky-solve
           #:dls-nullspace-posture
           #:contact-frame
           #:pgs-box-mlcp
           #:closed-chain-step))
(in-package :ortho-dsp.closed-chain)

;; With no sole standing and her knees, hands, hips or back bearing her, the inverted pendulum is gone: the physics
;; is a parallel manipulator - closed kinematic chains, Phi(q) = 0 under her joints' bounds - whose contacts stick
;; (|f_t| < mu f_n, v_t = 0) or slide (|f_t| = mu f_n against v_t). The upright step's convex least squares (soles
;; never sliding, her centre of mass's acceleration error minimised, every bearing point pinned at W_STILL) is the
;; wrong tool there: on a mattress it asked pushes the foam never gave and she spasmed at the foam's 1.9 Hz.
;; The regime: her driven joints follow her posture in the damped null space of the contacts' Jacobian
;; (Levenberg-Marquardt, P_null = I - J^+ J with J^+ = J^T (J J^T + lambda^2 I)^-1, Cholesky); the contacts'
;; forces solve the box-constrained mixed linear complementarity problem over her Delassus matrix on the dofs no
;; drive holds, M_c = J_P M_PP^-1 J_P^T: w = M_c z + b, 0 <= w_n, 0 <= z_n, one of them 0, |z_t|_inf <= mu z_n, by
;; projected Gauss-Seidel warm from what is felt; her torques follow with them, shrunk to her clamps.
(defconstants closed-chain-regime
  :dls-damping-m                0.02   ; lambda: the null space's damping, m - a chain near singular still answers
  :pgs-sweeps                   24     ; Gauss-Seidel sweeps a pass: 36 unknowns at most settle well within
  :contact-softness-m-s2-per-n  27.9   ; a contact's give over a pass, 1 / (k h^2): the viscoelastic mattress's k, her
                                       ; 31 kg at its 1.8 Hz (3 977 N/m), h her law's 3 ms pass - the force an
                                       ; acceleration asks of the foam comes a pass late; a rigid floor's would be 0,
                                       ; so the softest support she lies on sets it and what is felt leads
  :stick-bleed-per-s            20.0   ; a sticking contact's slip bled, 1/s (the upright step's CONTACT_K)
  :friction-mu                  0.6)   ; the box's friction (the upright step's MU)

;; -----------------------------------------------------------------------------
;; 1. CHOLESKY (BANACHIEWICZ, ROW BY ROW) ON A FLAT ROW-MAJOR (M x M) DOUBLE-FLOAT VECTOR
;; -----------------------------------------------------------------------------
(defun cholesky-banachiewicz (g l m)
  "L, lower, with L L^T = G: G and L flat row-major (m x m); NIL where G is not positive."
  (declare (type (simple-array double-float (*)) g l)
           (type fixnum m)
           (optimize (speed 3) (safety 0)))
  (fill l 0.0d0)
  (loop for i of-type fixnum from 0 below m do
    (loop for j of-type fixnum from 0 to i do
      (let ((s (aref g (+ (* i m) j))))
        (declare (type double-float s))
        (loop for k of-type fixnum from 0 below j do
          (setf s (- s (* (aref l (+ (* i m) k)) (aref l (+ (* j m) k))))))
        (if (= i j)
            (progn (when (<= s 0.0d0) (return-from cholesky-banachiewicz nil))
                   (setf (aref l (+ (* i m) i)) (sqrt s)))
            (setf (aref l (+ (* i m) j)) (/ s (aref l (+ (* j m) j))))))))
  l)

(defun cholesky-solve (l r y x m)
  "x with L L^T x = r: forward on L into y, back on L^T into x."
  (declare (type (simple-array double-float (*)) l r y x)
           (type fixnum m)
           (optimize (speed 3) (safety 0)))
  (loop for i of-type fixnum from 0 below m do
    (let ((s (aref r i)))
      (declare (type double-float s))
      (loop for k of-type fixnum from 0 below i do
        (setf s (- s (* (aref l (+ (* i m) k)) (aref y k)))))
      (setf (aref y i) (/ s (aref l (+ (* i m) i))))))
  (loop for i of-type fixnum from (1- m) downto 0 do
    (let ((s (aref y i)))
      (declare (type double-float s))
      (loop for k of-type fixnum from (1+ i) below m do
        (setf s (- s (* (aref l (+ (* k m) i)) (aref x k)))))
      (setf (aref x i) (/ s (aref l (+ (* i m) i))))))
  x)

;; -----------------------------------------------------------------------------
;; 2. LEVENBERG-MARQUARDT NULL-SPACE POSTURE ON HER DRIVEN JOINTS
;; a_des = (I - J^+ J) want, J^+ = J^T (J J^T + lambda^2 I)^-1: the contacts' rows J (rows x nd) hold their points
;; to the chain, the posture's acceleration goes where it moves them nothing.
;; -----------------------------------------------------------------------------
(defun dls-nullspace-posture (j want a-des rows nd damping g l r y x)
  "a-des (nd) of want (nd) in the damped null space of J (rows x nd), G, L (rows x rows), r, y, x (rows) scratch."
  (declare (type (simple-array double-float (*)) j want a-des g l r y x)
           (type fixnum rows nd)
           (type double-float damping)
           (optimize (speed 3) (safety 0)))
  (loop for i of-type fixnum from 0 below rows do
    (loop for c of-type fixnum from 0 below rows do
      (let ((s 0.0d0))
        (declare (type double-float s))
        (loop for k of-type fixnum from 0 below nd do
          (setf s (+ s (* (aref j (+ (* i nd) k)) (aref j (+ (* c nd) k))))))
        (setf (aref g (+ (* i rows) c)) s)))
    (setf (aref g (+ (* i rows) i)) (+ (aref g (+ (* i rows) i)) (* damping damping)))
    (let ((s 0.0d0))
      (declare (type double-float s))
      (loop for k of-type fixnum from 0 below nd do
        (setf s (+ s (* (aref j (+ (* i nd) k)) (aref want k)))))
      (setf (aref r i) s)))
  (cholesky-banachiewicz g l rows)
  (cholesky-solve l r y x rows)
  (loop for k of-type fixnum from 0 below nd do
    (let ((s 0.0d0))
      (declare (type double-float s))
      (loop for i of-type fixnum from 0 below rows do
        (setf s (+ s (* (aref j (+ (* i nd) k)) (aref x i)))))
      (setf (aref a-des k) (- (aref want k) s))))
  a-des)

;; -----------------------------------------------------------------------------
;; 3. A CONTACT'S FRAME: its normal the felt force's direction (up where it feels none), two tangents
;; -----------------------------------------------------------------------------
(defun contact-frame (f q at)
  "q's 9 from at on: the frame's rows (normal, tangent, tangent) of the felt force f (3)."
  (declare (type (simple-array double-float (*)) f q)
           (type fixnum at)
           (optimize (speed 3) (safety 0)))
  (let* ((norm (sqrt (+ (* (aref f 0) (aref f 0)) (* (aref f 1) (aref f 1)) (* (aref f 2) (aref f 2)))))
         (n0 (if (> norm 0.0d0) (/ (aref f 0) norm) 0.0d0))
         (n1 (if (> norm 0.0d0) (/ (aref f 1) norm) 1.0d0))
         (n2 (if (> norm 0.0d0) (/ (aref f 2) norm) 0.0d0))
         (e0 (if (> (abs n0) 0.9d0) 0.0d0 1.0d0))
         (e2 (if (> (abs n0) 0.9d0) 1.0d0 0.0d0))
         (t0 (- (* n1 e2) (* n2 0.0d0)))
         (t1 (- (* n2 e0) (* n0 e2)))
         (t2 (- (* n0 0.0d0) (* n1 e0)))
         (tn (sqrt (+ (* t0 t0) (* t1 t1) (* t2 t2)))))
    (declare (type double-float norm n0 n1 n2 e0 e2 t0 t1 t2 tn))
    (setf t0 (/ t0 tn) t1 (/ t1 tn) t2 (/ t2 tn))
    (setf (aref q (+ at 0)) n0 (aref q (+ at 1)) n1 (aref q (+ at 2)) n2
          (aref q (+ at 3)) t0 (aref q (+ at 4)) t1 (aref q (+ at 5)) t2
          (aref q (+ at 6)) (- (* n1 t2) (* n2 t1))
          (aref q (+ at 7)) (- (* n2 t0) (* n0 t2))
          (aref q (+ at 8)) (- (* n0 t1) (* n1 t0)))
    q))

;; -----------------------------------------------------------------------------
;; 4. BOX-PROJECTED GAUSS-SEIDEL MLCP OVER THE DELASSUS MATRIX IN THE CONTACTS' FRAMES
;; -----------------------------------------------------------------------------
(defun pgs-box-mlcp (delassus b z rows mu softness sweeps)
  "w = Delassus z + b, 3 rows a contact (normal, two tangents): each normal pressing alone (0 <= w_n, 0 <= z_n,
   one of them 0), each tangent within the box |z_t| <= mu z_n, sweeps over, z in place - warm from what is
   felt -, a contact's softness on the diagonal; the complementarity's residual, the largest |min(w_n, z_n)| after."
  (declare (type (simple-array double-float (*)) delassus b z)
           (type fixnum rows sweeps)
           (type double-float mu softness)
           (optimize (speed 3) (safety 0)))
  (loop repeat sweeps do
    (loop for r of-type fixnum from 0 below rows do
      (let ((w (aref b r))
            (d (+ (aref delassus (+ (* r rows) r)) softness))
            (x 0.0d0))
        (declare (type double-float w d x))
        (loop for c of-type fixnum from 0 below rows do
          (setf w (+ w (* (aref delassus (+ (* r rows) c)) (aref z c)))))
        (setf x (- (aref z r) (/ w d)))
        (if (= (mod r 3) 0)
            (setf (aref z r) (if (> x 0.0d0) x 0.0d0))
            (let ((lim (* mu (aref z (- r (mod r 3))))))
              (declare (type double-float lim))
              (setf (aref z r) (cond ((< x (- lim)) (- lim))
                                     ((> x lim) lim)
                                     (t x))))))))
  (let ((residual 0.0d0))
    (declare (type double-float residual))
    (loop for i of-type fixnum from 0 below rows by 3 do
      (let ((w (aref b i)))
        (declare (type double-float w))
        (loop for c of-type fixnum from 0 below rows do
          (setf w (+ w (* (aref delassus (+ (* i rows) c)) (aref z c)))))
        (let ((gap (abs (min w (aref z i)))))
          (declare (type double-float gap))
          (when (> gap residual) (setf residual gap)))))
    residual))

;; -----------------------------------------------------------------------------
;; 5. THE STEP: her posture in the chains' null space, the forces by complementarity, her torques with them
;; M (n x n) her mass matrix, h (n) her bias, v (n) her velocity, jc (3k x n) the contacts' Jacobians, jdv (3k) their
;; drift, f (3k) what they feel; P the dofs no drive holds (her base, her passive joints), A her driven, H her held.
;; -----------------------------------------------------------------------------
(defun shrunk-toward (tau tau0 lo hi na)
  "machine.stack.shrunk: the largest s in [0, 1] with every tau0 + s (tau - tau0) within [lo, hi] (0 within), each
   drive's in closed form, the least of them; 0 where none is."
  (declare (type (simple-array double-float (*)) tau tau0 lo hi)
           (type fixnum na)
           (optimize (speed 3) (safety 0)))
  (let ((least 1.0d0))
    (declare (type double-float least))
    (loop for i of-type fixnum from 0 below na do
      (let* ((t0 (if (and (>= (aref tau0 i) (aref lo i)) (<= (aref tau0 i) (aref hi i))) (aref tau0 i) 0.0d0))
             (d (- (aref tau i) t0))
             (leaves (if (> d 0.0d0) (/ (- (aref hi i) t0) d) (/ (- (aref lo i) t0) d)))
             (enters (if (> d 0.0d0) (/ (- (aref lo i) t0) d) (/ (- (aref hi i) t0) d)))
             (s (if (= d 0.0d0)
                    (if (and (>= (aref tau i) (aref lo i)) (<= (aref tau i) (aref hi i))) 1.0d0 0.0d0)
                    (min 1.0d0 leaves))))
        (declare (type double-float t0 d leaves enters s))
        (setf s (if (>= s (max 0.0d0 (if (= d 0.0d0) 0.0d0 enters))) s 0.0d0))
        (setf least (if (= i 0) s (min least s)))))
    (max 0.0d0 (min 1.0d0 least))))

(defun closed-chain-step (m h v jc jdv f want clamp n np na nh p a hh damping softness bleed mu sweeps tau qdd z work)
  "tau (na) her drives' torques, qdd (n) every acceleration, z (3k) the contacts' forces, world; (values tau qdd z
   residual scale). M (n x n), h, v (n); jc (3k x n) the contacts' Jacobians, jdv (3k) their drift, f (3k) what they
   feel; want (na) her posture's acceleration on her driven joints A, clamp (na) their bounds; P (np) the dofs no drive
   holds, HH (nh) her held joints; the regime's damping, softness, bleed, mu, sweeps (closed-chain-regime); work a flat
   scratch, carved. The twins spell every product in this order (machine.stack.chain, wbc_chain.c), the host's
   bounds its back-EMF's and WEP's too."
  (declare (type (simple-array double-float (*)) m h v jc jdv f want clamp tau qdd z work)
           (type (simple-array fixnum (*)) p a hh)
           (type fixnum n np na nh sweeps)
           (type double-float damping softness bleed mu)
           (optimize (speed 3) (safety 0)))
  (let* ((rows (length f))
         (ng (+ na nh))
         (k (floor rows 3))
         (held-k 50.0d0)
         (at 0))
    (declare (type fixnum rows ng k at) (type double-float held-k))
    (flet ((carve (count)
             (let ((out (make-array count :element-type 'double-float :displaced-to work :displaced-index-offset at)))
               (setf at (+ at count))
               out)))
      (let* ((ja (carve (* rows na))) (g (carve (* rows rows))) (l (carve (* rows rows)))
             (r (carve rows)) (y (carve rows)) (x (carve rows)) (a-des (carve na))
             (qg (carve ng)) (qg0 (carve ng)) (mpp (carve (* np np))) (lpp (carve (* np np)))
             (col (carve np)) (xcol (carve np)) (ycol (carve np)) (xs (carve (* rows np)))
             (dl (carve (* rows rows))) (rhs (carve np)) (rhs0 (carve np)) (yp (carve np))
             (b (carve rows)) (q (carve (* 9 k))) (dloc (carve (* rows rows))) (bloc (carve rows))
             (zl (carve rows)) (fi (carve 3)) (qdd0 (carve n)) (tau1 (carve na)) (tau0 (carve na))
             (lo (carve na)) (hi (carve na)) (zeros (carve na))
             (residual 0.0d0) (scale 1.0d0))
        (declare (type double-float residual scale))
        ;; 1. her posture in the contacts' damped null space on A
        (loop for i of-type fixnum from 0 below rows do
          (loop for c of-type fixnum from 0 below na do
            (setf (aref ja (+ (* i na) c)) (aref jc (+ (* i n) (aref a c))))))
        (dls-nullspace-posture ja want a-des rows na damping g l r y x)
        (loop for c of-type fixnum from 0 below na do
          (setf (aref qg c) (aref a-des c) (aref qg0 c) 0.0d0))
        (loop for i of-type fixnum from 0 below nh do
          (setf (aref qg (+ na i)) (* (- held-k) (aref v (aref hh i)))
                (aref qg0 (+ na i)) (aref qg (+ na i))))
        ;; 2. the Delassus matrix over the dofs no drive holds, M_PP by Cholesky: X_i = M_PP^-1 J_P[i]^T
        (loop for i of-type fixnum from 0 below np do
          (loop for c of-type fixnum from 0 below np do
            (setf (aref mpp (+ (* i np) c)) (aref m (+ (* (aref p i) n) (aref p c))))))
        (cholesky-banachiewicz mpp lpp np)
        (loop for i of-type fixnum from 0 below rows do
          (loop for c of-type fixnum from 0 below np do
            (setf (aref col c) (aref jc (+ (* i n) (aref p c)))))
          (cholesky-solve lpp col ycol xcol np)
          (loop for c of-type fixnum from 0 below np do
            (setf (aref xs (+ (* i np) c)) (aref xcol c))))
        (loop for i of-type fixnum from 0 below rows do
          (loop for j of-type fixnum from 0 below rows do
            (let ((s 0.0d0))
              (declare (type double-float s))
              (loop for c of-type fixnum from 0 below np do
                (setf s (+ s (* (aref jc (+ (* i n) (aref p c))) (aref xs (+ (* j np) c))))))
              (setf (aref dl (+ (* i rows) j)) s))))
        ;; its offset: what the dofs no drive holds do with no force, rhs = -h_P - M_PA a-des - M_PH qdd_H
        (flet ((offset (qgiven rhs-out)
                 (loop for i of-type fixnum from 0 below np do
                   (let ((s (- (aref h (aref p i)))))
                     (declare (type double-float s))
                     (loop for c of-type fixnum from 0 below na do
                       (setf s (- s (* (aref m (+ (* (aref p i) n) (aref a c))) (aref qgiven c)))))
                     (loop for c of-type fixnum from 0 below nh do
                       (setf s (- s (* (aref m (+ (* (aref p i) n) (aref hh c))) (aref qgiven (+ na c))))))
                     (setf (aref rhs-out i) s)))))
          (offset qg rhs)
          (offset qg0 rhs0))
        (cholesky-solve lpp rhs ycol yp np)
        (loop for i of-type fixnum from 0 below rows do
          (let ((s 0.0d0) (sv 0.0d0))
            (declare (type double-float s sv))
            (loop for c of-type fixnum from 0 below np do
              (setf s (+ s (* (aref jc (+ (* i n) (aref p c))) (aref yp c)))))
            (loop for c of-type fixnum from 0 below na do
              (setf s (+ s (* (aref jc (+ (* i n) (aref a c))) (aref qg c)))))
            (loop for c of-type fixnum from 0 below nh do
              (setf s (+ s (* (aref jc (+ (* i n) (aref hh c))) (aref qg (+ na c))))))
            (loop for c of-type fixnum from 0 below n do
              (setf sv (+ sv (* (aref jc (+ (* i n) c)) (aref v c)))))
            (setf (aref b i) (+ s (aref jdv i) (* bleed sv)))))
        ;; into the contacts' frames, z warm from what is felt, the Gauss-Seidel, z back in the world
        (loop for i of-type fixnum from 0 below k do
          (setf (aref fi 0) (aref f (* 3 i)) (aref fi 1) (aref f (+ (* 3 i) 1)) (aref fi 2) (aref f (+ (* 3 i) 2)))
          (contact-frame fi q (* 9 i)))
        (loop for i of-type fixnum from 0 below k do
          (loop for aa of-type fixnum from 0 below 3 do
            (let ((s 0.0d0) (u 0.0d0))
              (declare (type double-float s u))
              (loop for c of-type fixnum from 0 below 3 do
                (setf s (+ s (* (aref q (+ (* 9 i) (* 3 aa) c)) (aref b (+ (* 3 i) c))))
                      u (+ u (* (aref q (+ (* 9 i) (* 3 aa) c)) (aref f (+ (* 3 i) c))))))
              (setf (aref bloc (+ (* 3 i) aa)) s (aref zl (+ (* 3 i) aa)) u)
              (loop for j of-type fixnum from 0 below k do
                (loop for e of-type fixnum from 0 below 3 do
                  (setf s 0.0d0)
                  (loop for c of-type fixnum from 0 below 3 do
                    (loop for d of-type fixnum from 0 below 3 do
                      (setf s (+ s (* (aref q (+ (* 9 i) (* 3 aa) c))
                                      (aref dl (+ (* (+ (* 3 i) c) rows) (* 3 j) d))
                                      (aref q (+ (* 9 j) (* 3 e) d)))))))
                  (setf (aref dloc (+ (* (+ (* 3 i) aa) rows) (* 3 j) e)) s))))))
        (setf residual (pgs-box-mlcp dloc bloc zl rows mu softness sweeps))
        (loop for i of-type fixnum from 0 below k do
          (loop for c of-type fixnum from 0 below 3 do
            (let ((s 0.0d0))
              (declare (type double-float s))
              (loop for aa of-type fixnum from 0 below 3 do
                (setf s (+ s (* (aref q (+ (* 9 i) (* 3 aa) c)) (aref zl (+ (* 3 i) aa))))))
              (setf (aref z (+ (* 3 i) c)) s))))
        ;; 3. every acceleration and her torques with the forces: qdd_P = M_PP^-1 (J_P^T z + rhs), tau = (M qdd + h - J^T z)_A
        (flet ((answered (qgiven rhs-in qdd-out tau-out)
                 (loop for c of-type fixnum from 0 below np do
                   (let ((s (aref rhs-in c)))
                     (declare (type double-float s))
                     (loop for i of-type fixnum from 0 below rows do
                       (setf s (+ s (* (aref jc (+ (* i n) (aref p c))) (aref z i)))))
                     (setf (aref col c) s)))
                 (cholesky-solve lpp col ycol xcol np)
                 (fill qdd-out 0.0d0)
                 (loop for c of-type fixnum from 0 below np do (setf (aref qdd-out (aref p c)) (aref xcol c)))
                 (loop for c of-type fixnum from 0 below na do (setf (aref qdd-out (aref a c)) (aref qgiven c)))
                 (loop for c of-type fixnum from 0 below nh do (setf (aref qdd-out (aref hh c)) (aref qgiven (+ na c))))
                 (loop for i of-type fixnum from 0 below na do
                   (let ((s (aref h (aref a i))) (jt 0.0d0))
                     (declare (type double-float s jt))
                     (loop for c of-type fixnum from 0 below n do
                       (setf s (+ s (* (aref m (+ (* (aref a i) n) c)) (aref qdd-out c)))))
                     (loop for row of-type fixnum from 0 below rows do
                       (setf jt (+ jt (* (aref jc (+ (* row n) (aref a i))) (aref z row)))))
                     (setf (aref tau-out i) (- s jt))))))
          (answered qg rhs qdd tau1)
          (answered qg0 rhs0 qdd0 tau0))
        ;; shrunk to her clamps toward nothing asked of her joints, then toward 0
        (loop for i of-type fixnum from 0 below na do
          (setf (aref lo i) (- (aref clamp i)) (aref hi i) (aref clamp i) (aref zeros i) 0.0d0))
        (when (loop for i of-type fixnum from 0 below na thereis (> (abs (aref tau1 i)) (aref clamp i)))
          (setf scale (shrunk-toward tau1 tau0 lo hi na))
          (loop for i of-type fixnum from 0 below na do
            (setf (aref tau1 i) (+ (aref tau0 i) (* scale (- (aref tau1 i) (aref tau0 i))))))
          (loop for i of-type fixnum from 0 below n do
            (setf (aref qdd i) (+ (aref qdd0 i) (* scale (- (aref qdd i) (aref qdd0 i))))))
          (let ((s0 (if (loop for i of-type fixnum from 0 below na thereis (> (abs (aref tau0 i)) (aref clamp i)))
                        (shrunk-toward tau1 zeros lo hi na)
                        1.0d0)))
            (declare (type double-float s0))
            (when (< s0 1.0d0)
              (loop for i of-type fixnum from 0 below na do (setf (aref tau1 i) (* s0 (aref tau1 i))))
              (loop for i of-type fixnum from 0 below n do (setf (aref qdd i) (* s0 (aref qdd i)))))
            (setf scale (* s0 scale))))
        (loop for i of-type fixnum from 0 below na do
          (setf (aref tau i) (max (aref lo i) (min (aref hi i) (aref tau1 i)))))
        (values tau qdd z residual scale)))))
