;; =============================================================================
;; FILE: closed_chain_plan.lisp
;; DOMAIN: The Closed Chains' Plan - Tasks Answered Through Her Dynamics, the Contacts Held Round by Round
;; REQUIRES: closed_chain_complementarity.lisp
;; Hers: `machine.stack.chain.planned` runs it on her law and `wbc_chain.c` bit for bit; the regime's constants are
;; `closed-chain-regime`'s (closed_chain_complementarity.lisp). Written out on flat row-major double-float vectors.
;; =============================================================================
(defpackage :ortho-dsp.closed-chain-plan
  (:use :ortho-dsp.core
        :ortho-dsp.closed-chain)
  (:export #:closed-chain-plan))
(in-package :ortho-dsp.closed-chain-plan)

;; A task asked of her base - her pelvis's or trunk's turn, a sole's swing - reaches no drive through a least squares
;; over every acceleration: the free base dofs meet it, and the forces then overrule them. The plan is dynamics-
;; consistent: with the contacts held (each a point's normal and two tangents, rows3) and her dynamics, the dofs no
;; drive holds are affine in her driven ones, qdd_P = p0 + Pg qg, the forces known (Fk, a sliding contact's friction);
;; the tasks' rows go into the driven accelerations alone, the held joints fixed, and beside them the contacts'
;; internal forces - those with no effect on the dofs no drive holds, null(Ja_P^T), her drives alone would fight
;; them - are kept at the felt split: two contacts on one prescribed chain hold her in any split of her weight, and
;; the damped map alone pulled 280 N on her pelvis against 630 on her sole. Those rows by damped least squares
;; (task-damping), her posture in their null space (dls-damping-m); then the plan's own forces: a contact pulled on
;; goes free, one dragged past its box |z_t| <= mu z_n slides, its friction the box's; planned again, plan-rounds over.
;; A contact felt under hold-n starts free (her fingers' arrival at 2-4 N, held, threw her elbow).
;; The Gauss-Seidel of the step is warm from these forces: warm from what is felt, the mattress's lag kept them there
;; and the push the plan counted on never reached her drives. Her own gains (turn-kp, posture-kp): lying down is slow.
(defun closed-chain-plan (m h v jc jdv f q rp rg r m1 want held mm-p mm-g a n np na nh
                          damping task-damping rounds bleed mu hold-n w-force w-posture x qdd-p z work)
  "x (na) her driven accelerations, qdd-p (np) her base's and passive joints' the plan foresees, z (3k) the forces
   it counts on, world; m (n x n), h (n), v (n) hers, jc (3k x n) the contacts' Jacobians, jdv their drift, f (3k)
   what they feel, q (9k) their frames (contact-frame), rp (m1 x np) and rg (m1 x ng) the tasks' rows on the dofs no
   drive holds and on the given (driven then held), r (m1) their right-hand sides, want (na) the posture's
   acceleration, held (nh) the held joints' accelerations; mm-p (np) and mm-g (ng) the dofs' indices; a (na) the
   driven dofs. Every product in the order machine.stack.chain.planned and wbc_chain.c spell it."
  (declare (type (simple-array double-float (*)) m h v jc jdv f q rp rg r want held x qdd-p z work)
           (type (simple-array fixnum (*)) mm-p mm-g a)
           (type fixnum m1 n np na nh rounds)
           (type double-float damping task-damping bleed mu hold-n w-force w-posture)
           (optimize (speed 3) (safety 0)))
  (let* ((k3 (length f))
         (ng (+ na nh))
         (lam2 (* damping damping))
         (task2 (* task-damping task-damping))
         (at 0))
    (declare (type fixnum k3 ng at) (type double-float lam2 task2))
    (flet ((carve (count)
             (let ((out (make-array count :element-type 'double-float :displaced-to work :displaced-index-offset at)))
               (setf at (+ at count))
               out))
           (mm (out am an bn aa bb)
             ;; out (am x bn) = aa (am x an) bb (an x bn), row-major
             (loop for i of-type fixnum from 0 below am do
               (loop for j of-type fixnum from 0 below bn do
                 (let ((s 0.0d0))
                   (declare (type double-float s))
                   (loop for c of-type fixnum from 0 below an do
                     (setf s (+ s (* (aref aa (+ (* i an) c)) (aref bb (+ (* c bn) j))))))
                   (setf (aref out (+ (* i bn) j)) s))))))
      (let* ((rows3 (make-array k3 :element-type 'fixnum :initial-element 1))
             (fk (carve k3)) (mpp (carve (* np np))) (lpp (carve (* np np))) (minv (carve (* np np)))
             (col (carve np)) (ycol (carve np)) (xcol (carve np)) (mpg (carve (* np ng))) (mmg (carve (* np ng)))
             (jp (carve (* k3 np))) (jg (carve (* k3 ng))) (jap (carve (* k3 np))) (japt (carve (* np k3)))
             (jag (carve (* k3 ng))) (jav (carve k3)) (jad (carve k3)) (fa (carve k3)) (rhs0 (carve np)) (rhsk (carve np))
             (w1 (carve (* k3 np))) (dl (carve (* k3 k3))) (ldl (carve (* k3 k3))) (dinv (carve (* k3 k3)))
             (bb0 (carve k3)) (wm (carve (* k3 ng))) (bg (carve (* k3 ng))) (c0 (carve k3)) (xk (carve (* np k3)))
             (kk (carve (* np k3))) (p0 (carve np)) (kb (carve (* np ng))) (pg (carve (* np ng)))
             (g0 (carve (* np np))) (lg0 (carve (* np np))) (yi (carve (* np k3))) (nint (carve (* k3 k3)))
             (zd (carve (* k3 na))) (zx (carve (* k3 na))) (d2 (carve k3)) (zc0 (carve k3)) (zc (carve k3))
             (mr (+ m1 k3)) (r1x (carve (* mr na))) (r1 (carve mr)) (t1 (carve m1)) (tt (carve (* m1 nh))) (u (carve m1))
             (r1xt (carve (* na mr))) (gg1 (carve (* na na))) (h1 (carve (* na na))) (l1 (carve (* na na)))
             (g1 (carve na)) (y1 (carve na)) (x1 (carve na)) (ycol2 (carve na)) (nn (carve (* na na)))
             (r2n (carve (* na na))) (h2 (carve (* na na))) (l2 (carve (* na na))) (dd (carve na)) (g2 (carve na))
             (x2 (carve na)) (qg (carve ng)) (bq (carve k3)) (za (carve k3)) (zfull (carve k3)) (kcol (carve k3)))
        (declare (type fixnum mr))
        ;; her dynamics on the dofs no drive holds: M_PP by Cholesky, its inverse column by column (numpy's inv)
        (loop for i of-type fixnum from 0 below np do
          (loop for c of-type fixnum from 0 below np do
            (setf (aref mpp (+ (* i np) c)) (aref m (+ (* (aref mm-p i) n) (aref mm-p c)))))
          (loop for g of-type fixnum from 0 below ng do
            (setf (aref mpg (+ (* i ng) g)) (aref m (+ (* (aref mm-p i) n) (aref mm-g g)))))
          (setf (aref rhs0 i) (- (aref h (aref mm-p i)))))
        (cholesky-banachiewicz mpp lpp np)
        (loop for j of-type fixnum from 0 below np do
          (fill col 0.0d0)
          (setf (aref col j) 1.0d0)
          (cholesky-solve lpp col ycol xcol np)
          (loop for i of-type fixnum from 0 below np do (setf (aref minv (+ (* i np) j)) (aref xcol i))))
        (mm mmg np np ng minv mpg)
        (loop for rw of-type fixnum from 0 below k3 do
          (loop for c of-type fixnum from 0 below np do (setf (aref jp (+ (* rw np) c)) (aref jc (+ (* rw n) (aref mm-p c)))))
          (loop for g of-type fixnum from 0 below ng do (setf (aref jg (+ (* rw ng) g)) (aref jc (+ (* rw n) (aref mm-g g)))))
          (setf (aref fk rw) 0.0d0 (aref zfull rw) 0.0d0))
        (loop for i of-type fixnum from 0 below (floor k3 3) do
          (let ((on (if (>= (sqrt (+ (* (aref f (* 3 i)) (aref f (* 3 i))) (* (aref f (+ (* 3 i) 1)) (aref f (+ (* 3 i) 1)))
                                  (* (aref f (+ (* 3 i) 2)) (aref f (+ (* 3 i) 2))))) hold-n) 1 0)))
            (declare (type fixnum on))
            (setf (aref rows3 (* 3 i)) on (aref rows3 (+ (* 3 i) 1)) on (aref rows3 (+ (* 3 i) 2)) on)))
        (loop for rnd of-type fixnum from 0 below rounds do
          (let ((nk 0) (changed nil))
            (declare (type fixnum nk))
            ;; the contacts held this round, their rows gathered
            (loop for rw of-type fixnum from 0 below k3 do
              (when (= (aref rows3 rw) 1)
                (loop for c of-type fixnum from 0 below np do
                  (setf (aref jap (+ (* nk np) c)) (aref jp (+ (* rw np) c))))
                (loop for g of-type fixnum from 0 below ng do
                  (setf (aref jag (+ (* nk ng) g)) (aref jg (+ (* rw ng) g))))
                (let ((sv 0.0d0))
                  (declare (type double-float sv))
                  (loop for c of-type fixnum from 0 below n do
                    (setf sv (+ sv (* (aref jc (+ (* rw n) c)) (aref v c)))))
                  (setf (aref jav nk) sv))
                (setf (aref jad nk) (aref jdv rw) (aref fa nk) (aref f rw))
                (incf nk)))
            (loop for c of-type fixnum from 0 below np do
              (loop for i of-type fixnum from 0 below nk do
                (setf (aref japt (+ (* c nk) i)) (aref jap (+ (* i np) c)))))
            ;; rhsk = -h_P + J_P^T Fk
            (loop for c of-type fixnum from 0 below np do
              (let ((s (aref rhs0 c)))
                (declare (type double-float s))
                (loop for rw of-type fixnum from 0 below k3 do
                  (setf s (+ s (* (aref jp (+ (* rw np) c)) (aref fk rw)))))
                (setf (aref rhsk c) s)))
            (cond
              ((> nk 0)
               ;; the map: W = Ja_P M_PP^-1, Dl = W Ja_P^T + lam^2 I, bb0 = W rhsk + jdv, Bg = Ja_G - W M_PG,
               ;; c0 = -bleed Ja v - bb0, K = M_PP^-1 Ja_P^T Dl^-1, p0 = K c0 + M_PP^-1 rhsk, Pg = -(K Bg + M_PP^-1 M_PG)
               (mm w1 nk np np jap minv)
               (mm dl nk np nk w1 japt)
               (loop for i of-type fixnum from 0 below nk do
                 (setf (aref dl (+ (* i nk) i)) (+ (aref dl (+ (* i nk) i)) lam2)))
               (mm bb0 nk np 1 w1 rhsk)
               (loop for i of-type fixnum from 0 below nk do
                 (setf (aref bb0 i) (+ (aref bb0 i) (aref jad i))))
               (mm wm nk np ng w1 mpg)
               (loop for i of-type fixnum from 0 below (* nk ng) do
                 (setf (aref bg i) (- (aref jag i) (aref wm i))))
               (loop for i of-type fixnum from 0 below nk do
                 (setf (aref c0 i) (- (* (- bleed) (aref jav i)) (aref bb0 i))))
               (cholesky-banachiewicz dl ldl nk)
               (loop for j of-type fixnum from 0 below nk do
                 (fill kcol 0.0d0)
                 (setf (aref kcol j) 1.0d0)
                 (cholesky-solve ldl kcol za bq nk)
                 (loop for i of-type fixnum from 0 below nk do (setf (aref dinv (+ (* i nk) j)) (aref bq i))))
               (mm xk np np nk minv japt)
               (mm kk np nk nk xk dinv)
               (mm p0 np nk 1 kk c0)
               (mm col np np 1 minv rhsk)
               (loop for i of-type fixnum from 0 below np do
                 (setf (aref p0 i) (+ (aref p0 i) (aref col i))))
               (mm kb np nk ng kk bg)
               (loop for i of-type fixnum from 0 below (* np ng) do
                 (setf (aref pg i) (- (+ (aref kb i) (aref mmg i)))))
               ;; the internal forces: N_int = I - Ja_P (Ja_P^T Ja_P + lam^2 I)^-1 Ja_P^T
               (mm g0 np nk np japt jap)
               (loop for i of-type fixnum from 0 below np do
                 (setf (aref g0 (+ (* i np) i)) (+ (aref g0 (+ (* i np) i)) lam2)))
               (cholesky-banachiewicz g0 lg0 np)
               (loop for j of-type fixnum from 0 below nk do
                 (loop for i of-type fixnum from 0 below np do (setf (aref col i) (aref japt (+ (* i nk) j))))
                 (cholesky-solve lg0 col ycol xcol np)
                 (loop for i of-type fixnum from 0 below np do (setf (aref yi (+ (* i nk) j)) (aref xcol i))))
               (mm nint nk np nk jap yi)
               (loop for i of-type fixnum from 0 below nk do
                 (loop for c of-type fixnum from 0 below nk do
                   (setf (aref nint (+ (* i nk) c)) (- (if (= i c) 1.0d0 0.0d0) (aref nint (+ (* i nk) c))))))
               ;; Zx = N_int Dl^-1 Bg[:, driven], zc = N_int (Dl^-1 (c0 - Bg[:, held] held) - f)
               (loop for i of-type fixnum from 0 below nk do
                 (loop for c of-type fixnum from 0 below na do
                   (let ((s 0.0d0))
                     (declare (type double-float s))
                     (loop for e of-type fixnum from 0 below nk do
                       (setf s (+ s (* (aref dinv (+ (* i nk) e)) (aref bg (+ (* e ng) c))))))
                     (setf (aref zd (+ (* i na) c)) s))))
               (mm zx nk nk na nint zd)
               (loop for i of-type fixnum from 0 below nk do
                 (let ((s 0.0d0))
                   (declare (type double-float s))
                   (loop for c of-type fixnum from 0 below nh do
                     (setf s (+ s (* (aref bg (+ (* i ng) na c)) (aref held c)))))
                   (setf (aref d2 i) (- (aref c0 i) s))))
               (mm zc0 nk nk 1 dinv d2)
               (loop for i of-type fixnum from 0 below nk do
                 (setf (aref zc0 i) (- (aref zc0 i) (aref fa i))))
               (mm zc nk nk 1 nint zc0))
              (t
               (mm p0 np np 1 minv rhsk)
               (loop for i of-type fixnum from 0 below (* np ng) do
                 (setf (aref pg i) (- (aref mmg i))))))
            ;; the tasks' rows in the driven accelerations: R1x = Rp Pg[:, driven] + Rg[:, driven],
            ;; r1x = (r - Rp p0) - (Rp Pg[:, held] + Rg[:, held]) held; the force rows w (Zx x = zc) beneath them
            (loop for i of-type fixnum from 0 below m1 do
              (loop for c of-type fixnum from 0 below na do
                (let ((s (aref rg (+ (* i ng) c))))
                  (declare (type double-float s))
                  (loop for e of-type fixnum from 0 below np do
                    (setf s (+ s (* (aref rp (+ (* i np) e)) (aref pg (+ (* e ng) c))))))
                  (setf (aref r1x (+ (* i na) c)) s)))
              (let ((s 0.0d0) (uu 0.0d0))
                (declare (type double-float s uu))
                (loop for e of-type fixnum from 0 below np do
                  (setf s (+ s (* (aref rp (+ (* i np) e)) (aref p0 e)))))
                (setf (aref t1 i) s)
                (loop for c of-type fixnum from 0 below nh do
                  (let ((tc (aref rg (+ (* i ng) na c))))
                    (declare (type double-float tc))
                    (loop for e of-type fixnum from 0 below np do
                      (setf tc (+ tc (* (aref rp (+ (* i np) e)) (aref pg (+ (* e ng) na c))))))
                    (setf (aref tt (+ (* i nh) c)) tc)
                    (setf uu (+ uu (* tc (aref held c))))))
                (setf (aref u i) uu)
                (setf (aref r1 i) (- (- (aref r i) (aref t1 i)) uu))))
            (loop for i of-type fixnum from 0 below nk do
              (loop for c of-type fixnum from 0 below na do
                (setf (aref r1x (+ (* (+ m1 i) na) c)) (* w-force (aref zx (+ (* i na) c)))))
              (setf (aref r1 (+ m1 i)) (* w-force (aref zc i))))
            (let ((mt (+ m1 nk)))
              (declare (type fixnum mt))
              (cond
                ((> mt 0)
                 ;; x1 = (R1x^T R1x + task^2 I)^-1 R1x^T r1, N = I - (R1x^T R1x + task^2 I)^-1 R1x^T R1x
                 (loop for i of-type fixnum from 0 below mt do
                   (loop for c of-type fixnum from 0 below na do
                     (setf (aref r1xt (+ (* c mt) i)) (aref r1x (+ (* i na) c)))))
                 (mm gg1 na mt na r1xt r1x)
                 (loop for i of-type fixnum from 0 below (* na na) do (setf (aref h1 i) (aref gg1 i)))
                 (loop for i of-type fixnum from 0 below na do
                   (setf (aref h1 (+ (* i na) i)) (+ (aref h1 (+ (* i na) i)) task2)))
                 (mm g1 na mt 1 r1xt r1)
                 (cholesky-banachiewicz h1 l1 na)
                 (cholesky-solve l1 g1 ycol2 x1 na)
                 (loop for j of-type fixnum from 0 below na do
                   (loop for i of-type fixnum from 0 below na do (setf (aref dd i) (aref gg1 (+ (* i na) j))))
                   (cholesky-solve l1 dd ycol2 y1 na)
                   (loop for i of-type fixnum from 0 below na do
                     (setf (aref nn (+ (* i na) j)) (- (if (= i j) 1.0d0 0.0d0) (aref y1 i))))))
                (t
                 (loop for i of-type fixnum from 0 below na do
                   (setf (aref x1 i) 0.0d0)
                   (loop for c of-type fixnum from 0 below na do
                     (setf (aref nn (+ (* i na) c)) (if (= i c) 1.0d0 0.0d0)))))))
            ;; her posture in the null space: R2n = w N, x2 = (R2n^T R2n + lam^2 I)^-1 R2n^T w (want - x1), x = x1 + N x2
            (loop for i of-type fixnum from 0 below (* na na) do
              (setf (aref r2n i) (* w-posture (aref nn i))))
            (loop for i of-type fixnum from 0 below na do
              (loop for c of-type fixnum from 0 below na do
                (let ((s 0.0d0))
                  (declare (type double-float s))
                  (loop for e of-type fixnum from 0 below na do
                    (setf s (+ s (* (aref r2n (+ (* e na) i)) (aref r2n (+ (* e na) c))))))
                  (setf (aref h2 (+ (* i na) c)) s)))
              (setf (aref h2 (+ (* i na) i)) (+ (aref h2 (+ (* i na) i)) lam2))
              (setf (aref dd i) (* w-posture (- (aref want i) (aref x1 i)))))
            (loop for i of-type fixnum from 0 below na do
              (let ((s 0.0d0))
                (declare (type double-float s))
                (loop for e of-type fixnum from 0 below na do
                  (setf s (+ s (* (aref r2n (+ (* e na) i)) (aref dd e)))))
                (setf (aref g2 i) s)))
            (cholesky-banachiewicz h2 l2 na)
            (cholesky-solve l2 g2 ycol2 x2 na)
            (loop for i of-type fixnum from 0 below na do
              (let ((s (aref x1 i)))
                (declare (type double-float s))
                (loop for c of-type fixnum from 0 below na do
                  (setf s (+ s (* (aref nn (+ (* i na) c)) (aref x2 c)))))
                (setf (aref x i) s (aref qg i) s)))
            (loop for i of-type fixnum from 0 below nh do (setf (aref qg (+ na i)) (aref held i)))
            ;; what the plan foresees of her base, and its own forces: a pull frees a contact, a drag past its box
            ;; sets it sliding with its friction known
            (loop for i of-type fixnum from 0 below np do
              (let ((s (aref p0 i)))
                (declare (type double-float s))
                (loop for g of-type fixnum from 0 below ng do
                  (setf s (+ s (* (aref pg (+ (* i ng) g)) (aref qg g)))))
                (setf (aref qdd-p i) s)))
            (when (= nk 0)
              (loop for rw of-type fixnum from 0 below k3 do (setf (aref zfull rw) (aref fk rw)))
              (return))
            (loop for i of-type fixnum from 0 below nk do
              (let ((s (aref c0 i)))
                (declare (type double-float s))
                (loop for g of-type fixnum from 0 below ng do
                  (setf s (- s (* (aref bg (+ (* i ng) g)) (aref qg g)))))
                (setf (aref bq i) s)))
            (mm za nk nk 1 dinv bq)
            (let ((j 0))
              (declare (type fixnum j))
              (loop for rw of-type fixnum from 0 below k3 do
                (setf (aref zfull rw) (aref fk rw))
                (when (= (aref rows3 rw) 1)
                  (setf (aref zfull rw) (+ (aref fk rw) (aref za j)))
                  (incf j))))
            (loop for i of-type fixnum from 0 below (floor k3 3) do
              (when (= (aref rows3 (* 3 i)) 1)
                (let ((zn 0.0d0) (zt1 0.0d0) (zt2 0.0d0))
                  (declare (type double-float zn zt1 zt2))
                  (loop for c of-type fixnum from 0 below 3 do
                    (setf zn (+ zn (* (aref q (+ (* 9 i) c)) (aref zfull (+ (* 3 i) c))))
                          zt1 (+ zt1 (* (aref q (+ (* 9 i) 3 c)) (aref zfull (+ (* 3 i) c))))
                          zt2 (+ zt2 (* (aref q (+ (* 9 i) 6 c)) (aref zfull (+ (* 3 i) c))))))
                  (cond
                    ((< zn 0.0d0)
                     (loop for c of-type fixnum from 0 below 3 do
                       (setf (aref rows3 (+ (* 3 i) c)) 0 (aref fk (+ (* 3 i) c)) 0.0d0))
                     (setf changed t))
                    ((and (= (aref rows3 (+ (* 3 i) 1)) 1) (> (max (abs zt1) (abs zt2)) (* mu zn)))
                     (let* ((lim (* mu zn))
                            (tb1 (max (- lim) (min lim zt1)))
                            (tb2 (max (- lim) (min lim zt2))))
                       (declare (type double-float lim tb1 tb2))
                       (loop for c of-type fixnum from 0 below 3 do
                         (setf (aref fk (+ (* 3 i) c)) (+ (* (aref q (+ (* 9 i) 3 c)) tb1) (* (aref q (+ (* 9 i) 6 c)) tb2))))
                       (setf (aref rows3 (+ (* 3 i) 1)) 0 (aref rows3 (+ (* 3 i) 2)) 0 changed t)))))))
            (unless changed (return))))
        (loop for rw of-type fixnum from 0 below k3 do (setf (aref z rw) (aref zfull rw)))
        (values x qdd-p z)))))
