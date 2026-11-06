;; =============================================================================
;; FILE: closed_chain_release.lisp
;; DOMAIN: The Closed Chains' Release - the Plan's Own Forces Round by Round: Free, Yielding, Sliding
;; REQUIRES: core_biomechanics.lisp
;; Hers: `machine.stack.chain.planned` releases its contacts so each round and `wbc_chain_plan.c` bit for bit; the
;; plan (closed_chain_plan.lisp) calls it. Written out on flat row-major double-float vectors.
;; =============================================================================
(defpackage :ortho-dsp.closed-chain-release
  (:use :ortho-dsp.core)
  (:export #:closed-chain-release))
(in-package :ortho-dsp.closed-chain-release)

;; The plan's forces at each held contact, in its frame (normal, two tangents): zfull with her posture in the tasks'
;; null space, z1 her tasks' alone (x1). A contact pulled on goes free - unless her tasks press it and her posture
;; alone pulls: then it yields, bearing what it feels, a row beside the tasks from the next round on (on the floor
;; her rest's posture pulled her fists and feet free, -1 to -15 N, and their 88 N folded her elbows). Only a hand or a
;; let-go foot yields: what she sits or lies on and a standing sole (`chain.bearing`'s) go free - held, her rise off
;; the chair stalled in its thrust and her sit-up off the bed fell short. One dragged past its box |z_t| <= mu z_n
;; slides, its friction the box's.
(defun closed-chain-release (q zfull z1 rows3 fk yields mu k3)
  "t where a contact changed: rows3 (3k) the rows held, fk (3k) the forces known, yields (k) the yielding contacts,
   each in place; q (9k) the contacts' frames (contact-frame), zfull and z1 (3k) the plan's forces, world."
  (declare (type (simple-array double-float (*)) q)
           (type (array double-float (*)) zfull z1 fk)
           (type (simple-array fixnum (*)) rows3 yields)
           (type double-float mu)
           (type fixnum k3)
           (optimize (speed 3) (safety 0)))
  (let ((changed nil))
    (loop for i of-type fixnum from 0 below (floor k3 3) do
      (when (= (aref rows3 (* 3 i)) 1)
        (let ((zn 0.0d0) (zt1 0.0d0) (zt2 0.0d0) (z1n 0.0d0))
          (declare (type double-float zn zt1 zt2 z1n))
          (loop for c of-type fixnum from 0 below 3 do
            (setf zn (+ zn (* (aref q (+ (* 9 i) c)) (aref zfull (+ (* 3 i) c))))
                  zt1 (+ zt1 (* (aref q (+ (* 9 i) 3 c)) (aref zfull (+ (* 3 i) c))))
                  zt2 (+ zt2 (* (aref q (+ (* 9 i) 6 c)) (aref zfull (+ (* 3 i) c))))
                  z1n (+ z1n (* (aref q (+ (* 9 i) c)) (aref z1 (+ (* 3 i) c))))))
          (cond
            ((and (< zn 0.0d0) (= (aref yields i) 0) (= (aref rows3 (+ (* 3 i) 1)) 1) (>= z1n 0.0d0))
             (setf (aref yields i) 1 changed t))
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
    changed))
