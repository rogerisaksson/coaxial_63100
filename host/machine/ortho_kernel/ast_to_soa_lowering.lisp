;; -----------------------------------------------------------------------------
;; MODULE 2.7: ast_to_soa_lowering
;; -----------------------------------------------------------------------------
(defpackage :ortho-dsp.lowering
  (:use :cl :ortho-dsp.core)
  (:export #:compile-to-soa
           #:precompute-tustin-biquad
           #:seed-bumpless-biquad-states))
(in-package :ortho-dsp.lowering)

(defparameter *abi-field-descriptors*
  '((:bpm                         #x0000 0.016666668  0.0     4.0)
    (:stance-duty-pwm             #x0004 1.0          0.20    1.0)
    (:sole-stack-z-mm             #x0040 0.001        0.0     0.250)
    (:track-width-mm              #x0080 0.001        0.040   0.450)
    (:stride-length-mm            #x0084 0.001        0.0     1.600)
    (:swing-clearance-z-mm        #x0088 0.001        0.010   0.350)
    (:dc-offset-rad               #x0100 1.0         -1.5708  1.5708)
    (:barrier-gamma               #x0200 1.0          1.0    25.0)
    (:min-capture-margin-mm       #x0204 0.001        0.005   0.120)
    (:ecbf-pullback-stiffness-eta #x0240 1.0          0.0    50.0)
    (:min-shin-clearance-mm       #x0244 0.001        0.005   0.150)))

(defun precompute-tustin-biquad (filter-type cutoff-hz q-factor &optional (sample-rate-hz 1000.0))
  (let* ((nyquist-clamped-fc (min (max cutoff-hz 0.5) (* 0.45 sample-rate-hz)))
         (k    (tan (/ (* pi nyquist-clamped-fc) sample-rate-hz)))
         (k2   (* k k))
         (norm (/ 1.0 (+ 1.0 (/ k q-factor) k2))))
    (ecase filter-type
      (:lowpass
        (let ((b0 (* k2 norm)))
          (values b0 (* 2.0 b0) b0 (* 2.0 (- k2 1.0) norm) (* (+ 1.0 (- (/ k q-factor)) k2) norm))))
      (:notch
        (let ((b0 (* (+ 1.0 k2) norm))
              (b1 (* 2.0 (- k2 1.0) norm)))
          (values b0 b1 b0 b1 (* (+ 1.0 (- (/ k q-factor)) k2) norm)))))))

(defun seed-bumpless-biquad-states (b0 b1 b2 a1 a2 live-signal-x0)
  (let* ((dc-gain (/ (+ b0 b1 b2) (+ 1.0 a1 a2)))
         (y0      (* dc-gain live-signal-x0))
         (s2      (- (* b2 live-signal-x0) (* a2 y0)))
         (s1      (+ s2 (- (* b1 live-signal-x0) (* a1 y0)))))
    (values (coerce s1 'single-float) (coerce s2 'single-float))))

(defun compile-to-soa (&key ast (align-bytes 64) (sample-rate-hz 1000.0) target-bank live-telemetry schedule)
  (declare (ignore align-bytes))
  (let ((mmap-ptr (acquire-shadow-bank-pointer target-bank))
        (root-tree (or ast schedule)))
    (memcpy-aligned-64 mmap-ptr *nominal-soa-template* +soa-bank-byte-size+)
    (labels ((lower-node (sexp active-joint-index)
               (when (consp sexp)
                 (case (car sexp)
                   ((modulate-node dc-bias shape-transients)
                    (let ((joint-idx (resolve-joint-soa-index (second sexp))))
                      (dolist (sub (cddr sexp)) (lower-node sub joint-idx))))
                   ((butterworth-lpf configure-contact-compliance)
                    (let* ((fc   (or (getf (cdr sexp) :cutoff-hz) (getf (cdr sexp) :biquad-notch-hz) 42.0))
                           (q    (or (getf (cdr sexp) :q-factor) 0.70710678))
                           (mode (if (eq (car sexp) 'butterworth-lpf) :lowpass :notch))
                           (x0   (read-live-joint-state live-telemetry active-joint-index)))
                      (multiple-value-bind (b0 b1 b2 a1 a2)
                          (precompute-tustin-biquad mode fc q sample-rate-hz)
                        (multiple-value-bind (s1 s2)
                            (seed-bumpless-biquad-states b0 b1 b2 a1 a2 x0)
                          (write-soa-biquad-slot mmap-ptr active-joint-index b0 b1 b2 a1 a2 s1 s2)))))
                   (otherwise
                    (loop for (k v) on sexp by #'cddr
                          when (and (keywordp k) (numberp v))
                          do (let ((desc (assoc k *abi-field-descriptors*)))
                               (when desc
                                 (destructuring-bind (_ offset si-scale lo hi) desc
                                   (declare (ignore _))
                                   (let ((si-val (min hi (max lo (* (float v 1.0) si-scale)))))
                                     (write-f32-aligned mmap-ptr (+ offset (* 4 active-joint-index)) si-val))))))
                    (dolist (sub sexp) (when (consp sub) (lower-node sub active-joint-index))))))))
      (lower-node root-tree 0))
    (memory-ordering-fence :release)
    mmap-ptr))
