;; =============================================================================
;; FILE: substrate_overlay.lisp
;; DOMAIN: The universal deformable overlay terrestrial_locomotion_patches.lisp uses,
;;         no text defining it: the deformable groundwork baseline as one modifier
;; REQUIRES: deformable_groundwork_specializers.lisp
;; =============================================================================
(defpackage :ortho-dsp.substrate-overlay
  (:use :ortho-dsp.core
        :ortho-dsp.deformable-core
        :ortho-dsp.deformable-xforms)
  (:export #:apply-universal-deformable-overlay))
(in-package :ortho-dsp.substrate-overlay)

(defmodifier apply-universal-deformable-overlay [state]
  (-> state
      (xform-deformable-groundwork-base)))
