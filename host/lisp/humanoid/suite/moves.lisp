;; =============================================================================
;; FILE: moves.lisp
;; DOMAIN: Her Moves on Her Anatomy: a Move's Phases over Her Parts and the Numbers They Take
;; =============================================================================
(defpackage :ortho-dsp.moves
  (:use :ortho-dsp.core)
  (:export #:bed-exit))
(in-package :ortho-dsp.moves)

;; A move (`defmove`) is a function of her body's numbers to its table: its numbers (`:constants`) and its phases,
;; each its name, its seconds (none: it never ends by the clock), the phase after it, each part's goal (deg) and how
;; it goes - `:blend` over the phase from where it began, `:follow` her sensed angle and the goal past it, `:toward`
;; drawn toward its goal from her sensed angle by the pull at most, else its rate (deg/s) -, past an edge its parts'
;; own (`:over`), the trunk's pitch her hips hold (deg, forward +) or none. Her joints play it on her anatomy
;; (machine.moves.anatomy, wbc_anatomy.c) as the loop's moves (machine.process.joints.MOVES).

;; Her way out of a bed over its end (machine.moves.bedexit, wbc_bedexit.c): sat up on her arms, scooted to the
;; edge, seated there.
(defmove bed-exit []
  (let [;; a stroke: the shoulders from set to push (deg) as the knees pull from stretched to pulled; its phases' s
        set -10.0 push -40.0 stretched 10.0 pulled 60.0 push-s 0.8 stretch-s 0.4 slide-s 0.6
        ;; her trunk's pitch her hips hold stroking, scooting and seated, deg (forward +, her law's modal.pitched);
        ;; their gain, deg/s a deg, and range, deg (scooting they fold past 100: held there she rose and fell back)
        trunk -20.0 scoot-trunk 15.0 seated-trunk -5.0 trunk-gain 4.0 hip-lo -130.0 hip-hi -40.0
        ;; past the edge: her shanks' share of her weight under which they hang; her knees drawing her buttocks to
        ;; her feet (tuck, deg); a pull's most past where they are (`:toward`), deg; seated: her soles' share and her
        ;; knees' fold, deg; the strokes at most
        shin-share 0.02 tuck 105.0 pull 8.0 sole-share 0.25 knee-seat 100.0 strokes 30.0
        ;; seated, her knees and ankles, deg, scooted to on her heels at the frame's face (the low chair's seat, 104
        ;; and -20, sits her hips 0.26 m behind her ankles); her legs settling and the scoot, s. Her trunk 20 deg
        ;; forward stood her on her toes and threw her back onto the bed, 25 slid her off its end; her arms brought
        ;; forward as she scooted, 3 of 7 starts fell
        seated-knee 108.0 seated-ankle -28.0 settle-s 0.5 scoot-s 1.5
        ;; past the edge her knees fold to tuck and her ankles to 0 at the push's rates
        tucked (:knee [tuck 120.0] :ankle [0.0 60.0])]
    (:constants (:trunk-gain trunk-gain :hip-lo hip-lo :hip-hi hip-hi :shin-share shin-share :pull pull
                 :sole-share sole-share :knee-seat knee-seat :strokes strokes)
     ;; the sit-up: the elbows under the shoulders, legs straight; the arms lifting her as the hips fold. Her hands
     ;; set by her hips. A stroke: the push (to the settle once she is seated), the legs stretched, the arms forward
     ;; again. Her legs settling, her heels down the frame's face; the scoot. Her law's seat
     :phases [(:name :under :seconds 1.5 :next :up
               :goals (:hip [0.0 :blend] :knee [5.0 :blend] :shoulder [-90.0 :blend] :elbow [90.0 :blend]))
              (:name :up :seconds 1.5 :next :place
               :goals (:hip [-75.0 :blend] :shoulder [-120.0 :blend] :elbow [30.0 :blend]))
              (:name :place :seconds 1.0 :next :push :goals (:shoulder [set :blend] :elbow [0.0 :blend]) :trunk trunk)
              (:name :push :seconds push-s :next :stretch
               :goals (:shoulder [push (/ (abs (- push set)) (- push-s 0.1))] :knee [pulled 120.0] :elbow [0.0 200.0]
                       :ankle [0.0 60.0])
               :over tucked :trunk trunk)
              (:name :stretch :seconds stretch-s :next :slide :goals (:knee [stretched 150.0] :ankle [0.0 80.0])
               :over tucked :trunk trunk)
              (:name :slide :seconds slide-s :next :push
               :goals (:shoulder [set (/ (abs (- push set)) (- slide-s 0.1))])
               :over tucked :trunk trunk)
              (:name :settle :seconds settle-s :next :scoot :goals tucked :trunk trunk)
              (:name :scoot :seconds scoot-s :next :seat :goals (:knee [seated-knee :blend] :ankle [seated-ankle :blend])
               :trunk scoot-trunk)
              (:name :seat :seconds 1.0 :next :done
               :goals (:knee [seated-knee :blend] :ankle [seated-ankle :blend] :shoulder [25.0 :blend]
                       :elbow [19.0 :blend] :spine [-30.0 :blend])
               :trunk seated-trunk)
              (:name :done :next :done)])))
