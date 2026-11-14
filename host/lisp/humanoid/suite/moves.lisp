;; =============================================================================
;; FILE: moves.lisp
;; DOMAIN: Its Moves on Its Anatomy: a Move's Phases over Its Parts and the Numbers They Take
;; =============================================================================
(defpackage :ortho-dsp.moves
  (:use :ortho-dsp.core)
  (:export #:bed-exit
           #:bed-entry
           #:chair-mount
           #:chair-dismount
           #:scoot))
(in-package :ortho-dsp.moves)

;; A move (`defmove`) is a function of its body's numbers to its table: its numbers (`:constants`) and its phases,
;; each its name, its seconds (none: it never ends by the clock), the phase after it, each part's goal (deg) and how
;; it goes - `:blend` over the phase from where it began, `:follow` its sensed angle and the goal past it, `:toward`
;; drawn toward its goal from its sensed angle by the pull at most, else its rate (deg/s) -, past an edge its parts'
;; own (`:over`), the trunk's pitch its hips hold (deg, forward +) or none. Its joints play it on its anatomy
;; (machine.moves.anatomy, wbc_anatomy.c) as the loop's moves (machine.process.joints.MOVES).

;; Its way out of a bed over its end (machine.moves.bedexit, wbc_bedexit.c): sat up on its arms, scooted to the
;; edge, seated there.
(defmove bed-exit []
  (let [;; a stroke: the shoulders from set to push (deg) as the knees pull from stretched to pulled; its phases' s
        set -10.0 push -40.0 stretched 10.0 pulled 60.0 push-s 0.8 stretch-s 0.4 slide-s 0.6
        ;; its trunk's pitch its hips hold stroking, scooting and seated, deg (forward +, its law's modal.pitched);
        ;; their gain, deg/s a deg, and range, deg (scooting they fold past 100: held there it rose and fell back)
        trunk -20.0 scoot-trunk 15.0 seated-trunk -5.0 trunk-gain 4.0 hip-lo -130.0 hip-hi -40.0
        ;; past the edge: its shanks' share of its weight under which they hang; its knees drawing its buttocks to
        ;; its feet (tuck, deg); a pull's most past where they are (`:toward`), deg; seated: its soles' share and its
        ;; knees' fold, deg; the strokes at most
        shin-share 0.02 tuck 105.0 pull 8.0 sole-share 0.25 knee-seat 100.0 strokes 30.0
        ;; seated, its knees and ankles, deg, scooted to on its heels at the frame's face (the low chair's seat, 104
        ;; and -20, sits its hips 0.26 m behind its ankles); its legs settling and the scoot, s. Its trunk 20 deg
        ;; forward stood it on its toes and threw it back onto the bed, 25 slid it off its end; its arms brought
        ;; forward as it scooted, 3 of 7 starts fell
        seated-knee 108.0 seated-ankle -28.0 settle-s 0.5 scoot-s 1.5
        ;; past the edge its knees fold to tuck and its ankles to 0 at the push's rates
        tucked (:knee [tuck 120.0] :ankle [0.0 60.0])]
    (:constants (:trunk-gain trunk-gain :hip-lo hip-lo :hip-hi hip-hi :shin-share shin-share :pull pull
                 :sole-share sole-share :knee-seat knee-seat :strokes strokes)
     ;; the sit-up: the elbows under the shoulders, legs straight; the arms lifting it as the hips fold. Its hands
     ;; set by its hips. A stroke: the push (to the settle once it is seated), the legs stretched, the arms forward
     ;; again. The robot's legs settling, its heels down the frame's face; the scoot. Its law's seat
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

;; Its way into a bed over its end to its middle, told by where it lies on it - a move whose phases end by what it
;; senses (machine.moves.sensed, wbc_sensed.c): each phase ends by what it senses (`:ends` - :reached its pose, :feet
;; planted or that, :pulled its pelvis in or that and it still, :lying that and past its pitch, :stilled that and
;; still, :middle at the middle or that), none by the clock; each part's goal (deg) and rate (deg/s), `:least` no
;; straighter than it is, a phase's shared goals first (`:with`); the phase after it and the one while a stroke is due
;; (`:again`), the phase that counts a stroke (`:counts`) - the puppet's program's poses (machine.process.joints'
;; PROGRAMS), its heels drawn in from its stretch at its first stroke's pace. Its text goes on under it (`:holds`): its
;; flat stage runs under it so its rest limps it on the foam as it ends (held back, its rise after it was never said).
(defmove bed-entry []
  (let [;; its feet's share of its weight planted; its pelvis pulled in from the bed's end (m); still under its speed
        ;; (m/s: lying still 0.000-0.003, its pull 0.3-0.6); lying past its pitch (deg: flat -97 to -102); a stroke
        ;; again while this short of the middle (m), the strokes at most; a goal reached within tol deg
        plant 0.05 pull-in 0.30 still 0.01 lain -90.0 near 0.03 strokes 12.0 tol 1e-6
        ;; its trunk straightened; its legs square, their hips' roll and yaw at zero
        trunk (:spine [0.0 20.0] :waist [0.0 20.0] :neck [0.0 20.0])
        square (:hip-roll [0.0 15.0] :hip-yaw [0.0 15.0])]
    (:constants (:plant plant :pull-in pull-in :still still :lain lain :near near :strokes strokes :tol tol)
     :holds false
     :phases [(:name :feet :ends :feet :next :pull :again :pull :goals (:knee [95.0 9.0] :ankle [0.0 30.0]))
              (:name :pull :ends :pulled :next :lie :again :lie
               :goals (:shoulder [30.0 83.0] :elbow [0.0 30.0] :hip [-75.0 22.0] :knee [30.0 114.0] :ankle [0.0 46.0]))
              (:name :lie :ends :lying :next :up :again :up
               :goals (:shoulder [0.0 23.0] :elbow [5.0 4.0] :hip [-10.0 50.0] :knee [15.0 12.0]))
              (:name :up :ends :reached :next :fold :again :fold :goals (:hip [-75.0 108.0] :knee [6.0 15.0]))
              (:name :fold :ends :reached :next :lower :again :lower :goals (:knee [150.0 288.0]))
              (:name :lower :ends :reached :next :shove :again :shove :goals (:hip [-60.0 50.0]))
              (:name :shove :ends :reached :next :stretch :again :stretch :goals (:hip [-35.0 42.0] :knee [100.0 83.0]))
              (:name :stretch :ends :stilled :next :straight :again :draw :with [trunk]
               :goals (:knee [6.0 313.0] :hip [-3.0 107.0]))
              (:name :draw :ends :reached :next :bridge :again :bridge :with [square]
               :goals (:hip [-56.0 133.0] :knee [146.0 350.0]))
              (:name :plant :ends :reached :next :bridge :again :bridge :with [square]
               :goals (:hip [-56.0 25.0] :knee [146.0 95.0]))
              (:name :bridge :ends :reached :next :push :again :push :goals (:hip [-34.0 88.0] :knee [134.0 48.0]))
              (:name :push :ends :middle :next :set :again :set :goals (:hip [-32.0 7.0] :knee [104.0 100.0]))
              (:name :set :ends :stilled :next :straight :again :plant :counts true
               :goals (:hip [-46.0 70.0] :knee [108.0 20.0 :least]))
              (:name :straight :ends :reached :next :done :again :done :with [trunk square]
               :goals (:knee [6.0 255.0] :hip [-3.0 108.0]))
              (:name :done :ends :reached :next :done :again :done)])))

;; Its way up onto a seat too high for its feet, from its front edge, its fists on it beside its hips - the kernel's arm
;; vault (`xform-overheight-arm-vault-ingress`), a move whose phases end by what it senses as its bed entry's: each
;; phase ends by what it senses (`:ends` - :reached its pose, :stilled that and it still); each part's goal (deg) and
;; rate (deg/s), a puppet's program's poses at its pace from its sit (a part's larger side's change over its beat's
;; seconds: the vault 0.65 s, the hang 3). Its text's rows wait while it plays (`:holds`): its seat's stage then gives
;; its law where it sits.
(defmove chair-mount []
  (let [;; still under its speed (m/s); a goal reached within tol deg
        still 0.01 tol 1e-6]
    (:constants (:still still :tol tol)
     :holds true
     ;; the vault: its shoulders flexed against its fists, its elbows straight, its hips and knees folded - its pelvis
     ;; lifted off the edge and drawn back on its arms (0.152 m ahead of the seat's middle to 0.06, 7 cm up); the hang:
     ;; its shoulders let it down onto the seat (0.05 m behind its middle, its trunk upright, no fall)
     :phases [(:name :vault :ends :reached :next :hang
               :goals (:shoulder [30.0 89.0] :elbow [0.0 74.0] :hip [-90.0 11.0] :knee [90.0 15.0]))
              (:name :hang :ends :stilled :next :done :goals (:shoulder [0.0 10.0] :elbow [5.0 1.7]))
              (:name :done :ends :reached :next :done)])))

;; Its way down off that seat onto its feet, as its mount came up onto it - the kernel's arm-dip egress
;; (`xform-overheight-arm-dip-egress`), a move whose phases end by what it senses as its mount's. Its slide on its
;; fists asked of its law alone moved its none (its elbows kept bent, its fists unloaded 68 -> 22 N in 0.7 s), and
;; vaulted from where its mount left them its arms were already straight (its elbows 12 deg): its shoulders drew back
;; into its elbows, which folded to 30, and nothing lifted it. So as its mount starts it: its fists raised 5 cm and
;; set beside its hips, its elbows bent as its arm's links reach the seat there (`seated.fisted`: shoulder -35.5 elbow
;; 63.9 over it), pressed down onto it (-23.3 and 41.2 on it, the kernel's dip suspension's -26 and 43); then its
;; elbows braced to their stop (the scoot's `locked`) lift it and carry its shoulders forward over its fists - at 30
;; deg/s, the kernel's tangent speed on its arm's reach - and its shoulders drawn back after them at 14 (0.15 m/s on
;; 0.61 m), its hips and knees held as it sits, till its soles bear it as its rise unlatches its seat (`:feet` at
;; the concentric handover's 0.82): its palms bore up to 284 N, its pelvis 0.18 m forward, its soles 0.96 of it.
;; The robot's text's rows wait while it plays.
(defmove chair-dismount []
  (let [;; its soles' share planted, as its rise unlatches its seat; a goal reached within tol deg; its legs squared
        plant 0.82 tol 1e-6
        square (:hip-roll [0.0 30.0] :hip-yaw [0.0 30.0] :ankle-roll [0.0 30.0])]
    (:constants (:plant plant :tol tol)
     :holds true
     :phases [(:name :place :ends :reached :next :press :with [square]
               :goals (:shoulder [-35.5 60.0] :elbow [63.9 120.0] :hip [-90.0 11.0] :knee [90.0 15.0]))
              (:name :press :ends :reached :next :vault :goals (:shoulder [-23.3 30.0] :elbow [41.2 30.0]))
              (:name :vault :ends :feet :next :done :goals (:shoulder [-45.0 14.0] :elbow [-5.0 30.0] :hip [-102.0 20.0]))
              (:name :done :ends :reached :next :done)])))

;; Its scoot on the floor as its twist tells it (machine.moves.scoot, wbc_scoot.c): a crab on its seat, heels in turn,
;; then a stroke. Each part's how (:rate its goal at its rate, :end the stroke's end at its way's rate, :start the
;; stroke's start - a hip by its knee's way -, :setting its goal while its foot touches none, :even its pair's mean at
;; its rate - a hip holding its trunk as well, its stance square as it sits -, :unseat its goal while its trunk is
;; short of leant back of its lean), goal (deg; `:seat` its shoulders braced at its lean, seated.braced) and rate; each
;; phase over (`:ends` - :reached its goals, :braced those, its fists touching and its trunk at its lean, :set its heel
;; touching or its goals, :stroked its knees at their ends or its twist let go, :asking its twist asking, :off its trunk
;; leant back of its lean, :leaned its trunk within leant of it), the next asked (`:asked`) and idle (`:idle`).
(defmove scoot []
  (let [;; a stroke's knees, deg: folded as the crawl take's most (126), stretched where its hips have gone the
        ;; stroke's length (its text's 0.25 m: 78 deg). A hip lifts its foot to folded as its knee folds to a stroke back
        ;; (folded on the floor, its heels dragged its 0.12-0.21 m forward); stretching to a stroke forward it lifts only
        ;; while its foot touches (lifted to -100, it threw it back); a still side's foot stays down. A hip sets its
        ;; foot toward down till it touches; its trunk within leant of its lean as it braces and once a stroke is over
        fold 126.0 folded -112.0 down -40.0 leant 6.0
        ;; its elbows braced at their stop, deg (their range's -5): locked past straight, a load along its arm presses
        ;; them into it (straight at 0, they gave 17-73 deg as it went back over its fists and it fell; bent 75 deg to
        ;; sit it, they folded under it as a heel lifted and it fell back)
        locked -5.0
        ;; its trunk's pitch its hips hold, deg (forward +): the crawl take's lean, 47 deg off the floor (-40, 35 off
        ;; it, laid it down as its legs lifted); their gain, deg/s a deg, and range, deg (its bed exit's, -130: at -115
        ;; its hips could flex no further and it fell back). A hip holds its trunk only while a foot bears: sat, it
        ;; lifts its legs instead (a set foot rose off the floor again, -84 to -103 deg, and W pulled on one heel;
        ;; pulling, W went 0.03-0.27 m in 8 s)
        trunk -28.0 gain 12.0 hip-lo -130.0 hip-hi -35.0
        ;; its shoulders sat on straight arms, its fists on the floor behind it (`:seat`, as its text's sit braces it:
        ;; its seat bore 0.85 of it); vaulted, its straight arms its struts as its hips open it off its seat till its
        ;; trunk leans leant back of its lean (its pelvis 0.23 m up, as HEAD's set rose it: its shoulders alone, at
        ;; their 40 N m, held -56 to -67 deg and its seat bore 0.39-0.90 of it; opened till its arms stood up, its trunk
        ;; went back to -46 deg and its pelvis to 0.31 m, its arms never)
        vaulted -45.0
        ;; rates, deg/s: its limbs in the air, its shoulders, its hips lifting and setting its feet, its squared joints,
        ;; its legs evened
        air-rate 300.0 brace-rate 90.0 lift-rate 120.0 set-rate 60.0 square-rate 120.0 even-rate 30.0
        ;; a foot bears once it takes bear of its weight and touches at skim; a twist asks past ask; a goal is reached
        ;; within tol deg
        bear 0.1 skim 0.02 ask 0.01 tol 1e-6
        ;; a heading's error, deg, a whole way's share of a side (W's pulls turned its 10-12 deg each); a side's share of
        ;; a turn at most turn of its way, its aim a turn's lead ahead as it turns. Ahead or where it sits a side only
        ;; pulls, the inner one still (a part pull turns it as a whole one: at a share of 0.57 W drifted 22 deg); going
        ;; back the inner side keeps half its back (still, its leg anchored it: S+A 0.03 m in 8 s)
        stroke-turn 10.0 turn 1.0 half 0.5
        ;; its legs' rolls and yaws and its trunk's roll and twist, squared in every phase
        square (:hip-roll [:rate 0.0 square-rate] :hip-yaw [:rate 0.0 square-rate] :ankle-roll [:rate 0.0 square-rate]
                :spine-roll [:rate 0.0 square-rate] :waist [:rate 0.0 square-rate])
        even (:hip [:even 0.0 even-rate] :knee [:rate fold even-rate] :ankle [:even 0.0 even-rate])
        sat (:shoulder [:rate :seat brace-rate] :elbow [:rate locked air-rate])
        lift (:hip [:start 0.0 lift-rate] :knee [:start 0.0 air-rate])]
    (:constants (:fold fold :folded folded :down down :leant leant :locked locked :trunk trunk :gain gain
                 :hip-lo hip-lo :hip-hi hip-hi :vaulted vaulted :air-rate air-rate :brace-rate brace-rate
                 :lift-rate lift-rate :set-rate set-rate :square-rate square-rate :even-rate even-rate :bear bear
                 :skim skim :ask ask :tol tol :stroke-turn stroke-turn :turn turn :half half)
     ;; a hip not moving holds its trunk. It sits as both its feet lift (as HEAD's lift sat it: none tipped in 10,
     ;; its shoulders swung back as its feet lifted, 1 of 2); a step (lift, set) moves its heel only, every step taken
     ;; once it sits; past the last its arms reach back to its seat's brace (dragged round over its fists, A tipped it
     ;; at 43 deg), or it holds where its twist let go; its vault where both heels push
     :phases [(:name :brace :ends :braced :asked :sit :idle :hold :with [sat even square])
              (:name :hold :ends :asking :asked :sit :idle :hold :with [sat even square])
              (:name :sit :ends :reached :asked :lift :idle :lift :with [lift square])
              (:name :lift :ends :reached :asked :set :idle :set :with [lift square])
              (:name :set :ends :set :asked :lift :idle :lift :with [square] :goals (:hip [:setting down set-rate]))
              (:name :reach :ends :reached :asked :vault :idle :hold :with [sat square])
              (:name :vault :ends :off :asked :push :idle :hold :with [square]
               :goals (:hip [:unseat hip-hi set-rate] :shoulder [:rate vaulted brace-rate] :elbow [:rate locked air-rate]))
              (:name :push :ends :stroked :asked :lean :idle :lean :with [square] :goals (:knee [:end 0.0 0.0]))
              (:name :lean :ends :leaned :asked :sit :idle :sit :with [square])])))
