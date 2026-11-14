;; =============================================================================
;; FILE: moves.lisp
;; DOMAIN: Her Moves on Her Anatomy: a Move's Phases over Her Parts and the Numbers They Take
;; =============================================================================
(defpackage :ortho-dsp.moves
  (:use :ortho-dsp.core)
  (:export #:bed-exit
           #:bed-entry
           #:chair-mount
           #:chair-dismount
           #:scoot))
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

;; Her way into a bed over its end to its middle, told by where she lies on it - a move whose phases end by what she
;; senses (machine.moves.sensed, wbc_sensed.c): each phase ends by what she senses (`:ends` - :reached its pose, :feet
;; planted or that, :pulled her pelvis in or that and she still, :lying that and past her pitch, :stilled that and
;; still, :middle at the middle or that), none by the clock; each part's goal (deg) and rate (deg/s), `:least` no
;; straighter than it is, a phase's shared goals first (`:with`); the phase after it and the one while a stroke is due
;; (`:again`), the phase that counts a stroke (`:counts`) - the puppet's program's poses (machine.process.joints'
;; PROGRAMS), its heels drawn in from her stretch at its first stroke's pace. Her text goes on under it (`:holds`): its
;; flat stage runs under it so her rest limps her on the foam as it ends (held back, her rise after it was never said).
(defmove bed-entry []
  (let [;; her feet's share of her weight planted; her pelvis pulled in from the bed's end (m); still under her speed
        ;; (m/s: lying still 0.000-0.003, her pull 0.3-0.6); lying past her pitch (deg: flat -97 to -102); a stroke
        ;; again while this short of the middle (m), the strokes at most; a goal reached within tol deg
        plant 0.05 pull-in 0.30 still 0.01 lain -90.0 near 0.03 strokes 12.0 tol 1e-6
        ;; her trunk straightened; her legs square, their hips' roll and yaw at zero
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

;; Her way up onto a seat too high for her feet, from its front edge, her fists on it beside her hips - the kernel's arm
;; vault (`xform-overheight-arm-vault-ingress`), a move whose phases end by what she senses as her bed entry's: each
;; phase ends by what she senses (`:ends` - :reached its pose, :stilled that and she still); each part's goal (deg) and
;; rate (deg/s), a puppet's program's poses at its pace from her sit (a part's larger side's change over its beat's
;; seconds: the vault 0.65 s, the hang 3). Her text's rows wait while it plays (`:holds`): her seat's stage then gives
;; her law where she sits.
(defmove chair-mount []
  (let [;; still under her speed (m/s); a goal reached within tol deg
        still 0.01 tol 1e-6]
    (:constants (:still still :tol tol)
     :holds true
     ;; the vault: her shoulders flexed against her fists, her elbows straight, her hips and knees folded - her pelvis
     ;; lifted off the edge and drawn back on her arms (0.152 m ahead of the seat's middle to 0.06, 7 cm up); the hang:
     ;; her shoulders let her down onto the seat (0.05 m behind its middle, her trunk upright, no fall)
     :phases [(:name :vault :ends :reached :next :hang
               :goals (:shoulder [30.0 89.0] :elbow [0.0 74.0] :hip [-90.0 11.0] :knee [90.0 15.0]))
              (:name :hang :ends :stilled :next :done :goals (:shoulder [0.0 10.0] :elbow [5.0 1.7]))
              (:name :done :ends :reached :next :done)])))

;; Her way down off that seat onto her feet, as her mount came up onto it - the kernel's arm-dip egress
;; (`xform-overheight-arm-dip-egress`), a move whose phases end by what she senses as her mount's. Her slide on her
;; fists asked of her law alone moved her none (her elbows kept bent, her fists unloaded 68 -> 22 N in 0.7 s), and
;; vaulted from where her mount left them her arms were already straight (her elbows 12 deg): her shoulders drew back
;; into her elbows, which folded to 30, and nothing lifted her. So as her mount starts it: her fists raised 5 cm and
;; set beside her hips, her elbows bent as her arm's links reach the seat there (`seated.fisted`: shoulder -35.5 elbow
;; 63.9 over it), pressed down onto it (-23.3 and 41.2 on it, the kernel's dip suspension's -26 and 43); then her
;; elbows braced to their stop (the scoot's `locked`) lift her and carry her shoulders forward over her fists - at 30
;; deg/s, the kernel's tangent speed on her arm's reach - and her shoulders drawn back after them at 14 (0.15 m/s on
;; 0.61 m), her hips and knees held as she sits, till her soles bear her as her rise unlatches its seat (`:feet` at
;; the concentric handover's 0.82): her palms bore up to 284 N, her pelvis 0.18 m forward, her soles 0.96 of her.
;; Her text's rows wait while it plays.
(defmove chair-dismount []
  (let [;; her soles' share planted, as her rise unlatches its seat; a goal reached within tol deg; her legs squared
        plant 0.82 tol 1e-6
        square (:hip-roll [0.0 30.0] :hip-yaw [0.0 30.0] :ankle-roll [0.0 30.0])]
    (:constants (:plant plant :tol tol)
     :holds true
     :phases [(:name :place :ends :reached :next :press :with [square]
               :goals (:shoulder [-35.5 60.0] :elbow [63.9 120.0] :hip [-90.0 11.0] :knee [90.0 15.0]))
              (:name :press :ends :reached :next :vault :goals (:shoulder [-23.3 30.0] :elbow [41.2 30.0]))
              (:name :vault :ends :feet :next :done :goals (:shoulder [-45.0 14.0] :elbow [-5.0 30.0] :hip [-102.0 20.0]))
              (:name :done :ends :reached :next :done)])))

;; Her scoot on the floor as her twist tells it (machine.moves.scoot, wbc_scoot.c): a crab on her seat, heels in turn,
;; then a stroke. Each part's how (:rate its goal at its rate, :end the stroke's end at her way's rate, :start the
;; stroke's start - a hip by its knee's way -, :setting its goal while its foot touches none, :even its pair's mean at
;; its rate - a hip holding her trunk as well, her stance square as she sits -, :unseat its goal while her trunk is
;; short of leant back of its lean), goal (deg; `:seat` her shoulders braced at her lean, seated.braced) and rate; each
;; phase over (`:ends` - :reached its goals, :braced those, her fists touching and her trunk at its lean, :set its heel
;; touching or its goals, :stroked her knees at their ends or her twist let go, :asking her twist asking, :off her trunk
;; leant back of its lean, :leaned her trunk within leant of it), the next asked (`:asked`) and idle (`:idle`).
(defmove scoot []
  (let [;; a stroke's knees, deg: folded as the crawl take's most (126), stretched where her hips have gone the
        ;; stroke's length (her text's 0.25 m: 78 deg). A hip lifts its foot to folded as its knee folds to a stroke back
        ;; (folded on the floor, her heels dragged her 0.12-0.21 m forward); stretching to a stroke forward it lifts only
        ;; while its foot touches (lifted to -100, it threw her back); a still side's foot stays down. A hip sets its
        ;; foot toward down till it touches; her trunk within leant of its lean as she braces and once a stroke is over
        fold 126.0 folded -112.0 down -40.0 leant 6.0
        ;; her elbows braced at their stop, deg (their range's -5): locked past straight, a load along her arm presses
        ;; them into it (straight at 0, they gave 17-73 deg as she went back over her fists and she fell; bent 75 deg to
        ;; sit her, they folded under her as a heel lifted and she fell back)
        locked -5.0
        ;; her trunk's pitch her hips hold, deg (forward +): the crawl take's lean, 47 deg off the floor (-40, 35 off
        ;; it, laid her down as her legs lifted); their gain, deg/s a deg, and range, deg (her bed exit's, -130: at -115
        ;; her hips could flex no further and she fell back). A hip holds her trunk only while a foot bears: sat, it
        ;; lifts her legs instead (a set foot rose off the floor again, -84 to -103 deg, and W pulled on one heel;
        ;; pulling, W went 0.03-0.27 m in 8 s)
        trunk -28.0 gain 12.0 hip-lo -130.0 hip-hi -35.0
        ;; her shoulders sat on straight arms, her fists on the floor behind her (`:seat`, as her text's sit braces her:
        ;; her seat bore 0.85 of her); vaulted, her straight arms her struts as her hips open her off her seat till her
        ;; trunk leans leant back of its lean (her pelvis 0.23 m up, as HEAD's set rose her: her shoulders alone, at
        ;; their 40 N m, held -56 to -67 deg and her seat bore 0.39-0.90 of her; opened till her arms stood up, her trunk
        ;; went back to -46 deg and her pelvis to 0.31 m, her arms never)
        vaulted -45.0
        ;; rates, deg/s: her limbs in the air, her shoulders, her hips lifting and setting her feet, her squared joints,
        ;; her legs evened
        air-rate 300.0 brace-rate 90.0 lift-rate 120.0 set-rate 60.0 square-rate 120.0 even-rate 30.0
        ;; a foot bears once it takes bear of her weight and touches at skim; a twist asks past ask; a goal is reached
        ;; within tol deg
        bear 0.1 skim 0.02 ask 0.01 tol 1e-6
        ;; a heading's error, deg, a whole way's share of a side (W's pulls turned her 10-12 deg each); a side's share of
        ;; a turn at most turn of its way, her aim a turn's lead ahead as she turns. Ahead or where she sits a side only
        ;; pulls, the inner one still (a part pull turns her as a whole one: at a share of 0.57 W drifted 22 deg); going
        ;; back the inner side keeps half her back (still, its leg anchored her: S+A 0.03 m in 8 s)
        stroke-turn 10.0 turn 1.0 half 0.5
        ;; her legs' rolls and yaws and her trunk's roll and twist, squared in every phase
        square (:hip-roll [:rate 0.0 square-rate] :hip-yaw [:rate 0.0 square-rate] :ankle-roll [:rate 0.0 square-rate]
                :spine-roll [:rate 0.0 square-rate] :waist [:rate 0.0 square-rate])
        even (:hip [:even 0.0 even-rate] :knee [:rate fold even-rate] :ankle [:even 0.0 even-rate])
        sat (:shoulder [:rate :seat brace-rate] :elbow [:rate locked air-rate])
        lift (:hip [:start 0.0 lift-rate] :knee [:start 0.0 air-rate])]
    (:constants (:fold fold :folded folded :down down :leant leant :locked locked :trunk trunk :gain gain
                 :hip-lo hip-lo :hip-hi hip-hi :vaulted vaulted :air-rate air-rate :brace-rate brace-rate
                 :lift-rate lift-rate :set-rate set-rate :square-rate square-rate :even-rate even-rate :bear bear
                 :skim skim :ask ask :tol tol :stroke-turn stroke-turn :turn turn :half half)
     ;; a hip not moving holds her trunk. She sits as both her feet lift (as HEAD's lift sat her: none tipped in 10,
     ;; her shoulders swung back as her feet lifted, 1 of 2); a step (lift, set) moves its heel only, every step taken
     ;; once she sits; past the last her arms reach back to her seat's brace (dragged round over her fists, A tipped her
     ;; at 43 deg), or she holds where her twist let go; her vault where both heels push
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
