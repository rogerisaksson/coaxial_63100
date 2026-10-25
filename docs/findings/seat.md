# Findings: her seat

Her sits on the lodge's furniture, measured in the simulated world: where a
sit lands her, the kernel's Coulomb slide as her step's ask, and what her seat
is to her step. The kernel's other steps: [kernel](kernel.md).

## Her sit's dock

Why she sits on the chair's front edge leaning forward (`seats.py --sim
chair_behind sit_chair`, a human's band from the takes): her hips land 0.218 m
behind her ankles, 0.23 legs where a human's 0.31-0.47, her trunk seated 13.1
deg forward where within 7, her soles coming and going 14 times in 8 s.
`seated.landed` caps her hips a buttock past the seat's edge (0.140 + 0.078 m)
and has her thighs clear the edge where a human's rest on it. With her thighs
let rest the press her hips land with and the cap gone: 0.29 legs, 1.5 deg,
knee 97.7 (94-98), 6 come-and-goes - and her centre of mass at her heels'
edge the constraint left (0.32 legs at the text's 0.75 rad lean); risen from
the deeper seat she fell at 31.2 s where she stood in 0.94. Her rise needs
the seat's slide forward first (the kernel's Step 4): undone till then.
Landed so again with her seat one patch of her step and her lean panned:
hips 0.29 legs back, trunk 1.1 deg, knee 97.8, her soles coming and going 8
times (`seats.py --sim chair_behind sit_chair`; the human band 0.31-0.47,
within 7, 94-98); square and +2e-4 she sat and rose rolling 21 and 27 deg,
at -2e-4 her soles left the floor 3.2 s in, the seat bore 1.47 of her and she
rolled off it at 47 deg by 4.1 s. The kernel's ingress seats her with her
hands (`specialize-with-carpal-seat-boost`: the carpal heels on the seat up
to 0.20 of her, `appendage-admittance` seeking them down at 0.25 m/s): the
deep sit waits for her arms as supports (`borne` leaves them out). Undone.

Her bed the suite's way (`house.py --one 0.32 1.01 1.37 lie_bed --said`): the
supine sleep's dock, 70 mm, fell her off the mattress's edge in the settle 3
of 3 - the bed soft, rigid, raised to the chair's 0.47 m - where the chair's
85 mm held, and a chair lowered to the bed's 0.44 held (neck 213 deg/s).
Docked at 85 the side-lie (side_lie.lisp) sat her and laid her nowhere: her
list and turn modes at their reach, 0.9 and 1.2 rad, tilt her pelvis against
her standing legs; at rest her soles bore all of her, 305 N, the bed 0, her
pelvis 0.64 m up, her neck swung 1 363 deg/s. Her gym's text (motion.lisp)
kept: arms 535 deg/s, trunk 279, neck 834 against her walk's 174, 88, 110. A
roll of her base onto her trochanter with her legs off the floor is no mode
of hers (Step 4).

## The Coulomb slide

The kernel's slide (`drive-tangential-coulomb-slide`) on her loop: her text's
`(slide v)` (m/s along her way, forward +; `ortho.SLIDE` -> `latent.SLID`)
names the regime, her pressing contacts' load capped at its share of her
weight (`somatic.CAPS`: deeper 0.55, forward 0.70); the slide itself her
step feels (`wrench._patch`, `rounds.pushed`, wbc_patch.c): a point slipping
along the floor's plane past SLIDE_M_S, along the said slide, has its
friction the kinetic one against its slip, given, its rows up alone; stuck,
or slipping against the say, it is held still, its friction the step's
within the cone. Counted whatever the text said, a hand walked into the
wall slid by that measure, was let go of along the wall's normal and the
wall threw her back 1.18 m (the wall scene at +2e-4, main's met at 0.22),
and a strike's skid as she stood up from the chair (1 000 N, 0.4 m/s)
flipped the step's answer for 6 ms. Measured across the force a patch
feels, the slip's plane leaned 30 deg into the lean's push (the force is
no surface normal) and the regime let her hips drop along it. Unsaid the
step is as it was to the bit. Sat on the
chair at its 85 mm dock, her pelvis 44 mm behind the seat's front edge (the
edge sit), 2 s of asks of the step, the regime then an ask (`slide_probe`):

| ask | moved along the seat | seat's share after |
| --- | --- | --- |
| none | +0.001 m | 0.52 |
| the point's velocity row at W_STILL, +0.16 m/s | -0.095, off the seat | 0.00 |
| the same with her centre of mass driven back | -0.029, soles 0 N | 0.50 |
| her centre of mass driven back alone, rows still | +0.043 | 0.63 |
| the regime (rows freed, friction given), com driven | -0.19 to -0.22 | 0.59 |

- A velocity row on the thigh's point at the still rows' weight lifts her off
  the seat: cheaper than moving her. The point slides by her centre of mass.
- Freed of the stiction the step assumed of the seat, her seated posture
  pushes her hips forward off the edge: the edge sit stood on that assumption.
  The slide's driver is the phase's deeper posture (`seated.landed` behind)
  with the regime, not a velocity on the point.
- A lower hold on a contact's normal force (within GROW_N of the force felt)
  felled her sit as the seat's first touch spiked: the felt force is a pass
  late (`machine.borne`).
- Her text says it: `(slide -0.15)` in a phase's task constraints, m/s along
  her way, forward + (`ortho.SLIDE`, the row 304 bytes; `latent.SLID`,
  wbc_law.c alike; the step's ask from it). Nothing said, the sit and rise
  are as before to the digit. Derived instead from her sagittal reference's
  rate while borne, the slide was live in the sit's own settle: she sat 2-3 cm
  deeper (hips 0.067-0.075 m past the edge where 0.044, the seat bearing 0.63
  where 0.54) with one sole off the floor, and the errand's rise fell at 23.6
  s - undone for the text's say.
- Said with her hips asked 0.18 m deeper over 1.2 s, the knee as the edge
  sit's or as a seat at that depth has it: her hips moved 0.002 m. Her feet
  planted and her knee held set her hips' depth; a slide deeper needs her feet
  to come along - the reachability's equinus onto the forefoot (item 4), or
  the feet's own slide.
- The reachability on her loop (`machine.somatic`): each leg's reach, hip to
  sole over her leg's 845 mm. Standing 0.998 (her knee 6 deg), nothing borne:
  her soles her stride's; sat on the chair 0.70 and 0.75, under the text's
  0.86; 0.18 m deeper would be 0.92, her sole's centre of pressure half onto
  its ball by the seat's 0.85. Asked deeper from the edge she never got
  there (2 mm): the feet must step or slide along before the reach grows.

## Her seat as two points

The chair scene laid 2e-4 rad off (scenarios' chair, red on main): sat as
square to the digit for 2.5 s (the seat 0.59 of her, soles 63/63 N), then at
4.0 s the step's expectation of the seat's two contacts - a thigh each,
`machine.borne` a point a segment - chatters pass to pass, 63/65 -> 40/60 ->
146/112 -> 104/0 -> 4/4 -> 68/219 N, her knees -13 to +50 N m, her soles 13
to 320 N; she pops off the seat at 4.5 s, yaws -0.22 rad and rolls -10 deg by
5.7 s, and the rise at 8.0 fells her (8.44 s). Laid square the split stays
even by symmetry alone. Not it: the clip's scope (one total for the two
points, the split free: down at 6.9 s) and a prior toward the force felt (a
pass late and spiky: down at 4.2 s at every wobble, laid square too). The
Python step and its C twin fall to the digit alike. The text bears her on
one manifold with a centre of pressure (`ischial-bilateral`, the slide's
stop on its CoP): her step is to take a patch's contacts as one wrench, its
force one, its CoP within the points' spread as a sole's within its room.

Tried on the Python step (`chair_py.py`, the step swapped in, falls to the
digit as the C's) with the patch's centre (the felt forces' weighted point)
held still: the sit held at every wobble and the rise twisted - the centre
held while her pelvis must leave it, yaw +-0.25, one sole 350 N, knees -93
N m. Landed instead, in both (`wrench._patch`, wbc_patch.c): each contact's
point held still and its force along the patch's normal pinned at its share
of what is felt (W_PIN, as a clipped force is held). The step's expectation
of the two thighs stays within 5 N of the felt at every wobble; the chair
bears her 5.7 s square, 6.05 at -2e-4 (where she fell at 8.44), 5.67 at
+2e-4. What bears her off her soles is capped as told (`somatic.asked`: the
pan's share of her weight, the slide regime's 0.55 deeper or 0.70 forward,
never under the lever's rule - her centre of mass between the seat's points
and her heels; under it the step planned her soles 20/19 N where 250 came),
over every patch as felt: capped a patch at a time, a thigh on the seat's
front face let the seat plan 297 N under 214.

## The rise's lean

The handover's preload pans what bears her to her soles at the share its
crossfade waits for (0.82, `maneuver._panned`), her ZMP in her support, the
kernel's egress slide said (`(slide 0.15)`): left to the fade, the seat bore
0.9-1.1 of her through the lean and one sole took her at the thrust, 33/156
N. The slide as a regime assumed - the thighs' rows freed, their friction
pinned kinetic against the asked slide - fought the seat she sticks to: the
lean's first pass planned -128 N along her thighs and felt +148; its rows
taken apart (`step_rows.py`, the pass's least squares unclipped against the
rounds') put the centre of mass's residual at 4 589 and the points' at 16
201 where the unclipped answer had 8 and 9, her soles planned 4/3 N where
67/65 bore; at -2e-4 she bounced on the seat at 1.64 of her weight and fell
at 11 s. The regime felt, she rose square and at +2e-4 (roll 7.5 and 1.6
deg; main's 3.8 and 8.0) and collapsed onto the seat at -2e-4: her neck let
go while she sat (`motion.control`'s w_sit, errands findings), the lean
whipped her head and asked her neck and head 10 and 8 times their clamps,
the whole ask shrunk as one to 0.03-0.16 (`wrench.shrunk`) and the seat met
at 3.7 of her weight; braced seated as walking, the shrink never bites
(scale 1.00 through the lean). Her legs were at the seat's gain through the
lean, the sit's cross-fade's -24 dB (`substrate-rigid-chair`): panned to her
soles so, one sole took her, 274-401 N to the other's 0, and rolled her 13
deg square; her legs' springs back toward their rest by the pan's share
(`maneuver._legs`), she rises square and at +2e-4 with roll 2.4 deg and the
dock's rise stands, its lean on the staggered sole. At -2e-4 she sits where
main fell and rises on one sole (0/299 N through the stand, the lean's
strike at 1.2 of her weight having lifted the other), stumbles at 9.8 s and
is down at 11.3; the same lean rose at 3.3 deg with the regime left to count
a backward skid (chaos, not a cause: the gate decides the outcome at that
wobble). Left (docs/todo/protocol.md): one sole bears her through the lean
(0/257 N square) and the seat is met at 1.2 of her weight twice: capped
under what holding her hips still asks, the step rolls her pelvis for it
(the pass taken apart: the points' rows 13 900 where the unclipped answer
had 76); the handover wants the hips asked up off the seat as the pan goes,
not the seat's force capped under what it gives.
