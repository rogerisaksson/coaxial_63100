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

The kernel's slide (`drive-tangential-coulomb-slide`) as an ask of her step
(`wrench.step`, wbc_step.c, wbc_rounds.c): `slide`, a pressing contact's
point's velocity, frees its tangential rows and pins its friction to the
kinetic one against the slide, MU of its normal force - given, not the
step's to spend -; `fz_cap` caps that force. Unasked the step is as it was
to the bit. Sat on the chair at its 85 mm dock, her pelvis 44 mm behind the
seat's front edge (the edge sit), 2 s of asks (`slide_probe`):

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
digit as the C's): the seat's contacts one patch, its centre (the felt
forces' weighted point, J = sum w_i J_i) held still, each contact's rows the
patch's at the root of its share, their forces free as before. The sit holds
at every wobble (the chair borne 5.9 s where 3.0 at -2e-4; square 5.7, +2e-4
5.8), and the rise then twists: at the lean the centre is held while her
pelvis must slide forward off it - yaw +-0.25, one sole 350 N, knees -93 N
m; down at 11.3 s at -2e-4, square 13 cm off where 3. With the kernel's
egress slide said through the lean and thrust (0.15 m/s forward, the seat
capped at 0.55 of her weight) the lean frees: -2e-4 rises and stands clean
(rest at 11 s, soles 153/153 N, roll 0) and square falls at 9.9 s, the
mirror image - at the lean one sole takes the seat's surplus (123/0 N) with
the ZMP let go, which way a coin's toss. The change is one: the patch rows
in her step, the texts' slide phases (ingress deeper, egress forward) and
the lean's plantar share panned back as the handover has it, measured on
the whole gate before any of it lands.
