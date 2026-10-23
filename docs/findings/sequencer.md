# Findings: sequencer

The takes played as a sequencer plays a score: a posture take's beats as rows
on her loop (`machine.takes`), a gait take as amounts on her one standard
gait (`machine.gait`); the takes' bench: [stunts](stunts.md); the vocabulary:
docs/todo/scenarios.md.

## Beats

A take said at its beats, not its frames (`takes.beats`, `wire`): the frames
where a said voice turns or changes pace - its rate or its acceleration, read
over one natural time (sqrt(leg/g), 0.28 s), changes sign -, its first and
its last; each row laid as the beat before arrives and blended to its own
over their gap, so her loop draws one Catmull-Rom curve through them
(`latent.CONTIG`). A take shrinks to a half to a quarter of its frames (the
crouch 123 to 34, the sit 68 to 17, the dance 1 035 to 496).

| take | frames | beats, 3 voices | squats on legs and trunk |
| --- | --- | --- | --- |
| crouching_down | 4.7, fell 0.4 m/s | 2.1, fell 3.4 m/s | 0.8 |
| crouch_up | 5.6, fell 0.4 m/s | 5.9, fell 3.4 m/s | 0.0 met |
| kneeling_up | 4.3, fell 2.5 m/s | 6.6, fell 1.1 m/s | 0.0 met |
| kneeling_down | 0.0 met | 0.0 met | 0.0 met |
| stand_to_sit | 0.6 | 0.4 | 0.4 |
| sitting_on_bed | 0.2 | 1.4 | 1.4 |
| sit_to_stand | 6.4 | 7.6, fell 2.2 m/s | 7.6, fell 2.2 m/s |
| lying_down_from_sitting | 3.1 | 4.3 | 4.3 |
| situp_to_stand | 18.1, fell 1.5 m/s | 16.8, fell 1.5 m/s | 16.8, fell |
| hip_hop | 12.1, fell 1.4 m/s | 12.4, fell 1.5 m/s | 12.4, fell |
| turning_in_bed | 4.1, fell 2.0 m/s | 4.2, fell 1.4 m/s | 4.2, fell |
| standing_yawn | 5.1 | 5.1 | 5.1 |
| the twelve | 64.5 | 66.9 | 53.0 |

- By the voices' turns alone a rise from a crouch was one straight note: 10
  to 90 % of the way in 5.95 natural times for the take's 2.33 (3.16 with the
  pace's turns, met).
- The squats on the legs and the trunk alone (`takes.VOICES`): the crouch and
  the kneel-up tell the hips 2.1-2.4 half-soles ahead of the soles' middle, a
  sole past the toes, where the human's heels rise; told it her parry stepped
  her round the squat on one sole at a time (loads 503/0, 0/470 N from the
  bottom on) till she fell. With the centre of mass her balance's she squats
  on both soles: 0.60 legs down for the take's 0.66, knees 128 deg for 140.
- The crouch's descent reads 1.88 natural times from a tenth to nine tenths
  of the hips' way for the take's 3.65: his heels come down after his legs
  stop shortening and his hips go on down 0.4 s; hers stay flat (both heels
  up is no mode of hers; `foot_rock` is one sole against the other).
- The kneel-up's rise 3.09 for 3.88: the chained kneel (kneeling_down's end,
  axial 3.84) is deeper than the kneel-up take's start (2.41), and the enter
  blend rises her 0.17 m before the window opens.
- The seats are the seat's (stunts.md): on the bed by its beats her hips go
  0.14-0.22 legs back for the take's 0.41, the seat's touch flickers and her
  parry steps seated; in one run of two both soles unloaded and she lay back
  (3.2; 1.4 seated). The rise from the chair falls at 2.2 m/s where its
  frames waited on `risen_back` for ever.
- The yawn's arms, 123 deg both overhead, are no voice of hers: `arm_swing`
  is one arm ahead of the other, and an arm overhead reads through pi (the
  take's arms 9.3 units).

## Patterns

A gait take's pattern at her size: its stride's Hz from the drive voice, each
voice's mean, amplitude and phase at it by least squares; the walk's text has
0.9 Hz, drive 0.6, arms 0.8, lateral 0.31 at 108 deg.

| take | stride Hz | drive | lateral at deg | crouch | lean deg | arms |
| --- | --- | --- | --- | --- | --- | --- |
| jogging_forward | 1.27 | 0.80 | 0.52 at 128 | 0.58 | 5 | 1.21 |
| jogging_backwards | 1.13 | 0.70 | 0.43 at -83 | 0.83 | 1 | 0.68 |
| running_forward | 1.65 | 1.36 | 0.35 at 59 | 1.04 | 20 | 2.74 |
| running_backward | 1.50 | 0.99 | 0.35 at -100 | 0.73 | 2 | 2.67 |
| moonwalk | 1.06 | 0.55 | 0.18 at -93 | 0.41 | 7 | 0.88 |
| ascending_stairs | 0.86 | 0.38 | 0.23 at 111 | 0.86 | 7 | 0.97 |
| walking_up_the_stairs | 0.98 | 0.37 | 0.23 at 102 | 1.26 | 10 | 0.41 |

The dance has no one line (drive at 0.56 Hz, its bounce at 0.44, residuals
half its swing): a song of patterns. The lift's amplitude is the swing's
height (the jog 10.8, the run 21.8, the stairs 14.4-14.8 units of 2 cm), not
her clearance's, which her stride renders: a pattern says the tempo, the
drive, the crouch, the lean and the arms and leaves the rest the walk's.

### Played

A pattern laid over her walk said (`scenarios.PATTERN_S`, as the walk's last
entry has entered; the back ones over her walk back, on within 1.5 s of its
saying): the take's tempo, drive, arms, crouch and lean, the rest the walk's.

| take | told by twist | by its pattern | she does |
| --- | --- | --- | --- |
| jogging_forward | 13.6 | 6.0 | 1.3-1.4 m/s at 1.27 Hz, travel 4.1 legs of 5.7 |
| jogging_backwards | 11.4 | 4.0 | travel 1.8 of 2.9 |
| moonwalk | 8.0 | 2.8 | travel 1.4 of 2.3 |
| running_forward | 27.6 | 20.5 | 1.5-2.0 m/s, crouched, travel 2.6 of 6.6 |
| running_backward | 19.2 | 16.8 | 0.9-1.8 m/s folding to axial 2-3, down after |

- Her stride has no flight. The drive from the take's step length (its
  travel over its steps: 0.73 m, drive 1.2; the run's 1.30 m, 2.2) chattered
  her stance at 2.1-3.2 landings a second and fell her within the window;
  the take's own drive, the soles' separation, is what her stride renders,
  and the jog's 1.95 m/s at her size is a flight gait's.
- A stop from the jog's 1.3-1.5 m/s falls, by the brake's rows or by the
  walk's pattern laid back over 2 s first (the rhythm slowed to 0.92 Hz, her
  speed held at 1.3-1.6 m/s and the landings fell behind her capture point).
  The brake's row (`sprung.BRAKE`, 500 ms to drive 0 and lift 0) ends her
  stride at 1.0-1.6 m/s with her capture point 0.02-0.16 m past the front
  sole, where the jog's landings had caught it 0.03-0.26 m ahead each step;
  no step comes, both soles down she brakes 1.0 to 0.6 m/s in 0.3 s, her
  knees fold (axial 0.5 to 1.4) and she pitches over at 15.95 s: the
  stride's brake at speed, the law's. The bench's runs do not stop.
- The loop lays one row a tick: two rows due in the same tick add (a
  pattern laid in the twist's tick read Hz 1.78 = 0.86 + 0.92, lift 1.90). A
  pattern is laid after the gait's rows.
- The climb's pattern (0.86 Hz, drive 0.38, lean 7 deg, crouch 0.86) over the
  stairs takes' flight, not kept: with its crouch she squatted twice as deep
  (axial 2.4 read for 0.86) on the second tread and fell at 3.4 s; its tempo,
  drive and lean alone, laid as the walk's rows had entered, hopped her onto
  the first tread with her shin on the riser and toppled her backward at 4.0
  s (9.2 and 8.7 widths for the walk's 3.0 and 2.5). The climb's rhythm waits
  on the steering table's rise axis.
- The dance on the legs and the trunk alone (`takes.VOICES`): 12.4 to 9.0
  widths, down at 13.3 s for 4.1; with its arms said too, at 5.2 s.
- Told by their beats instead of their texts the chair's jumps fall from
  the crouch (jump_down 7.4 to 11.3 widths, jumping_down 12.7 to 18.8,
  jumping_up 21.5 to 23.1): a jump is the drives' throw (`machine.leap`),
  not a score. The dance on the legs alone drifts 2.6 legs and falls at
  21.8 s (16.4); the bed turn with the pelvis's list and turn said rolls
  60 deg for the take's 88 (4.1 for 4.2 without): neither kept.
- The bench whole: 5 of 31 met, 185.3 widths off (227.7 before the beats).
  Left to the law, not the score: the seats (the centre of mass off the
  soles onto a prop), the lie and the rise from it, the jumps' throw, the
  runs' flight and the brake at speed, the falls' parry; the yawn's
  overhead arms and both heels up are no modes of hers.

## Modifiers

One standard gait, moved by named amounts (`machine.gait`): the walk's
parameters as the lisp's streams fold them (stand, sway, step, walk), each
modifier one of them. A gait take is its amounts (`takes.amounts`), a word of
her movement language (`tools.sim.normal`) an amount; no gait a special.

| | pace Hz | stride | swing | sway | roll | turn | rock | crouch | lean | width |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| the standard | 0.90 | 0.60 | 0.80 | 0.31 | 0.40 | 0.40 | 0.60 | 0 | 0 | 0 |
| jogging_forward | +0.37 | +0.20 | +0.41 | | | | | +0.58 | +0.19 | |
| running_forward | +0.75 | +0.76 | +1.94 | | | | | +1.04 | +0.71 | |
| jogging_backwards | +0.23 | +0.10 | -0.12 | | | | | +0.83 | +0.05 | |
| running_backward | +0.60 | +0.39 | +1.87 | | | | | +0.73 | +0.07 | |
| moonwalk | +0.16 | -0.05 | +0.08 | | | | | +0.41 | +0.23 | |
| a woman's walk (tests/takes/walk.fbx) | -0.06 | +0.10 | -0.22 | +0.16 | +0.32 | +0.09 | -0.12 | +0.17 | +0.02 | -0.70 |
| a catwalk (Mocaps/woman_catwalk_test.fbx, its straight 5.3 s) | -0.05 | +0.40 | +1.13 | +0.12 | +0.86 | +0.49 | +0.38 | +0.26 | -0.16 | -0.48 |

Her walk so modified, 12 s after the row (scratchpad modifier_play.py):

| modifiers | named | pelvis roll deg | off a woman's | speed m/s |
| --- | --- | --- | --- | --- |
| none | - | 5.0 | 2.49 | 0.71 |
| roll +0.86 | a catwalk | 18.8 (the take's 23.3) | 3.44 | 0.73 |
| roll +0.86, width -0.48 | a catwalk | 18.8, feet apart 0.13 as before | 3.44 | 0.74 |
| the catwalk's eight | fell at 14.8 s | | | |

- The roll alone names her gait a catwalk (`normal.named`: swaying 0.38, not
  wide); the width's amount moves nothing - her stride lays each landing its
  own side of the other (`stride.meets`, APART_M), not the width mode's; the
  catwalk's stride +0.40 at 0.85 Hz falls her as the jog's step length did.
  The roll costs her knee at landing 35.5 to 40.8 deg (off 2.49 to 3.44).
- A woman's walk over her standard: roll +0.32, width -0.70, swing -0.22,
  crouch +0.17 - what her walk's words (landing bent, tripping) are not;
  her walk stays as judged (test_ortho's FORM), the amounts a reading.
- The lisp's gaits beside the walk (trip, jog, ballistic_walk, stairs_walk,
  ice_walk, slope_walk) are specials of the kind the modifiers replace: their
  amounts belong in the steering table's rows (the peer's rebuild), one walk
  under them.

## The runway patch

The patch as written (machine/runway.lisp: a dry state, the sneaker
modifier, the phase-locked morph, the 32-bar schedule) compiled onto her
voices and played on
her walk said: the dry laid at 10 s, the wet four bars on, at once, by a naive
linear crossfade over two bars at a phase of the stride, or by the morph
locked to the lateral voice's zero crossing (scratchpad patch_run.py with its
meter: steps, touchdowns, the feet's lateral gap while they pass, the swing's
clearance, the ZMP's margin in the hull of the standing soles, jerk).

- Read as voices on the first pass, the heel's transient as the rock's
  stiffening (13.6x K) felled her alone at 21.9 s; the hips' stereo width as
  the centre's lateral sway (x1.55) beside the width target widened her to
  230 mm and felled her. The heel's attack is her shoe's sole
  (`mjcf.SOLE_S`): 21.6 ms on a 9.5 ms sole, 24.6 on her 20, 27.3 on 35.
- With the voices she had, the naive crossfade at four phases of the stride
  and the morph alike: no scuff, the swing at least 32.7 mm up, the ZMP never
  out of the hull; the morph's fast high band (10 deg of lean in 3/4 of a
  bar) touched the hull's edge 0.04 % of its two bars. Her blend crosses
  parameters, not signals, and every swing is timed off one clock: her
  touchdowns lie 0.3 deg apart on it, no PLL to add.
- Two knobs laid, Python and C, her walk at their defaults to the last
  column (0.70 m/s, 409 J/m, 2.55 off): the sidechain (a row's DUCK: depth,
  attack, release; latched by each touchdown on her height's spring) and the
  track (her width asked narrower lowers the landings' floor APART_M, which
  her walk's 103 mm sits on; `stride.guarded` holds a swing APART_M off the
  standing sole while they overlap along her way).
- The morph locks on the ZMP her law asks, in the double support past the
  crossing (pi - 2 PHI_W, her soles both standing 0 to 0.6 rad past it): at
  the crossing itself the ZMP is mid-transfer, and her capture point runs
  ahead of her soles walking, so a lock on it never opened.

| the wet | laid | she does |
| --- | --- | --- |
| without its track | at once, naive at 0 and 0.5, the morph | stands; 114 BPM, 0.75 m/s, pelvis roll 0.096 rad (0.115), trunk 0.18 rad, bounce 14.9 mm (22.5) |
| whole, the track 42 mm | naive at four phases | falls 4-10 s on, the feet 31-37 mm into each other passing |
| whole | the morph | falls 24 s on, its track widening to 336 mm |

- The narrow track is not hers yet: at a width of -0.58 (42 mm) she stands
  only with the centre's lateral sway cut (0.31 to 0.11 or 0), her landings
  then 81-95 mm apart, not 42: the swing sole on one stance sole's moment
  reaches 75-87 mm (a 4 m/s^2 lateral against the 27 the pass at 42 asks).

## The ortho-DSP suite

The six modules as given (machine/ortho_dsp/) read by `machine.dsp`, each
patch voiced on her walk by its changes off the nominal manifold - the
nominal plays her walk to the last slot -, told its twist and grip, laid
3 s into her walk on its stages' scene, measured by the converter that
makes a modifier of a take (`tools/sim/suite.py`, 20 s).

| patch | scene | she does |
| --- | --- | --- |
| sneaker runway | flat | stands: 118 BPM, 0.78 m/s, pelvis roll 0.107 rad (her walk 0.044), trunk 0.21 rad; track 92 mm for the 42.5 asked |
| stealth crouch on ice | ice, told 0.08 | stands: 82 BPM, 0.13 m/s for the 0.31 told, her hips 505 mm up (749), a vault of -1.5 mm (her walk +18: level) |
| loaded boot, snow ascent | snow | falls at 10.5 s; on a flat floor stands, trunk 0.37 rad |
| barefoot stair descent | stairs down | falls at 3.4 s, as her walk alone does at 3.43 |
| asymmetric gallop | - | refused: inter-limb-phase-rad |
| ballistic sprint | - | refused: a duty under 0.5 |

- Its schedule to its first refusal on a flat floor, the worlds named, not
  laid: the runway at 3.0 s, the morph locked at 7.45 s, the crouch at
  10.15, the morph at 14.24, the descent at 16.76 - standing -, the gallop
  refused at bar 16.
- The thoracic cutoff read as her trunk's spring, her posture's 1.59 Hz
  moved by its ratio to the nominal's 8 Hz, bent her 0.85 rad on the runway
  and 1.03 on the runway's wet for 0.16 and 0.17 asked: voiceless.
- A stride read as her drive's amplitude (x1.54) moved hers 713 -> 759 mm
  where 1 220 was asked and felled three takes at 6.2-10.3 s: told as her
  speed alone, her step her capture point's.
- A take's way read 4.8 deg off its facing told her 0.08 m/s aside and
  strafed her (`se2.GAIT_BOXES`): 117 BPM, 0.16 m/s, the swing 4 mm up. A
  way is the nearest of her four; ahead past 0.96 m/s her twist picks her
  jog, a patch is told 0.96 and the speed asked said beside.
- Told the runway take's 0.95 m/s alone she falls at 11.9 s, without its
  arms at 8.8 s; with its row (arms x2.9, roll x4.3) she stands.

## The suite's maneuvers

The chair's sit and rise played from the suite's patches (`machine.maneuver`):
a patch's acts her phases, each paced to the spring it moves (1e3 sqrt(BENT) /
omega_n ms: the preload 350 -> 575), her arms held (`mode_arm_swing` is
antisymmetric: no counterweight to voice), her hips landed where her centre of
mass stands over her ankles, no further than a buttock behind the seen edge
(`seated.landed`), docked at the patch's standoff (chair 85 mm, bed 70).

- Risen at the dock 13-14 of 14 in 4.5-15 s, her soles within 3 mm, her arms
  0 deg/s; the room's chair by the viewport met; her walk 0.70 m/s, 409 J/m,
  2.55 off, unchanged.
- Her centre of mass's zero her soles' middle, the sagittal mode's: her
  stand's is 2.9 cm off it; a manifest mass model 4 mm off `figure.com`.
- Not kept: her pelvis sprung while seated (off the seat at 4.0 s); her pelvis
  after her trunk (leaned in 3 cm nearer); landed on the edge alone (9 of 14);
  a sole resting wherever it does not stand (her walk 2.55 -> 2.51 off, a bob
  in the stumble): it rests with her ZMP let go alone (`swing.rested`).
- The bed's patches read, not played: her step holds each borne point still
  in 3 axes, and lying back stops at her spine's 0.6 rad; weighed 0.003
  tangentially she lay back and rolled over as the cascade went high-Z
  mid-motion; the mattress gives 2.3 N/mm to the patch's 18.5, her descent
  2 cm short.
- Her hips landed at her soles' back edge with her thighs clear of the
  seat's edge (`seated.landed`, `clear`), her pelvis turned square to the
  seat and the arm on its side ahead (`maneuver._envelope`): laid at its dock
  grid she sits and holds 30 of 50 (16 before), rises 27 (8), falls 3 (18);
  said from every spot the chair 75 of 112 (67), 7 falls in the sit (16).
  Open: 20 placements never borne, 8 at her root 0.14 m off the edge, 12 at
  0.19.

## The autotuner

The text as given (machine/meta_ast_autotuner.lisp), read by `machine.tune`:
six classes, 33 keys (32 in the suite's names); its step's numbers its own (lambda
0.08, a literal's move 0.15 of its scale, tol 1e-3, four places).

- Her texts, the suite's and her gym's (20, given to her session as an
  argument, `tune.texts`): 554 literals, her gym's 199 bound to the suite's
  name voicing reads into the slot their clause lays (the clause compiled
  alone); the grammar names 173, a root walks 159, 100 move her by
  evaluation: in 32 of her gym's 34 behaviours, 5 of 8 gaits, the 3
  schedules, the 2 stunts. The rest no voice or phase of hers reads, their
  columns 0. Of the suite's alone (331): 75 named, 18 live.
- Her walk (her gym's four streams) reaches 5, 3 live: her stand's axial
  spring 3000 and damper 600 (compliance, damping ratio), her step's lift
  1.0; her lean's 0s have no scale. On her loop: the centroidal pivot under
  15 mm on 54 % of passes, the friction cone 0.40 used, her peak 1.80 of her
  weight, her master bus's tank (wbc_bus.c) bled 161.7 J in 16 s against its
  12 J: 13.47 of |r| 13.7 - the anti-spasm insert's tank at its floor through
  her walk.
- The classes now read the cone's use for shear (|F_t| over mu F_n, each
  contact's own mu; slip reported) and the tank's bleed over its capacity for
  passivity; the runway's and ice's numbers below were measured on slip and
  her drives' J/m. Her vertical mode fitted on her centre's acceleration read
  her step's rate (1.86 Hz, zeta 0.003), not the contact's: not kept.
- The runway reaches 17, 3 live, each by its voice's rule: the nominal's
  stride on her twist, its thoracic lean -2.0 on her pitch (unit 0.5), the
  catwalk's attack 1e-3 on her duck; a circumduction of 0 has no scale. The
  sit reaches 3, its hand-over's and descent's ms live.
- On her loop (`tools/sim/tune.py`, the suite's harness, 16 s from 4 s, 19 s
  a run) her walk with the runway's row: the centroidal pivot under the
  text's 15 mm on 55 % of passes, its median 12 mm - she rolls over her toes
  each step, no walk's invariant -; her load never under 0.698 of her weight
  (0.35), her feet never nearer than 12.6 mm (6.5), 274 mm of slip, 350 J/m:
  |r| 4.074, its energy 3.50 of it. A run repeats to the last digit; each live
  literal a unit of the text's fourth place off moves |r| 0.002-0.009: a
  step's floor.
- Each step that lowered |r| on the runway raised her landing (1.424 ->
  1.433-1.450 of her weight): the pivot bought with her form. Held in the step
  (each form measure's measured gradient kept from growing, `qp.solve`), its
  trust region halved on a refusal, at half it kept 640 -> 688 mm stride, lean
  0.04 -> 0.0295, attack 105 -> 112.9 ms: the pivot's shortfall 1.73 -> 1.26,
  slip 274 -> 221 mm, landing 1.42 -> 1.41, 350 -> 360 J/m, |r| 4.074 ->
  3.926. From there no step keeps her form (her landing's knee and load, her
  knee behind plumb) till the model's gain, 0.0079, is under its 0.009 floor.
- The stealth crouch on ice (22 walked, 7 live): her 1 747 J/m at 0.13 m/s is
  17.47 of |r| 17.47, the text's energy class the whole norm. Its first step
  kept by 0.125 over its 0.118 floor bought 1 734 J/m with slip 81 -> 94 mm,
  on ice; then none past its floor.
- Any root run (`tools/sim/telemetry.py`), the classes over whatever bears
  her, the impact the world's peak load, the energy her cost of transport (J
  a metre of her centre of mass's way over her model's 305.5 N): the runway's
  |r| 2.392, the same step kept (her peak 1.663 -> 1.596), converged. Run on
  every other root its literals reach, the omnidirectional schedule fell at
  22.03 s: not written.
- The omnidirectional schedule (48 s, four morphs): 36 walked, 10 live, 5 of
  them felling her at 0.15 of their scale; its floor 0.209 over its model's
  best gain 0.173: nothing to resolve.
- The chair's sit: its descent's duration fells her at 0.15 either way; its
  hand-over's 320 ms moved 0.75-48 ms felled her 3 times in 7 (6.46-9.87 s)
  and raised |r| the rest - her sit is chaotic in it -; under 0.4 ms her law
  rounds it away, her run the same bit for bit. Its |r| 8.23 is 8.23 of
  energy: a sit is no transport.

## Takes as modifiers

A take converted against her walk (`tools/sim/identify.py`: Zeni's strikes,
a period in 0.5-2.5 s, the vault's sign the topology, lengths and times at
her size) and played as the patches are. Her walk by it: 110.1 BPM, track
106.5 mm (her soles 108 by the world), vault +18 mm, duty 0.65 (Zeni's lift
late; her gate 0.6). Of 53 takes beside the repo 20 played and 8 of them
fell, 14 were refused by name, 19 not identified (a start, an end, a turn
in place, a strike a side).

| take / hers | BPM | track mm | roll rad | trunk rad | arms rad | m/s | stride mm |
| --- | --- | --- | --- | --- | --- | --- | --- |
| an injured run, treadmill | 131 / 129 | 211 / 202 | 0.096 / 0.104 | 0.088 / 0.087 | 0.325 / 0.311 | 0.87 / 0.84 | 794 / 781 |
| a hand-keyed slow walk | 92 / 90 | 54 / 92 | 0 / 0.021 | -0.051 / -0.050 | 0.108 / 0.121 | 0.55 / 0.47 | 724 / 625 |
| a hand-keyed phone walk | 92 / 90 | 56 / 101 | 0 / 0.031 | 0.124 / 0.126 | 0.002 / 0.096 | 0.55 / 0.47 | 724 / 632 |
| a forward walk | 110 / 118 | 192 / 162 | 0.102 / 0.108 | 0.014 / -0.009 | 0.314 / 0.367 | 1.12 / 0.70 | 1 220 / 713 |
| a runway catwalk | 101 / 118 | 72 / 108 | 0.19 / 0.23 | -0.005 / -0.026 | 0.73 / 0.83 | 0.93 / 0.74 | 1 100 / 757 |
| a catwalk loop | 98 / 118 | 3 / 100 | 0.057 / 0.061 | -0.006 / -0.014 | 0.259 / 0.258 | 0.87 / 0.72 | 1 060 / 736 |
| a suit's walk | 104 / 118 | 145 / 134 | 0.037 / 0.025 | 0.055 / 0.060 | 0.50 / 0.61 | 0.84 / 0.63 | 961 / 643 |

- Her trunk, her arms and her pelvis's roll follow a take within 10-20 %,
  its tempo within 2 % where she can stride its length. Past her 0.76 m she
  steps 118 BPM at her own stride, her twist capped at her walk's 0.96 m/s.
- A wider track she follows (192 -> 162); a narrower one she holds at
  92-108 mm: the width lowers her landings' floor, not her sway's reach.
- Refused by name: the runs by their vault (-68 to -81 mm), a treadmill run,
  a skip, the hand-keyed jog and run; a backward run by its duty (0.3); the
  stairs takes by their phase or vault on the climb (their floor is the
  stunts bench's, told).
- Five studio walks, a monster's walk and a relaxed loop key 22-77 steps a
  minute: she falls at 4.7-6.3 s, her centre diverging at 0.31 s; one gait
  cycle looped falls at 12.9 s. A curving walk turns her at 0.12 rad/s, her
  run no straight stride to measure.
