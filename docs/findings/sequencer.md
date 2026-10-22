# Findings: sequencer

The takes played as a sequencer plays a score: a take's beats as rows on her
loop (`machine.takes`), a gait take's pattern as its rhythm's parameters; the
takes' bench: [stunts](stunts.md); the vocabulary: docs/todo/scenarios.md.

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
  speed held at 1.3-1.6 m/s and the landings fell behind her capture point):
  the stride's brake at speed, the law's; the bench's runs do not stop.
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
