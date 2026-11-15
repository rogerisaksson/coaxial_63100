# Findings: leap

The robot's soles off the floor on the whole-body stack ([wbc](wbc.md)): which
stand, the others asked to their landing.

`machine/moves/leap.py` stands between a motion pass's ask and the stack's step;
`tools/sim/hop.py` throws the robot; tests/test_leap.py holds it.

- The stack and the loop both solve a step with no sole standing: no force,
  the robot's centre of mass at -9.81 m/s^2, the drives at 0 N m. What they are
  asked was missing: asked to stand on both soles 0.2 m over the floor the stack
  lays 153 N a sole on nothing.
- Which soles stand is told from one pass, nothing kept
  (`leap.standing`): a sole stands that bears 15 N; one that bears
  nothing stands within 2 mm of the floor beside one that bears, or with
  the robot's centre of mass under 0.1 m/s. What fell on the way:
  + By its load alone: the first sole to lighten was lifted and the other
    left the whole throw - 648 N for 50 ms -, 19 deg over.
  + Within 2 mm alone: the stack pressed a sole 1 mm over the floor down
    onto it as the robot left, 648 N: 23 deg over on the loop in C, down at the
    second of three hops on the python stack.
  + All soles or none while the robot rises over 0.1 m/s: the one it stood on
    after a landing flipped at that speed for 0.2 s, 21.2 deg over three
    hops.
  + An unloaded sole stood near the floor as the robot falls too: three hops
    stood at 2 mm and fell at 0.5; with it at rest the numbers are the
    same at 0.5, 1 and 2 mm.
  + The robot's centre of pressure brought 25 mm inside the standing soles' box:
    three hops 13.1 deg over against 9.8. Not there.
- The robot's soles' loads cross 15 N twice in the 3 ms they leave the floor in:
  no sole stands for 1 ms, both for 2, none from there. The robot flies while
  neither bears 15 N; `tools/sim/hop.py` counts a flight of 20 ms or
  more.
- A sole that does not stand is asked to its landing: under the robot's capture
  point, drawn in to its leg's reach (`reach.land`), the knee 25 deg.
  The stack brakes a knee going out 9.2 rad/s to rest in 40 ms at its clamp, 108
  N m.
- With no sole standing no turn is asked. Asked - even as they were, their
  rates alone bled - the stack spun the robot's legs to hold its trunk still:
  the knees out at 3 rad/s with no torque in them, on their stops at the
  landing, 1.9 kN, down.
- The throw. The robot's height asked down alone, the posture held its knees: 10
  mm of 60; crouched by the posture too (hip, knee and ankle by their terms),
  its pelvis straight down over its ankles, its centre of mass lay 26 mm behind
  its soles' middle. The robot's trunk asked upright through the throw, the
  stack put its weight on its heels' edges - 16 mm past the
  soles' margin - to turn its legs under it: 4.7 kg m^2/s about the
  robot at lift-off, 14.9 deg over at the landing; its trunk unasked,
  5.4 deg.
- The robot's legs follow it up to knees of 22-31 deg: 9-12 rad/s at the
  knee, of its drive's 16.8 with no load. The robot leaves there whatever is
  asked: 11 mm under its standing height at 0.7 m/s, 21 at 1.05; a push of 22
  m/s^2 flies 2 mm higher than 18. Asked to its time's end, 1.8-2.6 of the
  4.3-5.0 kg m^2/s about the robot at lift-off came in the last 20 ms, the
  robot's soles' loads falling from 730 N to none. The robot's throw is a spring
  instead, to 17.5 mm under its standing height, 700 1/s^2, no stronger than the
  push: 0.4-0.7 in those 20 ms, 2.7-3.1 as it leaves. Twelve springs (12.5-20
  mm, 500-900) over the four hardest rows: 46 of 48 stood; at 10 mm the loop was
  14-15 deg over as before.

| Let down, thrown, for | On | Flown | Under the robot's soles | Struck | Tilt | To its time's end |
| --- | --- | --- | --- | --- | --- | --- |
| 0.10 m, 10 m/s^2, 0.15 s | the python stack | 0.129 s | 16.3 mm | 1 848 N | 6.1 deg | 21.5 mm, 7.1 deg |
| 0.10 m, 14, 0.15 | python | 0.145 | 21.5 | 1 966 | 4.9 | 26.8, 5.4 |
| 0.12 m, 18, 0.2 | python | 0.215 | 46.4 | 2 553 | 6.1 | 46.3, 8.0 |
| 0.10 m, 14, 0.15, three in a row | python | 0.447 in all | 23.1 | 2 031 | 5.7 | 28.2, 11.5 |
| 0.10 m, 14, 0.15, the robot's soles never released | python | 0.103 | 9.4 | 745 | 4.9 | 8.7, 4.5 |
| 0.10 m, 10, 0.15 | the loop in C | 0.128 | 16.3 | 1 766 | 5.5 | 23.4, 10.3 |
| 0.10 m, 14, 0.15 | C | 0.145 | 21.5 | 1 933 | 4.6 | 30.8, 13.6 |
| 0.10 m, 10, 0.15, three in a row | C | 0.397 in all | 17.8 | 1 910 | 7.0 | 24.5, 10.3 |
| 0.12 m, 18, 0.2 | C | 0.220 | 47.7 | 2 576 | 15.2 | 56.2, down at the landing |
| 0.10 m, 14, 0.15, three in a row | C | 0.446 in all | 23.1 | 2 022 | 6.6 | 37.7, down at the second |
| 0.12 m, 22, 0.2 | python | 0.216 | 48.4 | 2 664 | 7.2 | - |
| 0.12 m, 22, 0.2 | C | 0.221 | 49.2 | 2 795 | 16.5 | - |

The robot stood every row as thrown now.

- The loop in C, two changes. A turn of all zeros is unasked. The soles' margins
  are laid for standing soles only: laid for a sole in the air too, their weight
  set the level's scale and the levels below came in as strong as a swinging
  sole's own ask - with no sole standing it met 33 of 115 m/s^2 where the python
  stack met 110; now 110, 0.006 m/s^2 apart. The robot's walk (8 s at 0.5 m/s:
  2.47 m, 0.41 m/s, 259 J/m, 15 steps, tilt 1.8) and four shoves (38 N from the
  robot's left, 60 from ahead and behind, 80 from its right) are as
  before to the digit.
- Open. The strike, 1.8-2.8 kN at 0.8 m/s: not the robot's height's law's (at a
  quarter of it 1 735 N; the torque as asked under it 1 414), nor the sole's
  speed onto the floor (eased onto the floor it left over the last 30 mm, 2 244
  N). Standing on one sole the loop meets 15 of 115 m/s^2 of a sole asked back
  where the python stack meets 97: the robot's form's weight on the turns'
  level, which its walk's 259 J/m is ([wbc](wbc.md), The margins soft). A hop on
  one sole is untried. The large throw leaves 21 mm under the robot's standing
  height, under the spring's zero: the loop 15-17 deg over there.
- tests/test_wbc_core.py stood in no suite list: the loop's suite ran in no
  gate. The suite is in the default set now, 21 of 21.
- The stride leap said after 1.8-2.3 s of walking, four leaps (walk and run,
  tapped and held): 17 of 24 stand, its outcome the robot's stride's phase at
  the word. The leap's gather braked over the robot's stand's blend
  (`se2.STAND_S`) - over its stop's stride the hop's stand cut it at
  0.4 - the robot's held leap goes further than its tap again: walk
  -0.05 -> 0.35 m, run 0.42 -> 0.97. Landed, the robot's capture point runs on
  at 1.6 m/s while its rhythm is still, and a swing at that rhythm's pace took
  0.40 s as it went over a sole's ball: hurried along its way too while its
  rhythm is still, the run's held leap falls at 1 of 6 walk times for 2. Left:
  no flight at 1.8-1.9 s in every tree; a split stance 27 cm long after the
  parry's step, the stand centring the robot between it, the robot drifts back
  on the leading sole. The robot's gait resumed from the landing, no stand
  between: 16 of 24, other phases down.
- The gather laid as the robot's swinging sole lands (`stride.beat`'s share of
  its swing left at its rhythm's rate) instead of on the clock: 21 of 24, the
  suite's four at its 2.0 s all flying (0.17-0.42 s), the robot's walk to the
  digit. Left: no flight at the run's 2.3 s (both soles never off), the held run
  at 2.1 s down after its landing. Measured, not kept: the leap derived from the
  human's running jump (`takes.jump('running_jump')`: his gather over one stride
  to his crouch, axial 1.98 leant 1.16, the drives' push from it, the robot's
  rhythm his limbs' lift through the gather) - 3, 0 and 1 of 24. His is a
  sprinter's at 4.7 m/s, hips 1.7 half-soles behind his plant; told them the
  robot went backward and sank on 40-90 N, its hips its own the throws flung it
  off its feet (air 1.3-3.4 s). Similarity scales his lengths and times to the
  robot's own, not his speed.
- The stride leap's flight told by what the robot senses a pass, no memory
  (`swing.flew`): thrown, no sole standing, and its centre going up or
  no sole left lying on the floor (within `leap.TOUCH_M` of it, not coming down
  past `leap.REST_M_S`). Thrown with none standing alone, the robot's dip's
  passes flew - its stance sole under a touch (0-13 N) still lay on the floor as
  it sank 0.57-0.83 m/s -, its swing was asked to the floor's speed there, 52.8
  m/s^2 back, and it bounced off its dip: 0.336 s aloft for 0.24. Aloft the sole
  that trails hangs level and the one that leads lands at the floor's speed
  (`swing.hung`), the robot's stride the sole's that lands first
  (`stride.flown`), a swing begun before its window counted from there
  (`stride.begins`); the push's rows on their plan's clock, damped critically
  (`ballistic.bound`). A walk's tap flies 0.195 s, 51 mm clear, 0.13 m; a run's
  0.219 s, 68 mm, 0.27 m (test_leap's bars back at 0.15 s and 45 mm). 32 plays
  from 9 starts each: 30 whole where 17, 284 of 288 stood where 214, none
  worse - the hop said 3.3-3.7 s into a walk and a jog 9 of 9 each where 0-9,
  the run's leap on the run grid 9 where 3. Left: the stutter and the sprint's
  grid 7 of 9 as before; the leading sole meets the floor at 0.95-1.17 m/s
  along; the hold's charge has no lever in stride - dipped 1.35 times deeper the
  robot flies no further (0.14 and 0.25 m): its stance sole behind the
  robot bears a third of it, it sinks 0.16-0.21 m for the
  0.11-0.15 told, outruns a push laid on the clock and coasts, its knee
  at 37-106 of its 171 N m.
- The robot's launch from its drives (`ballistic.launch`, test_ballistic): both
  knees at their back-EMF bounds, their rotors (0.177 kg m^2) spun with it,
  leave the floor at 15.8-15.9 of their 16.8 rad/s: 2.03-2.07 m/s from a
  crouch of 176-185 mm, an apex of 210-219 mm (the ballistic texts' 320-360
  at their 0.89 m leg, 304-342 at its own). One knee from 109 mm: 1.21 m/s,
  75 mm. The robot's ankles' heel rise reaches 2.15 m/s at most (18.4 rad/s on
  117 mm). The robot's run at the texts' Froude 1.93, 4.00 m/s: a sole's
  half-sine peaks at 743 N, its knee 59.6 of 108 N m at 0.42 rad.
