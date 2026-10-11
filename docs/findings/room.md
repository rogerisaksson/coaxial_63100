# The living room

The room round her on the HUMANOID page (`machine.room`, `physics.ROOM`, the
user's, 2026-10-11): glass walls 6 x 5 x 2.4 m that do not break (static boxes
with the floor's contacts, drawn as edges in glass's ink), a chair facing -z at
(1.5, 0.8), a bed at (-1.8, 1.8), a table at (1.5, -0.8), a floor lamp at
(-2.5, -1.0) with its switch on the left wall at 1.05 m, and a door 0.9 m wide
in the far wall hung on a hinge at x 0.3, swinging outward to 110 deg against
3 N m s/rad: 15 N m opens it fully in 1 s. Her hand on the switch toggles the
lamp (`room.touched`, not twice within 0.6 s). The page draws its 22 boxes as
wireframes; the door where it swings; the lamp lit.

## The words

`room.WORDS`: sit on the chair, lie on the bed, out through the door, the lamp;
each a target (x, z), a facing and the heading it is come at, walked to on the
one law (turns.md) and done there (`machine.errands`, keys U I E Y, U or
I again up): turned standing toward the way, the walk's row to a point 1.8 m
before the target on the word's heading, stood and aligned there, the slow
row straight at the target, the stop asked 0.3 m before it (the halt carries
her 0.15 m), turned standing to the facing, 1 s settled. Measured 2026-10-11
(`scratchpad/words_probe.py`):

| Word | Outcome |
| --- | --- |
| out | through the door - pushed open 102 deg by her toes and fingers -, standing beyond it 0.13 m from the target at 24 s |
| out, E pressed on the page as she walks (z 2.1) | halted, turned 48 deg to the approach point 0.6 m off, walked it, halted, turned back 33 deg, and the walk's restart fell at 16 s - she never reached the door (the user saw her meet its post, 2026-10-11); from her stand she crosses the line 0.39 m from the nearer post, her toes on the door alone |
| lamp | at the switch 0.43 m off, faced on it within 6 deg (the heading aimed so her right hand's line, 0.27 m to her right, runs through it); the arm raised 40 deg/s meets the wall under the switch and the push fells her: the hand's arc crosses the switch's 8 cm only 0.39-0.44 m off it, the halt scatters her 0.1-0.3 |
| sit | at 0.6 m before the seat, faced 180 deg; stepping round 0.45 m before it her toes met the chair's legs; sat from 0.45 m back the director read her 0.55 m pelvis as a fall - judged against the keyframe now; the plop from 0.6 m untried |
| lie | at the bed's foot, faced; lain across it (0.9 m wide) she rolled off its far side; lain along it from its foot, 0.7 m back, she tumbled onto it |

- The lamp wants her hand placed at a point - the arm's two joints solved
  for the switch, or the reach as she walks the last 0.4 m -; a sit and a
  lie a step back to the seat, or the pelvis let down on the law
  (docs/TODO.md 32).
- The waypoint loops (`room.WAYS`) went with the turn. An approach from the
  table's side ran under the table: the chair is come at along the gap
  between them. Stopped at the approach point from the walk's row she fell,
  dwelt on that row she overran the switch's target 0.5 m: the slow row from
  the approach point on.
- Positive heading is toward +x; the stand's row (-1) never hands her to the
  law: a word asks the walk's row (`errands.ask`).
- A probe that stepped the director twice in one millisecond at the ask
  felled every walk at 3.5 s, on the last commit too: the law's take-up from
  the stand tolerates no doubled pass.
