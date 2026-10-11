# The living room

The room round her on the HUMANOID page (`machine.room`, `physics.ROOM`): glass
walls 6 x 5 x 2.4 m that do not break (static boxes with the floor's contacts,
drawn as edges in glass's ink), a chair facing -z at (1.5, 0.8), a bed at
(-1.8, 1.8), a table at (1.5, -0.8), a floor lamp at (-2.5, -1.0) with its
switch on the left wall at 1.05 m, and a door 0.9 m wide in the far wall hung
on a hinge at x 0.3, swinging outward to 110 deg against 3 N m s/rad: 15 N m
opens it fully in 1 s. Her hand on the switch toggles the lamp (`room.touched`,
not twice within 0.6 s). The page draws its 22 boxes as wireframes; the door
where it swings; the lamp lit.

## The words

`room.WORDS` is text, compiled to rows (`machine.kin`) and met a row at a
time (`machine.errands`); a place is one of `room.FRAMES` - where a thing is
used from: x, z, the heading held there, its latch's side - and metres along
that heading. On the page she stands in the room on the law till a word is
asked: U sit, I lie, E through the door (in again from outside it), Y the
lamp; U or I again is up.

| Line | Asks | Met |
| --- | --- | --- |
| `go PLACE` | her way walked there round what she knows ([sight](sight.md)) | there; lines of `go` are one way |
| `face PLACE` | standing, her heading turned to the place's own | faced, her stand wanting no step |
| `stand` | the stand's row | settled 1 s |
| `pace ROW` | the row of her way: stand, walk, jog, run | at once |
| `hand LIMB at PLACE` | a hand raised before her within 1.3 m of the place | it holds while the word runs |
| `reach LIMB` | an arm raised till what it meets answers | the arm down again |
| `play NAME` | the arrival's keyframes of that name | the last frame held |

Her way is walked by `machine.pursuit`: aimed 0.8 m ahead on it, the heading
turned 0.3 rad/s at most ([turns](turns.md)); asked more than 0.5 rad off it,
and at a corner sharper than that, she turns standing; walking she stops to
turn only 1.0 rad off it.

| Word | Outcome |
| --- | --- |
| out, from where the page stands her | through the door 0.28-0.44 m from the nearer post, and on (test_room.py) |
| out, from within 0.3 m of there, facing within 25 deg | 6 of 12 through and standing |
| out, from anywhere, facing anywhere | 1 of 12 |
| lamp | at the switch 0.43 m off, faced on it within 6 deg; the arm raised 40 deg/s meets the wall under the switch and the push fells her: its arc crosses the switch's 8 cm only 0.39-0.44 m off it, the halt scatters her 0.1-0.3 |
| sit | at 0.6 m before the seat, faced 180 deg; stepping round 0.45 m before it her toes met the chair's legs; the plop from 0.6 m untried |
| lie | at the bed's foot, faced; lain across it (0.9 m wide) she rolled off its far side; lain along it from its foot, 0.7 m back, she tumbled onto it |

What fells her going out, traced a contact and a setpoint a pass:

- The door. Her latch-side hand meets the leaf at 0.8 m/s, 36-47 N; it is at
  25-50 deg as her body comes through, and her other hand meets it 0.02-0.17
  m from the hinge, 50-63 N for 0.45 s, where it does not give: she stops in
  the doorway. Both hands raised fell her; with none the latch's post did.
- Stopped while a speed is asked she stands on both feet, her capture point
  behind them, and goes over backward in 1.2 s: the step back is the
  stand's alone (`hold.staying`).
- A corner walked leaves her 0.2-0.3 m off the door's line 2 m on: her raised
  hand meets the wall by the latch.
- A corner met standing: the halt carries her 0.49 m on the open floor, up to
  1 m in the room, and she walks back to her way.

- The lamp wants her hand placed at a point - the arm's two joints solved
  for the switch, or the reach as she walks the last 0.4 m -; a sit and a
  lie a step back to the seat, or the pelvis let down on the law.
- Below the walk's row the law bends the knee for its short steps: landing
  knees 10.8 deg at the walk's row, 12.1 at -0.3, 15.5 at -0.45, 19.9 at
  -0.6, a crouch. Her ways are walked on the walk's row.
- Standing in the room she is on the law's stand row, upright (the walk's
  4.9 deg of lean read as a stoop), her shoulders 8 deg back. P is a push of
  80 N: standing she parries it from either side, and a nudge of 38; 120
  fells her. Walking she parries 38; 60 and 80 fell her.
- Positive heading is toward +x.
- A probe that stepped the director twice in one millisecond at the ask
  felled every walk at 3.5 s: the law's take-up from the stand tolerates no
  doubled pass.

## Walls

Walking on with no word, the stand's row is asked where her map bars the way
0.4-1.1 m before her (`errands.step`): she stands 0.94 m before the right wall
and 0.9 m before the far one. Without her eyes, walked into the near wall:

| Row | At the wall |
| --- | --- |
| the slow one, 0.3 m/s | her toes meet it at 5.8 s; she stands at it treading, 0.17 m off its face, 16 s on |
| the walk's, 0.76 m/s | her toes meet it at 3.9 s, she is thrown back at 1.5 m/s and falls at 5.1 s |

- A rule stepping her back when thrown back past 0.3 m/s was a case of its
  own and went: asked every pass it twisted her 73 deg in nine steps, and at
  the walk's row her torso met the wall whatever her feet did.
