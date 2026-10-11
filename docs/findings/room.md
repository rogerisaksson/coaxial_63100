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
each a target (x, z) and a facing, walked to on the one law and done there as
keyframes from her pose as read (`machine.errands`, keys U I E Y, U or I again
up). What the law can do of it, measured (`scratchpad/errand_probe.py`):

| Asked | Outcome |
| --- | --- |
| the walk's row straight ahead in the room | walks to the far wall and falls into it at z 3.2 m, 7 s |
| the law's heading turned 0.05 rad/s on the open floor | walks 16 s turning, 40 deg |
| 0.1 rad/s | falls at 11 s, 52 deg turned |
| 0.15 rad/s | falls at 5 s |
| out (bearing 10 deg, the door 4 m on) at 0.05 rad/s from 2 s into the walk | 7 deg turned when the wall stops her at 7 s |
| lamp (a loop of waypoints, 150 deg round) | the same wall at 7 s |

- The law walks a curve no tighter than 10 m (0.5 m/s at 0.05 rad/s) and has
  no turn on the spot: no word's target in a 6 x 5 m room is reached. The
  turn is the law's missing row (docs/TODO.md 38); the room, its door, its
  lamp and the words' plumbing wait on it.
- Positive heading is toward +x; the stand's row (-1) never hands her to the
  law: a word asks the walk's row (`errands.ask`).
- A probe that stepped the director twice in one millisecond at the ask
  felled every walk at 3.5 s, on the last commit too: the law's take-up from
  the stand tolerates no doubled pass.
