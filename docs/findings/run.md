# Findings: run

A bounce a foot, flight between (`machine/runner.py`, `tools/sim/run.py`),
and its demand on the drives. The walk: [walk](walk.md); the drives' sizing:
[drives](drives.md), [stacks](stacks.md).

## Speeds

The one stack as built, 48 V, from a flight at its speed, a minute each
(2026-10-05):

| Asked m/s | 1.25 | 1.5 | 1.75 | 2.0 |
| --- | --- | --- | --- | --- |
| held m/s | 1.27 | 1.49 | 1.72 | 1.94 |

- 287-253 J/m, 363-491 W; 3.1-3.3 steps/s, a stance 0.22 s, a flight
  0.08-0.10 s; tipped 9.0 deg at most; a sole 591-866 N.
- Eight starts nudged (a mm, 0.01 m/s, 0.05 deg) at 2.0 and four at 1.25: all
  ran 20 s.
- Asked 2.25: down in 10-16 s, the speed running away past it (2.3-2.7 m/s,
  the feet 0.25-0.33 m ahead); begun at 2.5 or 3.0, within 2 s.
- Down to 0.56 m/s it jogs on the spot, 533 J/m. Before it no gait had a
  flight: the walk at 1.2 and 1.4 strides/s is down at 10 s.

## Demand at 2.0 m/s

`drive_sizes --cached --run` folds it in (2026-10-05):

| Joint | rms N m | Held by its winding | T at 1.5x |
| --- | --- | --- | --- |
| hip | 47 | 66 | 1.17 |
| knee | 40 | - | 0.82 |
| ankle | 51 | 119 | 0.42 |

- The hip's and the knee's peaks are their 124 N m clamp, passes of 1-7 ms.
  It runs the same on boards of 40 A (99 N m; 1.92 m/s, 247 J/m) and, at 1.5
  m/s, of 35 (87 N m). 35 A at 2.0: down at the start; 70 and 100 A: down in 3
  s, the setpoints' kinks given the room.
- The knee turns 776-860 deg/s of the 864 its winding has at 48 V, braking;
  at 36, 40 and 55 V it runs the same; at 63 V it fell once at 9 s.
- The literature's 2 m/s per kg asks 18, 30 and 34 N m rms of the hip, the
  knee and the ankle: the run is wasteful, 253 J/m, 0.91 of its weight a
  metre.
- Where the torque goes (a tenth of the stance or the swing a row):

| Joint | Heat | At the clamp, ms a stride |
| --- | --- | --- |
| hip | 37 % in the last three tenths of its swing, 11 % in the stance's first | landing 5, swing's end 15 |
| knee | 40 % in the stance's last four, 40-55 deg bent pushing off; 15 % in its first; 16 % in the swing's first (the fold begun at full rate) | the fold's start 7, landing 4 |
| ankle | 44 % in the stance's middle (124 N m rms, the weight on the ball); 25 % in the swing's last three tenths | the fold's start 4, landing 5, swing's end 11 |

## Landing

A foot comes down 0.3-1.1 m/s on over the floor (4.1 after a short flight)
and at the body's sink, 0.4-1.2 m/s, its knee still unfolding 450 deg/s 65
ms before. It slides 65 mm in 60 ms bearing 10-30 N, and the weight comes on
it late: 650-800 N at 120 ms where the plan has 480, and 270 at 40. The pelvis
13-25 mm under its plan and rising 0.55-0.8 m/s at take-off where 0.49 is
asked; the flights long and short by turns (0.07-0.13 s).

## Tried and left

2026-10-05:

| Change | Result |
| --- | --- |
| the setpoints through a second order | down at 0.6 and 1.8 s at 30 and 45 Hz; the clamps as before at 60 and 100 |
| the fold with no rate at its ends, the leg ready 40-100 ms before its landing | down within 8 s, the unfolding leg under the body in the other leg's dip |
| the ball brought down on the floor's own point, 5 cm over it 80 ms before | the weight on it at once (377 N at 21 ms), the rise 0.48 m/s, but the feet 0.10 m ahead of their place, the hip's servo answering 5 Hz with the leg on it (900 N m/rad on 0.8 kg m^2), the speed 1.0-2.6 m/s |
| the speed's law without its integral | 1.57 m/s asked 2.0; the feet's neutral point 0.13-0.16 m ahead at 1.2-2.0 m/s where the law has half the way over it, 0.13-0.22 |

## On the motors as bought

The U8's rotor 2.5 times the frame's the run was found on (2026-10-05):

| Stance s | Result |
| --- | --- |
| 0.22 | holds 1.23-1.46 m/s at 403-451 J/m; down at 1.75 and 2.0 asked, whatever of its rotor the board feeds forward and at 1.15-1.6 of its peak torque (on the old rotor alone it ran 1.83) |
| 0.30 | half a minute at 0.72, 1.01, 1.22, 1.41 and 1.65 m/s asked 0.75-1.75, 357-267 J/m; the knee 635 deg/s of 960, its rms 49 N m of 74 (T 0.99 at 1.5x); asked 2.0, down in 8-25 s |
| 0.26, taken from 1.5 to 2.0 | 30 s at 1.91 m/s, 282 J/m |
| 0.32 | down at 10.6 s |

The stance should shorten with speed: about half a metre of floor a stance.

## Joined to the walk

2026-10-05; in the lab, never on the page.

- Into the run: from the walk's single support at 1.05 strides/s, the pelvis
  0.10 m past the standing ball, that leg pushes off in 0.15 s: on in 3
  timings of 4.
- Back: the walk begun again at a landing, in mid-stance, blended over
  0.1-1.0 s, at 0.9 m/s, after a last stance with no push to the walk's
  height, or with the other foot set down ahead as the walk lands one: down
  25 times of 25, 0.1-0.5 s after it. The walk's plan carries the pelvis to
  where its phase has it, 0.84 -> 2.39 m/s in 70 ms, a hip asked from -31 to
  +6 deg in 80.
- Two generators, a state each: no hand-over between them holds
  (docs/TODO.md item 28).
