# Findings: her run

The gynoid running: a bounce a foot, flight between (`machine/runner.py`,
`tools/sim/run.py`), and what it asks of her drives. Her walk is in
[walk](walk.md), her drives' sizing in [drives](drives.md) and
[stacks](stacks.md).

- She runs (2026-10-05; the one stack as built, 48 V, from a flight at her
  speed, a minute each): asked 1.25, 1.5, 1.75 and 2.0 m/s she holds 1.27,
  1.49, 1.72 and 1.94, 287-253 J/m, 363-491 W; 3.1-3.3 steps/s, a stance
  0.22 s, a flight 0.08-0.10; tipped 9.0 deg at most, a sole 591-866 N.
  Eight starts nudged (a mm, 0.01 m/s, 0.05 deg) at 2.0 and four at 1.25
  all ran 20 s. Asked 2.25 she is down in 10-16 s, her speed running away
  past it (2.3-2.7 m/s, the feet 0.25-0.33 m ahead); begun at 2.5 or 3.0,
  within 2 s. Down to 0.56 m/s she jogs on the spot, 533 J/m. Before it no
  gait of hers had a flight: the walk at 1.2 and 1.4 strides/s is down at
  10 s.
- What it asks at 2.0 m/s (`drive_sizes --cached --run` folds it in): the
  hip's rms 47 N m of the 66 its winding holds (T 1.17 at 1.5x), the
  knee's 40 (0.82), the ankle's 51 of 119 (0.42); the hip's and the knee's
  peaks their 124 N m clamp, passes of 1-7 ms - she runs the same on boards
  of 40 A (99 N m; 1.92 m/s, 247 J/m) and, at 1.5 m/s, of 35 (87 N m). 35
  A at 2.0 has her down at the start; 70 and 100 A throw her down in 3 s,
  her setpoints' kinks given the room. The knee turns 776-860 deg/s of the
  864 its winding has at 48 V, braking; at 36, 40 and 55 V she runs the
  same, at 63 V she fell once at 9 s. The literature's 2 m/s a kg asked
  18, 30 and 34 N m rms of the hip, the knee and the ankle: she runs
  wastefully, 253 J/m 0.91 of her weight a metre.
- Where the torque goes (a tenth of the stance or the swing a row, 2.0
  m/s): the hip's heat 37 % in the last three tenths of its swing, 11 % in
  the stance's first; the knee's 40 % in the stance's last four, 40-55 deg
  bent pushing off, 15 % in its first, 16 % in the swing's first - the
  fold begun at full rate; the ankle's 44 % in the stance's middle (124 N
  m rms, her weight on the ball), 25 % in the swing's last three tenths.
  At their clamps, ms a stride: the fold's start (knee 7, ankle 4), the
  landing (hip 5, knee 4, ankle 5), the swing's end (hip 15, ankle 11).
- How a foot comes down: 0.3-1.1 m/s on over the floor (4.1 after a short
  flight) and at her sink, 0.4-1.2 m/s, its knee still unfolding 450 deg/s
  65 ms before. It slides 65 mm in 60 ms bearing 10-30 N, and her weight
  comes on it late - 650-800 N at 120 ms where her plan has 480, and 270
  at 40 -, the pelvis 13-25 mm under its plan and rising 0.55-0.8 m/s at
  take-off where 0.49 is asked, her flights long and short by turns
  (0.07-0.13 s).
- Tried and left (2026-10-05). The setpoints through a second order: down
  at 0.6 and 1.8 s at 30 and 45 Hz, the clamps as before at 60 and 100.
  The fold with no rate at its ends, and the leg ready 40-100 ms before
  its landing: down within 8 s, the unfolding leg under her in the other
  leg's dip. The ball brought down on the floor's own point, 5 cm over it
  80 ms before: her weight on it at once (377 N at 21 ms), her rise 0.48
  m/s - and her feet 0.10 m ahead of their place, the hip's servo
  answering 5 Hz with the leg on it (900 N m/rad on 0.8 kg m^2), her speed
  1.0-2.6 m/s. Her speed's law without its integral: 1.57 m/s asked 2.0,
  her feet's neutral point 0.13-0.16 m ahead at 1.2-2.0 m/s where the law
  has half her way over it, 0.13-0.22.
- On her motors as bought (2026-10-05; the U8's rotor 2.5 times the frame's
  the run was found on): a stance of 0.22 s holds 1.23-1.46 m/s at 403-451
  J/m and is down at 1.75 and 2.0 asked, whatever of its rotor the board
  feeds forward and at 1.15-1.6 of its peak torque; on the old rotor alone
  she ran 1.83. At 0.30 s she runs half a minute at 0.72, 1.01, 1.22, 1.41
  and 1.65 m/s asked 0.75-1.75, 357-267 J/m, the knee 635 deg/s of 960 and
  its rms 49 N m of 74 (T 0.99 at 1.5x); asked 2.0 she is down in 8-25 s.
  At 0.26 s, taken from 1.5 to 2.0, she ran 30 s at 1.91 m/s, 282 J/m; at
  0.32, down at 10.6 s: the stance wants to shorten with her speed, about
  half a metre of floor a stance.
