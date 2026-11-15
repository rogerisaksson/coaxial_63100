# Findings: situation

Where the robot is, read from its sensors each pass
(`machine/senses/situation.py`), and the text that fits it laid over the robot's
law (`lisp/humanoid/situations.lisp`); tests/test_situation.py holds it.

- Six readings the robot's law has anyway: the floor's grip, the least either
  sole has felt of it; the floor's rise a metre along the robot's way under the
  robot's standing soles; the share of its weight borne off the robot's
  soles; its pelvis's up along the world's; the push on the robot's
  above its soles, of its weight; how high an edge met the
  robot's swings. A situation is bounds on them, `(defsituation NAME (READING
  below|above X) ..)`, entered over the fifth inside each bound, and the caps it
  lays on the robot's rhythm's amplitudes, blended in as far as it is in
  it (`law.gripped`). Seven named: slippery under 0.25 of grip with `ice_walk`'s
  caps - the surfaces' text before it -, uphill and downhill past 0.08, seated
  and lying by half the robot's weight borne and its pelvis's upright at
  0.5, held by a fiftieth of its weight on it, stepping over 2
  cm.
- Read as the robot's standing soles' mean, the ice was told 1.24 s after a sole
  landed on it: the left slid at 2.30 s and read 0.101, swung, and the right
  stood alone on 0.6 till the left stood alone again at 2.95. The least of the
  two tells it as the slide is read.
- Entered unannounced, by the world's own contacts, at 4 lays of 4 (0, 2e-4,
  -2e-4, 4e-4 rad): the ice told slippery 0.56-0.57 s after a sole lands on it
  bearing 160 N (1.71 s; its slide read at 2.30); the ramp uphill 0.16 s after
  a sole lands on it (2.23 s); the chair seated 1 ms after its seat bears half
  the robot's weight (1.82 s); the wall held 2 ms after 22 N on it (2.21 s).
- Told at the slide, the unaware ice's caps hold the robot 0.65 s sooner: in
  the scene's 6 s of walking it goes 1.88 m where 2.17, standing, at 13
  of 13 lays; its goal 2.0 -> 1.8 m. The other 21 held scenes, the 12 replays
  and the robot's flat walk the same to the digit: 51 s, 0.70 m/s, 408 J/m, 2.55
  off the band.
