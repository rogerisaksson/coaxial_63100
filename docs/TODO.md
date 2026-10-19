# TODO

Open work. An item is a gap and its DOD (done when); its measurements are in
the files under docs/FINDINGS.md. An item is deleted when its DOD is met and
gets a line at the end of [DONE](DONE.md). Subjects off the gynoid's list:
the table under [Subjects](#subjects).

## Goal

- A running biped of minimal mechatronic complexity with headroom in its
  dynamic range: the fewest motor types and gearboxes; every joint
  air-cooled, the board's back on an aluminium holder over a third of it, on
  its motor or gearbox, no other cooling.
- Drives sized in the gym's scenes for simplicity, DFM and printing: few
  variants of toleranced parts (gearboxes, electronics); stock lengths,
  linkages and ball bearings.
- End to end: the LLM writes text of balance, rhythm and elasticity, the gynoid
  humanoid robot latent loop runs it, its torques go over the bus to the boards'
  firmware and drives, emulated or on the bench.
- Under the text no rule a case and no branch: flat static arrays, a mode a
  spring, a landing read from the gynoid humanoid robot capture point.
- A special case means the problem needs a more general form.

## Subjects

| Subject | File |
| --- | --- |
| Scenarios, by the general case: each a test in MuJoCo, in order | [todo/scenarios.md](todo/scenarios.md) |
| The bench: what the board has still to show | [todo/bench.md](todo/bench.md) |
| Flight pages: the crafts in wind, the aerobat's landing | [todo/flight.md](todo/flight.md) |
| Host: native://, identification on the robot, her law compiled | [todo/host.md](todo/host.md) |
