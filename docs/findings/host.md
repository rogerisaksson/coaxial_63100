# Findings: host

The host's pages, tooling, suites and the local model's runner. The board's
own are in [FINDINGS](../FINDINGS.md).

- A file read whole past 6 k tokens costs a tenth of a window (38 of ours,
  test_views 32 k): split by subject (split_suite.py), the rest capped, held by
  test_structure and the pre-commit hook (2026-09-28).
- The offline gate: 544 s at 17 % busy, a job a suite; one relay, shards past
  half its work over the batons: 224 s at 31 % (2026-09-28).
- A FAIL behind a page's footer or screen clear went unnamed: the runner takes
  a line's first marker, escapes stripped (2026-09-28).
- A `Feed` slept its period after the read: a 20 ms read at 50 ms fed 14
  readings a second to a 20 fps page. Start to start now (2026-09-28).
- THERMAL OBSERVER said `AFE off` with AFE_ON high: it had no reading because
  the first sample is 30 s after opening. It says which now (2026-09-28).
- THERMAL OBSERVER and the pages' side columns, a review's 29 findings against
  the refit: `--switch` armed a real board with the break bypassed - AFE_ON
  off, the latch cleared, the break in circuit now, AFE_ON put back as found;
  the map's centre patch was stripped, its middle 9.6 K off; a hud counted
  one row since 2026-09-27 and the column never paged - the page's 78 lines
  cut at 42, SESSION's THERMAL 17 of its 20 nodes; TUBES 37 cells in 36, the
  NTC's tube an ellipsis; ten rows on three pages wider than their boxes;
  the stand-in's budget never had seconds to its limit, and its load cycle
  put 45 A through the hot swap; the trip's cap was asked equal to 1e-6 of a
  margin trimmed in 1e-3; SENSE's `err` read +9 K on a model 0.1 K out. The
  boxes are `terminal/views/thermal/boxes.py`. The real board's `--switch`
  on the bench at 23.9 V: armed with the break in circuit, three legs at
  50 % for 5 s - the observer's drivers 31 to 41 C blind, the thermistor
  0.9 K up after -, disarmed, AFE_ON off as found (2026-10-06).
- QUAD's pass on its four stand-ins was 5.1 ms of 20, its frames 16.2-17.0 a
  second on a lap: a slice's envelope weighed the identification's doubt a
  node, 42 times (0.6 ms a pass), and the frame's forces were crossed a rotor
  a step in the world's frame, 40 crosses (1.0 ms). One margin a walk of the
  nodes, the four's torque once a pass about the frame's own axes - the frame
  the same to 1e-13 over 8 s of tumbling: 3.4 ms, 17.9-18.8 frames, 19.2
  with nothing flown (2026-10-06).
- Model weights (7.6 GB) reloaded per suite were most of a run: loaded once,
  released once. A run killed from outside leaves 8.4 GB on the card.
- The offline gate was 400 s of sleeping on the stand-in's clock; suites run
  four at a time: 142 s.
- numpy's OpenBLAS pool costs ~499 MB commit per process; this laptop has no
  page file. Capped in `coaxial.model.blocks`.
- The board's raster on a GTX 1080 Ti (wgpu, Vulkan): 392x224 100 ms -> 2.1 ms;
  the menu's turntable 64 -> 13 ms at 52x18. BOARD ATTITUDE 66 -> 64 ms a
  frame at 200x60: its ground (32 ms) and paint (15) are the frame now
  (2026-09-25).
- `UL` is 64-bit on Linux: `-Wconversion` warned on CI only.
- Ollama answered 500 from 2026-09-03 to 09-12: the runner failed to start.
- Front page model drawn at inner height - 2 and inside a 1-column padding:
  a blank row top and bottom, a column each wall. Now the box's full
  inside (2026-09-23, checked at 90x28, 120x36, 200x60).
- The front page's stand shows the quad after her, 9 s alone in a hover and
  once over its nose: 10 ms a frame on the card, 69 on the CPU, where the
  stand keeps to the board (`terminal/stand.py`); QUAD's row had no kana
  (2026-10-05).
- Rotor observer at a 200x60 terminal (can 95 dots, tuned at 21): ring stroke
  grew to 3.0 dots half-width, pulled teeth floated loose. Stroke capped
  at 1.0 (0.8 broke into dots), undriven tooth length drawn as track,
  kept out of any cell an area holds (2026-09-23).
- `env.ps1` dot-sourced into `coaxial_tty.ps1`: its `foreach ($name ...)` was
  the caller's `[ValidateSet] $Name` and failed it; the loop is `$bundle`
  (2026-09-23).
- One DC bus connector's screw hole drew as a box, then vanished into the
  connector (724f950 absorbs nested blocks): the circle test centred on the
  centroid and 14 unevenly spaced points read dev 0.047 (limit 0.03).
  Least-squares centre: a drum, like the other four; no other primitive moved
  (2026-09-23).
- BOARD ATTITUDE face down drew the top's parts on the underside. The
  outline's grace adds the cell's depth span, and a tilted face spans
  0.036-0.08 a cell against a 0.032 slab: 533 of 535 drawn dots sat
  behind it. An edge on the slab's far face now gets the fixed grace
  only; the face art (the top's layout) is not read from behind
  (2026-09-23, rasters face down, up, tilted).
- A model's answer at 40 tokens/s (3 lines, 2.5 s of motion): first move 0.21 s
  fed a line at a time, 0.66 s sent whole. A `wait` wake costs 15 tokens after
  the first full `now` (~120); the humanoid prompt 1 360 characters. Every pass
  wrote unset outputs as 0, opening the stand-in pack's contactor: unset now
  holds what the node reads (2026-09-24).
- A step waits for its targets (1 % of their range) or its tests, its seconds a
  timeout; L and H alarm, logged as they come and go with a 1 % deadband against
  chatter; LL and HH trip; all in `machine.alarms`, beside the sequencer.
  The squat's down ends on arrival in 0.6-0.7 s of its 2; the scan's timeouts are
  answers, not alarms (it branches). The humanoid prompt: 1 434 characters
  (2026-09-24).
- An armada of her, each walking steady (2026-10-05; the user: a known
  law walking, the unknown swapped in, back to a known state at a fall, no
  kilowatts on repetitions; `tools/sim/armada.py`, `tools/sim/replay.py`):
  her world, her boards' processes, the loop and the director marked
  between passes and gone back to - the walk gone back to the same to the
  last digit of her pose and 332.9 W over 3 s; her boards held at the
  mark's setpoints as at a reset instead, 382 W. A candidate at three
  paces 12 s on robots kept up, 30 trials 28 s on 12 robots, a robot up
  and marked at three paces in 10 s; from the squat a run was 60-90 s. The
  law's modules reloaded under a walking robot: a constant changed in
  stance.py and back, her walk's numbers there and back to the digit. A
  file the host's scanner held killed a robot at a rename and 8 of 12 in a
  minute at their signs of life: each tried again now.
- With the bench attached the gate's test_mcp ran live, 397 s and 9 red:
  `program` scanned 16 units on the port and its rig's close wrote to the
  session's closed broker client, a ValueError past every `suppress(RigError)`;
  an AFE read behind a GPIOB write said on=1, the rail's users having raised
  AFE_ON again. The fleet's checks on the stand-in, the pin's on the write's
  own readback, the closed client a ConnectError: 59 in 28 s (2026-10-05).
