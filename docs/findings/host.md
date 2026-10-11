# Findings: host

Pages, tooling, suites and the local model's runner. The board's findings:
[FINDINGS](../FINDINGS.md).

## Tooling and suites

- A file read whole past 6 k tokens costs a tenth of a window (38 of ours,
  test_views 32 k): split by subject (split_suite.py), the rest capped, held
  by test_structure and the pre-commit hook (2026-09-28).
- Offline gate: 544 s at 17 % busy, a job a suite; one relay with shards past
  half its work over the batons: 224 s at 31 % (2026-09-28).
- The offline gate was 400 s of sleeping on the stand-in's clock; suites run
  four at a time: 142 s.
- A FAIL behind a page's footer or screen clear went unnamed: the runner takes
  a line's first marker, escapes stripped (2026-09-28).
- numpy's OpenBLAS pool costs ~499 MB commit per process; this laptop has no
  page file. Capped in `coaxial.model.blocks`.
- `UL` is 64-bit on Linux: `-Wconversion` warned on CI only.
- `env.ps1` dot-sourced into `coaxial_tty.ps1`: its `foreach ($name ...)` was
  the caller's `[ValidateSet] $Name` and failed it; the loop variable is
  `$bundle` (2026-09-23).
- With the bench attached the gate's test_mcp ran live, 397 s and 9 red:
  `program` scanned 16 units on the port and its rig's close wrote to the
  session's closed broker client, a ValueError past every
  `suppress(RigError)`; an AFE read behind a GPIOB write said on=1, the
  rail's users having raised AFE_ON again. Fix: the fleet's checks on the
  stand-in, the pin's on the write's own readback, the closed client a
  ConnectError: 59 in 28 s (2026-10-05).
- `test_sensorless` 20 of 20, 136 checks each, and `test_daq_api` 20 of 20,
  82 each, on the relay beside a quad grid (2026-10-10): the servo's 1 gate in
  4 and the first dt's failure of 2026-10-05 not seen.
- A pull rebuilds (2026-10-10): the terminal, opened in a console, asks each
  tree's ninja what it would build (0.04 s) and builds Debug and Release where
  either is behind, Release last, so `open()` loads the newest
  (tools/target/current.py). The host's C libraries are built in each process
  already. A source touched: both built in 3.9 s.

## Armada

`tools/sim/armada.py`, `tools/sim/replay.py` (2026-10-05): a known law
walking, the unknown swapped in, back to a known state at a fall, no
kilowatts spent on repetitions.

- The world, the boards' processes, the loop and the director marked between
  passes and restored: the walk restored identical to the last digit of the
  pose and 332.9 W over 3 s. The boards held at the mark's setpoints as at a
  reset instead: 382 W.
- A candidate at three paces 12 s on robots kept up; 30 trials 28 s on 12
  robots; a robot up and marked at three paces in 10 s. From the squat a run
  was 60-90 s.
- The law's modules reloaded under a walking robot: a constant changed in
  stance.py and back, the walk's numbers there and back to the digit.
- A file the host's scanner held killed a robot at a rename, and 8 of 12 in a
  minute at their signs of life: each retried now.

## Stand-in on the board's C

- Profile (2026-10-10, cProfile, the QUAD course on four boards): 19.4 s, 52 M
  calls; the stand-in's thermal mirror 12 M of them: the truth's integration
  11.5 s cumulative, the identification's propagation 8.1, `net_flows` 157 k
  calls, 19 M dict reads; MuJoCo 0.19.
- On the C (2026-10-10): its observer, identification and envelope
  `thermal_run.c` (the board's own since 1f3ad19) on a truth that is
  `world_heat.c`, a slot of `world_stand.c`'s; the Python a thin edge, its
  three mirror modules gone, its replies in the wire's units. A QUAD course on
  four stand-ins 19.4 -> 5.1 s, 52 -> 10 M calls, laps 14.18 and 14.96 s;
  test_simulated 60 -> 18 s.
- The rotor demo on the board's envelope (2026-10-10): the stand-in on the C
  throttles as the Python copy did (the derate within 0.02 at 36 and 70 A),
  but the page's rhythm moved: a dip under the held load slowed the rotor
  under SEND_FROM, its 10 A short of the load; stalled, the switches at 15-18
  % where 20. The dynamometer eased by the derate, as an operator backs a
  brake off; a spin's ramp up 4 s where 3.5 ended at 13 A. Green 4 of 4.

## Pages

- A `Feed` slept its period after the read: a 20 ms read at 50 ms fed 14
  readings a second to a 20 fps page. Start to start now (2026-09-28).
- HUMANOID's R crashed the page at once on 2026-10-03 with no traceback
  kept; headless it took R and saved 97 rows then, and R starting, three
  states and R saving is test_views_humanoid.py's since 2026-10-11. The
  suspect stands: a page started before `buses.FIELDS` changed (play and
  flex then; delta, delta_d and ahead on 2026-10-11) reads a block of
  another layout - a page is restarted after a pull.
- THERMAL OBSERVER said `AFE off` with AFE_ON high: no reading yet, the first
  sample is 30 s after opening. It says which now (2026-09-28).
- THERMAL OBSERVER and the side columns, a review's 29 findings against the
  refit (2026-10-06):
  + `--switch` armed a real board with the break bypassed; now AFE_ON off,
    the latch cleared, the break in circuit, AFE_ON restored as found.
  + The map's centre patch was stripped, its middle 9.6 K off.
  + A hud counted one row since 2026-09-27 and the column never paged: the
    page's 78 lines cut at 42; SESSION's THERMAL 17 of its 20 nodes.
  + TUBES 37 cells in 36, the NTC's tube an ellipsis; ten rows on three pages
    wider than their boxes. The boxes: `terminal/views/thermal/boxes.py`.
  + The stand-in's budget never had seconds to its limit; its load cycle put
    45 A through the hot swap.
  + The trip's cap was asked equal to 1e-6 of a margin trimmed in 1e-3.
  + SENSE's `err` read +9 K on a model 0.1 K out.
  + The real board's `--switch` on the bench at 23.9 V: armed with the break
    in circuit, three legs at 50 % for 5 s (the observer's drivers 31 -> 41 C
    blind, the thermistor 0.9 K up after), disarmed, AFE_ON off as found.
- QUAD's pass on four stand-ins was 5.1 ms of 20, 16.2-17.0 frames a second
  on a lap: a slice's envelope weighed the identification's doubt a node, 42
  times (0.6 ms a pass), and the frame's forces were crossed a rotor a step in
  the world's frame, 40 crosses (1.0 ms). One margin a walk of the nodes and
  the four's torque once a pass about the frame's own axes (the frame the
  same to 1e-13 over 8 s of tumbling): 3.4 ms, 17.9-18.8 frames, 19.2 with
  nothing flown (2026-10-06).
- QUAD's two plots inked and marked alike: a plot's left curve and its scale
  one ink, its right another, the bus a mark a row as the height (orange and
  red had been the bus's and the temperature's) (2026-10-06).
- The board's raster on a GTX 1080 Ti (wgpu, Vulkan): 392x224 100 ms -> 2.1
  ms; the menu's turntable 64 -> 13 ms at 52x18. BOARD ATTITUDE 66 -> 64 ms
  a frame at 200x60: its ground (32 ms) and paint (15) are the frame now
  (2026-09-25).
- The front page's model was drawn at inner height - 2 inside a 1-column
  padding: a blank row top and bottom, a column each wall. Now the box's
  full inside (2026-09-23, checked at 90x28, 120x36, 200x60).
- The front page's stand shows the quad after the gynoid, 9 s alone in a
  hover and once over its nose: 10 ms a frame on the card, 69 on the CPU,
  where the stand keeps to the board (`terminal/stand.py`); QUAD's row had no
  kana (2026-10-05).
- Rotor observer at a 200x60 terminal (can 95 dots, tuned at 21): the ring
  stroke grew to 3.0 dots half-width, pulled teeth floated loose. Stroke
  capped at 1.0 (0.8 broke into dots), the undriven tooth length drawn as
  track, kept out of any cell an area holds (2026-09-23).
- One DC bus connector's screw hole drew as a box, then vanished into the
  connector (724f950 absorbs nested blocks): the circle test centred on the
  centroid and 14 unevenly spaced points read dev 0.047 (limit 0.03).
  Least-squares centre: a drum, like the other four; no other primitive moved
  (2026-09-23).
- BOARD ATTITUDE face down drew the top's parts on the underside. The
  outline's grace adds the cell's depth span, and a tilted face spans
  0.036-0.08 a cell against a 0.032 slab: 533 of 535 drawn dots sat behind
  it. An edge on the slab's far face gets the fixed grace only; the face art
  (the top's layout) is not read from behind (2026-09-23, rasters face down,
  up, tilted).
- The tty's face (2026-10-10; VS Code 1.141's xterm 6.1 in Edge 154, WebGL,
  125 %): a cell is the face's W wide, its braille and boxes drawn to fill
  it.

| Face | Cell, device px | Note |
| --- | --- | --- |
| Consolas 14 | 9 x 22, 2.44 | 205 x 43 cells |
| Eurostile 14 | 17 x 19 | text strewn |
| Eurostile 12 px, lines 1.3, letters -3 | 11 x 22, 2:1 | read as Extended |
| Eurostile, letters -5 (VS Code's least), lines 1.2 | 9 x 20 | 205 x 47 cells |

  Latin glyphs past their cell overlap it, never rescaled. Too wide and
  uneven either way: back on VS Code's own. DTC's Eurostile Extended reaches
  Chromium as Times New Roman: a cmap subtable truncated.

## Local model

- Model weights (7.6 GB) reloaded per suite were most of a run: loaded once,
  released once. A run killed from outside leaves 8.4 GB on the card.
- Ollama answered 500 from 2026-09-03 to 09-12: the runner failed to start.
- An answer at 40 tokens/s (3 lines, 2.5 s of motion): first move 0.21 s fed
  a line at a time, 0.66 s sent whole. A `wait` wake costs 15 tokens after
  the first full `now` (~120); the humanoid prompt 1 360 characters. Every
  pass wrote unset outputs as 0, opening the stand-in pack's contactor; unset
  now holds what the node reads (2026-09-24).
- A step waits for its targets (1 % of their range) or its tests, its seconds
  a timeout; L and H alarm, logged as they come and go with a 1 % deadband
  against chatter; LL and HH trip; all in `machine.alarms`, beside the
  sequencer. The squat's down ends on arrival in 0.6-0.7 s of its 2; the
  scan's timeouts are answers, not alarms (it branches). The humanoid prompt
  1 434 characters (2026-09-24).
- Warmth asked reads the thermal observer, every node: the intent's thermal
  kind (2026-10-10). On llama3.1:8b on the CPU (Ollama 0.35.1's CUDA runner
  failed its PTX JIT on the RTX 4060's driver 617.42): three warmth rows
  called `thermal`, the NTC's still `analog_read`, 19-28 s a row.
