# Findings: host

Pages, tooling, suites and the local model's runner. The board's findings:
[FINDINGS](../FINDINGS.md).

## Tooling and suites

- A file read whole past 6 k tokens costs a tenth of a window (38 of ours,
  test_views 32 k): split by subject (split_suite.py), the rest capped, held
  by test_structure and the pre-commit hook.
- Offline gate: 544 s at 17 % busy, a job a suite; one relay with shards past
  half its work over the batons: 224 s at 31 %.
- test_controller's tied rows gave a knee 0.3 s to 20 deg: on the stand-in
  it takes 0.26 s (0.175 to 10 deg), and CI's runner had it at 17.4 of 20
  and 1.8 of 0 as the row ended, twice in five runs - the row's alarm in
  the failsafe's place. 10 deg in 0.4 s now, 0.225 s to spare as the
  test's other rows.
- The offline gate was 400 s of sleeping on the stand-in's clock; suites run
  four at a time: 142 s.
- The pre-push smoke (`tools/dev/smoke.py`): every `machine` module imported
  and her first 1.5 s on the loop told to walk, 2.7 s; a raise in
  `flight.check_contacts` stopped it in 0.4 s.
- test_controller's 1500 rpm lane on the wall read 1421 on Python 3.12: told
  its seconds (`hostclock.Told`) it reads 1446 every run, the lane ringing
  1420-1590 rpm at 0.7 Hz; its camera is read against the head's own reading
  of the pass before.
- That lane's loop ran at 25 Hz under a 3 Hz crossover, acting a pass and a
  half after it read - 65 deg of phase: its rotor 1447-1549 rpm. At 16 times
  its crossover (48 Hz, `motion.RATE_PER_HZ`) 1472-1520, at 50 Hz 1484-1508.
  The board's sensorless speed ripples 5 % about the rotor's, the lane's
  estimate with it (1412-1577; its model trusted 100 times more, 1467-1524,
  the rotor 1479-1500 and its mean 0.7 % low): a reading at an instant is
  that ripple.
- The ripple is the electrical frequency's, 175 Hz at 1500 rpm, read 48 times
  a second: aliased into a wander the loop chased. The lane's speed is now the
  observer's angle - its turns counted every step - risen over a pass
  (`parts.Rate`, `ctrl.c`'s twin), the pass's mean: the rotor 1489-1503 rpm past
  its ramp, 1488-1501 in test_controller's 3 s. A block's first pass takes no
  time (`Loop.run`): at 1e-6 s the rate read what the drive turned between two
  blocks as 7549 rad/s and braked a rotor at 900 rpm into reverse - the stop in
  test_sensorless, told its seconds, at 98 rpm where 0.5 now. A disturbance state
  in the speed's Kalman, trusting its model: 1481-1504, its mean 0.5 % low; not
  kept.
- CI went red on stand-ins run by the wall: the quad's take-off (-5585 rpm) and
  the rotor demo's climb (1862 rpm of 1870). On 2 cores beside 6 busy loops the
  quad tripped 6 of 6, its 3 Hz speed loop sampled a few times a second. A
  machine's run now tells its stand-ins its seconds (`machine.nodes.Told`): 0
  of 6, 3.0 s a run where 11.7. Told, the first run after the humanoid's
  tripped: its loop closed on the estimate at the switch (86 rpm off at 3 ms,
  4 at 15) and asked 0 of a rotor at 361 rpm. `started` waits 20 ms, a loop
  takes over at its reading: 361 -> 1747 rpm, no dip to 100. The demo's
  segments are told a frame a frame: starved to 8.6 frames a second, 2108 rpm.
- test_parity and test_bench passed 0 of 0 on CI: without a board they skip
  by name (`NEEDS_BOARD`); test_parity on the emulated image, 30 passed.
  Four suites were in no list - test_gvs green, test_house_tour 4 failed,
  test_stunts out of time, test_tps_gameplay 5 failed.
- A kept replay that stood here fell at 10.60 s on CI's runner, Python 3.12 and
  3.14 alike: another platform's floating point. Laid 0.2 mm aside it fell here
  too, at 11.33 s; a kept replay now stands laid 0.2 mm either way. The hop
  after a stop, a fall at 8.97 s when recorded, stands on her law since the
  sensor boundary at every wobble to 1 mm.
- A HUMANOID recording replayed (`tools/sim/replay.py`) through the worker's
  process: 7 commands over 2 epochs, the pelvis 0.0 mm off the recording, a
  400 N shove's fall at 4.162 s again. `stage` read off the gait's amplitude
  said stand at 0.9 m/s; measured, her first pass after a lay read 0 N on
  both soles and walk for a second till a lift counted only off a bearing sole.
- A FAIL behind a page's footer or screen clear went unnamed: the runner takes
  a line's first marker, escapes stripped.
- numpy's OpenBLAS pool costs ~499 MB commit per process; this laptop has no
  page file. Capped in `coaxial.model.blocks`.
- `UL` is 64-bit on Linux: `-Wconversion` warned on CI only.
- `env.ps1` dot-sourced into `coaxial_tty.ps1`: its `foreach ($name ...)` was
  the caller's `[ValidateSet] $Name` and failed it; the loop variable is
  `$bundle`.
- With the bench attached the gate's test_mcp ran live, 397 s and 9 red:
  `program` scanned 16 units on the port and its rig's close wrote to the
  session's closed broker client, a ValueError past every
  `suppress(RigError)`; an AFE read behind a GPIOB write said on=1, the
  rail's users having raised AFE_ON again. Fix: the fleet's checks on the
  stand-in, the pin's on the write's own readback, the closed client a
  ConnectError: 59 in 28 s.
- `test_sensorless` 20 of 20, 136 checks each, and `test_daq_api` 20 of 20,
  82 each, on the relay beside a quad grid: the servo's 1 gate in
  4 and the first dt's failure not seen.
- A pull rebuilds: the terminal, opened in a console, asks each
  tree's ninja what it would build (0.04 s) and builds Debug and Release where
  either is behind, Release last, so `open()` loads the newest
  (tools/target/current.py). The host's C libraries are built in each process
  already. A source touched: both built in 3.9 s.
- MuJoCo Warp on her world as built (mujoco_warp 3.15, an RTX 4080 SUPER;
  her model from its default pose, no control, 86 contacts): the CPU steps
  it in 0.31 ms, 3 200 a second a core; Warp 14 ms a step for one world,
  0.27 ms a world for 64, 0.039 for 512, 0.018 for 2 048, 0.015 for 8 192 -
  66 000 world steps a second, 21 cores' worth. A pass of her walk is 1.3 ms
  of wall, the world's step 0.1-0.3 of it: the physics batched alone gains a
  quarter at most, and pays once her law and her boards step as arrays
  beside it (docs/TODO.md, the latent stack).

- CI flakes diagnosed and resolved across 5 suites looped under heavy load:
  + `test_native_heat`: transient stability mark cleared if state drops out of
    STABLE before movement begins.
  + `test_sensorless`: 0.25 s settling time after rotor gain mismatch test
    dampens mechanical oscillations for accurate sign check; thread-safe lock in
    `simulated/drive/device.py:_now()` prevents concurrent advance race with
    background reader.
  + Emulator protocol reads: quiet timeout raised from 1 ms to 5 ms, preventing
    socket buffer truncation and byte desync under CPU load.
  + `test_wbc_core`: timing threshold on 100 loop ticks relaxed to 5 000 us to
    accommodate host CPU scheduler preemption.
  + `coaxial_mcp`: ring test window broadened to 0.35 s ensuring >= 3 zero
    crossings, and `RigError` caught in server tool calls.

## Stand-in on the board's C

- Profile (cProfile, the QUAD course on four boards): 19.4 s, 52 M
  calls; the stand-in's thermal mirror 12 M of them: the truth's integration
  11.5 s cumulative, the identification's propagation 8.1, `net_flows` 157 k
  calls, 19 M dict reads; MuJoCo 0.19.
- On the C: its observer, identification and envelope
  `thermal_run.c` (the board's own since 1f3ad19) on a truth that is
  `world_heat.c`, a slot of `world_stand.c`'s; the Python a thin edge, its
  three mirror modules gone, its replies in the wire's units. A QUAD course on
  four stand-ins 19.4 -> 5.1 s, 52 -> 10 M calls, laps 14.18 and 14.96 s;
  test_simulated 60 -> 18 s.
- The rotor demo on the board's envelope: the stand-in on the C
  throttles as the Python copy did (the derate within 0.02 at 36 and 70 A),
  but the page's rhythm moved: a dip under the held load slowed the rotor
  under SEND_FROM, its 10 A short of the load; stalled, the switches at 15-18
  % where 20. The dynamometer eased by the derate, as an operator backs a
  brake off; a spin's ramp up 4 s where 3.5 ended at 13 A. Green 4 of 4.
- The drive on the C: `drive.c` on `drive_model.c`, a slot of
  `world_drive.c`'s, named through the cores' harness; the closed forms
  (`_dq`, `_spin`, `_converge`, `_ih`) and the observers' Python copy gone.
  282 ns a period here, 14 ms a board second: QUAD's four rotors 60 ms a
  flight second where the closed form took 40.
- A QUAD rotor through the flight's loop, the law against the closed form:
  450.0 and 450.0 rad/s on 27.5 A, 690.0 on 58.0; 33 rad/s apart at most,
  in a 2 900 rad/s^2 spool.
- What the law asks that the closed form did not: the record commissioned
  (on the placeholders a 63100 trips at its first torque), the estimate
  locked before it, MOE for a mode on the converters, a trip that drops the
  stage, a shaft that turns when it is overpowered.
- The stand-in's heat on the drive's sync: each phase's mean square and the
  link's mean amps counted a period (`world_drive_window`), not one sample a
  slice ([envelope](envelope.md)).
- A rotor node (`coaxial.node.Rotor`) armed sensorless at rest: the machine
  types' routines tripped 2 runs of 3, a rotor backwards past its limit or
  10 A at an arm. Started as a stepper and stopped before its stage goes
  down ([drive](drive.md)): 36 runs of 36, 32 of them four processes side
  by side.
- The loop on the C: device 12 is `ctrl_loop.c`, the part's (`board_ctrl.c`
  its angle and its guard) and a stand-in's, ticked in the slot's own
  sample; the stand-in's thread and its 200 Hz gone. QUAD's rotors on it
  ([quad](quad.md)).
- Acquisition, STO and peripheral models on the C (`world_stand.c`,
  `world_parts.c`, `world_bno085.c`, `world_emu.c`): `SimulatedDaq` runs on
  `daq.c` and `filter.c` (`daq_window`), `SimulatedSto` on `world_sto.c`, and
  `board/native` and `board/emu` share `world_afe_t`, `world_a1335_t` and
  `world_bno085_t`; a model change is one change in every executive.

## Pages

- A `Feed` slept its period after the read: a 20 ms read at 50 ms fed 14
  readings a second to a 20 fps page. Start to start now.
- HUMANOID rendered under `--keys` (a key a frame, `tools/render/page.py`):
  O turns the view, X puts her at the other side, Shift+W runs her at 0.90
  m/s, W+A walks her an arc the chase follows, C has her down with `[F]
  #201 stand` offered, the chair scene offers `[F] sit chair` at 0.8 m. Her
  process ran at 0.2-0.3 of real time on four cores, 8-10 frames a second.
  A frame at 15 a second showed the same state again 65 times in 235, the
  rest 0.036 s of her time apart with 0.022 of spread, her process at 0.7 of
  real time in slices of 0.05 s (`terminal/views/playback.py`).
- HUMANOID's R crashed the page at once with no traceback
  kept; headless it took R and saved 97 rows then, and R starting, three
  states and R saving is test_views_humanoid.py's. The
  suspect stands: a page started before `buses.FIELDS` changed (play and
  flex then; delta, delta_d and ahead) reads a block of
  another layout - a page is restarted after a pull.
- THERMAL OBSERVER said `AFE off` with AFE_ON high: no reading yet, the first
  sample is 30 s after opening. It says which now.
- THERMAL OBSERVER and the side columns, a review's 29 findings against the
  refit:
  + `--switch` armed a real board with the break bypassed; now AFE_ON off,
    the latch cleared, the break in circuit, AFE_ON restored as found.
  + The map's centre patch was stripped, its middle 9.6 K off.
  + A hud counted one row and the column never paged: the
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
  nothing flown.
- QUAD's two plots inked and marked alike: a plot's left curve and its scale
  one ink, its right another, the bus a mark a row as the height (orange and
  red had been the bus's and the temperature's).
- The board's raster on a GTX 1080 Ti (wgpu, Vulkan): 392x224 100 ms -> 2.1
  ms; the menu's turntable 64 -> 13 ms at 52x18. BOARD ATTITUDE 66 -> 64 ms
  a frame at 200x60: its ground (32 ms) and paint (15) are the frame now.
- The front page's model was drawn at inner height - 2 inside a 1-column
  padding: a blank row top and bottom, a column each wall. Now the box's
  full inside (checked at 90x28, 120x36, 200x60).
- The front page's stand shows the quad after the gynoid, 9 s alone in a
  hover and once over its nose: 10 ms a frame on the card, 69 on the CPU,
  where the stand keeps to the board (`terminal/stand.py`); QUAD's row had no
  kana.
- Rotor observer at a 200x60 terminal (can 95 dots, tuned at 21): the ring
  stroke grew to 3.0 dots half-width, pulled teeth floated loose. Stroke
  capped at 1.0 (0.8 broke into dots), the undriven tooth length drawn as
  track, kept out of any cell an area holds.
- One DC bus connector's screw hole drew as a box, then vanished into the
  connector (724f950 absorbs nested blocks): the circle test centred on the
  centroid and 14 unevenly spaced points read dev 0.047 (limit 0.03).
  Least-squares centre: a drum, like the other four; no other primitive moved.
- BOARD ATTITUDE face down drew the top's parts on the underside. The
  outline's grace adds the cell's depth span, and a tilted face spans
  0.036-0.08 a cell against a 0.032 slab: 533 of 535 drawn dots sat behind
  it. An edge on the slab's far face gets the fixed grace only; the face art
  (the top's layout) is not read from behind (rasters face down,
  up, tilted).
- A shaded cell keeps 0.4 of its dots at the least (`lit.braille`, 0.40 + 0.60
  of its light against the blue noise): on 0.04 + 0.96 of its light to the
  1.5 the crafts' mid-grey shells and the gynoid's clothes read as clouds of
  points beside the glTF and FBX models they were judged against. Her jeans
  (70, 98, 150), 0.062-0.068 m a thigh where 0.072-0.088, 0.052 m a hem where
  0.074; her shoulders' rings 0.118 m at 0.344 m where 0.14.
- The helicopter after the T-REX 700 beside the repo: its canopy a 0.57 m
  teardrop over the frames, red over black; a loft's fourth number drops a
  ring, and the lime canopy had sat under the frames, unseen from the side.
  The ornithopter's cabin 0.18 x 0.15 m, its wings their 0.085 m chord where
  0.032 was drawn.
- The tty's face (VS Code 1.141's xterm 6.1 in Edge 154, WebGL,
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
- Ollama answered 500 for nine days: the runner failed to start.
- An answer at 40 tokens/s (3 lines, 2.5 s of motion): first move 0.21 s fed
  a line at a time, 0.66 s sent whole. A `wait` wake costs 15 tokens after
  the first full `now` (~120); the humanoid prompt 1 360 characters. Every
  pass wrote unset outputs as 0, opening the stand-in pack's contactor; unset
  now holds what the node reads.
- A step waits for its targets (1 % of their range) or its tests, its seconds
  a timeout; L and H alarm, logged as they come and go with a 1 % deadband
  against chatter; LL and HH trip; all in `machine.alarms`, beside the
  sequencer. The squat's down ends on arrival in 0.6-0.7 s of its 2; the
  scan's timeouts are answers, not alarms (it branches). The humanoid prompt
  1 434 characters.
- Warmth asked reads the thermal observer, every node: the intent's thermal
  kind. On llama3.1:8b on the CPU (Ollama 0.35.1's CUDA runner
  failed its PTX JIT on the RTX 4060's driver 617.42): three warmth rows
  called `thermal`, the NTC's still `analog_read`, 19-28 s a row.
- The desktop went out of memory with no page file (Windows' event 2004, its
  top three about 1 GB each): with nothing heavy running 15.1 of 23.7 GB was
  committed, and each relay sized its batons on the free commit alone - two
  sessions' relays booked the same headroom. A ledger every relay books its
  jobs in (`focus.LEDGER`), each counted at what it has still to commit,
  measured and booked under one lock: two relays side by side, room for 3
  jobs of 2 GB, held 5 before, room for 2 held 2 after.
- Her numpy compiled to C (`tools.cores.trace`, `emit`, `held`): traced on her
  arrays, each statement a loop nest at its layout, each einsum summed in the
  order numpy's own result shows (the regressor's 'nij,nj->ni' SSE's two
  lanes, its other 222 a plain sum), held bit for bit on the arrays traced and
  on fresh ones. One state's program at one state over any number: the
  regressor 0 of 5.35 M entries apart, its C 5.95 ms at -O3 for 400 states
  (8.0 at -O2, 8.5 at -O1, each level bit for bit), gcc 18 s and 0.79 GB.
  Built in the walking process its trace held her world 0.88 s: built in a
  process of its own, numpy's own meanwhile, kept by what its C is made of.
- Her sensing compiled on her body (`tools.cores.bodied`): `wbc.sense` traced
  on a twin of her body, her data's arrays and her scratch externs, each
  MuJoCo call mujoco.dll's by its prototype in mujoco.h, each matmul numpy's
  BLAS through `wbc_matmul`. 130 -> 32 us a call, 0 of 40 configurations
  apart; her walk 765 -> 713 us a pass, the two in one process pass by pass,
  her trajectory the same to the bit over 6000 passes. Traced and written
  10 ms at her law's first pass, built 0.9 s in a thread, held on the twin
  4.5 ms; a new library's first load 40 ms, loaded in the thread. Her step
  pointed at its block in one write, its 24 pointers from the block's base
  (`law.POINTED`): the step's call 142 -> 98 us, its Python 92 -> 47.
- numpy 2.5's reductions, read off its bits: a sum from +0.0 (`np.sum([-0.0])`
  is 0.0), a run innermost in memory and summed pairwise - eight running sums
  from 8 terms, halves past 128 -, an axis not innermost added in order; min
  and max from the first in order; a mean the sum over its count. Its maximum
  and minimum keep the second operand at a tie of zeros at every length (maxpd,
  minpd), Python's max and min the first, its clip x; `wbc_np.h`'s `npmax`
  and `npmin` Python's, `npmaximum` and `npminimum` numpy's. Compiled, 38
  statements of them bit for bit (`test_compiled`).
- Her law's functions compiled on her memory (`bodied.on_args`): each array
  given an extern in its own memory - `grip.felt`'s `seen` and
  `law.gripped`'s `out` written where Python writes them -, a dict's entry
  rebound written back, a number traced, the rest pinned; an einsum over two
  summed indices numpy's (`kjab,kjb->ka`: j outer, each run over b in SSE's
  two lanes, 500 of 500 draws). `grip.felt` 51 -> 16 us a call; her walk 797
  -> 696 us a pass with her sensing, felt, gripped and the step's pointers,
  the two in one process pass by pass, her trajectory the same to the bit.
