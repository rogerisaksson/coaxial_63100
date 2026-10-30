# Findings: host

Tooling, suites and the local model's runner; the pages': [pages](pages.md).
The board's findings: [FINDINGS](../FINDINGS.md).

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
  machine's run now tells its stand-ins its seconds (`machine.rig.nodes.Told`): 0
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
- The period at 448 ns, ucrt's floorf, fminf and fmaxf 38 % of it (a
  sampler on a flying rotor's snapshot): gcc keeps a NaN-honouring fminf a
  call. SSE4.1's roundss and ucrt's pair inline (`world_libm.h`, equal on
  all 144 pairs of zeros, NaNs, infinities): 277 ns, the state the same bits.
  The crafts a flown second, frames' hashes the same: quad 0.19 -> 0.155 s,
  helicopter 0.115 -> 0.10, aerobat 0.13 -> 0.11, ornithopter 0.115 -> 0.10.
  native:// built so too (`build.FAST`): her twenty boards 2.03 -> 1.81 CPU s
  a wall s.
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
  against chatter; LL and HH trip; all in `machine.rig.alarms`, beside the
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
