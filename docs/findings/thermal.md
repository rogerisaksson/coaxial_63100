# Findings: thermal

The board's thermal, measured on the bench: the network, the camera's
campaign, the observer and the envelope (`thermal/`,
docs/architecture/thermals.md). The board's other findings:
[FINDINGS](../FINDINGS.md).

- Dry at 50 kHz, converters calibrated, room ~22 C:
  + 300 s at 33.8 V: NTC 33.1 -> 54.5 C, MCU die +14.3 K, A1335 +12.3.
    150 s at 43.8 V: NTC +21.4, the dies +12.9 and +11.0. The network had
    the NTC at +4.2 and +3.6.
  + Fitted to both cooldowns (NTC rms 0.3-0.5 K, dies 0.4-1.4): a leg's dry
    watts from the camera's dump (1.20 W at 24.6 V for three, all in the
    legs) plus 0.40 W of gate drive, 3.4 and 4.4 W; the NTC 0.56 of the V
    patch over the centre, no lag of its own; laminate 21 J/K (the 49 of a
    25 min step is its parts'); in-plane graph unchanged.
  + Checks: the camera's switching state +19.0 K at the NTC for its 18.9;
    the 60 s point 9.55 for 9.75. The law runs 11 % over at 53.8 V, 7 % at
    60.8.
- The MCU die reads its package: 46.0 C awake where the camera has 47; 4.0 K
  lower asleep (0.49 W). With the uncalibrated converter's 40.5 K/W the
  observer held the laminate at 2 C, the legs at 91-104, the room at 7-33 C
  on a board at 33; now the NTC 33.2 for 33.5, the room 21.6. Under the drive,
  sampled as rank 3 on ADC3 with 810.5 cycles (~9 us) sample time in the 20 us
  period, latched into `board_sync_sample_t.mcu_die`.
- Refit on the board, record laminate 21 J/K, dry and blind:
  120 s at 53.8 V observer NTC 57.85 C for the thermistor's 57.90 (rise 25.4
  K for 24.3); 300 s at 23.9 V 51.9 for 50.1 (19.1 for 17.4). Thirty seconds
  on it runs 4 K over, for the anchor to take. Its room: 35-43 C in a room of
  22 on that record, 21.6 on the default 49 J/K. Idle two hours, AFE_ON off
  but for reads: UNCERTAIN, the air's scale at its floor 0.25,
  the capacity's 2.09 (44 J/K; the transient's 49), room 33.4 C, the board's
  own 33.
- The stand-in's load is its balanced `load_cycle`: 569ae47's turning vector
  walked the heat U, V, W, the hottest leg 9 times in 16 s.
- Tour on an emulated board's world (`coaxial.model.rooms`): thermal page on
  native STABLE at 244 s, the room on at 254, UNCERTAIN 259, STABLE again 351.
- Rotor demo on native: 0.85 of the span on the stand-in's model alone; on
  the world (`emu_world_drag`) 0.93-0.94, the clamp at 0.61-0.75, no trip.
- Network against the camera states, stand-in truth: worst miss 9.0 K, 20.1
  K before the emulator (regulators +28 for +8, AFE on read as passive).
  Open: regulators 3-9 K hot, bridge 4-6 K cool.
- Camera, 20 C room, four states x 25 min (tau 6.8 min): board and
  rises (MCU / regulators / bridge / AFE) passive 30.0 / +15.0 / +8.0 / +1.0
  / +1.0. NTC - TSEN: -0.74 C idle, +10.94 C switching.
- Measured: board 8.33 K/W to ambient (one passive point, 1.2 W), 49 J/K from
  a transient. Assumed: per-leg `to_board` 45.6 K/W (one camera zone x3),
  part capacities, NTC fraction 0.30 (pick-place geometry, R a floor).
- IAUCN10S7N021 datasheet: Rth JC 0.69 K/W (6.2 K at 100 A), Rth JA 25.9
  K/W, Rds(on) 1.8 typ / 2.1 max mOhm (the model books typ, -17 % worst
  case).
- SOA to that sheet (Rev 1.2):
  + A leg is judged on its FETs' junction: its node plus each FET's half of
    its watts through R_th,JC, Z_th at 100 ms (the envelope's slice) 0.68,
    against 175 C. Its node against 125 C left 44 K unused.
  + Rds(on) second order on Fig. 8: 5.55e-3/K and 1.78e-5/K^2, within 1.2 %
    from -55 to 175 C; the 0.78 %/K was 5.7 % off.
  + The 2EDL8034 sits 8-13 mm from its FETs on the leg's patch; the patch's
    105 C laminate holds it under its own 125.
  + Current is no ceiling: 175 A DC, 779 A for 100 us, against the board's
    100.
  + QUAD's course on four boards: 0.65-0.80 of their envelopes and the pull
    cut to 21-58 % became 0.48-0.58 and none cut; laps 14.8 and 14.3 s for
    15.3 and 15.2; in a stuffy room 17.9 and 17.0, where the laminate binds
    next. A board's stored record keeps its own ceilings.
- Applications: what a board is mounted in, thermal op 15 into
  the record (CAL_VERSION 16), laid over the still-air network by
  `thermal_app.c`, ballpark until a test cycle runs in each. At 3 W a leg,
  1.5 W its shunts, 20 W the winding, the legs settle at:

| Application | Legs, C |
| --- | --- |
| still air | 184 |
| a hover's wash, 1 470 rpm | 115 |
| sealed finned housing | 74 |
| a fan's sink | 53 |
| cold plate | 37 |
| PAO | 76 |
| transformer oil | 80 |

- A record load or defaults lays the observer's network anew; it ran the old
  one until a boot.
- A world mounts its board: its JSON names its link and its
  application, laid on the plant's heat and told to the board as the rig
  opens it (board/emu/worlds/dyno.json, 63 V on a fan's sink). The rotor
  observer's demo there:
  + still air, 63 V: switches at their ceiling, the clamp throttled to
    nothing in five segments of seven;
  + on the sink, 50 A: 0.24-0.60 of their span, never throttled;
  + 90 A: the observer's crossover rose to 129 rad/s, over DYNO's spool on
    the 10 A under it;
  + 70 A: 0.91 in SPIN, 0.81 in DYNO (the board 0.94), throttled to 0.57;
    the stand-in 0.93 and 0.95, to 0.42.
- The envelope must step and evaluate per 100 ms slice: a 1 s step let the
  driver node reach 178 C before the clamp saw it. Catch-up capped at 2 s.
- A rail another had just raised was read without a borrow's settle, and a
  reading without the reference is mid-scale: a 25.00 C thermistor and a 545
  C die, which the anchor took. Read after the settle now, never without the
  reference.
- Squaring one synced sample per step aliased; `Board_SyncMeanSquare`
  accumulates per leg in counts. Its slope was taken over one code, where the
  record's integer gain trim truncates to nothing: the observer's I^2 never
  saw a channel's gain. Over 32 768 codes now.
- The stand-in's losses were `phase_power`'s (no duty, link, dead time,
  tempco); now `thermal_power_estimate`'s, mirrored in
  `coaxial.model.thermal` and held to the C. Its held vector turned the
  current's size, not the phases: one leg heated.
- With the tempco a leg has no equilibrium past 28 K/W x P x 0.78 %/K = 1: 60
  A rms held ran one away before the winding warmed. The stand-in's setters
  wrote its live network and the identification's next update undid them;
  they write the base, the scales re-applied.
- Steady state at 5.30 mOhm: continuous 19.1 A against a 105 C laminate, 22.0
  A against a 125 C junction. Throttle at 90 % of span.
- Identification (host, against a ground truth): air path box 2.0 -> 1.93,
  fan 0.5 -> 0.50 in three cycles, innovation 0.05 K. Spread and NTC share
  are unobservable from a cooldown and held.
