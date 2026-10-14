# Findings: the drive

The drive and its observers on the stand-in, the emulator and native://; the
rotor's pages and their demos. The board's findings:
[FINDINGS](../FINDINGS.md).

## Rotor demo

- ROTOR OBSERVER's rotor is the flywheel (J 8e-3, b 5e-4), not the bare 2e-5,
  and the demo's loop is designed on it. The cycle: up each way against a
  propeller on the clamp, 0 -> 2 771 rpm at 50 A and 0.85 kW; a coast at iq 0
  on the drag and the propeller; a brake; a load at the continuous rating held
  at 993 of 1 000 rpm on 25 A.
- The demo's STEPPER at 15 mechanical deg a step (105 electrical) lost its
  rotor (180 asked, 580 turned); 45 electrical, eased: 63 of 60.
- The demo's loaded speed changes spool, waiting on the rotor, 800 rpm/s at
  most: 164 rpm/s the first half second where a constant rate stepped 552 on.
  QUAD's 0.4 s stabs made 49 changes a minute and a 48 A step; 16 and 22 now.
  The feed reads the drive every pass: 16.2 states a second of 8.8.
- At no current the demo's flywheel swings +-20 deg mechanical over 10 s on
  native: the stiffness of ~10 mA held by the loop, an offset's.
- The emulated bench's rotor: a `free` load carries no inertia, so the 2e-5 kg
  m^2 motor alone reached no-load in 0.2 s; the demo's 8e-3 flywheel is a
  drag-free `rotor` load.
- On the emulated MCU ROTOR OBSERVER drives its plant through the converters
  (heat, NTC, SOA as on the stand-in): sensorless held a 10 A
  reversal through zero; the clamp through zero ran the estimate to 1e5 rad/s,
  and the observer chain lost lock at the clamp's acceleration (0 at 2 500).
  I/f cannot carry the flywheel: on its current spring zeta is ~0.001, it rings
  and slips. The demo: aligned once, then sensorless, zero crossed at 10 A, the
  clamp above 1.5 w_hi, the brake proportional: 224 s native, 1 700 frames on
  the emulator, no trip.
- The demo on an emulated board ran the record's placeholders (J 2e-5, l1 0.1,
  l2 100): sensorless from standstill lost the rotor in 27 ms on the drive
  core; the hold's hand-over started on a free-running estimate (-38 rad/s)
  and tripped at 70 A. Commissioning's arithmetic (l1 0.025, l2 7.9, 0.16 V at
  fs/2, crossover 180 rad/s) and the held frame as the estimate: 1 600-2 000
  frames, no trip, native and emulator.

## Stand-in fidelity

| Stand-in | Effect | Now |
| --- | --- | --- |
| the hold turned its vector at theta_sp + omega_target t | a new rate jumped it 1.001 rad | drive.c's command_frame, ramped and integrated: 0.0001 |
| the angle page's stepped command | kicked the held rotor, +-0.9 deg at 8 Hz, 832 turnings in 65 s | a turning vector: 0 |
| torque faded linearly with speed, half of kt I at half the no-load speed, the rest unaccounted | 1 771 rpm, 0.61 kW at 50 A | the current held by the link, abs(v) <= vdc / sqrt 3, as the world core's: 2 771 rpm, 0.85 kW |
| the rotor stood with the stage off (2 451 rpm for 18 s) | - | it coasts |
| the thermal air took the rotor's rpm by the bench motor's 7 pole pairs: QUAD's 63100, 14, cooled its boards on 2 941 rpm at 1 471 | - | by the record's, as `board_thermal.c` |
| bare rotor on a 2 A hold (k 0.735 N.m/rad, J 2e-5, b 1e-5), zeta 0.0013 | rings at 30 Hz for seconds; a 25 Hz loop pumps it until poles slip (224 deg) | a joint's damping b 4e-3 (zeta ~0.5): 20 joints within 4.1 deg over three runs; a 25 Hz loop cannot damp 30 Hz: the board's loop's job |
| CubeMX's DTG 19 written; its die channel at mid-scale (545 C); its MCU die read under the drive; the triple's codes and the trigger unarmed; the STO pilot with the AFE off; native's 5 V rail at 0 V | - | each as the board reads it |

- QUAD's four stand-in observers: STABLE from 49-55 s of a
  flight, 66-70 with the routine; air 1.00, capacity 1.00-1.01, the room 25.1
  of 25.0 C. The page said `4 of 4 converging` whatever they were.
- AFE_ON is refused under an armed stage, taken before it.
- The firmware's hold turns its vector at `accel` toward `omega_target` and
  stands still at none; the stand-in's turns at once.
- The stand-in's plant held at 30 A pulls out and runs away at every rate from
  0.02 to 0.25 rev/s; at 5 A it follows the commanded angle.

## Observer at low speed

- With the stage off the firmware's observer lost the rotor on the emulator:
  3 308 -> 238 rpm while the flywheel turned on, stuck through the brake. The
  demo coasts at iq 0 with the bridge on. The stand-in's tracker follows the
  model off: a gap.
- Near standstill the firmware's estimate swings 100-150 rpm a sample, on
  native and Renode alike: the estimator, not the emulator.
- drive.c: in a command frame below the back-EMF's speed the estimate is the
  frame; into hold from sensorless above it, the frame starts on the estimate
  (a jump to the setpoint's angle at speed slipped poles).
- drive.c read the theta setpoint only as HOLD began: commissioning's three
  angles, the machine's ring test and the stepper and servo, all writing it
  mid-hold, held one angle on the firmware while the stand-in's frame
  followed. The frame moves by the setpoint's change now, the ramp's travel
  kept. The motion verbs waited on the wall's clock: at a 20th of real time
  the emulated flywheel was read mid-swing; on the board's clock now, the
  servo lands 30, 60, 0 deg within 0.2 on the emulator.
- At rest the injection's estimate ran away three ways, each measured on
  native against the world's shaft:

| Fault | Fix |
| --- | --- |
| the loop fed w lambda forward off it: at 2 V and l2 261 the demodulator read the current it drove as angle, -8 000 rad/s and 16 A inside 20 ms | the loop's speed is the back-EMF weight's share of it: held +-1.8 deg for 2 s |
| the weight came from the estimate: noise past w_hi handed over to a back-EMF of nothing, which read back the feed-forward | from the fundamental's abs(E) / lambda (2 ms): the core's case 2 243 -> 130 rad/s |
| the demodulator differenced currents each turned into its own frame: 12 A of the align's d current fed 4.4 of each correction back, a pole off, the demo's first up backwards to -527 rpm | stationary differences along the injection's axis: +-0.04 rad through it |

- Commissioning backed the injection to 20 dB after the filter: native's 0.057
  V left 2.5 rad an update, noise wrapped flat. It stops where an update keeps
  pi/6 now: 0.585 V on the record's noise, 30 rpm sd at rest; the demo's
  innovation near zero 1.08 -> 0.25 rad, its first up forward.
- A/B against the stand-in over the demo (tools/dev/ab.py, then the drive's
  state every 50 board ms through a cycle on all three): the can's
  wobble was the low-speed estimate before the three fixes above and the
  pi/6 linearity floor (`88e4d69`).
  + The front end's AC response (`electronic_simulations/afe/amplifiers.asc`,
    `V(v_diff) / I(RSHUNT)` over `.ac dec 100 10 10Meg`): `10.3124 mV/A` and
    `0.00 deg` at DC, `10.3117 mV/A` (`-0.001 dB`, `0.99993x`) and `-0.34 deg`
    (`76 ns`) at `12.5 kHz`, `10.3091 mV/A` (`-0.003 dB`, `0.99968x`) and
    `-0.68 deg` (`76 ns`) at `25 kHz`, `-3 dB` at `4.5 MHz`. The shunt's
    `2.5 nH` (`LSHUNT1 + LSHUNT2` on `3.5 mOhm`, zero at `222.8 kHz`) is
    pole-zero cancelled by the input differential filter (`R10 + R11 = 100 Ohm`
    with `C9 = 6.8 nF`, pole at `214.8 kHz`), so the static transfer in
    `coaxial_63100_afe.repl` is within `0.03 %` and `0.68 deg` at both
    injection frequencies.
  + In `board_pwm.c` (`enter_direct`), `RCR = 1` is loaded at an underflow so
    the compares transfer at each overflow alone, centring each pulse on the
    underflow and holding one compare triple from overflow to overflow.
    Sampling at the top (`CCR5 = ARR - 15`) therefore samples at the period
    boundary - the peaks of an `fs/2` (`drv_inj_periods = 1`) triangle wave,
    not the midpoints - and gives `0.995` of `demod_gain` at `n = 1` (`0.1170
    A/rad` at `v_inj = 0.585 V`). Both `board/native/native.c` (`EDGE_OVERFLOW`
    followed `15 ticks` later by `EDGE_TRIGGER`) and
    `board/emu/Coaxial63100_Plant.cs` (two `20 us` half-steps a period,
    `world_plant_step` sampling before advancing) integrate the full `40 us`
    period between top samples.
  + With the rotor locked on the d-axis (`theta = 0`, `j = 1e6`) at
    `v_inj = 0.585 V` (`n = 1`), `eps_amps` at `inj_phase = 0.0` is
    `+0.00000 A` on the drive core and `+0.00009 A` on `native://?world=bench`,
    and at `inj_phase = +0.3 rad` is `-0.03338 A` on the drive core, `-0.03264 A`
    on `native://`, and `-0.03394 A` on Renode against the model's
    `-0.5 * 0.1170 * sin(0.6) = -0.03302 A`. On a free flywheel (`j = 8e-3`)
    held at `id_ref = 12 A, iq_ref = 0 A` without an AFE zero tare, the untared
    zero offset (`PhaseZeroSigmaVolts = 1.807 mV`, `~0.17 A` DC) drives a small
    real q current that settles the rotor at `+0.038 rad` (`+2.2 deg`), where
    `eps_amps` faithfully reports `-0.0044 A` (`-0.038 * 0.1170`).
  + Through the demo's 5 s `+-200 rpm` sinusoidal rock (`146.6 rad/s < w_lo =
    240 rad/s`, crossing zero speed in `DRIVE_SENSORLESS`) with the emulation
    paused at each sample (`unpaused`, two Modbus round trips take `~12 ms` and
    add `1.76 rad` of shaft travel at `200 rpm`): on Renode, `theta_hat` out of
    RAM (`s.drive.theta_hat`) against `sysbus.gpioPortE.plant Shaft` stays
    within `0.138 rad` at worst (`0.039 rad` mean, `|eps| <= 0.293 rad`); on
    `native://?world=bench` within `0.117 rad` (`|eps| <= 0.251 rad`); on the
    drive core (`test_handover_under_d_current`) within `0.100 rad`.

## The law under the stand-in

- Commissioning's steps on the law and its model, 51 checks. The sign check
  read 0.14 +- 0.08 A on a rotor the gain step left turning at -129 rad/s,
  0.1912 +- 0.0000 after 0.15 s on the vector's own damping (`sign_check`'s
  settle). The L map at no bias read L / 2.3: the law's dead-time table and
  the model's error both followed the sampled current's sign, which an fs/2
  ripple turns each period. Evaluated on the fundamental phase currents
  (drive.c) and the two-period average (drive_model.c, pmsm.py), Ld and Lq
  recover within 2.5 % of truth at no bias (19.5 and 29.3 uH of 20 and 30),
  saliency 0.20, and smoothly bend under d-axis bias current.
- The chain beside the loop (`obs`, a PLL of 200 and 8 000) does not acquire
  a rotor spooled at 81 000 rad/s^2: 0-4 rad/s at a hover's 2 957.
  `board_thermal.c` took the air's rotor speed off it: off the loop's
  estimate now.
- A held rotor overpowered (1.2 N m on 3 A) turns at -11 460 deg/s on the
  bridge's braking, 20-28 A; the servo slewed after it for minutes. A shaft
  swinging a pole pitch while it is measured ends the move.
- The estimate's lag under 36 750 rad/s^2 on the placeholders: 0.001-0.008
  rad at l2 100, 0.13-0.14 at 4 (alpha ts / l2: 0.007, 0.18). At 0.2 A from
  rest with no injection it is lost under w_lo.
- From rest on the injection, a 63100 and its propeller asked 30 A: the
  estimate ran 2.8 rad ahead in 20 ms and the rotor spun back to -44 rad/s
  before it locked. Its gains laid for that spool (l1 0.18, l2 444) ran the
  estimate to 1 026 rad/s in 5 ms and tripped. Started as a stepper, the
  vector turned to 86 rad/s on 15 A in 0.4 s: 0.003 rad off at the
  hand-over.
- A bare rotor (b 1e-5) armed sensorless at rest: the estimate half a turn
  off at 7 rest angles of 9, as it and the rotor happened to lie. Aligned on
  a standing vector first, a quarter turn off and then on it, the rotor swung
  0.9-1.06 rad about it and 2 starts of 12 were half a turn off still. Its
  vector turned up as a stepper's, sensorless past the crossover: at 1.3 and
  2 times w_hi the swing had the rotor under the crossover at the hand-over,
  4 and 3 starts of 12 half a turn off there; at 3 times, none of 12. Braked
  to rest on its estimate, to 3 000 rpm and back: within 0.04 rad at rest,
  never backwards (`motion.started`).
- Armed sensorless onto a rotor left turning, the first pass read 10 A: a
  rotor node turns it down as a stepper before its stage goes down.
- The voltage limit scaled d and q alike, and at it the field's current left
  its reference: a 63100 under its propeller, asked 60 A, carried +7.3 A of
  d current at 55 V and +13.9 at 48 - 650.5 and 582.6 rad/s where those volts
  carry its load to 673.5 and 626.3 with none. The d axis whole, q what the
  circle leaves: 673.5 and 626.3 rad/s, the d current 0.0 (`current_loop`,
  test_drive_core's limit). A step of 90 A from the idle trips at 100.
- Device 12's theta command stood still on the part: `board_ctrl.c` wrote
  the setpoint, which a held frame reads at its mode alone. Through
  `drive_set_theta_setpoint` now, the part's and a stand-in's one tick
  (`ctrl_loop.c`).

## Pages

- The rotor page's mark (the angle over the pole pairs) skipped a pitch (51.4
  deg) each electrical turn. The drive counts theta_hat's turns (op 0, MINOR
  24): STEPPER +57.8/-53.7 of 60, 3.2 a frame at most.
- The rotor page: the magnets blurred through a 1/80 s shutter (208-386 cells,
  0.10 a frame at most; the smear's ring flipped 0.29), the windings whole at
  their current's brightness and gone on a coast (0 cells), the mark one place
  among them.
- SHAFT ANGLE's bob was not aliasing (8 deg a frame at most, against a 180 deg
  fold) but the rotor hunting about a free-running vector: 3 deg at 1.2 Hz, 46
  reversals in 20 s. The vector's angle is commanded now, off a raised cosine:
  one turn each way, tracked to 0.6 deg.
- The thermal page's load on a board was the demo vector at the watcher's 0.14
  Hz: on native the current sat in a leg for seconds, the hottest leg changed
  9 times in 20 s, 91 % apart. At 50 Hz of the board's time the legs rise
  together, U and W within 1 K, V 4-5 K under, as on the stand-in.
- ROTOR OBSERVER polled the thermals every 0.25 s on an emulated board and
  every 2 s on a real one: the slowest link took the most traffic. The three
  reads cost 18 ms of a 50 ms tick, over the 23 ms the other four already
  take, and the page froze and raced. Paced by the link.
- The page's remaining jitter is the draw, not the link: the same 130 ms p99
  on the stand-in. `cross_section.put` runs 3 800 times a frame and re-ranks
  the cell's owner at each dot; settling it once a frame is held up by
  `gauges` writing `frame.owner` from outside the class.
- The observer box's error beside a newer estimate: the page's sample replaced
  the state, then the model, a request apart; one update now.

## Meter, ring test, pole pairs

- The ring test stamped each angle read before its ask: a stall under the
  gate's load swapped neck and head (1 Hz apart). Stamped mid-read.
- An armed sync stays armed past drive.off: the meter is the injected group's
  until gate drivers op 3 gives it back (the emulated wire sweep).
- An armed sync leaves the meter the NTC and the DC link (`read_index`): a
  software-clocked sweep waited on phase U indefinitely, and METER BRIDGE's
  demo motor froze all ten meters on the emulator. The page runs no motor.
- `pole_pairs` counted shaft travel over a wall-clock walk against the nominal
  travel: 20.80 for 21 (the rotor pulling in from its rest angle), past the
  0.25 limit on a busy CI runner (red 834519a). Now the command's and the
  shaft's travels, sampled together, after the first quarter: 21.00 with six
  cores busy.
- Fitment by ring test, stand-in: a 2 A hold stepped 20 deg electrical onto a
  phase's axis rings at 21.2 Hz (hip, J 3.2e-5) to 30.9 Hz (head, 1.2e-5), a
  limb's neighbours 6.7 % apart or more, a board's reads within 0.5 %; J
  reads 30-63 % over the model's. The crossing count failed mid-suite (a
  read gap hid a pair: waist and neck swapped): a median.
  + Between two phases, 30 deg off an axis, a leg's current crosses zero:
    the same shaft rang 7 % higher (31.5 Hz where 29.4) and kept 0.70 of
    its swing a period where 0.93. Held where the estimate stood, a
    shoulder read over its elbow one arm in six.
  + By its half periods' median a decaying ring read 3 % either way: cut
    at its window's mean, off its centre, its halves alternate (19.8 and
    13.2 ms where 16.9). Whole periods' median now.
  + On the wall's clock, six processes on two logical cores: reads 131 ms
    apart, one board's two 100-134 % apart, none refused; CI's runner read
    no crossing in 0.25 s. The stand-in's law is told its seconds for the
    ring (`paced`): starved, the same 0.5 %, and test_controller 57 s
    where 73. 210 runs of it, 32 at a time: the ring test failed none.
- Hold stiffness in `node.py`: closed-loop current compliance accounts for
  loop softness at the ring frequency (`kp - ki * tau` and lag) and finite-amplitude
  swing correction (20 deg electrical). Across all 20 humanoid joints, J reads
  within 4.1 % of the model's (hip 2.7 %, knee 1.2 %).
- `Velocity`'s start (`motion.py`): sensorless startup uses `started()` to spin
  the rotor past crossover with controlled ramp and handover. The prefilter
  and speed regulator's previous speed are initialized from current velocity to
  prevent step derivative spikes into the feedforward term. Across 400 starts
  from any rest angle, zero faults.
- Observer chain at extreme spool (81 000 rad/s²): the dual observer's PLL integrator
  limits maximum tracking acceleration to $K_i = 8000$ rad/s² before slipping poles.
  Once handed over (`o->valid == true`), the observer speed and blend transition to
  `flux_omega` when it exceeds the lagging PLL speed. Blend reaches 1.0 and RMS angle
  error remains < 30 deg electrical through the spool up to 3760 rad/s, while cold
  starts (`o->valid == false`) remain strictly in dual mode requiring handover.
- Device 12, the board's loop (MINOR 20): a joint's feedback
  loaded as slots, rows of 0.8 s at 20 and -20 deg, 100 Hz on the stand-in:
  within 0.05 deg, the last held. Four legs with `node_hz=100`: the squat's
  down ends on its level at 0.65 s, the board's 90 deg/s slew over 58. On the
  board it ticks in the drive's sample; not yet run on the bench.

## Thermal anchor

The thermal observer's NTC anchor inverts the whole miss through the
element's lag (215 s) at every sample, a miss standing from sample to sample
included: at the design's 30 s a standing miss moves the leg patches 11.9
times itself, at 1 s 26.7 times. On native a sample a thermal second left the
legs 2-10 K under the world's, HEAD's firmware too. Inverting only the miss
grown since the last sample moved the identification's air scale off its
tuning (0.90 against 0.5 after four cycles). A sample was also folded in on
every slice of a poll: once now.
