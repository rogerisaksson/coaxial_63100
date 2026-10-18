# Done

What left [TODO](TODO.md), the oldest first; a line's numbers are in the file
it names. A closed item is cut from the list and gets its line here, last;
the fifty before these: [DONE_1-50](DONE_1-50.md).

1. The frame's core on the image computes as its host twin, to the wire's
   millionth of a newton: built with no multiply fused into an add, where
   its thrusts stood 3.6e-4 N off on CI's host and 1.7e-5 here; 150 442
   cycles a pass where 149 288 ([streams](findings/streams.md)).
2. A stand-in board's pass in flight one C call (`world_drive_pass`):
   three calls a rotor a pass and their dictionary allocations unified;
   the page's flight over 20 frames a second ([aircraft](findings/aircraft.md)).
3. The heat's clock in flight: flown on the flight's own seconds (clock=1),
   observers warmed at idle on the floor out of UNCERTAIN before a first
   flight;
   full tilt boards hold 58.7 A where 43.2, race laps 13.9 and 14.2 s, SOA
   0.55-0.62
   ([envelope](findings/envelope.md), [aircraft](findings/aircraft.md)).
4. The plan on boards that hold their pull: `quad_race.py --search` over
   `course.PULL` (0.37), `course.BRAKE` (0.735), `course.GO` (0.708) and
   `course.STOP` (0.45) on the core; laps 13.8 and 13.8 s across all eight
   board trials where 14.3-14.8 s, cost 27.69 where 28.24, 24 of 24 flights
   whole across seeds 3..8, 72-73 % of top thrust left unused on the mean
   (rows keeping >= 92 % of the clamp's pull on a full pack, 87 % from 60 %),
   WEP 0.6-0.8 s a flight ([envelope](findings/envelope.md),
   [streams](findings/streams.md)).
5. The axis asked brings its own rate: a row's force slewed on its braking
   curve within reach (8.0 rad/s, 200 rad/s^2, slope 10 1/s) with its rate in
   the payload fed forward onto pitch and roll rates; axis miss on a lap 10.2
   degrees rms where 29 (frame 4.5 rad/s rms), 12 of 12 race flights whole
   with 0 strikes ([streams](findings/streams.md)).
6. The loop's slew reaches WEP's with speed: `pull.slew(clamp, w)`
   interpolating
   branchlessly from the flight clamp's slew at idle (2 898 rad/s^2) to WEP's
   (4 347 rad/s^2) at turning speeds past 300 rad/s; passed per-rotor through
   `aero.reach`, rotors near idle retain full stability while spools at speed
   reach maximum acceleration ([envelope](findings/envelope.md)).
7. Thermal situation and horizon fit in hot rooms: boards configure their
   thermal situation and idle warm-up in room ambient before flight; in a
   toasty
   45 C room, boards hold 57.7 A at idle < 60 A, correctly failing `fit` and
   cooling on the floor rather than lifting into a dive strike
   ([envelope](findings/envelope.md)).
8. A `PI` on a current keeps its sum within what its sink carries (`most`),
   host and C (`machine/parts.py`, `ctrl/src/ctrl.c`): told its sink's clamp,
   its current falls at the first pass past its speed (0.2 rad/s under) where
   30 rad/s under untold ([envelope](findings/envelope.md)).
9. The law's vector laid on the angle it is applied at (`drive/src/drive.c`
   after `rotor_observer` and `command_frame` advance the frame), where it
   was laid on the angle it sampled at and met the rotor a period turned:
   its own v . i is 77 W of 77 at 211 rad/s, 766 of 766 at 500, and 1 814 of
   1 814 at 680 where 21 % under ([envelope](findings/envelope.md)).
10. A date refused: the remaining files stripped of dates and
    `test_no_dates_in_the_tree` in `test_layout.py` (`--structure`) holding
    the tree's text free of them.
11. L map at no bias recovered within 2.5 % of truth on the cores (`drive.c`,
    `drive_model.c`, `pmsm.py`, `test_sensorless.py`): dead-time voltage
    compensation evaluated on the fundamental current reconstructed from (id,
    iq)
    at th_out rather than raw sampled currents turned by an fs/2 ripple, and the
    model's dead-time drop on the two-period average current; Ld and Lq at no
    bias
    measure 19.5 and 29.3 uH against 20 and 30 uH (< 2.5 % error, DOD < 5 %),
    saliency 0.20 recovered ([drive](findings/drive.md)).
12. `Velocity`'s start and rotor start past crossover from any rest angle
    (`host/coaxial/control/motion.py`, `host/tests/test_sensorless.py`):
    `started()` spins up the rotor past crossover with controlled ramp and
    handover, prefilter ramp and speed regulator initialised from current speed;
    0 of 400 starts fault ([drive](findings/drive.md)).
13. The ring test's J within 5 % of the stand-in's (`host/coaxial/node.py`,
    `host/tests/test_controller.py`): hold compliance accounts for current loop
    softness at the ring frequency and finite-amplitude swing correction;
    error across all humanoid joints <= 4.1 % (hip 2.7 %, knee 1.2 %)
    ([drive](findings/drive.md)).
14. The chain beside the loop (`obs`) follows a QUAD's spool at 81 000 rad/s²
    (`drive/src/drive_observer.c`, `host/tests/test_drive_observer.py`):
    valid observer seamlessly tracks flux velocity when acceleration outruns
    the PLL; blend reaches 1.0 and RMS error < 30 deg at 81 000 rad/s²
    ([drive](findings/drive.md)).
15. `heat.py`'s dry loss fitted to bench measurements
    (`host/machine/heat.py`, `host/tests/test_thermal_app.py`):
    `dry_loss(volts)` models 2.4 W at 24 V to 4.4 W at 44 V (0.1 W/V);
    nominal 48 V bus yields 4.8 W dry switching loss.
16. `coaxial_63020` pin table in `boot_main.c` (`boot/src/boot_main.c`):
    `PINS_63020` and `BOARD_63020` defined and handled by `board_of`
    for `BOOT_TYPE_COAXIAL_63020`.
17. Debug `-Og` configuration and `board_mem.c` hot-path optimization
    (`cmake/gcc-arm-none-eabi.cmake`, `CMakeLists.txt`,
    `docs/findings/wrench.md`): `-Og -g3` and `board/src/board_mem.c` at `-O2`
    drop Debug flash 267 872 -> 220 464 B (74.76 %) and ITCMRAM 35 928 -> 33 112
    B,
    emitting 64-bit `strd` and 32-bit `ldr.w`/`str.w` loops.
18. Motion papers on the emulator (`world/src/world_emu.c`,
    `board/emu/Coaxial63100_Plant.cs`, `host/tools/emu/emulator.py`,
    `host/tools/cores/native.py`, `host/coaxial/devices/drive.py`):
    `drive.model` sets J, damping, and load on the emulated world plant
    (`emu_world_model`, `emu_plant_reset`); `motion.ipynb` and
    `applications.ipynb` execute cleanly on `EMULATED`.
19. The MCU's die under the drive (`board/src/board_sync.c`,
    `board/src/board_adc.c`, `board/native/native.c`,
    `world/src/world_stand.c`):
    ADC3 rank 3 injected channel samples the die temperature sensor (810.5
    cycles) into `board_sync_sample_t.mcu_die` while the drive is armed.
20. Flown from compiled streams, no rules (`machine/pull.py`,
    `machine/aerokin.py`,
    `terminal/views/quad/flight.py`, `host/tests/test_views_quad_flight.py`):
    flight inside the action space's shell with plan precomputed and 0 clauses
    above the core; 72-73 % pull left unused, 0.6-0.8 s WEP taken, 100 % whole
    across winds, rooms, and pack charges (cost 27.56)
    ([streams](findings/streams.md), [envelope](findings/envelope.md)).
21. Ballistic gait initiation and its Monte Carlo toolchain (`tools/sim/ballistic.py`)
    ([orthokinematics](findings/orthokinematics.md)).
22. Stunts as entries a case each - their measures stood on crutches since taken
    out ([orthokinematics](findings/orthokinematics.md)).
23. A1335 CRC (MINOR 28) on the stand-ins (`board_angle.c`,
    `Coaxial63100_A1335.cs`, `native_a1335.c`, `test_emulator.py`,
    `test_native.py`): replies end in CRC-4 (`x^4 + x + 1`, seed `0xF`) with 0
    CRC errors over >6 500 polled reads ([PROTOCOL](PROTOCOL.md)).
24. CI's flakes resolved across five suites (`test_native_heat.py`,
    `test_sensorless.py`, `protocol_emulator.py`, `test_wbc_core.py`,
    `coaxial/node.py`, `coaxial_mcp/server.py`): stability mark cleared leaving
    STABLE, 0.25 s sign-check settle and `_now()` lock, 5 ms socket quiet wait,
    5 000 us WBC tick ceiling, 0.35 s ring test window and MCP `RigError`
    handling ([host](findings/host.md)).
25. Sensorless below `w_lo`: the AFE's AC response, RCR 1 sampling the peaks, the
    rock tracked on Renode ([drive](findings/drive.md)).
26. One code, several executives: the stand-in's DAQ and STO on the C, the world's
    parts shared by native:// and Renode, hot paths in ITCM ([FINDINGS](FINDINGS.md),
    [host](findings/host.md)).
27. `native://` fallback and shared URL bus `Transport`
    (`host/coaxial/comm/session.py`, `host/coaxial/devices/board.py`,
    `host/tools/cores/native.py`, `host/tests/test_native.py`):
    `open_session` falls back to `native://` when nothing answers on a port,
    and rigs on one URL bus share one refcounted `Transport` until the last
    closes ([emulation](findings/emulation.md)).
28. Guards: a pre-push smoke, the rpm lane on told time, suites that skip by
    name, every suite in a list ([host](findings/host.md)).
29. Recordings that replay pass for pass, a measured stage, REC on every
    MuJoCo page ([host](findings/host.md)).
30. Her law on her sensors alone: `machine.edge` the one reader and writer of the
    world for it, `machine.estimate` her pelvis from her IMU and her insoles,
    `machine.law` the pass; test_structure holds it off the world
    ([sensing](findings/sensing.md)).
31. Mechatronics as data: motors, gearboxes and inverters a TOML library, her build
    a robot's file naming them (`machine.mechatronics`); `drives` and `servos` read
    them, DIMENSIONS.md unchanged; `tools/cores/manifest.py` writes her
    orthokinematic manifest, her text includes it, test_mirrors holds it
    ([drives](findings/drives.md)).
32. native://'s limb load the transport's time scale, a reply waited in the
    board's time; a suite's crash told with its test's header
    ([emulation](findings/emulation.md)).
33. A stop that settles: her capture point past her soles steps her, the stumble
    and the parry gone; a row that halts her rhythm waits while she steps; the
    brake keeps her rhythm; her heading goes with her pelvis only in a turn.
    Both kept falls stand at every wobble, in `tests/replays`; the hole met
    ([scenarios](findings/scenarios.md)).
34. A third-person page: W A S D a way from the view, turned to and walked as
    one twist, Q E a sidestep, Shift a sprint; Space, C and F verbs by what is
    near; a chase camera eased behind her, X the other shoulder; a prompt for
    F. The tour sends twists and verbs; no key says a gait
    ([scenarios](findings/scenarios.md)).
35. Stand-ins on told seconds: a machine's run tells them its own, a rotor's
    start settles its estimate 20 ms and its loop takes over at its reading;
    the rotor demo's and the virtual rotor's tests told theirs. CI's two
    flakes, 6 of 6 starved, 0 of 6 ([host](findings/host.md)).
36. A swing's own clock: its share led ahead of its rhythm as far as lands it
    before her capture point is a reach across past the standing soles, its
    own side; walking 80 N shoves 26 of 32, the 80 N DOD met
    ([scenarios](findings/scenarios.md)).
37. Her maneuver horizon, Z on the page: each sole's landing room as her law
    has it (REACH_M about her centre of mass, APART_M past the other sole),
    bright while it swings; her capture point; each arm's reach across her
    front - from her state's own reading (`coaxial.graphics.horizon`).
38. Her support, whatever bears her: a segment the skin feels pushed up in her
    support region, grounding her, a contact of her whole-body step
    (`machine.borne`, `wrench.pushed`); down onto a chair and up, onto a bed's
    edge and up ([scenarios](findings/scenarios.md)).
39. Her soles' wrench kept in the striding projection (`wrench.W_KEEP`,
    `wbc_wrench.c`): in double support no plan of nothing; her steady walk's
    peak jerk 0.5-0.8 of what it was ([scenarios](findings/scenarios.md)).
40. The stand-in's velocity lane within 1 %: its speed the observer's angle,
    turns counted, risen over a pass ([host](findings/host.md)).
41. The helicopter's head governed, its tail sized ([aircraft](findings/aircraft.md)).
42. The ornithopter's beat through a 6:1 crank, its pairs' lift pulsing.
43. The thrust arrows the force from the hub, inverted too, over the body.
44. The aerobat a plane: its run, its wing and tail, its law wing-borne.
45. The four drawn to read: painted skins, a pod, a 1.3 m plane; closer.
46. The aerobat's propeller pulls one way; a circuit and a flare.
47. Her body identified of her edge alone (`machine.sysid`)
    ([identification](findings/identification.md)).
48. The library's variants as data, a build naming them, `COAXIAL_ROBOT`
    running one: her figure on CubeMars' R80 and R60 ([stacks](findings/stacks.md)).
49. The quad framed a third closer, its arms painted.
50. The rotorcraft's laps on their own sphere, idled up; the aerobat's
    racetrack ([aircraft](findings/aircraft.md)).
51. Her horizon an externality each: the floor a leg reaches, the room her
    law allows it, an arm's band; Z cycles them.
52. The third-person controls as the games have them, a PNG each under
    `--keys`: orbit, follow, shoulder swap, sprint, crouch, the prompt.
53. The gym under her sensor boundary's check: `sprung`, `worker` and `tour`
    write none of the world's state, a lay, a scene and a shove its own calls.
54. Her situation read from her sensors: six readings, `situations.lisp`'s
    bounds and caps over her law in the surfaces' place; the ice, a ramp, a
    chair and a wall entered unannounced told within a second at every lay.
55. The wing stalls; the aerobat's circuit a rectangle at 9 m aimed at its
    runway, its figures at 14 m ([aircraft](findings/aircraft.md)).
56. The four drawn as the real things: an Edge 540, a tactical quad, a Goblin
    Raw, Dune's ornithopter ([aircraft](findings/aircraft.md)).
57. The four crafts undressed on S: shell, airframe, pack, bare; their motors
    sized to the sim's shaft power, the 63100 board behind each stator or on
    the frame's face.
58. A drive identified on its board through the wire: held and swung by the
    board's own loop, its DAQ's currents and A1335 the rows; the leg's four
    on native:// within 11 % of the world's J and 5 % of its gravity.
59. The lift at any pace and the burn a free fall, braked on all the rotors
    have from the lead their spool and the pass take; the quad's full tilt at the
    link's ceiling, its hold trending to it.
60. The STO chain's hold kept through a frame heard idle: a transaction on 1,
    2 and 4 native boards 18, 41 and 80 -> 9, 13 and 32 ms.
61. Her tee a tube hung plumb from the chest and bust over the hips, the seat
    tapering into the thighs, the hair a cap on the skull with a hairline, the
    fall and the locks close to it.
