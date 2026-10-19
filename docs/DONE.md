# Done

What left [TODO](TODO.md), the oldest first; a line's numbers are in the file
it names. A closed item is cut from the list and gets its line here, last;
the hundred before these: [DONE_1-50](DONE_1-50.md),
[DONE_51-100](DONE_51-100.md).

1. Her horizon an externality each: the floor a leg reaches, the room her
   law allows it, an arm's band; Z cycles them.
2. The third-person controls as the games have them, a PNG each under
   `--keys`: orbit, follow, shoulder swap, sprint, crouch, the prompt.
3. The gym under her sensor boundary's check: `sprung`, `worker` and `tour`
   write none of the world's state, a lay, a scene and a shove its own calls.
4. Her situation read from her sensors: six readings, `situations.lisp`'s
   bounds and caps over her law in the surfaces' place; the ice, a ramp, a
   chair and a wall entered unannounced told within a second at every lay.
5. The wing stalls; the aerobat's circuit a rectangle at 9 m aimed at its
   runway, its figures at 14 m ([aircraft](findings/aircraft.md)).
6. The four drawn as the real things: an Edge 540, a tactical quad, a Goblin
   Raw, Dune's ornithopter ([aircraft](findings/aircraft.md)).
7. The four crafts undressed on S: shell, airframe, pack, bare; their motors
   sized to the sim's shaft power, the 63100 board behind each stator or on
   the frame's face.
8. A drive identified on its board through the wire: held and swung by the
   board's own loop, its DAQ's currents and A1335 the rows; the leg's four
   on native:// within 11 % of the world's J and 5 % of its gravity.
9. The lift at any pace and the burn a free fall, braked on all the rotors
   have from the lead their spool and the pass take; the quad's full tilt at the
   link's ceiling, its hold trending to it.
10. The STO chain's hold kept through a frame heard idle: a transaction on 1,
    2 and 4 native boards 18, 41 and 80 -> 9, 13 and 32 ms.
11. Her tee a tube hung plumb from the chest and bust over the hips, the seat
    tapering into the thighs, the hair a cap on the skull with a hairline, the
    fall and the locks close to it.
12. Her rotors read on the steps inside their play, within 1.2 %; the boxed
    world's wind-up and damping off the link's stale speed.
13. Hung in a harness she reads as on the stand: 0 touches, residual 0.24 N
    m, 110 base parameters trusted.
14. An ask her drives cannot carry shrunk as one toward asking nothing, in
    the step and its C, in the clip's place: a hop down's hand 18.7 -> 11.1 of
    her walk's, turns 6.5 -> 3.8 ([jerk](findings/jerk.md)).
15. A load cell in her harness: her pelvis's six rows, ten more base
    parameters, her mass to 0.1 %.
16. The five drawn to their pictures: the orni a dusty pod with a canopy and
    slender wings, the quad a slate hump on round arms over two skid frames,
    the Edge navy with a yellow cowl, the Goblin a lime canopy and blade
    tips, her a fitted red tee, light jeans and a ponytail.
17. Her clothes bent at the joints they span (`graphics.drape`): the tee into
    the pelvis, the seat into each thigh, the shins and sleeves into theirs,
    two frames a corner by its height.
18. Her errands settle without hesitation or flicker (`machine.law`,
    `terminal/views/gym.py`, `coaxial/graphics/arrows.py`, `test_errands.py`):
    `held_back` lets a held row go after two of her rhythm's swings, F sits
    and rises her; a walk asked of a swaying stand no longer waits 1.35 s; a
    scene laid anew clears what she felt and is told the keys' twist again
    ([scenarios](findings/scenarios.md)).
19. Findings that told of the laws before the loop cut to what holds on the
    loop (body, feet, drives, stacks, normal, wbc); `test_layout` holds that no
    document names a module, a tool or a suite not in the tree.
20. The budget's throttling flag (`thermal/src/thermal.c`,
    `thermal/src/thermal_run.c`, `host/tests/test_thermal_core.py`):
    `throttling` means `derate < 1.0f` across both `thermal_budget` and
    `thermal_run_slice`, matching lookahead-triggered derating from cold and
    staying false when `derate == 1.0f`.
21. Bootloader torn flash word protection (`boot/src/boot_main.c`,
    `boot/src/boot_core.c`, `host/tests/test_boot_core.py`): 256-bit flash
    words probed under `FAULTMASK` and `SCB_CCR_BFHFNMIGN_Msk`, clearing
    `FLASH_SR_DBECCERR` and `SCB_CFSR_BUSFAULTSR_Msk` on a torn word while
    store CRCs and record dumps read word by word.
22. Drive thermal reciprocal precomputation, TIM1 dynamic ARR/CCR4 caching,
    and striding wrench parity (`drive/src/drive_thermal.c`,
    `board/src/board_drive.c`, `wbc/src/wbc_wrench.c`,
    `host/machine/wrench.py`): eight redundant `vdiv.f32` instructions
    replaced by reciprocals in `drive_thermal_step`, redundant APB2 MMIO
    writes skipped when `arr_dyn` is unchanged while preserving calibrated
    `drv_trigger_ticks` lead, and `wbc_wrench.c` / `machine.wrench` matching
    within `1.5e-10` across all 1 259 striding solves.
23. The aerobat's knife-edge pass out and back after its prop-hang, its full
    tilt apex under 10 deg (3.2 deg), its final flown wing-borne (`w = 1.0`) to
    a progressive flare touching at -0.85 m/s, and both laps of the 16-gate
    course flown nose first, banked and pulled on `COURSE_SPHERE`
    ([aircraft](findings/aircraft.md)).
24. Each hold on her halting rows counts from its own start
    (`machine.law.held_back`): a hop said after a stop no longer loads on one
    sole mid-step; `hop_after_a_stop.csv` stands at 8 of 9 wobbles where 6
    ([scenarios](findings/scenarios.md)).
25. Identification on-line, under a load (`machine.wear`, `machine.worker`,
    `terminal/views/show_humanoid.py`, `tests/test_wear.py`): her jeans' hems
    integrated as 2-DoF pendulums with their stops at the world's step, a
    slipped encoder zero read on both rigid and boxed drives, a load on her
    back read off her insoles (`wear.torso_load`), and her hands' and torso's
    loads laid into her law (`wear.loaded`) and shown on the page
    ([identification](findings/identification.md)).
26. Her map's boxes from their cells alone (`machine/sight.py`, `test_sight.py`):
    `FAR_M` 3.5 m maps the whole bed with no case by kind (0.05 m off at
    (-1.8, 1.8), 0.00 m off moved to (-0.9, 1.2), half sizes within 0.05 m,
    697 of 725 occupied cells within 0.15 m of a wall or a thing) and the
    unwired scaffolding (`rtu.FEED_REG`, `drives.wave_stage`,
    `figure.lumbar_curve`, `SHELL_ARMOR`, `skeleton.printed_holders`,
    `flange_mounts`, `campaign.de_ring_ankles`) is out
    ([sight](findings/sight.md)).
27. Stereo disparity on the card and columnar telemetry tables
    (`machine/stereo.py`, `machine/sight.py`, `tools/sim/tabular.py`,
    `tools/bench/analyze_phase_log.py`, `tools/sim/gvs.py`, `tools/sim/replay.py`,
    `tools/sim/look.py`, `coaxial/acquire/task.py`, `coaxial/graphics/lit.py`):
    3-stage WGSL compute / CuPy stereo block matching cuts 320x240 x 48
    disparity from 95.6 ms to 1.60 ms (60x faster, 99.85 % kept agreement, max
    0.0173 px), RAPIDS (`cudf` / `cupy`) and `pandas` / `numpy` columnar CSV
    ingestion and trend fitting replace row-by-row `csv.DictReader` loops, and
    unrolled 4x2 cell reductions in `lit.braille` shave 1.8 ms/frame bit-for-bit
    ([sight](findings/sight.md)).
28. Her doing read at her law's passes (`se2.doing`): a stand held while she
    swings no longer comes and goes at the world's passes between, 596 flips in
    a stop to 2; `tools/sim/told.py` times what a player's keys and F ask
    ([scenarios](findings/scenarios.md)).
29. Her grip bounded by what her soles read (`machine.grip`): a slide lowers it
    and never raises it, a slide read is taken whole; the slick patch stands at
    every wobble ([grip](findings/grip.md)).
30. Her fall judged by what she touches (`worker._is_fallen`, `floor.grounds`):
    a part above her shins on the floor, what lies on it or a step; a chair, a
    bed and a wall bear her. On her back on a bed she lies where a tilt past
    45 deg fell her ([scenarios](findings/scenarios.md)).
31. ARCHITECTURE.md an index; a subject a file in docs/architecture/: firmware,
    electronics, drivetrain, thermals, kinematics, solvers, planners, robotics,
    aerodynamics, quad, host, views, tests. Every machine/ module named in one
    of them; a drive's heat at its four nodes, not three.
32. Her rhythm at rest holds no row (`law.held_back`): sitting and rising, her
    centre of mass behind her heels before the seat bore her, a seat asked was
    held for ever - CI's 3.12 seated her in none of 5 s
    ([orthokinematics](findings/orthokinematics.md)).
33. Her slide kept through a pass no sole bears (`machine.estimate`): a swing's
    scuff no longer reads as a slide of the sole that holds, on the snow 393
    passes of 11 219 to 21 ([sensing](findings/sensing.md)).
34. The bed soft, her lie-down quicker (`floor`, `stunts_base.lisp`, `gynoid`):
    F at the bed lies her at 2.2 s where 4.2, still at 3.4 where 6.3, 7.5 cm into
    a foam's mattress where 5.5; the page draws its top a grid sagging where she
    presses ([scenarios](findings/scenarios.md)).
35. The aerobat's low passes flown by a sampling MPC on SE(3) (`machine.se3mpc`,
    `mppi/`, one C rollout for gcc and NVRTC with a numpy twin): sideways on
    the hover's law, the tail's lowest point 0.01 m median over the runway;
    a fuselage side plate in the plant and the wind observer; the takeoff
    rotates to its climb at the wing's speed.
36. The aerobat opens with a knife-edge back down the runway on its wing law
    (`wingborne.plane`'s `knife`): takeoff, a teardrop onto the runway's
    line, wings vertical, the nose 38 deg over the path at 11.9 m/s nose
    first, the tail's lowest point 0.029 m median; nose first it flies a
    speed, not a place; the landing stops on the runway
    ([aircraft](findings/aircraft.md)).
37. The aerobat on an airfield of its own (`machine.airfield`): its runway, six
    6 m arches on its straights and a pylon in each 22 m turn - a run-up off the
    runway into the straightest leg, a slalom of three - flown on its wing by
    the line of the way ahead, two laps through every gate in gusty wind and
    down on the runway; the routine among its standing things unstruck, home
    from the knife-edge on its wing, the prop-hang held at home in 7.4 s where
    it walked 53 m in 18.5 s ([aircraft](findings/aircraft.md)).
38. The aerobat flies on its wing (`wingborne`, `machine.aerobat`): its lift
    across the air, its airframe's drag, its moments trimmed, the wind read
    through its wing, the air's speed held; its circuit and teardrops at 14
    m/s, 7-8 deg, where 50 deg/s at 10.5 m/s flew 25 deg on the propeller
    ([aircraft](findings/aircraft.md)).
39. The airfield's course laid for a plane (`machine.airfield`): an air gate, a
    limbo bar, a hoop at the far turn's top, a slalom of three pylons, a
    knife gate taken on its side, on one line of arcs it flies by L1 and
    draws; two laps through all in gusty wind, every gate within 0.9 m.
40. The crafts' modules of a subject a file: `machine.aerobat`,
    `machine.helicopter` and `machine.ornithopter` packages - airframe, sky,
    law, observer, the SO(3) law's shared kit -, the quad's air and pack
    their own (`machine.weather`, `machine.flightpack`).
41. Her world past real time walking: her pass's glue and her ask in C bit for
    bit on numpy's own orders, 1.27 of real time ([body](findings/body.md)).
42. The crafts and the gynoid drawn solid (`lit.braille`): a shaded cell keeps
    0.4 of its dots at the least; her jeans darker and narrower, her
    shoulders sloped, judged on PNGs beside the glTF and FBX models
    ([host](findings/host.md)).
43. The aerobat lighter and pushed (`machine.aerobat`): a 1.6 m wing at 39
    g/dm^2, a 20 in propeller on its boards' loads at 9.7 g, its damping
    trimmed; off the runway a teardrop at 60 deg/s and a normal knife-edge
    down it at 19 m/s, the wings within 16 deg of upright, the lower tip 2 cm
    over the runway ([aircraft](findings/aircraft.md)).
44. Her law in C her python's bit for bit: her ask, latent step, coupling and
    whole-body step on numpy's own OpenBLAS (`tools.cores.law`), the generated
    law core cut; 1.55 of real time ([body](findings/body.md)).
45. The helicopter and the ornithopter drawn after their models: the T-REX
    700's red teardrop canopy over the frames (its loft had dropped below
    them), a swept fin; the ornithopter's cabin fuller, its wings their
    0.085 m chord in sand ([host](findings/host.md)).
46. The quad on its instruments (`machine.senses`): the board's IMU, an RTK
    receiver on the RS485 bus, a star camera where the signal is lost; a
    hover 5-6 cm rms off its spot in every air, 0.53 m without the signal
    ([quad](findings/quad.md)).
47. Seated on a chair 0.4 um off she was thrown off it: a sole let go, the other
    standing, rose where her folded legs put it; each sole that does not stand
    held at the floor while something bears her, her python's and her C's -
    16 of 16 offsets seated and risen, her walk the same to the bit
    ([support](findings/support.md)).
48. Her wear's window a part a pass: her worst pass walking 118-146 -> 18.2 ms,
    her trajectory and her wear the same to the bit; her regressor compiled
    from its numpy ([identification](findings/identification.md)).
