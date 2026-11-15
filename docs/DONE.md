# Done

What left [TODO](TODO.md), the oldest first; a line's numbers are in the file
it names. A closed item is cut from the list and gets its line here, last;
the two hundred before these: [DONE_1-50](DONE_1-50.md),
[DONE_51-100](DONE_51-100.md), [DONE_101-150](DONE_101-150.md),
[DONE_151-200](DONE_151-200.md).

1. The robot's DC link (`machine.body.dclink`): its packs, their BMS and
    the robot's boards' capacitance under its drives' power, bled on the
    robot's boards' d axes past the text's onset; under its 60 V through every
    scene, a hop down's landing 58.93 V (test_kernel.py,
    [kernel](findings/kernel.md)).
2. The robot's blind field (`machine.senses.field`): the kernel's admittance
    seek down where its rows wait on a seat not bearing it; a
    chair lower than seen found and sat on, a chair pulled before it bears the
    robot a squat (test_kernel.py, [kernel](findings/kernel.md)).
3. The somatic manifolds on the robot's observer (wbc_gmo.c, 8 manifolds): its
    forefoot by its soles' centre of pressure, its thighs' backs, its
    forearms' and carpal heels' shove; read against its skin walking,
    seated and lying ([kernel](findings/kernel.md)).
4. The robot's right-invariant EKF on SE_2+2(3) (`machine.senses.inekf`) beside
    its estimate: IMU and its soles' contacts by the observer's weights; walking
    0.039 m along the floor and 0.004 in height in 7.2 m where its estimate
    0.060 and 0.041 (test_kernel.py, `tools/sim/drifted.py`).
5. The kernel's two master texts as given, a module a file (26): the
    22-capsule body schema, the AST-to-SoA lowering - its functions
    evaluated as written -, the somatic schedule folded into the room's; the
    protocol's list rewritten to its seven steps and five tests.
6. The SoA ABI and the lowering (`machine.texts.lowering`, wbc_soa.h): a gait
    folded into a bank of the text's fields at its offsets over the robot's
    nominal's, a biquad a node seeded on its value, the manifolds'
    weights, the capsules; carried over its link's table, the robot's
    loop's biquad in C bit for bit; swapped settled, a step of 8.6e-7 where
    unseeded 0.30 ([kernel](findings/kernel.md)).
7. The robot's body schema (`machine.law.schema`): the kernel's 18 capsules on
    the robot's skin, its 13 pairs' clearances, the robot's posture held to each
    near pair's ECBF in its mass's metric, each floor raised by its
    braking distance through the robot's drives' back-EMF bounds
    (`machine.body.bemf`); walking every pair clear of its floor
    ([kernel](findings/kernel.md)).
8. The robot's law's torque box the robot's drives' back-EMF bounds
    (`machine.body.bemf`, `machine.stack.shrunk`): each driven joint's bounds by
    its speed on the robot's link's volts inside its clamp, in the robot's
    python step, wbc_step.c and the board's wbc_shrunk.c: the robot's ask shrunk
    to the robot's clamps, each drive clipped to its bounds, the robot's C twins
    as the robot's python ([kernel](findings/kernel.md)).
9. The insert's phase-margin probe (`machine.law.margin`,
    `tools/sim/margin.py`): the text's lock-in at 15, 22, 30 and 38 Hz, each in
    its turn, on the robot's loop broken at a drive's torque; a known loop read
    within 0.11 dB and 1.7 deg; the robot's gearboxes in, the robot's hip's loop
    at 22 Hz past 0 dB with 25 deg ([kernel](findings/kernel.md)).
10. The robot's things as the room's boxes (`machine.law.schema`,
    `machine.body.capsules` the kernel's capsules apart); its arms'
    shield against them through their elbows tried and undone: at a doorway's
    jamb it spared no strike and halved the robot's shoulder's yield
    ([kernel](findings/kernel.md)).
11. The wayfinder's gauge (`machine.way.wayfinder`, `tools/sim/doorway.py`): the
    robot's trunk twisted through the gap 0.6 s ahead; the lodge's door ajar at
    0.8 of its shoulders passed, its waist 28 deg, its arms 55 mm clear
    ([kernel](findings/kernel.md)).
12. The robot's map (`machine.eyes.registry`): the kernel's 256 slots of what
    the robot's looks told it within its eyes' frustum, a thing
    matched to a slot of its kind within its position's precision, weighed by
    the text's log-odds, blind spots kept 45 s and swollen, a mover at its
    velocity; 45 of the lodge's 53 boxes after the robot's bed errand
    ([kernel](findings/kernel.md)).
13. QUIT on the front page, Q and Enter on it; the RS485 plate tells its CRC
    from the robot's wear's drift alarms (WEAR), SERVICE's row named wear.
14. The robot's buses through a noisy line (`edge.noise`, `edge.burst`,
    wbc_sil.c): a corrupt frame refused once and resent the next pass; a bus
    refusing past its horizon 1/omega (306 passes) turns the robot's
    drives off till a restart; Renode's limb takes the same noise on its socket
    (test_buses).
15. The tube on and off (`terminal.ui.tube`), a retro set's switch: a dot over
    the shell's text, a line, the front page unfolding from it; the last frame
    folded to a line and a dot on leaving, 0.4 and 0.3 s.
16. The suite's side-lie (side_lie.lisp): perched on the bed's edge, a
    trochanteric pendulum along it; its handler (`acts._pendulum`) refuses
    the roll past the robot's modes' reach by name, its gym's lie_bed
    kept. stunts_base.lisp renamed motion.lisp.
17. The robot's eyes' things as its registry's looks under
    `physics.SIGHT` (`sight.told`, `vision.LOOKS`), the world's boxes their
    stand-in without it; the chair, table and bed told within 0.10 m.
18. Tour's corner on the robot's closing speed toward the stop (`se2.closing`),
    a spin's drift away no approach; the brake's wire by the gait it was told,
    not its sway's phase: the chair errand sits at 23.7 s where 25.5.
19. The kernel's Coulomb slide as an ask of the robot's step (`wrench.step`, the
    C step and its mirror): a pressing contact's tangential rows freed, its
    friction the kinetic one given, its normal force capped; unasked, the step
    to the bit as before. Measured on the chair's edge sit.
20. A slide said by the robot's text: `(slide v)` in a phase's task constraints
    (`ortho.SLIDE`, the row 304 bytes), carried by its latent (`latent.SLID`,
    wbc_law.c alike) to its step's ask; nothing said, nothing moved.
21. The kernel's reachability on the robot's loop (`machine.law.somatic`): each
    leg's reach over its leg's length rolls its sole's centre of
    pressure onto its ball from 0.86 to 0.98 of it by the furniture's told
    share; the slide's ask laid there too. Standing and sat on the chair,
    nothing moves.
22. The robot's seat one patch of its step (`pressed.patch`,
    wbc_patch.c): its contacts' split pinned as felt; the Coulomb regime as the
    step feels it (`rounds.pushed`, past SLIDE_M_S), the text's slide its cap;
    what bears the robot's capped as told over every patch, no lower than the
    lever's rule; the rise's lean panned to its soles at the handover's
    0.82, its legs' springs back with it, the robot's neck braced
    seated. The chair laid 2e-4 rad either way sat (main fell at -2e-4), square
    and +2e-4 risen at roll 2.4 deg, the dock stood, the robot's walk as it was;
    the -2e-4 rise stays open (protocol 5).
23. The robot's loop's layers live (`machine.process.introspect`,
    findings/introspect.md): latent, impedance and wrench a JSON line each law
    pass to any client attached, every field named from its code's constants;
    idle 604 us a pass where 615 without, 875 read.
24. The robot's telemetry gated (`machine.rig.gate`, `machine.rig.streams`,
    findings/ introspect.md): its analyzers beside its loop, a trigger's half
    swing before and after written gate after gate, 11 streams on their own
    clocks interleaved in time with their metadata; every gated section its
    passes' own to the bit (test_gate), the page's snapshots matched.
25. The robot's pose laid within its joints' stops
    (`physics.World.reset`): its fingers, held a fist at 80 deg, laid at
    the stand's 18 snapped in at 893.5 deg/s each lay, its wrists at
    69.1; now 0.4 and 3.8, its walk as it was (findings/introspect.md).
26. HUMANOID's prompt on its own model (`gynoid.MODELS`, gemma4:e4b: 4.6
    GB on the card where 12b held 8.1), warmed as the page opens and at
    `/`, released as it closes; the Ollama host 127.0.0.1, where localhost
    paid 2.04 s a request: a prompt 2.42 -> 0.11 s, a board-chat question
    8.0 -> 1.4 s (MODELS.md).
27. The robot's classifier on MuJoCo's furniture painted into its map
    (test_sight): each piece's kind, place and floor hull within a voxel at 0.05
    m, the bed moved and turned; a map's voxel its own (`voxels.Map`), the
    robot's eyes' at the 0.1 m they fill; a look's `things` 994 -> 174 ms at
    0.05 m (findings/sight.md).
28. The robot's fake camera, a vision SIL (`machine.eyes.camera`): the robot's
    eye's frames shot on the card where the world runs, its map and
    classifier in a process of their own, its things a look later; the
    robot's loop 1.09 x flat out with it, 1.08 without; a structure told in
    pieces by its cells (test_camera).
29. The rotor page's segments read its feed once a frame in test_views_segments:
    the stepper went 52-80 of 60 degrees under a full run, 56 and 52 on 24
    copies side by side.
30. Both workflows green on main, the offline gate whole on GitHub's
    runner on 3.12 and 3.14: Linux's chair rise, the camera's chair back,
    the replays' walk_turn_right, leap's stride leap and hop, the snow's
    wobble, the sat rise and the floor's scoot red there no more; 124 of
    130 suites, none red (findings/host.md).
31. The gate whole on GitHub's runner, test_wasd 28 of 28 on both jobs at
    587.0 and 593.9 s; each suite's work the runner's own measure in
    tests/work.json (`machine_detector.py --keep --tail`: 19 rows raised,
    none lowered) (findings/host.md).
32. WASD as asked (`humanoid_keys.twist`): A D turn the robot, in place
    alone and on an arc with W S; Q E sidestep; held on the page's worker
    by test_wasd.
33. The robot's buses ride a refused frame (`wbc_sil.c`,
    `machine.senses.edge`): a corrupt broadcast refused by every board on
    its CRC, each holding its last setpoint, the next pass sending anew; a
    bus refused past its balance's horizon (306 passes) raises BusFault,
    every drive off (test_buses).
34. The robot's corners rounded on arcs it walks (`route.rounded`): each
    of A*'s corners on the widest arc its map's free cells and its legs
    give, its turn told round it, a stop only where no arc of its slowest
    walk fits: 59 of the lodge's 60 corners, where a B-spline fitted 3 of
    56 plans (test_sight, [errands](findings/errands.md)).
