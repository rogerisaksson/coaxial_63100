# Scenarios, by the general case

Each a test in MuJoCo: placed, commanded as a player would, judged by no
fall, its goal reached and its jerk (`tools/sim/scenarios.py`; those that hold
in `test_scenarios`: [scenarios](../findings/scenarios.md)). In order: each a
mechanism, the scenes it serves its DOD; none an entry a case.

1. **The robot's support, whatever bears it** (`machine.senses.borne`): a
   segment the skin feels pushed up is in its support region, grounds it
   and is a contact of its whole-body step; the board's step (`wbc_wrench.c`)
   knows its soles alone. The floor's rise: item 14. DOD: chair, soft bed,
   floor mattress and floor, down and up, no fall called; the bed lain
   lengthwise from the errand's dock, its soles on the mattress, and risen
   (test_errands).
2. **Weight handed over, never switched**: a sole's share of the robot's
   support continuous in its height's reading, its support region and its
   step's bounds, where a flag now. Where its support flickers the robot's
   step makes its head and arms the lever: a sole let go on its toes, a
   hurried swing past a hip's clamp, a seat's touch flickering as it rises,
   a hop's landing, a row's blend stepping its asks, its head flung at a
   spin's stop, reversing and seated (`tools/sim/jerk.py`,
   [jerk](../findings/jerk.md)). DOD: every segment's peak through stand,
   walk, turn, strafe, back, stop, sit, rise, hop no higher than in its
   steady walk; a test holds it.
3. **A landing on what is there**: a swinging sole that meets something
   raises its swings by its height, one that meets nothing reaches on down,
   a sliding one lowers the friction it asks. The floor ahead a plane of the
   robot's standing soles' pitch (`modal.under`): past a crest or down a
   descent a swing hangs over the floor; on every descent its centre of mass
   runs on past a lone standing sole while the next swing is late; the first
   step down a flight, nothing told, a drop; its climb's rhythm the
   impedance's, not the takes' ([stairs](../findings/stairs.md)). DOD:
   stairs of 0.15 x 0.30 m up and down, a slope of 10 deg up, down and
   across; 3 times in 4 each.
4. **The robot's floor identified, a text over the general law**: what its
   standing soles meet - slip and sink under their load - read from its
   sensors against what its law assumes (its step's friction, a rigid floor)
   and estimated as data; outside that envelope a surface's text
   (`situations.lisp`) laid over the general law's - the friction its step
   asks, its rhythm, its step's length, its stiffness -, no branch in the
   law ([grip](../findings/grip.md)). Left: a floor its soles never slide on
   (asphalt, snow) read from what its holding soles use alone; its insoles
   reading their normal load alone; skates, a sole's room a line along its
   blade. DOD: on skates the robot stands 10 s and goes 2 m by strides
   across its blades.
5. **Reachable landings ahead**: where each limb lands before the robot's
   capture point is past it, read from its reach (`machine.body.reach`);
   none, its height lowered and a limb set to meet the fall - knees, then
   hands, the head off the floor ([body](../findings/body.md)). DOD: 200 N
   falls with the head under 1 m/s (`tools/sim/falls.py`).
6. **One jump** (`machine.moves.leap`, `terminal.views.humanoid_keys`):
   takeoff from the height and distance to land, its throw by `Space`'s
   hold. The chair's two texts reach no hop; off and onto a 0.45 m seat the
   robot's knees work at their peak, past the drives' 50 % margin - the
   seat's height or the knees' drives to decide
   ([demand](../findings/demand.md)). DOD: down from a chair, two-footed up
   onto one (0.45 m), in place; landed standing.
7. **The drives for it**: in the turns' play and the weave the robot's
   hips, knees and spine ask past 2/3 of their parts, a held drive granted
   its peak past its clamp (`ask['wep']`, `machine.stack.shrunk`)
   ([demand](../findings/demand.md)). DOD: each scenario's peak, rms, speed
   and heat under 2/3 of its parts (`tools/sim/demand.py`).
8. **The takes as the robot's moves** (`tools/sim/stunts.py`,
   `scenarios.STUNTS`): each take a scenario, told as a player would, its
   rows measured as the take's and judged within a sole's length, its list,
   its arm's cone and one natural time, its head no lower than the human's
   and under 1 m/s if it falls ([stunts](../findings/stunts.md)). Short of
   them: the seats, the lie, the jumps' throw, the runs' flight and their
   brake at speed, the falls, the yawn's overhead arms; the lisp's gaits
   beside the walk (trip, jog, ballistic_walk, stairs_walk, ice_walk) not
   yet amounts on its one walk (`machine.law.gait`). DOD: every take met at
   every wobble, each stunt's jerk within its walk's; the runway patch
   (`lisp/humanoid/runway.lisp`) wet, track and all, 26 bars standing, the
   feet never into each other passing; three of the pack's walks
   (`machine.texts.dsp`, `tools/sim/suite.py`) at their tempo and stride
   within 10 %, 20 s standing.
9. **The robot's place a glazed mountain lodge** (asked, pictures given;
   `machine.world.room`, [lodge](../findings/lodge.md)): the descent down
   the wooden stair falls as item 3's stairs do; the loft stair and the
   stone step walked by no scene; the pictures' snow on the ground, the
   valley's horizon and the gable's glass over the eaves undrawn. DOD:
   lodge_down and a climb of the loft stair held in test_scenarios; the
   ground snowed and the valley drawn beyond the rail.
10. **An errand from every spot** (`tools/sim/house.py`,
    `machine.way.route`): said from every spot of the lodge's floor the
    robot's chair and bed errands fall or spasm - the chair's rise leaves
    it going on past its soles; sat up on the bed swing_out throws its
    trunk; turning and backing before the table its hands touch the loft;
    its arms at its stop, its neck seated and reclining; the bed's dock
    turns on its start's last bits ([spots](../findings/spots.md),
    [errands](../findings/errands.md)). DOD: the chair's, the bed's and the
    door's errands from every pose the robot sees them from and said from
    every spot, the page's prompt among them, no fall, no spasm; the
    viewport's bed errand 11 of 11 starts 1e-4 rad apart, the chair's 5 of 5
    (`tools/sim/offered.py`).
11. **The robot's soles' friction a pyramid along them**: its step clips a
    sole's force to mu along the world's x and z (`machine.stack.rounds`'
    `bounded`, `pushed`, `faced`, `taken`; `wbc_rounds.c`, `wbc_cone.c`,
    `wbc_stack.c` alike), so its walk differs by heading
    ([jerk](../findings/jerk.md)). DOD: its walk the same every way, the
    python twin bit for bit, no scene lost.
12. **The anti-spasm insert's open stages** ([synth](../architecture/synth.md),
    [insert](../findings/insert.md), [kernel](../findings/kernel.md)): off the
    robot's loop - its velocity gate, its damped least squares on a swung leg
    (the heel's cap a woman's at lift with it), its D low-pass at 42 Hz, its
    zero crossing's bridge, its C2 85 ms handover and its Schmitt comparators
    on its soles; each alone, tried, felled a walk, a turn, a leap or a seat.
    DOD: each on its loop at its text's numbers, every suite as green as the
    base.
13. **Wear alarms off a seat and a fall**: the robot's wear filter
    (`machine.identify.wear`) reads a seat's and a fall's forces as its
    drives' drift and names it - sat on the lodge's chair, sliding off it
    ([identification](../findings/identification.md)). DOD: no alarm through
    the chair's sit, rise and a shove's fall.
14. **The lie as a floor primitive** ([lie](../findings/lie.md)): told to,
    the robot lies down where it stands and rests limp (`se2.LIMP`); a rise
    from the floor is refused by its legs' reach (`machine.texts.maneuver`):
    below a pelvis of 0.51 m the compression pattern binds the ankle at
    -45 deg - the heels-up redistribution (the kernel's
    :metatarsal-equinus-bilateral manifold) or a tibial kneeling bus. DOD:
    laid supine the robot rises to a stand, 3 of 3; the lie flat, rolled
    within LIST.
15. **The robot's posture a 1-jet a manifold** (asked; the kernel's
    `canonical_posture_bundle.lisp`): `modal.poses` lays its legs as what
    bears it holds them (`latent.MANIFOLDED`) - soles, side and back the
    crouch's column, a seat its thigh (`modal.seat`), a let-go leg an open
    chain (`machine.moves.emancipation`); sat, its ask's hip and ankle stand
    off its legs, the soles' share asking a crouch of legs the seat and the
    floor hold together; the atlas's origins and columns its text's, read by
    no law ([support](../findings/support.md)). DOD: sat, the ask's legs
    within 1 deg of its own (test_postures); a let-go thigh level; no scene
    lost.
