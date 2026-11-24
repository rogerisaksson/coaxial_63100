# Architecture: Robotics

The executive any robot here runs on, and the robot round its loop
([planners](planners.md)): its build, its world, its senses, the run, its
identification. The robot's figure: [kinematics](kinematics.md); its drives:
[drivetrain](drivetrain.md); its eyes and its way among its things:
[eyes](eyes.md). Findings by subject: [FINDINGS](../FINDINGS.md#subjects).

## The executive

`machine/rig/` (and `machine/errors`): any board family, no import of one.

| Module | Is |
| --- | --- |
| rig/roles | Input, Stream, Output, Controller; Part: Filter, Estimator, Regulator |
| errors | the library's exceptions |
| rig/controller | a Loop of Feedbacks over float channels |
| rig/cyclic | that pass data-oriented: a frame, the parts as steps over rows, one edge |
| rig/parts, panel, wiring | the parts, the panel, their wiring |
| rig/ansi | the palette |
| rig/sequencer | lines or tables: a step waits for its targets or tests, its time a timeout; jumps, routines; check, summary |
| rig/alarms | L H logged; LL HH and stop() trip; the sequencer's hooks |
| rig/nodes | Node, Nodes, FAMILIES |
| rig/machine | Machine, Actuator, fit; node_hz: a feedback its node runs, the host forwarding |
| rig/modes | where a machine's boards run: the hardware, a stand-in, their images emulated, or none; the tiers the robot's page affords in real time by the machine's cores (`TIERS`, `chosen`), the page's BUSES box saying which runs and its headroom |
| rig/routines | types: a body of subsystems, a bus each |
| rig/live | fed a line at a time, a buffer, woken with a line, a failsafe |
| rig/simulated | pack, camera |
| rig/virtual | VIRTUAL: a type's joints where told, no board |
| rig/dynamic | a body with mass as a Machine's nodes: a drive a joint, the pelvis's pose (`machine.world.physics`) |
| rig/streams | the robot's subsystems' streams, each on its own clock: what a gate records of the robot |
| rig/gate | the robot's streams gated to a file as a scope acquires: a trigger, the windows around it, re-armed |

## The robot

| Module | Is |
| --- | --- |
| body/build | the build as made: its chassis' tubes, its guards and its shells, what they hold, the drives where they sit |
| body/guards | the guards as built: foam slabs on the limbs and the trunk on steel struts off their tubes - their masses and the boxes the floor and the robot's own segments meet |
| body/gait | the robot's gait's numbers of its build: each derived on it, Froude-similar on another's leg and mass |
| body/capsules | the kernel's 18 capsules by its names and pairs on the robot's skin, their clearances in closed form and their log-sum-exp, each pair's and each capsule's evading joints (the text's barriers', its synergies') |
| body/bemf | the robot's drives' torque bounds under their back-EMF: the kernel's quadrant law on its DC link's volts, its boards' current loops' own |
| body/dclink | the robot's DC link: its packs and its boards' capacitance under its drives' power, the boards' d-axis bleed past the text's onset |
| world/physics | DYNAMIC: the figure in MuJoCo, a drive a joint |
| world/room | the robot's glazed mountain lodge: timber walls, a glass gable with the door in it, a roof under snow, a deck with its glass rail and a stone terrace a step up, a loft stair; the furniture, a lamp |
| world/floor | the slab, a sill, a patch, a rug, a stair, bricks, a board; a scene's lay (`lay`), the robot's grounds the lodge's deck and stairs among them |
| world/chair | a chair's body as the floor parks it: its seat at a height, its back, its four legs |
| world/loose | the room's loose things: free bodies on the floor, held there by their feet's grip alone |
| world/harness | the robot's harness: its pelvis hung where it was by springs on its place and its turn, and its load cell |
| world/steps | the world's steps between MuJoCo's in C, where nothing else is asked of them (`tools.cores.world`) |
| world/passing | the world's pass in C laid once (wbc_pass.c): its struct pointed at the robot's arrays and MuJoCo's |
| world/struck | what of the robot touches what is not its own, the world's truth: what struck, how hard, how fast |
| world/referee | the robot's fall judged, the referee's: what of the robot touches its ground, beside its law and not of it |
| senses/edge | the one reader of the world for the robot's law - IMU, encoders, sole loads, a skin a segment, insoles - and the one writer of its drives |
| senses/estimate | the robot's pelvis's place and speed of what it measures: the IMU carried, drawn to its bearing insoles; how fast they slide |
| senses/inekf | the robot's pelvis by a right-invariant EKF on SE_2+3(3) beside the estimate: the IMU's pass means, the soles' and the core's rolling contact points weighted by the observer |
| senses/grip | each sole's floor friction as the robot feels it: what a sliding sole uses |
| senses/touch | what the robot's skin and soles feel, with no branch: upper-body admittance yield, sole trip clearance and continuous twist gating |
| senses/borne | what bears the body off its soles as its skin feels it: the law's support, the step's contacts |
| senses/field | the robot's pelvis's blind seek: where the robot's rows wait on a seat not bearing the robot, the robot's height seeks down at the text's speed through its virtual mass and damping |
| senses/observer | the robot's generalized momentum observer, blind, in C (wbc_gmo.c): the robot's drives' torques, encoders and IMU over the robot's chain's M(q), at the kernel's 35 Hz, split over the kernel's patches by four least squares, each normal pushing; a pass over its working set alone (working_envelope_scoper.lisp): the unknowns that can push, the robot's capsules' pairs staged within the text's envelope, the same bits as the whole |
| senses/momentum | the robot's momentum observer laid in C once on its law's body: its points, pushes, stops and capsules |
| senses/situation | where the robot is, read from its sensors each pass: the text's caps (`defsituation`) blended over the law's rhythm as far as the robot is in each (`motion.gripped`), the deepest one's name to telemetry (`state['situation']`) |
| law/motion | the robot's branchless inner law's pass on its sensors alone: sensed, latent, impedance, wrench (or wbc), directional surety and the drives' observers |
| law/waits | the rows of the robot's table that wait a pass - halting its rhythm while its capture point steps it, raising it while something bears it, lowering it while nothing does - and those that wake its rhythm; in C bit for bit |
| law/gait | the robot's standard gait and its modifiers: the walk's parameters as the lisp's streams fold them, moved by named amounts (pace, stride, swing, sway, roll, turn, rock, crouch, lean, width): a gait take's, a catwalk's roll |
| law/swing | a sole that does not stand: where the robot's modes put it, or at rest on the floor while something bears the robot |
| law/schema | the robot's shield: its posture's ask held to each near pair's ECBF, least in its mass's metric on the pair's evading joints, within their peaks, the rest spilled; its things as the room's boxes, laid once a refresh |
| law/somatic | the kernel's Step 4 on the robot's loop: each leg's reach, hip to sole over the robot's leg's length, rolling the robot's sole's centre of pressure onto its ball from 0.86 to 0.98 of it by the furniture's told share; what bears the robot off the robot's soles capped at its told share of the robot's weight, at the slide regime the robot's text names (`ortho.SLIDE`: 0.55 deeper, 0.70 forward), never under the lever's rule |
| law/barrier | the robot's capture point's barrier: its centre of mass's ask held to each support edge's h' + gamma h >= 0 over what bears it, as far as its rows lean on what it does not feel |
| law/margin | the insert's discrete phase margin: the robot's loop's return ratio at the kernel's four probe frequencies by a first-order quadrature lock-in, each in its turn; the kernel's rollback of K_d and gamma |
| law/supervisor | the anti-spasm insert's numbers laid as the robot's master bus (anti_spasm_supervisor.lisp) |
| moves/seated | the robot's legs and centre of mass sat or sitting down: the knee and ankle a seen seat asks, where they put its centre of mass |
| stack/rounds | the step's unknowns' layout and its rounds' clips, as wbc_rounds.c: a sole's cone, a contact's push within GROW_N of what it bears and its cap, its friction kinetic against a slip felt past SLIDE_M_S along the slide the robot's text says, the faces a wrench is past, what the floor cannot give taken by the robot's inertia striding |
| stack/shrunk | the robot's step's answer inside the robot's drives' bounds (wbc_shrunk.c): each drive's box, the robot's clamp as derated within its back-EMF bounds, an ask shrunk as one into it |
| texts/dsp | the ortho-DSP suite (lisp/humanoid/suite/), the dual-domain kernel's (lisp/humanoid/kernel/, its ops' mechanisms `kernel`) and the runway's dialect evaluated to gaits: their stages, transitions and maneuvers' acts in order |
| texts/packages | the ortho-DSP's lisp modules on disk: a folder's files, each after those its package uses |
| texts/patch | a transition on the robot's loop: lisp read as forms, a morph as contiguous curve rows on the bar grid, begun locked to its double support |
| texts/maneuver | the suite's maneuvers as the robot's texts: a stunt's patch played on the one it rises from, its acts the robot's phases, each paced to its springs, its arms held; a preload before a cross-fade onto its soles pans what bears it and its legs' springs to them; the bed's patches read, not played |
| texts/acts | the suite's acts as phases of the robot's text (`maneuver`): an envelope's knee its height, a seat's landing (`seated`), a cross-fade the furniture taking it, a cascade or a gate a node a phase, a recline its trunk's ramp, a pendulum refused past its modes' reach |
| texts/curves | the suite's curves as shares of the way, and as quintics for a row's blend |
| texts/voicing | a gait voiced on the robot's walk's row by its changes off the nominal; what has no voice named, what the robot cannot do refused by name |
| texts/tune | the autotuner's text (meta_ast_autotuner.lisp) on the robot's texts, the suite's and its gym's given as an argument: its grammar, each literal a root reaches where it stands (the robot's gym's by the slot its clause lays), what each moves of the robot by evaluation, the damped least-squares step, the texts spliced |
| texts/kernel | the dual-domain kernel's ops by the mechanism each feeds; its texts read and their stages' defaults |
| texts/lowering | the kernel's AST-to-SoA lowering: a gait folded into a bank of its ABI over the robot's nominal's (wbc_soa.h: the text's fields at its offsets, a biquad a node seeded on the robot's value, the manifolds' weights, the capsules), carried as an rcu table's words; the robot's loop's biquad in C |
| texts/takes | the mocap takes as rows for the robot's loop: a posture take's voices at its beats, a gait take's amounts on the robot's standard gait, entered over the robot's leg's half period, told by the bench |
| texts/gcode | humanoid G-code: a general rig's joint angles from a mocap, made the robot's own, read as its program |
| texts/says | what the robot can be told, scanned from the sources themselves - no register written by hand |
| texts/calls | what the robot's law calls, by name, a count each, between two takes: its own functions and its C cores |
| texts/kept | a pure function's results kept pickled by their key: each call its own copy, made once |
| process/sprung | the latent loop run: the robot's world read and told through its edge, its law's pass between; its process, paced to the clock: the one thing that moves it |
| process/worker | the robot's outer loop and session: discrete commands mapped to continuous setpoints, each logged with its epoch and pass, falls recovered; the page's states |
| process/gaits | the robot's gaits as said: a gait entry heard, laid as rows on its table from where it is |
| process/joints | the robot's joints driven as said: a program of poses on the drives' own servos, its law's torques aside |
| process/sensed | the robot's law's sense of it at its lay (`wbc.sense`), as its sensors and its estimate have it |
| process/page | the robot's state as its page hears it: its stage, modes, horizon and things; its tilt as the world has it |
| process/rcu | the robot's loop and a tuner apart with no lock: rings of its passes, a double bank of tables in 64-byte lines (SoA), a seqlock and a took word between them; its loop's side C (wbc_rcu.h), the block locked in memory |
| process/tap | the robot's loop's side of a tuner: its raw passes copied (its contacts in C, `wbc_pass_tapped`), cooked and measured on the tuner's core, a table laid at its barrier (lateral crossing, both soles locked, its last blend done) blended minimum-jerk |
| process/worn | the robot's wear's windows beside its loop: each read and filtered, its load laid LAND passes on |
| process/watch | the robot's telemetry reduced: analyzers firing sparse events on its body's own triggers |
| process/introspect | the robot's loop's latent, impedance and wrench layers live, for a debugger or the local model to attach to |
| process/kinematics | the robot's measured modes, way-frame kinematics and drive commands named for introspection |
| process/apart | a world stepped in a process of its own, paced to the wall and kept to what its page has shown |
| process/aside | a process beside the robot's loop's: a module of this tree served on its own, asks pickled down its stdin |
| process/placed | the robot's processes placed by the host's own topology: a core apart, the robot's loop's on a cache of its own |
| process/commit | the commit the robot's recordings are made at: HEAD of the checkout their folder sits in (`head`) |
| identify/sysid | the robot's body identified of its edge alone: Newton-Euler down its tree, the base parameters; its pelvis's rows off its insoles or a load cell |
| identify/regressor | the robot's Newton-Euler regressor: tau = Y(q, v, a) theta, its drives' torques at N states of its model |
| identify/campaign | the stand's campaign; each gearbox and rotor read |
| identify/wear | the robot's drift and a load off its rows on-line; bands, alarms, service |
| identify/identified | the robot's corrections over its build: the least move of its links and drives that fits a campaign, TOML, laid over its law's model whole |
| analyzers/said | what the robot was told and how it went |
| analyzers/phases | the robot's doing's flips, as `told.Run` counts them |
| analyzers/motion | the robot's joints' motion a phase, on the peaks held between snapshots |
| analyzers/support | what bears the robot, as `told.Run.borne` and the fall probes read it |
| analyzers/landing | what of the robot strikes what it falls on, how hard and how fast: the world's truth (`struck`) |

- `host/tools/sim/`: look (a HUMANOID recording a stage a row); replay (a
  recording's commands run again, pass for pass); scenarios (each scene
  placed, commanded and judged, a table); stunts (the mocap takes as
  scenarios: a move's measures from joints' places, a take's and its own
  alike, the takes' kept in tests/takes/stunts.json); sysid (the robot's
  identification campaign on the stand, tabled, its corrections written);
  strides, normal, looks (a walk's measures, a woman's band, the robot's
  form); identify (a take as an ortho-DSP modifier against its walk,
  numpy alone); suite (the suite's patches and the takes' modifiers played on
  the robot, measured by it); told (the robot's scenarios, each alone on the
  robot's loop as the page runs it); house (its errands from
  every spot of the lodge, said as F says them); ail (the robot driven about
  the lodge on its keys, its faults judged by the world's truth); falls (its
  falls by the world's truth: what struck first, how hard, its spasms);
  pace (its loop's pace as its worker runs it, a craft's as its page flies
  it); introspected (its law's layers read as it runs, each pass's snapshot
  by name); recording (a recording of its loop: blocks, events, each window
  a table); mocap (the takes' walks measured as look measures its own);
  offered (F's errand walked into as test_errands judges it, an ensemble);
  wearing (its wear filter's step response, an ensemble); inertias (its
  figure's inertias as MuJoCo compiles them here, bit for bit); pivot (its
  pivot on an open floor: a half turn by a way of turning, measured).

## Suites

| Suite | Holds |
| --- | --- |
| `test_gynoid.py` | the body on fantasy boards (`physics.ENVELOPE` 0, their SOA never binding): clothes, look, a leg by its foot, a virtual and a dynamic machine, the skeleton, the drives' mounts and gearboxes, kinematics as motors |
| `test_dsp.py` | the ortho-DSP suite read as written, the kernel read whole, a patch voiced on the robot's walk by its changes, a gallop, a sprint and a carioca refused by name, the runway's dialect alike, the robot's walk converted against itself its walk |
| `test_kernel.py` | the dual-domain kernel on the robot's loop: its momentum observer against its skin walking and seated; its chest's yaw leaking nothing standing; what bears it weighed as felt; the capture point's barrier; a pulled chair; the observer's working set scoped as whole, bit for bit, walking and crouched |
| `test_tune.py` | the autotuner's text read; the robot's texts given play as its files; every literal walked where it stands, the robot's gym's by its slot; each one's reach on the robot - the runway's, the sit's, its walk's -; its step a QP; a run's residuals and the verdicts on a step; a link's rings and banks, the robot's walk linked as unlinked bit for bit, its raw passes cooked as MuJoCo reads them, its walk measured at runtime with its loop apart, a table laid at its barrier blending C2 |
| `test_floor.py` | the robot's floor: the plane under its soles, its grip as the robot feels it, the situations that cap its rhythm |
| `test_situation.py` | the robot's situation: the text's bounds and caps, each entered unannounced told within a second |
| `test_gvs.py` | the measures' dimensionless frame (Hof), SO(3)'s geodesic, the gates on them (`tools/sim/gvs.py`) |
| `test_scenarios.py` | the robot's scenarios that hold (`tools/sim/scenarios.py`): a scene placed, commanded as a player would, its goal met by the world's truth |
| `test_replays.py` | the robot's runs recorded on the page (`tests/replays`), each command at its pass, none ending in a fall; a fall recorded on the page kept in `tests/replays/falls` till it stands (none open) |
| `test_sysid.py` | the robot's identification: drives, links, a payload, boxes, rotors; corrections; a world 15 % off; a harness |
| `test_wear.py` | on-line: a load read walking, a knee's friction and a hip's torque named; a box's play |
| `test_sysid_live.py` | the robot's identification off the stand: its model held to its walk on a world off the figure; its weight and its moments standing on the robot's insoles |
| `test_compiled.py` | the robot's numpy compiled to C (`tools.cores.bodied`) beside numpy's own: its reductions and ties of zeros, the robot's state in place, its sensing at random configurations, its law's compiled parts at every pass walking |
| `test_ported.py` | the robot's law's hand ports in C beside their python at every pass of its walk, bit for bit: its surety field, its touch's filter, its twist's step, its table's waits, its brace on the solve; each step's time |

## Software Systems Engineering Architecture

The inner modules `test_ortho_texts` holds (`motion`, `touch`, `grip`,
`latent`, `modal`, `impedance`, `swing`, `poise`, `stride`, `treads`,
`rocker`, `vault`, `support`) are branchless: no `if`, no mode string, no
threshold. Commands enter through the outer setpoint loop (`worker`,
`se2.heard_twist`); a situation's depth caps the law's rhythm
(`motion.gripped`), its name goes out to telemetry (`state['situation']`).

![Software systems engineering and real-time control architecture](software_layout.svg)
