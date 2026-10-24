# Architecture: Robotics

The executive any robot here runs on, and the gynoid round her loop
([planners](planners.md)): her build, her world, her senses, the run, her
identification. Her figure: [kinematics](kinematics.md); her drives:
[drivetrain](drivetrain.md). Findings by subject:
[FINDINGS](../FINDINGS.md#subjects).

## The executive

`machine/`: any board family, no import of one.

| Module | Is |
| --- | --- |
| roles | Input, Stream, Output, Controller; Part: Filter, Estimator, Regulator |
| errors | the library's exceptions |
| controller | a Loop of Feedbacks over float channels |
| cyclic | that pass data-oriented: a frame, the parts as steps over rows, one edge |
| parts, panel, wiring | the parts, the panel, their wiring |
| ansi | the palette |
| sequencer | lines or tables: a step waits for its targets or tests, its time a timeout; jumps, routines; check, summary |
| alarms | L H logged; LL HH and stop() trip; the sequencer's hooks |
| nodes | Node, Nodes, FAMILIES |
| machine | Machine, Actuator, fit; node_hz: a feedback its node runs, the host forwarding |
| modes | where a machine's boards run: the hardware, a stand-in, their images emulated, or none; the tiers her page affords in real time by the machine's cores (`TIERS`, `chosen`), the page's BUSES box saying which runs and her headroom |
| routines | types: a body of subsystems, a bus each |
| live | fed a line at a time, a buffer, woken with a line, a failsafe |
| simulated | pack, camera |
| virtual | VIRTUAL: a type's joints where told, no board |
| dynamic | a body with mass as a Machine's nodes: a drive a joint, the pelvis's pose (`machine.physics`) |

## The gynoid

| Module | Is |
| --- | --- |
| build | the build as made: carbon-fibre shells over the shape, what they hold, the drives where they sit |
| physics | DYNAMIC: the figure in MuJoCo, a drive a joint |
| room | her glazed mountain lodge: timber walls, a glass gable with the door in it, a roof under snow, a deck with its glass rail and a stone terrace a step up, a loft stair; the furniture, a lamp |
| floor | the slab, a sill, a patch, a rug, a stair, bricks, a board; a scene's lay (`lay`), her grounds the lodge's deck and stairs among them |
| sight | humanoid robot eyes: a stereo pair on humanoid robot head, the disparity by block matching, the map she keeps, its things, humanoid robot way round them |
| stereo | the stereo disparity on the card (wgpu or cupy) or the CPU: block matching over DISPARITIES |
| gimbal | a camera on a stabilised gimbal: its sight the world's, leant a little into a turn |
| edge | the one reader of the world for her law - IMU, encoders, sole loads, a skin a segment, insoles - and the one writer of her drives |
| estimate | her pelvis's place and speed of what she measures: the IMU carried, drawn to her bearing insoles; how fast they slide |
| grip | each sole's floor friction as she feels it: what a sliding sole uses |
| touch | what her skin and soles feel, with no branch: upper-body admittance yield, sole trip clearance and continuous twist gating |
| borne | what bears the body off its soles as its skin feels it: the law's support, the step's contacts |
| situation | where she is, projected outward from her continuous sensors and modal state to telemetry (`state['situation']`) |
| motion | her branchless inner law's pass on her sensors alone: sensed, latent, impedance, wrench (or wbc), directional surety and the drives' observers |
| waits | the rows of her table that wait a pass - halting her rhythm while her capture point steps her, raising her while something bears her, lowering her while nothing does - and those that wake her rhythm; in C bit for bit |
| sprung | the latent loop run: her world read and told through her edge, her law's pass between; her process, paced to the clock: the one thing that moves her |
| gait | her standard gait and its modifiers: the walk's parameters as the lisp's streams fold them, moved by named amounts (pace, stride, swing, sway, roll, turn, rock, crouch, lean, width): a gait take's, a catwalk's roll |
| patch | a transition on her loop: lisp read as forms, a morph as contiguous curve rows on the bar grid, begun locked to her double support |
| dsp | the ortho-DSP suite (machine/ortho_dsp/), the dual-domain kernel's (machine/ortho_kernel/, its ops' mechanisms `kernel`) and the runway's dialect evaluated to gaits: their stages, transitions and maneuvers' acts in order |
| maneuver | the suite's maneuvers as her texts: a stunt's patch played on the one it rises from, its acts her phases, each paced to her springs, her arms held; the bed's patches read, not played |
| seated | her legs and centre of mass sat or sitting down: the knee and ankle a seen seat asks, where they put her centre of mass |
| furniture | what her eyes tell her of a thing a verb goes to: its top, its near edge, its centre |
| route | her way round the furniture to an errand's dock: what stands on her floor grown by her radius on her map's cells, A* through the rest, its corners walked through on a B-spline held in convex corridors by linear rows where it fits her arc; each errand's dock |
| curves | the suite's curves as shares of the way, and as quintics for a row's blend |
| swing | a sole that does not stand: where her modes put it, or at rest on the floor while something bears her |
| voicing | a gait voiced on her walk's row by its changes off the nominal; what has no voice named, what she cannot do refused by name |
| tune | the autotuner's text (meta_ast_autotuner.lisp) on her texts, the suite's and her gym's given as an argument: its grammar, each literal a root reaches where it stands (her gym's by the slot its clause lays), what each moves of her by evaluation, the damped least-squares step, the texts spliced |
| rcu | her loop and a tuner apart with no lock: rings of her passes, a double bank of tables in 64-byte lines (SoA), a seqlock and a took word between them; her loop's side C (wbc_rcu.h), the block locked in memory |
| observer | her generalized momentum observer, blind, in C (wbc_gmo.c): her drives' torques, encoders and IMU over her chain's M(q), at the kernel's 35 Hz, split over the kernel's patches by four least squares, each normal pushing |
| kernel | the dual-domain kernel's ops by the mechanism each feeds; its texts read and their stages' defaults |
| tap | her loop's side of a tuner: her raw passes copied (her contacts in C, `wbc_pass_tapped`), cooked and measured on the tuner's core, a table laid at her barrier (lateral crossing, both soles locked, her last blend done) blended minimum-jerk |
| takes | the mocap takes as rows for her loop: a posture take's voices at its beats, a gait take's amounts on her standard gait, entered over her leg's half period, told by the bench |
| worker | her outer loop and session: discrete commands mapped to continuous setpoints, each logged with its epoch and pass, falls recovered; the page's states |
| tour | the scenes one after another, commanded as an agent commands them |
| sysid | her body identified of her edge alone: Newton-Euler down her tree, the base parameters; her pelvis's rows off her insoles or a load cell |
| campaign | the stand's campaign; each gearbox and rotor read |
| wear | her drift and a load off her rows on-line; bands, alarms, service |
| identified | her corrections over her build: the least move of her links and drives that fits a campaign, TOML, laid over her law's model whole |

- `host/tools/sim/`: look (a HUMANOID recording a stage a row); replay (a
  recording's commands run again, pass for pass); scenarios (each scene
  placed, commanded and judged, a table); stunts (the mocap takes as
  scenarios: a move's measures from joints' places, a take's and hers
  alike, the takes' kept in tests/takes/stunts.json); sysid (her identification
  campaign on the stand, tabled, its corrections written); strides, normal,
  looks (a walk's measures, a woman's band, humanoid robot form); identify (a
  take as an ortho-DSP modifier against her walk, numpy alone); suite (the
  suite's patches and the takes' modifiers played on her, measured by it);
  told (her scenarios, each alone on her loop as the page runs her); house
  (her errands from every spot of the lodge, said as F says them).

## Suites

| Suite | Holds |
| --- | --- |
| `test_gynoid.py` | the body on fantasy boards (`physics.ENVELOPE` 0, their SOA never binding): clothes, look, a leg by its foot, a virtual and a dynamic machine, the skeleton, the drives' mounts and gearboxes, kinematics as motors |
| `test_sight.py` | the stereo against the renderer's depth; a look round, posed standing, into humanoid robot map against the room, the chair and the table told, a way planned clear; the page's picture |
| `test_dsp.py` | the ortho-DSP suite read as written, the kernel read whole, a patch voiced on her walk by its changes, a gallop, a sprint and a carioca refused by name, the runway's dialect alike, her walk converted against itself her walk |
| `test_kernel.py` | the dual-domain kernel on her loop: her momentum observer against her skin walking and seated; her chest's yaw leaking nothing standing |
| `test_tune.py` | the autotuner's text read; her texts given play as her files; every literal walked where it stands, her gym's by its slot; each one's reach on her - the runway's, the sit's, her walk's -; its step a QP; a run's residuals and the verdicts on a step; a link's rings and banks, her walk linked as unlinked bit for bit, her raw passes cooked as MuJoCo reads them, her walk measured at runtime with her loop apart, a table laid at her barrier blending C2 |
| `test_floor.py` | humanoid robot floor: the plane under humanoid robot soles, its grip as she feels it, the situations that cap humanoid robot rhythm |
| `test_situation.py` | humanoid robot situation: the text's bounds and caps, each entered unannounced told within a second |
| `test_gvs.py` | the measures' dimensionless frame (Hof), SO(3)'s geodesic, the gates on them (`tools/sim/gvs.py`) |
| `test_scenarios.py` | humanoid robot scenarios that hold (`tools/sim/scenarios.py`): a scene placed, commanded as a player would, its goal met by the world's truth |
| `test_replays.py` | humanoid robot runs recorded on the page (`tests/replays`), each command at its pass, none ending in a fall; the falls still open in `tests/replays/falls` |
| `test_sysid.py` | her identification: drives, links, a payload, boxes, rotors; corrections; a world 15 % off; a harness |
| `test_wear.py` | on-line: a load read walking, a knee's friction and a hip's torque named; a box's play |
| `test_errands.py` | errands: a stop settles, F heard, a new scene walked |
| `test_sysid_live.py` | her identification off the stand: her model held to her walk on a world off the figure; her weight and its moments standing on her insoles |
| `test_compiled.py` | her numpy compiled to C (`tools.cores.bodied`) beside numpy's own: its reductions and ties of zeros, her state in place, her sensing at random configurations, her law's compiled parts at every pass walking |
| `test_ported.py` | her law's hand ports in C beside their python at every pass of her walk, bit for bit: her surety field, her touch's filter, her twist's step, her table's waits, her brace on the solve; each step's time |

## Software Systems Engineering Architecture

All inner control, contact and observer modules (`motion`, `touch`, `grip`,
`se2`, `latent`, `modal`, `impedance`, `swing`, `poise`, `stride`, `support`, `borne`,
`wrench`) operate as a branchless physical and mathematical continuum on SE(3)
and modal vectors without conditional logic (`if`-statements, mode-string
branches or discrete thresholds). External commands enter through the outer
setpoint loop (`worker`, `se2.heard_twist`), and discrete situation labels are
projected strictly outward (`situation` -> `state['situation']`) for telemetry
and external planners.

![Software systems engineering and real-time control architecture](software_layout.svg)
