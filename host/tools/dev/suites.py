"""The suites: their names, which join a run, the tiers (the path map: `touches`)."""
import ast
import re
from pathlib import Path

from tools.dev.suites_views import VIEWS      # every live view's suite, a file a subject


ROOT = Path(__file__).resolve().parents[2]           # host/

STRUCTURE = 'test_structure.py'
#: The whole tree on one trunk, the terminal up to the shunts, its ties read off the code (tools.dev.atlas).
ATLAS = 'test_atlas.py'
#: Orthokinematics' texts: compiled to rows, what is no text refused, the robot's law's code branchless
#: (test_ortho's once).
ORTHO_TEXTS = 'test_ortho_texts.py'
#: The static meta-compiler held to itself: one source a number, a build anew its cache, similarity (machine.body.compile).
METACOMPILER = 'test_metacompiler.py'
#: The crafts' spec held to its compilers: a craft states its own alone, a wing's numbers its own (machine.aircraft.polars).
METACOMPILER_CRAFTS = 'test_metacompiler_crafts.py'
#: The rig held to the library: what a sheet states is read of its file and stated nowhere else (coaxial.model.inverter).
METACOMPILER_RIG = 'test_metacompiler_rig.py'
#: The static solver held to its physics: a loop's margin, its bounds, the inertia a drive sees (machine.solver).
METACOMPILER_SOLVER = 'test_metacompiler_solver.py'

#: The structure family, first in every run: host/ holding together, the tree's shape, the
#: firmware and the host agreeing - one 17 k-token file before; every path on the atlas's trunk;
#: the robot's texts compiling and its law's code branchless; what its specs state compiled the same twice.
STRUCTURES = (STRUCTURE, 'test_layout.py', 'test_mirrors.py', 'test_wire_shapes.py',
              'test_wire_requests.py', 'test_protocol_doc.py', ATLAS, ORTHO_TEXTS, METACOMPILER,
              METACOMPILER_CRAFTS, METACOMPILER_RIG, METACOMPILER_SOLVER)
#: Suites with prior CI or local regressions, scheduled right after structural checks so a regression stops the run at once:
#: what GitHub's last host run left red is laid at its head at every commit (tools/dev/ci.py, the pre-commit).
FAILED_FIRST = ('test_introspect.py', 'test_errands.py', 'test_leap.py', 'test_scenarios.py',
                'test_wasd.py', 'test_smart.py', 'test_sysid_live.py', 'test_ortho.py',
                'test_floor.py', 'test_wbc_core.py', 'test_wear.py', 'test_daq_api.py',
                'test_kernel.py', 'test_docks.py', 'test_ollama_runner.py', 'test_se3_core.py',
                'test_views_apps.py', 'test_native.py', 'test_mirrors.py', 'test_quick.py',
                'test_sysid.py', 'test_tick.py', 'test_dsp.py', 'test_wayfinder.py',
                'test_views_grove.py', 'test_camera.py', 'test_sight.py', 'test_gaze.py',
                'test_replays.py', 'test_bedside.py', 'test_substrate.py', 'test_vault.py',
                'test_emulator.py')

CORE = 'test_modbus_core.py'

SHTP = 'test_shtp_core.py'

DRIVE = 'test_drive_core.py'
DRIVE_OBSERVER = 'test_drive_observer.py'

FILTER = 'test_filter_core.py'

#: The thermal envelope and the applications' networks as the C that will run on the board.
THERMAL, THERMAL_APP = 'test_thermal_core.py', 'test_thermal_app.py'

#: The acquisition engine as the C that will run on the board - the
#: ring, the window, the ladder, the tone, the live accumulator -
#: hardware-free; before, tested only by the
#: bench after a flash.
DAQ_CORE = 'test_daq_core.py'
BOARD_MEM = 'test_board_mem.py'

#: The bootloader's state machine as the C that will run - the chunk
#: stream, the bitmap, the seal - on a RAM flash, before any register
#: is touched (docs/BOOT.md).
BOOT_CORE = 'test_boot_core.py'
#: The host's client, `load` and the front door's own-image step through that C.
BOOT_CLIENT = 'test_boot_client.py'

#: machine/'s parts and feedback as the C a board will loop on, stepped beside
#: host/machine/rig/parts.py.
CTRL_CORE = 'test_ctrl_core.py'

#: What the motors turn (world/): loads and bodies against closed forms.
WORLD_CORE = 'test_world_core.py'

#: The device clients through the firmware's own wire - comms/ built for this host over a
#: fake board (tools.cores.fakeboard): a disagreement between the two sides fails here.
WIRE = 'test_wire.py'

#: The real-time engine for SIL and HIL (tools.cores.native): the board layer's drive path on
#: this host keeps the wall's time; the firmware's validation is EMULATOR's.
NATIVE = 'test_native.py'
NATIVE_HEAT = 'test_native_heat.py'
#: A drive identified on its native board: held and swung by the board's loop, its DAQ the rows.
NATIVE_SYSID = 'test_native_sysid.py'

#: The firmware's own image on an emulated MCU (tools/emu, board/emu): the conformance suite
#: and test_wire's sweeps against the ELF, CubeMX's code and the HAL included.
EMULATOR = 'test_emulator.py'

SENSORLESS = 'test_sensorless.py'

#: The subjects a change can be about: pick_tests.py asks the model to choose
#: from them, and each names one test_ollama_<tag>.py.
TAGS = {
    'tools': 'the tool surface: schemas, arguments, which tool answers what',
    'runner': 'the plan runner, the sandbox, and the test tooling itself',
    'prompt': 'SYSTEM, the per-turn hints, what the model is told',
    'link': 'the serial link: ports, probing, diagnosis, recovery',
    'render': 'how a result reaches the screen: columns, blocks, clipping',
    'bus': 'nodes, segments, unit ids, broadcast',
    'board': 'the board, its channels, its pins, the AFE',
    'reply': 'what an answer means: retypes, blank answers, nudges',
    'language': 'the session language, its lock, and the phrase table',
}

#: The ollama suites, a file a subject - 12 to 218 checks - a budget chooses among them.
OLLAMA = tuple('test_ollama_%s.py' % tag for tag in TAGS)

BENCH = 'test_bench.py'

#: The robot's loop's pace, each errand alone at a benchmark's priority, against this host's recorded floor.
PACE = 'test_pace.py'

BROKER = 'test_broker.py'

#: The acquisition front door against the stand-in - naming, reading,
#: the record's shape, the buffers. No board and no compiler, so it is
#: one of the cheapest suites here and joins first.
DAQ_API = 'test_daq_api.py'

#: The master's side of the bootloader against the stand-in's blank
#: node - no board, no compiler, a second (docs/BOOT.md).
BOOT = 'test_boot.py'

#: The composed controller and its parts, against a toy rotor and the stand-in.
CONTROLLER = 'test_controller.py'

#: The gynoid's body on fantasy boards, their SOA never binding: its clothes, its look, its
#: skeleton, its drives' mounts and gearboxes; the whole-body stack; its eyes; its fake camera; its gaze.
GYNOID, WBC, SIGHT, CAMERA, GAZE = 'test_gynoid.py', 'test_wbc.py', 'test_sight.py', 'test_camera.py', 'test_gaze.py'
REACH, TERMS = 'test_reach.py', 'test_terms.py'
#: The loop in C against MuJoCo and the python stack (wbc/); leaps on both; `machine.stack.wrench`; the closed chains'.
WBC_CORE, LEAP, WRENCH, CHAIN = 'test_wbc_core.py', 'test_leap.py', 'test_wrench.py', 'test_chain.py'
#: WASD on the robot: the page's keys drive it, and the player's plays from each start stand without a
#: stumble.
WASD = 'test_wasd.py'
#: The stack's dual active set in C beside its python on random QPs (tools.cores.qp).
QP = 'test_qp.py'
#: The robot's body identified of its edge alone, on the stand (machine.identify.sysid).
SYSID, SYSID_LIVE = 'test_sysid.py', 'test_sysid_live.py'
#: The robot's numpy compiled to C beside numpy's own (tools.cores.trace, emit, held, bodied).
COMPILED = 'test_compiled.py'
#: The robot's compiled parts called whole in C beside their Python glue, call by call walking (tools.cores.quick's
#: runner).
QUICK = 'test_quick.py'
#: The robot's drives' pass in one C call beside its python, lying into a bed, resting limp, rising and walking
#: (tools.cores.tick).
TICK = 'test_tick.py'
#: The compiler vectorises the robot's cores as its table says, a target at a time: no loop is lost
#: (tools.cores.vectorized).
VECTORIZED = 'test_vectorized.py'
#: The robot's processes placed by its host's topology, on hosts of other shapes and live on this one
#: (machine.process.placed).
PLACED = 'test_placed.py'
#: A new machine's needs written where setup reads them: modules, requirements, programs (tools.dev.depends).
DEPENDS = 'test_depends.py'
#: What CI left red leads the suites that run first: a log tail read, FAILED_FIRST laid (tools.dev.ci).
CI = 'test_ci.py'
#: The gate as GitHub runs it: its one workflow's jobs and their inputs kept, the runner's image here, the robot's C
#: for its target, its suites in parts of equal work (tools.dev.linux, tools.dev.workflow, machine_detector.parts, ci's reference).
WORKFLOW = 'test_workflow.py'
#: Which suites a change reaches: the path map's rows, the tiers a budget buys (tools.dev.scope, touches).
SMART = 'test_smart.py'
#: The robot's law's hand ports in C beside their python at every pass of its walk (tools.cores.surety, touch, twist, wait,
#: brace).
PORTED = 'test_ported.py'
#: What bears the robot off its soles in C beside its python, lying into a bed and walking
#: (tools.cores.borne).
BORNE = 'test_borne.py'
#: The robot's map's look in C beside its numpy twin, drawn looks and the robot's lodge's
#: (tools.cores.registry).
REGISTRY = 'test_registry.py'
#: What the robot's eyes take a thing for in C beside its numpy twin, drawn looks and the robot's lodge's
#: (tools.cores.classes).
CLASSES = 'test_classes.py'
#: Orthokinematics on the robot: its latent loop, its walk on it, the page's gym.
ORTHO = 'test_ortho.py'
#: The robot's posture as what bears it holds it: the seat's chain, the blend, the kernel's atlas, its ask
#: sat.
POSTURES = 'test_postures.py'
#: The robot's limbs on all fours: the kernel's multimodal texts read, the torus clock, the fifth limb, the levelled stance,
#: the truss.
FOURS = 'test_fours.py'
#: The robot's body on the floor: the kernel's groundwork texts read, the roll's ring and girdles, its roll and its trough, the ground
#: postures on the robot's figure.
GROUNDWORK = 'test_groundwork.py'
#: What the robot stands on when it gives: the kernel's deformable texts read, the work loop's regimes, the robot's patches' pressures,
#: the tamp, a bed.
SUBSTRATE = 'test_substrate.py'
#: The robot's gaits off its feet: the kernel's terrestrial texts read, its hinges sideways, a sled's anchors, the origins, the
#: ratchet.
CRAWL = 'test_crawl.py'
#: At a seat's or a bed's edge: the kernel's bedside texts read, what meets a fixture behind the robot, a stool, the bed's pendulum, the
#: robot's roll on it.
BEDSIDE = 'test_bedside.py'
#: Onto and off a stool over the robot's hips: the kernel's overheight texts read, its dip hung from its fists, the stool, its
#: way down.
VAULT = 'test_vault.py'
#: Walk to run, a brake, a jump and a leap: the kernel's ballistic texts read, their regimes on the robot's leg, its launches from its
#: drives.
BALLISTIC = 'test_ballistic.py'
#: The ortho-DSP suite read, voiced on the robot's walk and refused by name; a take converted to a modifier.
DSP = 'test_dsp.py'
#: The dual-domain kernel on the robot's loop: its momentum observer against its skin.
KERNEL = 'test_kernel.py'
#: The dual-domain kernel's mechanisms apart from the robot's scenes: its lowering and bank, the robot's body
#: schema and its shield, the robot's drives' back-EMF box, the phase-margin probe, the wayfinder.
SHIELD = 'test_shield.py'
#: The robot's shield on the room and its wayfinder: the room's boxes and their affordances, the
#: robot's limbs held off them, its route's flow bent, a gap's width and the twist through it.
WAYFINDER = 'test_wayfinder.py'
#: The robot's say catalog, scanned from the sources and served as JSON.
SAYS = 'test_says.py'
#: The robot's loop's latent, impedance and wrench served live to a client attached.
INTROSPECT = 'test_introspect.py'
#: The robot's streams gated to a file on a trigger, each section as its passes made it.
GATE = 'test_gate.py'
#: The autotuner on the robot's texts: its text read, the robot's texts given, every literal walked and its
#: reach on the robot, its step, a run's residuals and the verdicts on a step.
TUNE = 'test_tune.py'
#: The robot's floor: the plane under its soles, its grip as the robot feels it, the situations that
#: cap the robot's rhythm.
FLOOR = 'test_floor.py'
#: The robot's situation read from its sensors: its text's bounds and caps, each entered unannounced told
#: within a second.
SITUATION = 'test_situation.py'
#: The robot's buses under faults: a burst inside its horizon ridden, one past it the robot's drives
#: off.
BUSES = 'test_buses.py'
#: The robot's drift from its build and a load in a hand read off its rows on-line (machine.identify.wear).
WEAR = 'test_wear.py'
#: The robot's errands on its loop: a stop that settles, F heard, a scene laid anew and walked at once.
ERRANDS = 'test_errands.py'
#: The robot's docks foreseen on its page: the bed's long side and its foot, on the world's boxes and
#: on the robot's eyes.
DOCKS = 'test_docks.py'
#: The robot's motion's measures in a dimensionless frame on SO(3) (tools/sim/gvs.py).
GVS = 'test_gvs.py'
#: The robot's recorded runs replayed, each without a fall (tests/replays, tools/sim/replay.py).
REPLAYS = 'test_replays.py'
#: The robot's scenarios that hold, each placed and commanded as a player would (tools/sim/scenarios.py).
SCENES = 'test_scenarios.py'
#: The frame's core (aero/) in a vacuum: its form, its share, its impedance; a text's streams
#: compiled and flown on it.
AERO_CORE, STREAMS = 'test_aero_core.py', 'test_quad_streams.py'
#: The plane's rollouts for its predictive control (mppi/, machine.aircraft.se3mpc).
MPPI_CORE = 'test_mppi_core.py'
#: A pose's step onto weighted points (se3/, tools.cores.se3): its form, its twin, its best pose.
SE3_CORE = 'test_se3_core.py'
#: The same step under friction (se3_grip, se3_anchored): its twin, its cone, the anchors a twist leaves.
SE3_GRIP = 'test_se3_grip.py'
#: Friction at rest and sliding: MuJoCo's contact rows stuck where they hold, sliding where they yield, in C as
#: their twins (machine.world.stiction).
STICTION = 'test_stiction.py'

#: The cyclic executive (machine.rig.cyclic): its steps against machine.rig.parts, a cycle on a toy rotor.
CYCLIC = 'test_cyclic.py'

#: The quad's suites; what a change reaches. The quad's action space (machine.aircraft.pull) against the
#: board's law on the stand-in.
QUAD, QUAD_COURSE, QUAD_PAGE = 'test_quad.py', 'test_quad_course.py', 'test_views_quad.py'
QUAD_PULL, SWEEP = 'test_quad_pull.py', 'test_sweep.py'
QUADS = (QUAD, QUAD_COURSE, QUAD_PAGE, QUAD_PULL, SWEEP, 'test_views_quad_flight.py',
         'test_views_flying.py', 'test_views_inset.py', STRUCTURE)

RENDER = 'test_render.py'
GROUND = 'test_ground.py'
PAINT = 'test_paint.py'

DEFAULT_SUITES = (STRUCTURES + (CORE, SHTP, DRIVE, DRIVE_OBSERVER, FILTER, THERMAL, THERMAL_APP,
                                DAQ_CORE, BOARD_MEM,
                                BOOT_CORE, BOOT_CLIENT,
                   CTRL_CORE, WORLD_CORE, WIRE, NATIVE, NATIVE_HEAT, NATIVE_SYSID, EMULATOR,
                   SENSORLESS,
                   BROKER, DAQ_API, CONTROLLER, GYNOID, SIGHT, CAMERA, GAZE, REACH, TERMS, GVS, REPLAYS, SCENES, WBC,
                   WBC_CORE, LEAP, WASD, WRENCH, CHAIN, QP, SYSID, SYSID_LIVE, COMPILED, QUICK, TICK, STICTION, VECTORIZED, PLACED, DEPENDS, CI, WORKFLOW, SMART, PORTED, BORNE, REGISTRY, CLASSES, WEAR, ORTHO, POSTURES, FOURS, GROUNDWORK, SUBSTRATE, CRAWL, BEDSIDE, VAULT, BALLISTIC, DSP, KERNEL, SHIELD, WAYFINDER, SAYS, INTROSPECT, GATE, TUNE, FLOOR, SITUATION, BUSES, ERRANDS, DOCKS,
                   CYCLIC, QUAD, QUAD_PULL,
                   AERO_CORE, MPPI_CORE, SE3_CORE, SE3_GRIP, STREAMS, QUAD_COURSE, SWEEP, BOOT)
                  + VIEWS
                  + (RENDER, GROUND, PAINT) + OLLAMA
                  + ('test_mcp.py', 'test_simulated.py', 'test_parity.py',
                     BENCH))

CONFORMANCE = 'test_conformance.py'

LIVE = 'test_live_model.py'
#: The robot's prompt through the local model on its loop, judged under the surface.
LIVE_GYNOID = 'test_live_gynoid.py'

#: The benchmarking suites (`run_tests.ps1 -Bench`): what they time, timed at a benchmark's priority on the relay - each
#: suite's processes above normal, the host's other loads under them (`focus.relay`'s bench).
BENCHMARKS = (BENCH, PACE)

ALL_SUITES = DEFAULT_SUITES + (CONFORMANCE, LIVE, LIVE_GYNOID, PACE)

#: Where each suite joins a tier, cheapest a check first (s a check: simulated 0.003,
#: ollama 0.019, core 0.03, parity 0.13, mcp 0.14, conformance 0.29, bench 5.0, live
#: 4.6). The ollama suites are in from the first tier, narrowed by their subject budget.
JOINS = (
    (10, 'test_simulated.py'),
    (12, DAQ_API),
    (12, CONTROLLER),
    (12, GYNOID),
    (12, SIGHT),
    (12, CAMERA),
    (12, GAZE),
    (12, REACH),
    (12, TERMS),
    (12, GVS),
    (12, REPLAYS),
    (12, SCENES),
    (12, WBC),
    (12, LEAP),
    (12, WASD),
    (12, ORTHO),
    (12, POSTURES),
    (12, FOURS),
    (12, GROUNDWORK),
    (12, SUBSTRATE),
    (12, CRAWL),
    (12, BEDSIDE),
    (12, VAULT),
    (12, BALLISTIC),
    (12, DSP),
    (12, KERNEL),
    (12, SHIELD),
    (12, WAYFINDER),
    (1, SAYS),
    (2, INTROSPECT),
    (2, GATE),
    (12, TUNE),
    (12, FLOOR),
    (12, SITUATION),
    (12, BUSES),
    (12, WEAR),
    (12, ERRANDS),
    (12, DOCKS),
    (12, WRENCH),
    (12, CHAIN),
    (12, CYCLIC),
    (12, QUAD),
    (12, QUAD_COURSE),
    (12, SWEEP),
    (12, QUAD_PULL),
    (12, STREAMS),
    (12, BOOT),
    (15, CORE),
    (20, SHTP),
    # The control law against a motor model, and the commissioning against the
    # stand-in: a compiler and a few seconds, no cable.
    (20, DRIVE),
    (20, DRIVE_OBSERVER),
    # The anti-alias chain against the transfer function it was designed from,
    # and a tone fed through it: a compiler and a second.
    (20, FILTER),
    # The SOA envelope, same shape and same cost: a compiler and a second.
    (20, THERMAL),
    (20, THERMAL_APP),
    # The bootloader's core: a compiler and a second, and the one thing that
    # decides whether a blank node ever runs anything.
    (20, BOOT_CORE),
    (20, BOOT_CLIENT),
    (20, CTRL_CORE),
    (20, WORLD_CORE),
    (20, WBC_CORE),
    (20, SYSID), (20, SYSID_LIVE), (20, COMPILED), (20, QUICK), (20, TICK), (80, VECTORIZED), (80, STICTION), (20, PLACED), (20, DEPENDS), (20, CI), (20, WORKFLOW), (20, SMART), (35, PORTED), (20, BORNE), (20, REGISTRY), (20, CLASSES),
    (20, AERO_CORE),
    (20, MPPI_CORE),
    (20, SE3_CORE),
    (20, SE3_GRIP),
    (20, WIRE),
    (20, SENSORLESS),
    (35, 'test_parity.py'),
    # Renode and the image: 47 s, 420 MB.
    (40, EMULATOR),
    # The SIL/HIL engine, not a validation: a compiler and two seconds of real time.
    (40, NATIVE),
    (40, NATIVE_HEAT),
    (40, NATIVE_SYSID),
    (45, 'test_mcp.py'),
    (65, CONFORMANCE),

    # The bench guards the board's loop rates against a recorded baseline; the pace the robot's loop's, this
    # host's, a whole run's alone: its five errands one after another, 3-4 min.
    (70, BENCH),
    (100, PACE),
)

#: Where the live suite joins, and where it stops being one section. The live
#: suite is 4.6 seconds per check against conformance's 0.29 and simulated's
#: 0.003, so it is the last thing any budget buys.
LIVE_FROM = 75

LIVE_ALL_FROM = 95

#: The resolution a tier can be named at.
STEP = 5

TIERS = tuple(range(STEP, 101, STEP))


def plan_for(percent):
    """(suites, live sections) a percentage buys."""
    suites = list(STRUCTURES) + list(OLLAMA)
    suites += [name for at, name in JOINS if percent >= at]

    if percent >= LIVE_ALL_FROM:
        return tuple(suites), 'all'
    if percent >= LIVE_FROM:
        return tuple(suites), 'tools'
    return tuple(suites), None


# A cable is not a regression: every suite opens through open_session(), which
# probes and falls back to the stand-in, and says which it got. These three mean a board:
# parity compares one with the stand-in (the emulated image, in the firmware's CI), the bench
# times one; without, they skip.
NEEDS_BOARD = (CONFORMANCE, 'test_parity.py', BENCH)

#: Suites that time the link or hold the model on the card: the host to themselves - the bench
#: suite measures the link's own rates, conformance its frame gaps, the pace the robot's loop's.
ALONE = (BENCH, PACE, CONFORMANCE, LIVE)

#: Suites that hold a board to the wall's time, a page drawing beside it: alone where the host
#: has four batons or fewer - CI's runner stood native's attitude page 4.7-8.3 s beside the
#: others while it drew on; the rotor demo's switches spent 16.3 % of their
#: margin where 20 is asked, a walking suite beside it (CI).
REAL_TIME = (NATIVE, NATIVE_HEAT, NATIVE_SYSID, QUAD_PAGE, 'test_views_demo.py')

#: Suites that may reach the board's port: one at a time, beside the rest.
PORT = ('test_mcp.py', 'test_parity.py')

#: Suites that run their tests through tools.dev.focus.chosen: past a slice of the run they go
#: on as shards side by side.
SHARDED = (SENSORLESS, CONTROLLER, GYNOID, ORTHO, SYSID, SYSID_LIVE, WEAR, SITUATION, 'test_simulated.py', 'test_views_segments.py',
           DAQ_API, QUAD_PAGE, REPLAYS, SCENES, ERRANDS, WASD, STRUCTURE,
           PORTED, 'test_mirrors.py', LEAP, TICK, STICTION, VECTORIZED, FLOOR, CAMERA, SIGHT, GAZE)

#: A suite's seconds where the tree holds no record of its own (tests/.counts.json, not committed) are its work
#: in tests/work.json over the pace of the host a run is on (tools/dev/machine_detector.py): numbers written
#: here - 900 s for the keys' suite, a cut on CI's runner told beside it - were this host's guesses of another's.

#: The emulator's groups and each one's time, s, a Renode each: the rig's took 150 of the
#: suite's 211 one after another here, and CI's runner ran the Release image's past 240.
#: The blank node's, its image streamed at the emulator's pace, 157 to past 480
#: on CI's runner against 133 here; the bus's echoes through a limb of three at 475 MIPS
#: 129 here at 56 wall s a board s, past 480 on CI's runner with the off-rate open (63)
#: in the same group.
EMULATOR_GROUPS = {'board': 480, 'blank': 900, 'bus': 900, 'rate': 480, 'conformance': 480,
                   'fallback': 480}
#: What a suite asks the relay with, a line of its own beside its tests: `RENODE = {word: machines}`.
ASKED_RE = re.compile(r'^RENODE = (\{.*\})$', re.M)


def asked(path):
    """{word: machines} the suite at `path` asks the relay for, its RENODE line read, not run: the tests a word
    of its command line picks start a Renode of so many machines, a CPU thread each, whose cores the relay
    sends along with their job (`focus.run`) - let in when they fit the machine's width, so a job never waits
    for an emulator's cores while it holds its own - and whose suite runs its rest without them (`--but=word`,
    `focus.chosen`); {} where it asks none."""
    try:
        with open(path, encoding='utf-8') as f:
            said = ASKED_RE.search(f.read())
    except OSError:
        return {}
    return ast.literal_eval(said.group(1)) if said else {}

#: What a change to a page the world's heat is judged on reaches.
HEATED = (STRUCTURE, *VIEWS, NATIVE_HEAT, 'test_simulated.py')

# Every this many commits, run the lot regardless of what changed.
FULL_EVERY = 10

#: Suites the map may settle alone: no board, no ollama - about 40 s all
#: six together, so asking the model costs a 7.6 GB load longer than the
#: run. Where the map has an explicit rule it is also the better answer,
#: written by someone reading the imports.
CHEAP = frozenset({*STRUCTURES, CORE, SHTP, DRIVE, DRIVE_OBSERVER, SENSORLESS,
                   'test_simulated.py', *VIEWS, RENDER, GROUND, PAINT})
