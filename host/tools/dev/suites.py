"""The suites: their names, which join a run, the tiers (the path map: `touches`)."""
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]           # host/

STRUCTURE = 'test_structure.py'

#: The structure family, first in every run: host/ holding together, the tree's shape, the
#: firmware and the host agreeing - one 17 k-token file before.
STRUCTURES = (STRUCTURE, 'test_layout.py', 'test_mirrors.py', 'test_wire_shapes.py',
              'test_wire_requests.py', 'test_protocol_doc.py')

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
#: host/machine/parts.py.
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

BROKER = 'test_broker.py'

#: The acquisition front door against the stand-in - naming, reading,
#: the record's shape, the buffers. No board and no compiler, so it is
#: one of the cheapest suites here and joins first.
DAQ_API = 'test_daq_api.py'

#: The master's side of the bootloader against the stand-in's blank
#: node - no board, no compiler, a second (docs/BOOT.md).
BOOT = 'test_boot.py'

#: Every live view against the stand-in, by subject: a file a subject, each under the token
#: budget (tools/dev/token_budget.py); one 32 k-token file before.
VIEWS = ('test_views_terminal.py', 'test_views_front.py', 'test_views_rotor.py',
         'test_views_drawing.py', 'test_views_gauges.py', 'test_views_thermal.py',
         'test_views_demo.py', 'test_views_segments.py',
         'test_views_quad.py', 'test_views_quad_flight.py', 'test_helicopter.py',
         'test_views_humanoid.py', 'test_views_inset.py')

#: The composed controller and its parts, against a toy rotor and the stand-in.
CONTROLLER = 'test_controller.py'

#: The gynoid's body on fantasy boards, their SOA never binding: her clothes, her look, her
#: skeleton, her drives' mounts and gearboxes; the whole-body stack; her eyes.
GYNOID, WBC, SIGHT = 'test_gynoid.py', 'test_wbc.py', 'test_sight.py'
REACH, TERMS = 'test_reach.py', 'test_terms.py'
#: The loop in C against MuJoCo and the python stack (wbc/); flight on both; `machine.wrench`.
WBC_CORE, FLIGHT, WRENCH = 'test_wbc_core.py', 'test_flight.py', 'test_wrench.py'
#: Orthokinematics: a text compiled, her latent loop, her walk on it, the page's gym.
ORTHO = 'test_ortho.py'
#: The frame's core (aero/) in a vacuum: its form, its share, its impedance; a text's streams
#: compiled and flown on it.
AERO_CORE, STREAMS = 'test_aero_core.py', 'test_quad_streams.py'

#: The cyclic executive (machine.cyclic): its steps against machine.parts, a cycle on a toy rotor.
CYCLIC = 'test_cyclic.py'

#: The quad's suites; what a change reaches. Its action space (machine.pull) against the
#: board's law on the stand-in.
QUAD, QUAD_COURSE, QUAD_PAGE = 'test_quad.py', 'test_quad_course.py', 'test_views_quad.py'
QUAD_PULL = 'test_quad_pull.py'
QUADS = (QUAD, QUAD_COURSE, QUAD_PAGE, QUAD_PULL, 'test_views_quad_flight.py',
         'test_views_inset.py', STRUCTURE)

RENDER = 'test_render.py'

DEFAULT_SUITES = (STRUCTURES + (CORE, SHTP, DRIVE, DRIVE_OBSERVER, FILTER, THERMAL, THERMAL_APP,
                                DAQ_CORE, BOARD_MEM,
                                BOOT_CORE, BOOT_CLIENT,
                   CTRL_CORE, WORLD_CORE, WIRE, NATIVE, NATIVE_HEAT, EMULATOR,
                   SENSORLESS,
                   BROKER, DAQ_API, CONTROLLER, GYNOID, SIGHT, REACH, TERMS, WBC,
                   WBC_CORE, FLIGHT, WRENCH, ORTHO, CYCLIC, QUAD, QUAD_PULL,
                   AERO_CORE, STREAMS, QUAD_COURSE, BOOT)
                  + VIEWS
                  + (RENDER,) + OLLAMA
                  + ('test_mcp.py', 'test_simulated.py', 'test_parity.py',
                     BENCH))

CONFORMANCE = 'test_conformance.py'

LIVE = 'test_live_model.py'

ALL_SUITES = DEFAULT_SUITES + (CONFORMANCE, LIVE)

#: Where each suite joins a tier, cheapest a check first (s a check: simulated 0.003,
#: ollama 0.019, core 0.03, parity 0.13, mcp 0.14, conformance 0.29, bench 5.0, live
#: 4.6). The ollama suites are in from the first tier, narrowed by their subject budget.
JOINS = (
    (10, 'test_simulated.py'),
    (12, DAQ_API),
    (12, CONTROLLER),
    (12, GYNOID),
    (12, SIGHT),
    (12, REACH),
    (12, TERMS),
    (12, WBC),
    (12, FLIGHT),
    (12, ORTHO),
    (12, WRENCH),
    (12, CYCLIC),
    (12, QUAD),
    (12, QUAD_COURSE),
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
    (20, AERO_CORE),
    (20, WIRE),
    (20, SENSORLESS),
    (35, 'test_parity.py'),
    # Renode and the image: 47 s, 420 MB.
    (40, EMULATOR),
    # The SIL/HIL engine, not a validation: a compiler and two seconds of real time.
    (40, NATIVE),
    (40, NATIVE_HEAT),
    (45, 'test_mcp.py'),
    (65, CONFORMANCE),

    # The bench guards the board's loop rates against a recorded baseline.
    (70, BENCH),
)

#: Where the live suite joins, and where it stops being one section. It is
#: 4.6 seconds per check against conformance's 0.29 and simulated's 0.003,
#: so it is the last thing any budget buys.
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
# probes and falls back to the stand-in, and says which it got.
NEEDS_BOARD = (CONFORMANCE,)

#: Suites that time the link or hold the model on the card: the host to themselves - the bench
#: suite measures the link's own rates, conformance its frame gaps.
ALONE = (BENCH, CONFORMANCE, LIVE)

#: Suites that hold a board to the wall's time, a page drawing beside it: alone where the host
#: has four batons or fewer - CI's runner stood native's attitude page 4.7-8.3 s beside the
#: others while it drew on; the rotor demo's switches spent 16.3 % of their
#: margin where 20 is asked, a walking suite beside it (CI).
REAL_TIME = (NATIVE, NATIVE_HEAT, QUAD_PAGE, 'test_views_demo.py')

#: Suites that may reach the board's port: one at a time, beside the rest.
PORT = ('test_mcp.py', 'test_parity.py')

#: Suites that run their tests through tools.dev.focus.chosen: past a slice of the run they go
#: on as shards side by side.
SHARDED = (SENSORLESS, CONTROLLER, GYNOID, ORTHO, 'test_simulated.py', 'test_views_segments.py', DAQ_API,
           QUAD_PAGE)

#: A sharded suite's seconds where none are recorded (tests/.counts.json, not committed):
#: the loop's suite 120 s here whole, a walk a shard; the quad's page 211 s, a flight a
#: shard.
FRESH_S = {ORTHO: 300.0, QUAD_PAGE: 250.0}

#: The emulator's groups and each one's time, s, a Renode each: the rig's took 150 of the
#: suite's 211 one after another here, and CI's runner ran the Release image's past 240.
#: The blank node's, its image streamed at the emulator's pace, 157 to past 480
#: on CI's runner against 133 here.
EMULATOR_GROUPS = {'board': 480, 'blank': 900, 'conformance': 480, 'bus': 480,
                   'fallback': 480}

#: What a change to a page the world's heat is judged on reaches.
HEATED = (STRUCTURE, *VIEWS, NATIVE_HEAT, 'test_simulated.py')

# Every this many commits, run the lot regardless of what changed.
FULL_EVERY = 10

#: Suites the map may settle alone: no board, no ollama - about 40 s all
#: six together, so asking the model costs a 7.6 GB load longer than the
#: run. Where the map has an explicit rule it is also the better answer,
#: written by someone reading the imports.
CHEAP = frozenset({*STRUCTURES, CORE, SHTP, DRIVE, DRIVE_OBSERVER, SENSORLESS,
                   'test_simulated.py', *VIEWS, RENDER})
