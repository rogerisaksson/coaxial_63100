"""The suites: their names, which join a run, the tiers, what each part of the tree reaches."""
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]           # host/

# Structure first: it answers "does host/ still hold together" in a fifth of a
# second, and every behavioural suite below it assumes the answer is yes.
STRUCTURE = 'test_structure.py'

CORE = 'test_modbus_core.py'

SHTP = 'test_shtp_core.py'

DRIVE = 'test_drive_core.py'

FILTER = 'test_filter_core.py'

#: The thermal envelope as the C that will run on the board. The
#: network had `check.c` - the calibration campaign's own report -
#: and the SOA arithmetic that gates a real stage had nothing at all,
#: only a tested Python mirror. That was the wrong way round.
THERMAL = 'test_thermal_core.py'

#: The acquisition engine as the C that will run on the board - the
#: ring, the window, the ladder, the tone, the live accumulator -
#: hardware-free since 2026-09-14 and, until then, tested only by the
#: bench after a flash.
DAQ_CORE = 'test_daq_core.py'

#: The bootloader's state machine as the C that will run - the chunk
#: stream, the bitmap, the seal - on a RAM flash, before any register
#: is touched (docs/BOOT.md).
BOOT_CORE = 'test_boot_core.py'

#: machine/'s parts and feedback as the C a board will loop on, stepped beside
#: host/machine/parts.py.
CTRL_CORE = 'test_ctrl_core.py'

#: The device clients through the firmware's own wire - comms/ built for this host over a
#: fake board (tools.cores.fakeboard): a disagreement between the two sides fails here.
WIRE = 'test_wire.py'

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

#: test_ollama.py was 5,496 lines and 733 checks - a third of every check
#: this tree has, in one file, and the reason a tier could not be asked for at
#: any useful resolution. One file per subject now: the largest is 218 checks
#: and the smallest 12, so a budget can actually choose.
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

VIEWS = 'test_views.py'

#: The composed controller and its parts, against a toy rotor and the stand-in.
CONTROLLER = 'test_controller.py'

RENDER = 'test_render.py'

DEFAULT_SUITES = ((STRUCTURE, CORE, SHTP, DRIVE, FILTER, THERMAL, DAQ_CORE, BOOT_CORE,
                   CTRL_CORE, WIRE, EMULATOR,
                   SENSORLESS,
                   BROKER, DAQ_API, CONTROLLER, BOOT, VIEWS,
                   RENDER) + OLLAMA
                  + ('test_mcp.py', 'test_simulated.py', 'test_parity.py',
                     BENCH))

CONFORMANCE = 'test_conformance.py'

LIVE = 'test_live_model.py'

ALL_SUITES = DEFAULT_SUITES + (CONFORMANCE, LIVE)

#: Where each suite joins a tier. Cheapest per check first, so a tier buys
#: the most checks for the least wall time. Measured, seconds per check:
#: simulated 0.003, ollama 0.019, core 0.03, parity 0.13, mcp 0.14,
#: conformance 0.29, bench 5.0, live 4.6.
#:
#: The ollama suites are not here: they are in from the first tier and narrow
#: THEMSELVES through their own subject budget, which is where the fine
#: resolution lives.
JOINS = (
    (10, 'test_simulated.py'),
    (12, DAQ_API),
    (12, CONTROLLER),
    (12, BOOT),
    (15, CORE),
    (20, SHTP),
    # The control law against a motor model, and the commissioning against the
    # stand-in: a compiler and a few seconds, no cable.
    (20, DRIVE),
    # The anti-alias chain against the transfer function it was designed from,
    # and a tone fed through it: a compiler and a second.
    (20, FILTER),
    # The SOA envelope, same shape and same cost: a compiler and a second.
    (20, THERMAL),
    # The bootloader's core: a compiler and a second, and the one thing that
    # decides whether a blank node ever runs anything.
    (20, BOOT_CORE),
    (20, CTRL_CORE),
    (20, WIRE),
    (20, SENSORLESS),
    (35, 'test_parity.py'),
    # Renode and the image: 47 s, 420 MB (2026-09-25).
    (40, EMULATOR),
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
    suites = [STRUCTURE] + list(OLLAMA)
    suites += [name for at, name in JOINS if percent >= at]

    if percent >= LIVE_ALL_FROM:
        return tuple(suites), 'all'
    if percent >= LIVE_FROM:
        return tuple(suites), 'tools'
    return tuple(suites), None


# A cable is not a regression: every suite opens through open_session(), which
# probes and falls back to the stand-in, and says which it got.
NEEDS_BOARD = (CONFORMANCE,)

#: Suites that may reach the board's port, or hold the model on the card.
#: Each wants the host to itself - the bench suite measures the link's
#: own rates, conformance its frame gaps - so they run one at a time after
#: the rest. Every other suite opens the stand-in or nothing at all.
ALONE = ('test_mcp.py', 'test_parity.py', BENCH, CONFORMANCE, LIVE, EMULATOR)

# What a change to each part of the tree can plausibly have broken.
TOUCHES = (
    ('host/coaxial_ollama/debug.py',           OLLAMA + ('live:all',)),
    ('host/coaxial_ollama/replies.py',         OLLAMA + ('live:tools',)),
    ('host/coaxial_ollama/language.py',        OLLAMA + ('live:language',)),
    ('host/coaxial_ollama/',                   OLLAMA),
    ('host/coaxial_mcp/tools.py',              ('test_mcp.py', 'test_parity.py',
                                                'live:tools') + OLLAMA),
    ('host/coaxial_mcp/render.py',             ('test_mcp.py', 'test_parity.py')
                                               + OLLAMA),
    ('host/coaxial_mcp/',                      ('test_mcp.py', 'test_parity.py')),
    # The broker is the port itself: every session goes through it when one is
    # up, so its own suite runs whenever it or the two files that reach for it
    # change.
    ('host/coaxial/comm/broker.py',            (BROKER, DAQ_API,
                                                'test_parity.py')),
    ('host/coaxial/comm/ports.py',             (BROKER, 'test_mcp.py',
                                                'test_ollama_link.py')),
    ('host/coaxial/rig.py',                    (DAQ_API, 'test_simulated.py',
                                                VIEWS)),
    ('host/coaxial/acquire/record.py',         (DAQ_API,)),
    ('host/coaxial/acquire/fanout.py',         (DAQ_API, BROKER)),
    ('host/coaxial/acquire/reader.py',         (DAQ_API,)),
    ('host/coaxial/devices/calibration.py',    (DAQ_API, 'test_simulated.py')),
    ('host/coaxial/devices/board.py',          (BROKER, 'test_simulated.py',
                                                'test_parity.py', 'test_mcp.py')),
    ('host/coaxial/comm/session.py',           (BROKER, 'test_mcp.py',
                                                'test_parity.py')),
    ('host/tools/target/session.py',           (BROKER,)),
    # The pure character renderers: a reading in, text out.
    ('host/coaxial/draw/orientation.py',       ('test_simulated.py', 'test_mcp.py',
                                                RENDER)),
    ('host/coaxial/graphics/engine.py',        (RENDER, VIEWS)),
    ('host/coaxial/graphics/wireframe.py',     (RENDER, VIEWS)),
    ('host/coaxial/draw/ascii3d.py',           ('test_simulated.py',)),
    ('host/coaxial/draw/desk.py',              ('test_simulated.py',)),
    ('host/coaxial/draw/dial.py',              ('test_simulated.py',)),
    ('host/coaxial/graphics/mesh.py',          ('test_simulated.py',)),
    ('host/machine/ansi.py',                   ('test_simulated.py', CONTROLLER)),
    ('host/machine/parts.py',                  (CONTROLLER, CTRL_CORE)),
    ('host/machine/',                          (CONTROLLER, 'test_simulated.py', 'test_mcp.py')),
    ('host/coaxial/node.py',                   (CONTROLLER, 'test_mcp.py')),
    ('host/coaxial/devices/ctrl.py',           (CONTROLLER, STRUCTURE)),
    ('host/coaxial/simulated/ctrl.py',         (CONTROLLER, STRUCTURE)),
    ('host/terminal/views/show_session.py',    (VIEWS,) + OLLAMA),
    ('host/terminal/views/session/',           (VIEWS,) + OLLAMA),
    # A live view is a loop, a screen and a cable around a renderer that is
    # tested on its own.
    ('host/terminal/views/',                   (STRUCTURE, VIEWS,
                                                'test_simulated.py')),
    ('host/terminal/ui/',                      (STRUCTURE, VIEWS,
                                                'test_simulated.py')),
    ('host/tools/cores/build.py',              (CORE, SHTP, DRIVE, FILTER, THERMAL, DAQ_CORE,
                                                BOOT_CORE, CTRL_CORE)),
    ('host/tools/cores/drive.py',              (STRUCTURE, DRIVE, SENSORLESS)),
    ('host/tools/cores/thermal.py',            (THERMAL,)),
    ('host/tools/dev/counts.py',               ('test_ollama_runner.py',)),
    ('host/tests/',                            ()),          # decided by name below
    # Firmware and protocol: the byte-level master is the point of it - but the
    # portable core is also compiled and run on this host, which is the only
    # check on it that does not need a cable.
    ('modbus/',                                (CORE, CONFORMANCE, 'test_mcp.py')),
    # The SHTP layer is hardware-free like the Modbus core, so the host build
    # is what covers it.
    ('shtp/',                                  (SHTP,)),
    # The decimating filter is hardware-free the same way, and its design lives
    # on the host beside it.
    ('filter/',                                (FILTER,)),
    ('ctrl/',                                  (CTRL_CORE,)),
    ('host/coaxial/acquire/bessel.py',         (FILTER, STRUCTURE)),
    # The control law is hardware-free like the SHTP layer, and its suite
    # closes the loop through a motor model - the only check on it that needs
    # no motor.
    ('drive/',                                 (DRIVE,)),
    ('host/coaxial/devices/drive.py',          (SENSORLESS, 'test_simulated.py',
                                                'test_parity.py')),
    ('host/coaxial/model/sensorless.py',       (SENSORLESS,)),
    ('host/motor/',                            (SENSORLESS, DRIVE, CONTROLLER)),
    ('host/coaxial/control/commission.py',     (SENSORLESS,)),
    ('host/tools/bench/commission.py',         (STRUCTURE, SENSORLESS)),
    # The stage constants and the host control loops are design arithmetic with
    # closed-form checks; the Monte Carlo drives the compiled law.
    ('host/coaxial/model/inverter.py',         (SENSORLESS,)),
    ('host/coaxial/model/blocks.py',           (SENSORLESS, DRIVE)),
    ('host/coaxial/control/motion.py',         (SENSORLESS, 'test_simulated.py')),
    ('host/tools/sim/montecarlo.py',           (STRUCTURE, DRIVE)),
    # BENCH: firmware in the main loop is what slows the board (the thermal
    # observer's per-poll ADC and SPI reads; a poll that lost a Modbus byte).
    ('comms/',                                 (WIRE, EMULATOR, CONFORMANCE, 'test_mcp.py',
                                                BENCH)),
    ('board/',                                 (WIRE, EMULATOR, CONFORMANCE, 'test_mcp.py',
                                                'test_parity.py', BENCH)),
    ('core/',                                  (EMULATOR, CONFORMANCE, BENCH)),
    ('host/tools/emu/',                        (EMULATOR,)),
    # The observer and its envelope are hardware-free like the filter, so the
    # host build is what covers them; the board glue that acts on the budget
    # lives in board/ and is the bench's.
    ('thermal/',                               (THERMAL, CONFORMANCE, BENCH)),
    # The acquisition engine is hardware-free like the observer, so the host
    # build covers it; the glue that reads the converter is board_daq.c and the
    # bench's, and the record's bytes cross the wire.
    ('boot/',                                  (BOOT_CORE, STRUCTURE)),
    ('host/coaxial/devices/boot.py',           (BOOT, STRUCTURE)),
    ('host/coaxial/simulated/boot.py',         (BOOT, STRUCTURE)),
    ('host/tools/target/flash_nodes.py',       (BOOT, STRUCTURE)),
    ('daq/',                                   (DAQ_CORE, CONFORMANCE, 'test_parity.py',
                                                BENCH)),
    ('host/coaxial/model/thermal.py',          (THERMAL, 'test_sensorless.py',
                                                STRUCTURE)),
    # A NOTEBOOK EXAMPLE reaches the library and nothing else reaches it.
    ('notebook_examples/',                     (STRUCTURE,)),
    # And the file the notebooks are written FROM.
    ('host/tools/notebooks/make_notebooks.py', (STRUCTURE,)),
    # A document can only break the docs index and the phrase table.
    ('docs/',                                  ('test_ollama_runner.py',)),
    ('CLAUDE.md',                              ('test_ollama_runner.py',)),
    ('README.md',                              ('test_ollama_runner.py',)),
    # PowerShell around the Python.
    ('terminal/',                              (STRUCTURE,)),
    ('coaxial_tty.ps1',                        (STRUCTURE,)),
    ('env.ps1',                                (STRUCTURE,)),
    ('host/run_tests.ps1',                     (STRUCTURE,)),
    ('setup.ps1',                              (STRUCTURE,)),
    # Neither the CAD export nor the schematic is read by a suite.
    ('render/',                                ()),
    ('electronics/',                           ()),
    ('datasheets/',                            ()),
    ('.gitignore',                             ()),
    ('.vscode/',                               ()),
    # A path takes the first row that prefixes it (scope.pick): the rows that
    # hold others come last.
    ('host/coaxial/simulated',                 ('test_simulated.py',
                                                'test_parity.py') + OLLAMA),
    ('host/coaxial/control/',                  (CONTROLLER, SENSORLESS, 'test_simulated.py')),
    ('host/coaxial/',                          ('test_simulated.py', 'test_parity.py',
                                                'test_mcp.py')),
    ('host/tools/',                            OLLAMA),
)

# Every this many commits, run the lot regardless of what changed.
FULL_EVERY = 10

#: Suites the map may settle alone: no board, no ollama - about 40 s all
#: six together, so asking the model costs a 7.6 GB load longer than the
#: run. Where the map has an explicit rule it is also the better answer,
#: written by someone reading the imports.
CHEAP = frozenset({STRUCTURE, CORE, SHTP, DRIVE, SENSORLESS,
                   'test_simulated.py', VIEWS, RENDER})
