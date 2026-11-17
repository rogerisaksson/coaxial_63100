#!/usr/bin/env python3
"""Which suites a change reaches (`tools.dev.scope`): the path map's rows, what settles without the model, the
tiers a budget buys and what the quarter and the three quarters hold."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_smart_selection(report):
    """Which suites a change can have broken."""
    from tools.dev import scope
    from tools.dev import suites as table

    for paths, expect_suites, expect_live, why in (
            (['host/coaxial_ollama/language.py'],
             set(table.OLLAMA), {'language'},
             'the language lock is half the live suite, not all of it'),
            (['host/coaxial_ollama/replies.py'],
             set(table.OLLAMA), {'tools'},
             'what an answer means is the tool-choice half'),
            (['host/coaxial_mcp/render.py'],
             {'test_mcp.py', 'test_parity.py'} | set(table.OLLAMA), set(),
             'a renderer cannot change which tool gets called'),
            (['docs/HARDWARE.md'], {'test_ollama_runner.py'}, set(),
             'a document can only break the docs index'),
            (['modbus/src/modbus_rtu.c'],
             {'test_modbus_core.py', 'test_conformance.py', 'test_mcp.py'},
             set(),
             'the byte-level master for the wire, and the host build for '
             'the framing itself - which needs no cable'),
            (['shtp/src/shtp.c'], {'test_shtp_core.py'}, set(),
             'the SHTP layer is built and run here, and touches no Modbus '
             'wire'),
            (['host/tests/test_parity.py'], {'test_parity.py'}, set(),
             'editing a suite is a reason to run it'),
            (['host/coaxial_ollama/debug.py'],
             set(table.OLLAMA), {'all'},
             'the prompt and the tool sets are both in there'),
    ):
        suites, live, _ = scope.pick(paths)
        report.check('%s -> %s' % (paths[0].rsplit('/', 1)[-1],
                                   ', '.join(sorted(suites)) or 'none'),
                     suites == expect_suites, ', '.join(sorted(suites)))
        report.check('   ...and live: %s (%s)'
                     % (', '.join(sorted(expect_live)) or 'not at all', why),
                     live == expect_live, ', '.join(sorted(live)) or 'none')

    # The first row that prefixes a path takes it.
    from tools.dev.touches import TOUCHES
    shadowed = [path for at, (path, _) in enumerate(TOUCHES)
                if any(path.startswith(above) for above, _ in TOUCHES[:at])]
    report.check('every row of the path map can match: none sits under a '
                 'shorter prefix above it', not shadowed, ', '.join(shadowed))

    # A path the map does not know is the case to fail safe on: run everything
    # rather than quietly cover nothing.
    suites, live, why = scope.pick(['nothing/the/map/knows.xyz'])
    report.check('an unmapped path runs everything, and says why',
                 table.CONFORMANCE in suites and live == {'all'}
                 and 'unmapped' in why[-1], why[-1][:52])

    # And the other half of that: an entry deliberately mapped to nothing is a
    # rule, not a hole, and must not fall through to the whole gate.
    suites, live, why = scope.pick(['datasheets/imu/UserGuide.pdf'])
    report.check('a path nothing reads runs nothing, and says that instead',
                 not suites and not live and 'nothing reads it' in why[-1],
                 why[-1][:52])

    report.check('and the whole lot goes every tenth commit',
                 table.FULL_EVERY == 10)

    # The map settles the cheap cases on its own.
    for paths, what in (
        (['terminal/adc.ps1'], 'a demo wrapper'),
        (['system_terminal.ps1'], 'the demo picker'),
        (['host/coaxial/draw/desk.py'], 'a pure renderer'),
        (['host/terminal/views/show_desk.py'], 'a live view'),
        (['datasheets/imu/UserGuide.pdf'], 'something no suite reads'),
    ):
        suites, _, why = scope.pick(paths)
        report.check('%s does not need the model' % what,
                     scope.settled(suites, why),
                     ', '.join(sorted(suites)) or 'nothing to run')

    for paths, what in (
        (['host/coaxial/devices/board.py'], 'the library'),
        (['board/src/board_cal.c'], 'firmware'),
        (['nothing/the/map/knows.xyz'], 'an unmapped path'),
    ):
        suites, _, why = scope.pick(paths)
        report.check('%s still goes to the model' % what,
                     not scope.settled(suites, why),
                     ', '.join(sorted(suites)))

    report.check('and what it settles on runs without a board or a model',
                 table.CONFORMANCE not in table.CHEAP
                 and table.LIVE not in table.CHEAP
                 and not (set(table.OLLAMA) & table.CHEAP),
                 ', '.join(sorted(table.CHEAP)))

    # A tier is a budget and the model spends inside it.
    class _Args:
        def __init__(self, coverage, files):
            self.coverage, self.file, self.live = coverage, list(files), True

    args = _Args(25, [table.STRUCTURE, 'test_simulated.py',
                      table.CONFORMANCE])
    left = scope._within_tier(args, 'all')
    report.check('the 25% tier drops what the model added past it',
                 table.CONFORMANCE not in args.file and left == ''
                 and not args.live, ', '.join(args.file))
    report.check('   ...and keeps what was inside it',
                 args.file == [table.STRUCTURE, 'test_simulated.py'],
                 ', '.join(args.file))

    # 75 % is the tier that does buy conformance and the live tool matrix.
    args = _Args(75, [table.STRUCTURE, table.CONFORMANCE])
    left = scope._within_tier(args, 'tools')
    report.check('a tier that does stretch to conformance keeps it',
                 table.CONFORMANCE in args.file and left == 'tools',
                 '%s live:%s' % (', '.join(args.file), left))

    args = _Args(None, [table.CONFORMANCE])
    report.check('and with no tier at all nothing is clamped',
                 scope._within_tier(args, 'all') == 'all'
                 and args.file == [table.CONFORMANCE])

    # A tier is arithmetic, not a table of three, so any 5 % step can be
    # named.
    grew = []
    for percent in table.TIERS:
        suites, sections = table.plan_for(percent)
        grew.append((percent, set(suites), sections))

    report.check('every 5 %% step from %d to 100 is a tier'
                 % table.STEP, len(table.TIERS) == 20
                 and table.TIERS[0] == 5 and table.TIERS[-1] == 100,
                 '%d tiers' % len(table.TIERS))
    report.check('structure is in every one of them - it is the precondition '
                 'for reading any other result',
                 all(table.STRUCTURE in s for _, s, _ in grew))
    report.check('a deeper tier never runs less than a shallower one',
                 all(grew[i][1] <= grew[i + 1][1]
                     for i in range(len(grew) - 1)),
                 ' -> '.join('%d:%d' % (p, len(s)) for p, s, _ in grew[:6]))
    report.check('the live suite is the last thing any budget buys',
                 all(sections is None for p, _, sections in grew
                     if p < table.LIVE_FROM),
                 'joins at %d %%' % table.LIVE_FROM)

    quarter = {*table.STRUCTURES, table.CORE, table.SHTP, 'test_simulated.py', table.DRIVE, table.DRIVE_OBSERVER, table.FILTER, table.THERMAL, table.THERMAL_APP, table.BOOT_CORE,
               table.BOOT_CLIENT, table.CTRL_CORE, table.WORLD_CORE, table.WIRE, table.SENSORLESS, table.DAQ_API, table.CONTROLLER, table.GYNOID,
               table.SIGHT, table.CAMERA, table.GAZE, table.REACH, table.TERMS, table.GVS, table.REPLAYS, table.SCENES, table.WBC, table.WBC_CORE, table.CHAIN, table.LEAP, table.WASD,
               table.ORTHO, table.POSTURES, table.FOURS, table.GROUNDWORK, table.SUBSTRATE, table.CRAWL, table.BEDSIDE, table.VAULT, table.BALLISTIC, table.DSP, table.KERNEL, table.SHIELD, table.WAYFINDER, table.SAYS, table.INTROSPECT,
               table.GATE, table.TUNE, table.FLOOR, table.SITUATION, table.BUSES, table.WRENCH, table.SYSID, table.SYSID_LIVE, table.WEAR, table.ERRANDS, table.DOCKS, table.CYCLIC, table.QUAD, table.QUAD_PULL,
               table.QUAD_COURSE, table.SWEEP, table.COMPILED, table.QUICK, table.TICK, table.LIBM, table.PLACED, table.DEPENDS, table.CI, table.WORKFLOW, table.BORNE, table.REGISTRY, table.CLASSES, table.AERO_CORE, table.MPPI_CORE, table.SE3_CORE,
               table.SE3_GRIP, table.SMART, table.STREAMS, table.BOOT} | set(table.OLLAMA)
    for percent, expect in ((25, quarter),
                            (75, quarter | {'test_parity.py', 'test_mcp.py', table.EMULATOR, table.PORTED,
                                            table.NATIVE, table.NATIVE_HEAT, table.NATIVE_SYSID, table.CONFORMANCE, table.BENCH})):
        suites, _ = table.plan_for(percent)
        report.check('%d %% is what it always was' % percent,
                     set(suites) == expect,
                     ', '.join(sorted(set(suites) ^ expect)) or 'unchanged')


ROSTER = (test_smart_selection,)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
