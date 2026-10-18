#!/usr/bin/env python3
"""Her runs recorded and replayed: R's rows and the commands beside them; each kept replay
(`tests/replays`, `tools/sim/replay.py`) every command at the pass it came at, none ending in a
fall. A fall recorded on the page is kept in `tests/replays/falls` till it stands."""
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

REPLAYS = sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'replays', '*.csv')))


def test_r_records_and_saves(report):
    """R starts a recording, every state becomes a row, R again writes the CSV under RECORDINGS;
    the epoch's commands so far ride the first row, those heard after the row after them."""
    import json
    import tempfile
    from machine.figure import JOINTS
    from terminal.ui import recorder
    from terminal.views import humanoid_keys
    now = {'t': 1.25, 'stage': 'walk', 'speed': 0.5, 'phase': 0.3, 'loads': (200.0, 300.0),
           'where': (0.0, 0.9, 1.0), 'turn': (1.0, 0.0, 0.0, 0.0), 'set': {'left_knee': 20.0},
           'watts': 150.0, 'angles': {j: 0.0 for j in JOINTS}, 'epoch': 2}
    laid, walk, turn = (2, 0, 0.0, {'laid': {'every': 3}}), (2, 40, 0.04, {'walk': True}), (2, 900, 0.9, {'twist': (0.5, 0.0, 0.3)})
    was, humanoid_keys.RECORDINGS = humanoid_keys.RECORDINGS, tempfile.mkdtemp()
    try:
        st: dict = {'recording': None, 'recorded': None, 'yaw': 60.0}
        humanoid_keys.heard(st, [(1, 0, 0.0, {'laid': {}}), laid, walk])
        humanoid_keys._recorded(st)
        recorder.added(st['recording'], humanoid_keys.rows_of(st, [now, now]))
        badge = recorder.corner(st['recording'], 80)
        humanoid_keys.heard(st, [turn])
        recorder.added(st['recording'], humanoid_keys.rows_of(st, [now]))
        humanoid_keys._recorded(st)
        path = st['recorded']
        with open(path, newline='', encoding='utf-8') as f:
            rows = list(csv.reader(f))
        report.check('R, three states, R: a CSV of the header and three rows, every field filled',
                     rows[0] == humanoid_keys.HEADER and len(rows) == 4
                     and all(len(r) == len(humanoid_keys.HEADER) for r in rows[1:])
                     and st['recording'] is None and os.path.basename(path).startswith('humanoid_'))
        said = [json.loads(r[-1]) for r in rows[1:]]
        report.check("the epoch's commands from its lay on the first row, a later one on the row after it",
                     said == [humanoid_keys.plain([laid, walk]), [], humanoid_keys.plain([turn])], said)
        report.check('while it records the drawing carries REC, its wheel and its rows',
                     ''.join(chr(c) for c, _ink in badge.values()).strip() == '● REC ⠹ 2',
                     ''.join(chr(c) for c, _ink in badge.values()))
        from tools.sim import tabular
        cols = tabular.numeric_columns(path, ('t', 'watts', 'left_knee'))
        m, p2p, drift, _sd, rsd = tabular.fit_line([0.0, 1000.0, 2000.0], [10.0, 12.0, 14.0])
        report.check('tabular reads numeric columns and fits linear trends without row loops',
                     len(cols['t']) == 3 and len(tabular.read_rows(path)) == 3
                     and abs(m - 12.0) < 1e-9 and abs(p2p - 4.0) < 1e-9
                     and abs(drift - 120.0) < 1e-6 and rsd < 1e-9)
    finally:
        humanoid_keys.RECORDINGS = was


def _replay(path):
    """A test of one kept replay: replayed to its last row, standing."""
    def test(report):
        from tools.sim import replay, tabular
        rows = tabular.read_rows(path)
        for wobble in replay.WOBBLES:
            got = replay.replayed(replay.entries(rows), rows, wobble=wobble)
            report.check('%s, %d commands, to %.1f s, laid %+.1f mm aside: no fall' % (
                os.path.basename(path), len(replay.entries(rows)), got['t'], 1e3 * wobble),
                got['fell'] is None, got['fell'] and 'fell at %.3f s of epoch %d' % got['fell'][::-1])
    test.__name__ = 'test_' + os.path.splitext(os.path.basename(path))[0]
    return test


ROSTER = [test_r_records_and_saves] + [_replay(path) for path in REPLAYS]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
