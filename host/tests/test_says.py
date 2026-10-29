"""Her say catalog (`machine.says`): scanned from the sources, served as JSON, nothing registered by hand."""
import json
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_catalog_scans_her_sources(report):
    """`says.catalog()` carries every dock verb, every suite stunt, every gym entry and every take,
    each row from its source, once; the whole serves as JSON; its counts are the sources' own, so a
    say added to a source appears with no hand on the catalog."""
    import numpy as np
    from machine import dock, maneuver, says, sprung, takes
    rows = says.catalog()
    by = {}
    for r in rows:
        by.setdefault(r['kind'], set()).add(r['op'])
    ops = [r['op'] for r in rows]
    docks = set(dock.docks())
    stunts = set(maneuver.PLAYED) - docks
    names = sorted(set(k[:-2] for k in np.load(takes.FILE).files if k.endswith('_t')))
    entries = {label for label, _w, _n in sprung.entries()}
    served = json.loads(json.dumps(rows))
    report.check('every dock verb an errand row, every suite stunt a stunt row, every take a take '
                 'row, every other gym entry a gait or behavior row, each once, the loop\'s words one '
                 'row, all JSON',
                 by.get('errand') == docks and by.get('stunt') == stunts and by.get('take') == set(names)
                 and entries - docks - stunts <= by.get('gait', set()) | by.get('behavior', set())
                 and len(ops) == len(set(ops)) + 0 and by.get('word') == {'heard'}
                 and all(set(sprung.HEARD) == set(r['command']) for r in rows if r['kind'] == 'word')
                 and len(served) == len(rows),
                 '%d rows: %s' % (len(rows), {k: len(v) for k, v in sorted(by.items())}))


def test_a_chosen_row_is_what_hears_it(report):
    """`gynoid.plan` on a stub of the model: every catalog row but the loop's words a choice; the
    chosen row's own command sent - a say, a gait's twist, a take's rows as the bench says them, the
    tour the page's -, null nothing."""
    from coaxial_ollama import client, gynoid
    from machine import says, se2
    rows = says.catalog()
    answers = []

    class Stub:
        def __init__(self, *a, **k):
            pass

        def chat(self, messages, **k):
            return {'content': answers.pop(0)}

    was = client.Ollama
    client.Ollama = Stub
    try:
        got = {}
        for op in ('lie_floor', 'walk', 'backflip', 'tour', None):
            answers.append('{"op": %s}' % ('null' if op is None else '"%s"' % op))
            got[op] = gynoid.plan('x', 'rest', 'stub', rows)
    finally:
        client.Ollama = was
    take = got['backflip']['cmds'][0] if got['backflip']['cmds'] else {}
    report.check("each chosen row's own command: a say, a gait's twist, a take's rows, the tour; null nothing",
                 set(gynoid.choices(rows)) == {r['op'] for r in rows if r['kind'] != 'word'}
                 and got['lie_floor']['cmds'] == [{'say': 'lie_floor'}]
                 and got['walk']['cmds'] == [{'twist': se2.NOMINAL_TWIST['walk']}]
                 and take.get('entries', [('',)])[0][0] == 'backflip'
                 and got['tour']['tour'] and got['tour']['cmds'] == [] and got[None]['cmds'] == [],
                 {k: (v['cmds'][0] if v['cmds'] else v['tour']) for k, v in got.items() if k != 'backflip'})


ROSTER = (test_the_catalog_scans_her_sources, test_a_chosen_row_is_what_hears_it)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
