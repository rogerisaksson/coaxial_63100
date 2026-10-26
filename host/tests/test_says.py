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
    from machine import maneuver, route, says, sprung, takes
    rows = says.catalog()
    by = {}
    for r in rows:
        by.setdefault(r['kind'], set()).add(r['op'])
    ops = [r['op'] for r in rows]
    docks = set(route.docks())
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


ROSTER = (test_the_catalog_scans_her_sources,)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
