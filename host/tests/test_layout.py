"""The tree keeps its shape: the target briefs its files, emulation stays transparent, counts are measured, notebooks are papers."""
import glob
import io
import os
import re
import sys

from tools.dev.focus import chosen
from structure_kit import HERE, REPO, Report


#: What only this host builds: the fake board, its silicon (native://), Renode's models.
HOST_ONLY = ('board/fake', 'board/native', 'board/emu')


#: A firmware source asking where it runs.
HOST_CONDITION = re.compile(
    r'^\s*#\s*(if|ifdef|ifndef|elif)\b.*\b(NATIVE|FAKE\w*|EMU\w*|RENODE|_WIN32|__linux__|'
    r'__x86_64__|__APPLE__)\b', re.M)


#: Prose that quotes a suite size. Each drifts the moment a check is added,
#: and each was wrong at once: CLAUDE.md said 1679 with structure at 300
#: when it was 349, and three more files carried a third total.
COUNTED = ('CLAUDE.md', 'host/run_tests.ps1', 'docs/ARCHITECTURE.md',
           'docs/TODO.md')


#: The notebooks' shape, as `tools/notebooks/parts.paper` lays it out.
#: A markdown cell is a fact, not an essay.
PROSE_MAX = 400


_FORBIDDEN = (('plt.subplots(', 'a figure outside coaxial.draw.figures'),
              ('figsize=', 'a figure size of its own'),
              ('!', 'an exclamation mark'))


def _paper(path):
    """The cells of one notebook, and every way it departs from the shape."""
    import json
    with io.open(path, encoding='utf-8') as handle:
        book = json.load(handle)
    cells = book['cells']
    text = lambda c: ''.join(c['source'])
    kinds = [c['cell_type'] for c in cells]
    said = [text(c) for c in cells]
    outputs = [o for c in cells for o in c.get('outputs', [])]
    wrong = []
    if 'widgets' in book['metadata'] or any(
            'application/vnd.jupyter.widget-view+json' in o.get('data', {}) for o in outputs):
        wrong.append('a widget kept - no viewer draws one without its kernel')
    if any(o.get('name') == 'stderr' for o in outputs):
        wrong.append('a cell wrote to stderr')
    if any('image/png' in o.get('data', {}) and 'image/jpeg' in o['data'] for o in outputs):
        wrong.append('a picture kept twice, png and jpeg')
    if any(o['output_type'] == 'execute_result' and set(o['data']) == {'text/plain'}
           for o in outputs):
        wrong.append('a cell ends on a bare value - print what it means')
    head = said[0].splitlines() if said else []
    if kinds[:2] != ['markdown', 'markdown'] or not head or not head[0].startswith('# '):
        wrong.append('no title cell')
    elif len(head) != 3 or not head[2]:
        wrong.append('the title cell is not a title and one line')
    opens = len(said) > 3 and 'Coaxial63100(' in said[3]
    if len(cells) < 4 or said[1] != '## 1 Setup' or 'MODE = ' not in said[2]:
        wrong.append('Setup is not the knob, and the open cell if it opens a device')
    headings = [s for k, s in zip(kinds, said) if k == 'markdown' and s.startswith('## ')]
    numbers = [h.split()[1] for h in headings if h[3].isdigit()]
    if numbers != [str(n) for n in range(1, len(numbers) + 1)]:
        wrong.append('sections not numbered 1.. in order: %s' % numbers)
    if not headings or not headings[-1].startswith('## References'):
        wrong.append('References is not the last section')
    if len(headings) < 2 or not headings[-2].endswith(' Results'):
        wrong.append('Results is not the section before References')
    if not any(s.startswith('**Bench.** ') for s in said):
        wrong.append('no Bench line')
    long = [s[:40] for k, s in zip(kinds, said)
            if k == 'markdown' and len(s) > PROSE_MAX and not s.startswith('## References')]
    if long:
        wrong.append('%d markdown cells over %d characters: %s' % (len(long), PROSE_MAX, long[0]))
    closes = any(k == 'code' and 'device.close()' in s for k, s in zip(kinds, said))
    if opens != closes:
        wrong.append('the device is never closed' if opens else 'a device closed, never opened')
    for k, c in zip(kinds, cells):
        if k != 'code':
            continue
        if not c.get('outputs') and 'print(' in text(c):
            wrong.append('a cell that prints has no output - not executed')
            break
        if any(o.get('output_type') == 'error' for o in c['outputs']):
            wrong.append('a cell raised: %s' % next(
                o['ename'] for o in c['outputs'] if o.get('output_type') == 'error'))
            break
    code = '\n'.join(s for k, s in zip(kinds, said) if k == 'code')
    prose = '\n'.join(s for k, s in zip(kinds, said) if k == 'markdown')
    wrong += ['%s in the code' % why for what, why in _FORBIDDEN[:2] if what in code]
    wrong += [why for what, why in _FORBIDDEN[2:] if what in prose]
    return wrong


def notebook_areas():
    """The areas `tools/notebooks` builds, by name."""
    from tools.notebooks import AREAS
    return AREAS


def test_target_briefs(r):
    """Every firmware file opens on `/** name - brief */`, one line, <= 100."""
    from tools.dev import target_map
    bad = []
    for d in target_map.DIRS:
        for path in glob.glob(os.path.join(REPO, d, '**', '*.[chs]'), recursive=True):
            if os.sep + 'test' + os.sep in path:
                continue
            text = io.open(path, encoding='utf-8').read()
            first = text.split('\n', 1)[0].lstrip('\ufeff').rstrip()
            if not (first.startswith('/**') and first.endswith('*/') and len(first) <= 100
                    and target_map.brief(text)):
                bad.append(os.path.relpath(path, REPO))
    r.check('every target file opens on a one-line brief (tools/dev/target_map.py)',
            not bad, '; '.join(bad[:4]))


def test_emulation_is_transparent(r):
    """fakeboard://, native:// and emulator:// run the firmware's source as the part does: the
    target builds none of theirs, and no source of its - CubeMX's included - asks where it
    runs."""
    from tools.dev import target_map
    cmake = io.open(os.path.join(REPO, 'CMakeLists.txt'), encoding='utf-8').read()
    reached = [d for d in HOST_ONLY if d in cmake]
    r.check('the target builds nothing of %s' % ', '.join(HOST_ONLY),
            not reached, ', '.join(reached))
    asks = []
    for d in target_map.DIRS + ('core',):
        for path in glob.glob(os.path.join(REPO, d, '**', '*.[chs]'), recursive=True):
            rel = os.path.relpath(path, REPO).replace(os.sep, '/')
            if rel.startswith(HOST_ONLY) or '/test/' in rel:
                continue
            if HOST_CONDITION.search(io.open(path, encoding='utf-8', errors='replace').read()):
                asks.append(rel)
    r.check('no firmware source asks whether it runs on this host or an emulator',
            not asks, '; '.join(asks[:4]))


def test_counts_are_measured(r):
    """Every suite size written in prose matches `.counts.json`."""
    import glob
    import json

    try:
        suites = json.loads(
            open(os.path.join(HERE, '.counts.json'), encoding='utf-8').read()
        )['suites']
    except (OSError, ValueError, KeyError):
        suites = {}
    if not suites:
        # A fresh clone has measured nothing - .counts.json is per-host by
        # design.
        r.check('no suite sizes measured on this host yet - the '
                'documents keep the last measured totals', True)
        return
    total = sum(suites.values())

    # A total needs every suite, and a tier runs a subset by design - the
    # default one runs two.
    everything = set(os.path.basename(p)
                     for p in glob.glob(os.path.join(HERE, 'test_*.py')))
    complete = not (everything - set(suites))

    for name in COUNTED:
        path = os.path.join(REPO, *name.split('/'))
        text = open(path, encoding='utf-8').read()

        wrong = ['%s (%d)' % (suite, said)
                 for suite, said in re.findall(
                     r'`(test_\w+\.py)`\s*\((\d+)', text)
                 for said in [int(said)]
                 if suites.get(suite, said) != said]
        r.check('%s quotes every suite at its size' % name,
                not wrong, ', '.join(wrong))

        totals = set(int(n) for n in re.findall(r"tree's (\d+) checks", text))
        totals |= set(int(n) for n in
                      re.findall(r'suites, (\d+) checks', text))
        totals |= set(int(n) for n in re.findall(r'\| (\d+) checks,', text))
        if complete:
            r.check('%s quotes the total as %d' % (name, total),
                    totals <= {total},
                    ', '.join(str(n) for n in sorted(totals)))
        else:
            r.check('%s keeps its total until all %d suites have run here'
                    % (name, len(everything)), True)


def test_notebooks_are_papers(r):
    """Every example notebook is one paper in the builder's shape, executed."""
    folder = os.path.join(REPO, 'notebook_examples')
    names = sorted(n for n in os.listdir(folder) if n.endswith('.ipynb'))
    for name in names:
        wrong = _paper(os.path.join(folder, name))
        r.check('%s is a paper in the builder\'s shape, executed' % name,
                not wrong, '; '.join(wrong[:3]))
    r.check('and there is one notebook per functional area',
            names == ['%s.ipynb' % a for a in sorted(notebook_areas())],
            ', '.join(names))


ROSTER = (test_target_briefs, test_emulation_is_transparent, test_counts_are_measured,
          test_notebooks_are_papers)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
