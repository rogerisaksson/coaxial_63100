#!/usr/bin/env python3
"""The papers their sources' or laid: a pull leaves them behind, and a reader opens what stands.

    python tools/notebooks/current.py            # said, and laid or run where behind; 1 where one failed
    python tools/notebooks/current.py --behind   # the papers behind, comma-joined; nothing where none

A paper in notebook_examples/ is behind where its cells are not the builder's (tools/notebooks) or one of
its code cells never ran: four of thirteen stood so, one on a dead time the record had since changed. One
whose code is what ran - its cells apart from the builder's by prose and comments alone - is laid again
with the outputs it has and nothing is run; one whose code changed is run apart (STAGED) and laid where it
ran whole. A paper that fails leaves the one that stood and is not run again till what it runs changes:
a pull does not spend its minutes twice. The index beside them (README.md) is laid with them. setup.ps1
and git's hooks after a pull and a rebase ask this, as they ask tools/target/current.py for the images.
"""
import argparse
import ast
import hashlib
import io
import json
import os
import sys

from tools import notebooks
from tools.notebooks import make_notebooks
from tools.notebooks.make_notebooks import NOTEBOOKS, OUT_DIR
from tools.notebooks.parts import KERNEL_DISPLAY

#: Where a paper behind is run: one that fails leaves the checked-in paper as it stood.
STAGED = os.path.join(os.path.dirname(OUT_DIR), 'build', 'notebooks_current')

#: The papers that failed, each by what it ran: not run again till that changes.
FAILED = os.path.join(os.path.dirname(OUT_DIR), 'build', 'notebooks_failed.json')

#: The index beside the papers, and its text: how many papers, the kernel's name, a row a paper.
INDEX = 'README.md'
INDEX_TEXT = """# The notebooks

%d short papers on the board, each run as it stands: no board, no cable.
They are generated: `host/tools/notebooks/` is their source.

## Before the first one

1. `setup.ps1` once: the packages, the kernel `%s`,
   the firmware image, Renode.
2. Open a paper in VS Code and pick that kernel.
3. Run All. `MODE` in its Setup says where it runs: `EMULATED` is this
   host's firmware image on Renode, and the host's own stand-in where no
   emulator runs.

## The papers, in reading order

| paper | what it shows | runs on |
| --- | --- | --- |
%s

## At the bench

In a paper's Setup: `MODE = ExecutionMode.HARDWARE` and `PORT` the board's
port - `coaxial.comm.ports.find()` names the first that answers.

## Changing one

A paper is written from `host/tools/notebooks/<its name>.py`: edit that, not
the notebook - what is typed into a notebook is gone when it is next laid.

```powershell
python host/tools/notebooks/current.py
```

It lays the papers behind their sources - run again where their code
changed, with the outputs they have where only their prose did; `setup.ps1`
and every pull and rebase do the same.
"""

#: What a paper's mode is called in the index.
RUNS_ON = {'EMULATED': 'the image on Renode', 'SIMULATED': 'the stand-in'}


def index():
    """The index's text: what a reader does before the first paper, the papers in their order - each its
    title, what it shows, what it runs on -, the bench's knob, how one is changed."""
    rows = ['| [%s](%s.ipynb) | %s | %s |' % (area.TITLE, name, area.SUMMARY, RUNS_ON[notebooks.mode_of(name)[0]])
            for name, area in notebooks.MODULES.items()]
    return INDEX_TEXT % (len(rows), KERNEL_DISPLAY, '\n'.join(rows))


def index_behind(out_dir=OUT_DIR):
    """Whether the index in `out_dir` is not the builder's text."""
    try:
        with io.open(os.path.join(out_dir, INDEX), encoding='utf-8', newline='') as handle:
            return handle.read() != index()
    except OSError:
        return True


def _stood(name, out_dir):
    """The paper `name` as it stands in `out_dir`; None where none does, or what stands is no notebook."""
    try:
        with io.open(os.path.join(out_dir, name + '.ipynb'), encoding='utf-8') as handle:
            book = json.load(handle)
        return book if isinstance(book['cells'], list) else None
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _cells(book):
    """A notebook's cells: each its kind and its source."""
    return [(c['cell_type'], ''.join(c['source'])) for c in book['cells']]


def behind(names, out_dir=OUT_DIR):
    """The papers of `names` behind their sources in `out_dir`: none there, cells other than the builder's -
    their kinds and their sources -, or a code cell that never ran."""
    late = []
    for name in names:
        book = _stood(name, out_dir)
        if (book is None or _cells(book) != _cells(make_notebooks.notebook(name, NOTEBOOKS[name]))
                or any(c['cell_type'] == 'code' and c.get('execution_count') is None for c in book['cells'])):
            late.append(name)
    return late


def _runs(source):
    """What a code cell runs: its syntax tree, which holds no comment and no blank line - its text where an
    editor's own line keeps it from parsing."""
    try:
        return ast.dump(ast.parse(source))
    except SyntaxError:
        return source


def _run_by(name):
    """What the builder's paper `name` runs, as one word: its code cells' trees in their order."""
    cells = make_notebooks.notebook(name, NOTEBOOKS[name])['cells']
    told = '\n'.join(_runs(''.join(c['source'])) for c in cells if c['cell_type'] == 'code')
    return hashlib.sha1(told.encode('utf-8')).hexdigest()


def carried(name, book):
    """The builder's paper `name` with the outputs of `book`, the paper that stands - where each of its code
    cells ran and runs what the builder's does now, prose and comments alone apart; None where one differs,
    is new, never ran, or no paper stands."""
    laid = make_notebooks.notebook(name, NOTEBOOKS[name])
    ran = [c for c in book['cells'] if c['cell_type'] == 'code'] if book else None
    new = [c for c in laid['cells'] if c['cell_type'] == 'code']
    if ran is None or len(ran) != len(new) or any(
            old.get('execution_count') is None or _runs(''.join(old['source'])) != _runs(''.join(cell['source']))
            for old, cell in zip(ran, new)):
        return None
    for old, cell in zip(ran, new):
        cell.update(outputs=old['outputs'], execution_count=old['execution_count'],
                    metadata=old.get('metadata', {}))
    laid['metadata'] = book['metadata']
    return laid


def _failed():
    """{a paper that failed: what it ran}, as the last run here left it."""
    try:
        with io.open(FAILED, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def current(names, out_dir=OUT_DIR, jobs=None):
    """The papers of `names` behind their sources laid in `out_dir` - with the outputs they have where their
    code is what ran, run apart and laid where they ran whole where it changed - and the index beside them;
    nothing said where none is behind. 1 where a paper failed, now or on the same code before."""
    if index_behind(out_dir):
        with io.open(os.path.join(out_dir, INDEX), 'w', encoding='utf-8', newline='\n') as handle:
            handle.write(index())
        print('NOTEBOOKS  %s laid' % INDEX, flush=True)
    late = behind(names, out_dir)
    if not late:
        return 0
    import nbformat
    books = {name: carried(name, _stood(name, out_dir)) for name in late}
    for name, book in books.items():
        if book is not None:
            with io.open(os.path.join(out_dir, name + '.ipynb'), 'w', encoding='utf-8', newline='\n') as handle:
                nbformat.write(nbformat.from_dict(book), handle)
    laid = [name for name in late if books[name] is not None]
    if laid:
        print('NOTEBOOKS  %d laid with the outputs they have, their code what ran (%s)'
              % (len(laid), ', '.join(laid)), flush=True)
    failed = _failed()
    stuck = [name for name in late if books[name] is None and failed.get(name) == _run_by(name)]
    for name in stuck:
        print('NOTEBOOKS  %s stands behind: it failed on this code (make_notebooks.py --execute %s says where)'
              % (name, name), flush=True)
    run = [name for name in late if books[name] is None and name not in stuck]
    if run:
        print('NOTEBOOKS  %d behind, their code changed (%s): running' % (len(run), ', '.join(run)), flush=True)
        os.makedirs(STAGED, exist_ok=True)
        fell = make_notebooks.executed(run, argparse.Namespace(out=STAGED, mode=None, jobs=jobs))
        for name in run:
            if name in fell:
                failed[name] = _run_by(name)
            else:
                failed.pop(name, None)
                os.replace(os.path.join(STAGED, name + '.ipynb'), os.path.join(out_dir, name + '.ipynb'))
        with io.open(FAILED, 'w', encoding='utf-8') as handle:
            json.dump(failed, handle, indent=1, sort_keys=True)
        if fell:
            print('NOTEBOOKS  %s failed: the papers that stood are kept' % ', '.join(fell), flush=True)
        stuck += fell
    return 1 if stuck else 0


def main(argv=None):
    """`--behind`: the papers behind and the index where it is, comma-joined, 0; else `current`."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    parser.add_argument('--behind', action='store_true',
                        help='name the papers behind their sources; nothing where none is')
    parser.add_argument('--jobs', type=int, help='batons: papers run at once, the physical cores')
    args = parser.parse_args(argv)
    names = sorted(NOTEBOOKS)
    if args.behind:
        print(','.join(behind(names) + ([INDEX] if index_behind() else [])))
        return 0
    return current(names, jobs=args.jobs)


if __name__ == '__main__':
    sys.exit(main())
