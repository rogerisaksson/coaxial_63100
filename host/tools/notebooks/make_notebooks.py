#!/usr/bin/env python3
"""notebook_examples/*.ipynb out of tools/notebooks/, checked in executed.

    python tools/notebooks/make_notebooks.py                  # write, outputs empty
    python tools/notebooks/make_notebooks.py --execute [name..]  # write and run
    python tools/notebooks/make_notebooks.py --compare [name..]  # the stand-in against the emulator
    python tools/notebooks/make_notebooks.py --kernel status|install

Executing needs jupyter, nbclient, pandas, matplotlib; the emulator or the stand-in, no board.
Several run side by side, a process each - a kernel and an emulator each (tools.dev.focus).
"""
import argparse
import datetime
import io
import json
import os
import re
import sys

from tools import notebooks
from tools.dev.focus import WORKER_GB, Job, relay

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))),
    'notebook_examples')

#: A name only this kernelspec carries: two CPython 3.14.7s here, one without
#: the packages, and the editor picked that one by version (2026-09-13).
KERNEL = 'coaxial_63100'
KERNEL_DISPLAY = 'Python (coaxial_63100)'

NOTEBOOKS = notebooks.AREAS

#: A cell's time and a notebook's, s.
CELL_S, NOTEBOOK_S = 600, 900

#: Where --compare writes its two runs: not the checked-in papers.
COMPARED = os.path.join(os.path.dirname(OUT_DIR), 'build', 'notebooks_compared')

#: Each paper's last seconds on either knob: the relay takes the longest first.
SECONDS = os.path.join(os.path.dirname(OUT_DIR), 'build', 'notebooks_seconds.json')

#: A paper's commit on the relay, GB: the kernel alone on the stand-in, with a Renode beside it
#: on the emulator.
STAND_IN_GB = 0.4

#: A cell whose emulated run takes this many times the stand-in's and this many seconds more
#: is out of proportion: the emulator runs a board second in 2-20 wall s. Numbers a cell
#: prints apart by more than this share of the larger, on both sides alike in count.
SLOW_RATIO, SLOW_S, APART = 25.0, 20.0, 0.5


def cell(kind, text, ident):
    """One notebook cell, in nbformat 4's shape."""
    lines = text.split('\n')
    source = [line + '\n' for line in lines[:-1]] + [lines[-1]]
    if kind == 'markdown':
        return {'cell_type': 'markdown', 'id': ident, 'metadata': {},
                'source': source}
    return {'cell_type': 'code', 'id': ident, 'metadata': {}, 'source': source,
            'outputs': [], 'execution_count': None}


def notebook(name, cells):
    """One notebook as the dict nbformat writes."""
    return {
        'cells': [cell(kind, text, '%s-%02d' % (name.replace('_', '-'), i))
                  for i, (kind, text) in enumerate(cells)],
        'metadata': {
            'kernelspec': {'display_name': KERNEL_DISPLAY, 'language': 'python',
                           'name': KERNEL},
            'language_info': {'name': 'python'},
        },
        'nbformat': 4, 'nbformat_minor': 5,
    }


def write(name, out_dir, mode=None):
    """Write one notebook, outputs empty, on its own mode or `mode`'s. Returns the path."""
    path = os.path.join(out_dir, name + '.ipynb')
    cells = NOTEBOOKS[name] if mode is None else notebooks.paper_of(name, (mode, 'compared'))
    with io.open(path, 'w', encoding='utf-8', newline='\n') as handle:
        json.dump(notebook(name, cells), handle, indent=1,
                  ensure_ascii=False)
        handle.write('\n')
    return path


#: A widget view names a live kernel's model: no viewer draws it from the file.
WIDGET_VIEW = 'application/vnd.jupyter.widget-view+json'


def execute(path, out_dir, timeout=CELL_S):
    """Run a notebook in place; what it keeps is what a viewer with no kernel can draw."""
    import warnings

    import nbformat
    from jupyter_client.kernelspec import NoSuchKernel
    from nbclient import NotebookClient

    # zmq on Windows' proactor loop, at every kernel: said on stderr, not a finding.
    warnings.filterwarnings('ignore', message='Proactor event loop does not implement add_reader')

    book = nbformat.read(path, as_version=4)
    try:
        NotebookClient(book, timeout=timeout, kernel_name=KERNEL,
                       resources={'metadata': {'path': out_dir}},
                       allow_errors=True, store_widget_state=False).execute()
    except NoSuchKernel:
        return ('kernel %s is not registered on this python: '
                'make_notebooks.py --kernel install' % KERNEL)
    book.metadata.pop('widgets', None)
    for one in book.cells:
        for output in one.get('outputs', []):
            data = output.get('data', {})
            data.pop(WIDGET_VIEW, None)
            if 'image/png' in data:
                data.pop('image/jpeg', None)      # a picture once, lossless
    nbformat.write(book, path)
    for number, one in enumerate(book.cells):
        for output in one.get('outputs', []):
            if output.get('output_type') == 'error':
                return 'cell %d: %s: %s' % (number, output.get('ename'),
                                            output.get('evalue'))
    return None


def _kernelspec_api():
    try:
        import ipykernel.kernelspec
        import jupyter_client.kernelspec
    except ImportError:
        raise ImportError("the notebooks' kernel needs ipykernel: "
                          'python -m pip install -r host/requirements.txt') from None
    return ipykernel.kernelspec, jupyter_client.kernelspec


def kernel_interpreter():
    """The interpreter the registered kernel starts, or None when none is."""
    _, registry = _kernelspec_api()
    try:
        return registry.KernelSpecManager().get_kernel_spec(KERNEL).argv[0]
    except registry.NoSuchKernel:
        return None


def install_kernel():
    """Register the kernel on this interpreter, for this user."""
    installer, _ = _kernelspec_api()
    return installer.install(user=True, kernel_name=KERNEL,
                             display_name=KERNEL_DISPLAY)


def kernel_status():
    """The registered kernel's interpreter, and whether it is this one."""
    found = kernel_interpreter()
    if found is None:
        return 'not registered', False
    if os.path.exists(found) and os.path.samefile(found, sys.executable):
        return found, True
    return '%s - not this python (%s)' % (found, sys.executable), False


def kernel_command(action):
    """status: exit 1 unless the kernel starts this python; install: register it."""
    try:
        detail, fine = (('%s -> %s' % (install_kernel(), sys.executable), True)
                        if action == 'install' else kernel_status())
    except ImportError as absent:
        detail, fine = str(absent), False
    print(detail)
    return 0 if fine else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    parser.add_argument('names', nargs='*', help='notebooks; default all')
    parser.add_argument('--execute', action='store_true',
                        help='run each one and keep its outputs')
    parser.add_argument('--out', default=OUT_DIR)
    parser.add_argument('--jobs', type=int, help='batons: notebooks at once, the physical cores')
    parser.add_argument('--mode', choices=('SIMULATED', 'EMULATED'),
                        help="on this knob rather than the paper's own")
    parser.add_argument('--compare', action='store_true',
                        help='each on the stand-in and on the emulator, the two held apart')
    parser.add_argument('--kernel', choices=('status', 'install'),
                        help='the kernel the notebooks name: which python '
                             'it starts, or register it on this one')
    args = parser.parse_args(argv)
    if args.kernel:
        return kernel_command(args.kernel)

    unknown = [n for n in args.names if n not in NOTEBOOKS]
    if unknown:
        parser.error('no such notebook: %s. There are %d: %s'
                     % (', '.join(unknown), len(NOTEBOOKS),
                        ', '.join(sorted(NOTEBOOKS))))
    wanted = args.names or sorted(NOTEBOOKS)
    if not os.path.isdir(args.out):
        os.makedirs(args.out)

    if args.compare:
        return compare(wanted, args)
    if args.execute and len(wanted) > 1:
        return side_by_side_execute(wanted, args)
    failed = []
    for name in wanted:
        path = write(name, args.out, args.mode)
        if not args.execute:
            print('wrote %s' % os.path.basename(path))
            continue
        sys.stdout.write('%-30s ' % name)
        sys.stdout.flush()
        error = execute(path, args.out)
        print('FAILED  %s' % error if error else 'ok')
        if error:
            failed.append(name)

    print('%d notebook%s%s' % (len(wanted), '' if len(wanted) == 1 else 's',
                               ', %d failed' % len(failed) if failed else ''))
    return 1 if failed else 0


def _seconds():
    """Each paper's last seconds, by `name:MODE`."""
    try:
        with io.open(SECONDS, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def _relayed(jobs, batons):
    """`jobs` on the relay, the longest last time first; each one's seconds kept for the next."""
    took = _seconds()
    order = sorted(jobs, key=lambda job: -took.get(job.name, float('inf')))
    for job, out, code, seconds in relay(order, batons):
        took[job.name] = round(seconds, 1)
        yield job, out, code, seconds
    os.makedirs(os.path.dirname(SECONDS), exist_ok=True)
    with io.open(SECONDS, 'w', encoding='utf-8') as handle:
        json.dump(took, handle, indent=1, sort_keys=True)


def _mode_of(name, mode=None):
    """The ExecutionMode a paper runs on: `mode`, else its own."""
    return mode or notebooks.mode_of(name)[0]


def side_by_side_execute(wanted, args):
    """`wanted` written and run on the relay, a process each; a line each as it ends."""
    jobs = [Job('%s:%s' % (name, _mode_of(name, args.mode)),
                [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--execute',
                 '--out', args.out] + (['--mode', args.mode] if args.mode else []) + [name],
                STAND_IN_GB if _mode_of(name, args.mode) == 'SIMULATED' else WORKER_GB,
                NOTEBOOK_S) for name in wanted]
    failed = []
    for job, out, code, took in _relayed(jobs, args.jobs):
        name = job.name.split(':')[0]
        told = [line[line.index('FAILED'):] for line in out.splitlines() if 'FAILED' in line]
        said = ('ok' if code == 0 else 'FAILED  out of time at %.0f s' % NOTEBOOK_S
                if code is None else told[-1] if told else 'FAILED  %s' % out.strip()[-300:])
        print('%-30s %s  %.0f s' % (name, said, took), flush=True)
        if code != 0:
            failed.append(name)
    print('%d notebooks%s' % (len(wanted), ', %d failed' % len(failed) if failed else ''))
    return 1 if failed else 0


def cells_run(path):
    """Each code cell of an executed notebook: (seconds or None, error or None, its numbers,
    its first line)."""
    with io.open(path, encoding='utf-8') as handle:
        book = json.load(handle)
    out = []
    for one in (c for c in book['cells'] if c['cell_type'] == 'code'):
        timing = one.get('metadata', {}).get('execution', {})
        began, ended = timing.get('iopub.execute_input'), timing.get('shell.execute_reply')
        took = ((datetime.datetime.fromisoformat(ended) - datetime.datetime.fromisoformat(began))
                .total_seconds() if began and ended else None)
        error, text = None, ''
        for output in one.get('outputs', []):
            if output.get('output_type') == 'error':
                error = '%s: %s' % (output.get('ename'), output.get('evalue', '')[:120])
            text += ''.join(output.get('text', '')) + ''.join(
                output.get('data', {}).get('text/plain', ''))
        numbers = [float(n) for n in re.findall(r'-?\d+\.\d+|-?\d+', text)]
        out.append((took, error, numbers, ''.join(one['source']).split('\n')[0][:60]))
    return out


def apart(a, b):
    """The largest share two lists of numbers part by, pair for pair; None unlike in count."""
    if len(a) != len(b) or not a:
        return None
    return max(abs(x - y) / max(abs(x), abs(y)) if max(abs(x), abs(y)) > 1e-3 else 0.0
               for x, y in zip(a, b))


def compare(wanted, args):
    """Each of `wanted` on the stand-in and on the emulator side by side, a process each; per
    paper the two totals, the errors, the cells out of proportion and the numbers apart."""
    runs = {mode: os.path.join(COMPARED, mode.lower()) for mode in ('SIMULATED', 'EMULATED')}
    for folder in runs.values():
        os.makedirs(folder, exist_ok=True)
    jobs = [Job('%s:%s' % (name, mode),
                [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--execute', '--mode',
                 mode, '--out', runs[mode], name],
                WORKER_GB if mode == 'EMULATED' else STAND_IN_GB, NOTEBOOK_S)
            for mode in ('EMULATED', 'SIMULATED') for name in wanted]
    for job, _out, code, took in _relayed(jobs, args.jobs):
        name, mode = job.name.split(':')
        print('%-16s %-9s %s  %.0f s' % (name, mode.lower(), 'ran' if code == 0 else
                                        'out of time' if code is None else 'FAILED', took),
              flush=True)
    worth = 0
    print('\n%-16s %9s %9s %6s  what differs' % ('paper', 'stand-in', 'emulated', 'x'))
    for name in wanted:
        sim, emu = (cells_run(os.path.join(runs[m], name + '.ipynb')) for m in runs)
        total = [sum(t for t, _, _, _ in side if t) for side in (sim, emu)]
        said = []
        for k, (s, e) in enumerate(zip(sim, emu)):
            if e[1] or s[1]:
                said.append('cell %d raised - stand-in %s, emulated %s' % (k, s[1], e[1]))
            elif s[0] is not None and e[0] is not None and e[0] > SLOW_RATIO * max(s[0], 0.05) \
                    and e[0] - s[0] > SLOW_S:
                said.append('cell %d %.1f -> %.1f s: %s' % (k, s[0], e[0], s[3]))
            elif (apart(s[2], e[2]) or 0.0) > APART:
                said.append('cell %d numbers apart %.0f %%: %s'
                            % (k, 100.0 * (apart(s[2], e[2]) or 0.0), s[3]))
        if len(sim) != len(emu) or not total[0] or not total[1]:
            said.insert(0, 'a run is missing or incomplete')
        worth += bool(said)
        print('%-16s %8.1fs %8.1fs %6.1f  %s' % (name, total[0], total[1],
                                                total[1] / total[0] if total[0] else 0.0,
                                                said[0] if said else '-'))
        for line in said[1:]:
            print('%52s%s' % ('', line))
    print('\n%d papers, %d with something to look at' % (len(wanted), worth))
    return 1 if worth else 0


if __name__ == '__main__':
    sys.exit(main())
