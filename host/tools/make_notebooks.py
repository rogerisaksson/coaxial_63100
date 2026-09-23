#!/usr/bin/env python3
"""Write notebook_examples/*.ipynb, and optionally execute them.

The notebooks are checked in WITH their outputs so they read without a
kernel, which makes them artefacts rather than sources: editing the
JSON by hand is how a cell's code and its printed output part company.
The source is `tools/notebooks/`, one module per functional area, each
a short paper laid out by `notebooks.parts.paper` so every one reads
the same way; this file writes them and `--execute` runs them, so what
is checked in is what the code actually printed.

    python tools/make_notebooks.py                  # write them
    python tools/make_notebooks.py --execute        # write and run
    python tools/make_notebooks.py --execute acquisition thermal
    python tools/make_notebooks.py --kernel status    # which python runs them
    python tools/make_notebooks.py --kernel install   # this one; setup.ps1 does it

Executing needs a kernel and the library: `jupyter`, `nbclient`,
`pandas` and `matplotlib`. Writing needs none of them. The notebooks
run against the stand-in, so no board is needed either - the knob at
the top of each is what a reader flips at the bench.

test_structure.py parses the code cells of every notebook as one module
(`notebook_source`), so a rename that leaves a dead call in a cell
fails there. What it cannot check is that the outputs match the code,
which is what --execute is for.
"""
import argparse
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import notebooks                                    # noqa: E402

#: notebook_examples/ sits beside host/, not under it: this file is
#: host/tools/make_notebooks.py, so the repository is two levels up.
OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))),
    'notebook_examples')

#: THE KERNEL THE NOTEBOOKS NAME, registered on the interpreter setup.ps1
#: installs into (`--kernel install`). An editor matches a notebook to a
#: kernel by this name first and by the language's version second, and
#: the second is not enough: this bench carries two CPython 3.14.7s, one
#: with the packages and one - uv-managed - with none, and the editor
#: opened every notebook on the empty one (2026-09-13). A name only one
#: kernelspec carries settles it, and that kernelspec starts the
#: interpreter by its absolute path.
KERNEL = 'coaxial_63100'
KERNEL_DISPLAY = 'Python (coaxial_63100)'

#: Name -> cells, one notebook per functional area, in the README's order.
NOTEBOOKS = notebooks.AREAS


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


def write(name, out_dir):
    """Write one notebook, outputs empty. Returns the path."""
    path = os.path.join(out_dir, name + '.ipynb')
    with io.open(path, 'w', encoding='utf-8', newline='\n') as handle:
        json.dump(notebook(name, NOTEBOOKS[name]), handle, indent=1,
                  ensure_ascii=False)
        handle.write('\n')
    return path


def execute(path, out_dir, timeout=1800):
    """Run a notebook in place."""
    import nbformat
    from jupyter_client.kernelspec import NoSuchKernel
    from nbclient import NotebookClient

    book = nbformat.read(path, as_version=4)
    try:
        NotebookClient(book, timeout=timeout, kernel_name=KERNEL,
                       resources={'metadata': {'path': out_dir}},
                       allow_errors=True).execute()
    except NoSuchKernel:
        return ('kernel %s is not registered on this python: '
                'make_notebooks.py --kernel install' % KERNEL)
    nbformat.write(book, path)
    for number, one in enumerate(book.cells):
        for output in one.get('outputs', []):
            if output.get('output_type') == 'error':
                return 'cell %d: %s: %s' % (number, output.get('ename'),
                                            output.get('evalue'))
    return None


def _kernelspec_api():
    """The registry and the installer, which arrive with ipykernel -
    optional here, like nbclient: reading a notebook needs neither."""
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
    """One line on the registered kernel, and whether it is this
    interpreter - the property setup.ps1 checks."""
    found = kernel_interpreter()
    if found is None:
        return 'not registered', False
    if os.path.exists(found) and os.path.samefile(found, sys.executable):
        return found, True
    return '%s - not this python (%s)' % (found, sys.executable), False


def kernel_command(action):
    """`--kernel status` prints which interpreter the notebooks' kernel
    starts and exits 1 unless it is this one; `--kernel install`
    registers it here.
    """
    try:
        detail, fine = (('%s -> %s' % (install_kernel(), sys.executable), True)
                        if action == 'install' else kernel_status())
    except ImportError as absent:
        detail, fine = str(absent), False
    print(detail)
    return 0 if fine else 1


def main(argv=None):
    """Write the notebooks named on the command line, or all of them."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    parser.add_argument('names', nargs='*', help='notebooks; default all')
    parser.add_argument('--execute', action='store_true',
                        help='run each one and keep its outputs')
    parser.add_argument('--out', default=OUT_DIR)
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

    failed = []
    for name in wanted:
        path = write(name, args.out)
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


if __name__ == '__main__':
    sys.exit(main())
