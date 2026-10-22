"""`python -m terminal`: the loader, guarded so a crew's spawned worker importing it runs nothing.

A page named on the command line has its drawing's process (its AHEAD) started first, before the
loader's own imports take their 0.27 s: opened by name, it finds its drawing that much nearer up.
"""
import importlib
import sys


def _drawing_first(argv):
    """The named page's drawing started now, where the name is a module of pages/ with an AHEAD;
    a smoke run's never."""
    named = next((word for word in argv if not word.startswith('-')), None)
    try:
        page = importlib.import_module('terminal.pages.' + named) if named else None
    except (ImportError, ValueError):
        return
    drawing = getattr(page, 'AHEAD', None)
    if drawing is not None and '--frames' not in argv:
        from terminal.ui import ahead
        ahead.keep(*drawing)


if __name__ == '__main__':
    _drawing_first(sys.argv[1:])
    from .loader import main
    sys.exit(main())
