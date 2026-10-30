#!/usr/bin/env python3
"""The files over a model's reading budget: a file an agent reads whole, in tokens.

    python tools/dev/token_budget.py            # every file over BUDGET, the largest first
    python tools/dev/token_budget.py --all      # the whole distribution

A token is taken as four characters: code and English alike. ST's generated code (core/,
Drivers/, the startup) and a file whose first line says a tool writes it (GENERATED) are
never read and not counted.
"""
import argparse
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__)))))

#: Non-English (Swedish / Swenglish) characters and words forbidden in code, docs and commits;
#: coaxial_ollama's multilingual dictionaries and tests and measure_first's prompt matcher are
#: exempt by path.
SWEDISH_RE = re.compile(
    r'[\u00e5\u00e4\u00f6\u00c5\u00c4\u00d6]|\b('
    r'j\u00e4mfota\w*|jamfota\w*|spr\u00e5ng\w*|h\u00f6gt|g\u00e5ng\w*|l\u00f6pning\w*|'
    r's\u00e4ng\w*|stol\w*|d\u00f6rr\w*|knapp\w*|huvud\w*|v\u00e4nster\w*|h\u00f6ger\w*|'
    r'bak\u00e5t|fram\u00e5t|svenska|swenglish|fas\s+\d)\b',
    re.IGNORECASE,
)
MULTILINGUAL_ALLOW = (
    'host/coaxial_ollama/',
    'host/tests/test_ollama_',
    'host/tests/test_live_model.py',
    'host/tests/test_live_gynoid.py',
    'host/tools/dev/measure_first.py',
    'host/tools/dev/token_budget.py',
)


def english_text_faults(path, text):
    """['path:line: match'] of non-English / Swenglish tokens in `text` for `path`."""
    rel = path.replace('\\', '/')
    if any(rel.startswith(a) or ('/' + a) in rel for a in MULTILINGUAL_ALLOW):
        return []
    out = []
    for i, line in enumerate(text.splitlines(), 1):
        m = SWEDISH_RE.search(line)
        if m:
            out.append('%s:%d: non-English token %r' % (rel, i, m.group(0)))
    return out


def english_faults():
    """Every non-English / Swenglish token across the repository's read files."""
    out = []
    for rel, _t in files():
        with open(os.path.join(REPO, rel), encoding='utf-8', errors='replace') as f:
            out.extend(english_text_faults(rel, f.read()))
    return out

#: What a file may hold, tokens: past the host's 90th percentile (5.6 k), a file
#: read whole takes a tenth of a window's working room.
BUDGET = 6000

#: The files still over BUDGET, each capped where it stood when the budget came in: a split
#: takes one off, none may grow, and a new file over the budget fails test_structure.
#: Tokens.
HEAVY = {
    'host/tests/test_simulated.py': 22300,
    'host/tests/test_ollama_tools.py': 18600,
    'host/tests/test_render.py': 18100,
    'host/tests/test_thermal_core.py': 17300,
    'host/tests/test_ollama_runner.py': 13600,
    'host/tests/test_controller.py': 10300,
    'host/tools/notebooks/drive.py': 10000,
    'host/tests/test_ollama_link.py': 9900,
    'host/tests/test_ollama_prompt.py': 9400,
    'host/tests/test_sensorless.py': 9100,
    'host/coaxial/draw/cross_section.py': 8700,
    'thermal/src/thermal.c': 8500,
    'host/machine/world/physics.py': 7500,
    'board/native/native.c': 6800,
    'host/tests/test_conformance.py': 7400,
    'host/tests/test_modbus_core.py': 7300,
    'board/src/board_thermal.c': 7200,
    'host/tools/emu/emulator.py': 6300,
    'docs/PROTOCOL.md': 7000,
    'host/terminal/views/show_rotor_observer.py': 7000,
    'host/coaxial/draw/dial.py': 6900,
    'setup/toolchain.ps1': 6600,
    'host/coaxial/control/commission.py': 6400,
    'thermal/src/thermal_ident.c': 6400,
    'board/src/board_adc.c': 6200,
    'host/coaxial/kalman/thermal_ident.py': 6200,
    'board/src/board_pwm.c': 6100,
    'host/coaxial/model/thermal.py': 6100,
    'host/tools/sim/gvs.py': 7500,
    'docs/findings/orthokinematics.md': 6500,
}

#: What is read, by extension.
EXTENSIONS = ('.py', '.c', '.h', '.cs', '.md', '.ps1', '.lisp')

#: A file whose first line carries this is written by a tool, never read.
GENERATED = 'not by hand'

#: Trees never read: generated, vendored, built.
SKIP = ('core', 'drivers', 'build', '.git', '.venv', 'node_modules', '__pycache__',
        'notebook_examples', 'cmake', 'electronic_simulations')


def tokens(path):
    """A file's size in tokens, four characters each."""
    with open(path, encoding='utf-8', errors='replace') as f:
        return len(f.read()) / 4.0


def files():
    """Every file read, repo-relative with forward slashes, and its tokens."""
    for top, dirs, names in os.walk(REPO):
        dirs[:] = [d for d in dirs if d.lower() not in SKIP and not d.startswith('.')]
        for name in names:
            if name.endswith(EXTENSIONS) and not name.startswith('startup_'):
                path = os.path.join(top, name)
                with open(path, encoding='utf-8', errors='replace') as f:
                    if GENERATED in f.readline():
                        continue
                yield os.path.relpath(path, REPO).replace(os.sep, '/'), tokens(path)


def over(budget=BUDGET):
    """(tokens, path) of every file over `budget`, the largest first."""
    return sorted(((t, p) for p, t in files() if t > budget), reverse=True)


def breaches(budget=BUDGET):
    """(path, tokens, what it may hold) of every file past the budget and its cap."""
    # Case apart: a Windows checkout keeps a directory's case through a rename upstream, Thermal/
    # for git's thermal/ on the bench.
    caps = {path.lower(): cap for path, cap in HEAVY.items()}
    return [(p, t, caps.get(p.lower(), budget)) for t, p in over(budget)
            if t > caps.get(p.lower(), budget)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--all', action='store_true', help='the whole distribution')
    parser.add_argument('--budget', type=int, default=BUDGET)
    args = parser.parse_args(argv)
    if args.all:
        sizes = sorted(t for _p, t in files())
        for q in (0.5, 0.9, 0.99):
            print('%2d th percentile %6.1f k' % (100 * q, sizes[int(q * (len(sizes) - 1))] / 1e3))
        print('%d files, %.0f k tokens' % (len(sizes), sum(sizes) / 1e3))
    heavy = over(args.budget)
    print('%d files over %.1f k tokens, %.0f k in all' % (len(heavy), args.budget / 1e3,
                                                         sum(t for t, _p in heavy) / 1e3))
    for t, path in heavy:
        print('%6.1f k  %s' % (t / 1e3, path))
    return 1 if heavy else 0


if __name__ == '__main__':
    sys.exit(main())
