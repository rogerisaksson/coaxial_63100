#!/usr/bin/env python3
"""The files over a model's reading budget: a file an agent reads whole, in tokens.

    python tools/dev/token_budget.py            # every file over BUDGET, the largest first
    python tools/dev/token_budget.py --all      # the whole distribution

A token is taken as four characters: code and English alike. ST's generated code (core/,
Drivers/, the startup) is never read and not counted.
"""
import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__)))))

#: What a file may hold, tokens: past the host's 90th percentile (5.6 k, 2026-09-28), a file
#: read whole takes a tenth of a window's working room.
BUDGET = 6000

#: The files still over BUDGET, each capped where it stood when the budget came in: a split
#: takes one off, none may grow, and a new file over the budget fails test_structure
#: (2026-09-28). Tokens.
HEAVY = {
    'host/tests/test_simulated.py': 22300,
    'docs/FINDINGS.md': 20500,
    'host/tests/test_ollama_tools.py': 18600,
    'host/tests/test_render.py': 18100,
    'host/tests/test_thermal_core.py': 17300,
    'host/tests/test_ollama_runner.py': 13600,
    'host/coaxial/graphics/gynoid.py': 11800,
    'host/tests/test_controller.py': 10300,
    'host/tools/notebooks/drive.py': 10000,
    'host/tests/test_ollama_link.py': 9900,
    'host/tests/test_ollama_prompt.py': 9400,
    'host/tests/test_sensorless.py': 9100,
    'host/coaxial/draw/cross_section.py': 8700,
    'thermal/src/thermal.c': 8500,
    'host/machine/physics.py': 8200,
    'board/native/native.c': 7600,
    'host/tests/test_boot_core.py': 7400,
    'host/tests/test_conformance.py': 7400,
    'host/tests/test_modbus_core.py': 7300,
    'board/src/board_thermal.c': 7200,
    'host/tools/emu/emulator.py': 7100,
    'docs/PROTOCOL.md': 7000,
    'host/terminal/views/show_rotor_observer.py': 7000,
    'host/coaxial/draw/dial.py': 6900,
    'setup/toolchain.ps1': 6600,
    'host/machine/gait.py': 6500,
    'host/coaxial/simulated/acquire/daq.py': 6400,
    'host/coaxial/control/commission.py': 6400,
    'thermal/src/thermal_ident.c': 6400,
    'host/terminal/views/show_thermal_observer.py': 6300,
    'board/src/board_adc.c': 6200,
    'host/coaxial/kalman/thermal_ident.py': 6200,
    'board/src/board_pwm.c': 6100,
    'host/coaxial/model/thermal.py': 6100,
}

#: What is read, by extension.
EXTENSIONS = ('.py', '.c', '.h', '.cs', '.md', '.ps1')

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
                yield os.path.relpath(path, REPO).replace(os.sep, '/'), tokens(path)


def over(budget=BUDGET):
    """(tokens, path) of every file over `budget`, the largest first."""
    return sorted(((t, p) for p, t in files() if t > budget), reverse=True)


def breaches(budget=BUDGET):
    """(path, tokens, what it may hold) of every file past the budget and its cap."""
    return [(p, t, HEAVY.get(p, budget)) for t, p in over(budget) if t > HEAVY.get(p, budget)]


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
