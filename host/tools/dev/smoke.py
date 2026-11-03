#!/usr/bin/env python3
"""The push's smoke: every `machine` module imported, her first seconds headless, her C model hers.

    python tools/dev/smoke.py       # .githooks/pre-push; a fault stops the push

She stands on her latent loop at the page's rate, is told to walk at WALK_S, and is followed to
SECONDS: a raise or her pelvis past TILT_DEG is a fault, as is a module that does not import.
"""
import importlib
import os
import pkgutil
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

SECONDS, WALK_S, TILT_DEG = 1.5, 0.5, 25.0


def imported(package='machine'):
    """[(module, its fault)] of the modules under `package` that do not import."""
    bad = []
    for info in pkgutil.walk_packages(importlib.import_module(package).__path__, package + '.'):
        try:
            importlib.import_module(info.name)
        except Exception as exc:                # any fault stops the push
            bad.append((info.name, '%s: %s' % (type(exc).__name__, exc)))
    return bad


def ran():
    """None, or how her first SECONDS on the loop went wrong."""
    from machine.process import sprung
    g, gym = sprung.laid(sprung.STACK_EVERY), sprung.entries()
    told = False
    try:
        while g['t'] < SECONDS:
            if not told and g['t'] >= WALK_S:
                sprung._heard(g, gym, {'walk': True, 'twist': (0.5, 0.0, 0.0)})
                told = True
            sprung.passed(g)
            if sprung.tilt(g) > TILT_DEG:
                return 'down at %.2f s, pelvis %.0f deg' % (g['t'], sprung.tilt(g))
    except Exception:                           # any fault stops the push
        return 'raised at %.3f s:\n%s' % (g['t'], traceback.format_exc())
    return None


def modelled():
    """None, or how her C core's model is not her build's: its whole mass against her body's (`model.built`)."""
    import numpy as np
    from machine.stack import wbc
    from tools.cores import wbc as cores
    core = cores.Core()
    core.pose([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], np.zeros(core.n - 6))
    held, hers = float(core.mass()[3, 3]), float(wbc.Body().mass)
    return None if held == hers else 'her C core holds %.4f kg, her build %.4f' % (held, hers)


def main():
    from tools.dev import token_budget
    began = time.perf_counter()
    non_en = token_budget.english_faults()
    for fault in non_en[:30]:
        print('smoke: English-only violation - %s' % fault)
    bad = imported()
    for name, fault in bad:
        print('smoke: %s does not import - %s' % (name, fault))
    fault = None if (bad or non_en) else ran()
    if fault:
        print('smoke: her first %.1f s on the loop: %s' % (SECONDS, fault))
    held = None if (bad or non_en or fault) else modelled()
    if held:
        print('smoke: %s' % held)
    failed = bad or fault or non_en or held
    print('smoke: %s (%.1f s)' % ('FAILED' if failed else 'clean', time.perf_counter() - began))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
