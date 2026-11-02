"""Moved from host/tools/dev/runner.py: run_one - nothing in the tree named it."""
from tools.dev.runner import _parse
from tools.dev.runner import run_captured
from tools.dev.suites import ROOT
import sys
import time


def run_one(path, timeout=300, extra=()):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None)."""
    started = time.monotonic()
    done = run_captured([sys.executable, str(path)] + list(extra), timeout,
                        cwd=str(ROOT))
    return _parse((done.stdout or '') + (done.stderr or ''), done.returncode,
                  time.monotonic() - started, timeout)
