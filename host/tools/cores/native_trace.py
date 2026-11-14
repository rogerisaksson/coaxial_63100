"""The native transport's slow moments, written where asked: what found a limb 16 s behind the wall.

    COAXIAL_NATIVE_TRACE=FILE python -X utf8 tools/dev/run_tests.py --file test_native.py   # FILE a JSON line each

A send (`native.Limb.send`) whose lock, catch-up or lag forgiven, or a pace (`native.Limb._pace`) whose gap,
lock or run took past `native.BURST_S` of the wall, a line each: on the N2 runner a limb fell 10-16 s behind
beside the page drawing and a send ran it through in one call. Unset, nothing is timed.
"""
import json
import os
import time

#: Where the slow moments are written, a JSON line each; None: not asked, none timed.
TRACE = os.environ.get('COAXIAL_NATIVE_TRACE')


def traced(kind, **said):
    """A slow moment written to TRACE (unset: nothing): the wall's time, its kind, what it says."""
    if TRACE:
        with open(TRACE, 'a', encoding='utf-8') as f:
            f.write(json.dumps(dict(wall=round(time.monotonic(), 3), kind=kind, **said)) + '\n')
