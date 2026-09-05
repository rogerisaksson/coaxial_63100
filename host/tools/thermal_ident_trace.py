"""Trace the thermal identification sample by sample against the ground
truth `test_thermal_core.py` uses: every innovation with its regressor,
and the scales after each update.

    python tools/thermal_ident_trace.py            # three box cycles, air 2.0
    python tools/thermal_ident_trace.py 3 0.5      # three fan cycles

Builds `thermal/` with THERMAL_IDENT_TRACE, which puts a printf in
`thermal_ident.c`'s update loop and nowhere else; the firmware build never
defines it. Nothing in FINDINGS' 2026-09-05 identification entry was found
by looking at the estimate alone - it was found here, one line a sample:

    TRACE 1 e=+2.295 h=+4.889 +4.209 +2.705 +0.000  s=1.82 1.10 1.00 1.00

`e` is the innovation in kelvin on channel 0 NTC, 1 MCU die, 2 AFE die;
`h` its sensitivity to air, capacity, spread, ntc; `s` the scales after.
"""
import ctypes
import os
import subprocess
import sys

HOST = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(HOST, 'tests'))
os.chdir(HOST)

import test_modbus_core as mb                      # noqa: E402
import test_thermal_core as tc                     # noqa: E402


def build_traced():
    """The core as the suite builds it, plus the trace define."""
    cc = tc.find_cc()
    if cc is None:
        sys.exit('no host C compiler; setup.ps1 installs one')
    # Where the suites put theirs, so nothing new needs ignoring.
    os.makedirs(mb.OUT, exist_ok=True)
    lib_path = os.path.join(mb.OUT, 'thermaltrace'
                            + ('.dll' if os.name == 'nt' else '.so'))
    cmd = ([cc, '-shared', '-o', lib_path] + mb.FLAGS
           + ['-DTHERMAL_IDENT_TRACE'] + tc.SOURCES
           + ['-I', os.path.join(tc.THERMAL, 'inc')])
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode:
        sys.exit(done.stderr)
    return ctypes.CDLL(lib_path)


def main(argv):
    cycles = int(argv[1]) if len(argv) > 1 else 3
    air = float(argv[2]) if len(argv) > 2 else 2.0
    lib = build_traced()
    truth = tc.GroundTruth(lib, air=air)
    observer = tc.Model(lib)
    ident = tc.Ident(lib, observer)
    watt = tc.power(lib, phase_sq=(900.0, 900.0, 900.0),
                    duty=(0.5, 0.5, 0.5), link_volts=48.0, switching=True)
    sys.stdout.flush()
    for cycle in range(cycles):
        truth.cycle(ident, watt, 600.0, 1200.0)
        sys.stdout.flush()
        print('CYCLE %d: %s scales %s sigma %s innovation %.2f K' % (
            cycle, ident.state(),
            ['%.2f' % ident.scale(s) for s in tc.SCALES],
            ['%.2f' % ident.sigma(s) for s in tc.SCALES],
            ident.innovation()))
        sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
