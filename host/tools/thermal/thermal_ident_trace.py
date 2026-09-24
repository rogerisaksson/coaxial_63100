"""Trace the thermal identification sample by sample against `tools/cores/thermal.py`'s ground truth.

Every innovation with its regressor, and the scales after each update.

    python tools/thermal/thermal_ident_trace.py            # three box cycles, air 2.0
    python tools/thermal/thermal_ident_trace.py 3 0.5      # three fan cycles

Builds `thermal/` with THERMAL_IDENT_TRACE, which puts a printf in
`thermal_ident.c`'s update loop and nowhere else; the firmware build never
defines it. FINDINGS' 2026-09-05 identification entry came from this trace,
not from the estimate, one line a sample:

    TRACE 1 e=+2.295 h=+4.889 +4.209 +2.705 +0.000  s=1.82 1.10 1.00 1.00

`e` is the innovation in kelvin on channel 0 NTC, 1 MCU die, 2 AFE die;
`h` its sensitivity to air, capacity, spread, ntc; `s` the scales after.
"""
import argparse
import ctypes
import os
import sys

from tools.cores.build import build, find_cc
from tools.cores.thermal import SCALES, SOURCES, THERMAL, GroundTruth, Ident, Model, power


def build_traced():
    """The core as the suite builds it, plus the trace define."""
    cc = find_cc()
    if cc is None:
        sys.exit('no host C compiler; setup.ps1 installs one')
    # build() passes its sources to the compiler verbatim and has no defines.
    lib_path, _warnings = build(cc, ['-DTHERMAL_IDENT_TRACE'] + SOURCES,
                                [os.path.join(THERMAL, 'inc')], 'thermaltrace')
    return ctypes.CDLL(lib_path)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('cycles', nargs='?', type=int, default=3)
    p.add_argument('air', nargs='?', type=float, default=2.0)
    a = p.parse_args()
    lib = build_traced()
    truth = GroundTruth(lib, air=a.air)
    observer = Model(lib)
    ident = Ident(lib, observer)
    watt = power(lib, phase_sq=(900.0, 900.0, 900.0),
                 duty=(0.5, 0.5, 0.5), link_volts=48.0, switching=True)
    sys.stdout.flush()
    for cycle in range(a.cycles):
        truth.cycle(ident, watt, 600.0, 1200.0)
        sys.stdout.flush()
        print('CYCLE %d: %s scales %s sigma %s innovation %.2f K' % (
            cycle, ident.state(),
            ['%.2f' % ident.scale(s) for s in SCALES],
            ['%.2f' % ident.sigma(s) for s in SCALES],
            ident.innovation()))
        sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
