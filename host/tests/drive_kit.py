"""The drive core built for this host with the firmware's flags, and a suite run on it."""
import ctypes
import os
import sys

from tools.cores.build import build, find_cc
from tools.cores.drive import DRIVE, SOURCES
from tools.dev.focus import chosen

from test_modbus_core import Report


def suite(roster, name, argv=None):
    """`roster`'s tests - those the command line's words name, or its --shard k/n - on the core
    built as `name`: a library each suite, the relay running them side by side. 1 on a failure."""
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(DRIVE, 'inc')], name=name)
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('drive/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in chosen(roster, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0
