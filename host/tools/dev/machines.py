#!/usr/bin/env python3
"""The gate's machines: how many each target takes at once, the host's suites in parts over the rest.

    python3 tools/dev/machines.py 20 '3.12 3.14' 'Debug Release' 'sve2 avx512'   # KEY=json lines, the plan job's outputs

GitHub runs twenty jobs of a public repository at once and queues the rest, so the gate plans twenty machines a
run (gate.yml, its plan job): the firmware's presets take one each, the robot's C targets one each, and the
host's suites split in as many parts a Python as are left (run_tests.py --part k/n, machine_detector.parts) -
sixteen runners of the twenty for two Pythons, eight parts each. Asked by hand with fewer machines, fewer parts;
with one Python, twice as many. Plain Python: the plan job runs it before anything is installed.
"""
import json
import sys

#: The runner each of the robot's C targets is built and held on: SVE2 on GitHub's arm64 runner (Cobalt 100,
#: Neoverse N2), AVX-512 where the x86 runner GitHub gives has it (the job says so where not).
RUNNERS = {'sve2': 'ubuntu-24.04-arm', 'avx512': 'ubuntu-latest'}


def plan(machines, pythons, presets, isas):
    """{'pythons', 'presets': their lists; 'arch': [{'runner', 'isa'}] of the targets; 'n': the host's parts a
    Python - what is left of `machines` after the presets and the targets, shared by the Pythons, one at least -;
    'parts': 1 to n; 'jobs': the run's machines}."""
    pythons, presets, isas = [list(filter(None, str(w).split())) for w in (pythons, presets, isas)]
    n = max(1, (int(machines) - len(presets) - len(isas)) // max(1, len(pythons)))
    return {'pythons': pythons, 'presets': presets, 'arch': [{'runner': RUNNERS[isa], 'isa': isa} for isa in isas],
            'n': n, 'parts': list(range(1, n + 1)), 'jobs': n * len(pythons) + len(presets) + len(isas)}


def main(argv=None):
    """The plan's outputs as GitHub's output file takes them, KEY=json a line, and a line saying the plan."""
    words = sys.argv[1:] if argv is None else list(argv)
    if len(words) != 4:
        print('usage: machines.py MACHINES PYTHONS PRESETS ISAS', file=sys.stderr)
        return 2
    laid = plan(*words)
    for key, value in laid.items():
        print('%s=%s' % (key, json.dumps(value)))
    print('::notice::%d machines: host %d Pythons x %d parts, firmware %d, arch %d - %d jobs' % (
        int(words[0]), len(laid['pythons']), laid['n'], len(laid['presets']), len(laid['arch']), laid['jobs']), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
