#!/usr/bin/env python3
"""Line coverage of the hand-written code, off the offline suites.

    python tools/dev/cover.py              # run, then the table
    python tools/dev/cover.py --files      # and every file under 100 %
    python tools/dev/cover.py --report     # the last run's table, no run

The host's Python under coverage.py, followed into every suite's process
(`[tool.coverage.run]`); the portable C cores under gcov, built with
COAXIAL_GCOV by `tools.cores.build`. Not counted: the CubeMX code - core/,
startup_*.s, cmake/stm32cubemx/, the HAL - which is generated, and board/ and
comms/, which run on the target only and are the bench's conformance suite's.
"""
import argparse
import glob
import json
import os
import subprocess
import sys

from tools import REPO
from tools.cores.build import OUT
from tools.dev.suites import ROOT

WHERE = os.path.join(REPO, 'build', 'coverage')
PY_DATA = os.path.join(WHERE, 'python.cov')
PY_JSON = os.path.join(WHERE, 'python.json')
C_JSON = os.path.join(WHERE, 'c.json')

#: The portable cores, as their directories under the repo.
CORES = ('modbus', 'drive', 'thermal', 'filter', 'daq', 'shtp', 'boot', 'ctrl')


def run():
    """The offline suites under coverage, the cores under gcov; the exit code."""
    os.makedirs(WHERE, exist_ok=True)
    for stale in glob.glob(os.path.join(OUT, '*.gcda')) + glob.glob(PY_DATA + '*'):
        os.remove(stale)
    env = dict(os.environ, COVERAGE_FILE=PY_DATA, COAXIAL_GCOV='1')
    done = subprocess.run([sys.executable, '-m', 'coverage', 'run',
                           os.path.join('tools', 'dev', 'run_tests.py'), '--offline'],
                          cwd=ROOT, env=env)
    subprocess.run([sys.executable, '-m', 'coverage', 'combine', '-q'], cwd=ROOT, env=env,
                   check=True)
    subprocess.run([sys.executable, '-m', 'coverage', 'json', '-q', '-o', PY_JSON],
                   cwd=ROOT, env=env, check=True)
    with open(C_JSON, 'w', encoding='utf-8') as f:
        json.dump(c_lines(), f)
    return done.returncode


def c_lines():
    """{source: [lines, covered]} for every core source a suite built, the union
    over the libraries that compiled it."""
    hit = {}
    for gcda in glob.glob(os.path.join(OUT, '*.gcda')):
        got = subprocess.run(['gcov', '--json-format', '--stdout', os.path.basename(gcda)],
                             cwd=OUT, capture_output=True, text=True, encoding='utf-8',
                             errors='replace')
        for line in got.stdout.splitlines():
            if not line.startswith('{'):
                continue
            for record in json.loads(line)['files']:
                source = os.path.relpath(os.path.join(REPO, record['file']), REPO)
                if source.replace('\\', '/').split('/')[0] not in CORES:
                    continue
                seen = hit.setdefault(source.replace('\\', '/'), {})
                for entry in record['lines']:
                    at = entry['line_number']
                    seen[at] = seen.get(at, 0) + entry['count']
    return {source: [len(lines), sum(1 for n in lines.values() if n)]
            for source, lines in hit.items()}


def python_lines():
    """{file under host/: [statements, covered]} off the last run."""
    with open(PY_JSON, encoding='utf-8') as f:
        files = json.load(f)['files']
    return {name.replace('\\', '/'): [data['summary']['num_statements'],
                                      data['summary']['covered_lines']]
            for name, data in files.items()}


def table(files=False):
    """The report: each part, its lines and the share covered; with `files`, every
    file under 100 %, least covered first."""
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    py = python_lines()
    groups = {}
    for name, (lines, covered) in py.items():
        part = name.split('/')[0]
        got = groups.setdefault(('python', part), [0, 0])
        got[0] += lines
        got[1] += covered
    for name, (lines, covered) in c.items():
        got = groups.setdefault(('c', name.split('/')[0]), [0, 0])
        got[0] += lines
        got[1] += covered
    print('%-8s %-16s %7s %7s %7s' % ('', 'part', 'lines', 'covered', 'share'))
    for (kind, part), (lines, covered) in sorted(groups.items()):
        print('%-8s %-16s %7d %7d %6.1f%%' % (kind, part, lines, covered,
                                              100.0 * covered / max(1, lines)))
    for kind in ('python', 'c'):
        lines = sum(v[0] for (k, _p), v in groups.items() if k == kind)
        covered = sum(v[1] for (k, _p), v in groups.items() if k == kind)
        print('%-8s %-16s %7d %7d %6.1f%%' % (kind, 'all', lines, covered,
                                              100.0 * covered / max(1, lines)))
    if files:
        short = sorted(((covered / max(1, lines), name, lines, covered)
                        for name, (lines, covered) in list(py.items()) + list(c.items())
                        if covered < lines))
        print()
        for share, name, lines, covered in short:
            print('%6.1f%%  %5d of %5d  %s' % (100.0 * share, covered, lines, name))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--files', action='store_true', help='every file under 100 %%')
    parser.add_argument('--report', action='store_true', help="the last run's table only")
    args = parser.parse_args(argv)
    code = 0 if args.report else run()
    table(args.files)
    return code


if __name__ == '__main__':
    sys.exit(main())
