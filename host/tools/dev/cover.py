#!/usr/bin/env python3
"""Line coverage of the hand-written code, off the offline suites.

    python tools/dev/cover.py              # run, then the table
    python tools/dev/cover.py --files      # and every file under 100 %
    python tools/dev/cover.py --report     # the last run's table, no run
    python tools/dev/cover.py --readme     # and the table into README.md's section

The host's Python under coverage.py, followed into every suite's process
(`[tool.coverage.run]`); the portable C cores under gcov, built with
COAXIAL_GCOV by `tools.cores.build`. Not counted: the CubeMX code - core/,
startup_*.s, cmake/stm32cubemx/, the HAL - which is generated, and board/ and
comms/, which run on the target only and are the bench's conformance suite's. Parts
run elsewhere (ELSEWHERE) are shown apart and left out of the total: the bench's and the
flash tools need a board, the papers run as notebooks.
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

#: Parts the offline suites do not run, and what does.
ELSEWHERE = {'tools/bench': 'a board', 'tools/target': 'a board',
             'tools/thermal': 'a board', 'tools/notebooks': 'the papers, as notebooks'}

README = os.path.join(REPO, 'README.md')
MARKS = ('<!-- coverage -->', '<!-- /coverage -->')


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


def parts():
    """[(kind, part, lines, covered)], each file under its part - a top package, a
    folder of tools, a core."""
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    groups = {}
    for kind, files in (('python', python_lines()), ('c', c)):
        for name, (lines, covered) in files.items():
            bits = name.split('/')
            part = '/'.join(bits[:2]) if bits[0] == 'tools' and len(bits) > 2 else bits[0]
            got = groups.setdefault((kind, part), [0, 0])
            got[0] += lines
            got[1] += covered
    return [(kind, part, lines, covered) for (kind, part), (lines, covered)
            in sorted(groups.items())]


def _share(lines, covered):
    return 100.0 * covered / max(1, lines)


def table(files=False):
    """The report: each part, its lines and the share covered, the parts run elsewhere
    apart; with `files`, every file under 100 %, least covered first."""
    rows = parts()
    print('%-8s %-16s %7s %7s %7s' % ('', 'part', 'lines', 'covered', 'share'))
    for kind, part, lines, covered in rows:
        print('%-8s %-16s %7d %7d %6.1f%%%s' % (kind, part, lines, covered,
                                                _share(lines, covered),
                                                '  (%s)' % ELSEWHERE[part]
                                                if part in ELSEWHERE else ''))
    for kind in ('python', 'c'):
        mine = [r for r in rows if r[0] == kind and r[1] not in ELSEWHERE]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        print('%-8s %-16s %7d %7d %6.1f%%' % (kind, 'all', lines, covered,
                                              _share(lines, covered)))
    if files:
        with open(C_JSON, encoding='utf-8') as f:
            every = list(python_lines().items()) + list(json.load(f).items())
        print()
        for share, name, lines, covered in sorted((covered / max(1, lines), name, lines,
                                                   covered)
                                                  for name, (lines, covered) in every
                                                  if covered < lines):
            print('%6.1f%%  %5d of %5d  %s' % (100.0 * share, covered, lines, name))


def markdown():
    """The table as the README shows it: the offline parts, their totals, the rest
    apart."""
    rows = parts()
    out = ['| Code | Lines | Covered |', '| --- | ---: | ---: |']
    for kind, label in (('python', 'Python'), ('c', 'C, portable cores')):
        mine = [r for r in rows if r[0] == kind and r[1] not in ELSEWHERE]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        out.append('| **%s** | %d | **%.1f %%** |' % (label, lines, _share(lines, covered)))
        out += ['| %s | %d | %.1f %% |' % (part, lines, _share(lines, covered))
                for k, part, lines, covered in mine if k == kind]
    away = [r for r in rows if r[1] in ELSEWHERE]
    if away:
        out += ['', 'Run elsewhere, not in the totals:', '']
        out += ['- %s, %d lines, %.1f %% offline: %s' % (part, lines, _share(lines, covered),
                                                          ELSEWHERE[part])
                for _k, part, lines, covered in away]
    return out


def readme():
    """The README's coverage section, between its MARKS, rewritten."""
    with open(README, encoding='utf-8', newline='') as f:
        text = f.read()
    nl = '\r\n' if '\r\n' in text else '\n'
    head, rest = text.split(MARKS[0], 1)
    _old, tail = rest.split(MARKS[1], 1)
    with open(README, 'w', encoding='utf-8', newline='') as f:
        f.write(head + MARKS[0] + nl + nl.join(markdown()) + nl + MARKS[1] + tail)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--files', action='store_true', help='every file under 100 %%')
    parser.add_argument('--report', action='store_true', help="the last run's table only")
    parser.add_argument('--readme', action='store_true', help="the table into README.md")
    args = parser.parse_args(argv)
    code = 0 if args.report else run()
    table(args.files)
    if args.readme:
        readme()
    return code


if __name__ == '__main__':
    sys.exit(main())
