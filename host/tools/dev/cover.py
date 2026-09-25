#!/usr/bin/env python3
"""Line coverage of the hand-written code, off the offline suites.

    python tools/dev/cover.py              # run, then the table
    python tools/dev/cover.py --files      # and every file under 100 %
    python tools/dev/cover.py --report     # the last run's table, no run
    python tools/dev/cover.py --readme     # and the table into README.md's section

The host's Python under coverage.py, followed into every suite's process
(`[tool.coverage.run]`); the portable C cores under gcov, built with
COAXIAL_GCOV by `tools.cores.build`, and comms/ with them - built for this host over the
fake board (tools.cores.fakeboard). Not counted: the CubeMX code - core/, startup_*.s,
cmake/stm32cubemx/, the HAL - which is generated; board/, which runs on the target only
and is the bench's conformance suite's, and its fake, which is scaffolding. tools/ is
shown apart, a folder a line, each with what runs it (TOOLS), and left out of the totals:
the product is the rest.
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
CORES = ('modbus', 'drive', 'thermal', 'filter', 'daq', 'shtp', 'boot', 'ctrl', 'comms')

#: tools/ by folder, and what runs each: a folder not named here is run by hand.
TOOLS = {'tools/bench': 'a board', 'tools/target': 'a board', 'tools/thermal': 'a board',
         'tools/notebooks': 'the papers, as notebooks', 'tools/cores': 'the core suites',
         'tools/dev': 'the gate and by hand'}

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
    """{source: [lines, covered, missing lines]} for every core source a suite built, the
    union over the libraries that compiled it."""
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
    return {source: [len(lines), sum(1 for n in lines.values() if n),
                     sorted(at for at, n in lines.items() if not n)]
            for source, lines in hit.items()}


def python_lines():
    """{file under host/: [statements, covered, missing lines]} off the last run."""
    with open(PY_JSON, encoding='utf-8') as f:
        files = json.load(f)['files']
    return {name.replace('\\', '/'): [data['summary']['num_statements'],
                                      data['summary']['covered_lines'], data['missing_lines']]
            for name, data in files.items()}


def parts():
    """[(kind, part, lines, covered)], each file under its part - a top package, a
    folder of tools, a core."""
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    groups = {}
    for kind, files in (('python', python_lines()), ('c', c)):
        for name, (lines, covered, *_missing) in files.items():
            bits = name.split('/')
            part = '/'.join(bits[:2]) if bits[0] == 'tools' and len(bits) > 2 else bits[0]
            part = 'tools/dev' if part == 'tools' else part
            got = groups.setdefault((kind, part), [0, 0])
            got[0] += lines
            got[1] += covered
    return [(kind, part, lines, covered) for (kind, part), (lines, covered)
            in sorted(groups.items())]


def _share(lines, covered):
    return 100.0 * covered / max(1, lines)


def _ranges(lines):
    """[3, 4, 5, 9] as '3-5, 9'."""
    out, start = [], None
    for i, at in enumerate(lines):
        if start is None:
            start = at
        if i + 1 == len(lines) or lines[i + 1] != at + 1:
            out.append(str(start) if start == at else '%d-%d' % (start, at))
            start = None
    return ', '.join(out)


def _tool(part):
    return part.startswith('tools/')


def _runner(part):
    return TOOLS.get(part, 'by hand')


def table(files=False):
    """The report: each part, its lines and the share covered, tools/ apart; with
    `files`, every file under 100 %, least covered first."""
    rows = parts()
    print('%-8s %-16s %7s %7s %7s' % ('', 'part', 'lines', 'covered', 'share'))
    for kind, part, lines, covered in rows:
        print('%-8s %-16s %7d %7d %6.1f%%%s' % (kind, part, lines, covered,
                                                _share(lines, covered),
                                                '  (%s)' % _runner(part)
                                                if _tool(part) else ''))
    for kind in ('python', 'c'):
        mine = [r for r in rows if r[0] == kind and not _tool(r[1])]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        print('%-8s %-16s %7d %7d %6.1f%%' % (kind, 'all', lines, covered,
                                              _share(lines, covered)))
    if files:
        with open(C_JSON, encoding='utf-8') as f:
            every = list(python_lines().items()) + list(json.load(f).items())
        print()
        for share, name, lines, covered, missing in sorted(
                (row[1] / max(1, row[0]), name, row[0], row[1], row[2] if len(row) > 2 else [])
                for name, row in every if row[1] < row[0]):
            print('%6.1f%%  %5d of %5d  %s  %s' % (100.0 * share, covered, lines, name,
                                                   _ranges(missing)))


def markdown():
    """The tables as the README shows them: the product's code with its totals, then
    tools/ by folder with what runs each."""
    rows = parts()
    out = ['| Product | Lines | Covered |', '| --- | ---: | ---: |']
    for kind, label in (('python', 'Python'), ('c', 'C, portable cores')):
        mine = [r for r in rows if r[0] == kind and not _tool(r[1])]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        out.append('| **%s** | %d | **%.1f %%** |' % (label, lines, _share(lines, covered)))
        out += ['| %s | %d | %.1f %% |' % (part, lines, _share(lines, covered))
                for k, part, lines, covered in mine if k == kind]
    out += ['', '| Tools | Lines | Offline | Run by |', '| --- | ---: | ---: | --- |']
    out += ['| %s | %d | %.1f %% | %s |' % (part, lines, _share(lines, covered), _runner(part))
            for _k, part, lines, covered in rows if _tool(part)]
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
