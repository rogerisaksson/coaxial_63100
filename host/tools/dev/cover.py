#!/usr/bin/env python3
"""The offline suites' checks and the line coverage of the hand-written code, off one run.

    python tools/dev/cover.py              # run, then the tables
    python tools/dev/cover.py --files      # and every file under 100 %
    python tools/dev/cover.py --report     # the last run's tables, no run
    python tools/dev/cover.py --readme     # and the tables into README.md's section

The host's Python under coverage.py, followed into every suite's process
(`[tool.coverage.run]`); the firmware's C under gcov, built with COAXIAL_GCOV by
`tools.cores.build`: the portable cores, comms/ and board/src as native:// and the fake board
build them for this host. Not counted: the CubeMX code - core/, startup_*.s,
cmake/stm32cubemx/, the HAL - which is generated. The firmware's files no host build compiles
run on the target only, the bench's conformance suite theirs, and are named. The emulation's
own C - the world, the chip native:// runs over, the fake board - and tools/ are shown apart,
each with what runs it, and left out of the totals: the product is the rest.
"""
import argparse
import ast
import datetime
import glob
import json
import os
import re
import subprocess
import sys
import textwrap

from tools import REPO
from tools.cores.build import OUT
from tools.dev.suites import ROOT

WHERE = os.path.join(REPO, 'build', 'coverage')
PY_DATA = os.path.join(WHERE, 'python.cov')
PY_JSON = os.path.join(WHERE, 'python.json')
C_JSON = os.path.join(WHERE, 'c.json')
RUN_JSON = os.path.join(WHERE, 'run.json')

#: The firmware's hand-written C, by directory: the portable cores, comms/, the board layer.
FIRMWARE = ('boot', 'comms', 'ctrl', 'daq', 'drive', 'filter', 'modbus', 'shtp', 'thermal',
            'board/src')
#: The emulation's own C: the world, the chip native:// runs over, the fake board.
EMULATION = ('world', 'board/native', 'board/fake')

#: tools/ by folder, and what runs each: a folder not named here is run by hand.
TOOLS = {'tools/bench': 'a board', 'tools/target': 'a board', 'tools/thermal': 'a board',
         'tools/notebooks': 'the papers, as notebooks', 'tools/cores': 'the core suites',
         'tools/dev': 'the gate and by hand', 'tools/emu': 'the emulator suite'}

#: The runner's line a suite: its name, checks passed, failed, seconds - or CRASHED.
SUITE_RE = re.compile(r'^(test_\w+\.py)\s+(\d+) passed, (\d+) failed\s+([\d.]+)s$')
CRASH_RE = re.compile(r'^(test_\w+\.py)\s+CRASHED')
TOTAL_RE = re.compile(r'^Total: ~?(\d+)\s+Passed: (\d+), Skipped: ~?(\d+), Failed: (\d+)')

README = os.path.join(REPO, 'README.md')
MARKS = ('<!-- coverage -->', '<!-- /coverage -->')


def run():
    """The offline suites under coverage, the C under gcov: each suite's tally kept; the exit
    code."""
    os.makedirs(WHERE, exist_ok=True)
    for stale in glob.glob(os.path.join(OUT, '*.gcda')) + glob.glob(PY_DATA + '*'):
        os.remove(stale)
    env = dict(os.environ, COVERAGE_FILE=PY_DATA, COAXIAL_GCOV='1')
    suites, total = [], None
    with subprocess.Popen([sys.executable, '-m', 'coverage', 'run',
                           os.path.join('tools', 'dev', 'run_tests.py'), '--offline'],
                          cwd=ROOT, env=env, stdout=subprocess.PIPE, text=True,
                          encoding='utf-8', errors='replace') as proc:
        for line in proc.stdout or ():
            print(line, end='', flush=True)
            line = line.rstrip()
            got, crashed, totals = SUITE_RE.match(line), CRASH_RE.match(line), TOTAL_RE.match(line)
            if got:
                suites.append([got.group(1), int(got.group(2)), int(got.group(3)),
                               float(got.group(4))])
            elif crashed:
                suites.append([crashed.group(1), None, None, None])
            elif totals:
                total = [int(v) for v in totals.groups()]
        code = proc.wait()
    subprocess.run([sys.executable, '-m', 'coverage', 'combine', '-q'], cwd=ROOT, env=env,
                   check=True)
    subprocess.run([sys.executable, '-m', 'coverage', 'json', '-q', '-o', PY_JSON],
                   cwd=ROOT, env=env, check=True)
    with open(C_JSON, 'w', encoding='utf-8') as f:
        json.dump(c_lines(), f)
    commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO,
                            capture_output=True, text=True).stdout.strip()
    with open(RUN_JSON, 'w', encoding='utf-8') as f:
        json.dump({'suites': suites, 'total': total, 'commit': commit,
                   'date': datetime.date.today().isoformat()}, f)
    return code


def _area(source):
    """A source's directory as FIRMWARE and EMULATION name them."""
    bits = source.split('/')
    return '/'.join(bits[:2]) if bits[0] == 'board' else bits[0]


def c_lines():
    """{source: [lines, covered, missing lines]} for every hand-written source a suite built,
    the union over the libraries that compiled it."""
    hit = {}
    for gcda in glob.glob(os.path.join(OUT, '*.gcda')):
        got = subprocess.run(['gcov', '--json-format', '--stdout', os.path.basename(gcda)],
                             cwd=OUT, capture_output=True, text=True, encoding='utf-8',
                             errors='replace')
        for line in got.stdout.splitlines():
            if not line.startswith('{'):
                continue
            for record in json.loads(line)['files']:
                source = os.path.relpath(os.path.join(REPO, record['file']),
                                         REPO).replace('\\', '/')
                if _area(source) not in FIRMWARE + EMULATION:
                    continue
                seen = hit.setdefault(source, {})
                for entry in record['lines']:
                    at = entry['line_number']
                    seen[at] = seen.get(at, 0) + entry['count']
    return {source: [len(lines), sum(1 for n in lines.values() if n),
                     sorted(at for at, n in lines.items() if not n)]
            for source, lines in hit.items()}


def target_only(c):
    """The firmware's C files no host build compiled: the target's alone."""
    every = []
    for area in FIRMWARE:
        every += glob.glob(os.path.join(REPO, *area.split('/'), 'src', '*.c')
                           if '/' not in area else os.path.join(REPO, *area.split('/'), '*.c'))
    names = sorted(os.path.relpath(p, REPO).replace('\\', '/') for p in every)
    return [name for name in names if name not in c]


def python_lines():
    """{file under host/: [statements, covered, missing lines]} off the last run."""
    with open(PY_JSON, encoding='utf-8') as f:
        files = json.load(f)['files']
    return {name.replace('\\', '/'): [data['summary']['num_statements'],
                                      data['summary']['covered_lines'], data['missing_lines']]
            for name, data in files.items()}


def parts():
    """[(kind, part, lines, covered)], each file under its part - a top package, a folder of
    tools, a directory of C; kind 'python', 'c' (the firmware) or 'emulation'."""
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    groups = {}
    for kind, files in (('python', python_lines()), ('c', c)):
        for name, (lines, covered, *_missing) in files.items():
            bits = name.split('/')
            if kind == 'c':
                part = _area(name)
                kind_of = 'emulation' if part in EMULATION else 'c'
            else:
                part = '/'.join(bits[:2]) if bits[0] == 'tools' and len(bits) > 2 else bits[0]
                part = 'tools/dev' if part == 'tools' else part
                kind_of = kind
            got = groups.setdefault((kind_of, part), [0, 0])
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


def _brief(suite):
    """A suite's module docstring, its first sentence."""
    path = os.path.join(ROOT, 'tests', suite)
    try:
        with open(path, encoding='utf-8') as f:
            doc = ast.get_docstring(ast.parse(f.read())) or ''
    except (OSError, SyntaxError):
        return ''
    first = ' '.join(doc.split('\n\n')[0].split())
    return re.split(r'(?<=[.?])\s', first)[0].replace('|', '/')


def _run():
    with open(RUN_JSON, encoding='utf-8') as f:
        return json.load(f)


def table(files=False):
    """The report: each suite's tally, then each part, its lines and the share covered, the
    emulation and tools/ apart; with `files`, every file under 100 %, least covered first."""
    ran = _run()
    for suite, passed, failed, seconds in ran['suites']:
        print('%-26s %s' % (suite, 'CRASHED' if passed is None
                            else '%5d passed %3d failed %6.1f s' % (passed, failed, seconds)))
    rows = parts()
    print('\n%-10s %-16s %7s %7s %7s' % ('', 'part', 'lines', 'covered', 'share'))
    for kind, part, lines, covered in rows:
        print('%-10s %-16s %7d %7d %6.1f%%%s' % (kind, part, lines, covered,
                                                 _share(lines, covered),
                                                 '  (%s)' % _runner(part)
                                                 if _tool(part) else ''))
    for kind in ('python', 'c'):
        mine = [r for r in rows if r[0] == kind and not _tool(r[1])]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        print('%-10s %-16s %7d %7d %6.1f%%' % (kind, 'all', lines, covered,
                                               _share(lines, covered)))
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    print('\ntarget only:', ', '.join(target_only(c)) or 'none')
    if files:
        every = list(python_lines().items()) + list(c.items())
        print()
        for share, name, lines, covered, missing in sorted(
                (row[1] / max(1, row[0]), name, row[0], row[1], row[2] if len(row) > 2 else [])
                for name, row in every if row[1] < row[0]):
            print('%6.1f%%  %5d of %5d  %s  %s' % (100.0 * share, covered, lines, name,
                                                   _ranges(missing)))


def markdown():
    """The section as the README shows it: the suites and their checks, the product's code
    with its totals, the firmware's target-only files, then the emulation's C and tools/."""
    ran = _run()
    out = textwrap.wrap('Generated %s at `%s` by `python host/tools/dev/cover.py --readme`: one '
                        'offline run (`host/tools/dev/run_tests.py --offline`), the Python under '
                        'coverage.py, the C under gcov.' % (ran['date'], ran['commit']), 80)
    out += ['', '| Suite | What it holds | Checks | Failed | s |',
           '| --- | --- | ---: | ---: | ---: |']
    for suite, passed, failed, seconds in ran['suites']:
        if passed is None:
            out.append('| %s | %s | crashed | | |' % (suite, _brief(suite)))
        elif passed or failed:
            out.append('| %s | %s | %d | %d | %.0f |' % (suite, _brief(suite), passed, failed,
                                                         seconds))
    if ran['total']:
        _all, passed, skipped, failed = ran['total']
        out.append('| **all** | skipped here: %d, a board\'s | **%d** | **%d** | |'
                   % (skipped, passed, failed))
    rows = parts()
    out += ['', '| Product | Lines | Covered |', '| --- | ---: | ---: |']
    for kind, label in (('python', 'Python'),
                        ('c', 'C, the firmware (CubeMX and the HAL not counted)')):
        mine = [r for r in rows if r[0] == kind and not _tool(r[1])]
        lines, covered = sum(r[2] for r in mine), sum(r[3] for r in mine)
        out.append('| **%s** | %d | **%.1f %%** |' % (label, lines, _share(lines, covered)))
        out += ['| %s | %d | %.1f %% |' % (part, lines, _share(lines, covered))
                for k, part, lines, covered in mine if k == kind]
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    alone = target_only(c)
    if alone:
        out += [''] + textwrap.wrap('The target only, the bench\'s conformance suite theirs: '
                                    '%s.' % ', '.join('`%s`' % name for name in alone), 80)
    out += ['', '| Emulation, C | Lines | Covered |', '| --- | ---: | ---: |']
    out += ['| %s | %d | %.1f %% |' % (part, lines, _share(lines, covered))
            for kind, part, lines, covered in rows if kind == 'emulation']
    out += ['', '| Tools | Lines | Offline | Run by |', '| --- | ---: | ---: | --- |']
    out += ['| %s | %d | %.1f %% | %s |' % (part, lines, _share(lines, covered), _runner(part))
            for _k, part, lines, covered in rows if _tool(part)]
    return out


def readme():
    """The README's section, between its MARKS, rewritten."""
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
    parser.add_argument('--report', action='store_true', help="the last run's tables only")
    parser.add_argument('--readme', action='store_true', help='the tables into README.md')
    args = parser.parse_args(argv)
    code = 0 if args.report else run()
    table(args.files)
    if args.readme:
        readme()
    return code


if __name__ == '__main__':
    sys.exit(main())
