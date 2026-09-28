#!/usr/bin/env python3
"""The offline suites' checks and the line coverage, host and target, off one run.

    python tools/dev/cover.py              # run, then the tables
    python tools/dev/cover.py --files      # and every file under 100 %
    python tools/dev/cover.py --report     # the last run's tables, no run
    python tools/dev/cover.py --readme     # and the tables into README.md's section

Two totals. The host: host/'s Python under coverage.py, followed into every suite's process
(`[tool.coverage.run]`), and the emulation's own C - the world, the chip native:// runs over,
the fake board. The target: the firmware's hand-written C - the portable cores, comms/ and
board/src - under gcov, built with COAXIAL_GCOV by `tools.cores.build` as native:// and the fake
board build it for this host; the files no host build compiles counted too, their executable
lines as the ARM toolchain's gcov counts them, none covered here (the bench's conformance suite
is theirs). Not counted: ST's generated code - core/, startup_*.s, cmake/stm32cubemx/, the HAL.
"""
import argparse
import ast
import datetime
import glob
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
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
    c = c_lines()
    c.update(target_only(c))
    with open(C_JSON, 'w', encoding='utf-8') as f:
        json.dump(c, f)
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


def _gcov_json(gcov, notes, cwd):
    """[(source, {line: count})] out of gcov's JSON for `notes` (a .gcda or .gcno)."""
    got = subprocess.run([gcov, '--json-format', '--stdout', notes], cwd=cwd,
                         capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = []
    for line in got.stdout.splitlines():
        if line.startswith('{'):
            for record in json.loads(line)['files']:
                source = os.path.relpath(os.path.join(cwd, record['file']), REPO)
                out.append((source.replace('\\', '/'),
                            {entry['line_number']: entry['count'] for entry in record['lines']}))
    return out


def c_lines():
    """{source: [lines, covered, missing lines]} for every hand-written source a suite built,
    the union over the libraries that compiled it."""
    hit = {}
    for gcda in glob.glob(os.path.join(OUT, '*.gcda')):
        for source, counts in _gcov_json('gcov', os.path.basename(gcda), OUT):
            if _area(source) not in FIRMWARE + EMULATION:
                continue
            seen = hit.setdefault(source, {})
            for at, n in counts.items():
                seen[at] = seen.get(at, 0) + n
    return {source: [len(lines), sum(1 for n in lines.values() if n),
                     sorted(at for at, n in lines.items() if not n)]
            for source, lines in hit.items()}


def _arm_gcc():
    """arm-none-eabi-gcc: on PATH, or the STM32Cube bundle's; None without one."""
    found = shutil.which('arm-none-eabi-gcc')
    if found:
        return found
    bundles = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'stm32cube', 'bundles')
    built = sorted(glob.glob(os.path.join(bundles, 'gnu-tools-for-stm32', '*', 'bin',
                                          'arm-none-eabi-gcc*')))
    return built[-1] if built else None


def target_only(c):
    """{source: [lines, 0, every line]} for the firmware's files no host build compiled: each
    compiled as the Debug build compiles it with -ftest-coverage by the ARM toolchain, its
    lines as that toolchain's gcov counts them - none executed here. {} without the toolchain
    or the build."""
    every = []
    for area in FIRMWARE:
        where = (os.path.join(REPO, *area.split('/'), '*.c') if '/' in area
                 else os.path.join(REPO, area, 'src', '*.c'))
        every += [os.path.relpath(p, REPO).replace('\\', '/') for p in glob.glob(where)]
    alone = sorted(name for name in every if name not in c)
    gcc = _arm_gcc()
    database = os.path.join(REPO, 'build', 'Debug', 'compile_commands.json')
    if not alone or gcc is None or not os.path.exists(database):
        return {}
    gcov = re.sub(r'gcc(\.exe)?$', r'gcov\1', gcc)
    with open(database, encoding='utf-8') as f:
        entries = {os.path.relpath(e['file'], REPO).replace('\\', '/'): e for e in json.load(f)}
    out = {}
    with tempfile.TemporaryDirectory() as work:
        for name in alone:
            entry = entries.get(name)
            if entry is None:
                continue
            argv = entry.get('arguments') or shlex.split(entry['command'], posix=False)
            keep, skip = [], False
            for arg in argv[1:]:
                if skip or arg == '-o':
                    skip = not skip
                    continue
                if arg == '-c' or arg.startswith('-O'):
                    continue
                keep.append(arg)
            obj = os.path.join(work, os.path.basename(name) + '.o')
            done = subprocess.run([gcc] + keep + ['-c', '-O0', '-ftest-coverage',
                                                  '-fprofile-arcs', '-o', obj],
                                  cwd=entry['directory'], capture_output=True, text=True)
            if done.returncode:
                continue
            for source, counts in _gcov_json(gcov, obj[:-2] + '.gcno', work):
                if source.endswith(os.path.basename(name)):
                    out[name] = [len(counts), 0, sorted(counts)]
    return out


def python_lines():
    """{file under host/: [statements, covered, missing lines]} off the last run."""
    with open(PY_JSON, encoding='utf-8') as f:
        files = json.load(f)['files']
    return {name.replace('\\', '/'): [data['summary']['num_statements'],
                                      data['summary']['covered_lines'], data['missing_lines']]
            for name, data in files.items()}


def parts():
    """[(side, kind, part, lines, covered)]: each file under its part - a top package, a folder
    of tools, a directory of C - on the host's side or the target's."""
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    groups = {}
    for name, (lines, covered, *_missing) in python_lines().items():
        bits = name.split('/')
        part = '/'.join(bits[:2]) if bits[0] == 'tools' and len(bits) > 2 else bits[0]
        part = 'tools/dev' if part == 'tools' else part
        key = ('host', 'tools' if part.startswith('tools/') else 'python', part)
        got = groups.setdefault(key, [0, 0])
        got[0] += lines
        got[1] += covered
    for name, (lines, covered, *_missing) in c.items():
        part = _area(name)
        key = ('host', 'emulation', part) if part in EMULATION else ('target', 'c', part)
        got = groups.setdefault(key, [0, 0])
        got[0] += lines
        got[1] += covered
    return [key + tuple(value) for key, value in sorted(groups.items())]


def totals(rows):
    """{'host': (lines, covered), 'target': (lines, covered)}."""
    out = {}
    for side in ('host', 'target'):
        mine = [r for r in rows if r[0] == side]
        out[side] = (sum(r[3] for r in mine), sum(r[4] for r in mine))
    return out


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
    """The report: the two totals, each suite's tally, then each part; with `files`, every file
    under 100 %, least covered first."""
    ran = _run()
    rows = parts()
    for side, (lines, covered) in totals(rows).items():
        print('%-7s %6d lines %6.1f %%' % (side, lines, _share(lines, covered)))
    print()
    for suite, passed, failed, seconds in ran['suites']:
        print('%-26s %s' % (suite, 'CRASHED' if passed is None
                            else '%5d passed %3d failed %6.1f s' % (passed, failed, seconds)))
    print('\n%-7s %-10s %-16s %7s %7s %7s' % ('', '', 'part', 'lines', 'covered', 'share'))
    for side, kind, part, lines, covered in rows:
        print('%-7s %-10s %-16s %7d %7d %6.1f%%%s'
              % (side, kind, part, lines, covered, _share(lines, covered),
                 '  (%s)' % _runner(part) if kind == 'tools' else ''))
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
    """The section as the README shows it: the two totals and the checks, then each suite and
    each part folded under them."""
    ran = _run()
    rows = parts()
    both = totals(rows)
    out = textwrap.wrap('Generated %s at `%s` by `python host/tools/dev/cover.py --readme`: one '
                        'offline run (`host/tools/dev/run_tests.py --offline`), the Python under '
                        'coverage.py, the C under gcov. ST\'s generated code - CubeMX\'s core/, '
                        'startup and cmake/stm32cubemx/, the HAL - is not counted.'
                        % (ran['date'], ran['commit']), 80)
    out += ['', '| Code | Lines | Covered |', '| --- | ---: | ---: |',
            '| **Host**: host/\'s Python, the emulation\'s C | %d | **%.1f %%** |'
            % (both['host'][0], _share(*both['host'])),
            '| **Target**: the firmware\'s C, the target-only files uncovered | %d | **%.1f %%** |'
            % (both['target'][0], _share(*both['target']))]
    if ran['total']:
        _all, passed, skipped, failed = ran['total']
        suites = sum(1 for row in ran['suites'] if row[1] is None or row[1] or row[2])
        out += [''] + textwrap.wrap('%d checks in %d suites: %d passed, %d failed; %d skipped, '
                                    'a board\'s.' % (passed + failed, suites, passed, failed,
                                                     skipped), 80)
    out += ['', '<details><summary>Each suite, each part</summary>', '',
            '| Suite | What it holds | Checks | Failed | s |',
            '| --- | --- | ---: | ---: | ---: |']
    for suite, passed, failed, seconds in ran['suites']:
        if passed is None:
            out.append('| %s | %s | crashed | | |' % (suite, _brief(suite)))
        elif passed or failed:
            out.append('| %s | %s | %d | %d | %.0f |' % (suite, _brief(suite), passed, failed,
                                                         seconds))
    out += ['', '| Host | Lines | Covered | Run by |', '| --- | ---: | ---: | --- |']
    out += ['| %s | %d | %.1f %% | %s |'
            % (part, lines, _share(lines, covered),
               _runner(part) if kind == 'tools' else 'the suites' if kind == 'python'
               else 'the emulation')
            for side, kind, part, lines, covered in rows if side == 'host']
    out += ['', '| Target | Lines | Covered |', '| --- | ---: | ---: |']
    out += ['| %s | %d | %.1f %% |' % (part, lines, _share(lines, covered))
            for side, _kind, part, lines, covered in rows if side == 'target']
    with open(C_JSON, encoding='utf-8') as f:
        c = json.load(f)
    alone = sorted(name for name, (lines, covered, *_m) in c.items()
                   if _area(name) in FIRMWARE and covered == 0 and lines)
    if alone:
        out += [''] + textwrap.wrap('Built for the target only, none of their lines run here '
                                    '(the bench\'s conformance suite is theirs): %s.'
                                    % ', '.join('`%s`' % name for name in alone), 80)
    return out + ['', '</details>']


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
