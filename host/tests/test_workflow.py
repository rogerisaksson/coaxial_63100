#!/usr/bin/env python3
"""The gate as GitHub runs it, held here: its one workflow's jobs take what they need from outside kept
(.github/actions/inputs), the tools pinned (.github/inputs.txt); the runner's image here is of its host job
(`tools.dev.linux`); the robot's C built for its target (`tools.cores.build.TARGETS`); its suites split in parts
of equal work, every suite once (`tools.dev.machine_detector.parts`); and the last commit each job and the gate
whole left green named as the reference (`ci.covered`, `ci.newest`, `ci.whole`, `ci.behind`, `ci.said`)."""
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_the_jobs_take_their_inputs_kept(report):
    """.github/inputs.txt pins each tool a job takes from outside: a row a tool, the sha256 of what its
    source serves, its program inside that, and an https archive of a kind .github/inputs.sh unpacks; a
    tool's sources of one kind name one program. The workflows running the robot's suites (the gate, the
    firmware's, the runner probe's) take their inputs through the inputs
    action (.github/actions/inputs, kept in the Actions cache) and fetch nothing themselves - apt's
    mirrors not resolving had failed the host's gate before a suite ran, three pushes running. Every
    fetcher's fallbacks are checked by the inputs workflow, run when the fetcher changes."""
    root = pathlib.Path(__file__).resolve().parents[2] / '.github'
    rows = [line.split() for line in (root / 'inputs.txt').read_text(encoding='utf-8').splitlines()
            if line.strip() and not line.startswith('#')]
    kinds = ('.zip', '.tar.gz', '.deb')
    bad = [row for row in rows if len(row) != 4 or not re.fullmatch('[0-9a-f]{64}', row[1])
           or not row[3].startswith('https://') or not row[3].endswith(kinds)]
    report.check('every pinned source: a tool, its sha256, its program, an https archive', rows and not bad,
                 str(bad[:2]))
    programs = {}
    for tool, _sum, program, source in (row for row in rows if len(row) == 4 and row[3].endswith(kinds)):
        programs.setdefault((tool, next(k for k in kinds if source.endswith(k))), set()).add(program)
    report.check("a tool's sources of one kind name one program",
                 all(len(names) == 1 for names in programs.values()), str(programs))
    raw = [name for name in ('gate.yml',)
           if './.github/actions/inputs' not in (text := (root / 'workflows' / name).read_text(encoding='utf-8'))
           or re.search(r'apt-get|curl |pip install', text)]
    report.check('every job running the robot\'s suites takes its inputs kept, fetching nothing itself', not raw, str(raw))
    health = (root / 'workflows' / 'inputs.yml').read_text(encoding='utf-8')
    watched = re.findall(r'paths: \[([^\]]*)\]', health)
    unchecked = [name for name in ('inputs.sh', 'apt.sh')
                 if not watched or any("'.github/%s'" % name not in paths for paths in watched)
                 or not re.search(r'run: bash \.github/%s --every' % re.escape(name), health)]
    report.check("every fetcher's fallbacks checked, whenever it changes", not unchecked, str(unchecked))
    jobs = re.findall(r'runs-on: (\S+)\n(?:(?!    runs-on).)*?run: bash \.github/apt\.sh --every', health, re.S)
    report.check("apt's fallbacks checked on both machines it serves: the archive's on amd64, the ports' on arm64",
                 'ubuntu-latest' in jobs and 'ubuntu-24.04-arm' in jobs, str(jobs))


def test_the_runner_here_is_gate_yml_s(report):
    """`tools.dev.linux` on gate.yml and on engines laid here: its image is of the release, the Pythons, the apt
    packages and the environment gate.yml's host job names; a container takes whole cores to the runner's four
    processors - two of two threads, four of one -, apart from the first core and from each other, one set where
    the engine has too few, none where it holds fewer than four; and this host counts what its group is given - the
    system's own count too (glibc's, os.cpu_count): a container shown its processors online as Linux writes them,
    an LF alone (with a CR glibc read the engine's 64)."""
    from tools.dev import focus, linux
    info = linux.workflow()
    report.check("gate.yml's host job's own: an Ubuntu release, its matrix, its apt packages, its environment and folder",
                 bool(info['release']) and bool(info['pythons'])
                 and all(re.fullmatch(r'\d+\.\d+', minor) for minor in info['pythons'])
                 and 'libegl1' in info['apt'] and 'MUJOCO_GL' in info['env'] and info['directory'] == 'host', str(info))
    smt, single = [0b11 << 2 * k for k in range(32)], [1 << k for k in range(8)]
    took = [linux.shaped(smt, 2), linux.shaped(single, 2), linux.shaped(smt[:2], 2), linux.shaped(smt[:1], 1)]
    report.check('two cores of two threads a container, apart; four of one, the first core last; one set of two '
                 'cores; none of one core', took == [[0x3c, 0x3c0], [0x1e, 0xe1], [0xf], []], str(took))
    told = focus.counted()
    report.check("this host's count: its control group's where Linux tells one, the batch sized by the same",
                 told is None or told == (focus.physical_cores(), focus.threads()), str(told))
    report.check("and the system's own count the same: os.cpu_count, its group's processors online",
                 told is None or os.cpu_count() == told[1], '%s of %s' % (os.cpu_count(), told))
    action = (linux.REPO / '.github' / 'actions' / 'inputs' / 'action.yml').read_text(encoding='utf-8')
    image = linux.DOCKERFILE.read_text(encoding='utf-8')
    held = [bool(info['constraints']) and (linux.REPO / str(info['constraints'])).is_file(),
            '${CONSTRAINTS:+-c "$CONSTRAINTS"}' in action, '${CONSTRAINTS:+"$CONSTRAINTS"}' in action,
            '-c /tmp/inputs/constraints.txt' in image]
    report.check("gate.yml's environment held to its lock: the file there, CI installing through it and keying its "
                 "cached environment by it, the runner's image installing through it", all(held), held)
    arm = linux.environment(info, 'arm64', 'sme2', 'max,sve-default-vector-length=64')
    own = linux.environment(info)
    report.check("an arm64 container: gate.yml's environment without its x86 keys, the robot's target and QEMU's processor "
                 "named; the runner's as gate.yml's; each image its own tag, the arch the image's build is told",
                 not set(linux.X86_ENV) & set(arm) - {'COAXIAL_ISA'} and arm['COAXIAL_ISA'] == 'sme2'
                 and arm['QEMU_CPU'] == 'max,sve-default-vector-length=64' and own == info['env']
                 and linux.tagged(info, '3.12') != linux.tagged(info, '3.12', 'arm64') and '"${ARCH}"' in image,
                 '%s | %s' % (arm, linux.tagged(info, '3.12', 'arm64')))


def test_the_gate_builds_the_robots_c_for_its_target(report):
    """gate.yml's host job names the robot's cores' target (COAXIAL_ISA), one of `build.TARGETS`, its flags those
    `native` gives under it, nothing contracted; a name none of them refused; unset, this host's own
    (-march=native). And the probe (isa.c) says of this processor what numpy finds where nothing disables its
    sets: AVX on x86-64-v2, AVX2 as x86-64-v3."""
    from tools.cores import build
    from tools.dev import linux
    found = set(build._host().split(','))
    off = set(os.environ.get('NPY_DISABLE_CPU_FEATURES', '').replace(',', ' ').split())
    probed = {isa: (build.held(isa), needs <= found) for isa, needs in
              (('avx', {'X86_V2', 'AVX'}), ('avx2', {'X86_V3'})) if not needs & off}
    report.check("the probe's targets numpy's sets", all(a == b for a, b in probed.values()), str(probed))
    isa, kept = linux.workflow()['env'].get('COAXIAL_ISA'), os.environ.pop('COAXIAL_ISA', None)
    try:
        os.environ['COAXIAL_ISA'] = str(isa)
        given = build.native() if isa in build.TARGETS and build.held(isa) else None
        os.environ['COAXIAL_ISA'] = 'none'
        try:
            build.native()
            refused = False
        except RuntimeError:
            refused = True
        del os.environ['COAXIAL_ISA']
        own = build.native()
    finally:
        os.environ.pop('COAXIAL_ISA', None)
        if kept is not None:
            os.environ['COAXIAL_ISA'] = kept
    report.check("the gate's target one of the robot's targets, its flags given under it, nothing contracted",
                 isa in build.TARGETS and given in (None, ('-ffp-contract=off',) + build.TARGETS[isa]),
                 '%s: %s' % (isa, given))
    report.check('a name none of the robot\'s targets refused; unset, this host\'s own', refused
                 and own[0] == '-ffp-contract=off' and all(flag.endswith('=native') for flag in own[1:]), str(own))


def test_the_gate_s_parts_cover_its_suites(report):
    """`machine_detector.parts`: the offline suites in n parts, each suite in one part in the given order, the parts'
    work apart by one suite's at most, the same split asked twice. And gate.yml's workflow job (`workflow.plan`): its defaults
    stated once, twenty machines every one taken - the host's parts a Python what the firmware's presets and the arch's
    targets leave -, the matrices and --part from its outputs; fewer machines fewer parts, one Python all left."""
    from tools.dev import machine_detector, suites, workflow
    offline = [s for s in suites.DEFAULT_SUITES if s not in suites.NEEDS_BOARD]
    work = machine_detector.table()
    heaviest = max(work.get(s, 0.0) for s in offline)
    held = []
    for n in (1, 8, 20):
        parts = machine_detector.parts(offline, n)
        loads = [sum(work.get(s, 0.0) for s in part) for part in parts]
        held.append(len(parts) == n and sorted(sum(parts, [])) == sorted(offline)
                    and all(part == [s for s in offline if s in part] for part in parts)
                    and max(loads) - min(loads) <= heaviest and parts == machine_detector.parts(offline, n))
    report.check('each suite in one of n parts, in order, the parts within one suite of each other, the split the same '
                 'asked twice', all(held), '%s; %d parts %s' % (held, n, [round(l) for l in loads]))
    gate = (pathlib.Path(__file__).resolve().parents[2] / '.github' / 'workflows' / 'gate.yml').read_text(encoding='utf-8')
    told = dict(re.findall(r"^\s+(MACHINES|PYTHONS|PRESETS|ISAS): \$\{\{ inputs\.\w+ \|\| '([^']*)' \}\}", gate, re.M))
    laid = workflow.plan(told.get('MACHINES', 0), told.get('PYTHONS', ''), told.get('PRESETS', ''), told.get('ISAS', ''))
    wired = ('--part ${{ matrix.part }}/${{ needs.workflow.outputs.n }}', 'python: ${{ fromJSON(needs.workflow.outputs.pythons) }}',
             'part: ${{ fromJSON(needs.workflow.outputs.parts) }}', 'preset: ${{ fromJSON(needs.workflow.outputs.presets) }}',
             'include: ${{ fromJSON(needs.workflow.outputs.arch) }}', 'job: ${{ fromJSON(needs.workflow.outputs.probe) }}')
    report.check("gate.yml's workflow job: its defaults stated once, twenty machines every one taken, the matrices and --part "
                 "from its outputs", len(told) == 4 and laid['jobs'] == int(told.get('MACHINES', 0)) == 20 and laid['n'] == 8
                 and all(w in gate for w in wired), '%s -> n %s, %s jobs' % (told, laid['n'], laid['jobs']))
    fewer = workflow.plan(10, '3.12', 'Release', 'sve2')
    report.check('fewer machines fewer parts, one Python all that are left: 10 machines, Release and sve2 -> 8 parts of one '
                 'Python, 10 jobs; 3 machines -> one part', fewer['n'] == 8 and fewer['jobs'] == 10
                 and fewer['arch'] == [{'runner': 'ubuntu-24.04-arm', 'isa': 'sve2'}]
                 and workflow.plan(3, '3.12 3.14', 'Debug Release', 'sve2 avx512')['n'] == 1, str(fewer))


def test_the_last_green_commit_is_the_reference(report):
    """`ci.whole`: the newest commit green in every workflow's list, the lists newest first; `ci.behind`: how
    many commits HEAD is past it, None for one not here; `ci.said`: a line a name - its commit, that distance
    and its subject -, 'none green' where none ended green; `ci.main` on another word: usage, 2."""
    import subprocess
    from tools.dev import ci
    host = [('c', '3', 3), ('b', '2', 2), ('a', '1', 1)]
    firmware = [('d', '4', 4), ('b', '2', 2), ('a', '1', 1)]
    report.check('the newest commit green in both lists, each newest first; none where they share none',
                 ci.whole(host, firmware) == ('b', '2', 2) and ci.whole(host, [('z', '0', 0)]) is None
                 and ci.whole(host) == host[0] and ci.whole([], firmware) is None, ci.whole(host, firmware))
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, cwd=ci.ROOT).stdout.strip()
    report.check('HEAD is 0 behind itself, its parent 1; a commit not here None',
                 ci.behind(head) == 0 and ci.behind(head + '~1') == 1 and ci.behind('0' * 40) is None, ci.behind(head))
    lines = ci.said({'host': (head, '', 1), 'gate': None})
    report.check("a line a name: the commit, how far HEAD is past it, its subject; 'none green' where none",
                 len(lines) == 2 and head[:8] in lines[0] and '0 commits behind HEAD' in lines[0]
                 and lines[1].startswith('ci: gate') and 'none green' in lines[1], lines)
    jobs = [{'name': 'host (3.12, 1)', 'status': 'completed', 'conclusion': 'success'},
            {'name': 'host (3.14, 1)', 'status': 'completed', 'conclusion': 'failure'},
            {'name': 'firmware (Release)', 'status': 'completed', 'conclusion': 'success'},
            {'name': 'arch (ubuntu-latest, avx512)', 'status': 'completed', 'conclusion': 'cancelled'}]
    report.check("a job green in a run where each of its name ended green, one at least; the newest run by its time",
                 not ci.covered('host', jobs) and ci.covered('firmware', jobs) and not ci.covered('arch', jobs)
                 and not ci.covered('probe', jobs) and ci.newest(None, ('a', '1', 1), ('b', '2', 2)) == ('b', '2', 2)
                 and ci.newest(None) is None, str([ci.covered(j, jobs) for j in ci.JOBS]))
    report.check('another word: usage, 2; the gate one workflow, its jobs host, firmware and arch',
                 ci.main(['--red']) == 2 and ci.WORKFLOW == 'gate.yml' and tuple(ci.JOBS) == ('host', 'firmware', 'arch'))


ROSTER = (test_the_jobs_take_their_inputs_kept, test_the_runner_here_is_gate_yml_s,
          test_the_gate_builds_the_robots_c_for_its_target, test_the_gate_s_parts_cover_its_suites,
          test_the_last_green_commit_is_the_reference)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
