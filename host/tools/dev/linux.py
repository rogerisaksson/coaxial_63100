#!/usr/bin/env python3
"""The offline gate on Linux, here: GitHub's runner as a container on this host's engine.

    python -X utf8 tools/dev/run_tests.py --linux [--python 3.12] [--parts N] [the relay's arguments]
    python -X utf8 tools/dev/run_tests.py --linux --arch arm64 --isa sme2 --cpu max,sve-default-vector-length=64
    python -X utf8 tools/dev/linux.py --build [--fresh]   # the runner's images alone; --fresh: from scratch
    python -X utf8 tools/dev/linux.py --kept              # whether each stands: 0 where all do

What .github/workflows/gate.yml's host job runs, run as that job runs on GitHub: an Ubuntu runner image
(.github/runner.Dockerfile), the tree copied in as a checkout and installed offline, and the gate invoked as
runner with CI true and --offline. Each container takes the runner's shape (two cores of two threads, 16 GB,
8 GB /dev/shm) and books two cores and BOOKED_GB in tools.dev.ledger; --parts runs multiple 2-core containers
side by side (--part k/n each, capped by host and engine memory headroom).
"""
import argparse
import io
import os
import queue
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
from contextlib import suppress

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.dev import background, census, ledger
from tools.dev.suites import ROOT

#: The tree's root, the workflow of the gate (its host job the one run here) and the image of its runner.
REPO = ROOT.parent
WORKFLOW, DOCKERFILE = REPO / '.github' / 'workflows' / 'gate.yml', REPO / '.github' / 'runner.Dockerfile'
#: GitHub's standard runner of a public repository: four logical processors - two cores of two threads, its
#: own detector line says - and 16 GB; its /dev/shm a tmpfs of half of it (8.36 GB of 16.7 on both its
#: architectures), where the robot's loop's rings are mapped (`machine.process.rcu`) - the engine's 64 MB
#: faulted test_tune (SIGBUS).
PROCESSORS, MEMORY, SHM = 4, '16g', '8g'
#: A container's commit booked in the machine's ledger (tools.dev.ledger): the reserve a relay keeps - the gate
#: inside books its own jobs in the container's ledger, and the engine's VM takes the host's commit as it goes.
BOOKED_GB = ledger.RESERVE_GB
#: Physical cores a runner container takes, and the background memory kept free on host and engine VM.
RUNNER_CORES, HEADROOM_HOST_GB, HEADROOM_ENGINE_GB, HEADROOM_SHARE = PROCESSORS // 2, 8.0, 6.0, 0.25
#: The runner's user and its id; the image's name, its tag a release and a Python.
USER, UID, IMAGE = 'runner', 1001, 'coaxial-runner'
#: The images' architectures as the engine names them, each as actions/python-versions names its builds: the
#: runner's first.
ARCHS = {'amd64': 'x64', 'arm64': 'arm64'}
#: gate.yml's host job's keys an x86 runner's alone: numpy's x86 sets disabled, the robot's cores' x86 target.
X86_ENV = ('NPY_DISABLE_CPU_FEATURES', 'COAXIAL_ISA')
#: Where a container is shown its processors online: glibc counts them there, and os.cpu_count with it.
ONLINE = '/sys/devices/system/cpu/online'
#: The workflow job's one stated default of a list, quoted in its expression: PYTHONS: ${{ inputs.pythons || '3.12 3.14' }}.
PLANNED = re.compile(r"'([^']*)'")
#: The relay's arguments that ask for this host - a board, a model, a benchmark's priority -: not a container's.
HOSTS = ('--conformance', '--live', '--model', '--sections', '--match', '--tags', '--only', '--bench')
#: In the container, from its workspace: the tree a commit - a submodule its commit alone, an empty folder, as
#: the runner's checkout leaves it -, the package installed as the inputs action does (no index), the gate as
#: gate.yml's host step runs it, the relay's arguments after.
GATE = ('set -eo pipefail\n'
        'git init -q\n'
        'git -c user.name=%s -c user.email=%s@localhost commit --allow-empty -qm parent\n'
        'git add -A\n'
        'while read -r sha path; do [ -n "$sha" ] || continue; mkdir -p "$path"\n'
        '  git update-index --add --cacheinfo "160000,$sha,$path"; done <<< "$GITLINKS"\n'
        'git -c user.name=%s -c user.email=%s@localhost commit -qm tree\n'
        'python -m pip install -q --no-deps --no-build-isolation -e "$EDITABLE"\n'
        'cd "$DIRECTORY" && exec python tools/dev/run_tests.py --offline "$@"\n' % (USER, USER, USER, USER))
#: The containers running now: removed at once where the run is stopped.
LIVE = []


def workflow(path=WORKFLOW):
    """What gate.yml's host job runs on and runs: {'release': its runner's Ubuntu ('latest' of ubuntu-latest),
    'pythons': the workflow job's default Pythons (its PYTHONS line, the one place they are stated), 'apt': the packages, 'requirements', 'constraints' (its lock, tools/dev/pinned.py) and
    'editable': what its inputs action is given, 'env': the gate's step's environment, 'directory': where that step
    runs}."""
    import yaml
    with open(path, encoding='utf-8') as f:
        jobs = yaml.safe_load(f)['jobs']
    job, planned = jobs['host'], PLANNED.search(str(jobs['workflow']['env']['PYTHONS']))
    runs, steps = str(job['runs-on']), job['steps']
    if not runs.startswith('ubuntu-'):
        raise SystemExit('linux: gate.yml runs on %s - an image here is of an Ubuntu runner' % runs)
    given = next(step.get('with', {}) for step in steps if step.get('uses') == './.github/actions/inputs')
    gate = next(step for step in steps if 'run_tests.py' in step.get('run', ''))
    return {'release': runs.split('-', 1)[1], 'pythons': planned.group(1).split() if planned else [],
            'apt': str(given.get('apt', '')).split(), 'requirements': given['requirements'],
            'constraints': given.get('constraints') or None, 'editable': given['editable'], 'env': {k: str(v) for k, v in gate.get('env', {}).items()},
            'directory': gate.get('working-directory', '.')}


def engine():
    """The container engine's client (the docker SDK, host/requirements-linux.txt), once it answers."""
    try:
        import docker
        from docker.errors import DockerException
    except ImportError:
        raise SystemExit('linux: the engine is driven from Python by the docker SDK: '
                         'pip install -r host/requirements-linux.txt (setup.ps1 installs it)') from None
    try:
        client = docker.from_env(timeout=600)
        client.ping()
    except DockerException as e:
        raise SystemExit('linux: no container engine answers (%s) - setup.ps1 installs one; start it' % e) from None
    return client


def tagged(info, minor, arch='amd64'):
    """The tag of the runner's image for Python `minor` on `arch`: the runner's arch unnamed."""
    return '%s:%s-%s%s' % (IMAGE, info['release'], minor, '' if arch == next(iter(ARCHS)) else '-' + arch)


def environment(info, arch='amd64', isa=None, cpu=None):
    """A container's environment: gate.yml's host job's - on another arch than the runner's without its x86 keys
    (X86_ENV) -, the robot's cores' target `isa` where given (COAXIAL_ISA), the emulated processor `cpu`
    (QEMU_CPU)."""
    env = {k: v for k, v in info['env'].items() if arch == next(iter(ARCHS)) or k not in X86_ENV}
    env.update({k: v for k, v in (('COAXIAL_ISA', isa), ('QEMU_CPU', cpu)) if v})
    return env


def built(info, minor, fresh=False, arch='amd64'):
    """The runner's image for Python `minor` on `arch`, built by the engine's command line (BuildKit, each step kept;
    from scratch where `fresh`) of the Dockerfile, the requirements, their lock and the package's build: its tag."""
    program = shutil.which('docker')
    if program is None:
        raise SystemExit("linux: the engine's command line (docker) is not on PATH - a shell opened after its install")
    tag = tagged(info, minor, arch)
    context = io.BytesIO()
    # ustar: the command line tells a context from a Dockerfile by its first 512 bytes, and a pax header's
    # record (a file's mtime to the nanosecond) ends past them - the archive was read as a Dockerfile
    with tarfile.open(fileobj=context, mode='w', format=tarfile.USTAR_FORMAT) as tar:
        for name, path in (('Dockerfile', DOCKERFILE), ('requirements.txt', REPO / info['requirements']),
                           ('pyproject.toml', REPO / info['editable'] / 'pyproject.toml')):
            tar.add(str(path), arcname=name)
        lock = REPO / info['constraints'] if info['constraints'] else None
        if lock is not None and lock.is_file():
            tar.add(str(lock), arcname='constraints.txt')
        else:       # no lock yet (before its first laying): every version as the requirements allow
            tar.addfile(tarfile.TarInfo('constraints.txt'))
    told = {'RELEASE': info['release'], 'PYTHON': minor, 'APT': ' '.join(info['apt']), 'USER': USER, 'UID': UID,
            'ARCH': ARCHS[arch]}
    argv = [program, 'build', '--progress', 'plain', '--tag', tag, '--platform', 'linux/' + arch] + (
        ['--pull', '--no-cache'] if fresh else [])
    for key, value in told.items():
        argv += ['--build-arg', '%s=%s' % (key, value)]
    print('linux: image %s - built where its inputs changed, minutes the first time' % tag, flush=True)
    done = subprocess.run(argv + ['-'], input=context.getvalue(), capture_output=True, check=False)
    if done.returncode:
        said = (done.stdout + done.stderr).decode('utf-8', 'replace').splitlines()
        raise SystemExit('\n'.join(said[-40:]) + '\nlinux: the image of Python %s was not built (exit %d)'
                         % (minor, done.returncode))
    return tag


def kept(client, info, pythons, arch='amd64'):
    """{Python: whether its image on `arch` stands on the engine}."""
    from docker.errors import ImageNotFound
    stands = {}
    for minor in pythons:
        try:
            client.images.get(tagged(info, minor, arch))
            stands[minor] = True
        except ImageNotFound:
            stands[minor] = False
    return stands


def cores(client, tag):
    """The engine's physical cores, each its logical processors as a mask, as its sysfs pairs their threads -
    asked of a container of `tag`."""
    from machine.process.placed import listed
    # its log read here: run() hands back a container's output only where the engine logs in json-file or
    # journald, and none on another driver (the engine here logs `local`)
    probe = client.containers.run(tag, ['sh', '-c', 'cat /sys/devices/system/cpu/cpu[0-9]*/topology/thread_siblings_list'],
                                  detach=True)
    try:
        probe.wait()
        said = probe.logs(stdout=True, stderr=False)
    finally:
        probe.remove(force=True)
    return sorted({listed(line) for line in said.decode('ascii').split()})


def room(width=None, free=None, engine_gb=None, client=None):
    """Runner containers of two cores and BOOKED_GB that fit at once while leaving the desk's cores and a quarter
    of host commit and engine memory (8 GB and 6 GB at least) free for background work; at least one."""
    if engine_gb is None and client is not None:
        with suppress(Exception):
            total = int((client.info() or {}).get('MemTotal') or 0)
            engine_gb = total / 1024 ** 3 if total > 0 else None
    width = background.widths()[-1] if width is None else width
    free = census.free_commit_gb() if free is None else free
    by_cpu = max(1, width // RUNNER_CORES)
    by_host = by_cpu if free is None else max(1, int((free - max(HEADROOM_HOST_GB, HEADROOM_SHARE * free)) // BOOKED_GB))
    by_vm = by_cpu if engine_gb is None else max(1, int((engine_gb - max(HEADROOM_ENGINE_GB, HEADROOM_SHARE * engine_gb)) // BOOKED_GB))
    return min(by_cpu, by_host, by_vm)


def busy(client):
    """The processor mask held right now by live gate containers on `client`."""
    from machine.process.placed import listed
    mask = 0
    with suppress(Exception):
        for container in client.containers.list(filters={'label': 'coaxial.relay'}):
            cpus = ((container.attrs or {}).get('HostConfig') or {}).get('CpusetCpus') or ''
            if cpus:
                mask |= listed(cpus)
    return mask


def shaped(found, count, processors=PROCESSORS, held=0):
    """Up to `count` sets of whole cores of `found` (each its processors, a mask), each of `processors` logical
    processors at least - the runner's -, apart from each other, from `held` and from the first core (the system's
    interrupts') where there are enough; none where `found` holds fewer than `processors`."""
    if count <= 0:
        return []
    order = sorted(found)
    rest = order[1:] + order[:1]
    ordered = [c for c in rest if not (c & held)] + [c for c in rest if c & held]
    sets, mask = [], 0
    for core in ordered:
        mask |= core
        if bin(mask).count('1') >= processors:
            sets.append(mask)
            if len(sets) == count:
                break
            mask = 0
    return sets


def tree(top, root=REPO):
    """(the tree as it stands here - committed or not, nothing ignored, each file's mode as git keeps it - as a
    tar under `top`, its folders and files the runner's, all of one time as a checkout's are; its submodules,
    'commit path' a line)."""
    def ls(*flags):
        out = subprocess.run(['git', 'ls-files', '-z', *flags], cwd=root, capture_output=True, check=True).stdout
        return [entry.decode('utf-8') for entry in out.split(b'\0') if entry]

    modes, links = {}, []
    for entry in ls('--stage'):
        meta, path = entry.split('\t', 1)
        if meta.startswith('160000'):
            links.append('%s %s' % (meta.split()[1], path))
            continue
        modes[path] = 0o755 if meta.startswith('100755') else 0o644
    for path in ls('--others', '--exclude-standard'):
        modes.setdefault(path, 0o644)

    checkout = int(time.time())

    def header(name, kind, mode, size=0):
        info = tarfile.TarInfo(name)
        info.type, info.mode, info.size, info.mtime = kind, mode, size, checkout
        info.uid, info.gid, info.uname, info.gname = UID, UID, USER, USER
        return info

    buf, folders = io.BytesIO(), set()
    with tarfile.open(fileobj=buf, mode='w') as tar:
        for path in sorted(modes):
            full = os.path.join(root, path)
            if not os.path.isfile(full):
                continue          # deleted here, still in the index
            parts = ('%s/%s' % (top, path)).split('/')
            for depth in range(1, len(parts)):
                folder = '/'.join(parts[:depth])
                if folder not in folders:
                    folders.add(folder)
                    tar.addfile(header(folder, tarfile.DIRTYPE, 0o755))
            with open(full, 'rb') as f:
                tar.addfile(header('/'.join(parts), tarfile.REGTYPE, modes[path], os.path.getsize(full)), f)
    return buf.getvalue(), '\n'.join(links)


def ran(client, tag, info, cpus, argv, say, checkout, name):
    """The gate in a container of `tag` held to processors `cpus` (a mask), the tree `checkout` (`tree`) in its
    workspace of the repository `name`, its lines to `say`: its exit code."""
    tar, links = checkout
    from docker.errors import APIError
    from docker.types import Mount
    listed =','.join(str(cpu) for cpu in range(cpus.bit_length()) if cpus >> cpu & 1)
    # an LF alone, as Linux writes it: text mode wrote a CR on Windows, glibc's parse stopped at it and counted the
    # engine's processors instead (os.cpu_count 64 or 5, not 4)
    with tempfile.NamedTemporaryFile('w', suffix='.online', delete=False, encoding='ascii', newline='\n') as online:
        online.write(listed + '\n')
    env = dict(info['env'], EDITABLE=info['editable'], DIRECTORY=info['directory'], GITLINKS=links)
    container = None
    try:
        container = client.containers.create(
            tag, ['bash', '-c', GATE, 'gate', *argv], user=USER, environment=env,
            working_dir='/home/%s/work/%s/%s' % (USER, name, name), cpuset_cpus=listed, mem_limit=MEMORY,
            memswap_limit=MEMORY, shm_size=SHM, mounts=[Mount(ONLINE, online.name, type='bind', read_only=True)],
            labels={'coaxial.relay': str(os.getpid()), 'coaxial.tree': str(REPO)})   # its owner and its tree (ledger.clean)
        LIVE.append(container)
        container.put_archive('/home/%s/work' % USER, tar)
        container.start()
        rest = b''
        for chunk in container.logs(stream=True, follow=True):
            *lines, rest = (rest + chunk).split(b'\n')
            for line in lines:
                say(line.decode('utf-8', 'replace').rstrip('\r'))
        if rest:
            say(rest.decode('utf-8', 'replace'))
        return container.wait()['StatusCode']
    finally:
        if container is not None:
            with suppress(APIError):
                container.remove(force=True)
        os.unlink(online.name)


def repository():
    """The repository's name, as its checkout's folder there is named: its common git folder's parent's."""
    common = subprocess.run(['git', 'rev-parse', '--path-format=absolute', '--git-common-dir'], cwd=REPO,
                            capture_output=True, text=True, check=False).stdout.strip()
    return os.path.basename(os.path.dirname(common)) or REPO.name


def main(argv=None):
    """The gate on Linux, a container a Python of gate.yml's matrix (or those --python names), side by side: the
    first exit code not 0 in the matrix's order, else 0; --build and --kept: its images alone."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--python', action='append', help="a Python of gate.yml's matrix, it alone (repeatable)")
    parser.add_argument('--build', action='store_true', help='build the images and stop')
    parser.add_argument('--fresh', action='store_true', help='build them from scratch, the base pulled anew')
    parser.add_argument('--kept', action='store_true', help='whether each image stands: 0 where all do')
    parser.add_argument('--arch', choices=list(ARCHS), default=next(iter(ARCHS)),
                        help="the image's architecture: arm64 emulated by the engine's QEMU")
    parser.add_argument('--isa', help="the robot's cores' target there (COAXIAL_ISA), in place of gate.yml's")
    parser.add_argument('--cpu', help="with --arch arm64: the emulated processor (QEMU_CPU)")
    parser.add_argument('--parts', type=int, default=1,
                        help='runner containers a Python side by side (--part k/n each; 0 fits RAM and cores)')
    args, relayed = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    hosts = [arg for arg in relayed if arg.split('=', 1)[0] in HOSTS]
    if hosts:
        raise SystemExit('linux: %s asks for this host - a board, a model, a benchmark -: the container runs the '
                         'offline gate' % ' '.join(hosts))
    if args.cpu and args.arch == next(iter(ARCHS)):
        raise SystemExit('linux: --cpu is the emulated processor of an --arch arm64 image')
    if args.parts < 0:
        raise SystemExit('linux: --parts is 0 (fit RAM and cores) or at least 1')
    if args.parts != 1 and any(arg == '--part' or arg.startswith('--part=') for arg in relayed):
        raise SystemExit('linux: --parts lays --part k/n; pass one or the other')
    info = workflow()
    pythons = args.python or info['pythons']
    if not set(pythons) <= set(info['pythons']):
        raise SystemExit("linux: gate.yml's matrix is %s" % ', '.join(info['pythons']))
    client = engine()
    for line in ledger.clean(client=client):
        print('linux: ' + line, flush=True)
    if args.kept:
        stands = kept(client, info, pythons, args.arch)
        print(', '.join('%s %s' % (minor, ('missing', 'kept')[ok]) for minor in stands if (ok := stands[minor]) is not None))
        return 0 if all(stands.values()) else 1
    tags = [built(info, minor, args.fresh, args.arch) for minor in pythons]
    info = dict(info, env=environment(info, args.arch, args.isa, args.cpu))
    if args.build:
        return 0
    found = cores(client, tags[0])
    cap = room(client=client)
    parts = max(1, cap // len(pythons)) if args.parts == 0 else args.parts
    slots = [(minor, tag, part) for minor, tag in zip(pythons, tags)
             for part in (range(1, parts + 1) if parts > 1 else (None,))]
    sets = shaped(found, min(len(slots), cap), held=busy(client)) or [sum(found)]
    name = repository()
    checkout, lock, codes = tree('%s/%s' % (name, name)), threading.Lock(), {}
    todo: queue.SimpleQueue = queue.SimpleQueue()
    pool: queue.SimpleQueue = queue.SimpleQueue()
    for slot in slots:
        todo.put(slot)
    for cpus in sets:
        pool.put(cpus)

    def one(minor, tag, part, cpus):
        tag_part = '%s %d/%d' % (minor, part, parts) if part and len(pythons) > 1 else (
            '%d/%d' % (part, parts) if part else (minor if len(pythons) > 1 else ''))
        prefix = '%s| ' % tag_part if tag_part else ''

        def say(line):
            with lock:
                print(prefix + line, flush=True)
        held_cores = sum(1 for core in found if core & cpus)
        label = 'linux-%s%s' % (minor, '-%d' % part if part else '')
        extra = ['--part', '%d/%d' % (part, parts)] if part else []
        say('linux: Python %s%s, %s, processors %s (%d cores, %d threads), %s' % (
            minor, ' part %d/%d' % (part, parts) if part else '', tag,
            ','.join(str(c) for c in range(cpus.bit_length()) if cpus >> c & 1),
            held_cores, bin(cpus).count('1'), MEMORY))
        # its cores booked in the machine's ledger beside every relay's jobs, waited for while the machine is taken:
        # what it waits for said as it begins to - silent, it waited for commit as long as none was let go
        with ledger.reserved(held_cores, BOOKED_GB, label, say=lambda what: say('linux: ' + what)) as booking:
            if booking.waited >= ledger.POLL_S:
                say('linux: waited %.0f s for the machine' % booking.waited)
            codes[(minor, part)] = ran(client, tag, info, cpus, relayed + extra, say, checkout, name)

    def worker():
        while True:
            try:
                minor, tag, part = todo.get_nowait()
            except queue.Empty:
                return
            cpus = pool.get()
            try:
                one(minor, tag, part, cpus)
            finally:
                pool.put(cpus)

    jobs = [threading.Thread(target=worker, daemon=True) for _ in sets]
    try:
        for job in jobs:
            job.start()
        for job in jobs:
            while job.is_alive():
                job.join(0.5)
    except KeyboardInterrupt:
        for container in LIVE:
            with suppress(Exception):
                container.remove(force=True)
        print('\nlinux: stopped - its containers removed')
        return 130
    if len(slots) > 1:
        print('linux: ' + ', '.join('Python %s%s exit %s' % (
            minor, ' %d/%d' % (part, parts) if part else '', codes.get((minor, part), 'none'))
            for minor, _tag, part in slots))
    return next((codes.get((minor, part), 1) for minor, _tag, part in slots if codes.get((minor, part), 1)), 0)


if __name__ == '__main__':
    sys.exit(main())
