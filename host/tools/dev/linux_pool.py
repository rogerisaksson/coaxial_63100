"""Warm container pool for focus.relay across x86_64 and QEMU-ARM64 targets."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
import os
import queue
import tempfile
import threading
import time

from tools.dev import ledger, linux

#: Default QEMU processor flags for arm64 vector and streaming targets when `--cpu` is omitted.
ARM_CPUS = {'sve2': 'max,sve-default-vector-length=64',
            'sme2': 'max,sve-default-vector-length=64,sme-default-vector-length=64'}
#: Target presets `(arch, isa)` for `--container TARGET` and `linux.py --target TARGET`.
PRESETS = {'linux': ('amd64', None), 'amd64': ('amd64', None), 'x64': ('amd64', None),
           'avx': ('amd64', 'avx'), 'avx2': ('amd64', 'avx2'), 'avx512': ('amd64', 'avx512'),
           'arm64': ('arm64', None), 'sve2': ('arm64', 'sve2'), 'sme2': ('arm64', 'sme2')}
#: Pure-Python suites that build no shared C library in `build/hosttest`.
PURE_SUITES = frozenset({'test_platform.py', 'test_workflow.py', 'test_ci.py', 'test_smart.py', 'test_depends.py',
                         'test_wire_shapes.py', 'test_wire_requests.py', 'test_protocol_doc.py', 'test_ortho_texts.py',
                         'test_metacompiler_crafts.py', 'test_metacompiler_solver.py', 'test_says.py'})


def resolved(target=None, arch=None, isa=None, cpu=None):
    """`(arch, isa, cpu)` resolved from preset `target` (`PRESETS`) and explicit overrides."""
    if target is not None:
        if target not in PRESETS:
            raise SystemExit('linux: unknown target %s (choose from %s)' % (target, ', '.join(PRESETS)))
        preset_arch, preset_isa = PRESETS[target]
        arch = arch or preset_arch
        isa = isa or preset_isa
    elif arch is None and isa in ARM_CPUS:
        arch = 'arm64'
    arch = arch or next(iter(linux.ARCHS))
    if arch not in linux.ARCHS:
        raise SystemExit('linux: unknown arch %s (choose from %s)' % (arch, ', '.join(linux.ARCHS)))
    if cpu and arch == next(iter(linux.ARCHS)):
        raise SystemExit('linux: --cpu is the emulated processor of an --arch arm64 image')
    if arch == 'arm64' and cpu is None and isa in ARM_CPUS:
        cpu = ARM_CPUS[isa]
    return arch, isa, cpu


def _decoded(out):
    """`exec_run`'s output decoded as UTF-8 text when `bytes`, else empty."""
    return out.decode('utf-8', 'replace') if isinstance(out, (bytes, bytearray)) else ''


class Pool:
    """Warm persistent runner containers fed as batons by `focus.relay`, sharing one C core build."""

    def __init__(self, target='linux', python=None, batons=None, arch=None, isa=None, cpu=None, suites=(),
                 float_cpus=False):
        self.arch, self.isa, self.cpu = resolved(target, arch, isa, cpu)
        self.python, self.batons, self.suites, self.float_cpus = python, batons, tuple(suites), float_cpus
        self.width, self.workdir, self.host = 1, '', {}
        self._free: queue.SimpleQueue = queue.SimpleQueue()
        self._slots: list = []
        self._aborted = False

    def __enter__(self):
        from docker.types import Mount
        from tools.dev.suites import SHARDED
        info = linux.workflow()
        minor = self.python or info['pythons'][0]
        if minor not in info['pythons']:
            raise SystemExit("linux: gate.yml's matrix is %s" % ', '.join(info['pythons']))
        self.client = client = linux.engine()
        for line in ledger.clean(client=client):
            print('linux: ' + line, flush=True)
        tag = linux.built(info, minor, arch=self.arch)
        found = linux.cores(client, tag)
        cap = linux.room(client=client, arch=self.arch)
        demand = sum(4 if s in SHARDED else 1 for s in self.suites) if self.suites else cap
        wanted = max(1, min(cap, self.batons if self.batons else demand))
        name = linux.repository()
        self.workdir = '/home/%s/work/%s/%s' % (linux.USER, name, name)
        tar, links = linux.tree('%s/%s' % (name, name))
        sets = linux.shaped(found, wanted, held=linux.busy(client),
                            cores=linux.ARM_CORES if self.arch == 'arm64' else 0) or [sum(found)]
        self.env = dict(linux.environment(info, self.arch, self.isa, self.cpu), CI='true',
                        EDITABLE=info['editable'], DIRECTORY=info['directory'], GITLINKS=links,
                        PYTHONIOENCODING='utf-8', PYTHONUNBUFFERED='1', PYTHONFAULTHANDLER='1',
                        OPENBLAS_NUM_THREADS='1', COAXIAL_FALLBACK='simulated')
        if self.float_cpus:
            from machine.process.placed import FLOAT
            self.env[FLOAT] = '1'
        lock = threading.Lock()

        def start_one(idx, cpus):
            listed = ','.join(str(c) for c in range(cpus.bit_length()) if cpus >> c & 1)
            held_cores = sum(1 for core in found if core & cpus)
            label = 'linux-%s-r%d' % (minor, idx + 1)
            wait_s = None if idx == 0 or self.batons else 2.0
            booking = ledger.reserved(
                held_cores, linux.BOOKED_GB, label, wait_s=wait_s,
                say=(lambda what: print('linux: ' + what, flush=True)) if idx == 0 else None).__enter__()
            if booking.path is None:
                return None
            online = tempfile.NamedTemporaryFile('w', suffix='.online', delete=False, encoding='ascii', newline='\n')
            online.write(listed + '\n')
            online.close()
            container = None
            try:
                container = client.containers.create(
                    tag, ['sleep', 'infinity'], user=linux.USER, environment=self.env, working_dir=self.workdir,
                    cpuset_cpus=listed, mem_limit=linux.MEMORY, memswap_limit=linux.MEMORY, shm_size=linux.SHM,
                    mounts=[Mount(linux.ONLINE, online.name, type='bind', read_only=True)],
                    labels={'coaxial.relay': str(os.getpid()), 'coaxial.tree': str(linux.REPO)})
                with lock:
                    linux.LIVE.append(container)
                container.put_archive('/home/%s/work' % linux.USER, tar)
                container.start()
                code, out = container.exec_run(['bash', '-c', linux.SETUP], workdir=self.workdir, environment=self.env)
                if code != 0:
                    raise RuntimeError(_decoded(out)[-800:])
                with lock:
                    print('linux: relay baton %d/%d, Python %s, %s, processors %s (%d cores, %d threads), %s'
                          % (idx + 1, len(sets), minor, tag, listed, held_cores, bin(cpus).count('1'), linux.MEMORY),
                          flush=True)
                return container, online.name, booking, held_cores, bin(cpus).count('1')
            except Exception:
                if container is not None:
                    with suppress(Exception):
                        container.remove(force=True)
                with suppress(OSError):
                    os.unlink(online.name)
                booking.__exit__(None, None, None)
                if idx == 0:
                    raise
                return None

        try:
            with ThreadPoolExecutor(len(sets)) as ex:
                started = [slot for slot in ex.map(lambda p: start_one(*p), enumerate(sets)) if slot is not None]
            if not started:
                raise SystemExit('linux: no container baton started')
            self._slots = started
            self.width = len(started)
            self._warm_c()
            self.host = self._measure(started[0][0], started[0][3], started[0][4])
            for slot in started:
                self._free.put(slot[0])
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def _warm_c(self):
        """Build shared C libraries once in baton 0 and copy `build/` into sibling batons."""
        if len(self._slots) < 2 or set(self.suites) <= PURE_SUITES:
            return
        mods, s = [], set(self.suites)
        if not s or any(x not in PURE_SUITES | {'test_se3_core.py', 'test_se3_grip.py', 'test_mppi_core.py',
                                                'test_aero_core.py', 'test_libm.py'} for x in s):
            mods.append('from tools.cores import wbc; wbc.library()')
        if not s or {'test_se3_core.py', 'test_se3_grip.py'} & s:
            mods.append('from tools.cores import se3; se3.library(False); se3.library(True)')
        if not s or {'test_mppi_core.py', 'test_aerobat.py', 'test_helicopter.py'} & s:
            mods.append('from tools.cores import mppi; mppi.library()')
        if not mods:
            return
        c0 = self._slots[0][0]
        code, _ = c0.exec_run(['python', '-X', 'utf8', '-c', '; '.join(mods)], workdir=self.workdir + '/host',
                              environment=dict(self.env, PYTHONPATH=self.workdir + '/host'))
        if code != 0:
            return
        with suppress(Exception):
            bits, _ = c0.get_archive(self.workdir + '/build')
            blob = b''.join(bits)
            for c, *_rest in self._slots[1:]:
                c.put_archive(self.workdir, blob)

    def _measure(self, c0, held_cores, threads):
        """Measure container pace with a short probe so QEMU calibration stays under a few seconds."""
        turns = 400 if self.arch == 'arm64' else 3000
        probe = ('from tools.dev import machine_detector as m\n'
                 'm.work(40)\ndt = m.work(%d)\n'
                 'print("%%.6f|%%s|%%s" %% ((%d / m.TURNS) / max(dt, 1e-6), m.platform.machine(), m.processor()))\n'
                 % (turns, turns))
        code, out = c0.exec_run(['python', '-X', 'utf8', '-c', probe], workdir=self.workdir + '/host',
                                environment=dict(self.env, PYTHONPATH=self.workdir + '/host'))
        text = _decoded(out).strip().splitlines()
        pace_s, mach, proc = (text[-1].split('|', 2) + ['1.0', self.arch, self.arch])[:3] if code == 0 and text else (
            '1.0', self.arch, self.arch)
        pace = max(0.005, float(pace_s))
        return {'runner': True, 'cores': held_cores, 'threads': threads, 'processor': proc,
                'machine': mach, 'system': 'Linux', 'width': self.width, 'pace': pace,
                'paces': {self.width: pace}}

    def __call__(self, job):
        """Run one `focus.Job` inside a warm container baton: `(output, exit_code_or_None, elapsed)`."""
        if self._aborted:
            return '', 1, 0.0
        container = self._free.get()
        started = time.monotonic()
        cmd = list(job.argv)
        if job.timeout is not None:
            cmd = ['timeout', '-s', 'ABRT', '-k', '10s', '%.1fs' % job.timeout] + cmd
        env = dict(self.env, PYTHONPATH=self.workdir + '/host')
        try:
            code, out = container.exec_run(cmd, workdir=self.workdir + '/host', environment=env)
            elapsed = time.monotonic() - started
            timed_out = job.timeout is not None and code in (124, 134, 137) and elapsed >= 0.9 * job.timeout
            return _decoded(out), (None if timed_out else code), elapsed
        except Exception as exc:
            if self._aborted:
                return '', 130, time.monotonic() - started
            return '%s: %s' % (type(exc).__name__, exc), 1, time.monotonic() - started
        finally:
            if not self._aborted:
                self._free.put(container)

    def abort(self):
        """Stop all warm containers immediately so in-flight `exec_run` calls return at once."""
        self._aborted = True
        for container, *_rest in self._slots:
            with suppress(Exception):
                container.remove(force=True)

    def __exit__(self, *exc):
        self._aborted = True
        for container, online, booking, *_rest in self._slots:
            with suppress(Exception):
                container.remove(force=True)
            if container in linux.LIVE:
                linux.LIVE.remove(container)
            with suppress(OSError):
                os.unlink(online)
            booking.__exit__(None, None, None)
        self._slots = []
