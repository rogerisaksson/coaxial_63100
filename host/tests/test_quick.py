#!/usr/bin/env python3
"""The robot's compiled parts called whole in C (`tools.cores.quick`'s runner, quick_run.c) beside their Python glue - each part
of its law walking given twins of its arguments made alike -, and the robot's step asked in C (quick_step.c)
beside its Python from the same struct: both ways' results the same trees, types, views and bits at every call. The
robot's control system compiled before its page lays it (`tools.sim.warmed`): a line a beat while its builds are under
way, off its page's core, its strip a clock - none of the three, the page stood on [STANDING] and was taken for hung."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The robot's walk's passes; its lie into a bed and its rise off it, s.
PASSES, LIE_S, RISE_S = 3000, 12.0, 8.0


def _copied(v):
    """`v`'s arrays copied, its dicts, lists and tuples anew: a twin both of a part's ways are given alike."""
    if isinstance(v, np.ndarray):
        return v.copy()
    if isinstance(v, dict):
        return {k: _copied(x) for k, x in v.items()}
    return type(v)(_copied(x) for x in v) if isinstance(v, (list, tuple)) else v


def _apart(a, b):
    """Whether two results differ: their trees, types, dtypes, shapes, views or bits."""
    if type(a) is not type(b):
        return True
    if isinstance(a, dict):
        return list(a) != list(b) or any(_apart(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)):
        return len(a) != len(b) or any(_apart(x, y) for x, y in zip(a, b))
    if isinstance(a, (np.ndarray, np.generic)):
        return (a.dtype != b.dtype or np.shape(a) != np.shape(b) or np.asarray(a).tobytes() != np.asarray(b).tobytes()
                or (isinstance(a, np.ndarray) and (a.base is None) != (b.base is None)))
    return not (a == b or (a != a and b != b))


def test_a_part_called_whole_in_c_as_by_its_glue(report, c):
    """Each compiled part of the robot's law walking, called whole in C (its head, walk, C and result's views) and
    through its Python glue on twins of its arguments made alike - its turn kept as the robot's own call finds it -:
    their results the same trees, types, views and bits at every call."""
    from machine.process import worker
    from tools.cores import bodied, quick
    if quick.module() is None:
        report.check('its parts called whole in C, where the extension is built', True, 'not built here')
        return
    tally, call = {'calls': 0, 'apart': 0, 'parts': set()}, bodied.Compiled.__call__

    def beside(self, *args, paced=True):
        b = self.c
        if b is not None and b.runner is not None:
            twins = [tuple(a if k == self.body else _copied(a) for k, a in enumerate(args)) for _ in range(2)]
            turn, got = int(b.turns[0]), b.runner(twins[0])
            if got is not quick.MISS and got is not quick.SLOW and b.fits(twins[1]) is not None:
                tally['apart'] += _apart(got, b(twins[1]))
                tally['calls'] += 1
                tally['parts'].add(self.name)
            b.turns[0] = turn
        out = call(self, *args, paced=False if b is None else paced)
        if self.c is None and self.job is not None:
            self.job['done'].wait()
        return out
    bodied.Compiled.__call__ = beside
    try:
        s = worker.session(walks=True)
        for _ in range(PASSES):
            worker.stepped(s)
    finally:
        bodied.Compiled.__call__ = call
    report.check('its parts called whole in C as by their glue, %d calls over %d parts walking'
                 % (tally['calls'], len(tally['parts'])),
                 tally['calls'] > 100 and len(tally['parts']) >= 15 and not tally['apart'], '%d apart' % tally['apart'])


def _numbers(k):
    """The step's struct `k` but its pointers: each field's bytes not a pointer's, in its order."""
    import ctypes

    def pointer(t):
        return t is ctypes.c_void_p or (hasattr(t, '_length_') and pointer(t._type_))
    at = ctypes.addressof(k)
    return b''.join(ctypes.string_at(at + getattr(type(k), name).offset, ctypes.sizeof(t))
                    for name, t in type(k)._fields_ if not pointer(t))


def test_the_robots_step_asked_in_c_as_by_python(report, c):
    """The robot's step's ask and senses laid into its struct, its C called and its answer made in one C call
    (quick_step.c) beside `_stepped`'s Python from the same struct - lying into a bed, rising off it and walking -:
    every answer the same trees, types, views and bits, `b.load` too, and the struct but its pointers the same
    after."""
    import ctypes
    from machine.process import worker
    from tools.cores import quick, step as cs
    from tools.sim import told
    if quick.module() is None:
        report.check('its step asked in C, where the extension is built', True, 'not built here')
        return
    tally, real = {'calls': 0, 'apart': 0, 'miss': 0, 'contacts': 0}, cs._stepped

    def beside(k, step, v, b, s, ask):
        q = k.quick
        if q is None:
            return real(k, step, v, b, s, ask)
        snap, load = ctypes.string_at(ctypes.addressof(k), ctypes.sizeof(k)), getattr(b, 'load', None)
        got = q[0](q[1], k, b, s, ask)
        if got is q[2]:
            tally['miss'] += 1
            return real(k, step, v, b, s, ask)
        after, load_c = _numbers(k), b.load
        ctypes.memmove(ctypes.addressof(k), snap, len(snap))
        b.load, k.quick = load, None
        try:
            mine = real(k, step, v, b, s, ask)
        finally:
            k.quick = q
        tally['apart'] += _apart(got, mine) or _apart(load_c, b.load) or after != _numbers(k)
        tally['calls'] += 1
        tally['contacts'] += len(s.get('contacts', ()))
        return mine
    cs._stepped = beside
    try:
        r = told.Run('bed_behind')
        r.tell(say='lie_bed')
        r.go(LIE_S)
        r.tell(say='rise_bed')
        r.go(RISE_S)
        s = worker.session(walks=True)
        for _ in range(PASSES):
            worker.stepped(s)
    finally:
        cs._stepped = real
    report.check('its step asked in C as by Python, %(calls)d calls over %(contacts)d contacts (%(miss)d missed)' % tally,
                 tally['calls'] > 3000 and tally['contacts'] > 1000 and not tally['apart'], '%(apart)d apart' % tally)


def test_a_runner_lets_its_part_go(report, c):
    """A part's runner (quick_run.c) holds its Bound and the Bound's leaf, and the Bound holds the runner: the
    collector walks what the runner holds, so a part dropped is let go - as a capsule the runner hid that cycle, and
    every Bound laid outlived its world."""
    import functools
    import gc
    import weakref
    from tools.cores import quick
    q = quick.module()
    if q is None:
        report.check('a runner lets its part go, where the extension is built', True, 'not built here')
        return

    class Part:
        def __init__(self):
            self.runner = functools.partial(q.run, q.runner(self, None, -1, (), (), (), np.array_equal, None, [], self.leaf,
                                                            (), 0, False, (None,) * 6, (), np.zeros(1, np.int64), 0, (),
                                                            (4, None), np.linalg.LinAlgError, quick.MISS, quick.SLOW))

        def leaf(self, _j, _a):
            return True
    part = Part()
    gone = weakref.ref(part)
    del part
    gc.collect()
    report.check('a part\'s runner lets the part go: the collector walks what the runner holds', gone() is None, '')


def test_a_world_laid_anew_lets_those_before_go(report, c):
    """The robot's world laid anew again and again - a restart, a fall's recovery -: the worlds laid before it let
    go by the worker itself, no more of theirs held after six than after three - the last body bound to a part till
    its next call -. A part's runner hid its cycle from the collector, a cache by the world's model and a part
    guarding the robot's edge kept each world's, and a world's cycles waited on the collector's full passes: 34 MB a
    world laid anew."""
    import weakref
    from machine.process import worker
    from machine.world import struck
    s, laid, held = worker.session(walks=True), [], {}
    for k in range(6):
        for _ in range(PASSES // 10):
            worker.stepped(s)
        struck.touching(s['g']['world'])
        laid += [weakref.ref(s['g']['world'].model), weakref.ref(s['g']['body'])]
        worker.apply(s, {'restart': True})
        if k in (2, 5):
            held[k + 1] = sum(w() is not None for w in laid)
    report.check('a world laid anew lets those before it go: %d of their models and bodies held after three, %d after '
                 'six' % (held[3], held[6]), held[6] <= held[3], '')


def test_the_robots_compile_says_each_build_as_it_lands(report, c):
    """The robot's compile before its page (`tools.sim.warmed.built`): a line a beat while no build lands, each build
    counted as it lands in any order, on the cores they are built on, the strip's share crept from its low to its
    high. Told only as the list's first landed, the page stood on [STANDING] 57 s and was taken for hung."""
    import threading
    import time
    from tools.cores import held
    from tools.sim import warmed
    jobs = [{'done': threading.Event()} for _ in range(3)]
    lines = warmed.built('standing', 0.125, 0.25, jobs, beat=0.05)
    began = time.perf_counter()
    said = [next(lines) for _ in range(4)]
    quiet = time.perf_counter() - began
    for k in (2, 0, 1):
        jobs[k]['done'].set()
        said.append(next(lines))
    said += list(lines)
    told = ['STANDING: %d OF 3 BUILT ON %d CORES' % (n, held.WIDTH[0]) for n in (0, 0, 0, 0, 1, 2, 3)]
    report.check('the robot\'s compile says a line a beat while no build lands (4 in %.2f s), then each build as it '
                 'lands, in any order, on its cores' % quiet,
                 [text for _share, text in said] == told and quiet < 2.0, ' | '.join(t for _s, t in said))
    report.check('its strip\'s share crept from its errand\'s low to its high as the builds land',
                 [share for share, _text in said] == [0.125] * 4 + [0.125 + 0.125 * n / 3 for n in (1, 2, 3)],
                 ' '.join('%.4f' % share for share, _text in said))


#: The compile's process as its page starts it, on the page's core till it spreads: its processors before and after,
#: those free off the page's core and SMT siblings.
SPREAD = ('import sys; from machine.process import placed; from tools.sim import warmed; sys.stdin.readline(); '
          'was = placed.now(); warmed.spread(); '
          'print("%x %x %x" % (was, placed.now(), placed.free_processors()), flush=True)')


def test_the_robots_compile_leaves_its_pages_core(report, c):
    """The compile the robot's page starts (`tools.sim.warmed.spread`), started on a core as the page holds one and
    told to float as the page tells it: on one processor per free physical core off the page's core once it spreads -
    neither trapped on the page's core nor contending across SMT siblings."""
    import subprocess
    from machine.process import placed
    cores = placed.cores()
    mask, holding = placed.claim()
    if not mask or len(cores) < 3:
        placed.free(holding)
        report.check('the robot\'s compile leaves its page\'s core (this host tells none to bind, or none is free)',
                     True, '')
        return
    env = dict(os.environ, PYTHONPATH=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
               **{placed.FLOAT: '1'})
    try:
        child = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', SPREAD], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, text=True, env=env)
        bound = placed.bind(child.pid, mask)
        said = child.communicate('\n', timeout=120)[0].split()
    finally:
        placed.free(holding)
    was, now, given = (int(x, 16) for x in said) if len(said) == 3 else (0, 0, -1)
    no_smt = all(((now & core) & ((now & core) - 1)) == 0 for core in cores)
    left_page = not (now & mask)
    report.check('the robot\'s compile started on its page\'s core spreads onto 1 thread per free physical core off '
                 'the page\'s core: %d processors of %d, from %d' % (bin(now).count('1'), bin(given).count('1'),
                                                                     bin(was).count('1')),
                 bound and was == mask and now == given and left_page and no_smt and bool(now),
                 '%x -> %x of %x (mask %x)' % (was, now, given, mask))
    ui_env = dict(os.environ, PYTHONPATH=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ui_child = subprocess.Popen(
        [sys.executable, '-X', 'utf8', '-c',
         'from machine.process import placed; ok = placed.placed(ui=True); '
         'cored = placed._CORED[1] if len(placed._CORED) > 1 else 0; '
         'print("%d %x %x %x" % (ok, placed.now(), placed.free_processors(), cored), flush=True)'],
        stdout=subprocess.PIPE, text=True, env=ui_env)
    ui_said = (ui_child.communicate(timeout=30)[0] or '').split()
    ui_ok, ui_now, ui_free, ui_cored = (int(x, 16) for x in ui_said) if len(ui_said) == 4 else (0, 0, 0, 0)
    no_smt_ui = all(((ui_now & core) & ((ui_now & core) - 1)) == 0 for core in cores)
    no_smt_free = all(((ui_free & core) & ((ui_free & core) - 1)) == 0 for core in cores)
    report.check('terminal UX placement keeps child/loader mask free of SMT sibling contention and reserves its UI core',
                 bool(ui_ok and ui_now and ui_free and no_smt_ui and no_smt_free
                      and (not (ui_free & ui_cored) or bin(ui_free).count('1') <= 1)),
                 'ui %x, free %x, cored %x' % (ui_now, ui_free, ui_cored))


def test_the_compiles_strip_keeps_time(report, c):
    """The strip over the robot's compile (`terminal.ui.stage.strip`) shows the time its step has stood: a step's
    text a minute unchanged still ticks, not a page taken for hung."""
    import io
    from rich.console import Console
    from terminal.ui import stage
    court = Console(file=io.StringIO(), width=100, color_system=None, theme=stage.THEME)
    bar = stage.strip(court)
    bar.add_task('STANDING: 0 OF 25 BUILT ON 28 CORES', total=100)
    bar.tasks[0].start_time = bar.get_time() - 65.0
    with court.capture() as cap:
        court.print(bar.get_renderable())
    shown = cap.get().strip()
    report.check('the compile\'s strip ticks: a step stood 65 s shows 0:01:05 beside its text',
                 '0:01:05' in shown and '[STANDING: 0 OF 25 BUILT ON 28 CORES]' in shown, shown[-64:])


def _cold_probe(x):
    return x + 1.0


def test_warm_only_and_cache_fallbacks(report, c):
    """`barrier.ahead(..., warm_only=True)` skips cold hulls, `kept.keep_shape` removes its temp file on replace
    failure, and `held.building` falls back to rebuilding when loading a cached library raises `OSError`."""
    import hashlib
    from machine.law import barrier
    from tools.cores import bodied, held, kept
    cold_left, cold_part = [11, 12], bodied.on_shapes(_cold_probe, 'cold_probe', (0,))
    barrier.ahead(cold_part, cold_left, warm_only=True)
    bad_path, orig_replace = kept._shape_file(cold_part.at), os.replace
    stale = '%s.%d' % (bad_path, os.getpid())
    try:
        os.replace = lambda _a, _b: (_ for _ in ()).throw(PermissionError('busy'))
        kept.keep_shape(cold_part.at, ((3,),))
    finally:
        os.replace = orig_replace
    src = 'void dummy_fallback(void) {}\n'
    orig_cached, orig_loaded, orig_built = held._cached_path, held.loaded, held.built
    rebuilt = []
    try:
        held._cached_path = lambda _s, _n, _m: 'missing_cached.so'
        held.loaded = lambda p, n, batch=False, solves=False: (
            (_ for _ in ()).throw(OSError('pruned')) if p == 'missing_cached.so'
            else (rebuilt.append(p) or (None, None)))
        held.built = lambda _s, _n, _m, _h=None: 'rebuilt.so'
        job = held.building(src, 'dummy_fallback', False)
        job['done'].wait(5.0)
    finally:
        held._cached_path, held.loaded, held.built = orig_cached, orig_loaded, orig_built
        held._BUILDING.pop(hashlib.sha1(src.encode()).hexdigest(), None)
    report.check('warm_only leaves cold hulls untraced, keep_shape removes temp on error, held retries on OSError',
                 cold_left == [11, 12] and not cold_part and not os.path.exists(stale)
                 and job['done'].is_set() and 'why' not in job and rebuilt == ['rebuilt.so'],
                 'left=%s stale=%s rebuilt=%s why=%s' % (cold_left, os.path.exists(stale), rebuilt, job.get('why')))


def test_warmed_errand_hulls_and_content_stamp(report, c):
    """`warmed._errand` and `_hold_hulls` read `pace.Run.s['g']` without `AttributeError`, `compiling._read` clears a
    machine on its `DONE` line, and `kept.stamp` stays invariant across timestamp-only touches."""
    import io
    import tempfile
    from machine.law import barrier
    from terminal import compiling
    from tools.cores import bodied, kept
    from tools.sim import warmed

    class FakeRun:
        def __init__(self):
            self.s = {'g': {'ringed': bodied.on_shapes(barrier.ringed, 'ringed_w', (0, 1, 2, 3, 4, 5)),
                            'ringed_ahead': [4, 11]}}
            self.told, self.ran = [], []

        def tell(self, **kw):
            self.told.append(kw)

        def go(self, sec):
            self.ran.append(sec)

    fake = FakeRun()
    orig_ahead = barrier.ahead
    calls = []
    try:
        barrier.ahead = lambda law, left, warm_only=False, paced=True: (calls.append((list(left), paced)) or left.clear())
        warmed._errand(fake, (0.5, 0.0, 0.0), 1.0)
        warmed._hold_hulls(fake)
    finally:
        barrier.ahead = orig_ahead

    class FakeProc:
        def __init__(self, lines):
            self.stdout = io.StringIO(lines)

        def poll(self):
            return 0

        def wait(self):
            return 0

    done_seen = []
    with compiling._LOCK:
        compiling.STATE['robot'] = [0.5, 'WALKING', 0.0]
        compiling.STATE['quad'] = [0.0, 'WAITING', 0.0]
    try:
        said = warmed.told(FakeProc('done robot 39 0 0 4.2\n'),
                           done=lambda m: (done_seen.append(m), compiling.STATE.pop(m, None)))
        robot_cleared = 'robot' not in compiling.STATE and 'quad' in compiling.STATE
        fault_said = warmed.told(FakeProc('AttributeError: boom\n'))
    finally:
        with compiling._LOCK:
            compiling.STATE.pop('robot', None)
            compiling.STATE.pop('quad', None)

    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'probe.py')
        with open(src, 'w', encoding='utf-8') as f:
            f.write('X = 1\n')
        orig_sources = kept.SOURCES
        try:
            kept.SOURCES = (tmp,)
            kept.stamp.cache_clear()
            s1 = kept.stamp()
            st = os.stat(src)
            os.utime(src, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000_000))
            kept.stamp.cache_clear()
            s2 = kept.stamp()
            with open(src, 'w', encoding='utf-8') as f:
                f.write('X = 2\n')
            kept.stamp.cache_clear()
            s3 = kept.stamp()
        finally:
            kept.SOURCES = orig_sources
            kept.stamp.cache_clear()

    report.check('warmed._errand reads Run.s, _hold_hulls sweeps unheld HULLS, DONE clears machine, content stamp ignores utime',
                 calls[0] == ([4, 11], False) and calls[1][0] == list(barrier.HULLS) and calls[1][1] is False
                 and fake.told == [{'twist': (0.5, 0.0, 0.0)}] and fake.ran == [1.0]
                 and done_seen == ['robot'] and robot_cleared and '39 parts held' in said.get('robot', '')
                 and fault_said.get('_fault') == 'AttributeError: boom' and s1 == s2 and s2 != s3,
                 'calls=%s done=%s s1==s2=%s s2!=s3=%s' % (calls[:2], done_seen, s1 == s2, s2 != s3))


ROSTER = (test_a_part_called_whole_in_c_as_by_its_glue, test_the_robots_step_asked_in_c_as_by_python,
          test_a_runner_lets_its_part_go, test_a_world_laid_anew_lets_those_before_go,
          test_the_robots_compile_says_each_build_as_it_lands, test_the_robots_compile_leaves_its_pages_core,
          test_the_compiles_strip_keeps_time, test_warm_only_and_cache_fallbacks,
          test_warmed_errand_hulls_and_content_stamp)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
