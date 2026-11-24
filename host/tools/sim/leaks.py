#!/usr/bin/env python3
"""The robot's memory as it lives: worlds laid anew, recordings and pages' frames, A/B to a tree.

    python -X utf8 tools/sim/leaks.py relays                 # MB a relay: the loop's own collector, and collected through
    python -X utf8 tools/sim/leaks.py recordings             # the hooks and writer processes recordings leave behind
    python -X utf8 tools/sim/leaks.py pages                  # kB and objects a frame, each page in a process apart
    python -X utf8 tools/sim/leaks.py pages humanoid quad --frames 200 1000
    python -X utf8 tools/sim/leaks.py relays --against C:/sources/other   # the same on another tree's host/, then here

Memory is the process's private bytes, the commit it holds: MuJoCo's arenas and a C core's blocks are native, out of
tracemalloc's sight. A relay is a world laid anew as the page's restart and a fall's recovery lay one (`worker.laid`),
once the held builds settle (`tools.cores.held.settled`: a fresh tree runs numpy, not C, for its first minutes) -
measured two ways: as the robot lives, on the loop's own collector, whose full passes wait on object counts blind to
native memory, and collected through (`gc.collect` till a pass finds none: a finalizer's garbage is freed a pass on),
with the worlds laid before still held told by weak references to their models and the law's bodies. A recording is
the page's R (`watch.recorded`): what one begun and ended leaves on the robot's passes and in processes. A page is
drawn by the page tool (`tools.render.page.frame`) in a process of its own at two lengths, the difference told a
frame with the kinds of objects grown - each process booked in the machine's ledger a core and its commit
(`tools.dev.ledger`), waiting its turn beside the relays', its stdout let go (a page draws there: left unread, a full
pipe held 14 of them) and its result read off its stderr.
"""
import argparse
import gc
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import weakref
from concurrent.futures import ThreadPoolExecutor

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: A world's passes before it is laid anew, and the relays measured each way.
PASSES, RELAYS = 300, 16
#: A page's commit booked in the ledger, GB: the menu, the most, held 0.64 GB private through 3000 frames.
PAGE_GB = 1.0
#: A page's process cut after this long, s, and a build waited for this long: the robot's page drew 1000 frames
#: in 250 s, a fresh tree's builds took 114 s.
PAGE_S = 1200.0
#: A page's process: the page drawn `n` frames, then its private MB, its objects and their kinds on stderr.
CHILD = r'''
import collections, faulthandler, gc, json, sys
import psutil
faulthandler.dump_traceback_later(float(sys.argv[3]), exit=True)
from tools.render import page
name, n = sys.argv[1], int(sys.argv[2])
page.frame(name, 150, 44, frames=n)
while gc.collect():
    pass
info = psutil.Process().memory_info()
mb = (getattr(info, 'private', None) or psutil.Process().memory_full_info().uss) / 2 ** 20
kinds = collections.Counter(type(o).__name__ for o in gc.get_objects())
sys.stderr.write('\nRESULT' + json.dumps({'mb': mb, 'objs': sum(kinds.values()), 'kinds': kinds}) + '\n')
'''


def private_mb():
    """This process's private memory, MB (its unique set where the system tells no private bytes)."""
    import psutil
    info = psutil.Process().memory_info()
    return (getattr(info, 'private', None) or psutil.Process().memory_full_info().uss) / 2 ** 20


def collected():
    """`private_mb` once the collector finds nothing more."""
    while gc.collect():
        pass
    return private_mb()


def _relay(s, passes):
    """The robot's world walked `passes` and laid anew as the page's restart lays it: the lay's seconds."""
    from machine.process import worker
    for _ in range(passes):
        worker.stepped(s)
    t0 = time.perf_counter()
    worker.apply(s, {'restart': True})
    return time.perf_counter() - t0


def relays(count=RELAYS, passes=PASSES):
    """{own: MB a relay on the loop's own collector (its later half), through: MB a relay collected through,
    held: (models, bodies) of the worlds laid before still alive, lay_ms: a relay's median}."""
    from machine.process import worker
    from machine.world import struck
    from tools.cores import held
    s = worker.session(walks=True)
    for _ in range(3):
        _relay(s, passes)
        held.settled(PAGE_S)
    lays, half, mid = [], count // 2, private_mb()
    for k in range(count):
        lays.append(_relay(s, passes))
        if k == half - 1:
            mid = private_mb()
    own = (private_mb() - mid) / (count - half)
    laid, first = [], collected()
    for _ in range(count):
        g = s['g']
        struck.touching(g['world'])
        laid += [weakref.ref(g['world'].model), weakref.ref(g['body'])]
        del g
        lays.append(_relay(s, passes))
    through = (collected() - first) / count
    lays.sort()
    return {'own': own, 'through': through, 'lay_ms': 1e3 * lays[len(lays) // 2],
            'held': (sum(w() is not None for w in laid[0::2]), sum(w() is not None for w in laid[1::2]))}


def _writers(me):
    """The gate writers process `me` runs (`gate.Scope._writer`)."""
    import psutil
    out = []
    for child in me.children(recursive=True):
        try:
            out += [child] if 'machine.rig.gate' in ' '.join(child.cmdline()) else []
        except psutil.Error:
            pass
    return out


def recordings(count=3, passes=60):
    """{hooks: hooks left on the robot's passes, writers: writer processes left, sizes: each recording's bytes}
    of `count` recordings begun and ended as the page's R does, `passes` each."""
    import psutil
    from machine.process import worker
    me, s = psutil.Process(), worker.session()
    hooks, ran = len(s['each']), len(_writers(me))
    with tempfile.TemporaryDirectory() as folder:
        for k in range(count):
            worker.apply(s, {'record': os.path.join(folder, 'r%d' % k)})
            for _ in range(passes):
                worker.stepped(s)
            worker.apply(s, {'record': None})
        until = time.monotonic() + 30.0
        while len(_writers(me)) > ran and time.monotonic() < until:
            time.sleep(0.05)
        sizes = [os.path.getsize(os.path.join(folder, name)) for name in sorted(os.listdir(folder))]
    return {'hooks': len(s['each']) - hooks, 'writers': len(_writers(me)) - ran, 'sizes': sizes}


def _page(host, name, frames):
    """Page `name` drawn `frames` in a process of its own on `host`, booked in the ledger: its result, or why none."""
    from tools.dev import ledger
    with ledger.reserved(1, PAGE_GB, 'leaks %s %d' % (name, frames)):
        # a page that sticks says where (its threads' stacks on stderr) and ends, a minute before it is cut
        proc = subprocess.Popen([sys.executable, '-X', 'utf8', '-c', CHILD, name, str(frames), str(PAGE_S - 60.0)],
                                cwd=host,
                                env=dict(os.environ, PYTHONPATH=host), stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, text=True, encoding='utf-8', errors='replace')
        try:
            err = proc.communicate(timeout=PAGE_S)[1]
        except subprocess.TimeoutExpired:
            proc.kill()
            err = 'cut at %.0f s\n' % PAGE_S + proc.communicate()[1]
    found = re.findall(r'RESULT(\{.*\})', err)
    if found:
        return json.loads(found[-1])
    stuck = [line.strip() for line in err.splitlines() if line.strip().startswith('File ')]
    return 'stuck at %s' % ' <- '.join(stuck[:3]) if stuck else (err.strip().splitlines() or ['no result'])[-1]


def pages(names, short=200, long=1000, host=HOST):
    """{page: {kb, objs: a frame's growth from `short` to `long` frames, grown: [(kind, count)], mb} or {why}},
    every page drawn at both lengths in processes of their own, as many at once as the ledger admits."""
    jobs = [(name, n) for name in names for n in (short, long)]
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as pool:
        got = dict(zip(jobs, pool.map(lambda job: _page(host, *job), jobs)))
    rows, span = {}, float(long - short)
    for name in names:
        a, b = got[(name, short)], got[(name, long)]
        if not (isinstance(a, dict) and isinstance(b, dict)):
            rows[name] = {'why': a if isinstance(a, str) else b}
            continue
        grown = sorted(((b['kinds'].get(k, 0) - a['kinds'].get(k, 0), k) for k in b['kinds']), reverse=True)[:3]
        rows[name] = {'kb': 1024.0 * (b['mb'] - a['mb']) / span, 'objs': (b['objs'] - a['objs']) / span,
                      'grown': [(k, d) for d, k in grown if d > 0], 'mb': b['mb']}
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('what', choices=('relays', 'recordings', 'pages'))
    ap.add_argument('names', nargs='*', help='the pages drawn (pages); every page the tool draws where none')
    ap.add_argument('--frames', nargs=2, type=int, default=(200, 1000), metavar=('SHORT', 'LONG'))
    ap.add_argument('--relays', type=int, default=RELAYS)
    ap.add_argument('--passes', type=int, default=PASSES)
    ap.add_argument('--against', help="another tree: the same measured on its host/ first, then this one's")
    ap.add_argument('--host', default=HOST, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.against:
        again = [args.what] + args.names + ['--frames', str(args.frames[0]), str(args.frames[1]),
                                            '--relays', str(args.relays), '--passes', str(args.passes)]
        for tree in (args.against, os.path.dirname(HOST)):
            print('== %s' % tree, flush=True)
            subprocess.run([sys.executable, '-X', 'utf8', os.path.abspath(__file__)] + again
                           + ['--host', os.path.join(tree, 'host')], check=False)
        return 0
    sys.path.insert(0, args.host)
    if args.what == 'relays':
        r = relays(args.relays, args.passes)
        print('relays: %d worlds laid anew, %d passes each: %+.2f MB a relay on the loop\'s own collector, %+.2f MB '
              'collected through; of the %d worlds before, %d models and %d bodies held; a relay %.0f ms'
              % (args.relays, args.passes, r['own'], r['through'], args.relays, r['held'][0], r['held'][1], r['lay_ms']))
    elif args.what == 'recordings':
        r = recordings()
        print('recordings: 3 begun and ended: %d hooks left on the robot\'s passes, %d writer processes left, '
              'written %s bytes' % (r['hooks'], r['writers'], r['sizes']))
    else:
        from tools.render import page
        names = args.names or sorted(page.PAGES)
        print('%-18s %10s %11s %9s  %s' % ('page', 'kB/frame', 'objs/frame', 'MB', 'grown most'))
        for name, row in pages(names, args.frames[0], args.frames[1], args.host).items():
            if 'why' in row:
                print('%-18s %s' % (name, row['why'][:100]))
            else:
                print('%-18s %+10.1f %+11.2f %9.0f  %s' % (name, row['kb'], row['objs'], row['mb'],
                                                           ', '.join('%s +%d' % kd for kd in row['grown'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
