"""The loader: reads pages/, lists them on the front page, preloads, and runs the pick in-process."""
import argparse
import ctypes
import importlib
import inspect
import os
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
for _path in (HOST, os.path.join(HOST, 'tools')):
    if _path not in sys.path:
        sys.path.insert(0, _path)

#: A view's answer asking for the front page rather than the door -
#: screen.TO_MENU, spelled here so the loader imports no view to know it.
TO_MENU = 64
#: The front page answers with FIRST + the entry's index; a second
#: question's later options take codes past the list.
FIRST = 101


def pages():
    """Every page under pages/, in ORDER: the modules themselves."""
    folder = os.path.join(HERE, 'pages')
    found = [importlib.import_module('terminal.pages.' + name[:-3])
             for name in sorted(os.listdir(folder))
             if name.endswith('.py') and not name.startswith('_')]
    return sorted(found, key=lambda page: page.ORDER)


def listing():
    """What the front page draws and answers with, off the pages: ENTRIES as
    (key, headline, what); SUB as {entry: (caption, ((key, name, what,
    code), ...))} for a page with items; OPEN as {name: (entry, pick)},
    the views the front page reopens on; and PICKS, {code: (page, name)}
    - what each answer means.
    """
    entries, sub, opens, picks = [], {}, {}, {}
    listed = pages()
    extra = FIRST + len(listed)
    for i, page in enumerate(listed):
        entries.append((page.KEY, page.HEADLINE, page.WHAT))
        items = getattr(page, 'ITEMS', None)
        if not items:
            picks[FIRST + i] = (page, page.NAME)
            continue
        caption, options = items
        rows = []
        for k, (key, headline, what, name) in enumerate(options):
            if k == 0:
                code = FIRST + i
            else:
                code, extra = extra, extra + 1
            rows.append((key, headline, what, code))
            picks[code] = (page, name)
            opens[name] = (i, k)
        sub[i] = (caption, tuple(rows))
    return tuple(entries), sub, opens, picks


def names():
    """Every name a page or an item answers to, for the command line."""
    return [name for _page, name in listing()[3].values()]


def by_name(name):
    """(page, name) for a page's or an item's name."""
    for page, pick in listing()[3].values():
        if pick == name:
            return page, pick
    raise KeyError(name)


def call(main, argv):
    """A view's `main` with `argv` as its command line - handed the list
    where it takes one, put in sys.argv where it reads that."""
    if inspect.signature(main).parameters:
        return main(argv)
    saved, sys.argv = sys.argv, [main.__module__ + '.py'] + list(argv)
    try:
        return main()
    finally:
        sys.argv = saved


def common(args, hz=None):
    """The flags every view takes, off the loader's own: --port,
    --simulated, --frames, and the page's --hz."""
    argv = ['--port', args.port]
    if hz is not None:
        argv += ['--hz', str(hz)]
    if args.simulated:
        argv.append('--simulated')
    if args.frames:
        argv += ['--frames', str(args.frames)]
    return argv


def resident():
    """This process's resident set in words - what the preload holds."""
    try:
        if sys.platform == 'win32':
            class Counters(ctypes.Structure):
                _fields_ = [('cb', ctypes.c_ulong),
                            ('PageFaultCount', ctypes.c_ulong),
                            ('PeakWorkingSetSize', ctypes.c_size_t),
                            ('WorkingSetSize', ctypes.c_size_t),
                            ('QuotaPeakPagedPoolUsage', ctypes.c_size_t),
                            ('QuotaPagedPoolUsage', ctypes.c_size_t),
                            ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t),
                            ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                            ('PagefileUsage', ctypes.c_size_t),
                            ('PeakPagefileUsage', ctypes.c_size_t)]
            counters = Counters()
            counters.cb = ctypes.sizeof(Counters)
            # The pseudo-handle is -1 in a HANDLE's width; typed as ctypes'
            # default int it goes out truncated and the call answers
            # ERROR_INVALID_HANDLE - measured, 6.
            kernel32 = ctypes.windll.kernel32
            kernel32.GetCurrentProcess.restype = ctypes.c_void_p
            info = ctypes.windll.psapi.GetProcessMemoryInfo
            info.argtypes = (ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong)
            if info(kernel32.GetCurrentProcess(), ctypes.byref(counters),
                    counters.cb):
                return '%.0f mb' % (counters.WorkingSetSize / 2**20)
        else:
            with open('/proc/self/statm', encoding='ascii') as statm:
                pages_ = int(statm.read().split()[1])
                return '%.0f mb' % (pages_ * os.sysconf('SC_PAGE_SIZE') / 2**20)
    except (OSError, AttributeError, ValueError):
        pass
    return 'unmeasured'


def fresh():
    """A preload's state for the front page's readout to print."""
    return {'steps': [], 'status': 'idle'}


def preload(state, model=None):
    """Everything a page will ask for, into this process's memory, once and
    off the frame loop: the model's decimates - the preload's pickle
    taken as it stands when its stamp matches, else built and written for
    next time when the machine has the room - the outline's loops, the
    pre-scan's primitives, the shadow casters.
    """
    from coaxial import orientation
    from coaxial.graphics import creases, shading, stereotype, wireframe
    from coaxial.graphics import preload as pickled
    path = model or orientation.MODEL
    ram, disk = pickled.room()
    state.update(
        model='%s  %.1f mb' % (os.path.basename(path),
                               os.path.getsize(path) / 2**20),
        memory='unknown' if ram is None else '%.1f gb free' % (ram / 2**30),
        disk='unknown' if disk is None else '%.1f gb free' % (disk / 2**30),
        status='loading')

    def say(line):
        state['steps'] = state['steps'][-3:] + [line]

    began = time.perf_counter()
    if pickled.load(path) is None:
        why = pickled.refusal()
        if why is None:
            say('written: %.1f mb' % (pickled.build(path, progress=say) / 2**20))
        else:
            say(why)
    say('decimates: %d in memory' % len(wireframe._lods()))
    say('outline: %d loops' % len(creases._outline_source()[1]))
    say('primitives: %d' % len(stereotype._stereotypes()))
    shading._shadowmap((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    say('shadow casters')
    state['status'] = 'resident: %s, %.1f s' % (resident(),
                                                time.perf_counter() - began)


def front_page(args, opened, state):
    """The front page, here: its choice as the code it answers with."""
    import menu
    argv = ['--port', args.port]
    if args.simulated:
        argv.append('--simulated')
    if opened:
        argv += ['--open', opened]
    if args.frames:
        argv += ['--frames', str(args.frames)]
    return menu.main(argv, preload=state)


def run_page(page, name, args):
    """One page in this process: its answer."""
    try:
        return page.run(args, name)
    except KeyboardInterrupt:
        return 0
    except Exception:                                     # noqa: BLE001
        traceback.print_exc()
        print('\n  %s failed - its last lines above say why' % name)
        try:
            input('  Enter for the front page ')
        except EOFError:
            pass
        return TO_MENU


def leave(args):
    """On the way out: the session opened once more and whatever a page
    left running stopped, so "nothing was left running" is measured."""
    import show_session
    argv = ['--leave', '--port', args.port]
    if args.simulated:
        argv.append('--simulated')
    call(show_session.main, argv)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='the terminal: the front page and the live views')
    parser.add_argument('name', nargs='?', choices=names(),
                        help='straight to this page or item')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--simulated', action='store_true',
                        help='no cable: every value invented, and said so')
    parser.add_argument('--frames', type=int, default=0,
                        help='draw this many and exit - the smoke')
    args = parser.parse_args(argv)
    state = fresh()
    threading.Thread(target=preload, args=(state,), daemon=True).start()
    if args.frames and not args.name:
        return front_page(args, None, state)
    view = by_name(args.name) if args.name else None
    opened, code = None, 0
    while True:
        if view is None:
            picked = front_page(args, opened, state)
            if picked < FIRST:
                break
            view = listing()[3][picked]
        page, name = view
        code = run_page(page, name, args)
        opened = name if name in listing()[2] else None
        view = None
        if code != TO_MENU:
            break
    leave(args)
    return code
