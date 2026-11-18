"""The machine's ledger every relay and container shares: a container's booking, the view of it all.

    with ledger.reserved(cores, gb, name) as booking:   # a container's cores and commit, waited for, held while it runs
    python -X utf8 tools/dev/run_tests.py --ledger    # the machine: its cores kept and taken, each relay's jobs, each container

The relay books each job in LEDGER while it runs and admits one only while every relay's live job processes stay
under the batch's width - the cores less those kept for the desk (`background.widths`) - and the commit holds
(`focus.relay`). A container of the gate on Linux (tools/dev/linux.py), x86 or arm64 under QEMU, is booked here the
same way, its cpuset's cores counted beside the jobs' processes, and so are a job sent an emulator's cores
(`Job.cores`, in place of its processes) and an emulator that took its own (tools.emu.probes.Cored): so a
company of agents' relays, containers and emulators on one
desk share its cores and leave the desk its own, each waiting its turn instead of taking the whole machine. Each
booking names the tree its relay runs, each container its relay (`clean`: what relays that are gone left, removed).
"""
import json
import os
import re
import time
from contextlib import suppress

from tools.dev import background, census
from tools.dev.focus import LEDGER, POLL_S, RESERVE_GB, _Locked, booked, queued, tally, waiting


def short(gb, room):
    """What a request that waits says beside its cores where the commit is what it waits for: `gb` asked of the
    `room` free over what is owed and the reserve (None: untold); '' where the commit holds it. A container waited
    153 s for 2 cores with 6 free: 5.0 GB of commit was asked, 3.5 free."""
    if room is None or room >= gb:
        return ''
    return ', short of commit (%.1f GB asked, %.1f free over the reserve)' % (gb, room)


class reserved:
    """`cores` of the machine and `gb` of its commit booked as `name` in the ledger while the block runs, waited for
    till they fit: the width (`background.widths`, or `width`) less what is taken and what the wide requests waiting
    since before it wait for (`focus.queued`: its turn kept, theirs left), and the free commit (measured, or `free`)
    less what is owed and RESERVE_GB; asked again every POLL_S, `wait_s` at most (None: as long as it takes) -
    `path` the booking, None where it never fit, `waited` the seconds it took; `say` told once, where the request
    must wait, what for (`short`)."""

    def __init__(self, cores, gb, name, ledger=LEDGER, width=None, wait_s=None, free=None, say=None):
        self.cores, self.gb, self.name, self.ledger = cores, gb, name, ledger
        self.width, self.wait_s, self.free, self.say = width, wait_s, free, say
        self.path, self.waited = None, 0.0

    def __enter__(self):
        started, since = time.monotonic(), time.time()
        ticket = os.path.join(self.ledger, '.wait-%d-%s' % (os.getpid(), self.name))
        while True:
            with _Locked():
                width = background.widths()[-1] if self.width is None else self.width
                used, owed, *_rest = tally(self.ledger)
                free = census.free_commit_gb() if self.free is None else self.free
                room = None if free is None else free - owed - RESERVE_GB
                if (used + self.cores + queued(self.ledger, ticket, since) <= width
                        and (room is None or room >= self.gb)):
                    self.path = os.path.join(self.ledger, '%d-%s' % (os.getpid(), self.name))
                    booked(self.path, self.gb, None, os.getpid(), self.cores)
                    waiting(ticket, 0)
                    self.waited = time.monotonic() - started
                    return self
                waiting(ticket, self.cores, since, self.gb)
            if self.say is not None:
                self.say('waiting for %d cores%s (run_tests.py --ledger)' % (self.cores, short(self.gb, room)))
                self.say = None
            if self.wait_s is not None and time.monotonic() - started >= self.wait_s:
                waiting(ticket, 0)
                return self
            time.sleep(POLL_S)

    def __exit__(self, *exc):
        if self.path is not None:
            with suppress(OSError):
                os.remove(self.path)
            self.path = None


def clean(ledger=LEDGER, client=None):
    """What a relay that is gone left, gone too: its bookings (`focus.tally` drops them), its tickets and half-written
    files, and the engine's containers of the gate it ran (their label coaxial.relay, `linux.ran`) - `client` the
    engine's where given, asked for where None, left alone where False or none answers. The lines that say what went."""
    gone = []
    tally(ledger)
    with suppress(OSError):
        for name in os.listdir(ledger):
            digits = re.search(r'(\d+)', name)
            if ((name.startswith('.wait-') or name.endswith('.part')) and digits
                    and not census.alive(int(digits.group(1)))):
                with suppress(OSError):
                    os.remove(os.path.join(ledger, name))
                gone.append('cleaned: %s of relay %s, gone' % (name, digits.group(1)))
    if client is False:
        return gone
    try:
        if client is None:
            import docker
            client = docker.from_env(timeout=5)
        for container in client.containers.list(filters={'label': 'coaxial.relay'}):
            pid = int(container.labels.get('coaxial.relay') or 0)
            if pid and not census.alive(pid):
                container.remove(force=True)
                gone.append('cleaned: container %s of relay %d, gone (%s)' % (
                    container.name, pid, os.path.basename(container.labels.get('coaxial.tree', ''))))
    except Exception:          # no engine here, or none answering: nothing of its to clean
        pass
    return gone


def said(ledger=LEDGER, width=None, free=None):
    """The machine as the ledger tells it, a line each: its cores, those kept for the desk, the width and what is
    taken of it; the commit free (measured, or `free`), owed and reserved; then each relay alive with its tree, jobs
    and commit, each container with its cores; last the queue - each relay waiting with its jobs, its first wide
    job and each container with their cores, the commit where that is what one waits for (`short`), and how long
    each has waited."""
    used, owed, _running, cores = tally(ledger)
    width = background.widths()[-1] if width is None else width
    free = census.free_commit_gb() if free is None else free
    room = None if free is None else free - owed - RESERVE_GB
    lines = ['machine: %d cores, %d kept for the desk, width %d: %d taken (%d jobs\' processes, %d cores booked by '
             'containers and emulators), %d free' % (census.physical_cores(), background.kept(), width, used,
                                                   used - cores, cores, max(0, width - used)),
             'commit: %s free, %.1f GB owed, %.1f reserved' % ('%.1f GB' % free if free is not None else 'untold', owed, RESERVE_GB)]
    relays, queue = {}, []
    with suppress(OSError):
        for name in sorted(os.listdir(ledger)):
            if name.startswith('.wait-') and not name.endswith('.part'):
                with suppress(OSError, ValueError), open(os.path.join(ledger, name), encoding='utf-8') as f:
                    held = json.load(f)
                    if census.alive(held['relay']):
                        what = name.split('-', 2)[2]
                        who = {'jobs': 'relay', 'wide': "a relay's wide job"}.get(what, 'container ' + what)
                        queue.append('waiting: %s (relay %s): %d %s%s, %.0f s' % (
                            who, held['relay'], held['count'], 'jobs' if what == 'jobs' else 'cores',
                            short(float(held.get('gb', 0.0)), room), time.time() - held['since']))
                continue
            if name.startswith('.'):
                continue
            try:
                with open(os.path.join(ledger, name), encoding='utf-8') as f:
                    job = json.load(f)
            except (OSError, ValueError):
                continue
            tree, what = os.path.basename(str(job.get('tree', ''))), name.split('-', 1)[1]
            if job.get('cores') and not what.isdigit():        # a relay's jobs are numbered, the rest named
                whose = ('emulator', 'process') if what.startswith('renode-') else ('container', 'relay')
                lines.append('%s %s (%s %s%s, %s): %d cores, %.1f GB' % (
                    whose[0], what, whose[1], job['relay'], '' if census.alive(job['relay']) else ', gone', tree,
                    job['cores'], job['gb']))
            else:
                jobs, gb, _tree, sent = relays.get(job['relay'], (0, 0.0, tree, 0))
                relays[job['relay']] = (jobs + 1, gb + job['gb'], tree, sent + int(job.get('cores') or 0))
    lines += ['relay %s%s (%s): %d jobs%s, %.1f GB' % (pid, '' if census.alive(pid) else ', gone', tree, jobs,
                                                       ' (%d cores with their emulators)' % sent if sent else '', gb)
              for pid, (jobs, gb, tree, sent) in sorted(relays.items())]
    return lines + queue
