#!/usr/bin/env python3
"""Read an ollama model's weight blob(s) into the OS file cache, then measure
whether ollama's own reported `load_duration` actually gets shorter for it.

Windows holds recently-read files in its standby list while nothing else needs
the RAM, so a normal `dbg`/`board_chat` session warms whatever it loads as a
side effect. This covers the case that does not: a model untouched for a while,
warmed on purpose rather than discovering the hard way that the standby list
let it go.

    python tools/warm_model.py llama3.1:8b             # warm it, then measure
    python tools/warm_model.py llama3.1:8b --measure-only   # skip the read,
                                                              # just time two loads
    python tools/warm_model.py llama3.1:8b --auto      # decide first, quietly

Nothing here changes what ollama does: a read of files it already owns, and a
timing measurement through the same `/api/chat` empty-message trick
`client.py`'s `preload()` uses. Both loads pass `keep_alive=0`, so the model is
left unloaded, as found.

--auto is the unattended one, from board_chat.ps1's preflight. Measured
2.8-2.9 GB/s reading these blobs on this machine - three NVMe/SSDs, nothing
spinning - so warming buys nothing and --auto skips it. A slower disk with RAM
to spare should not need a different flag: the decision comes from what is
measured on the machine, not from a constant.
"""
import argparse
import ctypes
import json
import os
import platform
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

API = 'http://localhost:11434'
MODEL_LAYERS = ('application/vnd.ollama.image.model',
                'application/vnd.ollama.image.projector')


def _ollama_dir():
    override = os.environ.get('OLLAMA_MODELS')
    return Path(override) if override else Path.home() / '.ollama' / 'models'


def _ram_gb():
    """(total, free) in GiB. Windows only - same call capability.py uses."""
    if platform.system() != 'Windows':
        return None, None

    class Status(ctypes.Structure):
        _fields_ = [('dwLength', ctypes.c_ulong),
                    ('dwMemoryLoad', ctypes.c_ulong),
                    ('ullTotalPhys', ctypes.c_ulonglong),
                    ('ullAvailPhys', ctypes.c_ulonglong),
                    ('ullTotalPageFile', ctypes.c_ulonglong),
                    ('ullAvailPageFile', ctypes.c_ulonglong),
                    ('ullTotalVirtual', ctypes.c_ulonglong),
                    ('ullAvailVirtual', ctypes.c_ulonglong),
                    ('ullAvailExtendedVirtual', ctypes.c_ulonglong)]
    status = Status()
    status.dwLength = ctypes.sizeof(Status)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
        return None, None
    return (status.ullTotalPhys / float(2 ** 30),
            status.ullAvailPhys / float(2 ** 30))


def _manifest_path(tag):
    name, _, version = tag.partition(':')
    return (_ollama_dir() / 'manifests' / 'registry.ollama.ai' / 'library'
            / name / (version or 'latest'))


def blob_paths(tag):
    """(path, size) for every blob that is actual model weight - not the
    template, license or params layers, which are a few KB each and not
    what a cold load waits on."""
    manifest = json.loads(_manifest_path(tag).read_text())
    blobs = _ollama_dir() / 'blobs'
    out = []
    for layer in manifest['layers']:
        if layer['mediaType'] in MODEL_LAYERS:
            digest = layer['digest'].replace('sha256:', 'sha256-')
            out.append((blobs / digest, layer['size']))
    return out


def warm(paths, chunk=64 * 1024 * 1024):
    """Read every byte, sequentially, discarded. The read is the point -
    Windows caches what it just read as long as nothing else claims the RAM
    back before the next load asks for it."""
    total = 0
    started = time.monotonic()
    for path, _ in paths:
        with open(path, 'rb', buffering=0) as handle:
            while handle.read(chunk):
                total += chunk
    return total, time.monotonic() - started


# Below this measured throughput, OS-cache warming is plausibly worth its
# own read: a modern NVMe/SSD clears it several times over (2.8-2.9 GB/s
# Measured - see the module docstring), a spinning disk does
# not (80-160 MB/s is typical). Deliberately not a media-type check: a
# `MediaType` lookup can be wrong or unavailable, and it answers a different
# question anyway - this machine's disks are all reported SSD, and still get
# measured rather than trusted, because "SSD" spans a wide enough speed
# range on its own to be the wrong signal here.
SLOW_DISK_MB_S = 1000.0
# Free RAM must be this many times the model's own size before warming is
# worth it - the same shape of margin capability.py leaves the desktop on
# the GPU side, applied to system RAM instead.
RAM_MARGIN = 1.3


def probe_read_speed(path, sample=128 * 1024 * 1024):
    """MB/s for one real, timed read of the front of `path` - never a guess
    from the disk's reported type. If the front of the file happens to
    already be cached from something else, this reads fast and says so
    honestly; that is still the right answer to "is warming worth it right
    now", just not to "what can this disk do cold"."""
    started = time.monotonic()
    read = 0
    with open(path, 'rb', buffering=0) as handle:
        while read < sample:
            chunk = handle.read(min(64 * 1024 * 1024, sample - read))
            if not chunk:
                break
            read += len(chunk)
    elapsed = time.monotonic() - started
    return (read / 2 ** 20) / elapsed if elapsed > 0 else float('inf')


def should_warm(tag):
    """(decide, reason, paths, total_gb). Touches disk only for one probe
    read of the largest blob - cheap regardless of which way it decides."""
    try:
        paths = blob_paths(tag)
    except FileNotFoundError:
        return False, 'no manifest for %r - is it pulled?' % tag, [], 0.0
    if not paths:
        return False, 'no model blob in the manifest', [], 0.0

    total_gb = sum(size for _, size in paths) / 2 ** 30
    largest = max(paths, key=lambda p: p[1])[0]
    mb_s = probe_read_speed(largest)

    if mb_s >= SLOW_DISK_MB_S:
        return (False, 'disk already does %.0f MB/s - caching would not '
                'help much' % mb_s, paths, total_gb)

    total_ram, free_ram = _ram_gb()
    if total_ram is None:
        return False, 'could not read free RAM on this platform', paths, total_gb
    needed = total_gb * RAM_MARGIN
    if free_ram < needed:
        return (False, 'only %.1f GB free, %.1f GB wanted with margin'
                % (free_ram, needed), paths, total_gb)

    return (True, '%.0f MB/s disk, %.1f GB free of %.1f' % (mb_s, free_ram,
                                                            total_ram),
            paths, total_gb)


def warm_if_worthwhile(tag, out=sys.stdout):
    """The --auto path: decide, act if it says to, one or two lines either
    way. Returns True if it actually warmed something."""
    decide, reason, paths, total_gb = should_warm(tag)
    if not decide:
        print('warm %s: skipped - %s' % (tag, reason), file=out)
        return False
    print('warm %s: %s - reading %.1f GB...' % (tag, reason, total_gb),
         file=out)
    _, read_s = warm(paths)
    print('warm %s: done in %.1fs' % (tag, read_s), file=out)
    return True


def _post(payload, timeout):
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(API + '/api/chat', data=data,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def measure_load(tag, timeout=180):
    """load_duration in seconds - ollama's own figure, in the response to a
    real generation. An empty message list (`client.py`'s own preload()
    trick) loads the model too, but takes a `done_reason: "load"` path that
    never reports load_duration at all - measured directly against this
    daemon, not assumed. `num_predict: 1` keeps the generation itself to a
    single token, so this is still overwhelmingly a load, not an answer."""
    reply = _post({'model': tag, 'messages': [{'role': 'user', 'content': 'hi'}],
                  'options': {'num_predict': 1}, 'keep_alive': '5s',
                  'stream': False}, timeout)
    _post({'model': tag, 'messages': [], 'keep_alive': 0, 'stream': False},
         timeout)
    return reply.get('load_duration', 0) / 1e9


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('tag')
    parser.add_argument('--measure-only', action='store_true',
                        help='skip the warming read, just time two loads')
    parser.add_argument('--auto', action='store_true',
                        help='decide from a measured disk speed and free '
                             'RAM, act on it, one or two lines - no '
                             'before/after load timing. What board_chat.ps1 '
                             'runs on its own.')
    args = parser.parse_args(argv)

    if args.auto:
        warm_if_worthwhile(args.tag)
        return 0

    try:
        paths = blob_paths(args.tag)
    except FileNotFoundError:
        print('ERR no manifest for %r - is it pulled? `ollama list`' % args.tag)
        return 1

    total_gb = sum(size for _, size in paths) / 2 ** 30
    total_ram, free_ram = _ram_gb()
    print('%s: %.1f GB across %d file(s)' % (args.tag, total_gb, len(paths)))
    if total_ram is not None:
        print('this machine: %.1f GB free of %.1f GB' % (free_ram, total_ram))
        if free_ram < total_gb * 1.2:
            print('WARNING: not much headroom above the model itself - '
                  'warming this may just evict something else.')

    try:
        print('measuring the load as it stands now...')
        before = measure_load(args.tag)
        print('  load_duration: %.2fs' % before)

        if not args.measure_only:
            print('reading %.1f GB to warm the OS file cache...' % total_gb)
            read_bytes, read_s = warm(paths)
            print('  read %.1f GB in %.1fs (%.0f MB/s)'
                  % (read_bytes / 2 ** 30, read_s,
                     read_bytes / 2 ** 20 / read_s if read_s else 0))

        print('measuring the load again...')
        after = measure_load(args.tag)
        print('  load_duration: %.2fs' % after)
    except urllib.error.URLError as exc:
        print('ERR could not reach ollama at %s: %s' % (API, exc))
        return 1

    if before > 0:
        delta = (before - after) / before * 100
        print('%.0f%% %s (%.2fs -> %.2fs)'
              % (abs(delta), 'faster' if after < before else 'slower',
                 before, after))
    return 0


if __name__ == '__main__':
    sys.exit(main())
