"""What this machine can run, and which local model to run on it.

A bench PC is whatever was on the shelf. The same repository is cloned onto a
16-core laptop with 8 GB of VRAM and onto a Threadripper with a 4080, and
picking one model tag for both means one of them is either crawling or leaving
most of its hardware idle. So the machine is measured and the tag follows from
the measurement.

Three numbers decide it, and they are all measured rather than assumed:

  * VRAM, minus a reserve. The reserve is the point: a bench PC also drives the
    screens, and a model that fills the card to the brim makes the desktop
    stutter and eventually gets itself evicted. Default is a quarter of the
    card, floor 2 GB.
  * System RAM, which is what a model runs in when it does not fit the card.
  * Cores, which decide whether running off the GPU is merely slow or hopeless.

Two things were measured here rather than reasoned about, and both contradict
the advice usually given (RTX 4080 SUPER 16 GB, Threadripper 3970X, gemma4:12b
Q4_K_M, 48 layers, num_ctx 8192):

  * `num_gpu` does not need a Modelfile. It is an ordinary entry in `options`
    on a normal /api/chat call, which is better than a Modelfile because a
    Modelfile is a second tag to keep in step with the first.

        num_gpu default (48)   7.8 GB VRAM    64.3 tok/s
        num_gpu 24 (half)      4.3 GB VRAM    12.7 tok/s
        num_gpu 0 (CPU only)   0.0 GB VRAM     6.7 tok/s

  * So a hybrid split is expensive - five times slower for half the VRAM back -
    and on a card this size it is also unnecessary: the whole 12B model fits in
    7.8 GB and still leaves 8 GB free. That is why the rule below is "the
    largest model that fits *entirely* within the budget", and hybrid is what
    happens when nothing does, not something to reach for.

  * Raising `num_thread` on 64 threads bought nothing and cost a little:

        default   6.4 tok/s      32 threads   6.3 tok/s
        16        6.4 tok/s      64 threads   5.7 tok/s

    Decode is bandwidth-bound, not core-bound, and filling both SMT siblings of
    every core makes it worse. Nothing here sets num_thread; the daemon's own
    choice was as good as any tried against it.

Only tools-capable tags are candidates. Everything in coaxial_ollama reaches
the board through tool calls, so a tag without them cannot drive this bench at
all - it will describe a measurement instead of taking one.
"""
import ctypes
import json
import os
import platform
import re
import subprocess
import urllib.request

# Approximate resident size at Q4_K_M, in GB, and the layer count. Sizes are
# what the daemon actually reported where a tag was pulled here, and ollama's
# published figures elsewhere; they are used to compare candidates, not to
# promise an exact allocation.
#
# Every entry is tools-capable. That is the entry requirement, not a feature.
CATALOGUE = [
    {'tag': 'llama3.1:8b',  'gb': 4.9,  'layers': 32, 'ram_gb': 8,
     'note': 'small and quick; measured here inventing tool arguments - see FINDINGS'},
    {'tag': 'qwen2.5:7b',   'gb': 4.7,  'layers': 28, 'ram_gb': 8,
     'note': 'the small one to try when llama3.1 disappoints'},
    {'tag': 'gemma4:12b',   'gb': 7.8,  'layers': 48, 'ram_gb': 16,
     'note': 'this bench default: careful with tools, and it checks the AFE first'},
    {'tag': 'qwen2.5:14b',  'gb': 9.7,  'layers': 48, 'ram_gb': 16,
     'note': 'the balanced one on a 12 GB card'},
    {'tag': 'qwen2.5:32b',  'gb': 20.0, 'layers': 64, 'ram_gb': 32,
     'note': 'strong on code and logic; a workstation model'},
    {'tag': 'llama3.3:70b', 'gb': 42.0, 'layers': 80, 'ram_gb': 64,
     'note': 'only with 64 GB of system RAM behind it'},
]

DEFAULT_TAG = 'gemma4:12b'

# The largest model worth putting on a CPU, in GB. See choose() for the
# measurement this comes from.
CPU_CEILING_GB = 8.0


class Machine(object):
    """What was found, with how it was found kept alongside it."""

    def __init__(self, cores, threads, ram_gb, gpus, system, notes,
                 ram_free_gb=None, cpu_busy=None):
        self.cores = cores          # physical, when the OS will say
        self.threads = threads      # logical
        self.ram_gb = ram_gb
        self.ram_free_gb = ram_gb if ram_free_gb is None else ram_free_gb
        self.cpu_busy = cpu_busy    # percent, or None when not measured
        self.gpus = gpus            # [{'name':..., 'vram_gb':..., 'via':...}]
        self.system = system
        self.notes = notes          # how each number was arrived at

    @property
    def vram_used_gb(self):
        """What is on the largest card already - the desktop, mostly."""
        if not self.gpus:
            return 0.0
        largest = max(self.gpus, key=lambda g: g['vram_gb'])
        return largest.get('used_gb', 0.0)

    @property
    def vram_gb(self):
        """The largest single card. Ollama does not split one model across two
        by default, so the total across cards would be a number that flatters
        the machine without helping it."""
        if not self.gpus:
            return 0.0
        return max(g['vram_gb'] for g in self.gpus)

    def as_dict(self):
        return {'cores': self.cores, 'threads': self.threads,
                'ram_gb': self.ram_gb, 'ram_free_gb': self.ram_free_gb,
                'cpu_busy': self.cpu_busy, 'gpus': self.gpus,
                'vram_gb': self.vram_gb, 'vram_used_gb': self.vram_used_gb,
                'system': self.system, 'notes': self.notes}

    def line(self):
        gpu = 'no GPU'
        if self.gpus:
            first = self.gpus[0]
            gpu = '%s %.0f GB' % (first['name'], first['vram_gb'])
            if len(self.gpus) > 1:
                gpu += ' (+%d more)' % (len(self.gpus) - 1)
        load = ''
        if self.cpu_busy is not None:
            load = ' (%.0f%% busy)' % self.cpu_busy
        return '%d cores / %d threads%s, %.0f GB RAM (%.0f free), %s' % (
            self.cores, self.threads, load, self.ram_gb, self.ram_free_gb, gpu)


class Choice(object):
    def __init__(self, tag, options, why, warnings=None, entry=None):
        self.tag = tag
        self.options = options      # merge into Ollama(options); may be empty
        self.why = why
        self.warnings = warnings or []
        self.entry = entry or {}

    def as_dict(self):
        return {'model': self.tag, 'options': self.options, 'why': self.why,
                'warnings': self.warnings}


# ---- measuring the machine -------------------------------------------------

def _cpu():
    """Physical cores if the OS will say, logical either way.

    Physical matters because the two numbers differ by 2x on anything with SMT,
    and a "64 core" machine that is really 32 is the sort of thing that makes a
    threading recommendation nonsense.
    """
    threads = os.cpu_count() or 1
    cores = threads
    note = 'os.cpu_count'
    if platform.system() == 'Windows':
        try:
            out = subprocess.check_output(
                ['powershell', '-NoProfile', '-Command',
                 '(Get-CimInstance Win32_Processor | '
                 'Measure-Object -Property NumberOfCores -Sum).Sum'],
                stderr=subprocess.DEVNULL, universal_newlines=True, timeout=30)
            value = int(out.strip())
            if value > 0:
                cores, note = value, 'Win32_Processor.NumberOfCores'
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
    return cores, threads, note


def _ram_gb():
    if platform.system() == 'Windows':
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
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return (status.ullTotalPhys / float(2 ** 30),
                    status.ullAvailPhys / float(2 ** 30), 'GlobalMemoryStatusEx')
        return 0.0, 0.0, 'GlobalMemoryStatusEx failed'
    try:
        size = os.sysconf('SC_PAGE_SIZE')
        total = os.sysconf('SC_PHYS_PAGES') * size / float(2 ** 30)
        try:
            free = os.sysconf('SC_AVPHYS_PAGES') * size / float(2 ** 30)
        except (ValueError, OSError, AttributeError):
            free = total
        return total, free, 'sysconf'
    except (ValueError, OSError, AttributeError):
        return 0.0, 0.0, 'unknown'


def _cpu_busy():
    """How much of the machine is already spoken for, as a percentage.

    Only ever a warning. A snapshot of CPU load is the wrong thing to choose a
    model tag with - the build that is running now finishes in a minute and the
    tag stays for the session - but it is the right thing to say out loud when
    the choice is about to be a CPU-bound one, because every tok/s figure in
    this file was measured on an idle machine.
    """
    if platform.system() != 'Windows':
        try:
            return min(100.0, 100.0 * os.getloadavg()[0] / (os.cpu_count() or 1))
        except (OSError, AttributeError):
            return None
    try:
        out = subprocess.check_output(
            ['powershell', '-NoProfile', '-Command',
             '(Get-CimInstance Win32_Processor | '
             'Measure-Object -Property LoadPercentage -Average).Average'],
            stderr=subprocess.DEVNULL, universal_newlines=True, timeout=30)
        return float(out.strip())
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def _ollama_vram_gb(host='http://localhost:11434'):
    """What ollama is holding on the card right now.

    This has to come off the 'already used' figure or the reserve ratchets: the
    probe runs while a model from the last question is still resident, counts
    our own 7.8 GB as somebody else's desktop, reserves 12.8 GB of a 16 GB card
    and picks something smaller - which then becomes the new baseline next
    time. Measured exactly that way before this function existed.
    """
    try:
        with urllib.request.urlopen(host.rstrip('/') + '/api/ps',
                                    timeout=5) as reply:
            data = json.loads(reply.read().decode('utf-8'))
    except Exception:
        return 0.0
    return sum(entry.get('size_vram', 0) or 0
               for entry in data.get('models', [])) / float(2 ** 30)


def _gpus_nvidia_smi():
    """Cards, and what is already on them.

    `memory.used` matters as much as the total. A card is not empty before the
    model loads: measured on this bench, a two-screen Windows desktop with the
    usual browser and editor open was holding 2.6 GB of a 16 GB card at 0 %
    utilisation, before anything of ours ran. A reserve computed as a flat
    fraction of the total quietly assumes that space is free, and the machine
    pays for the assumption in compositor stutter rather than in an error.
    """
    try:
        out = subprocess.check_output(
            ['nvidia-smi', '--query-gpu=name,memory.total,memory.used',
             '--format=csv,noheader,nounits'],
            stderr=subprocess.DEVNULL, universal_newlines=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return []
    found = []
    for line in out.strip().splitlines():
        parts = line.split(',')
        if len(parts) < 2:
            continue
        try:
            total = float(parts[1].strip())
            used = float(parts[2].strip()) if len(parts) > 2 else 0.0
        except ValueError:
            continue
        found.append({'name': parts[0].strip(), 'vram_gb': total / 1024.0,
                      'used_gb': used / 1024.0, 'via': 'nvidia-smi'})
    return found


def _gpus_registry():
    """Windows, any vendor, when nvidia-smi is not the answer.

    qwMemorySize and not AdapterRAM: Win32_VideoController.AdapterRAM is a
    32 bit field and reports 4 GB for every card larger than that, which is
    exactly the range where this decision matters. Measured here: qwMemorySize
    16.0 GB, AdapterRAM 4.0 GB, on a 16 GB card.
    """
    if platform.system() != 'Windows':
        return []
    try:
        import winreg
    except ImportError:
        return []

    base = (r'SYSTEM\CurrentControlSet\Control\Class'
            r'\{4d36e968-e325-11ce-bfc1-08002be10318}')
    found = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as parent:
            for index in range(16):
                try:
                    name = winreg.EnumKey(parent, index)
                except OSError:
                    break
                if not re.match(r'^\d{4}$', name):
                    continue
                try:
                    with winreg.OpenKey(parent, name) as key:
                        raw, _ = winreg.QueryValueEx(
                            key, 'HardwareInformation.qwMemorySize')
                        if isinstance(raw, bytes):
                            raw = int.from_bytes(raw, 'little')
                        try:
                            label, _ = winreg.QueryValueEx(key, 'DriverDesc')
                        except OSError:
                            label = 'GPU ' + name
                        size = int(raw) / float(2 ** 30)
                        if size > 0.5:
                            found.append({'name': label, 'vram_gb': size,
                                          'via': 'registry qwMemorySize'})
                except OSError:
                    continue
    except OSError:
        return []
    return found


def probe(host='http://localhost:11434'):
    """Measure this machine: what it has, and what is left of it.

    The daemon is asked one question - what is it holding on the card - and a
    machine with no daemon simply reports zero. Nothing here touches the board.
    """
    cores, threads, cpu_note = _cpu()
    ram, ram_free, ram_note = _ram_gb()
    gpus = _gpus_nvidia_smi()
    how = 'nvidia-smi'
    if not gpus:
        gpus = _gpus_registry()
        how = 'registry' if gpus else 'none found'

    ours = _ollama_vram_gb(host)
    if ours > 0 and gpus:
        largest = max(gpus, key=lambda g: g['vram_gb'])
        largest['used_gb'] = max(0.0, largest.get('used_gb', 0.0) - ours)
        largest['ollama_gb'] = ours
        how += ' minus ollama'

    return Machine(cores=cores, threads=threads, ram_gb=ram,
                   ram_free_gb=ram_free, cpu_busy=_cpu_busy(), gpus=gpus,
                   system='%s %s' % (platform.system(), platform.machine()),
                   notes={'cpu': cpu_note, 'ram': ram_note, 'gpu': how})


# ---- choosing a model ------------------------------------------------------

# What the desktop is allowed to grow by, over what it is using right now: a
# second 4K surface, a video that starts playing, a browser tab with a canvas
# in it. Transient allocations are what the stutter is - the driver has to
# evict something to satisfy them, and the something is the model.
HEADROOM_GB = 2.0

# One machine's desktop is not another's. A workstation driving two 4K screens
# with a browser, an editor and a video call has a different transient appetite
# than a headless bench PC, and no formula gets both right. COAXIAL_VRAM_RESERVE_GB
# is how a machine says "hold back this much", once, for every entry point.
RESERVE_ENV = 'COAXIAL_VRAM_RESERVE_GB'


def reserve_for(vram_gb, used_gb=0.0):
    """VRAM this picks deliberately not to use.

    Three numbers, whichever is largest: a quarter of the card, 2 GB, or what
    the card is *already* holding plus room to grow. That third one is the one
    that matters on a workstation - measured here, the desktop alone was using
    2.6 GB, so a flat quarter of a 16 GB card left it 1.4 GB of slack for
    everything it might do next, which is not enough and shows up as momentary
    hangs rather than as an error anyone can read.
    """
    if vram_gb <= 0:
        return 0.0
    override = os.environ.get(RESERVE_ENV)
    if override:
        try:
            return max(0.0, min(float(override), vram_gb))
        except ValueError:
            pass
    # Clamped to the card. A reading where the card is already fuller than it
    # is large is not a reason to print a reserve larger than the hardware; it
    # is a reason for the budget to be zero, which sends the choice to the CPU
    # on its own.
    return min(vram_gb, max(2.0, vram_gb * 0.25, used_gb + HEADROOM_GB))


def choose(machine, prefer='speed', reserve_gb=None, catalogue=None):
    """Which tag to run, and with which options.

    prefer='speed'      the largest model that fits entirely in the VRAM budget.
    prefer='capability' allow a bigger model to hang half out of the card, which
                        measured five times slower per token here. Worth it when
                        the answer matters more than the wait; not by default.
    """
    catalogue = catalogue or CATALOGUE
    vram = machine.vram_gb
    if reserve_gb is None:
        reserve_gb = reserve_for(vram, machine.vram_used_gb)
    budget = max(0.0, vram - reserve_gb)

    # Free RAM, not installed RAM. A workstation with 64 GB and 20 free cannot
    # hold a 42 GB model however impressive the sticker is, and the failure
    # mode is the machine swapping rather than an error worth reading. Total is
    # the fallback for a probe that could not measure the free figure.
    ram = machine.ram_free_gb or machine.ram_gb
    fits = [e for e in catalogue if e['gb'] <= budget and e['ram_gb'] <= ram]
    if fits:
        best = max(fits, key=lambda e: e['gb'])
        why = ('%.0f GB card with %.1f GB already on it, %.1f GB held back for '
               'the desktop, so %.1f GB to spend: %s fits whole and runs '
               'entirely on the GPU'
               % (vram, machine.vram_used_gb, reserve_gb, budget, best['tag']))
        choice = Choice(best['tag'], {}, why, entry=best)
        if prefer != 'capability':
            return choice

        # Capability: is there a bigger one that RAM can hold, hybrid?
        bigger = [e for e in catalogue
                  if e['gb'] > best['gb'] and e['ram_gb'] <= machine.ram_gb]
        if not bigger:
            choice.warnings.append(
                'nothing larger fits this machine either way; speed and '
                'capability pick the same tag here')
            return choice
        step = min(bigger, key=lambda e: e['gb'])
        return _hybrid(step, budget, machine, vram, reserve_gb, [
            '%s would fit whole and run about five times faster per token; '
            'this is the capability choice, not the quick one' % best['tag']])

    # Nothing fits whole. Either hybrid, or the CPU.
    affordable = [e for e in catalogue if e['ram_gb'] <= ram]
    if not affordable:
        smallest = min(catalogue, key=lambda e: e['gb'])
        return Choice(smallest['tag'], {'num_gpu': 0},
                      'nothing in the catalogue fits %.0f GB of free RAM; %s '
                      'is the smallest and it will be tight'
                      % (ram, smallest['tag']),
                      warnings=['this machine is under-specified for a local '
                                'model - consider --ollama-host on a bench '
                                'server, or a smaller quantisation'],
                      entry=smallest)
    # What can run off the card is bounded by patience rather than by RAM. A
    # 12B model with nothing on the GPU managed 6.4 tok/s on 32 cores here, and
    # every size up is proportionally worse - a 32B on the CPU is a model you
    # ask one question a day. So the ceiling is a size, not a share of RAM.
    ceiling = max(budget + CPU_CEILING_GB, CPU_CEILING_GB)
    within = [e for e in affordable if e['gb'] <= ceiling]
    step = (max(within, key=lambda e: e['gb']) if within
            else min(affordable, key=lambda e: e['gb']))
    return _hybrid(step, budget, machine, vram, reserve_gb, [])


def _hybrid(entry, budget, machine, vram, reserve_gb, warnings):
    """As many layers on the card as the budget holds, the rest on the CPU."""
    per_layer = entry['gb'] / float(entry['layers'])
    layers = int(budget / per_layer) if per_layer > 0 else 0
    layers = max(0, min(entry['layers'], layers))

    warnings = list(warnings)
    if layers == 0:
        why = ('%.0f GB card leaves %.1f GB after the reserve, which holds no '
               'part of %s worth having: it runs on the CPU'
               % (vram, budget, entry['tag']))
        if machine.cores < 8:
            warnings.append('%d cores for a CPU-only model is going to be slow '
                            'enough to notice on every question' % machine.cores)
    else:
        why = ('%s does not fit %.1f GB whole, so %d of its %d layers go on the '
               'card and the rest on %d cores'
               % (entry['tag'], budget, layers, entry['layers'], machine.cores))
        warnings.append('a split model measured about five times slower per '
                        'token than one wholly on the GPU')
    # Every tok/s figure in this file was measured on an idle machine, and the
    # part of this choice that runs on the CPU is the part a busy machine slows
    # down. Say so rather than letting the numbers read as a promise.
    if machine.cpu_busy is not None and machine.cpu_busy >= 40:
        warnings.append('this machine is %.0f%% busy right now, and the CPU '
                        'half of this choice will be slower than the figures '
                        'in capability.py, which were measured idle'
                        % machine.cpu_busy)
    if entry.get('note'):
        warnings.append(entry['tag'] + ': ' + entry['note'])
    return Choice(entry['tag'], {'num_gpu': layers}, why, warnings, entry)


def report(machine=None, prefer='speed', reserve_gb=None):
    """The whole thing as text, for a human at a bench."""
    machine = machine or probe()
    choice = choose(machine, prefer=prefer, reserve_gb=reserve_gb)
    lines = ['machine: ' + machine.line(),
             '         (%s)' % ', '.join('%s via %s' % (k, v)
                                         for k, v in sorted(machine.notes.items())),
             '',
             'model:   ' + choice.tag]
    if choice.options:
        lines.append('options: ' + json.dumps(choice.options))
    lines.append('because: ' + choice.why)
    for warning in choice.warnings:
        lines.append('  note:  ' + warning)
    return '\n'.join(lines)


def pulled(host='http://localhost:11434'):
    """Tags already on this machine, so a recommendation can prefer one.

    Best effort: no daemon is not an error here, it just means nothing is
    pulled yet as far as this function knows.
    """
    try:
        with urllib.request.urlopen(host.rstrip('/') + '/api/tags',
                                    timeout=5) as reply:
            data = json.loads(reply.read().decode('utf-8'))
        return [entry['name'] for entry in data.get('models', [])]
    except Exception:
        return []


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m coaxial_ollama.capability',
        description='What this machine can run, and which local model to run.')
    parser.add_argument('--prefer', choices=('speed', 'capability'),
                        default='speed',
                        help='speed: the largest model that fits the card '
                             'whole. capability: allow a bigger one to spill '
                             'onto the CPU, measured ~5x slower per token.')
    parser.add_argument('--reserve-gb', type=float, default=None,
                        help='VRAM to hold back for the desktop. Overrides both'
                             ' the measured default and %s. Raise it if the'
                             ' desktop stutters while the model is answering.'
                             % RESERVE_ENV)
    parser.add_argument('--json', action='store_true',
                        help='machine readable, for setup.ps1')
    args = parser.parse_args(argv)

    machine = probe()
    if args.json:
        choice = choose(machine, prefer=args.prefer, reserve_gb=args.reserve_gb)
        out = choice.as_dict()
        out['machine'] = machine.as_dict()
        print(json.dumps(out, indent=2, sort_keys=True))
        return 0
    print(report(machine, prefer=args.prefer, reserve_gb=args.reserve_gb))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
