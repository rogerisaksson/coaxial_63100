"""What this machine can run, and which local model to run on it."""
import argparse
import json
import os
import platform
import re
import subprocess
import urllib.request
from contextlib import suppress

from coaxial.memory import physical

try:
    import winreg              # _adapters and _gpu_at read it too: module level, not a local
except ImportError:            # not Windows
    winreg = None

# Approximate resident size at Q4_K_M, in GB, and the layer count.
CATALOGUE = [
    {'tag': 'llama3.1:8b',  'gb': 4.9,  'layers': 32, 'ram_gb': 8,
     'note': 'small and quick; Measured inventing tool arguments - see FINDINGS'},
    {'tag': 'qwen2.5:7b',   'gb': 4.7,  'layers': 28, 'ram_gb': 8,
     'note': 'the small one to try when llama3.1 disappoints'},
    {'tag': 'gemma4:12b',   'gb': 7.8,  'layers': 48, 'ram_gb': 16,
     'note': 'the default: careful with tools, and it checks the AFE first'},
    {'tag': 'qwen2.5:14b',  'gb': 9.7,  'layers': 48, 'ram_gb': 16,
     'note': 'the balanced one on a 12 GB card'},
    {'tag': 'qwen2.5:32b',  'gb': 20.0, 'layers': 64, 'ram_gb': 32,
     'note': 'strong on code and logic; a workstation model'},
    {'tag': 'llama3.3:70b', 'gb': 42.0, 'layers': 80, 'ram_gb': 64,
     'note': 'only with 64 GB of system RAM behind it'},
]


# The largest model worth putting on a CPU, in GB.
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
        self.gpus: list = gpus      # [{'name':..., 'vram_gb':..., 'via':...}]
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
        """The largest single card."""
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
        gpu = _gpu_words(self.gpus)
        load = ''
        if self.cpu_busy is not None:
            load = ' (%.0f%% busy)' % self.cpu_busy
        return '%d cores / %d threads%s, %.0f GB RAM (%.0f free), %s' % (
            self.cores, self.threads, load, self.ram_gb, self.ram_free_gb, gpu)


class Choice(object):
    """Which tag to run and with which options, plus why - the reason is
    printed, because a picker nobody can question is a picker nobody
    trusts."""
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

def _gpu_words(gpus):
    """The first card and its memory, and how many more there are."""
    if not gpus:
        return 'no GPU'
    first = gpus[0]
    more = ' (+%d more)' % (len(gpus) - 1) if len(gpus) > 1 else ''
    return '%s %.0f GB%s' % (first['name'], first['vram_gb'], more)


def _windows_cores():
    """Physical cores off WMI, or None when PowerShell will not say."""
    try:
        out = subprocess.check_output(
            ['powershell', '-NoProfile', '-Command',
             '(Get-CimInstance Win32_Processor | '
             'Measure-Object -Property NumberOfCores -Sum).Sum'],
            stderr=subprocess.DEVNULL, universal_newlines=True, timeout=30)
        value = int(out.strip())
        return value if value > 0 else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def _cpu():
    """Physical cores if the OS will say, logical either way."""
    threads = os.cpu_count() or 1
    physical = _windows_cores() if platform.system() == 'Windows' else None
    if physical is None:
        return threads, threads, 'os.cpu_count'
    return physical, threads, 'Win32_Processor.NumberOfCores'


def _windows_ram():
    """Installed and available memory in GB (`coaxial.memory`)."""
    total, free = physical()
    if total is None or free is None:
        return 0.0, 0.0, 'GlobalMemoryStatusEx failed'
    return total / float(2 ** 30), free / float(2 ** 30), 'GlobalMemoryStatusEx'


def _ram_gb():
    if platform.system() == 'Windows':
        return _windows_ram()
    sysconf = getattr(os, 'sysconf', None)         # POSIX only
    try:
        if sysconf is None:
            raise AttributeError('no sysconf')
        size = sysconf('SC_PAGE_SIZE')
        total = sysconf('SC_PHYS_PAGES') * size / float(2 ** 30)
        try:
            free = sysconf('SC_AVPHYS_PAGES') * size / float(2 ** 30)
        except (ValueError, OSError, AttributeError):
            free = total
        return total, free, 'sysconf'
    except (ValueError, OSError, AttributeError):
        return 0.0, 0.0, 'unknown'


def _posix_busy():
    """The one-minute load average as a share of the cores, or None."""
    loadavg = getattr(os, 'getloadavg', None)     # POSIX only
    if loadavg is None:
        return None
    try:
        return min(100.0, 100.0 * loadavg()[0] / (os.cpu_count() or 1))
    except OSError:
        return None


def _cpu_busy():
    """How much of the machine is already spoken for, as a percentage."""
    if platform.system() != 'Windows':
        return _posix_busy()
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
    """What ollama is holding on the card right now."""
    try:
        with urllib.request.urlopen(host.rstrip('/') + '/api/ps',
                                    timeout=5) as reply:
            data = json.loads(reply.read().decode('utf-8'))
    except Exception:
        return 0.0
    return sum(entry.get('size_vram', 0) or 0
               for entry in data.get('models', [])) / float(2 ** 30)


def _gpus_nvidia_smi():
    """Cards, and what is already on them."""
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
    """Windows, any vendor, when nvidia-smi is not the answer."""
    if winreg is None:
        return []

    base = (r'SYSTEM\CurrentControlSet\Control\Class'
            r'\{4d36e968-e325-11ce-bfc1-08002be10318}')
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as parent:
            found = [_gpu_at(parent, name) for name in _adapters(parent)]
    except OSError:
        return []
    return [card for card in found if card]


def _registry():
    """winreg, where there is one; `gpus_from_registry` has checked."""
    if winreg is None:
        raise OSError('no registry on this platform')
    return winreg


def _adapters(parent):
    """The four-digit adapter subkeys, in order, until they run out."""
    reg = _registry()
    for index in range(16):
        try:
            name = reg.EnumKey(parent, index)
        except OSError:
            return
        if re.match(r'^\d{4}$', name):
            yield name


def _gpu_at(parent, name):
    """One adapter's card, or None for anything without usable VRAM."""
    reg = _registry()
    try:
        with reg.OpenKey(parent, name) as key:
            raw, _ = reg.QueryValueEx(
                key, 'HardwareInformation.qwMemorySize')
            if isinstance(raw, bytes):
                raw = int.from_bytes(raw, 'little')
            try:
                label, _ = reg.QueryValueEx(key, 'DriverDesc')
            except OSError:
                label = 'GPU ' + name
    except OSError:
        return None
    size = int(raw) / float(2 ** 30)
    if size <= 0.5:
        return None
    return {'name': label, 'vram_gb': size, 'via': 'registry qwMemorySize'}


def probe(host='http://localhost:11434'):
    """Measure this machine: what it has, and what is left of it."""
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
# in it.
HEADROOM_GB = 2.0

# One machine's desktop is not another's.
RESERVE_ENV = 'COAXIAL_VRAM_RESERVE_GB'


def reserve_for(vram_gb, used_gb=0.0):
    """VRAM this picks deliberately not to use."""
    if vram_gb <= 0:
        return 0.0
    override = os.environ.get(RESERVE_ENV)
    if override:
        with suppress(ValueError):
            return max(0.0, min(float(override), vram_gb))
    # Clamped to the card.
    return min(vram_gb, max(2.0, vram_gb * 0.25, used_gb + HEADROOM_GB))


def choose(machine, prefer='speed', reserve_gb=None, catalogue=None):
    """Which tag to run, and with which options."""
    catalogue = catalogue or CATALOGUE
    vram = machine.vram_gb
    if reserve_gb is None:
        reserve_gb = reserve_for(vram, machine.vram_used_gb)
    budget = max(0.0, vram - reserve_gb)

    # Free RAM, not installed RAM.
    ram = machine.ram_free_gb or machine.ram_gb
    fits = [e for e in catalogue if e['gb'] <= budget and e['ram_gb'] <= ram]
    if fits:
        return _fitting(fits, catalogue, machine, vram, reserve_gb, budget,
                        prefer)

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
    # What can run off the card is bounded by patience rather than by RAM.
    ceiling = max(budget + CPU_CEILING_GB, CPU_CEILING_GB)
    within = [e for e in affordable if e['gb'] <= ceiling]
    step = (max(within, key=lambda e: e['gb']) if within
            else min(affordable, key=lambda e: e['gb']))
    return _hybrid(step, budget, machine, vram, reserve_gb, [])


def _fitting(fits, catalogue, machine, vram, reserve_gb, budget, prefer):
    """The largest model that fits the card whole - or, asked for
    capability, the next size up that RAM can hold, hanging half out."""
    best = max(fits, key=lambda e: e['gb'])
    why = ('%.0f GB card with %.1f GB already on it, %.1f GB held back for '
           'the desktop, so %.1f GB to spend: %s fits whole and runs '
           'entirely on the GPU'
           % (vram, machine.vram_used_gb, reserve_gb, budget, best['tag']))
    choice = Choice(best['tag'], {}, why, entry=best)
    if prefer != 'capability':
        return choice
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


#: Fewer cores than this and a CPU-only model is slow enough to notice.
FEW_CORES = 8


def _hybrid(entry, budget, machine, vram, reserve_gb, warnings):
    """As many layers on the card as the budget holds, the rest on the CPU."""
    per_layer = entry['gb'] / float(entry['layers'])
    layers = int(budget / per_layer) if per_layer > 0 else 0
    layers = max(0, min(entry['layers'], layers))

    warnings = list(warnings)
    cpu_only = layers == 0
    why = (('%.0f GB card leaves %.1f GB after the reserve, which holds no '
            'part of %s worth having: it runs on the CPU'
            % (vram, budget, entry['tag'])) if cpu_only else
           ('%s does not fit %.1f GB whole, so %d of its %d layers go on the '
            'card and the rest on %d cores'
            % (entry['tag'], budget, layers, entry['layers'], machine.cores)))
    if cpu_only and machine.cores < FEW_CORES:
        warnings.append('%d cores for a CPU-only model is going to be slow '
                        'enough to notice on every question' % machine.cores)
    if not cpu_only:
        warnings.append('a split model measured about five times slower per '
                        'token than one wholly on the GPU')
    # Every tok/s figure in this file was measured on an idle machine, and the
    # part of this choice that runs on the CPU is the part a busy machine slows
    # down.
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
    """Tags already on this machine, so a recommendation can prefer one."""
    try:
        with urllib.request.urlopen(host.rstrip('/') + '/api/tags',
                                    timeout=5) as reply:
            data = json.loads(reply.read().decode('utf-8'))
        return [entry['name'] for entry in data.get('models', [])]
    except Exception:
        return []


def main(argv=None):
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
