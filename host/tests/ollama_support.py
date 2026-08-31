#!/usr/bin/env python3
"""Everything the split-out ollama suites share.

The fixtures, the scripted model, the report, and the helpers that
were top-level in test_ollama.py before it was one file per subject.
Nothing here is a check: every `test_` function moved to the file
named by its first subject.
"""
#!/usr/bin/env python3
"""Offline test of the Ollama runner: no board, no ollama, no network.

The board and the model are both simulated, on purpose. What is under test
here is not whether a language model can read a thermistor - that is what a
bench is for - but whether this runner keeps its promises when the model
behaves badly:

  * the limit never reaches the model;
  * the verdict never comes from the model;
  * a model that loops, answers in prose, calls a tool that does not exist or
    writes code that raises still leaves a complete transcript and a recorded
    step;
  * a state change cannot happen without the operator's flag.

Every one of those is a property of this package, so every one of them is
testable on a desk with nothing plugged in.

Imported by the test_ollama_* suites; not a suite itself.
"""
# Every import here is also the suites' import: test_ollama_* take io,
# json, simulated, detail and the rest FROM this module, so pyflakes'
# 'unused' on any of them is wrong - removing eight crashed three suites.
import io                                                  # noqa: F401
import json                                                # noqa: F401
import os
import random
import sys
import tempfile                                            # noqa: F401
import types                                               # noqa: F401
import threading                                           # noqa: F401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from coaxial import simulated                              # noqa: E402,F401
from coaxial.errors import ConnectError, DeviceStateError   # noqa: E402
from tests import counts                                   # noqa: E402
from coaxial_ollama import plan as planmod                 # noqa: E402
from coaxial_ollama import replies                         # noqa: E402,F401
from coaxial_ollama import runner as runmod                # noqa: E402
from coaxial_ollama import tools as toolmod                # noqa: E402
from coaxial_ollama.sandbox import Scope, Shell            # noqa: E402
from coaxial_mcp import detail                             # noqa: E402,F401
BSLASH = chr(92)


# ---- the simulated bench ---------------------------------------------------

class SimulatedLink:
    def __init__(self, board):
        self.board = board
        self.stats_reads = 0

    def echo(self, data):
        return data

    def stats(self):
        self.stats_reads += 1
        if self.board.broken:
            raise ConnectError('cable pulled')
        if self.board.dead_handle:
            # Distinct from `broken`: a real cable pull can leave the OS
            # handle Session.board cached permanently invalid, since a USB
            # VCP re-enumerates on replug rather than reviving the same
            # handle - measured directly against real hardware. Unlike
            # `broken`, nothing but session.reset() clears this one; the
            # tests using it are the ones proving debug.py actually calls
            # reset() rather than just retrying on the same dead handle.
            raise ConnectError('Attempting to use a port that is not open')
        return {'unit_id': 1, 'bus_message': 42, 'char_overrun': 0}
class SimulatedSystem:
    def self_test(self):
        return [{'name': 'PLL lock', 'status': 'pass', 'value': 0},
                {'name': 'ADC calibrated', 'status': 'pass', 'value': 0},
                {'name': 'flash checksum', 'status': 'info', 'value': 0x1234}]

    def clock(self):
        return {'sysclk_hz': 475000000, 'hclk_hz': 237500000, 'source': 'PLL1'}
class SimulatedAfe:
    def __init__(self):
        self.on = False

    def state(self):
        return {'on': self.on, 'pe15': not self.on}

    def is_on(self):
        """The third stand-in for this subsystem, and it was missing this.

        The structure suite checks the library's against the real class; it
        cannot see one that lives in a test.
        """
        return self.on

    def enable(self):
        self.on = True
        return True

    def disable(self):
        self.on = False
        return False

    def toggle(self):
        self.on = not self.on
        return self.on

    def require(self):
        if not self.on:
            raise DeviceStateError('the analog front end is off')
CHANNELS = [
    {'index': 0, 'adc': 1, 'pin': 'PA0', 'differential': True, 'signal': 'Phase U'},
    {'index': 1, 'adc': 1, 'pin': 'PA1', 'differential': True, 'signal': 'Phase V'},
    {'index': 2, 'adc': 3, 'pin': 'PC0', 'differential': False, 'signal': 'NTC'},
    {'index': 3, 'adc': 3, 'pin': 'PC1', 'differential': False, 'signal': 'DC bus'},
]
class SimulatedAnalog:
    """A four-channel board, smaller than the package's stand-in on purpose.

    It shares a NAME with `coaxial.simulated.SimulatedAnalog` and not a
    class, which is worth knowing: a method added there does not arrive here,
    and the AttributeError says `SimulatedAnalog` either way. Adding one to
    the real subsystem means adding it to both stand-ins or to neither.
    """

    def __init__(self, board):
        self.board = board

    def scaling(self, refresh=False):
        """The conversion parameters, as the real subsystem reports them.

        The fallback set, because there is no calibration record behind a
        four-channel double - and saying so is the point: a value cooked here
        is the schematic's arithmetic, not a board's.
        """
        del refresh
        from coaxial import scaling as _scaling
        return _scaling.from_calibration({})

    def channels(self, refresh=False):
        return CHANNELS

    def burst(self, mask, samples, rate=None):
        if self.board.broken:
            raise ConnectError('cable pulled mid-burst')
        if self.board.dead_handle:
            raise ConnectError('Attempting to use a port that is not open')
        chosen = {}
        for channel in CHANNELS:
            if mask >> channel['index'] & 1:
                chosen[channel['index']] = {'mean_raw': 32768.0 + channel['index'],
                                            'min_raw': 32700, 'max_raw': 32800}
        return {'samples': samples, 'rate_hz': rate or 0.0, 'channels': chosen}
class SimulatedBoard:
    def __init__(self):
        # A cable pull is a transport-level failure: it takes down every
        # subsystem's calls at once, not just the one a test happens to be
        # driving - so this lives here, not on SimulatedAnalog alone, and
        # SimulatedLink fails the same way analog does. One flag, shared, the
        # way a real dead Transport actually behaves.
        self.broken = False
        self.dead_handle = False
        self.link = SimulatedLink(self)
        self.system = SimulatedSystem()
        self.afe = SimulatedAfe()
        self.analog = SimulatedAnalog(self)

    def close_binary(self):
        pass
class SimulatedSession:
    def __init__(self):
        self.board = SimulatedBoard()
        self.resets = 0

    def info(self, refresh=False):
        version = {'device': 'coaxial_63100', 'mcu': 'STM32H753',
                   'firmware': '2.0.0', 'proto_major': 2, 'proto_minor': 1,
                   'build': 'test', 'commands': 21}
        return version, self.board.system.clock(), CHANNELS

    def reset(self):
        self.resets += 1
        self.board.dead_handle = False

    def close(self):
        pass
class ScriptedModel:
    """An Ollama stand-in that replays a list of assistant messages."""

    def __init__(self, turns, model='scripted', num_ctx=8192):
        self.turns = list(turns)
        self.model = model
        self.prompts = []          # every messages list it was handed
        # A real window, so every test that drives a Chat or a Runner through
        # this stand-in also proves the prompt those loops build actually fits
        # one - see context.py. A client with no options at all is a separate
        # case and is tested directly.
        self.options = {'num_ctx': num_ctx, 'num_predict': 300}
        self.notes = []

    def chat(self, messages, tools=None):
        self.prompts.append([dict(m) for m in messages])
        self.tools = tools
        if not self.turns:
            return {'role': 'assistant', 'content': 'I have nothing more.'}
        return dict(self.turns.pop(0))

    def usage(self):
        return {'calls': len(self.prompts), 'prompt_tokens': 0,
                'eval_tokens': 0}
def call(name, **args):
    """One tool call in the shape Ollama emits."""
    return {'role': 'assistant', 'content': '',
            'tool_calls': [{'function': {'name': name, 'arguments': args}}]}
class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-44s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-44s %s' % (name, detail))
def build(tasks, turns, allow_writes=False, confirm=None, allow=('python',),
          transcript=None, allow_code=True):
    plan = planmod.Plan({
        'product': 'coaxial_63100 BLDC inverter', 'revision': 'test',
        'plan_version': 'test', 'measurement_system_study': 'none, a unit test',
        'tasks': tasks})
    session = SimulatedSession()
    toolbox = toolmod.Toolbox(session, shell=Shell(allow, timeout=60),
                              scope=Scope(), allow_writes=allow_writes,
                              allow_code=allow_code, confirm=confirm)
    model = ScriptedModel(turns)
    runner = runmod.Runner(plan, model, toolbox,
                           transcript=runmod.Transcript(transcript), echo=False)
    return runner, model, session
class _Held:
    """A session on a port something else has open."""
    port, baud, unit = 'COM_TEST', 115200, 1
    _board = None
class _NotATty:
    """A pipe, for _printable: records what it was reconfigured to."""
    asked = None

    def isatty(self):
        return False

    def reconfigure(self, **kw):
        _NotATty.asked = kw
def _flat(text):
    """Text with Python's string quotes and line breaks taken out, so a
    literal split across three source lines still reads as one run."""
    for mark in (chr(39), chr(34), chr(92)):      # quotes and a continuation
        text = text.replace(mark, ' ')
    return ' '.join(text.split())
def _unformat(template):
    """The longest literal run of a template - what to look for in the source,
    since the source may split a string across lines and %-specs sit where the
    values go."""
    import re
    return max(re.split(r'%(?:\.\d+)?[a-z]', template), key=len)
def _capability_tags(report, cap, machine):
    """Which tag a machine chooses, and why it never splits one."""

    # A workstation card: the biggest tag that fits WHOLE, never a split one.
    # Measured on the bench: wholly resident is ~5x faster per token than half.
    big = cap.choose(machine(32, 64, 16))
    report.check('16 GB card takes the largest model that fits whole',
                 big.tag == 'qwen2.5:14b' and 'num_gpu' not in big.options,
                 big.tag)

    # A decent laptop: 8 GB card, 2 GB reserve, 6 GB to spend.
    laptop = cap.choose(machine(8, 32, 8))
    report.check('8 GB card drops to a model that fits it',
                 laptop.entry['gb'] <= 6.0 and 'num_gpu' not in laptop.options,
                 laptop.tag)

    # No GPU at all: everything on the CPU, and it says so rather than
    # recommending something that cannot load.
    headless = cap.choose(machine(64, 128, 0))
    report.check('no GPU means num_gpu 0', headless.options.get('num_gpu') == 0,
                 headless.tag)
    report.check('no GPU is explained, not silently slow',
                 'CPU' in headless.why or 'cpu' in headless.why)

    # Under-specified: 4 GB of RAM cannot hold anything in the catalogue.
    tiny = cap.choose(machine(2, 4, 0))
    report.check('a machine too small for any of them says so',
                 any('under-specified' in w for w in tiny.warnings))

    # Capability mode is allowed to spill onto the CPU, and must warn that it
    # costs - otherwise it is a slow default nobody chose.
    step = cap.choose(machine(32, 64, 16), prefer='capability')
    report.check('capability mode reaches for the bigger model',
                 step.tag == 'qwen2.5:32b' and step.options['num_gpu'] > 0,
                 '%s num_gpu=%s' % (step.tag, step.options.get('num_gpu')))
    report.check('and says what it costs',
                 any('slower' in w for w in step.warnings))

    # The reserve is the whole point of the budget: a card must never be
    # filled to the brim, because the desktop lives there too.
    report.check('a quarter of the card is held back, floor 2 GB',
                 cap.reserve_for(16) == 4.0 and cap.reserve_for(4) == 2.0
                 and cap.reserve_for(0) == 0.0)

    # And what the card already holds counts. Measured on this bench: a
    # two-screen desktop was using 2.6 GB before anything of ours ran, so a
    # flat quarter of a 16 GB card left it 1.4 GB to grow into - which is not
    # an error, it is a stutter, which is worse because nobody can read it.
    report.check('what the desktop already uses raises the reserve',
                 cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB,
                 '%.1f GB' % cap.reserve_for(16, 2.6))
    report.check('an empty card still gets the flat reserve',
                 cap.reserve_for(16, 0.1) == 4.0)
    report.check('the reserve never exceeds the card',
                 cap.reserve_for(8, 20) <= 8 + cap.HEADROOM_GB)

    import os
    os.environ[cap.RESERVE_ENV] = '8'
    try:
        report.check('a machine can say how much to hold back',
                     cap.reserve_for(16, 2.6) == 8.0)
        stingy = cap.choose(machine(32, 64, 16))
        report.check('and that changes the model, not just the number',
                     stingy.entry['gb'] <= 8.0, stingy.tag)
        os.environ[cap.RESERVE_ENV] = 'nonsense'
        report.check('an unreadable override falls back rather than crashing',
                     cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB)
    finally:
        del os.environ[cap.RESERVE_ENV]
    report.check('without the override the measurement decides again',
                 cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB)
    fits = [e for e in cap.CATALOGUE if e['gb'] <= 16 - cap.reserve_for(16)]
    report.check('nothing recommended for a 16 GB card fills it',
                 all(e['gb'] < 16 for e in fits))



def _capability_budget(report, cap, machine):
    """The reserve, the layer arithmetic, and the ratchet."""
    # Layer arithmetic: as many as the budget holds, never more than the model
    # has, never negative.
    for vram, ram in ((10, 32), (6, 16), (24, 64), (0, 64)):
        picked = cap.choose(machine(16, ram, vram), prefer='capability')
        layers = picked.options.get('num_gpu')
        if layers is not None:
            report.check('layer count stays inside the model (%d GB card)' % vram,
                         0 <= layers <= picked.entry['layers'],
                         '%s %s/%s' % (picked.tag, layers, picked.entry['layers']))

    # Free RAM, not the sticker. A 64 GB workstation with 8 GB left cannot hold
    # a 42 GB model, and the failure mode is swapping rather than an error.
    squeezed = cap.choose(machine(32, 64, 0, free=8))
    report.check('the choice is made on free RAM, not installed RAM',
                 squeezed.entry['ram_gb'] <= 8, squeezed.tag)
    report.check('a roomy machine still reaches the big models',
                 cap.choose(machine(32, 64, 0, free=64)).entry['gb']
                 >= squeezed.entry['gb'])

    # A busy machine gets a warning, not a different tag: load now says nothing
    # about load in ten minutes, and the tag is chosen for the session.
    busy = cap.choose(machine(32, 64, 0, free=64, busy=90))
    idle = cap.choose(machine(32, 64, 0, free=64, busy=1))
    report.check('a busy machine is warned, not quietly downgraded',
                 busy.tag == idle.tag
                 and any('busy' in w for w in busy.warnings)
                 and not any('busy' in w for w in idle.warnings))

    # The ratchet: our own resident model must not count as somebody's desktop,
    # or every run reserves more than the last and walks the choice downhill.
    real_smi, real_ours = cap._gpus_nvidia_smi, cap._ollama_vram_gb
    try:
        # A 16 GB card reading 10.8 used, of which 7.8 is the model we loaded
        # ourselves. The desktop is 3.0, and that is what the reserve is for.
        cap._gpus_nvidia_smi = lambda: [{'name': 'test', 'vram_gb': 16.0,
                                         'used_gb': 10.8, 'via': 'test'}]
        cap._ollama_vram_gb = lambda host='': 7.8
        probed = cap.probe()
        report.check("ollama's own VRAM is not counted as the desktop's",
                     abs(probed.vram_used_gb - 3.0) < 0.01,
                     '%.1f GB' % probed.vram_used_gb)
        report.check('and the probe says it did that',
                     'minus ollama' in probed.notes['gpu'])
        report.check('so the reserve does not ratchet upwards',
                     cap.reserve_for(16, probed.vram_used_gb)
                     < cap.reserve_for(16, 10.8),
                     '%.1f against %.1f' % (cap.reserve_for(16, probed.vram_used_gb),
                                            cap.reserve_for(16, 10.8)))

        cap._ollama_vram_gb = lambda host='': 0.0
        report.check('a card with nothing of ours on it is unchanged',
                     abs(cap.probe().vram_used_gb - 10.8) < 0.01)
    finally:
        cap._gpus_nvidia_smi, cap._ollama_vram_gb = real_smi, real_ours

    report.check('every candidate is tools-capable by construction',
                 all(entry.get('note') is not None for entry in cap.CATALOGUE))

    # And the probe itself has to survive whatever it finds, on any OS.
    found = cap.probe()
    report.check('probe returns a usable machine',
                 found.threads >= 1 and found.cores >= 1 and found.ram_gb >= 0)
    report.check('probe records how it measured each number',
                 set(found.notes) == set(['cpu', 'ram', 'gpu']), str(found.notes))
    report.check('report is text a human can read',
                 'machine:' in cap.report(found) and 'model:' in cap.report(found))


def _test_capability(report, cap):
    """The capability picker, in two halves.

    Split at 146 lines because the structure suite covers this file - it
    did not cover the one file these used to live in, where a 756-line
    check sat unnoticed.
    """
    def machine(cores, ram, vram, name='card', used=0.0, free=None, busy=None):
        gpus = ([{'name': name, 'vram_gb': vram, 'used_gb': used, 'via': 'test'}]
                if vram else [])
        return cap.Machine(cores=cores, threads=cores * 2, ram_gb=ram,
                           ram_free_gb=ram if free is None else free,
                           cpu_busy=busy, gpus=gpus, system='test', notes={})

    _capability_tags(report, cap, machine)
    _capability_budget(report, cap, machine)
def safe_head(text, n=44):
    return (text or '').strip().splitlines()[0][:n] if (text or '').strip() else '(nothing)'


#: The subjects a change can be about. The catalogue lives here rather than
#: beside any one suite, because pick_tests.py asks the model to choose from
#: it and every split file is named after one of them.
TAGS = {
    'prompt': 'SYSTEM, the per-turn hints, what the model is told',
    'tools': 'the tool surface: schemas, arguments, which tool answers what',
    'reply': 'what an answer means: retypes, blank answers, nudges',
    'language': 'the session language, its lock, and the phrase table',
    'render': 'how a result reaches the screen: columns, blocks, clipping',
    'board': 'the board, its channels, its pins, the AFE',
    'bus': 'nodes, segments, unit ids, broadcast',
    'link': 'the serial link: ports, probing, diagnosis, recovery',
    'runner': 'the plan runner, the sandbox, and the test tooling itself',
}


def select(roster, chosen, seed, coverage=None):
    """Which of `roster` to run, as a set. Empty means the file whole.

    Three claims on the budget, in order, because they are not equally
    valuable:

    1. **One test from every subject the pick left out, and the smallest
       group from the pick itself.** The picker is a model and can be wrong
       the expensive way, by not thinking of a subject at all. This is the
       floor - if it alone overshoots the target it still runs, and the
       printed percentage says so.
    2. **The picked subjects**, largest group first, until the budget is spent.
    3. **Whatever else fits**, drawn at random from the remainder.

    Sizes come from counts.py, so a percentage is of checks, not of groups:
    the groups run from 2 checks to 77, and half the groups is not half the
    coverage. With nothing measured yet, the pick runs whole.

    `roster` is a parameter now rather than a module global: one file per
    subject means there is no single roster left to close over.
    """
    if not chosen:
        return set()

    rng = random.Random(seed)
    drawn = set()
    for tag in sorted(set(TAGS) - chosen):
        pool = [t for t, marks in roster
                if tag in marks and not (chosen & set(marks))]
        if pool:
            drawn.add(rng.choice(pool))

    picked = {t for t, marks in roster if chosen & set(marks)}
    sizes = counts.load().get('groups') or {}
    if not coverage or not sizes:
        return drawn | picked

    total = sum(sizes.get(t.__name__, 0) for t, _ in roster)
    budget = total * coverage / 100.0
    spent = sum(sizes.get(t.__name__, 0) for t in drawn)

    smallest = sorted(picked, key=lambda t: sizes.get(t.__name__, 0))
    if smallest:
        drawn.add(smallest[0])
        spent += sizes.get(smallest[0].__name__, 0)

    for test in sorted(picked, key=lambda t: -sizes.get(t.__name__, 0)):
        if test in drawn:
            continue
        cost = sizes.get(test.__name__, 0)
        if spent + cost > budget:
            continue
        drawn.add(test)
        spent += cost

    rest = [t for t, _ in roster if t not in drawn]
    rng.shuffle(rest)
    for test in rest:
        cost = sizes.get(test.__name__, 0)
        if spent + cost > budget:
            continue
        drawn.add(test)
        spent += cost

    return drawn


def run_file(roster, argv=None):
    """Run one subject file and print the tally it counted itself.

    The same shape every suite in this tree prints, because tools/run_tests.py
    parses it and never asks anybody to summarise anything.
    """
    picked = seed = coverage = only = None
    argv = list(sys.argv[1:] if argv is None else argv)
    while len(argv) > 1:
        if argv[0] == '--tags':
            picked = argv[1]
        elif argv[0] == '--seed':
            seed = int(argv[1])
        elif argv[0] == '--coverage':
            coverage = float(argv[1])
        elif argv[0] == '--only':
            only = argv[1]
        argv = argv[2:]

    report = Report()
    chosen = {t.strip() for t in (picked or '').split(',') if t.strip()}
    unknown = chosen - set(TAGS)
    if unknown:
        print('unknown tag(s): %s' % ', '.join(sorted(unknown)))
        print('have: %s' % ', '.join(sorted(TAGS)))
        return 2

    if seed is None:
        seed = random.randrange(10000)

    # `only` names test functions outright - the shortest way back after
    # changing one thing. It overrides tags, the draw and any coverage
    # target: asked for three tests, run three tests. A name this file does
    # not have is not an error here: with one file per subject the run_tests
    # caller offers the same names to every file, and the one that owns them
    # runs them.
    if only:
        want = {n.strip().lower().lstrip('-') for n in only.split(',')
                if n.strip()}
        drawn = {t for t, _ in roster if t.__name__.lower() in want
                 or t.__name__[5:].lower() in want}
        if not drawn:
            print('\n0 passed, 0 failed, 0 skipped')
            return 0
        chosen = set()
    else:
        drawn = select(roster, chosen, seed, coverage)

    ran, sizes = 0, {}
    for test, _marks in roster:
        if drawn and test not in drawn:
            continue
        was = report.passed + report.failed
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
        sizes[test.__name__] = report.passed + report.failed - was
        ran += 1

    # What did not run, in checks rather than groups - measured, not
    # guessed: every group records its own size as it goes, so a narrowed
    # run reads back what the skipped ones came to the last time they ran.
    left = [t.__name__ for t, _ in roster if drawn and t not in drawn]
    counts.record('groups', sizes)
    skipped, unmeasured = counts.missing('groups', left)

    if drawn:
        cover = 100.0 * (report.passed + report.failed) / max(
            1, report.passed + report.failed + skipped)
        print('\nran %d of %d groups%s, seed %d, %.0f%% of checks'
              % (ran, len(roster),
                 ': ' + ','.join(sorted(chosen)) if chosen else '',
                 seed, cover))

    print('\n%d passed, %d failed, %s%d skipped'
          % (report.passed, report.failed,
             '~' if unmeasured else '', skipped))
    return 1 if report.failed else 0
