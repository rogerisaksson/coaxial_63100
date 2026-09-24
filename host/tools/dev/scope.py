"""Which suites a change reaches: the files it touched, the tier, the model's pick."""
import subprocess

from coaxial_ollama import client as clientmod
from coaxial_ollama.capability import choose, probe
from tools.dev import pick_tests
from tools.dev.suites import (CHEAP, CONFORMANCE, DEFAULT_SUITES, FULL_EVERY, LIVE, OLLAMA, ROOT,
                              STRUCTURE, TOUCHES, plan_for)


# Every tag this run put on the card, so one `finally` can hand them back.
_LOADED = []


def hold_model(tag):
    """Load the model before the first suite that needs it."""
    try:
        client = clientmod.Ollama(tag, keep_alive='30m')
        client.model = client.require_model()
        _LOADED.append(client)
        print('holding %s for the run' % client.model)
        client.preload()
        return client
    except clientmod.FAULTS as exc:
        print('could not preload %s: %s' % (tag, exc))
        return None


def _client_for(tag):
    """A handle on a tag, for unloading it. None if ollama is not there."""
    try:
        return clientmod.Ollama(tag)
    except clientmod.FAULTS:
        return None


def release_model(client=None):
    """Hand the card back, once, when the run is over."""
    held = [client] if client is not None else list(_LOADED)
    del _LOADED[:]
    done = set()
    for one in held:
        if one is None or one.model in done:
            continue
        tag = one.model
        done.add(tag)
        try:
            one.unload()
            print('released %s' % tag)
        except clientmod.FAULTS as exc:
            print('could not release %s: %s' % (tag, exc))


def _within_tier(args, live_sections):
    """Hold the model's pick inside the tier's budget."""
    if not args.coverage:
        return live_sections

    allowed, sections = plan_for(args.coverage)
    keep = set(allowed) | {STRUCTURE} | set(OLLAMA)
    dropped = [name for name in args.file if name not in keep]

    args.file = [name for name in args.file if name in keep]
    if not sections and live_sections:
        dropped.append('live:' + live_sections)
        live_sections = ''

    args.live = bool(live_sections)

    if dropped:
        print('   the %d%% tier does not stretch to: %s'
              % (args.coverage, ', '.join(dropped)))
    return live_sections


def _ask_model(args, live_sections):
    """The model's own pick, held inside whatever tier is in force."""
    # The picker loads the model too.
    if args.model == 'auto':
        # The machine's own pick, resolved once - a hardcoded default tag asked
        # a 16 GB bench to test against a model an 8 GB one runs.
        try:
            args.model = choose(probe()).tag
        except (OSError, ValueError, KeyError, AttributeError,
                subprocess.SubprocessError):       # what the probe can meet
            args.model = 'gemma4:12b'
    plan, reason = pick_tests.pick(args.model)
    _LOADED.append(_client_for(args.model))

    if plan is None:
        print('   the model picked nothing: %s' % reason)
        print('   falling back to the path map above')
        return None, live_sections

    # Structure is not the model's to drop.
    args.file = [STRUCTURE] + [f for f in plan.suites
                               if f not in (LIVE, STRUCTURE)]
    tags = ','.join(plan.tags) or None
    live_sections = '' if plan.live == 'none' else plan.live
    args.live = bool(live_sections)

    print('   the model picked: %s%s'
          % (', '.join(args.file) or 'nothing',
             '  live:' + live_sections if live_sections else ''))
    print('   %s' % (plan.why or 'no reason given'))

    return tags, _within_tier(args, live_sections)


def settled(chosen, why):
    """True when the map knew every path and the answer costs seconds."""
    if any('unmapped' in line for line in why):
        return False

    # An empty pick counts.
    return set(chosen) <= CHEAP


def changed_files(against='HEAD'):
    """Paths touched in the working tree and in the last commit."""
    paths = set()
    for args in (['diff', '--name-only', against],
                 ['diff', '--name-only', '--cached'],
                 ['diff', '--name-only', '%s~1' % against, against]):
        try:
            done = subprocess.run(['git'] + args, cwd=str(ROOT.parent),
                                  capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
                                  timeout=30)
        except (OSError, subprocess.SubprocessError):
            continue
        if done.returncode == 0:
            paths |= {line.strip().replace('\\', '/')
                      for line in done.stdout.splitlines() if line.strip()}
    return sorted(paths)


def pick(paths):
    """(suites, live_sections, why) for these changed paths."""
    suites, live, why = set(), set(), []
    for path in paths:
        rule = next((wanted for prefix, wanted in TOUCHES
                     if path.startswith(prefix)), None)
        if rule is None:
            why.append('%s -> unmapped, running everything' % path)
            return set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'}, why
        _touched(path, rule, suites, live, why)
    return suites, live, why


def _touched(path, wanted, suites, live, why):
    """One changed path against its rule."""
    name = path.rsplit('/', 1)[-1]
    own = not wanted and path.startswith('host/tests/')
    if own and name.startswith('test_'):
        suites.add(name)
        why.append('%s -> itself' % path)
    if own:
        return
    live.update(item.split(':', 1)[1] for item in wanted
                if item.startswith('live:'))
    suites.update(item for item in wanted if not item.startswith('live:'))
    why.append('%s -> %s' % (path, ', '.join(wanted) or 'nothing reads it'))


def _plan(args):
    """Which suites, which subjects, which live sections."""

    live_sections = 'all'
    tags = args.tags
    if args.structure:
        args.file, args.smart, args.live = [STRUCTURE], False, False
    if args.match:
        # One live row and nothing else.
        args.file, args.smart, args.live = [LIVE], False, True
        live_sections = args.sections or 'all'
    if args.coverage:
        args.smart = True
    if args.only:
        # One file, the named tests, nothing else.
        args.file, args.smart, args.live = list(OLLAMA), False, False
    planned = (_smart(args, tags, live_sections)
               if args.smart and not args.file else (tags, live_sections))
    if planned is None:
        return None                # the plan was the whole point of the run
    tags, live_sections = planned

    # Typed explicitly, so it wins over the 'all' default and over a tier's own
    # pick.
    if args.sections and args.live:
        live_sections = args.sections

    return tags, live_sections


def _commits():
    """How many commits this tree has, or 0 outside git."""
    try:
        return int(subprocess.run(
            ['git', 'rev-list', '--count', 'HEAD'], cwd=str(ROOT.parent),
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', timeout=30).stdout.strip())
    except (OSError, ValueError, subprocess.SubprocessError):
        return 0


def _chosen(paths, full, count):
    """(suites, live sections, why) - everything on the full sweep, the
    path map's pick otherwise."""
    if full:
        return (set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'},
                ['commit %d is a multiple of %d - everything'
                 % (count, FULL_EVERY)])
    chosen, picked_live, why = pick(paths)
    if not chosen and not picked_live:
        why.append('nothing changed that any suite covers')
    return chosen, picked_live, why


def _tiered(args, live_sections):
    """The tier's cut of the smart pick: which subjects inside the big
    suites is the judgement call, and it goes to the model - which can
    only ever cost seconds by over-picking, because every way it fails
    returns None and this runs the file whole.
    """
    allowed, sections = plan_for(args.coverage)
    args.file = [f for f in args.file if f in OLLAMA or f in allowed]
    live_sections = sections or ''
    args.live = bool(sections)
    if sections:
        args.file.append(LIVE)
    print('   %d%% tier: %s%s'
          % (args.coverage, ', '.join(args.file),
             ' live:' + sections if sections else ''))
    return live_sections


def _smart(args, tags, live_sections):
    """The smart plan: the changed files against the path map, the full
    sweep every FULL_EVERY commits, the tier's cut, and the model's pick
    of subjects where the map does not settle it.
    """
    count = _commits()
    paths = changed_files()
    full = bool(count) and count % FULL_EVERY == 0 and not args.coverage
    chosen, picked_live, why = _chosen(paths, full, count)
    print('-- smart: %d file%s changed --'
          % (len(paths), '' if len(paths) == 1 else 's'))
    for line in why[:12]:
        print('   ' + line)
    order = list(DEFAULT_SUITES) + [CONFORMANCE]
    # The live suite is not run by name from --file: it is the one with
    # sections, and it is added below.
    if LIVE in chosen:
        picked_live = picked_live or {'all'}
    args.file = [name for name in order if name in chosen]
    live_sections = ','.join(sorted(picked_live)) if picked_live else ''
    if live_sections and 'all' in picked_live:
        live_sections = 'all'
    if live_sections:
        args.live = True
    print('   suites: %s%s' % (', '.join(args.file) or 'none',
                               '  live: ' + live_sections
                               if live_sections else ''))
    if args.coverage:
        live_sections = _tiered(args, live_sections)
    # The model decides the list, not the path map.
    if not tags and not full and settled(chosen, why):
        print('   the map knew every path and the answer is seconds - '
              'not asking the model')
    elif not tags and not full:
        tags, live_sections = _ask_model(args, live_sections)
    if args.dry_run:
        return None
    return tags, live_sections
