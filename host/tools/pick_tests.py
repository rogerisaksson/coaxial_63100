#!/usr/bin/env python3
"""Which tests a change can have broken, decided by the local model.

The model reads the diff and names three things: the suites, the subjects
inside the big one, and which live section - if any - has to run. The path
map in run_tests.py is the fallback, not the primary: it is coarse by
construction, pulling four suites for any line in one file.

Every way the model can fail lands on the same answer, run everything:

  * ollama not reachable, or the tag not pulled
  * a reply that is not the JSON it was asked for
  * a reply naming no suite this repository has
  * a reply naming every subject, which is the same as naming none

That is the one direction a wrong answer here may fail in: running too much
costs seconds, running too little hides a regression until the next sweep.

    python tools/pick_tests.py                 # against the working tree
    python tools/pick_tests.py --explain       # ...and say why
"""
import argparse
import collections
import json
import os
import subprocess
import sys

# What the model was asked for, once it has answered.
Plan = collections.namedtuple('Plan', 'suites tags live why')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # host/
sys.path.insert(0, ROOT)

# The subject catalogue lives with the tests it names, so a tag cannot be
# added in one place and mean nothing in the other.
from tests.test_ollama import TAGS                          # noqa: E402

# How much of the diff the model sees. A whole refactor does not fit an 8k
# window beside the catalogue and the answer, and the first lines of each
# hunk are what say what a change is about - a truncated diff still names
# every file, which is the coarse signal the path map already has.
DIFF_CHARS = 6000

SUITES = {
    'test_ollama.py': 'the host: prompt, tools, replies, language, render, '
                      'bus, link, and the test tooling itself. No board, no '
                      'model, no network.',
    'test_mcp.py': 'the MCP tool surface and what it renders',
    'test_simulated.py': 'the stand-in board',
    'test_parity.py': 'board against stand-in, every number masked out',
    'test_conformance.py': 'a byte-level Modbus master against the firmware',
    'test_live_model.py': 'the real local model choosing real tools. Minutes.',
}

LIVE_SECTIONS = {
    'tools': 'which tool one question reaches for, asked from a clean history',
    'sequence': 'two questions in a row, history kept',
    'language': 'the session language and its lock',
    'all': 'every live section',
    'none': 'the model does not need to run',
}

ASK = """Which tests can this change have broken?

Suites:
%s

Subjects inside test_ollama.py:
%s

Live sections (test_live_model.py only):
%s

Files changed (all of them):
%s

Too many wastes seconds; too few hides a regression, so include one you are
unsure about. JSON only:
{"suites": ["..."], "tags": ["..."], "live": "...", "why": "one sentence"}

Diff (clipped - the file list above is complete, this is not):
%s"""

SCHEMA = {
    'type': 'object',
    'properties': {
        'suites': {'type': 'array',
                   'items': {'type': 'string', 'enum': sorted(SUITES)}},
        'tags': {'type': 'array',
                 'items': {'type': 'string', 'enum': sorted(TAGS)}},
        'live': {'type': 'string', 'enum': sorted(LIVE_SECTIONS)},
        'why': {'type': 'string'},
    },
    'required': ['suites', 'tags'],
}


def diff_text(against='HEAD'):
    """The working tree and the last commit, as one patch.

    Both, because a picker run before committing wants the first and one run
    in a hook wants the second, and asking which is meant is a flag nobody
    would remember to pass.
    """
    parts = []
    for args in (['diff', against], ['diff', '--cached'],
                 ['diff', '%s~1' % against, against]):
        try:
            done = subprocess.run(['git'] + args, cwd=os.path.dirname(ROOT),
                                  capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
                                  timeout=30)
        except Exception:                                     # noqa: BLE001
            continue
        # `or ''`: measured None here, from a git invocation that returned 0
        # with nothing captured. A picker that raises is worse than one that
        # says "nothing changed" and runs everything.
        if done.returncode == 0 and (done.stdout or '').strip():
            parts.append(done.stdout)
    return '\n'.join(parts)


def changed(against='HEAD'):
    """Every path the diff touches, names only."""
    seen = []
    for args in (['diff', '--name-only', against],
                 ['diff', '--name-only', '--cached'],
                 ['diff', '--name-only', '%s~1' % against, against]):
        try:
            done = subprocess.run(['git'] + args, cwd=os.path.dirname(ROOT),
                                  capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
                                  timeout=30)
        except Exception:                                     # noqa: BLE001
            continue
        for line in (done.stdout or '').splitlines():
            if line.strip() and line.strip() not in seen:
                seen.append(line.strip())
    return seen


def clip(text, limit=DIFF_CHARS):
    """The head of the diff, plus a line saying what was left out.

    The head rather than the tail: `diff --git` lines come first in each
    file's section, so a clipped patch still names files it could not show.
    """
    if len(text) <= limit:
        return text
    return (text[:limit] + '\n[... %d more characters of diff]'
            % (len(text) - limit))


def parse(reply, catalogue=None):
    """A Plan, or (None, reason) when the reply told us nothing usable.

    None means the caller runs everything. Returned for a reply that will not
    parse, one naming no suite this repository has, and one naming every
    subject - which is the same answer as no answer and should not be dressed
    up as a decision.
    """
    catalogue = set(TAGS if catalogue is None else catalogue)
    try:
        got = json.loads(reply)
        suites = [str(x).strip() for x in got.get('suites') or []]
        wanted = [str(t).strip().lower() for t in got.get('tags') or []]
        live = str(got.get('live') or 'none').strip().lower()
        why = str(got.get('why') or '').strip()
    except Exception:                                         # noqa: BLE001
        return None, 'the reply was not the JSON it was asked for'

    files = [name for name in SUITES if name in suites]
    if not files:
        return None, ('named no suite this repository has (%s)'
                      % (', '.join(suites) or 'nothing at all'))
    tags = sorted(t for t in catalogue if t in wanted)
    if set(tags) == catalogue:
        tags = []                 # every subject is the same as no narrowing
    if live not in LIVE_SECTIONS:
        live = 'all'
    dropped = [t for t in wanted if t not in catalogue]
    dropped += [x for x in suites if x not in SUITES]
    if dropped:
        why = ((why + ' ' if why else '')
               + '[dropped: %s]' % ', '.join(sorted(dropped)))
    return Plan(files, tags, live, why), why


def _catalogue(entries, width=18):
    return '\n'.join('  %-*s %s' % (width, name, entries[name])
                     for name in sorted(entries))


def pick(model='gemma4:12b', against='HEAD', keep_alive='30m'):
    """(Plan, why). Plan is None when the model told us nothing usable."""
    patch = diff_text(against)
    if not patch.strip():
        return None, 'nothing has changed'

    # The file list in full, beside a clipped diff. Measured: the diff was
    # cut at DIFF_CHARS before it reached host/, so the model saw a README
    # edit and nothing else, and picked the conformance suite for a change to
    # the prompt loop. Names cost a line each; hunks cost the whole budget.
    catalogue = (_catalogue(SUITES), _catalogue(TAGS, 9),
                 _catalogue(LIVE_SECTIONS, 9),
                 '\n'.join('  ' + name for name in changed(against)),
                 clip(patch))
    try:
        from coaxial_ollama.client import Ollama
        # think=False, and not just for the tokens. Measured: with thinking
        # on, gemma4:12b spent the whole num_predict budget reasoning about
        # the diff and returned `content: ''` - an empty answer that reads
        # as "the model said nothing" when what happened is that it never
        # got to the part it was asked for. This is a classification with a
        # schema; there is nothing here to reason aloud about.
        client = Ollama(model, keep_alive=keep_alive, fmt=SCHEMA,
                        think=False, num_predict=400)
        client.model = client.require_model()
        message = client.chat([{'role': 'user',
                                'content': ASK % catalogue}])
    except Exception as exc:                                  # noqa: BLE001
        return None, 'could not ask %s: %s' % (model, exc)
    return parse((message.get('content') or '').strip())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--model', default='gemma4:12b')
    parser.add_argument('--against', default='HEAD')
    parser.add_argument('--explain', action='store_true',
                        help="print the model's reason as well as its choice")
    args = parser.parse_args(argv)

    plan, why = pick(args.model, args.against)
    if plan is None:
        print('all')
        if args.explain:
            print('  %s - running everything' % why, file=sys.stderr)
        return 0
    print('suites: %s' % ' '.join(plan.suites))
    print('tags:   %s' % (','.join(plan.tags) or 'all'))
    print('live:   %s' % plan.live)
    if args.explain:
        print('  %s' % (plan.why or 'no reason given'), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
