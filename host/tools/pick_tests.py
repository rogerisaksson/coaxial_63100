#!/usr/bin/env python3
"""Which subjects a change can have broken, decided by the local model.

`tools/run_tests.py` already maps changed *paths* to suite *files*. That map
is coarse by construction: a line moved in `coaxial_mcp/tools.py` pulls in
four suites whatever the line was. This reads the diff itself and picks
**tags** inside the big suite, where 645 of the checks live and where
narrowing is worth anything.

The division is deliberate. The path map decides which files run and is the
safe, dumb half; the model decides which subjects inside them and is the
half that needs judgement. Every way the model can fail lands on the same
answer - run everything:

  * ollama not reachable, or the tag not pulled
  * a reply that is not the JSON it was asked for
  * a reply naming no tag this repository has
  * a reply naming every tag, which is the same as saying nothing

None of that is defensive scaffolding. It is the one direction a wrong
answer here may fail in: a picker that runs too much costs seconds, and one
that runs too little hides a regression until the next full sweep.

    python tools/pick_tests.py                 # against the working tree
    python tools/pick_tests.py --explain       # ...and say why
    python tools/pick_tests.py --model TAG
"""
import argparse
import json
import os
import subprocess
import sys

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

ASK = """Which subjects can this change have broken?

Subjects:
%s

Answer with the ones that could be affected. Too many wastes seconds; too
few hides a regression, so include a subject you are unsure about. Answer
with JSON only: {"tags": ["..."], "why": "one sentence"}

Diff:
%s"""

SCHEMA = {
    'type': 'object',
    'properties': {
        'tags': {'type': 'array', 'items': {'type': 'string'}},
        'why': {'type': 'string'},
    },
    'required': ['tags'],
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
    """(tags, why) from the model's reply, or (None, reason) if unusable.

    None means "this told us nothing" and the caller runs everything. It is
    returned for a reply that will not parse, for one naming no tag this
    repository has, and for one naming all of them - which is the same
    answer as no answer and should not be dressed up as a decision.
    """
    catalogue = set(TAGS if catalogue is None else catalogue)
    try:
        got = json.loads(reply)
        wanted = [str(t).strip().lower() for t in got.get('tags') or []]
        why = str(got.get('why') or '').strip()
    except Exception:                                         # noqa: BLE001
        return None, 'the reply was not the JSON it was asked for'

    known = [t for t in catalogue if t in wanted]
    dropped = [t for t in wanted if t not in catalogue]
    if not known:
        return None, ('named no subject this repository has (%s)'
                      % (', '.join(dropped) or 'nothing at all'))
    if set(known) == catalogue:
        return None, 'named every subject, which is the same as none'
    if dropped:
        why = (why + ' ' if why else '') + \
            '[dropped: %s]' % ', '.join(sorted(dropped))
    return sorted(known), why


def pick(model='gemma4:12b', against='HEAD', keep_alive='30m'):
    """(tags, why). tags is None when the model told us nothing usable."""
    patch = diff_text(against)
    if not patch.strip():
        return None, 'nothing has changed'

    catalogue = '\n'.join('  %-9s %s' % (name, TAGS[name])
                          for name in sorted(TAGS))
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
                                'content': ASK % (catalogue, clip(patch))}])
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

    tags, why = pick(args.model, args.against)
    if tags is None:
        print('all')
        if args.explain:
            print('  %s - running everything' % why, file=sys.stderr)
        return 0
    print(','.join(tags))
    if args.explain:
        print('  %s' % (why or 'no reason given'), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
