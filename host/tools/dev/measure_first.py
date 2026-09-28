#!/usr/bin/env python3
"""UserPromptSubmit hook: a message about how the gynoid moves gets Claude told to measure first.

Reads the hook's JSON on stdin; when the prompt names her movement, prints the context that
sends Claude to tools/sim/look.py - simulated, and on the user's newest recording (R) - with
the numbers quoted before any explanation. Silent otherwise.
"""
import json
import re
import sys

#: Words of her movement, Swedish and English, matched at a word's start.
WORDS = ('lut', 'nig', 'bakåt', 'framåt', 'knä', 'höft', 'huvud', 'gång', 'går ', 'gick', 'steg',
         'reser', 'rest ', 'står', 'stå ', 'tå', 'gynoid', 'svaj', 'bugar', 'sjunk', 'hopp',
         'vingl', 'skarv', 'catwalk', 'feminin', 'bål', 'överkropp', 'axl', 'arm', 'fot', 'fött',
         'lean', 'knee', 'walk', 'step', 'hip', 'head', 'bob', 'nod', 'dip', 'curts', 'sway',
         'torso', 'gait')

CONTEXT = (
    "The user describes how the gynoid moves. What they see is so (CLAUDE.md). Before any "
    "explanation, measure: `cd host; python -X utf8 tools/sim/look.py` (from the squat, as the "
    "page runs her) and `python -X utf8 tools/sim/look.py --last` (their newest HUMANOID "
    "recording, R). Quote the stage row and the column that shows what they describe; if none "
    "shows it, add the measure that would, not an argument. Fix, run it again, quote both. No "
    "reasoning in place of a number.")


def main():
    try:
        prompt = json.load(sys.stdin).get('prompt', '')
    except (ValueError, AttributeError):
        return 0
    text = ' ' + prompt.lower() + ' '
    if any(re.search(r'(?<![a-zåäö])' + re.escape(w), text) for w in WORDS):
        print(json.dumps({'hookSpecificOutput': {'hookEventName': 'UserPromptSubmit',
                                                 'additionalContext': CONTEXT}}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
