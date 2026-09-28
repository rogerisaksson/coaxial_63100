#!/usr/bin/env python3
"""UserPromptSubmit hook: a message about how the gynoid moves gets Claude told to measure first.

Reads the hook's JSON on stdin; when the prompt names her movement, prints the context that
sends Claude to tools/sim/look.py - simulated, and on the user's newest recording (R) - with
the numbers quoted before any explanation. Silent otherwise.
"""
import json
import re
import sys

#: Words of her movement that say so alone, Swedish and English, matched at a word's start.
MOVES = ('lut', 'nig', 'knä', 'höft', 'svaj', 'bugar', 'vingl', 'catwalk', 'feminin', 'bål',
         'överkropp', 'axl', 'fött', 'gynoid', 'lean', 'knee', 'hip', 'torso', 'gait', 'sway',
         'curts')

#: Words that are her movement only beside a word naming her: alone they are "varje gång", "går
#: igenom", "det står", "nästa steg", "nodes", "arm the stage" - the hook fired on all of them
#: (2026-09-28).
PLAIN = ('bakåt', 'framåt', 'huvud', 'gång', 'går ', 'gick', 'steg', 'reser', 'rest ', 'står',
         'stå ', 'tå', 'sjunk', 'hopp', 'skarv', 'arm', 'fot', 'walk', 'step', 'head', 'bob',
         'nod', 'dip')

#: Words naming her.
HER = ('hon', 'henne', 'hennes', 'gynoid', 'humanoid', 'she', 'her', 'kroppen', 'benen')

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

    def said(words, whole=False):
        tail = r'(?![a-zåäö])' if whole else ''
        return any(re.search(r'(?<![a-zåäö])' + re.escape(w) + tail, text) for w in words)
    if said(MOVES) or (said(PLAIN) and said(HER, whole=True)):
        print(json.dumps({'hookSpecificOutput': {'hookEventName': 'UserPromptSubmit',
                                                 'additionalContext': CONTEXT}}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
