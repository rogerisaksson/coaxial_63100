#!/usr/bin/env python3
"""UserPromptSubmit hook: a message about how the robot moves gets Claude told to measure first.

Reads the hook's JSON on stdin; when the prompt names the robot's movement, prints the context
that sends Claude to its walk on the loop (tools/sim/ortho.py --walk --look) and to the user's
newest recording (tools/sim/look.py --last) - with the numbers quoted before any explanation.
Silent otherwise.
"""
import json
import re
import sys

#: Words of the robot's movement that say so alone, Swedish and English, matched at a word's start.
MOVES = ('lut', 'nig', 'kn\u00e4', 'h\u00f6ft', 'svaj', 'bugar', 'vingl', 'catwalk', 'feminin',
         'b\u00e5l', '\u00f6verkropp', 'axl', 'f\u00f6tt', 'gynoid', 'lean', 'knee', 'hip', 'torso',
         'gait', 'sway', 'curts')

#: Words that are the robot's movement only beside a word naming the robot: alone they are
#: "varje g\u00e5ng", "g\u00e5r igenom", "det st\u00e5r", "n\u00e4sta steg", "nodes", "arm the
#: stage" - the hook fired on all of them.
PLAIN = ('bak\u00e5t', 'fram\u00e5t', 'huvud', 'g\u00e5ng', 'g\u00e5r ', 'gick', 'steg', 'reser',
         'rest ', 'st\u00e5r', 'st\u00e5 ', 't\u00e5', 'sjunk', 'hopp', 'skarv', 'arm', 'fot',
         'walk', 'step', 'head', 'bob', 'nod', 'dip')

#: Words naming the robot.
NAMED = ('robot', 'roboten', 'maskinen', 'gynoid', 'humanoid', 'kroppen', 'benen')

CONTEXT = (
    "The user describes how the robot moves. What they see is so (CLAUDE.md). Before any "
    "explanation, measure: `cd host; python -X utf8 tools/sim/ortho.py --walk --look` (the robot's walk "
    "on its latent loop, as the page runs it) and `python -X utf8 tools/sim/look.py --last` "
    "(their newest HUMANOID recording, R). Quote the row and the measure that shows what they describe; if none "
    "shows it, add the measure that would, not an argument. Fix, run it again, quote both. No "
    "reasoning in place of a number.")


def main():
    try:
        prompt = json.load(sys.stdin).get('prompt', '')
    except (ValueError, AttributeError):
        return 0
    text = ' ' + prompt.lower() + ' '

    def said(words, whole=False):
        tail = '(?![a-z\u00e5\u00e4\u00f6])' if whole else ''
        return any(re.search('(?<![a-z\u00e5\u00e4\u00f6])' + re.escape(w) + tail, text) for w in words)
    if said(MOVES) or (said(PLAIN) and said(NAMED, whole=True)):
        print(json.dumps({'hookSpecificOutput': {'hookEventName': 'UserPromptSubmit',
                                                 'additionalContext': CONTEXT}}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
