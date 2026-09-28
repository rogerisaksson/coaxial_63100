#!/usr/bin/env python3
"""PreToolUse hook: a suite run straight - one process, one core, the rest idle - is denied.

Suites go through the relay: tools/dev/run_tests.py, every suite a change reaches in one call,
the long ones as shards, a baton a physical core. Reads the hook's JSON on stdin; silent where
the command runs no suite or runs them through the runner.
"""
import json
import re
import sys

#: A test file an interpreter runs.
SUITE_RE = re.compile(r'\bpy(?:thon[\d.]*)?(?:\.exe)?\b[^;&|\n]*?\btests[\\/]+test_\w+\.py')

#: The tools that put suites on the relay themselves.
RELAYED_RE = re.compile(r'\btools[\\/]+dev[\\/]+(?:run_tests|ab|gate|cover)\.py')

REASON = (
    'Suites go through the relay, all a change reaches in one call: `python '
    'tools/dev/run_tests.py --file test_a.py --file test_b.py:word,word` (--smart: what the '
    'diff reaches; --offline: the gate) - a baton a physical core, the long suites as shards. '
    'A suite run straight holds one core while the rest idle (the user, 2026-09-28).')


def main():
    try:
        event = json.load(sys.stdin)
    except ValueError:
        return 0
    command = (event.get('tool_input') or {}).get('command') or ''
    if SUITE_RE.search(command) and not RELAYED_RE.search(command):
        print(json.dumps({'hookSpecificOutput': {
            'hookEventName': 'PreToolUse', 'permissionDecision': 'deny',
            'permissionDecisionReason': REASON}}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
