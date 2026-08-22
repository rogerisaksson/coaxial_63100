#!/usr/bin/env python3
"""Shortest path to a question: `python dbg.py "why is the NTC at 25.00?"`.

Two lines of shim, because `python -m coaxial_ollama.debug` is a lot to type
sixty times in an afternoon. The work is in coaxial_ollama/debug.py.
"""
import sys

from coaxial_ollama.debug import main

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
