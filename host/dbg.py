#!/usr/bin/env python3
"""Shortest path to a question: `python dbg.py "why is the NTC at 25.00?"`."""
import sys

from coaxial_ollama.debug import main

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
