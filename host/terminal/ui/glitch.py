"""A frame torn between two pictures: the *bzzt* as one gives way to the other.

    lines = torn(old, new, k, seed)      # k 0..1 of the way; ANSI lines, alike in size

Each row is the old picture's or the new one's by a draw against `k`; toward the middle of the
way some rows slide up to TEAR columns aside and some snow with braille in a hot colour.
"""
import random
import re

_TOKEN = re.compile(r'(\x1b\[[0-9;]*m)|(.)', re.S)
RESET = '\x1b[0m'

#: The snow's characters and colours, and the share of rows it takes and slides at the worst.
SNOW = '⠁⠂⠄⡀⢀⠠⠐⠈⣀⣤⣶⣿⡇⢸'
HOT = ((255, 64, 96), (64, 255, 220), (255, 255, 255), (120, 90, 255))
TEAR, SNOWS, SLIDES = 4, 0.2, 0.35


def cells(line):
    """A line's cells: [(the SGR codes in force, its character)]."""
    out, sgr = [], ''
    for code, ch in _TOKEN.findall(line):
        if code:
            sgr = '' if code in (RESET, '\x1b[m') else sgr + code
        else:
            out.append((sgr, ch))
    return out


def joined(row):
    """Cells back into a line, each change of colour said once."""
    out, was = [], ''
    for sgr, ch in row:
        if sgr != was:
            out.append(RESET + sgr)
            was = sgr
        out.append(ch)
    return ''.join(out) + RESET


def torn(old, new, k, seed=0):
    """The frame `k` of the way from `old` to `new`, lists of lines alike in size and width."""
    rng = random.Random(seed)
    heat = 1.0 - abs(2.0 * k - 1.0)
    out = []
    for a, b in zip(old, new):
        row = cells(b if rng.random() < k else a)
        roll = rng.random()
        if roll < SNOWS * heat:
            ink = '\x1b[38;2;%d;%d;%dm' % rng.choice(HOT)
            row = [(ink, rng.choice(SNOW)) if rng.random() < 0.6 else c for c in row]
        elif roll < (SNOWS + SLIDES) * heat and row:
            n = min(len(row) - 1, rng.randint(1, TEAR))
            row = [('', ' ')] * n + row[:-n] if rng.random() < 0.5 else row[n:] + [('', ' ')] * n
        out.append(joined(row))
    return out
