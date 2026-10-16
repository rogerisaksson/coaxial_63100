"""A page's recording: rows to a CSV from R to R, a counter on the page while it runs.

    rec = recorder.started('humanoid', HEADER)      # R
    recorder.added(rec, rows)                        # a frame's rows
    recorder.badge(rec)                              # '● REC ⠹ 1 234' while it records
    recorder.corner(rec, width)                      # the same in a drawing's top-right corner
    path = recorder.saved(rec)                       # R again: build/recordings/humanoid_<time>.csv
"""
import csv
import os
import time

from coaxial.graphics.callouts import packed
from tools import REPO

#: Where a page's recordings go, a CSV from a press to the next.
RECORDINGS = os.path.join(REPO, 'build', 'recordings')
#: The counter's wheel: a cell on per row written; its inks in a drawing.
SPIN = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'
INK, GROUND = (255, 96, 96), (10, 22, 35)


def started(name, header):
    """A recording of rows under `header`, its file named after `name`."""
    return {'name': name, 'header': list(header), 'rows': [], 'began': time.monotonic()}


def added(rec, rows):
    """`rows` written to `rec`, in order."""
    rec['rows'].extend(rows)


def badge(rec):
    """'● REC', the wheel a cell on per row and the rows written."""
    n = len(rec['rows'])
    return '● REC %s %s' % (SPIN[n % len(SPIN)], format(n, ',').replace(',', ' '))


def corner(rec, width):
    """The badge as a drawing's inset cells {(row, column): (code, ink)}, top right; none with no
    recording or under 24 columns."""
    if rec is None or width < 24:
        return {}
    text, ink = ' %s ' % badge(rec), packed(INK, GROUND)
    return {(0, width - len(text) + i): (ord(ch), ink) for i, ch in enumerate(text)}


def saved(rec, folder=None):
    """`rec` written to a CSV in `folder` (RECORDINGS): its path."""
    folder = folder or RECORDINGS
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime(rec['name'] + '_%Y%m%d_%H%M%S.csv'))
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(rec['header'])
        writer.writerows(rec['rows'])
    return path
