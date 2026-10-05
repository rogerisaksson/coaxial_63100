#!/usr/bin/env python3
"""The main terminal page: the chooser, drawn like the reference screens.

A list to the left under `// MAIN TERMINAL ACCESS`, the board itself to the
right, turning slowly - the toon mesh off the CAD export, so the first thing
the terminal shows is the hardware it is for - and in turn what it drives, her
and the quad (`terminal.stand`). No session is opened here:
the page has to be instant, so the only live datum is whether a broker is
serving, fetched by a background thread: probed inline it cost 2 029 ms a
frame (tools/render/uibench.py), 0.5 frames/s.

The choice leaves as `main()`'s return - the loader (`terminal.loader`)
runs this page in its own process and reads it - or as the exit code
when run as a script; stdout is the drawing's either way:
    0            quit
    101 + index  the picked entry, in ENTRIES order
"""
import argparse
import sys
import threading
import time

from rich import box
from rich.align import Align
from rich.console import Group
from rich.layout import Layout
from rich.panel import Panel
from rich.text import Text

from terminal import loader
from terminal import readout
from terminal import stand
from terminal.ui import screen as _screen
from terminal.ui.console import Keys
from terminal.ui.marquee import Marquee
from terminal.ui.rate import Corner, rate_of
from terminal.ui.screen import PORT, ENTER_KEYS, paced
from terminal.ui.chrome import KANA, Chrome
from terminal.ui.stage import band_of, curtain, footer, live, mode_of, modes, stage

_screen.CHATTER = False     # the boot bar replaced the scroll

#: The list, its second questions and the views the page reopens on
#: come off the pages under terminal/pages/ (`terminal.loader.listing`):
#: a new page is a file there and nothing here. The answer indexes
#: ENTRIES from 101; a second question's later options take codes past
#: the list, and the loader reads every one back.
ENTRIES, SUB, OPEN, _PICKS = loader.listing()

#: What the masthead knows. `held` is how many sessions have the port,
#: `board` whether one answers anywhere at all - None until asked, 'emulated' where none does
#: and the pages open the emulator;
#: `identity` what the readout prints, off the bus once it is known.
_BROKER: dict = {'held': None, 'board': None, 'identity': None,
                 'said': None}


def _learn(port, simulated):
    """The readout's identity off the bus - the board's when one answers,
    the stand-in's when none does - read once through a short session and
    closed again, off the frame loop.
    """
    from coaxial.errors import LINK_FAULTS, RigError

    try:
        _BROKER['identity'] = readout.identity_of(port, simulated)
        _BROKER['said'] = None
    except LINK_FAULTS + (RigError, ValueError) as exc:
        _BROKER['identity'] = None
        _BROKER['said'] = str(exc)

#: How often the front page re-asks whether a board answers. The broker
#: question is a socket connect; this one probes every port and was
#: measured at 8.4 s with the board unpowered, so it is asked once on the
#: way up and sparingly after.
PROBE_EVERY = 30.0

#: The most a smoke run waits for a port's first identity, s: the probe's 8.4 s, and native's
#: build before its first answer.
FIRST_IDENTITY_S = 60.0


def _watch_broker():
    """Keep the masthead's broker status fresh, off the frame loop."""
    from coaxial.comm import broker
    from coaxial.errors import LINK_FAULTS

    while True:
        try:
            # Nobody serving means no connect: the file outlives a killed
            # broker, and a connect nothing answers takes its full 2 s.
            count = broker.clients() if broker.serving() else None
        except LINK_FAULTS + (ValueError,):   # the socket, the address file
            count = None
        # The broker holding the port is a session; its clients ride on it.
        _BROKER['held'] = count + 1 if count is not None else 0
        time.sleep(3.0)


def _watch_link(port):
    """Whether a board answers anywhere, on its own slow clock; where none does, whether the
    pages open the emulator (coaxial.comm.session.emulator_here). A board on an emulated URL is
    this process's own and never goes: asked once. Asked on, its session shared the line with
    the page's - a hand-off into binary between a page's request and its reply - and the
    attitude page stood still 5 s at a time (2026-09-28)."""
    from coaxial.comm.session import board_answers, emulator_here, url_kind
    from coaxial.errors import LINK_FAULTS

    while True:
        try:
            answered = board_answers(port)
        except LINK_FAULTS:
            answered = False
        _BROKER['board'] = answered or ('emulated' if emulator_here() else False)
        # The readout follows the link: the board's own identity once one
        # answers, the stand-in's until then, and again if it moves.
        known = _BROKER.get('identity')
        if known is None or known['real'] != bool(_BROKER['board']):
            _learn(port, not _BROKER['board'])
        if url_kind(str(port)) == 'emulator':
            return
        time.sleep(PROBE_EVERY)


def masthead(port):
    """The top strip: the views' band, and the chip a view would wear."""
    from coaxial.comm.session import url_kind

    held, board = _BROKER['held'], _BROKER['board']
    if board is False:
        tag = modes('SIM')
    elif board == 'emulated' or (board and url_kind(port) == 'emulator'):
        tag = modes('EMU' if board == 'emulated' else mode_of(port))
    elif board is None or held is None:
        tag = Text('LINK: PROBING', style='bar.dim')
    else:
        tag = live(held) if held else None
    return band_of('COAXIAL 63100', 'PORT: %s' % port, tag)


def roster(picked):
    """The access list."""
    # Every row is one line, picked or not: a framed highlight changed the
    # list's height and the page jumped on each keypress.
    lines = [Text('')]
    for i, (key, name, what) in enumerate(ENTRIES):
        kana = '%-12s' % KANA[name][0] if name in KANA else ' ' * 12
        if i == picked:
            row = Text.assemble((' > ', 'value'), (key, 'value'),
                                ('  ', ''), ('%-18s' % name, 'value'),
                                (kana, 'name'), ('  ' + what.upper(), 'label'))
        else:
            row = Text.assemble('   ', (key, 'label'), ('  ', ''),
                                ('%-18s' % name, 'label'), (kana, 'frame.hud'),
                                ('  ' + what.upper(), 'label'))
        # Cropped, never wrapped: on a narrow tty a wrapped row doubled the
        # list's height and the page scrolled.
        row.no_wrap, row.overflow = True, 'ellipsis'
        lines.append(row)
        lines.append(Text(''))
    return Group(*lines)


def asking(sub):
    """An entry's second question: the same quiet rows, only the pick lit."""
    entry, who = sub
    caption, options = SUB[entry]
    lines = [Text(''),
             Text.assemble('   ', (ENTRIES[entry][1], 'name'),
                           ('   ' + caption, 'label')),
             Text('')]
    for i, (key, name, what, _code) in enumerate(options):
        style = 'value' if i == who else 'label'
        row = Text.assemble((' > ' if i == who else '   ', 'value'),
                            (key, style), ('  ', ''),
                            (name, style), ('   ' + what, 'label'))
        row.no_wrap, row.overflow = True, 'ellipsis'
        lines.append(row)
        lines.append(Text(''))
    return Group(*lines)


#: The turntable's box: this many columns of the page, and the drawing
#: fills the box's inside, so a zoom past 1.0 clips into the frame.
BOX = 58


def readout_rows(tall):
    """Rows of the right column the readout takes, frame included: a
    third of the body, never under a status row and five lines, never
    over twelve lines - the model keeps the rest."""
    return max(8, min(14, tall // 3 + 2))


def compose(port, picked, view, size=None, who=None, rate=''):

    tall = max(8, (size.height if size else 24) - 4)
    # The stand never outgrows the menu: at most BOX columns, at most half the
    # terminal - on a small tty the board shrinks, the list does not.
    wide = min(BOX, max(26, (size.width if size else 100) // 2))
    # The right column is two boxes since 2026-09-23: the model turning above,
    # and under it the readout - what the board says it is, printed as a
    # console of the era printed it (terminal/readout.py).
    below = readout_rows(tall)
    above = max(6, tall - below)
    state = view.setdefault('readout', readout.fresh(time.monotonic()))
    column = Layout()
    column.split_column(
        Layout(Panel(Corner(Chrome(Align(Marquee(stand.turntable(view, wide - 2, above)),
                                         align='center', vertical='middle'),
                                   'COAXIAL 63100', tags=False), rate),
                     title=Text(' COAXIAL 63100 ', style='name'),
                     title_align='left', box=box.HEAVY, border_style='frame',
                     padding=(0, 0), expand=True), name='model'),
        Layout(Panel(readout.draw(state, _BROKER.get('identity'), wide - 4,
                                  below - 2, note=_BROKER.get('said'),
                                  preload=_BROKER.get('preload')),
                     title=Text(' READOUT ', style='name'),
                     title_align='left', box=box.HEAVY, border_style='frame',
                     padding=(0, 1), expand=True), name='readout',
               size=below))
    body = Layout()
    body.split_row(
        Layout(Panel(Chrome(asking(who) if who is not None else roster(picked),
                            'COAXIAL 63100', lock=False),
                     box=box.ROUNDED,
                     title=Text(' MAIN TERMINAL ACCESS ', style='name'),
                     title_align='left', border_style='frame.hud',
                     padding=(1, 1), expand=True),
               name='list'),
        Layout(column, name='board', size=wide))

    whole = Layout()
    whole.split_column(
        Layout(masthead(port), size=1),
        Layout(body, name='body'),
        Layout(footer((('UP DOWN', 'NAVIGATE'), ('ENTER', 'SELECT'),
                       (' '.join(o[0] for o in SUB[who[0]][1]), 'DIRECT'),
                       ('ESC', 'BACK'), ('Q', 'EXIT'))
                      if who is not None else
                      (('UP DOWN', 'NAVIGATE'), ('ENTER', 'SELECT'),
                       ('S B A M G T C', 'DIRECT'), ('F', 'MOUSE'),
                       ('Q', 'EXIT'))),
               size=1))
    return whole


def _act(typed, picked, hotkeys):
    """(new pick, chosen exit code or None) for one frame of keys."""
    for key in typed:
        if key == 'up':
            picked = (picked - 1) % len(ENTRIES)
        elif key == 'down':
            picked = (picked + 1) % len(ENTRIES)
        elif key in ENTER_KEYS:
            return picked, 101 + picked
        elif key.isdigit() and 1 <= int(key) <= len(ENTRIES):
            return picked, 101 + int(key) - 1
        elif key.lower() in hotkeys:
            return picked, 101 + hotkeys[key.lower()]
    return picked, None


def _sub_act(typed, sub):
    """(new (entry, pick), chosen exit code or None) for a second
    question's list."""
    entry, who = sub
    options = SUB[entry][1]
    by_key = {option[0].lower(): (i, option[3])
              for i, option in enumerate(options)}
    for key in typed:
        if key == 'up':
            who = (who - 1) % len(options)
        elif key == 'down':
            who = (who + 1) % len(options)
        elif key in ENTER_KEYS:
            return (entry, who), options[who][3]
        elif key.lower() in by_key:
            picked, code = by_key[key.lower()]
            return (entry, picked), code
    return (entry, who), None


def _typed_choice(line):
    """The chooser's answer to a line typed with no terminal to page on: an
    entry's number or key, or a second question's option by name - for a
    line that cannot be asked twice.
    """
    numbered = {str(i + 1): 101 + i for i in range(len(ENTRIES))}
    keyed = {key.lower(): 101 + i
             for i, (key, _name, _what) in enumerate(ENTRIES)}
    named = {name.lower(): code for _caption, options in SUB.values()
             for _key, name, _what, code in options}
    return numbered.get(line) or keyed.get(line) or named.get(line) or 0


def _second_question(chosen):
    """The second question an exit code opens - (entry, first option) -
    or None when the code is an answer in itself."""
    if chosen is None or (chosen - 101) not in SUB:
        return None
    return (chosen - 101, 0)


def main(argv=None, preload=None):
    """The page until a choice: its code - 0 to quit, 101 + the entry -
    returned to the loader that runs this page in its own process, or the
    exit code as a script.
    """
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--frames', type=int, default=0,
                        help='draw this many and exit 0 - the smoke test')
    parser.add_argument('--simulated', action='store_true',
                        help='say SIMULATED without probing for a board; '
                             'the page opens no session either way')
    parser.add_argument('--open', choices=sorted(OPEN), default=None,
                        help='open on the second question this view was '
                             'picked from, with it lit - the way back '
                             'from ESC in a view under one')
    args = parser.parse_args(argv)

    page = stage()
    console = page.is_terminal
    picked, frame, began = 0, 0, time.monotonic()
    who = OPEN.get(args.open)
    if who is not None:
        picked = who[0]
    view = stand.fresh(began)
    last = began

    if not console and not args.frames:
        # No terminal to page on: read the choice as a line.
        return _typed_choice(sys.stdin.readline().strip().lower())

    threading.Thread(target=_watch_broker, daemon=True).start()
    learn = None
    if args.simulated:
        # Asked for by name: no probe, and the chip says so from the first
        # frame rather than eight seconds into the page.
        _BROKER['board'] = False
        learn = threading.Thread(target=_learn, args=(args.port, True),
                                 daemon=True)
        learn.start()
    else:
        threading.Thread(target=_watch_link, args=(args.port,),
                         daemon=True).start()
    warm = threading.Thread(target=stand.warm, daemon=True)
    warm.start()
    state = preload if preload is not None else loader.fresh()
    _BROKER['preload'] = state
    if preload is None and not args.frames:
        # Run alone, the page preloads for itself; the smoke draws the page,
        # not the model.
        threading.Thread(target=loader.preload, args=(state,),
                         daemon=True).start()
    if args.frames:
        warm.join()      # the smoke test draws the board, not the wait
        if learn is not None:
            learn.join()     # and the readout with something to print
        else:
            # A port's readout comes off the link watcher's first probe.
            until = time.monotonic() + FIRST_IDENTITY_S
            while (_BROKER['identity'] is None and _BROKER['said'] is None
                   and time.monotonic() < until):
                time.sleep(0.05)
    hotkeys = {key.lower(): i for i, (key, _n, _w) in enumerate(ENTRIES)}

    with curtain(page) as live, Keys(console, mouse=True) as keys:
        while True:
            frame += 1
            now = time.monotonic()
            stand.idle(view, now, now - last)
            last = now
            live.update(compose(args.port, picked, view, page.size, who,
                                rate_of(page).label()), refresh=True)
            rate_of(page).tick(now)
            if args.frames and frame >= args.frames:
                return 0

            # Frame to frame, like run_view: the turntable's 0.08 s is the
            # period, not a pause after a draw that already took most of it.
            leave, moved, typed = paced(
                keys, max(0.0, 0.08 - (time.monotonic() - now)))
            if leave == 'menu' and who is not None:
                who = None            # ESC backs out of the second question
                continue
            if leave:
                return 0
            stand.grab(view, keys, moved, time.monotonic())
            if who is not None:
                who, chosen = _sub_act(typed, who)
            else:
                picked, chosen = _act(typed, picked, hotkeys)
                who = _second_question(chosen)
                chosen = None if who else chosen
            if chosen is not None:
                return chosen


if __name__ == '__main__':
    sys.exit(main())
