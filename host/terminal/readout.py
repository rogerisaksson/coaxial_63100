"""The front page's lower box: what the board is, printed as a late-seventies console printed it.

Terse, upper case, a prompt and its answer, dotted leaders, in the tree's
palette, under the turning model. The register is the point, not the quotes:
the bench struck the ship's-computer talk and the film's own lines as silly
(2026-09-23).

EVERY HARDWARE FACT IS THE BUS'S. The identity page is command 0x41 -
the unit's name, type, firmware, protocol, MCU and its own description
line - and the fitment page is the parts list (0x6D kind 4), read once
through a short session that is closed again so a view can have the
port; with no board answering the stand-in answers, and the masthead's
chip already says SIMULATED. Nothing here names a part: an identity
with an empty parts list prints no parts. The third page is the host's
own account of how the software was built - engineered in dialogue with
Claude, a local LLM with the board's tools over MCP at the bench - and
the suites' size is read off `tests/.counts.json`, measured, or said to
be unmeasured on this terminal.

THE MOTION: a page types in at teletype pace, holds, and decays from
the top row by row like phosphor before the next page ticks in -
identity, fitment, provenance, round again - with a block cursor
blinking throughout and a status row naming the inquiry. Nothing
scrolls mid-line and nothing jumps: a page longer than the box is
split into as many inquiries as it takes.
"""
import os
import sys
import textwrap
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rich.text import Text  # noqa: E402

#: Characters a second while a page types in: a teletype, not a modem.
CPS = 90.0
#: Seconds a finished page stands before it decays.
HOLD_S = 4.0
#: Seconds between one row going and the next, top down.
DECAY_ROW_S = 0.07
#: The cursor's blink, cycles a second.
BLINK_HZ = 2.0
#: The block the machine leaves its cursor on.
CURSOR = '█'
#: The leader column is the page's: its longest label, a space and at
#: least one dot - never past half the box. A fixed column either put a
#: label that filled it a column out or left it with no dot at all, and
#: the bench asked for both the line-up and the dot (UART5 TERMINATION,
#: 2026-09-23).
def _column(labels, width):
    longest = max((len(label) for label in labels), default=0)
    return min(longest + 2, max(6, width // 2))
#: The host's portable cores, host-tested through gcc (docs/ARCHITECTURE.md).
CORES = 'MODBUS DRIVE THERMAL FILTER DAQ SHTP BOOT'


def identity_of(port, simulated):
    """`{'info', 'parts', 'origin', 'real'}` read once off the bus - the
    stand-in's when `simulated` - and the session closed again."""
    from coaxial.comm.session import open_session

    session, origin = open_session(port, simulated=True if simulated else None)
    try:
        info = session.info()[0]
        parts = session.board.system.channel_map().get('parts', [])
    finally:
        session.close()
    return {'info': dict(info), 'parts': [dict(p) for p in parts],
            'origin': origin.label, 'real': origin.real}


def suites_measured():
    """(suites measured here, suites in the tree, checks measured) or None:
    the counts file is per machine, a fresh clone has measured nothing,
    and a tier runs a subset - so the readout says how many of the tree's
    suites this terminal has run.
    """
    tests = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'tests')
    try:
        sys.path.insert(0, tests)
        import counts
        suites = counts.load().get('suites') or {}
        in_tree = len([f for f in os.listdir(tests)
                       if f.startswith('test_') and f.endswith('.py')])
    except (OSError, ImportError, ValueError, AttributeError):
        return None
    if not suites:
        return None
    return len(suites), in_tree, sum(suites.values())


def _leader(label, value, width, cols):
    """`LABEL ......"""
    label = label.upper()
    dots = '.' * max(1, cols - len(label) - 1)
    head = label + ' ' + dots + ' '
    room, below = width - len(head), max(4, width - cols - 1)
    value = str(value).upper()
    parts = textwrap.wrap(value, room) if room >= 6 else []
    if parts:
        rows = [(('label', label + ' '), ('frame', dots + ' '),
                 ('value', parts[0]))]
        rest = textwrap.wrap(' '.join(parts[1:]), below) if parts[1:] else []
    else:
        rows = [(('label', label + ' '), ('frame', dots))]
        rest = textwrap.wrap(value, below)
    for more in rest:
        rows.append((('label', ' ' * (cols + 1)), ('value', more)))
    return rows


def _said(text, width, style='name'):
    """A line in the machine's own voice, wrapped."""
    return [((style, line),) for line in textwrap.wrap(text.upper(), width)
            or ['']]


def pages(identity, width, note=None, preload=None):
    """`[(title, rows)]` - each row a tuple of (style, text) runs, no row
    wider than `width`.
    """
    def table(pairs):
        cols = _column([label for label, _v in pairs], width)
        rows = []
        for label, value in pairs:
            rows += _leader(label, value, width, cols)
        return rows

    ahead = []
    if preload:
        fetched = [('fetched', line) for line in preload.get('steps', [])]
        ahead = [('PRELOAD',
                  _said('> query preload', width)
                  + table([('model', preload.get('model', '?')),
                           ('memory', preload.get('memory', 'unknown')),
                           ('disk', preload.get('disk', 'unknown'))]
                          + fetched
                          + [('status', preload.get('status', 'idle'))]))]
    if identity is None:
        return ahead + [('AWAITING LINK',
                         _said('readout online. ready for inquiry', width)
                         + _said('> identify unit', width)
                         + table([('status', 'awaiting link')]
                                 + ([('link said', note)] if note else [])))]
    info, parts = identity['info'], identity['parts']
    # The origin's label says SIMULATED itself; a real one earns LIVE.
    link = identity['origin'] + (' live' if identity['real'] else '')
    first = (_said('readout online. ready for inquiry', width)
             + _said('> identify unit', width)
             + table([
                 ('unit', info.get('device', 'unknown')),
                 ('type', str(info.get('type', 'unknown')).replace('_', ' ')),
                 ('firmware', '%s  protocol %s.%s' % (
                     info.get('firmware', '?'), info.get('proto_major', '?'),
                     info.get('proto_minor', '?'))),
                 ('mcu', info.get('mcu', 'unknown')),
                 ('commands', info.get('commands', '?')),
                 ('link', link)])
             + _said('> describe unit', width)
             + _said(info.get('description', 'unable to clarify'), width,
                     'value'))
    fitment = _said('> enumerate fitment', width)
    fitment += table([(part.get('name', '?'), part.get('what', ''))
                      for part in parts])
    if not parts:
        fitment += _said('no fitment reported', width, 'value')
    measured = suites_measured()
    proof = ('%d of %d suites run here, %d checks' % measured if measured
             else 'unmeasured on this terminal')
    third = (_said('> query origin of systems', width)
             + table([
                 ('engineered', 'in dialogue with claude / anthropic'),
                 ('at the bench', 'local llm, board tools over mcp'),
                 ('console', 'ccc local / anthropic claude'),
                 ('cores', CORES),
                 ('verification', proof),
                 ('record', 'findings: every measurement kept')])
             + _said('end of inquiry. standing by', width))
    return ahead + [('IDENTITY', first), ('FITMENT', fitment),
                    ('PROVENANCE', third)]


def inquiries(pages_, rows):
    """The pages cut to the box: `[(title, rows)]` with at most `rows`
    rows each, a long page becoming as many inquiries as it takes."""
    out = []
    for title, lines in pages_:
        for at in range(0, max(1, len(lines)), rows):
            out.append((title, lines[at:at + rows]))
    return out


def _length(lines):
    return sum(sum(len(t) for _s, t in row) for row in lines)


def fresh(now):
    return {'page': 0, 'typed': 0.0, 'phase': 'type', 'since': now,
            'last': now}


def step(state, now, count, lines):
    """One frame of the motion on `state`: typing, holding, decaying,
    then the next of `count` inquiries."""
    dt = max(0.0, now - state['last'])
    state['last'] = now
    if state['page'] >= count:
        state.update(page=0, typed=0.0, phase='type', since=now)
    if state['phase'] == 'type':
        state['typed'] += CPS * dt
        if state['typed'] >= _length(lines):
            state.update(phase='hold', since=now)
    elif state['phase'] == 'hold':
        if now - state['since'] >= HOLD_S:
            state.update(phase='decay', since=now)
    elif state['phase'] == 'decay':
        gone = int((now - state['since']) / DECAY_ROW_S)
        if gone > len(lines):
            state.update(page=(state['page'] + 1) % max(1, count),
                         typed=0.0, phase='type', since=now)


def _typed(lines, budget):
    """The rows with the first `budget` characters typed, and whether the
    page is finished."""
    out, left = [], int(budget)
    for row in lines:
        if left <= 0:
            break
        runs = []
        for style, text in row:
            take = text[:left]
            left -= len(take)
            runs.append((style, take))
            if left <= 0:
                break
        out.append(tuple(runs))
    return out


def frame(state, inquiry, count, now, width):
    """The box's body as rich Text: the status row, the page as it
    stands this frame, the cursor."""
    title, lines = inquiry
    page = Text(no_wrap=True, overflow='crop')
    page.append('INQUIRY %d/%d' % (state['page'] + 1, count), style='name')
    page.append('  ' + title, style='label')
    page.append('\n')
    if state['phase'] == 'type':
        shown, dim = _typed(lines, state['typed']), -1
    elif state['phase'] == 'hold':
        shown, dim = lines, -1
    else:
        gone = int((now - state['since']) / DECAY_ROW_S)
        shown, dim = lines[gone:], 0
    for i, row in enumerate(shown):
        for style, text in row:
            page.append(text, style='frame' if i == dim else style)
        if i < len(shown) - 1:
            page.append('\n')
    if int(now * BLINK_HZ * 2) % 2 == 0 and state['phase'] != 'decay':
        page.append(CURSOR, style='value')
    return page


def draw(state, identity, width, rows, now=None, note=None, preload=None):
    """One frame of the readout for a box `width` cells wide with `rows`
    rows inside it, stepping `state` on the way.
    """
    now = time.monotonic() if now is None else now
    if state.get('preloaded'):
        preload = None
    cut = inquiries(pages(identity, width, note, preload), max(1, rows - 1))
    step(state, now, len(cut), cut[state['page'] % len(cut)][1])
    ahead = sum(1 for title, _rows in cut if title == 'PRELOAD')
    if ahead and state['page'] >= ahead:
        state['preloaded'] = True
        state['page'] -= ahead
        cut = inquiries(pages(identity, width, note), max(1, rows - 1))
    return frame(state, cut[state['page'] % len(cut)], len(cut), now, width)
