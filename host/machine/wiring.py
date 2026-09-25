"""A wired `Loop` as the terminal draws it.

A box per setpoint group, source, part and sink in flow order, each channel in one ink
wherever it appears; one feedback loop as a block diagram.
"""
from machine import ansi

#: A channel's ink, the same in every box it touches: the motif first, then its neighbours.
INKS = (ansi.TEAL, ansi.AMBER, 110, 176, 150, 74, 216, 146, 80, 180)
FRAME = 23
TITLE = ansi.TEAL
LABEL = 66
BOX = 30
JOIN = ' ▸ '


def _nodes(loop):
    """(name, kind, inputs [(port, channel)], outputs [(port, channel)], params) in flow order."""
    made = {loop.channel_of(n, p) for n, part in loop.parts.items() for p in part.OUTPUTS}
    used = ({ch for key, ch in loop.wires.items()
             if key.partition('.')[2] in loop.parts[key.partition('.')[0]].INPUTS}
            | set(loop.outputs.values()))
    fed = sorted(ch for ch in used if ch not in made
                 and loop.source_of(ch) is None and ch != 't')
    nodes = [('setpoints', 'write()', [], [(ch, ch) for ch in fed], [])] if fed else []
    for name, source in loop.sources.items():
        read = sorted(ch for ch in used if loop.source_of(ch) == name)
        nodes.append((name, type(source).__name__, [],
                      [(ch[len(name) + 1:], ch) for ch in read], []))
    for name, part in loop.parts.items():
        inner = getattr(part, 'part', part)
        kind = type(inner).__name__ + (' @%g Hz' % (1.0 / part.pause) if inner is not part else '')
        nodes.append((name, kind,
                      [(p, loop.wires.get('%s.%s' % (name, p), '-')) for p in part.INPUTS],
                      [(p, loop.channel_of(name, p)) for p in part.OUTPUTS],
                      ['%s %.4g' % kv for kv in part.params().items()]))
    for name, sink in loop.sinks.items():
        nodes.append((name, type(sink).__name__,
                      [(key.rpartition('.')[2], ch) for key, ch in loop.outputs.items()
                       if key.rpartition('.')[0] == name], [], []))
    return nodes


def _box(node, ink, paint):
    """A node as rows of BOX visible cells."""
    name, kind, inputs, outputs, params = node
    inner = BOX - 4
    head = ('%s %s' % (name, kind))[:inner - 2]
    rows = [paint('╭─ ', FRAME) + paint(name[:inner - 2], TITLE)
            + paint(head[len(name):], ansi.ASH)
            + paint(' ' + '─' * (inner - len(head) - 1) + '╮', FRAME)]
    width = max([len(p) for p, _ in inputs + outputs] + [1])

    def edge(text):
        return paint('│ ', FRAME) + text + paint(' │', FRAME)

    for arrow, pairs in (('◂', inputs), ('▸', outputs)):
        for port, channel in pairs:
            named = channel == port or channel.endswith('.' + port)
            label = '' if named else port.ljust(width) + ' '
            room = inner - 2 - len(label)
            shown = channel if len(channel) <= room else channel[:room - 1] + '…'
            text = (paint(arrow + ' ', ansi.ASH) + paint(label, LABEL)
                    + paint(shown, ink(channel) if channel != '-' else ansi.ASH))
            rows.append(edge(text + ' ' * (room - len(shown))))
    line = ''
    for item in params:
        if len(line) + len(item) + 2 > inner:
            rows.append(edge(paint(line.ljust(inner), ansi.ASH)))
            line = ''
        line += ('  ' if line else '') + item
    if line:
        rows.append(edge(paint(line.ljust(inner), ansi.ASH)))
    rows.append(paint('╰' + '─' * (BOX - 2) + '╯', FRAME))
    return rows


def diagram(loop, width=100, colour=True):
    """The loop's boxes left to right, wrapping at `width`, joined by ▸."""
    inks = {}

    def ink(channel):
        return inks.setdefault(channel, INKS[len(inks) % len(INKS)])

    def paint(text, number):
        return ansi.paint(text, number) if colour else text

    boxes = [_box(node, ink, paint) for node in _nodes(loop)]
    per_row = max(1, (width + len(JOIN)) // (BOX + len(JOIN)))
    out = []
    for at in range(0, len(boxes), per_row):
        group = boxes[at:at + per_row]
        tall = max(len(b) for b in group)
        for r in range(tall):
            cells = [b[r] if r < len(b) else ' ' * BOX for b in group]
            join = paint(JOIN, FRAME) if r == 1 else ' ' * len(JOIN)
            out.append(join.join(cells).rstrip())
        out.append('')
    return '\n'.join(out).rstrip('\n')


#: A block's width in cells, in a system's picture (`blocks`).
BLOCK = 30


def _block(title, kind, lines, paint, width):
    """A block as rows of `width` visible cells: its title and kind, then its lines."""
    inner = width - 4
    head = (title + (' ' + kind if kind else ''))[:inner - 2]
    rows = [paint('╭─ ', FRAME) + paint(head[:len(title)], TITLE)
            + paint(head[len(title):], ansi.ASH)
            + paint(' ' + '─' * (inner - len(head) - 1) + '╮', FRAME)]
    for line in lines:
        shown = line if len(line) <= inner else line[:inner - 1] + '…'
        rows.append(paint('│ ', FRAME) + paint(shown.ljust(inner), LABEL) + paint(' │', FRAME))
    rows.append(paint('╰' + '─' * (width - 2) + '╯', FRAME))
    return rows


def blocks(rows, width=BLOCK, colour=True):
    """A system as blocks `width` wide, top to bottom: `rows` [(blocks, down)], each block
    (title, kind, [line]), a row's joined left to right by ▸; `down` the words on the ▾ to the
    next row."""
    def paint(text, number):
        return ansi.paint(text, number) if colour else text

    out = []
    for row, down in rows:
        drawn = [_block(title, kind, lines, paint, width) for title, kind, lines in row]
        for r in range(max(len(b) for b in drawn)):
            cells = [b[r] if r < len(b) else ' ' * width for b in drawn]
            join = paint(JOIN, FRAME) if r == 1 else ' ' * len(JOIN)
            out.append(join.join(cells).rstrip())
        if down:
            out.append(paint(' ' * (width // 2) + '▾ ', FRAME) + paint(down, ansi.ASH))
    return '\n'.join(out)


class _Canvas:

    """Cells of (glyph, ink), rendered as runs."""

    def __init__(self, width, height):
        self.rows = [[(' ', None)] * width for _ in range(height)]

    def put(self, x, y, text, ink=None):
        for i, glyph in enumerate(text):
            if 0 <= x + i < len(self.rows[y]):
                self.rows[y][x + i] = (glyph, ink)

    def render(self, colour):
        lines = []
        for row in self.rows:
            line, run, ink = '', '', None
            for glyph, now in row + [('', 'end')]:
                if now != ink and run:
                    line += ansi.paint(run, ink) if colour and ink is not None else run
                    run = ''
                run, ink = run + glyph, now
            lines.append(line.rstrip())
        return '\n'.join(lines).rstrip('\n')


#: The feedback picture's columns and rows: a box is F_BOX wide.
F_BOX = 22
F_PRE, F_SUM, F_REG, F_MEAS, F_PLANT = 4, 29, 34, 60, 86
F_TOP, F_BOTTOM = 2, 8


def _slot(canvas, x, y, title, kind, lines, ink):
    """A slot's box at (x, y): its title, then (arrow, channel) lines."""
    frame = FRAME if kind else ansi.DIM
    inner = F_BOX - 4
    named = (kind or '-')[:inner - len(title) - 2]
    canvas.put(x, y, '╭─ ', frame)
    canvas.put(x + 3, y, title, LABEL)
    canvas.put(x + 4 + len(title), y, named, TITLE if kind else ansi.DIM)
    head = len(title) + 1 + len(named)
    canvas.put(x + 3 + head, y, ' ' + '─' * (inner - head - 1) + '╮', frame)
    for i, (arrow, channel) in enumerate(lines, start=1):
        shown = channel if len(channel) <= inner - 2 else channel[:inner - 3] + '…'
        canvas.put(x, y + i, '│', frame)
        canvas.put(x + 2, y + i, arrow, ansi.ASH)
        canvas.put(x + 4, y + i, shown, ink(channel) if kind and channel != '-' else ansi.DIM)
        canvas.put(x + F_BOX - 1, y + i, '│', frame)
    canvas.put(x, y + len(lines) + 1, '╰' + '─' * (F_BOX - 2) + '╯', frame)


def feedback(loop, name, colour=True):
    """One feedback loop as a block diagram: prefilter, the sum, the regulator, the plant
    (its sink and source), and back through measure and estimator."""
    inks = {}

    def ink(channel):
        return inks.setdefault(channel, INKS[len(inks) % len(INKS)])

    f = loop.feedbacks[name]
    at = {slot: loop.channel_of('%s/%s' % (name, slot), port)
          for slot, port in (('prefilter', 'y'), ('measure', 'y'), ('estimator', 'estimate'),
                             ('regulator', 'command'))}
    kind = {slot: type(getattr(part, 'part', part)).__name__ if part is not None else None
            for slot, part in f.slots().items()}
    ref = at['prefilter'] if f.prefilter else (f.setpoint or '-')
    value = at['measure'] if f.measure else (f.measured or '-')
    estimate = at['estimator'] if f.estimator else value
    command = at['regulator']
    c = _Canvas(F_PLANT + F_BOX, F_BOTTOM + 5)
    _slot(c, F_PRE, 0, 'prefilter', kind['prefilter'],
          [('◂', f.setpoint or '-'), ('▸', ref)], ink)
    _slot(c, F_REG, 0, 'regulator', kind['regulator'],
          [('◂', ref), ('◂', estimate), ('▸', command)], ink)
    _slot(c, F_REG, F_BOTTOM - 2, 'estimator', kind['estimator'],
          [('◂', value), ('◂', command), ('▸', estimate)], ink)
    _slot(c, F_MEAS, F_BOTTOM - 2, 'measure', kind['measure'],
          [('◂', f.measured or '-'), ('▸', value)], ink)
    # the plant: what the command drives and what the measurement comes back from
    c.put(F_PLANT, 0, '╭─ plant ' + '─' * (F_BOX - 10) + '╮', ansi.ASH)
    for y in range(1, F_BOTTOM + 3):
        c.put(F_PLANT, y, '│', ansi.ASH)
        c.put(F_PLANT + F_BOX - 1, y, '│', ansi.ASH)
    c.put(F_PLANT + 2, F_TOP, '◂ ' + (f.sink or 'no sink')[:F_BOX - 6],
          ink(f.sink) if f.sink else ansi.DIM)
    c.put(F_PLANT + 2, F_BOTTOM, '▸ ' + (f.measured or 'no source')[:F_BOX - 6],
          ink(f.measured) if f.measured else ansi.DIM)
    c.put(F_PLANT, F_BOTTOM + 3, '╰' + '─' * (F_BOX - 2) + '╯', ansi.ASH)
    # the wires: in, through the sum, out to the plant, back along the bottom
    c.put(0, F_TOP, '───▸', FRAME)
    c.put(F_PRE + F_BOX, F_TOP, '──▸', FRAME)
    c.put(F_SUM, F_TOP, 'Σ', ansi.AMBER)
    c.put(F_SUM + 1, F_TOP, '───▸', FRAME)
    c.put(F_REG + F_BOX, F_TOP, '─' * (F_PLANT - F_REG - F_BOX - 1) + '▸', FRAME)
    c.put(F_MEAS + F_BOX, F_BOTTOM, '◂' + '─' * (F_PLANT - F_MEAS - F_BOX - 1), FRAME)
    c.put(F_REG + F_BOX, F_BOTTOM, '◂' + '─' * (F_MEAS - F_REG - F_BOX - 1), FRAME)
    c.put(F_SUM, F_TOP + 1, '▴', FRAME)
    for y in range(F_TOP + 2, F_BOTTOM):
        c.put(F_SUM, y, '│', FRAME)
    c.put(F_SUM, F_BOTTOM, '└' + '─' * (F_REG - F_SUM - 1), FRAME)
    return c.render(colour)
