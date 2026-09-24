"""A loop's panel in a notebook, in the terminal's motif.

The feedback loops in a list with + and -, the selected one's picture, its channels, its
slots' kinds and parameters, save. Every change applies to the running loop at once.
"""
import io

from coaxial.control.controller import Feedback
from coaxial.devices.roles import Part
from coaxial.draw import ansi
from coaxial.draw.wiring import FRAME, LABEL, TITLE, feedback

NONE = '-'


def _hex(number):
    return '#%02x%02x%02x' % ansi.rgb(number)


#: The terminal's field: black, Consolas, ash labels, teal values, the amber action.
CSS = ('<style>'
       '.coaxial {background: #000; padding: 8px; font-family: Consolas, monospace}'
       '.coaxial .widget-label, .coaxial .widget-html-content {color: %s;'
       ' font-family: Consolas, monospace}'
       '.coaxial select, .coaxial input {background: #000; color: %s; border: 1px solid %s;'
       ' font-family: Consolas, monospace}'
       '.coaxial button {background: #000; color: %s; border: 1px solid %s;'
       ' font-family: Consolas, monospace}'
       '</style>') % (_hex(LABEL), _hex(ansi.TEAL), _hex(FRAME), _hex(ansi.AMBER),
                      _hex(ansi.AMBER))


def picture(loop, name):
    """One feedback loop drawn as the terminal draws it: PNG bytes."""
    buf = io.BytesIO()
    ansi.image(feedback(loop, name)).save(buf, 'PNG')
    return buf.getvalue()


def kinds(role):
    """Every part kind this process knows for a slot's role, by name."""
    return sorted(k for k, cls in Part.KINDS.items() if issubclass(cls, role))


def sink_keys(loop):
    return ['%s.%s' % (name, key) for name, sink in loop.sinks.items()
            for key in getattr(sink, 'WRITES', ())] + [k for k in loop.outputs]


def panel(loop, path=None):
    """The widget: returned, so a notebook shows it as the cell's value."""
    import ipywidgets as w

    wide = {'description_width': '84px'}
    fit = w.Layout(width='320px')
    listing = w.Select(options=list(loop.feedbacks), rows=6, layout=w.Layout(width='200px'))
    named = w.Text(placeholder='new loop', layout=w.Layout(width='200px'))
    add, drop = w.Button(description='+'), w.Button(description='-')
    for button in (add, drop):
        button.layout = w.Layout(width='48px')
    image = w.Image(format='png')
    detail, status = w.VBox(), w.HTML()
    save = w.Button(description='save' if path else 'save (no path)', disabled=not path)

    def channels():
        return [NONE] + loop.channels()

    def pick(label, options, value, apply):
        options = options if value in options else options + [value]
        box = w.Dropdown(options=options, value=value, description=label, style=wide,
                         layout=fit)
        box.observe(lambda c: apply(None if c['new'] == NONE else c['new']), 'value')
        return box

    def show(name):
        if name is None:
            image.value, detail.children = b'', ()
            return
        f = loop.feedbacks[name]
        image.value = picture(loop, name)

        def rewired(field):
            def apply(value):
                setattr(f, field, value)
                loop.add(name, f)
                image.value = picture(loop, name)
            return apply

        def swapped(slot):
            def apply(kind):
                setattr(f, slot, Part.KINDS[kind]() if kind else None)
                loop.add(name, f)
                show(name)
            return apply

        rows = [w.HTML('<span style="color:%s">channels</span>' % _hex(TITLE))]
        setpoint = w.Combobox(value=f.setpoint or '', options=channels()[1:], description='setpoint',
                              ensure_option=False, style=wide, layout=fit)
        setpoint.observe(lambda c: rewired('setpoint')(c['new'] or None), 'value')
        rows.append(setpoint)
        rows.append(pick('measured', channels(), f.measured or NONE, rewired('measured')))
        command = w.Text(value=f.command or '', placeholder='%s/command' % name,
                         description='command', continuous_update=False, style=wide, layout=fit)
        command.observe(lambda c: rewired('command')(c['new'] or None), 'value')
        rows.append(command)
        rows.append(pick('sink', [NONE] + sink_keys(loop), f.sink or NONE, rewired('sink')))
        for slot, role in Feedback.SLOTS.items():
            part = getattr(f, slot)
            options = kinds(role) if slot == 'regulator' else [NONE] + kinds(role)
            current = type(getattr(part, 'part', part)).__name__ if part is not None else NONE
            rows.append(w.HTML('<span style="color:%s">%s</span>' % (_hex(TITLE), slot)))
            rows.append(pick('kind', options, current, swapped(slot)))
            for param, value in (part.params().items() if part is not None else ()):
                field = w.FloatText(value=value, description=param, style=wide, layout=fit)
                field.observe(lambda c, part=part, param=param: part.configure(
                    **{param: c['new']}), 'value')
                rows.append(field)
        detail.children = rows

    def refresh(select=None):
        listing.options = list(loop.feedbacks)
        listing.value = select if select in loop.feedbacks else (
            listing.options[0] if listing.options else None)
        show(listing.value)

    def added(_):
        name = named.value.strip() or 'loop%d' % (len(loop.feedbacks) + 1)
        loop.add(name, Feedback())
        named.value = ''
        refresh(name)

    def removed(_):
        if listing.value is not None:
            loop.remove(listing.value)
            refresh()

    listing.observe(lambda c: show(c['new']), 'value')
    add.on_click(added)
    drop.on_click(removed)
    save.on_click(lambda _: setattr(status, 'value', 'saved %s' % loop.save(path)))
    refresh(listing.options[0] if listing.options else None)
    left = w.VBox([listing, w.HBox([named]), w.HBox([add, drop]), save, status],
                  layout=w.Layout(width='220px'))
    box = w.VBox([w.HTML(CSS), w.HBox([left, image]),
                  w.Box([detail], layout=w.Layout(max_height='520px', overflow_y='auto'))])
    box.add_class('coaxial')
    return box
