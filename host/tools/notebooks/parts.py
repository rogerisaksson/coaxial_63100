"""The cells every notebook shares, and the one shape they are laid out in."""
#: The one line a reader flips at the bench.
KNOB = """SIMULATED = True          # False, and PORT, at the bench
PORT = 'COM4'"""

#: The device opened the same way in every notebook. `device` is the
#: name throughout; a notebook with two boards names them for what they
#: drive and says so.
OPEN = """from coaxial import Coaxial63100

device = Coaxial63100(port=PORT, device=SIMULATED).open()
print(device)"""

#: The device put back as found: the port, and the supply if this
#: session switched it. Every notebook ends its measurements with it.
CLOSE = """device.close()
print(device)"""


def md(text):
    return ('markdown', text)


def code(text):
    return ('code', text)


def section(heading, *cells):
    """One numbered section: its heading and the cells under it."""
    return (heading, list(cells))


def paper(title, subtitle, abstract, sections, conclusions, bench, references):
    """The cells of one notebook, in the shape above."""
    cells = [md('# %s\n\n*%s*\n\n**Abstract.** %s' % (title, subtitle, abstract)),
             md('## 1 Setup'), code(KNOB), code(OPEN)]
    for number, (heading, body) in enumerate(sections, start=2):
        cells.append(md('## %d %s' % (number, heading)))
        cells.extend(body)
    cells.append(code(CLOSE))
    cells.append(md('## %d Conclusions' % (len(sections) + 2)))
    cells.extend(conclusions)
    cells.append(md('**At the bench.** ' + bench))
    cells.append(md('## References\n\n' + '\n'.join(
        '* `%s` - %s' % (path, why) for path, why in references)))
    return cells
