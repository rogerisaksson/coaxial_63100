"""The cells every notebook shares, and the one shape they are laid out in.

A notebook here is a short paper. `paper` takes the parts and returns
the cells in the same order every time, so uniformity is a property of
the builder and not a discipline each author keeps:

    # Title
    subtitle, one line, in italics
    **Abstract.** what was asked, on what, how, the headline numbers, and
        what a reader does with them
    ## 1 Setup            the knob and the open cell, identical everywhere
    ## 2 .. n <topic>     a paragraph saying what is measured and why, the
                          code that measures it, a sentence reading the
                          number back - figures through coaxial.figures
    the close cell        the device, and the supply, put back
    ## n+1 Conclusions    the numbers, numbered; then "At the bench", the
                          pragmatic paragraph: what to run, what to look
                          at, what the stand-in could not show
    ## References         the tree's own files: modules, documents, the
                          suites that pin what the notebook showed
"""

#: The one line a reader flips at the bench.
KNOB = """SIMULATED = True          # False, and PORT, at the bench
PORT = 'COM4'"""

#: The device opened the same way in every notebook. `device` is the
#: name throughout; a notebook with two boards names them for what they
#: drive and says so.
OPEN = """from coaxial import Coaxial63100

device = Coaxial63100(port=PORT, simulated_device=SIMULATED).open()
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
    """One numbered section: its heading and the cells under it. The
    number is the builder's, so a section moved keeps no stale one."""
    return (heading, list(cells))


def paper(title, subtitle, abstract, sections, conclusions, bench, references):
    """The cells of one notebook, in the shape above.

    `sections` are `section(...)` tuples, numbered from 2 after Setup - a
    paper that arms the stage or opens more than the device says so in a
    section of its own. `conclusions` are cells: the code that prints the
    numbers, and the markdown that reads them. `bench` is the pragmatic
    paragraph. `references` is a list of `(path, why)`.
    """
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
