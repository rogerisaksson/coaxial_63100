"""The cells every notebook shares, and the one shape they are laid out in."""
#: The knob flipped at the bench.
KNOB = """from coaxial import ExecutionMode

MODE = ExecutionMode.SIMULATED   # .HARDWARE, and PORT, at the bench; .EMULATED: the image on Renode
PORT = 'COM4'"""

OPEN = """from coaxial import Coaxial63100

device = Coaxial63100(port=PORT, execution_mode=MODE).open()
print(device)"""

CLOSE = """device.close()
print(device)"""


def md(text):
    return ('markdown', text)


def code(text):
    return ('code', text)


def section(heading, *cells):
    return (heading, list(cells))


def paper(title, summary, sections, results, bench, references, device=True):
    """# title, summary; 1 Setup; the sections; close; Results; Bench; References.
    `device`: the paper opens a Coaxial63100 as `device` and closes it; else only the knob."""
    cells = [md('# %s\n\n%s' % (title, summary)), md('## 1 Setup'), code(KNOB)]
    if device:
        cells.append(code(OPEN))
    for number, (heading, body) in enumerate(sections, start=2):
        cells.append(md('## %d %s' % (number, heading)))
        cells.extend(body)
    if device:
        cells.append(code(CLOSE))
    cells.append(md('## %d Results' % (len(sections) + 2)))
    cells.extend(results)
    cells.append(md('**Bench.** ' + bench))
    cells.append(md('## References\n\n' + '\n'.join(
        '* `%s` - %s' % (path, why) for path, why in references)))
    return cells
