"""The cells every notebook shares, and the one shape they are laid out in."""
#: A name only this kernelspec carries: two CPython 3.14.7s here, one without
#: the packages, and the editor picked that one by version.
KERNEL = 'coaxial_63100'
KERNEL_DISPLAY = 'Python (coaxial_63100)'

#: What a reader needs before the first cell: that it runs as it stands, on what, on which
#: kernel, and the knob to flip at the bench.
GUIDE = ("Run All works as it stands - no board, no cable: `EMULATED` is this host's firmware image "
         "on Renode, and the host's own stand-in where no emulator runs; `SIMULATED` is that stand-in "
         "asked for. The kernel is `%s`, which `setup.ps1` registers. At the bench: `MODE = "
         "ExecutionMode.HARDWARE` and `PORT` the board's port - `coaxial.comm.ports.find()` names the "
         "first that answers." % KERNEL_DISPLAY)

#: The knob flipped at the bench: the mode, and why it is that one.
KNOB = """from coaxial import ExecutionMode

MODE = ExecutionMode.%s   # %s; .HARDWARE and PORT at the bench
PORT = 'COM4'"""

#: A paper's mode unless it names its own (`MODE` in its module).
EMULATED = ('EMULATED', 'the image on Renode, the stand-in where none runs')

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


def paper(title, summary, sections, results, bench, references, device=True, mode=EMULATED):
    """# title, summary; 1 Setup: the guide, the knob; the sections; close; Results; Bench; References.
    `device`: the paper opens a Coaxial63100 as `device` and closes it; else only the knob.
    `mode`: (the ExecutionMode's name, why)."""
    cells = [md('# %s\n\n%s' % (title, summary)), md('## 1 Setup'), md(GUIDE), code(KNOB % mode)]
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
