"""Where a machine's boards run: the hardware, a stand-in, their images emulated, or none. Her page's tiers.

    modes.chosen(os.cpu_count(), boards)    # the richest tier her body is wired to that fits the cores
    modes.told(cores, boards)               # one line: the tier chosen, what each other would take
"""
import enum


class ExecutionMode(enum.Enum):

    """HARDWARE: the boards on the port - a stand-in where none answers, if the caller allows
    one; SIMULATED: the stand-in; EMULATED: each board's own image on an emulated MCU;
    VIRTUAL: no board, each actuator where it is told (`machine.virtual`); DYNAMIC: no board,
    each joint a drive on a body with mass under gravity (`machine.physics`)."""

    HARDWARE = 'hardware'
    SIMULATED = 'simulated'
    EMULATED = 'emulated'
    VIRTUAL = 'virtual'
    DYNAMIC = 'dynamic'


HARDWARE = ExecutionMode.HARDWARE
SIMULATED = ExecutionMode.SIMULATED
EMULATED = ExecutionMode.EMULATED
VIRTUAL = ExecutionMode.VIRTUAL
DYNAMIC = ExecutionMode.DYNAMIC

#: Where her boards can run under her page in real time: each tier's cores a board on this
#: host's kind of core (docs/findings/emulation.md: a native board's layer 2.2 cores for 20 in a
#: limb's lockstep, Renode 4.2 a board under the drive; her own process's SIL frames cost her
#: boards none), and whether her body is wired to it (docs/todo/host.md, native://).
TIERS = (('in-process', 0.0, True), ('native', 0.11, False), ('emulated', 4.2, False))
#: The cores her law's process and her page take beside her boards'.
TAKEN = 2.0


def afforded(cores, boards):
    """[(tier, the cores its `boards` take, whether they fit `cores` beside TAKEN)], TIERS' order."""
    return [(name, a_board * boards, a_board * boards + TAKEN <= cores) for name, a_board, _wired in TIERS]


def chosen(cores, boards):
    """The last tier of TIERS her body is wired to whose boards fit `cores` (`afforded`)."""
    return [name for (name, _c, fits), (_n, _a, wired) in zip(afforded(cores, boards), TIERS)
            if fits and wired][-1]


def told(cores, boards):
    """One line: the tier chosen for `boards` on `cores`, and what each other tier would take of them."""
    pick = chosen(cores, boards)
    rest = ['%s %.1f %s%s' % (name, c, 'fits' if fits else 'does not', '' if wired else ' (unwired)')
            for (name, c, fits), (_n, _a, wired) in zip(afforded(cores, boards), TIERS) if name != pick]
    return '%s; of %d cores %s' % (pick, cores, ', '.join(rest))
