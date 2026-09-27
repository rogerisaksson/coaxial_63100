"""The STO chain in a word for the top row: lit while the pilot supplies the gate drivers.

Read off the gate drivers' state by a watcher on the view's rig: lit while the master's pilot is
heard, the chain releases FAULTOUT (PE15) and the drivers are supplied; otherwise dim, with why.
"""
import threading
import time
from contextlib import suppress

from rich.text import Text

from coaxial.devices.gates import GateStage
from coaxial.errors import RigError
from coaxial.model.inverter import GATE_UVLO_V

#: How often the watcher reads, s.
EVERY_S = 0.5

#: The gate drivers' state last read, or None: the one the band shows.
SEEN = {'state': None}


def watch(rig):
    """Read the chain every EVERY_S while `rig` is open."""
    def run():
        while rig.session is not None:
            with suppress(RigError, OSError, AttributeError, KeyError):
                SEEN['state'] = rig.board.gate_drivers.state()
            time.sleep(EVERY_S)
        SEEN['state'] = None
    threading.Thread(target=run, name='sto watch', daemon=True).start()


def flag(state):
    """(word, why) for the chain in `state`, or None where the board says nothing of it (before
    MINOR 23). With AFE_ON low the pins read mid-scale and PE15 alone is heard."""
    if not state or state.get('nfault') is None:
        return None
    want = dict(GateStage.INTERLOCK)
    afe, vgate = state['afe_on'], state['vgate_mv']
    # Unread while the current loop holds the converters: PE15, U9's enable, alone.
    supplied = (not afe) or vgate is None or vgate >= GATE_UVLO_V * 1e3
    if state['nfault'] and supplied:
        return 'STO ON', ''
    if not afe:
        return 'STO OFF', 'AFE off'
    if state['pilot_microvolts'] < want['Cinj'] * 1e6:
        return 'STO OFF', 'no pilot'
    if state['level_microvolts'] < want['Clevel'] * 1e6:
        return 'STO OFF', 'no keepalive'
    return 'STO OFF', 'PGD' if not state['nfault'] else 'no supply'


def chip():
    """The chip, or None: sodium while the drivers are supplied, the band's own dim otherwise."""
    said = flag(SEEN['state'])
    if said is None:
        return None
    word, why = said
    if not why:
        return Text(' %s ' % word, style='chip.sto')
    return Text(' %s: %s ' % (word, why), style='chip.sto.off')
