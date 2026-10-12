"""The HUMANOID page's gym: her latent loop's moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: the digits, >, W, P and N
    gym.boxes(now)               # the side column's LATENT and SAY boxes
    gym.status(now)              # what she is doing, in a line

She stands on the loop (`machine.sprung`) and walks from rest (`sprung.WALK`). A digit says an
entry of the gym's text (`sprung.entries`) in place of what was to come - a payload injected, a
behaviour begun -, > takes the gym's other text (`sprung.GYMS`), read anew each time - an edit
to machine/moves.lisp is two presses away -, W walks her from rest again, P and N shove her, A
lands her anew. LATENT shows each mode as it is asked and as she is, and its spring.
"""
from machine import sprung
from terminal.ui.stage import hud

#: The gym's next text; her walk from rest.
NEXT, WALKS = '>', 'wW'
#: A digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: P and N shove her (`sprung.SHOVES`), from each side in turn; shifted, from behind and ahead
#: in turn.
PUSHES = {'p': ('push', False), 'n': ('nudge', False), 'P': ('push', True), 'N': ('nudge', True)}
#: Each mode's name in the box.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')


def _next(state):
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def _shoved(event, along):
    def shove(state):
        state['side'] = -state['side']
        n = sprung.SHOVES[event] * state['side']
        state['body'].send(push=(0.0, 0.0, n) if along else (n, 0.0, 0.0), seconds=sprung.SHOVE_S)
    return shove


def table(keys):
    """`keys` (the page's own) with the gym's laid over them."""
    out = dict(keys)
    out[NEXT] = _next
    out.update((k, lambda state, i=i: state['body'].send(say=i)) for i, k in enumerate(DIGITS))
    out.update((k, lambda state: state['body'].send(walk=True)) for k in WALKS)
    out.update((k, _shoved(*how)) for k, how in PUSHES.items())
    return out


def status(now):
    """Her status: what she is doing; down, counting to her landing again; a text or a command
    refused, why."""
    if now is None:
        return 'STARTING'
    if now['fallen']:
        return 'DOWN in %.0f s' % max(0.0, now['recover'])
    if now.get('refused'):
        return 'REFUSED  %s' % now['refused']
    return 'LATENT  %s' % now['doing']


def boxes(now):
    """The side column's boxes: each mode asked and as she is, its spring's N/m and N s/m; what
    each digit says."""
    if not now:
        return []
    return [hud('LATENT', [(name, '%+.2f is %+.2f %6.0f %5.0f' % row[1:])
                           for name, row in zip(SHORT, now['modes'])]),
            hud('SAY  ' + now['says'][0].upper(), list(zip(DIGITS, now['says'][1:])))]
