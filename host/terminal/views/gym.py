"""The HUMANOID page's gym: her latent loop a key away, its moves said by digit, her modes shown.

    KEYS = gym.table(KEYS)       # the page's keys with the gym's: <; on the loop >, the digits, P and N
    gym.boxes(now)               # the side column's LATENT and SAY boxes, on the loop
    gym.body(state)              # her process: the loop's, or the director's, as state['ortho'] has it

On the loop (`machine.sprung`) she stands at rest; a digit says an entry of the gym's text
(`sprung.entries`) - a payload injected, a behaviour begun -, > takes the gym's other text
(`sprung.GYMS`), read anew each time - an edit to machine/moves.lisp is two presses away -, P
and N shove her, A lands her anew. LATENT shows each mode as it is asked
and as she is, and its spring.
"""
from machine import sprung
from machine.events import SHOVE_S, SHOVES
from machine.running import Running
from terminal.ui.stage import hud

#: Her body handed to her latent loop, standing, and back to her director; on the loop, the
#: gym's next text.
TOGGLE, NEXT = '<', '>'
#: On the loop a digit says the gym's entry of its place: 1 the first, 0 the tenth.
DIGITS = '1234567890'
#: On the loop P and N shove her (`events.SHOVES`), from each side in turn; shifted, from
#: behind and ahead in turn.
PUSHES = {'p': ('push', False), 'n': ('nudge', False), 'P': ('push', True), 'N': ('nudge', True)}
#: Each mode's name in the box.
SHORT = ('axial', 'lateral', 'drive', 'pitch', 'sagittal', 'lift', 'width', 'list', 'turn', 'arms', 'rock')


def body(state):
    """Her process: on the loop where state['ortho'], else the director's (state['director'])."""
    return Running(worker=sprung.run) if state['ortho'] else state['director']()


def _toggled(state):
    state['body'].close()
    state.update(ortho=not state['ortho'], law=False, pace=0.0, rig=None, gym=sprung.GYMS[0],
                 playback=type(state['playback'])())
    state['body'] = body(state)


def _next(state):
    state['gym'] = sprung.GYMS[(sprung.GYMS.index(state.get('gym', sprung.GYMS[0])) + 1) % len(sprung.GYMS)]
    state['body'].send(gym=state['gym'])


def _shoved(event, along):
    def shove(state):
        state['side'] = -state['side']
        n = SHOVES[event] * state['side']
        state['body'].send(push=(0.0, 0.0, n) if along else (n, 0.0, 0.0), seconds=SHOVE_S)
    return shove


def _on_loop(mine, theirs):
    def act(state):
        (mine if state.get('ortho') else theirs)(state)
    return act


def table(keys):
    """`keys` (the page's own) with the gym's laid over them."""
    out = dict(keys)
    out[TOGGLE] = _toggled
    out[NEXT] = _on_loop(_next, keys.get(NEXT, lambda state: None))
    out.update((k, _on_loop(lambda state, i=i: state['body'].send(say=i), keys[k]))
               for i, k in enumerate(DIGITS))
    out.update((k, _on_loop(_shoved(*how), keys[k])) for k, how in PUSHES.items())
    return out


def status(now):
    """Her status on the loop: what she is doing; down, counting to her landing again; her
    gym's text refused, why."""
    if now['fallen']:
        return 'DOWN in %.0f s' % max(0.0, now['recover'])
    if now.get('refused'):
        return 'REFUSED  %s' % now['refused']
    return 'LATENT  %s' % now['doing']


def boxes(now):
    """The side column's boxes on the loop: each mode asked and as she is, its spring's N/m
    and N s/m; what each digit says."""
    if not now or 'modes' not in now:
        return []
    return [hud('LATENT', [(name, '%+.2f is %+.2f %6.0f %5.0f' % row[1:])
                           for name, row in zip(SHORT, now['modes'])]),
            hud('SAY  ' + now['says'][0].upper(), list(zip(DIGITS, now['says'][1:])))]
