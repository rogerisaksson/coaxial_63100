"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import ctypes
import functools
import math
import os
import queue
import time
from machine.process import apart, gaits, sprung, worn
from machine.world import physics, referee
from machine.senses import edge
from machine.way import peek, planner
from machine.law import se2
from machine.eyes import vision
from machine.identify import wear
from machine.errors import BusFault, MachineError


def _apply_say(g, gym, by, command, errand_step):
    from machine.way import errand
    from machine.texts import maneuver
    from machine.law import se2
    sc = command['say']
    sc_str = sprung.ALIASES.get(str(sc), str(sc))
    if not errand_step and errand.start_errand(g, gym, sc_str, heard, gaits.heard_say, command.get('at')):
        return
    is_idx = (isinstance(sc, int) or (isinstance(sc, str) and sc.isdigit())) and 0 <= int(sc) < len(gym)
    entry = gym[int(sc)] if is_idx else by.get(sc_str)
    if entry is None:
        return
    if entry[0] == 'hop' and 'charge' not in command and g.get('_jump_until', 0.0) <= g['t']:
        g['_hop_charge'] = HOP_CHARGE
    if 'twist' not in command and entry[0] != 'hop' and not (g.get('_twist_active') and entry[0] in se2.WASD_GAITS):
        g['_twist_active'], g['_twist_stop_until'], g['_halt_at'] = False, 0.0, None
        g['twist'] = g['asked'] = (0.0, 0.0, 0.0)
    if 'heading' not in command and entry[0] in gaits.TURN_ALL:
        g['target_heading'], g['has_heading'] = 0.0, 0.0
    from machine.moves import clocked
    g['programs'] = dict(clocked.BOUND[entry[0]]) if entry[0] in clocked.BOUND else g['programs']
    gaits.heard_say(g, gym, maneuver.said(g, entry))


def heard(g, gym, command):
    from machine.law import latent, modal, se2, stride
    from machine.moves import leap
    from machine.process import joints
    from machine.stack import wbc
    errand_step = bool(command.get('_errand_cmd'))
    unknown = sorted(set(command) - set(sprung.HEARD) - set(joints.HEARD) - {'_errand_cmd'})
    if unknown:
        raise MachineError('the loop hears %s: not %s' % (', '.join(sprung.HEARD), ', '.join(unknown)))
    if not errand_step and any(k in command for k in ('scenario', 'twist', 'say', 'walk', 'entries')):
        g['_errand'] = None
        if 'heading' not in command:
            g['target_heading'], g['has_heading'] = 0.0, 0.0
    if 'scenario' in command:
        sprung.reset_for_scenario(g, command['scenario'])
    if ('say' in command or command.get('walk') or command.get('entries')) and g['st'][latent.NOW][latent.GAIN] < 1e-6 and g.get('_jump_until', -9.0) + 0.03 <= g['t']:
        g['st'][latent.THETA] = 0.0
    if 'joints' in command:
        joints.said(g, command)
    elif not errand_step and any(k in command for k in ('say', 'entries', 'scenario')):
        joints.ended(g)
    if 'twist' in command and ('say' in command or not joints.scooted(g, command['twist'])):
        se2.heard_twist(g, gym, command['twist'], 'say' in command, gaits.heard_say)
    by = {e[0]: e for e in gym}
    if 'charge' in command:
        g['_hop_charge'] = max(0.0, min(1.0, float(command['charge'])))
    if 'charge' in command and 'say' not in command and g.get('_jump_until', 0.0) > g['t']:
        leap.update_leap_charge(g, by, se2._WALK_ON_WIRE, g['_hop_charge'])
    if 'say' in command:
        _apply_say(g, gym, by, command, errand_step)
    if command.get('walk') and not (abs(float(g['out'][latent.SWAY][modal.DRIVE])) > stride.STIRS
                                    or abs(float(g['out'][latent.SWAY][modal.LATERAL])) > 0.08
                                    or g['st'][latent.NOW][latent.GAIN] > 0.05):
        gaits.start_mode(g, by, by['walk'], at=g['t'] + 0.001) if 'walk' in by else sprung.walked(g)
    for entry in command.get('entries', ()):
        sprung.said(g, *entry)
    if 'heading' in command:
        g['has_heading'] = 0.0 if command['heading'] is None else 1.0
        g['target_heading'] = 0.0 if command['heading'] is None else float(command['heading'])
    if 'push' in command:
        g['world'].push(command['push'], command.get('seconds', sprung.SHOVE_S))
    if 'grip' in command:
        g['grip'][:] = g['grip_prior'][:] = min(float(command['grip']), wbc.MU)
    if 'floor' in command:
        told_floor(g, command['floor'])


def told_floor(g, profile):
    """A floor told her into her frame's map (`modal.MAP_*`): (along, rise) m from where she
    stands along her heading, its heights over the floor under her; None forgets it."""
    import numpy as np
    from machine.law import modal
    fr = g['fr']
    fr[modal.MAP_N] = 0.0
    if profile is None:
        return
    pts, hd = np.array(profile, float)[:modal.MAP_K], float(g['heading'])
    fr[modal.MAP_AT], fr[modal.MAP_WAY] = (g['est']['pos'][0], g['est']['pos'][2]), (math.sin(hd), math.cos(hd))
    fr[modal.MAP_ALONG][:len(pts)], fr[modal.MAP_RISE][:len(pts)] = pts[:, 0], pts[:, 1]
    fr[modal.MAP_N] = float(len(pts))


#: A hop's charge, 0 to 1, when a `hop` is said without one (the page says it with; tools/sim/knobs.py `charge`).
HOP_CHARGE = 0.0
#: A pass of her world, s; her drives' torque, power and watts averaged over AVERAGE_S; a wear
#: window's passes.
DT = 1.0 / sprung.RATE_HZ
#: The environment's port her loop's layers are served on (`introspect.attach`), unset none.
TELEMETRY = 'COAXIAL_TELEMETRY'
K = DT / sprung.AVERAGE_S


def session(every=sprung.STACK_EVERY, walks=False, heard=None, texts=None, ber=0.0):
    """Her run on the loop, paced by nothing, of her texts `texts` ({key: text}: her gym's and the
    suite's) where given, else their files': her world `g`, the gym and what it says, the last
    refusal, when she fell, her drives' torque, power and watts, the epoch - a world laid anew
    each - and the passes in it. Each command heard goes to `heard` (a list's append, a queue's
    put) as (epoch, pass, t, command), each epoch's lay first."""
    from machine.texts import dsp
    suite = None if texts is None else dsp.read(dsp.SUITE, texts)
    s = {'every': every, 'walks': walks, 'gym_name': sprung.GYMS[0], 'gym': sprung.entries(texts=texts, suite=suite),
         'texts': texts, 'suite': suite, 'each': [], 'links': [],
         'refused': None, 'epoch': -1, 'ratio': 1.0, 'headroom': 1.0, 'heard': heard or (lambda entry: None),
         'ber': ber, 'worn': worn.Here(DT)}
    s['says'] = [s['gym_name']] + [entry[0] for entry in s['gym']]
    laid(s)
    return s


def laid(s):
    """A new epoch: her world laid anew, standing (walking where the session `walks`); the last one's camera
    ended, its planner and its vision kept."""
    import numpy as np
    if s.get('g') is not None and s['g']['keep'] is not None:
        s['g']['keep'].close()
    was = s.get('g')
    g = s['g'] = sprung.laid(s['every'], texts=s['texts'], suite=s['suite'])
    if was is not None:
        g['planner'], g['vision'] = was['planner'], was['vision']
    if s['walks']:
        sprung.walked(g)
    edge.noise(g['world'], s['ber'], seed=s['epoch'] + 2)
    n = len(g['world'].peak)
    s.update(fallen_at=None, shutdown=None, torque=np.zeros(n), power=np.zeros(n), watts=0.0, epoch=s['epoch'] + 1,
             passes=0, wear=wear.laid(g['body']))
    s['worn'].laid(s['wear'])
    s['wear_told'] = wear.told(s['wear'])
    s['heard']((s['epoch'], 0, g['t'], {'laid': {'every': s['every'], 'walks': s['walks'], 'gym': s['gym_name'],
                                                 'room': physics.ROOM, 'sight': physics.SIGHT}}))


def apply(s, command):
    """`command` heard at this pass, logged before it is done; a restart lays a new epoch, a link a
    tuner's hook on each pass (`tap.attached`). A recording begun or ended (`watch.recorded`)
    is not hers: never logged, never replayed."""
    if 'record' in command:
        from machine.process import watch
        watch.recorded(s, command['record'])
        return
    s['heard']((s['epoch'], s['passes'], s['g']['t'], command))
    if command.get('restart'):
        laid(s)
        s['refused'] = None
    if 'link' in command:
        from machine.process import tap
        hook, link = tap.attached(command['link'])
        s['each'].append(hook)
        s['links'].append(link)
    try:
        if command.get('gym') in sprung.GYMS:
            s['gym_name'], s['gym'] = command['gym'], sprung.entries(name=command['gym'], texts=s['texts'], suite=s['suite'])
            s['says'], s['refused'] = [command['gym']] + [entry[0] for entry in s['gym']], None
        if 'text' in command:
            command, s['refused'] = dict(command, entries=sprung.entries(command['text'])), None
        heard(s['g'], s['gym'], command)
        if 'scenario' in command:
            s['fallen_at'] = None
            s['worn'].emptied()
    except (SyntaxError, MachineError) as error:
        s['refused'] = str(error)


def readied(s):
    """Her processes apart booted before her first pass - her vision's C warm (`vision.Apart.ready`), her camera's model
    and renderer laid (`camera.Camera.ready`) -, beside her laying: her vision's boot held her first pass 0.66 s."""
    s['g']['vision'].ready()
    if s['g']['keep'] is not None:
        s['g']['keep'].ready()


@functools.cache
def _errand():
    """`machine.way.errand`, imported at her first pass and not walked again each (a package's fromlist)."""
    from machine.way import errand
    return errand


def stepped(s):
    """A pass of her world, judged: fallen, she lies RECOVER_S, then a new epoch is laid; shut down
    by a bus (`edge.held`), her drives stay off till a restart."""
    g, world = s['g'], s['g']['world']
    if s['shutdown'] is not None:
        g['t'] += DT
        world.advance()
        s['passes'] += 1
        return None
    if s['fallen_at'] is not None and g['t'] - s['fallen_at'] >= sprung.RECOVER_S:
        return laid(s)
    try:
        if s['fallen_at'] is None:
            gaits.halted(g, s['gym'])
            _errand().step_errand(g, s['gym'], heard, gaits.heard_say)
            sprung.passed(g, DT)
            if s['worn'].put(g['sense'], world.motor):
                s['worn'].shut(s['passes'])
        else:
            # fallen she lies with her drives let go, as the bench's STO drops them: driven on
            # through the fall the rest law fought the ground - 1 658 deg/s, 3 793 N into it
            # supine, 40 deg/s and 45 N let go - the spasm of every fall and tipped lie.
            s['worn'].emptied()
            g['surety'] = 0.0
            edge.felt(world, g['sense'])
            edge.off(world)
            g['t'] += DT
            world.advance()
        if s['worn'].due_at is not None:
            # a part of her wear's window a pass, or its read apart; its load laid LAND passes on (`worn`):
            # in one pass the window held her world 0.12 s
            t_wear = time.perf_counter()
            said = s['worn'].due(s['passes'], g)
            if said is not None:
                s['wear_told'] = said
            s['_wear_s'] = time.perf_counter() - t_wear
    except BusFault as fault:
        s['shutdown'], s['fallen_at'] = str(fault), g['t']
        edge.off(world)
    except MachineError:
        s['fallen_at'] = g['t']
    if s['fallen_at'] is None and referee.fallen(g, world):
        s['fallen_at'] = g['t']
    s['passes'] += 1
    _paired, averaged, address = referee.passing()
    averaged(ctypes.byref(world.passing), K, address(s['torque']), address(s['power']))
    s['watts'] += K * (world.drawn() - s['watts'])
    for hook in s['each']:
        hook(s)


def told(s) -> dict:
    """Her state as the page hears it (`sprung.state`), her epoch, her wear and her docks foreseen
    (`peek.told`) beside it, as its last window left it."""
    return dict(sprung.state(s['g'], s['torque'], s['power'], s['watts'], s['ratio'], s['fallen_at'],
                             s['says'], s['refused'], s['headroom']), epoch=s['epoch'], wear=s['wear_told'],
                shutdown=s['shutdown'], docks=peek.told(s))


class Her:

    """Her session (`session`) as `apart.paced` steps it: her time, her epoch, a command applied,
    a pass - its wear's seconds left out of the pace -, her state told at her ratio and headroom."""

    def __init__(self, s):
        self.s = s

    @property
    def t(self):
        return self.s['g']['t']

    @property
    def epoch(self):
        return self.s['epoch']

    def apply(self, command):
        apply(self.s, command)

    def step(self):
        stepped(self.s)
        peek.peeked(self.s)
        return self.s.pop('_wear_s', 0.0)

    def said(self, ratio, headroom=1.0):
        if ratio is not None:
            self.s['ratio'], self.s['headroom'] = ratio, headroom
        return told(self.s)


def run(commands, states, shown, heard=None, every=sprung.STACK_EVERY, walks=False, realtime=True, ber=0.0):
    """The worker: she stands on the loop, paced to the clock (`realtime`, or flat out without it);
    commands heard - each put on `heard` at its pass -, her state said. Laid, she stands at rest
    unless `walks`. Run apart (`apart.paced`), no further than its lead past what her page has
    `shown`. Her registry's things are her eyes' (`physics.SIGHT`: her stereo pair drawn and matched, its map
    classified), her vision's panel their view - the world's boxes are her page's drawing alone; her planner and
    her vision (`vision.Apart`) spawned warm, her vision and her camera waited for warm before her first pass
    (`readied`); placed as a loop's by the process it is run in (`apart.looped`)."""
    physics.ROOM, physics.CAMERA, physics.SIGHT = 1.0, 1.0, 1.0
    s = session(every, walks, heard.put if heard is not None else None, ber=ber)
    s['g']['planner'], s['g']['vision'], s['peeker'] = planner.Planner(), vision.Apart(), planner.Planner()
    s['g']['planner'].start()
    s['peeker'].start()
    s['g']['vision'].start()
    s['worn'] = worn.Apart(DT).start()
    s['worn'].laid(s['wear'])
    readied(s)
    if os.environ.get(TELEMETRY):
        from machine.process import introspect
        introspect.attach(s, int(os.environ[TELEMETRY]))
    apart.paced(Her(s), commands, states, shown=shown, realtime=realtime, say_s=sprung.SAY_S,
                rest_s=sprung.SAY_S / 4.0, slice_s=sprung.SLICE_S)


class Running(apart.Apart):

    """Her worker process on the loop (`run`, apart): `send` it commands, read its `latest` state
    and the commands it has `heard`, tell it what her page has `shown` - from `shown`, none held
    back till told; laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False, realtime=True, shown=math.inf, ber=0.0):
        self._heard = apart.CONTEXT.Queue()
        super().__init__(run, self._heard, every, walks, realtime, ber, states=8, kept=(self._heard,),
                         shown=shown)

    def send(self, **command):
        """{'say': the gym's entry by its place, in place of what was to come} | {'walk': True}:
        her walk from rest | {'entries': [(label, wire, names)]} | {'gym': one of GYMS} |
        {'text': orthokinematics, begun now} | {'push': (x, y, z) N, 'seconds': s} |
        {'restart': True}: laid anew | {'record': a file's name, or None}: recorded (`watch.recorded`).
        Any other key is refused, said with her state."""
        super().send(**command)

    def heard(self):
        """The commands she has heard since this was last asked, (epoch, pass, t, command) each:
        none is dropped, as a state can be."""
        out = []
        try:
            while True:
                out.append(self._heard.get_nowait())
        except queue.Empty:
            pass
        return out

