"""Her prompt through the local model on her loop: each said as the page says it, judged under the surface.

Each case types its prompt into the page's own path (`gym.submit_prompt`: the local model asked off
the page's thread, `drain_plan` sending what it chose to her worker) and steps her session, then
reads what her loop did - her doing's phases, the errand laid, her pelvis, a fall's epoch, her
drives' clamps, a refusal's words, how far she went - against what the prompt meant. Skipped where
no model is here (`coaxial_ollama.gynoid.model`).
"""
import math
import sys
from types import SimpleNamespace as SN

from tools.dev.focus import chosen
from views_kit import Report


def _said(prompt, room=0.0, before=(), lead_s=0.5):
    """(session, page state, sent) after `prompt` went through the page's path to her loop: her
    session laid (`room` the lodge or the flat floor), `before` commands said and `lead_s` stepped,
    then the prompt typed and its plan dispatched."""
    from machine.world import physics
    from machine.process import worker
    from terminal.views import gym, humanoid_keys, viewpoint
    physics.ROOM, physics.SIGHT = room, 0.0
    s = worker.session()
    if not room:
        worker.apply(s, {'scenario': 'flat'})
    for cmd in before:
        worker.apply(s, cmd)
    _steps(s, lead_s)
    sent = []
    st = {'body': SN(send=lambda **cmd: (sent.append(cmd), worker.apply(s, cmd))), 'pad': humanoid_keys.pad(),
          'tour': None, **viewpoint.HOME, 'now': worker.told(s), 'chat_rows': []}
    gym.submit_prompt(st, prompt)
    if st.get('asking') is not None:
        st['asking'].join(180.0)
    gym.drain_plan(st)
    return s, st, sent


def _steps(s, seconds, until=None):
    """Her session `seconds` of passes on, or till `until(s)` or a fall; the seconds it took, None
    never met. Passes counted: a fall lays a new epoch's `s['g']`, and a loop on the old one's time
    never ended (the first run hung 1200 s)."""
    from machine.process import worker
    for k in range(int(round(seconds / worker.DT))):
        worker.stepped(s)
        if until is not None and until(s):
            return (k + 1) * worker.DT
        if s['fallen_at'] is not None:
            return None
    return None


def _where(s):
    d = s['g']['world'].data
    return float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2])


def _model(report, case):
    from coaxial_ollama import gynoid
    name = gynoid.model()
    if name is None:
        report.skip(case, 'no local model here (ollama pull %s)' % gynoid.MODELS[0])
    return name


def test_lie_down_on_the_floor_said_in_swedish(report):
    """'lägg dig på golvet' on the flat floor: lie_floor said; her doing through kneel and tip to
    floor_rest, her pelvis under 0.25 m, no fall's epoch, her drives' clamps zero (limp)."""
    import numpy as np
    from machine.process import sprung
    case = 'lie down on the floor, said in Swedish'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('lägg dig på golvet')
    took = _steps(s, 14.0, lambda s: sprung.doing(s['g']) == 'floor_rest' and _where(s)[1] < 0.25)
    _steps(s, 2.0)
    g = s['g']
    report.check(case, sent == [{'say': 'lie_floor'}] and took is not None and s.get('epoch', 0) == 0
                 and _where(s)[1] < 0.25 and not np.any(g['cmd']['limit']),
                 'sent %s, floor_rest at %s s, pelvis %.3f, epoch %s' % (sent, took, _where(s)[1], s.get('epoch', 0)))


def test_sit_on_the_chair_said_in_swedish(report):
    """'sätt dig på stolen' in the lodge: the sit errand laid on a chair, seated within 40 s, unfallen."""
    from machine.process import sprung
    case = 'sit on the chair, said in Swedish'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('sätt dig på stolen', room=1.0)
    kind = (s['g'].get('_errand') or {}).get('kind')
    took = _steps(s, 40.0, lambda s: sprung.doing(s['g']) == 'seated')
    report.check(case, sent == [{'say': 'sit_chair'}] and kind == 'sit_chair' and took is not None and s['fallen_at'] is None,
                 'sent %s, errand %s, seated at %s s, fallen %s' % (sent, kind, took, s['fallen_at']))


def test_lie_on_the_bed_said_in_english(report):
    """'lie on the bed' in the lodge: the bed errand laid, bed_rest within 40 s, unfallen."""
    from machine.process import sprung
    case = 'lie on the bed, said in English'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('lie on the bed', room=1.0)
    kind = (s['g'].get('_errand') or {}).get('kind')
    took = _steps(s, 40.0, lambda s: sprung.doing(s['g']) == 'bed_rest')
    report.check(case, sent == [{'say': 'lie_bed'}] and kind == 'lie_bed' and took is not None and s['fallen_at'] is None,
                 'sent %s, errand %s, bed_rest at %s s, fallen %s' % (sent, kind, took, s['fallen_at']))


def test_out_through_the_door_said_in_swedish(report):
    """'gå ut genom dörren' in the lodge: the door errand laid, past the door's line within 30 s, unfallen."""
    from machine.world import room
    case = 'out through the door, said in Swedish'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('gå ut genom dörren', room=1.0)
    kind = (s['g'].get('_errand') or {}).get('kind')
    took = _steps(s, 30.0, lambda s: _where(s)[2] > room.TO)
    report.check(case, sent == [{'say': 'exit_door'}] and kind == 'exit_door' and took is not None and s['fallen_at'] is None,
                 'sent %s, errand %s, out at %s s, fallen %s' % (sent, kind, took, s['fallen_at']))


def test_walk_forward_said_in_swedish(report):
    """'gå framåt' on the flat floor: her walk's twist, 0.5 m on within 3 s, unfallen."""
    case = 'walk forward, said in Swedish'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('gå framåt')
    x0, _y, z0 = _where(s)
    took = _steps(s, 3.0, lambda s: math.hypot(_where(s)[0] - x0, _where(s)[2] - z0) > 0.5)
    report.check(case, len(sent) == 1 and 'twist' in sent[0] and took is not None and s['fallen_at'] is None,
                 'sent %s, 0.5 m at %s s' % (sent, took))


def test_dance_said_in_swedish(report):
    """'dansa' on the flat floor: the dance said, one of its phases her doing within 4 s."""
    from machine.texts import says
    from machine.process import sprung
    case = 'dance, said in Swedish'
    if _model(report, case) is None:
        return
    phases = set(next(r for r in says.catalog() if r['op'] == 'dance').get('phases', ()))
    s, _st, sent = _said('dansa')
    took = _steps(s, 4.0, lambda s: sprung.doing(s['g']) in phases)
    report.check(case, sent == [{'say': 'dance'}] and took is not None and s['fallen_at'] is None,
                 'sent %s, dancing at %s s' % (sent, took))


def test_nothing_she_can_do_moves_nothing(report):
    """'baka en kaka' on the flat floor: nothing she can do with her body - nothing sent, she stays
    within 5 cm of where she stood over 3 s."""
    case = 'bake a cake moves nothing'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('baka en kaka')
    x0, _y, z0 = _where(s)
    _steps(s, 3.0)
    went = math.hypot(_where(s)[0] - x0, _where(s)[2] - z0)
    report.check(case, sent == [] and went < 0.05, 'sent %s, went %.3f m' % (sent, went))


def test_rise_from_the_floor_refused_by_name(report):
    """'res dig' lying on the floor: the rise said and refused by name - her legs' floor, item 13 -,
    she lies on, quiet."""
    import numpy as np
    from machine.process import sprung
    case = 'rise from the floor refused by name'
    if _model(report, case) is None:
        return
    s, _st, sent = _said('res dig', before=({'say': 'lie_floor'},), lead_s=12.0)
    _steps(s, 3.0)
    d = s['g']['world'].data
    report.check(case, sent == [{'say': 'rise_floor'}] and 'item 13' in (s.get('refused') or '')
                 and sprung.doing(s['g']) == 'floor_rest' and float(np.abs(d.qvel[6:]).max()) < math.radians(30.0),
                 'sent %s, refused %r, doing %s' % (sent, (s.get('refused') or '')[:80], sprung.doing(s['g'])))


ROSTER = (test_lie_down_on_the_floor_said_in_swedish, test_sit_on_the_chair_said_in_swedish,
          test_lie_on_the_bed_said_in_english, test_out_through_the_door_said_in_swedish,
          test_walk_forward_said_in_swedish, test_dance_said_in_swedish, test_nothing_she_can_do_moves_nothing,
          test_rise_from_the_floor_refused_by_name)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
