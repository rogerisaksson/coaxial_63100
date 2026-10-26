"""Her prompt on the HUMANOID page: the chat dock its one surface, the local model asked off the
page's thread when one is here, the keyword rules when none (`gym.submit_prompt`)."""
import sys
from types import SimpleNamespace as SN

from tools.dev.focus import chosen
from views_kit import Report, typed


def test_the_prompt_lives_in_the_chat_dock(report):
    """/ opens the prompt in the chat dock; without a model the keyword rules compile chair, bed,
    door and the tour; with one the plan is asked off the page's thread and sent as it lands."""
    from terminal.views import gym, humanoid_keys, show_humanoid, viewpoint
    sent, pad = [], humanoid_keys.PAD
    st = {'body': SN(send=lambda **cmd: sent.append(cmd)), 'pad': humanoid_keys.pad(), 'tour': None,
          **viewpoint.HOME, 'now': {'doing': 'rest', 'where': (0.0, 0.9, 0.0)}}
    from coaxial_ollama import gynoid as llm
    was = llm.model, llm.plan
    try:
        llm.model = lambda timeout=0.5: None
        act = lambda s: typed(pad, s, st)
        act('/sit in the chair\r')
        act('/go to bed')
        cb_open = str(show_humanoid.chat_box(st, 80).renderable)
        for q in ('\r', '/exit out the door\r', '/tour the room\r'):
            act(q)
        cb_done = str(show_humanoid.chat_box(st, 80).renderable)
        report.check('/ prompts in the chat dock alone, no model here: the keyword rules; chair/bed/door/tour compiled',
                     sent == [{'say': 'sit_chair'}, {'say': 'lie_bed'}, {'say': 'exit_door'}]
                     and 'go to bed' in cb_open and 'say=sit_chair' in cb_open and gym.touring(st)
                     and '> sit in the chair' in cb_open and 'cmd: tour=start' in cb_done
                     and ('/', 'CHAT') in pad['bar'], (sent, cb_open[-60:]))
        sent.clear()
        gym.cancel_tour(st)
        llm.model = lambda timeout=0.5: 'a model'
        llm.plan = lambda q, doing, name, rows=None, timeout=60.0: {'thought': 'lie down', 'cmds': [{'say': 'lie_floor'}],
                                                                    'tour': False}
        act('/lay on floor\r')
        thinking = str(show_humanoid.chat_box(st, 80).renderable)
        st['asking'].join(5.0)
        before = list(sent)
        gym.drain_plan(st)
        report.check("a model here: asked off the page's thread, the plan sent as it lands, the chat thinking meanwhile",
                     before == [] and sent == [{'say': 'lie_floor'}] and 'thinking (a model)' in thinking, (before, sent))
    finally:
        llm.model, llm.plan = was


ROSTER = (test_the_prompt_lives_in_the_chat_dock,)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
